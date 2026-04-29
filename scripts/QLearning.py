#!/usr/bin/env python

# COM760 Group 30 - Collapsed School Rescue Robot
# QLearning.py - Q-Learning based autonomous navigation (Advanced Feature)
#
# Implements Q-Learning for robot navigation as an advanced feature
# alongside the Bug2 algorithm.
#
# Q-Learning fundamentals (W10 Lecture):
#   - State space: 5 laser zones = 32 possible binary states (2^5)
#   - Actions: forward, turn left, turn right, backward (4 actions)
#   - Rewards: +100 survivor found/goal reached, -100 collision, +5 closer
#   - Q-value update (Bellman equation):
#       Q(s,a) = Q(s,a) + alpha * (r + gamma * max(Q(s',a')) - Q(s,a))
#   - Epsilon-greedy exploration: random action with prob epsilon,
#     best known action otherwise
#   - Epsilon decays each episode (exploration -> exploitation)
#
# Reference: W10 Lecture (Reinforcement Learning in Robotics)
# Reference: W10 Practical (Q-Learning navigation)
# Activated via MineRescueSetQLStatus service
# Uses RLAction and RLReward custom messages for action/reward communication
#
# Improvements merged from M3 (Salman):
#   - reset_episode() method: separates episode reset logic cleanly
#   - Laser noise filter: r > 0.05 prevents false collision from sensor noise
#   - laser_zones initialised to safe default before first callback fires

import rospy
import math
import numpy as np
import random
import pickle
import os
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from com760cw2_group30.msg import RLAction, RLReward, SurvivorDetected
from com760cw2_group30.srv import (
    MineRescueSetQLStatus,
    MineRescueSetQLStatusResponse)
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class QLearning:

    # Actions available to the robot
    # Each entry: (label, linear_velocity, angular_velocity)
    ACTIONS = {
        0: ('forward',    0.3,  0.0),
        1: ('turn_left',  0.0,  0.5),
        2: ('turn_right', 0.0, -0.5),
        3: ('backward',  -0.2,  0.0),
    }
    N_ACTIONS = 4
    N_ZONES   = 5     # front, front-left, front-right, left, right
    N_STATES  = 32    # 2^5 binary states from laser zones

    def __init__(self):
        rospy.init_node('q_learning')

        self.active = False

        # Q-table: rows = states (32), columns = actions (4)
        # Initialised to zeros - all actions equally unknown
        # Reference: W10 Lecture Slide (Q-table initialisation)
        self.q_table = np.zeros((self.N_STATES, self.N_ACTIONS))

        # Q-Learning hyperparameters
        # Reference: W10 Lecture (Hyperparameters)
        self.alpha         = 0.5    # learning rate: how fast Q-values update
        self.gamma         = 0.9    # discount factor: weight of future rewards
        self.epsilon       = 0.9    # exploration rate: prob of random action
        self.epsilon_min   = 0.05   # minimum exploration rate
        self.epsilon_decay = 0.995  # decay per episode
        self.max_episodes  = 500    # total training episodes

        # Training statistics
        self.episode      = 0
        self.step_count   = 0
        self.max_steps    = 300     # max steps per episode before reset
        self.total_reward = 0.0

        # Navigation goal - emergency base (final waypoint)
        self.goal = Point()
        self.goal.x = 11.0
        self.goal.y = 0.0

        # Survivor locations for reward shaping
        # Coordinates match world file (school_rescue.world) and SurvivorDetector.py
        self.survivors = [
            {'id': 1, 'x': -6.0, 'y':  3.0, 'found': False},
            {'id': 2, 'x':  0.0, 'y': -3.0, 'found': False},
            {'id': 3, 'x':  5.0, 'y':  3.0, 'found': False},
        ]
        self.survivors_found = 0

        # Robot state
        self.position       = Point()
        self.yaw            = 0.0
        self.prev_dist_goal = float('inf')
        self.prev_dist_survivor = float('inf')  # for shaped reward toward survivors

        # Improvement (M3): initialise laser_zones to safe default list so it
        # is never read as None before the first laser callback fires
        self.laser_zones    = [0] * self.N_ZONES
        self.obs_threshold  = 0.5   # obstacle detection threshold (metres)

        # Q-table save path - persists learning between runs
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        self.q_table_path = os.path.join(pkg_dir, 'q_table_school.pkl')

        # Publishers
        # cmd_vel: robot movement commands
        self.pub_vel = rospy.Publisher(
            '/group30Bot/cmd_vel', Twist, queue_size=1)
        # RLAction: custom message publishing chosen action
        # Reference: Assignment Brief - custom messages requirement
        self.pub_action = rospy.Publisher(
            '/group30Bot/rl_action', RLAction, queue_size=1)
        # RLReward: custom message publishing reward signal
        self.pub_reward = rospy.Publisher(
            '/group30Bot/rl_reward', RLReward, queue_size=1)

        # Subscribers
        self.sub_laser = rospy.Subscriber(
            '/group30Bot/laser/scan', LaserScan, self.callback_laser)
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.callback_odom)

        # Service: activate/deactivate Q-Learning with hyperparameter control
        # Reference: Assignment Brief - custom services requirement
        self.srv = rospy.Service(
            'q_learning_switch',
            MineRescueSetQLStatus,
            self.handle_switch)

        # Load existing Q-table if available
        self.load_q_table()

        rospy.loginfo('=' * 50)
        rospy.loginfo('[QLearning] School rescue Q-Learning ready!')
        rospy.loginfo('[QLearning] States: %d  Actions: %d',
                      self.N_STATES, self.N_ACTIONS)
        rospy.loginfo('[QLearning] Call q_learning_switch to start training.')
        rospy.loginfo('=' * 50)

        # Main loop at 5Hz (slower than Bug2 - RL needs time between steps)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.active:
                self.run_step()
            rate.sleep()

    # ------------------------------------------------------------------
    # Service handler: start/stop Q-Learning with configurable params
    # ------------------------------------------------------------------

    def handle_switch(self, req):
        self.active = req.flag
        if req.flag:
            # Set hyperparameters from service request
            self.alpha        = req.learning_rate if req.learning_rate > 0 else 0.5
            self.gamma        = req.discount      if req.discount > 0      else 0.9
            self.epsilon      = req.epsilon       if req.epsilon > 0       else 0.9
            self.max_episodes = req.max_episodes  if req.max_episodes > 0  else 500
            self.episode      = 0
            # Reset survivors for fresh training
            for s in self.survivors:
                s['found'] = False
            self.survivors_found = 0
            rospy.loginfo('=' * 50)
            rospy.loginfo('[QLearning] Training started!')
            rospy.loginfo(
                '[QLearning] alpha=%.2f  gamma=%.2f  epsilon=%.2f  episodes=%d',
                self.alpha, self.gamma, self.epsilon, self.max_episodes)
            rospy.loginfo('=' * 50)
            return MineRescueSetQLStatusResponse(
                success=True, message='Q-Learning training started!')
        else:
            self.stop_robot()
            self.save_q_table()
            return MineRescueSetQLStatusResponse(
                success=True, message='Q-Learning stopped. Q-table saved.')

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------

    def callback_odom(self, msg):
        self.position = msg.pose.pose.position
        q = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)
        self.yaw = transformations.euler_from_quaternion(q)[2]

    # ------------------------------------------------------------------
    # Laser callback: convert ranges to 5-zone binary state
    # Reference: W10 Lecture, W8 Lecture Slide 19
    # ------------------------------------------------------------------

    def callback_laser(self, msg):
        ranges = list(msg.ranges)
        max_r  = msg.range_max
        n      = len(ranges)
        s      = max(1, int(n * 30 / 360))

        # Improvement (M3): filter r > 0.05 in addition to nan/inf checks
        # Prevents sensor noise very close to the robot causing false collisions
        clean = [r if (not math.isnan(r) and
                       not math.isinf(r) and
                       r > 0.05) else max_r
                 for r in ranges]

        # Extract 5 directional zones
        raw = {
            'front':       min(min(clean[-s:]), min(clean[:s])),
            'front_left':  min(clean[s: s * 3]) if s * 3 < n else max_r,
            'front_right': min(clean[-(s * 3):-s]) if s * 3 < n else max_r,
            'left':        min(clean[s * 3: n // 2]) if s * 3 < n // 2 else max_r,
            'right':       min(clean[n // 2: -(s * 3)]) if s * 3 < n // 2 else max_r,
        }
        # Binary: 1 = obstacle within threshold, 0 = clear
        t = self.obs_threshold
        self.laser_zones = [
            1 if raw['front']       < t else 0,
            1 if raw['front_left']  < t else 0,
            1 if raw['front_right'] < t else 0,
            1 if raw['left']        < t else 0,
            1 if raw['right']       < t else 0,
        ]

    # ------------------------------------------------------------------
    # Q-Learning core methods
    # Reference: W10 Lecture (Q-Learning algorithm)
    # ------------------------------------------------------------------

    def get_state(self):
        """Convert 5 binary laser zones to integer state index (0-31)."""
        return int(''.join(str(z) for z in self.laser_zones), 2)

    def choose_action(self, state):
        """Epsilon-greedy action selection.
        Random with prob epsilon (explore), best known with prob 1-epsilon (exploit).
        Reference: W10 Lecture (Exploration vs Exploitation)"""
        if random.random() < self.epsilon:
            return random.randint(0, self.N_ACTIONS - 1)
        return int(np.argmax(self.q_table[state]))

    def execute_action(self, action_id):
        """Execute chosen action and publish RLAction custom message."""
        label, linear, angular = self.ACTIONS[action_id]
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub_vel.publish(msg)
        # Publish RLAction custom message for monitoring/logging
        action_msg = RLAction()
        action_msg.action_id     = action_id
        action_msg.linear_speed  = linear
        action_msg.angular_speed = angular
        action_msg.description   = label
        self.pub_action.publish(action_msg)

    def check_survivors(self):
        """Check proximity to survivors and return bonus reward if found."""
        for survivor in self.survivors:
            if survivor['found']:
                continue
            dist = math.sqrt(
                (self.position.x - survivor['x']) ** 2 +
                (self.position.y - survivor['y']) ** 2)
            if dist < 1.5:
                survivor['found'] = True
                self.survivors_found += 1
                rospy.logwarn(
                    '*** QL: SURVIVOR %d FOUND! Location: (%.1f, %.1f) ***',
                    survivor['id'], survivor['x'], survivor['y'])
                return 100  # large positive reward
        return 0

    def distance_to_nearest_survivor(self):
        """Return distance to the nearest unfound survivor.
        Used for reward shaping to guide the robot during training.
        Reference: W10 Lecture (reward shaping)"""
        min_dist = float('inf')
        for s in self.survivors:
            if not s['found']:
                d = math.sqrt(
                    (self.position.x - s['x']) ** 2 +
                    (self.position.y - s['y']) ** 2)
                min_dist = min(min_dist, d)
        return min_dist

    def calculate_reward(self):
        """Calculate reward signal for current state.
        Reward structure:
        -1   per step (encourages efficiency)
        -100 for collision (front 3 zones blocked)
        +100 for reaching emergency base
        +100 for finding a survivor
        +5   for moving closer to emergency base
        +2   for moving closer to nearest unfound survivor
        -5   for moving further away
        Reference: W10 Lecture (Reward shaping)"""
        dist           = self.distance_to_goal()
        dist_survivor  = self.distance_to_nearest_survivor()
        collision      = any(z == 1 for z in self.laser_zones[:3])
        goal_reached   = dist < 0.35

        survivor_reward = self.check_survivors()
        reward = -1  # step penalty

        if collision:
            reward = -100
        elif goal_reached:
            reward = 100
            rospy.logwarn('*** QL: EMERGENCY BASE REACHED! MISSION COMPLETE! ***')
        elif survivor_reward > 0:
            reward = survivor_reward
        else:
            # Progress reward: moving toward goal
            if dist < self.prev_dist_goal:
                reward += 5
            else:
                reward -= 5
            # Shaped reward: moving toward nearest unfound survivor
            # Reference: W10 Lecture (reward shaping for multi-target tasks)
            if dist_survivor < self.prev_dist_survivor:
                reward += 2

        self.prev_dist_goal     = dist
        self.prev_dist_survivor = dist_survivor

        # Publish RLReward custom message
        reward_msg = RLReward()
        reward_msg.reward            = reward
        reward_msg.collision         = collision
        reward_msg.goal_reached      = goal_reached
        reward_msg.distance_to_goal  = dist
        reward_msg.state_description = str(self.laser_zones)
        self.pub_reward.publish(reward_msg)

        return reward, collision, goal_reached

    def update_q_table(self, state, action, reward, next_state):
        """Update Q-table using Bellman equation.
        Q(s,a) = Q(s,a) + alpha * (r + gamma * max(Q(s',a')) - Q(s,a))
        Reference: W10 Lecture (Bellman equation)"""
        best_next = np.max(self.q_table[next_state])
        current   = self.q_table[state, action]
        self.q_table[state, action] = (
            current + self.alpha * (
                reward + self.gamma * best_next - current))

    def reset_episode(self):
        """Reset robot to spawn position and clear episode state.
        Teleports robot back to start using Gazebo set_model_state service.
        Reference: W10 Lecture (episode reset in RL training)"""
        self.stop_robot()
        rospy.sleep(0.5)

        # Teleport robot back to spawn position for clean next episode
        # Uses Gazebo's built-in model state service
        try:
            from gazebo_msgs.srv import SetModelState
            from gazebo_msgs.msg import ModelState
            rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
            set_state = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = 'group30Bot'
            state.pose.position.x = -7.0
            state.pose.position.y =  0.0
            state.pose.position.z =  0.1
            state.pose.orientation.x = 0.0
            state.pose.orientation.y = 0.0
            state.pose.orientation.z = 0.0
            state.pose.orientation.w = 1.0
            state.reference_frame = 'world'
            set_state(state)
            rospy.loginfo('[QLearning] Robot reset to spawn position.')
        except Exception as e:
            rospy.logwarn('[QLearning] Spawn reset failed: %s', e)

        rospy.sleep(0.5)

        # Reset survivor flags and distance tracker
        for s in self.survivors:
            s['found'] = False
        self.survivors_found = 0
        self.prev_dist_goal = self.distance_to_goal()
        self.prev_dist_survivor = float('inf')

    def run_step(self):
        """Execute one Q-Learning step: observe state, act, get reward, update Q-table."""
        if self.episode >= self.max_episodes:
            rospy.loginfo(
                '[QLearning] Training complete! %d episodes.', self.episode)
            self.active = False
            self.save_q_table()
            return

        state  = self.get_state()
        action = self.choose_action(state)
        self.execute_action(action)
        rospy.sleep(0.2)   # allow action to take effect before sensing

        next_state = self.get_state()
        reward, collision, done = self.calculate_reward()
        self.update_q_table(state, action, reward, next_state)

        self.total_reward += reward
        self.step_count   += 1

        episode_done = (collision or done or self.step_count >= self.max_steps)

        if episode_done:
            rospy.loginfo(
                '[QLearning] Ep %d | Steps: %d | Reward: %.1f | '
                'Epsilon: %.3f | Survivors: %d/3',
                self.episode, self.step_count,
                self.total_reward, self.epsilon, self.survivors_found)

            self.episode     += 1
            self.step_count   = 0
            self.total_reward = 0.0

            # Decay epsilon - less random exploration as training progresses
            # Reference: W10 Lecture (Epsilon decay schedule)
            self.epsilon = max(
                self.epsilon_min,
                self.epsilon * self.epsilon_decay)

            # Improvement (M3): use reset_episode() for clean separation
            self.reset_episode()

    # ------------------------------------------------------------------
    # Q-table persistence
    # ------------------------------------------------------------------

    def save_q_table(self):
        """Save Q-table to disk so learning persists between runs."""
        try:
            with open(self.q_table_path, 'wb') as f:
                pickle.dump(self.q_table, f)
            rospy.loginfo('[QLearning] Q-table saved to %s', self.q_table_path)
        except Exception as e:
            rospy.logwarn('[QLearning] Could not save Q-table: %s', e)

    def load_q_table(self):
        """Load Q-table from disk if it exists - continues previous training."""
        if os.path.exists(self.q_table_path):
            try:
                with open(self.q_table_path, 'rb') as f:
                    self.q_table = pickle.load(f)
                rospy.loginfo('[QLearning] Q-table loaded from previous training!')
            except Exception as e:
                rospy.logwarn('[QLearning] Could not load Q-table: %s', e)

    def distance_to_goal(self):
        return math.sqrt(
            (self.goal.x - self.position.x) ** 2 +
            (self.goal.y - self.position.y) ** 2)

    def stop_robot(self):
        self.pub_vel.publish(Twist())


if __name__ == '__main__':
    try:
        QLearning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass