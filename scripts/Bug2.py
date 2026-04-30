#!/usr/bin/env python

# COM760 Group 30 - Collapsed School Rescue Robot
# Bug2.py - Master coordinator implementing the Bug2 algorithm
#
# Mission: Navigate collapsed school building to find survivors
#   Waypoint 1: Child 1    (-6.0,  3.0) - NW classroom
#   Waypoint 2: Teacher    ( 0.0, -3.0) - south corridor
#   Waypoint 3: Child 2    ( 5.0,  3.0) - east corridor
#   Waypoint 4: Base       (11.0,  0.0) - emergency base (exit)
#
# Bug2 Algorithm (Lumelsky & Stepanov, 1987):
#   1. Draw imaginary M-line from start to goal
#   2. Drive along M-line toward goal (GoToPoint active)
#   3. On obstacle: record hit point, switch to FollowWall
#   4. Follow wall until back on M-line AND closer to goal than hit point
#   5. Switch back to GoToPoint, repeat
#
# Reference: W8 Lecture, W9 Lecture (Bug Algorithms)
# Source: https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/
# Algorithm paper: Lumelsky & Stepanov (1987)

import rospy
import math
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
from com760cw2_group30.srv import (
    MineRescueSetBugStatus,
    MineRescueSetBugStatusRequest,
    MineRescueSetBugStatusResponse)
from com760cw2_group30.msg import SurvivorDetected

class Bug2:

    def __init__(self):
        rospy.init_node('bug2_coordinator')

        # Navigation states
        # 0 = Standing by   - waiting for sensors, then auto-starts
        # 1 = GoToPoint     - driving straight toward goal along M-line
        # 2 = FollowWall    - circumnavigating an obstacle
        # 3 = Waypoint reached - brief pause before advancing to next goal
        self.nav_state = 0
        self.nav_labels = {
            0: 'Standing by',
            1: 'GoToPoint',
            2: 'FollowWall',
            3: 'Waypoint reached'
        }

        # Robot pose - updated from odometry
        self.position = Point()
        self.yaw      = 0.0

        # Mission waypoints for search and rescue scenario
        # IMPORTANT: coordinates must match SurvivorDetector.py and world file
        # Reference: Assignment Brief - "goal pose" for navigation
        self.waypoints = [
            (-6.0,  3.0),   # Child 1  - NW classroom
            ( 0.0, -3.0),   # Teacher  - south corridor
            ( 5.0,  3.0),   # Child 2  - east corridor
            (11.0,  0.0),   # Emergency base - mission complete
        ]
        self.waypoint_labels = [
            'Child 1 - NW Classroom',
            'Teacher - South Corridor',
            'Child 2 - East Corridor',
            'Emergency Base - Mission Complete',
        ]
        self.current_waypoint = 0

        # Current navigation goal
        self.goal   = Point()
        self.goal.x = self.waypoints[0][0]
        self.goal.y = self.waypoints[0][1]

        # Bug2 M-line parameters
        # M-line is the imaginary line from start position to goal
        # Reference: W8 Lecture (Bug2 algorithm), W9 Lecture Slide
        self.start_position   = Point()
        self.m_line_slope     = 0.0
        self.m_line_intercept = 0.0

        # Obstacle hit-point tracking for M-line return condition
        self.obstacle_hit_point = Point()
        self.obstacle_hit_dist  = float('inf')
        self.wall_follow_start  = 0.0  # timestamp when FollowWall began

        # Navigation parameters
        self.speed     = 0.5   # m/s - passed to sub-behaviours via service
        self.direction = 'left'  # default turn direction

        # Detection thresholds (metres)
        self.obstacle_threshold   = 0.35  # switch to wall follow at this distance
        self.m_line_threshold     = 0.35  # perpendicular distance to be "on" M-line
        self.goal_threshold       = 0.40  # close enough to consider waypoint reached
        self.min_wall_follow_secs = 3.0   # minimum wall follow time before M-line check
        self.m_line_progress_buf  = 0.40  # must be 0.4m closer than hit point to exit

        # Laser front distance
        self.laser_front = float('inf')

        # Sensor-ready flags - auto-start fires once both are True
        self.got_odom  = False
        self.got_laser = False

        # Publisher: emergency stop only - normal movement via sub-behaviours
        self.pub_vel = rospy.Publisher(
            '/group30Bot/cmd_vel', Twist, queue_size=1)

        # Subscribers
        # Odometry for position tracking - Reference: W8 Lecture Slide 28
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.callback_odom)

        self.sub_survivor = rospy.Subscriber(
            '/group30Bot/survivor_detected',
            SurvivorDetected, self.callback_survivor)
        
        # Laser for obstacle detection in Bug2 state machine
        self.sub_laser = rospy.Subscriber(
            '/group30Bot/laser/scan', LaserScan, self.callback_laser)

        # SurvivorDetected custom message - logs when proximity triggers
        # Reference: Assignment Brief - custom messages requirement
        self.sub_survivor = rospy.Subscriber(
            '/group30Bot/survivor_detected',
            SurvivorDetected, self.callback_survivor)

        # External homing override service
        # Allows operator to immediately recall robot to base
        # Reference: Assignment Brief p.2 - "homing signal" scenario
        self.srv_homing = rospy.Service(
            'mine_rescue_homing',
            MineRescueSetBugStatus,
            self.handle_homing_signal)

        # Wait for sub-behaviour services to become available
        rospy.loginfo('[Bug2] Waiting for navigation services...')
        rospy.wait_for_service('go_to_point_switch')
        rospy.wait_for_service('wall_follower_switch')

        self.client_gtp = rospy.ServiceProxy(
            'go_to_point_switch', MineRescueSetBugStatus)
        self.client_fw = rospy.ServiceProxy(
            'wall_follower_switch', MineRescueSetBugStatus)

        rospy.loginfo('=' * 55)
        rospy.loginfo('[Bug2] SYSTEM READY - Mission waypoints:')
        for i, (wp, lbl) in enumerate(
                zip(self.waypoints, self.waypoint_labels)):
            rospy.loginfo('  %d. %s -> (%.1f, %.1f)', i + 1, lbl, wp[0], wp[1])
        rospy.loginfo('[Bug2] Auto-starting once sensors are live...')
        rospy.loginfo('=' * 55)

        # Main control loop at 20Hz
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.nav_state == 0:
                self.stand_by()
            elif self.nav_state == 1:
                self.bug2_go_to_point()
            elif self.nav_state == 2:
                self.bug2_follow_wall()
            elif self.nav_state == 3:
                self.waypoint_reached_behaviour()
            rate.sleep()

    # ------------------------------------------------------------------
    # State 0: Stand by - auto-start once odometry and laser are ready
    # ------------------------------------------------------------------

    def stand_by(self):
        if self.got_odom and self.got_laser:
            rospy.loginfo('[Bug2] Sensors live. Starting mission.')
            self.start_go_to_point()

    # ------------------------------------------------------------------
    # State 1: GoToPoint - drive toward goal; switch if obstacle found
    # Bug2 condition: switch to FollowWall when obstacle blocks path
    # Reference: W8 Lecture (Bug2), W9 Lecture
    # ------------------------------------------------------------------

    def bug2_go_to_point(self):
        dist = self.distance_to_goal()

        # Check if waypoint reached
        if dist < self.goal_threshold:
            self.deactivate_go_to_point()
            self.change_state(3)
            return

        # Check for obstacle on the M-line path
        if self.laser_front < self.obstacle_threshold:
            # Record hit point for M-line return condition
            self.obstacle_hit_point.x = self.position.x
            self.obstacle_hit_point.y = self.position.y
            self.obstacle_hit_dist    = dist
            rospy.loginfo(
                '[Bug2] Obstacle at %.2f m — switching to FollowWall.', dist)
            self.deactivate_go_to_point()
            self.start_follow_wall()

    # ------------------------------------------------------------------
    # State 2: FollowWall - hug obstacle; return to M-line when possible
    # Bug2 exit condition: on M-line AND closer to goal than hit point
    # Reference: W8 Lecture (Bug2 M-line condition)
    # ------------------------------------------------------------------

    def bug2_follow_wall(self):
        dist = self.distance_to_goal()

        if dist < self.goal_threshold:
            self.deactivate_follow_wall()
            self.change_state(3)
            return

        elapsed = rospy.get_time() - self.wall_follow_start
        if elapsed > 30.0 and dist >= self.obstacle_hit_dist:
            old_dir = self.direction
            self.direction = 'right' if self.direction == 'left' else 'left'
            rospy.logwarn(
                '[Bug2] Stuck detected (%.0fs). Direction flip: %s -> %s',
                elapsed, old_dir, self.direction)
            self.deactivate_follow_wall()
            self.execute_recovery()    # Back up and rotate before retrying
            self.wall_follow_start = rospy.get_time()
            self.start_follow_wall()

        # Bug2 M-line return condition
        if (self.on_m_line() and
                dist < self.obstacle_hit_dist - self.m_line_progress_buf):
            rospy.loginfo(
                '[Bug2] On M-line at dist=%.2f (hit=%.2f). Resuming GoToPoint.',
                dist, self.obstacle_hit_dist)
            self.deactivate_follow_wall()
            self.start_position.x = self.position.x
            self.start_position.y = self.position.y
            self.compute_m_line()
            self.obstacle_hit_dist = float('inf')
            self.start_go_to_point()
            return

        # Stuck detection: flip turn direction if no progress after 30 seconds
        # Reference: Assignment Brief / FollowWall.py starter file comment:
        # "The robot should be able to try different turning direction if it
        #  cannot find a path to the goal"
        if elapsed > 30.0 and dist >= self.obstacle_hit_dist:
            old_dir = self.direction
            self.direction = 'right' if self.direction == 'left' else 'left'
            rospy.logwarn(
                '[Bug2] Stuck detected (%.0fs, dist=%.2f). '
                'Flipping direction: %s -> %s',
                elapsed, dist, old_dir, self.direction)
            self.deactivate_follow_wall()
            self.execute_recovery()    # Back up and rotate before retrying
            self.wall_follow_start = rospy.get_time()
            self.start_follow_wall()    

    # ------------------------------------------------------------------
    # State 3: Waypoint reached - advance to next goal
    # ------------------------------------------------------------------

    def waypoint_reached_behaviour(self):
        lbl = self.waypoint_labels[self.current_waypoint]
        rospy.loginfo('[Bug2] *** Reached: %s ***', lbl)

        self.current_waypoint += 1

        if self.current_waypoint >= len(self.waypoints):
            rospy.loginfo('[Bug2] *** MISSION COMPLETE. All survivors found! ***')
            self.stop_robot()
            # Publish mission complete notification via custom message
            # Reference: Assignment Brief - custom messages requirement
            complete_msg = SurvivorDetected()
            complete_msg.survivor_id = 0
            complete_msg.position_x  = self.position.x
            complete_msg.position_y  = self.position.y
            complete_msg.distance    = 0.0
            complete_msg.status      = 'MISSION_COMPLETE'
            complete_msg.timestamp   = str(rospy.get_time())
            self.pub_vel.publish(Twist())
            rospy.logwarn('=' * 55)
            rospy.logwarn('[Bug2] All 3 survivors located. Robot at base.')
            rospy.logwarn('=' * 55)
            rospy.sleep(2.0)
            rospy.signal_shutdown('Mission complete')
            return

        # Set next goal and restart navigation
        next_wp = self.waypoints[self.current_waypoint]
        self.goal.x = next_wp[0]
        self.goal.y = next_wp[1]
        rospy.loginfo(
            '[Bug2] Next waypoint: %s (%.1f, %.1f)',
            self.waypoint_labels[self.current_waypoint],
            next_wp[0], next_wp[1])

        # Recompute M-line for new start-to-goal segment
        self.start_position.x = self.position.x
        self.start_position.y = self.position.y
        self.compute_m_line()
        self.obstacle_hit_dist = float('inf')
        rospy.sleep(0.5)
        self.start_go_to_point()

    # ------------------------------------------------------------------
    # M-line mathematics
    # The M-line is the straight line from start to goal
    # Reference: Bug2 algorithm (Lumelsky & Stepanov, 1987)
    # ------------------------------------------------------------------

    def compute_m_line(self):
        """Calculate slope and intercept of line from start to goal."""
        dx = self.goal.x - self.start_position.x
        dy = self.goal.y - self.start_position.y
        if abs(dx) > 1e-6:
            self.m_line_slope     = dy / dx
            self.m_line_intercept = (self.start_position.y
                                     - self.m_line_slope * self.start_position.x)
        else:
            # Vertical line - slope is infinite, intercept is the x value
            self.m_line_slope     = float('inf')
            self.m_line_intercept = self.start_position.x

    def on_m_line(self):
        """Check if robot's current position is within threshold of M-line.
        Uses perpendicular distance from point to line formula."""
        if math.isinf(self.m_line_slope):
            # Vertical line case
            return abs(self.position.x - self.m_line_intercept) < self.m_line_threshold
        # Perpendicular distance: |ax + by + c| / sqrt(a^2 + b^2)
        # where line is: slope*x - y + intercept = 0
        perp = (abs(self.m_line_slope * self.position.x
                    - self.position.y
                    + self.m_line_intercept) /
                math.sqrt(self.m_line_slope ** 2 + 1))
        return perp < self.m_line_threshold

    def distance_to_goal(self):
        """Euclidean distance from current position to current goal."""
        return math.sqrt(
            (self.goal.x - self.position.x) ** 2 +
            (self.goal.y - self.position.y) ** 2)

    # ------------------------------------------------------------------
    # Sub-behaviour activation via custom services
    # Reference: W3 Lecture (Custom Services), Assignment Brief p.2
    # ------------------------------------------------------------------

    def start_go_to_point(self):
        """Activate GoToPoint with current goal coordinates."""
        self.start_position.x = self.position.x
        self.start_position.y = self.position.y
        self.compute_m_line()

        req = MineRescueSetBugStatusRequest()
        req.flag   = True
        req.speed  = self.speed
        req.goal_x = self.goal.x
        req.goal_y = self.goal.y
        try:
            self.client_gtp(req)
            self.change_state(1)
        except rospy.ServiceException as exc:
            rospy.logerr('[Bug2] go_to_point_switch failed: %s', exc)

    def deactivate_go_to_point(self):
        """Deactivate GoToPoint - stops the robot."""
        req = MineRescueSetBugStatusRequest()
        req.flag = False
        try:
            self.client_gtp(req)
        except rospy.ServiceException as exc:
            rospy.logerr('[Bug2] go_to_point deactivate failed: %s', exc)

    def start_follow_wall(self):
        """Activate FollowWall with speed and turn direction."""
        req = MineRescueSetBugStatusRequest()
        req.flag      = True
        req.speed     = self.speed
        req.direction = self.direction
        try:
            self.client_fw(req)
            self.wall_follow_start = rospy.get_time()
            self.change_state(2)
        except rospy.ServiceException as exc:
            rospy.logerr('[Bug2] wall_follower_switch failed: %s', exc)

    def deactivate_follow_wall(self):
        """Deactivate FollowWall - stops the robot."""
        req = MineRescueSetBugStatusRequest()
        req.flag = False
        try:
            self.client_fw(req)
        except rospy.ServiceException as exc:
            rospy.logerr('[Bug2] wall_follower deactivate failed: %s', exc)

    def execute_recovery(self):
        """Recovery behaviour: back up then rotate to escape trapped positions.
        Triggered when stuck detection fires after 30 seconds.
        Reference: W9 Lecture (Bug algorithm recovery)"""
        rospy.logwarn('[Bug2] Executing recovery behaviour...')
        
        # Back up for 2 seconds to create clearance
        msg = Twist()
        msg.linear.x = -0.3
        start = rospy.get_time()
        while rospy.get_time() - start < 2.0 and not rospy.is_shutdown():
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)
        
        # Stop briefly
        self.stop_robot()
        rospy.sleep(0.5)
        
        # Rotate 90 degrees to face a new direction
        msg = Twist()
        msg.angular.z = 0.5
        start = rospy.get_time()
        while rospy.get_time() - start < 3.0 and not rospy.is_shutdown():
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)
        
        self.stop_robot()
        rospy.logwarn('[Bug2] Recovery complete. Resuming navigation.')

    def stop_robot(self):
        self.pub_vel.publish(Twist())

    def change_state(self, new_state):
        if self.nav_state != new_state:
            rospy.loginfo(
                '[Bug2] State: %s -> %s',
                self.nav_labels.get(self.nav_state, '?'),
                self.nav_labels.get(new_state, '?'))
            self.nav_state = new_state

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def callback_odom(self, msg):
        """Extract position and yaw from odometry.
        Reference: W8 Lecture Slides 28, 32, 34"""
        self.position = msg.pose.pose.position
        # Convert quaternion to Euler for yaw
        # Reference: W8 Lecture Slide 32
        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.yaw      = transformations.euler_from_quaternion(quat)[2]
        self.got_odom = True

    def callback_laser(self, msg):
        """Extract front-facing distance from laser scan.
        angle_min=-π: index 0=rear, index n//2=forward
        Uses ±30 degree cone centred on forward direction.
        Reference: W8 Lecture Slide 19"""
        ranges = list(msg.ranges)
        max_r  = msg.range_max
        n      = len(ranges)
        s      = max(1, int(n * 30 / 360))
        clean  = [r if (not math.isnan(r) and
                        not math.isinf(r) and
                        r > 0.05) else max_r
                for r in ranges]
        # mid = n//2 is forward (angle=0) for angle_min=-π laser
        mid = n//2
        self.laser_front = min(clean[mid - s: mid + s])
        self.got_laser   = True

    def callback_survivor(self, msg):
        """Log confirmed survivor detection from SurvivorDetector node.
        Reference: Assignment Brief - custom messages requirement"""
        rospy.logwarn(
            '[Bug2] Survivor confirmed: ID=%d at (%.1f, %.1f) — Status: %s',
            msg.survivor_id, msg.position_x, msg.position_y, msg.status)

    # ------------------------------------------------------------------
    # External homing override service
    # Operator can trigger immediate return to base via rosservice call
    # Reference: Assignment Brief p.2 - "homing signal" scenario
    # ------------------------------------------------------------------

    def handle_homing_signal(self, req):
        if req.flag:
            rospy.logwarn('[Bug2] EXTERNAL HOMING SIGNAL — returning to base!')
            # Skip remaining waypoints and go directly to emergency base
            self.current_waypoint = len(self.waypoints) - 1
            self.goal.x = self.waypoints[-1][0]
            self.goal.y = self.waypoints[-1][1]
            self.deactivate_follow_wall()
            self.start_go_to_point()
            return MineRescueSetBugStatusResponse(
                success=True,
                message='Homing to emergency base at (%.1f, %.1f)'
                        % (self.goal.x, self.goal.y))
        return MineRescueSetBugStatusResponse(
            success=False,
            message='Homing flag not set')


if __name__ == '__main__':
    try:
        Bug2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
