#!/usr/bin/env python

# COM760 Group 30 - Collapsed School Rescue Robot
# FollowWall.py - Makes robot follow walls and obstacles
# Part of Bug2 navigation algorithm
#
# State machine:
#   0 - Find wall  : move forward until wall detected
#   1 - Turn       : rotate away from obstacle ahead
#   2 - Follow wall: drive parallel to wall maintaining set distance
#
# Activated/deactivated by Bug2.py via the wall_follower_switch service
# Turn direction (left/right) is injected through the service request
#
# Reference: W8 Lecture Slides 38-41, theconstructsim.com Bug tutorial
# Source: https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from com760cw2_group30.srv import (
    MineRescueSetBugStatus,
    MineRescueSetBugStatusResponse)

class FollowWall:

    def __init__(self):
        rospy.init_node('follow_wall')

        self.active = False

        # State machine: 0=Find wall, 1=Turn, 2=Follow wall
        # Reference: W8 Lecture Slides 38-41
        self.state = 0
        self.state_labels = {
            0: 'Find wall',
            1: 'Turn',
            2: 'Follow wall'
        }

        # Speed parameters - overridden by service request
        self.linear_speed   = 0.3
        self.angular_speed  = 0.3

        # Turn direction set by Bug2 via service - 'left' or 'right'
        # Reference: Assignment Brief p.2 - "turning direction on obstacle contact"
        self.turn_direction = 'left'

        # Laser region distances (initialised to safe large value)
        # 360 degree laser split into front, left and right sectors
        # Reference: W8 Lecture Slide 19 and 24
        self.front = float('inf')
        self.left  = float('inf')
        self.right = float('inf')

        # Wall detection thresholds (metres)
        self.wall_threshold = 0.6   # obstacle detected within this distance
        self.follow_dist    = 0.55  # desired distance to maintain from wall

        # Minimum time in each state to prevent rapid flickering
        self.last_change    = 0.0
        self.min_state_time = 1.0   # seconds

        # Publisher: sends velocity commands to robot
        # Topic: /group30Bot/cmd_vel (required by assignment brief)
        self.pub_vel = rospy.Publisher(
            '/group30Bot/cmd_vel', Twist, queue_size=1)

        # Subscriber: receives laser scan data for obstacle detection
        # Topic: /group30Bot/laser/scan (required by assignment brief)
        # 360 samples covering full 360 degrees
        # Reference: W8 Lecture Slide 10, W8 Practical Part A
        self.sub_laser = rospy.Subscriber(
            '/group30Bot/laser/scan',
            LaserScan, self.callback_laser)

        # Service server: Bug2 calls this to activate/deactivate FollowWall
        # Reference: W3 Lecture (Custom Services), Assignment Brief p.2
        self.srv = rospy.Service(
            'wall_follower_switch',
            MineRescueSetBugStatus,
            self.handle_switch)

        rospy.loginfo('[FollowWall] Ready and waiting for activation...')

        # Main control loop at 10Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue

            msg = Twist()
            if self.state == 0:
                msg = self.find_wall()
            elif self.state == 1:
                msg = self.turn()
            elif self.state == 2:
                msg = self.follow_the_wall()
            else:
                rospy.logerr('[FollowWall] Unknown state: %d', self.state)

            self.pub_vel.publish(msg)
            rate.sleep()

    # ------------------------------------------------------------------
    # Service handler: called by Bug2 to enable/disable wall following
    # ------------------------------------------------------------------

    def handle_switch(self, req):
        self.active = req.flag
        if req.flag:
            # Override speed if provided
            if req.speed > 0:
                self.linear_speed = req.speed * 0.6  # wall follow is slower
            # Set turn direction from service request
            self.turn_direction = (req.direction
                if req.direction in ['left', 'right'] else 'left')
            # Reset state machine
            self.state       = 0
            self.last_change = rospy.get_time()
            rospy.loginfo(
                '[FollowWall] Activated. Turn direction: %s Speed: %.2f',
                self.turn_direction, self.linear_speed)
            return MineRescueSetBugStatusResponse(
                success=True,
                message='FollowWall activated. Direction: %s'
                        % self.turn_direction)
        else:
            self.stop_robot()
            return MineRescueSetBugStatusResponse(
                success=True,
                message='FollowWall deactivated')

    # ------------------------------------------------------------------
    # Laser callback: extract front, left and right distances
    # 360 degree laser divided into sectors
    # Reference: W8 Lecture Slides 19, 24
    # ------------------------------------------------------------------

    def callback_laser(self, msg):
        ranges = list(msg.ranges)
        max_r  = msg.range_max
        n      = len(ranges)

        # Sector size: ±30 degrees from each direction
        s = max(1, int(n * 30 / 360))

        # Clean invalid readings: replace nan/inf with max range
        clean = [r if (not math.isnan(r) and
                       not math.isinf(r) and
                       r > 0.05) else max_r
                 for r in ranges]

        # Front sector: covers both ends of the array (0° = front for 360° laser)
        self.front = min(min(clean[-s:]), min(clean[:s]))

        # Left and right sectors
        self.left  = (min(clean[s * 3: n // 2])
                      if s * 3 < n // 2 else max_r)
        self.right = (min(clean[n // 2: -(s * 3)])
                      if s * 3 < n // 2 else max_r)

    # ------------------------------------------------------------------
    # State 0: Find wall - drive forward until obstacle detected
    # Reference: W8 Lecture Slide 40
    # ------------------------------------------------------------------

    def find_wall(self):
        if rospy.get_time() - self.last_change > self.min_state_time:
            if self.front < self.wall_threshold:
                # Obstacle ahead - switch to turn
                self.change_state(1)
            elif self.left < 0.7 or self.right < 0.7:
                # Wall alongside - switch to follow
                self.change_state(2)

        msg = Twist()
        msg.linear.x  = self.linear_speed
        msg.angular.z = 0.0
        return msg

    # ------------------------------------------------------------------
    # State 1: Turn - rotate until front is clear
    # Turn direction determined by service request from Bug2
    # Reference: W8 Lecture Slide 40, Assignment Brief p.2
    # ------------------------------------------------------------------

    def turn(self):
        if rospy.get_time() - self.last_change > 1.5:
            if self.front > self.wall_threshold:
                # Front clear - switch to follow
                self.change_state(2)

        msg = Twist()
        msg.linear.x  = 0.0
        # Positive angular.z = counter-clockwise (left)
        # Negative angular.z = clockwise (right)
        # Reference: ROS right-hand rule, W8 Lecture Slide 25
        msg.angular.z = (self.angular_speed
            if self.turn_direction == 'left'
            else -self.angular_speed)
        return msg

    # ------------------------------------------------------------------
    # State 2: Follow wall - drive parallel maintaining distance
    # Proportional control keeps robot at follow_dist from wall
    # Reference: W8 Lecture Slide 41
    # ------------------------------------------------------------------

    def follow_the_wall(self):
        if rospy.get_time() - self.last_change > self.min_state_time:
            if self.front < self.wall_threshold - 0.1:
                # Obstacle ahead while following - turn
                self.change_state(1)
            elif self.left > 1.5 and self.right > 1.5:
                # Lost the wall - find it again
                self.change_state(0)

        msg = Twist()
        msg.linear.x = self.linear_speed

        # Proportional correction to maintain follow_dist from wall
        if self.turn_direction == 'left':
            # Following wall on right side
            error = self.follow_dist - self.right
            msg.angular.z = max(-0.3, min(0.3, -error * 0.4))
        else:
            # Following wall on left side
            error = self.follow_dist - self.left
            msg.angular.z = max(-0.3, min(0.3, error * 0.4))

        return msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def change_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(
                '[FollowWall] State: %s -> %s',
                self.state_labels.get(self.state, '?'),
                self.state_labels.get(new_state, '?'))
            self.state       = new_state
            self.last_change = rospy.get_time()

    def stop_robot(self):
        self.pub_vel.publish(Twist())


if __name__ == '__main__':
    try:
        FollowWall()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
