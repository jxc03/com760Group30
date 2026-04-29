#!/usr/bin/env python

# COM760 Group 30 - Collapsed School Rescue Robot
# GoToPoint.py - Moves robot from current position to target point
# Part of Bug2 navigation algorithm
#
# State machine:
#   0 - Fix heading  : rotate until facing the goal
#   1 - Go straight  : drive toward goal with heading correction
#   2 - Goal reached : stop and signal done
#
# Activated/deactivated by Bug2.py via the go_to_point_switch service
# Goal coordinates are injected through the service request (goal_x, goal_y)
#
# Reference: W8 Lecture Slides 31-36, theconstructsim.com Bug tutorial
# Source: https://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/

import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from com760cw2_group30.srv import (
    MineRescueSetBugStatus,
    MineRescueSetBugStatusResponse)

class GoToPoint:

    def __init__(self):
        rospy.init_node('go_to_point')

        self.active  = False

        # State machine: 0=Fix heading, 1=Go straight, 2=Goal reached
        # Reference: W8 Lecture Slide 31
        self.state   = 0
        self.state_labels = {
            0: 'Fix heading',
            1: 'Go straight',
            2: 'Goal reached'
        }

        # Robot position and orientation from odometry
        # Reference: W8 Lecture Slide 28 (Odometry)
        self.position = Point()
        self.yaw      = 0.0

        # Goal position - set by Bug2 via service call
        self.desired_position   = Point()
        self.desired_position.x = 0.0
        self.desired_position.y = 0.0

        # Speed parameters - overridden by service request
        self.linear_speed  = 0.5
        self.angular_speed = 0.5

        # Thresholds for state transitions
        # yaw_threshold: acceptable heading error before driving forward
        # dist_threshold: acceptable distance to consider goal reached
        # Reference: W8 Lecture Slide 35-36
        self.yaw_threshold  = math.pi / 90   # 2 degrees in radians
        self.dist_threshold = 0.25            # metres

        # Publisher: sends velocity commands to robot
        # Topic: /group30Bot/cmd_vel (as required by assignment brief)
        self.pub_vel = rospy.Publisher(
            '/group30Bot/cmd_vel', Twist, queue_size=1)

        # Subscriber: receives odometry for position and heading
        # Reference: W8 Lecture Slide 34
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.callback_odom)

        # Service server: Bug2 calls this to activate/deactivate GoToPoint
        # and to inject the goal coordinates
        # Reference: W3 Lecture (Custom Services), Assignment Brief p.2
        self.srv = rospy.Service(
            'go_to_point_switch',
            MineRescueSetBugStatus,
            self.handle_switch)

        rospy.loginfo('[GoToPoint] Ready and waiting for activation...')

        # Main control loop at 20Hz
        # Reference: W8 Practical Part E
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.active:
                rate.sleep()
                continue
            if self.state == 0:
                self.fix_heading(self.desired_position)
            elif self.state == 1:
                self.go_straight(self.desired_position)
            elif self.state == 2:
                self.done()
            else:
                rospy.logerr('[GoToPoint] Unknown state: %d', self.state)
            rate.sleep()

    # ------------------------------------------------------------------
    # Service handler: called by Bug2 to enable/disable and set goal
    # ------------------------------------------------------------------

    def handle_switch(self, req):
        self.active = req.flag
        if req.flag:
            # Override speed if provided
            if req.speed > 0:
                self.linear_speed = req.speed
            # Set goal from service request - coordinates injected by Bug2
            self.desired_position.x = req.goal_x
            self.desired_position.y = req.goal_y
            # Reset state machine from beginning
            self.state = 0
            rospy.loginfo(
                '[GoToPoint] Activated. Goal: (%.2f, %.2f) Speed: %.2f',
                req.goal_x, req.goal_y, self.linear_speed)
            return MineRescueSetBugStatusResponse(
                success=True,
                message='GoToPoint activated. Navigating to (%.2f, %.2f)'
                        % (req.goal_x, req.goal_y))
        else:
            self.stop_robot()
            return MineRescueSetBugStatusResponse(
                success=True,
                message='GoToPoint deactivated')

    # ------------------------------------------------------------------
    # Odometry callback: extract position and yaw from quaternion
    # Reference: W8 Lecture Slides 32, 34
    # ------------------------------------------------------------------

    def callback_odom(self, msg):
        self.position = msg.pose.pose.position
        # ROS stores orientation as quaternion - convert to Euler for yaw
        # Reference: W8 Lecture Slide 32 (Quaternions in ROS)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler    = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]   # yaw = rotation around Z axis

    # ------------------------------------------------------------------
    # State 0: Rotate in place until heading error is within threshold
    # Reference: W8 Lecture Slide 35
    # ------------------------------------------------------------------

    def fix_heading(self, target):
        # Calculate desired heading toward target
        desired_yaw = math.atan2(
            target.y - self.position.y,
            target.x - self.position.x)
        yaw_error = self.normalise_angle(desired_yaw - self.yaw)

        msg = Twist()
        if math.fabs(yaw_error) > self.yaw_threshold:
            # Rotate: positive = counter-clockwise, negative = clockwise
            # Reference: ROS right-hand rule, W8 Lecture Slide 25
            msg.angular.z = (self.angular_speed
                if yaw_error > 0 else -self.angular_speed)
        else:
            # Heading correct - transition to go straight
            msg.angular.z = 0.0
            self.change_state(1)
        self.pub_vel.publish(msg)

    # ------------------------------------------------------------------
    # State 1: Drive forward toward goal with continuous heading correction
    # Reference: W8 Lecture Slide 36
    # ------------------------------------------------------------------

    def go_straight(self, target):
        dist = self.distance_to(target)

        if dist > self.dist_threshold:
            # Recalculate heading correction while moving
            desired_yaw = math.atan2(
                target.y - self.position.y,
                target.x - self.position.x)
            yaw_error = self.normalise_angle(desired_yaw - self.yaw)

            msg = Twist()
            msg.linear.x  = self.linear_speed
            # Proportional heading correction while driving
            msg.angular.z = 0.3 * yaw_error
            self.pub_vel.publish(msg)
        else:
            # Within threshold - goal reached
            self.change_state(2)

    # ------------------------------------------------------------------
    # State 2: Stop and signal completion to Bug2
    # ------------------------------------------------------------------

    def done(self):
        self.stop_robot()
        rospy.loginfo('[GoToPoint] Goal reached!')
        self.active = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def change_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(
                '[GoToPoint] State: %s -> %s',
                self.state_labels.get(self.state, '?'),
                self.state_labels.get(new_state, '?'))
            self.state = new_state

    def stop_robot(self):
        self.pub_vel.publish(Twist())

    def distance_to(self, target):
        return math.sqrt(
            (target.x - self.position.x) ** 2 +
            (target.y - self.position.y) ** 2)

    @staticmethod
    def normalise_angle(angle):
        """Normalise angle to range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == '__main__':
    try:
        GoToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
