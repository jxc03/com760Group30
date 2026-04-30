#!/usr/bin/env python

# COM760 Group 30 - Collapsed School Rescue Robot
# SurvivorDetector.py - Detects survivors by proximity in simulation
#
# Monitors robot's odometry position and publishes a SurvivorDetected
# custom message when the robot comes within detection_range of a
# known survivor location.
#
# Survivor locations (must match Bug2.py waypoints and world file):
#   Child 1  at (-8.0,  3.0) - NW classroom
#   Teacher  at ( 0.0, -3.0) - south corridor
#   Child 2  at ( 7.0,  3.0) - east corridor
#
# Reference: Assignment Brief - custom messages requirement
# Custom message: SurvivorDetected.msg

import rospy
import math
from nav_msgs.msg import Odometry
from com760cw2_group30.msg import SurvivorDetected
from com760cw2_group30.srv import (
    MineRescueSetBugStatus,
    MineRescueSetBugStatusRequest)

class SurvivorDetector:

    def __init__(self):
        rospy.init_node('survivor_detector')

        # Survivor positions from ROS parameter server
        # Edit coordinates in school_rescue_bug2.launch, not here
        # Reference: Assignment Brief - custom messages requirement
        self.survivors = [
            {'id': 1,
            'x': rospy.get_param('survivor_1_x', -6.0),
            'y': rospy.get_param('survivor_1_y',  3.0),
            'found': False, 'type': 'Survivor'},
            {'id': 2,
            'x': rospy.get_param('survivor_2_x',  0.0),
            'y': rospy.get_param('survivor_2_y', -3.0),
            'found': False, 'type': 'Survivor'},
            {'id': 3,
            'x': rospy.get_param('survivor_3_x',  5.0),
            'y': rospy.get_param('survivor_3_y',  3.0),
            'found': False, 'type': 'Survivor'},
        ]
        self.homing_sent = False

        # Detection range in metres
        # Robot must be within this distance to trigger detection
        self.detection_range = 2.0
        self.position_x  = 0.0
        self.position_y  = 0.0
        self.total_found = 0

        # Publisher: SurvivorDetected custom message
        # Reference: Assignment Brief - custom messages requirement
        # Topic: /group30Bot/survivor_detected
        self.pub_survivor = rospy.Publisher(
            '/group30Bot/survivor_detected',
            SurvivorDetected, queue_size=10)

        # Subscriber: robot position from odometry
        self.sub_odom = rospy.Subscriber(
            '/odom', Odometry, self.callback_odom)

        rospy.loginfo('=' * 50)
        rospy.loginfo('[SurvivorDetector] Mission targets (from parameter server):')
        for s in self.survivors:
            rospy.loginfo('[SurvivorDetector]   Survivor %d at (%.1f, %.1f)',
                          s['id'], s['x'], s['y'])
        rospy.loginfo('[SurvivorDetector]   Ambulance at (%.1f, %.1f)',
                      rospy.get_param('ambulance_x', 11.0),
                      rospy.get_param('ambulance_y',  0.0))
        rospy.loginfo('[SurvivorDetector] Detection range: %.1f m', self.detection_range)
        rospy.loginfo('=' * 50)

        # Check for survivors at 5Hz
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.check_survivors()
            rate.sleep()

    def callback_odom(self, msg):
        """Update robot position from odometry."""
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

    def check_survivors(self):
        """Check proximity to all unfound survivors and publish detections."""
        for survivor in self.survivors:
            if survivor['found']:
                continue

            dist = math.sqrt(
                (self.position_x - survivor['x']) ** 2 +
                (self.position_y - survivor['y']) ** 2)

            if dist < self.detection_range:
                survivor['found'] = True
                self.total_found += 1

                # Publish custom SurvivorDetected message
                # Reference: Assignment Brief - custom messages requirement
                msg = SurvivorDetected()
                msg.survivor_id = survivor['id']
                msg.position_x  = survivor['x']
                msg.position_y  = survivor['y']
                msg.distance    = dist
                msg.status      = 'ALIVE'
                msg.timestamp   = str(rospy.get_time())
                self.pub_survivor.publish(msg)

                rospy.logwarn('=' * 50)
                rospy.logwarn('*** SCHOOL SURVIVOR FOUND! ***')
                rospy.logwarn('*** Type: %s ***', survivor['type'])
                rospy.logwarn('*** ID: %d  Location: (%.1f, %.1f) ***',
                              survivor['id'], survivor['x'], survivor['y'])
                rospy.logwarn('*** Distance: %.2f metres ***', dist)
                rospy.logwarn('*** Total found: %d/3 ***', self.total_found)
                rospy.logwarn('=' * 50)

                if self.total_found == len(self.survivors):
                    rospy.logwarn('=' * 50)
                    rospy.logwarn('*** ALL SURVIVORS FOUND! ***')
                    rospy.logwarn('*** Robot returning to emergency base! ***')
                    rospy.logwarn('=' * 50)
                    
                if self.total_found == len(self.survivors) and not self.homing_sent:
                    self.signal_return_to_base()

    def callback_survivor_detected(self, msg):
        """Sync our found-flags with any detection published on the topic
        (e.g. by Bug2's proximity check) so homing fires at the right time."""
        idx = msg.survivor_id - 1   # survivor_id is 1-based
        if 0 <= idx < len(self.survivors) and not self.survivors[idx]['found']:
            self.survivors[idx]['found'] = True
            self.total_found += 1
            rospy.loginfo('[SurvivorDetector] Synced external detection:'
                          ' survivor %d (total %d/3)', msg.survivor_id, self.total_found)
            if self.total_found == 3 and not self.homing_sent:
                self.signal_return_to_base()

    def signal_return_to_base(self):
        """Call Bug2 homing service so robot returns to emergency base."""
        rospy.logwarn('=' * 50)
        rospy.logwarn('*** ALL 3 SURVIVORS FOUND! ***')
        rospy.logwarn('*** Sending homing signal to Bug2 ***')
        rospy.logwarn('*** Robot returning to emergency base ***')
        rospy.logwarn('=' * 50)
        try:
            rospy.wait_for_service('mine_rescue_homing', timeout=3.0)
            svc = rospy.ServiceProxy(
                'mine_rescue_homing', MineRescueSetBugStatus)
            req      = MineRescueSetBugStatusRequest()
            req.flag = True
            svc(req)
            self.homing_sent = True
        except Exception as exc:
            rospy.logwarn('[SurvivorDetector] Homing service not available: %s', exc)

if __name__ == '__main__':
    try:
        SurvivorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
