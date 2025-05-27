#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
import time

class NavigationTester:
    def __init__(self):
        rospy.init_node('navigation_tester', anonymous=True)
        
        # Publishers for robot commands
        self.mover_lata_pub = rospy.Publisher('/info_robot_mover_lata', String, queue_size=1)
        self.mover_a_lata_pub = rospy.Publisher('/info_robot_mover_a_lata', String, queue_size=1)
        
        # Subscribers for camera responses
        self.mover_lata_sub = rospy.Subscriber('/info_camara_mover_lata', String, self.camera_response_callback)
        self.mover_a_lata_sub = rospy.Subscriber('/info_camara_mover_a_lata', String, self.camera_response_callback)
        
        rospy.loginfo("Navigation Tester initialized")
        
    def camera_response_callback(self, msg):
        """Handle responses from the camera navigation system"""
        rospy.loginfo("Camera response: %s", msg.data)
        
    def test_move_can(self, can_id, origin_zone, dest_zone):
        """Test moving a can from origin to destination"""
        rospy.loginfo("Testing move can %d from zone %d to zone %d", can_id, origin_zone, dest_zone)
        
        # Create command message
        command = {
            "lata": can_id,
            "origen": origin_zone,
            "destino": dest_zone
        }
        
        message = String()
        message.data = json.dumps(command)
        
        # Publish command
        self.mover_lata_pub.publish(message)
        rospy.loginfo("Published move can command: %s", message.data)
        
    def test_move_to_can(self, origin_zone, dest_zone):
        """Test moving robot to a can location"""
        rospy.loginfo("Testing move to can from zone %d to zone %d", origin_zone, dest_zone)
        
        # Create command message
        command = {
            "origen": origin_zone,
            "destino": dest_zone
        }
        
        message = String()
        message.data = json.dumps(command)
        
        # Publish command
        self.mover_a_lata_pub.publish(message)
        rospy.loginfo("Published move to can command: %s", message.data)
        
    def run_tests(self):
        """Run a series of navigation tests"""
        rospy.loginfo("Starting navigation tests...")
        
        # Wait for system to be ready
        rospy.sleep(2.0)
        
        # Test 1: Move can from zone 1 to zone 2
        self.test_move_can(can_id=1, origin_zone=1, dest_zone=2)
        rospy.sleep(5.0)
        
        # Test 2: Move robot to zone 3
        self.test_move_to_can(origin_zone=2, dest_zone=3)
        rospy.sleep(5.0)
        
        # Test 3: Move can from zone 3 to zone 1
        self.test_move_can(can_id=2, origin_zone=3, dest_zone=1)
        rospy.sleep(5.0)
        
        rospy.loginfo("Navigation tests completed")

def main():
    try:
        tester = NavigationTester()
        
        # Run automated tests
        tester.run_tests()
        
        # Keep node alive for manual testing
        rospy.loginfo("Tester ready for manual commands. Use the following methods:")
        rospy.loginfo("  tester.test_move_can(can_id, origin_zone, dest_zone)")
        rospy.loginfo("  tester.test_move_to_can(origin_zone, dest_zone)")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation tester shutting down")

if __name__ == '__main__':
    main() 