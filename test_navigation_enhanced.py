#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import json
import time

class EnhancedNavigationTester:
    def __init__(self):
        rospy.init_node('enhanced_navigation_tester', anonymous=True)
        
        # Publishers for robot commands
        self.mover_lata_pub = rospy.Publisher('/info_robot_mover_lata', String, queue_size=1)
        self.mover_a_lata_pub = rospy.Publisher('/info_robot_mover_a_lata', String, queue_size=1)
        
        # Subscribers for camera responses
        self.mover_lata_sub = rospy.Subscriber('/info_camara_mover_lata', String, self.camera_response_callback)
        self.mover_a_lata_sub = rospy.Subscriber('/info_camara_mover_a_lata', String, self.camera_response_callback)
        
        # Subscriber for gripper control monitoring
        self.gripper_sub = rospy.Subscriber('/gripper_control', String, self.gripper_response_callback)
        
        # Track responses
        self.last_action = None
        self.last_gripper_action = None
        self.test_results = []
        
        rospy.loginfo("Enhanced Navigation Tester initialized")
        rospy.loginfo("Monitoring topics:")
        rospy.loginfo("  - /info_camara_mover_lata")
        rospy.loginfo("  - /info_camara_mover_a_lata") 
        rospy.loginfo("  - /gripper_control")
        
    def camera_response_callback(self, msg):
        """Handle responses from the camera navigation system"""
        self.last_action = msg.data
        rospy.loginfo("Camera response: %s", msg.data)
        
        # Log specific actions for analysis
        if "accion:Atras" in msg.data:
            rospy.loginfo("*** BACKWARD MOVEMENT DETECTED ***")
        elif "accion:Destino" in msg.data:
            rospy.loginfo("*** DESTINATION REACHED ***")
            
    def gripper_response_callback(self, msg):
        """Handle gripper control commands"""
        self.last_gripper_action = msg.data
        rospy.loginfo("Gripper command: %s", msg.data)
        
        # Log gripper actions for analysis
        if "gripper:open" in msg.data:
            rospy.loginfo("*** GRIPPER OPENED ***")
        elif "gripper:close" in msg.data:
            rospy.loginfo("*** GRIPPER CLOSED ***")
        
    def test_move_can_with_gripper(self, can_id, origin_zone, dest_zone):
        """Test moving a can with automatic gripper control"""
        rospy.loginfo("="*60)
        rospy.loginfo("TEST: Move can %d from zone %d to zone %d", can_id, origin_zone, dest_zone)
        rospy.loginfo("Expected behavior:")
        rospy.loginfo("  1. Navigate to origin zone %d", origin_zone)
        rospy.loginfo("  2. Open gripper when approaching can %d", can_id)
        rospy.loginfo("  3. Close gripper when reaching can %d", can_id)
        rospy.loginfo("  4. Navigate to destination zone %d", dest_zone)
        rospy.loginfo("  5. Open gripper to drop can at destination")
        rospy.loginfo("="*60)
        
        # Create command message
        command = {
            "lata": can_id,
            "origen": origin_zone,
            "destino": dest_zone
        }
        
        message = String()
        message.data = json.dumps(command)
        
        # Reset tracking variables
        self.last_action = None
        self.last_gripper_action = None
        
        # Publish command
        self.mover_lata_pub.publish(message)
        rospy.loginfo("Published move can command: %s", message.data)
        
        return self.monitor_test_execution("mover_lata", 30.0)
        
    def test_move_to_can_with_gripper(self, origin_zone, dest_zone):
        """Test moving robot to a can location with gripper control"""
        rospy.loginfo("="*60)
        rospy.loginfo("TEST: Move robot from zone %d to zone %d", origin_zone, dest_zone)
        rospy.loginfo("Expected behavior:")
        rospy.loginfo("  1. Navigate from zone %d to zone %d", origin_zone, dest_zone)
        rospy.loginfo("  2. Open gripper when approaching destination")
        rospy.loginfo("  3. Close gripper when reaching destination")
        rospy.loginfo("="*60)
        
        # Create command message
        command = {
            "origen": origin_zone,
            "destino": dest_zone
        }
        
        message = String()
        message.data = json.dumps(command)
        
        # Reset tracking variables
        self.last_action = None
        self.last_gripper_action = None
        
        # Publish command
        self.mover_a_lata_pub.publish(message)
        rospy.loginfo("Published move to can command: %s", message.data)
        
        return self.monitor_test_execution("mover_a_lata", 20.0)
        
    def monitor_test_execution(self, test_type, timeout):
        """Monitor test execution and collect results"""
        start_time = rospy.Time.now()
        actions_received = []
        gripper_actions = []
        backward_detected = False
        destination_reached = False
        
        rospy.loginfo("Monitoring test execution for %.1f seconds...", timeout)
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            # Check for new actions
            if self.last_action and self.last_action not in actions_received:
                actions_received.append(self.last_action)
                
                if "accion:Atras" in self.last_action:
                    backward_detected = True
                elif "accion:Destino" in self.last_action:
                    destination_reached = True
                    break  # Test completed
                    
            # Check for gripper actions
            if self.last_gripper_action and self.last_gripper_action not in gripper_actions:
                gripper_actions.append(self.last_gripper_action)
                
            rospy.sleep(0.1)
            
        # Analyze results
        test_result = {
            'type': test_type,
            'duration': (rospy.Time.now() - start_time).to_sec(),
            'actions_received': actions_received,
            'gripper_actions': gripper_actions,
            'backward_detected': backward_detected,
            'destination_reached': destination_reached,
            'success': destination_reached
        }
        
        self.test_results.append(test_result)
        self.print_test_results(test_result)
        
        return test_result
        
    def print_test_results(self, result):
        """Print detailed test results"""
        rospy.loginfo("-"*50)
        rospy.loginfo("TEST RESULTS:")
        rospy.loginfo("  Type: %s", result['type'])
        rospy.loginfo("  Duration: %.1f seconds", result['duration'])
        rospy.loginfo("  Success: %s", "YES" if result['success'] else "NO")
        rospy.loginfo("  Backward movement used: %s", "YES" if result['backward_detected'] else "NO")
        rospy.loginfo("  Actions received: %d", len(result['actions_received']))
        for action in result['actions_received']:
            rospy.loginfo("    - %s", action)
        rospy.loginfo("  Gripper actions: %d", len(result['gripper_actions']))
        for action in result['gripper_actions']:
            rospy.loginfo("    - %s", action)
        rospy.loginfo("-"*50)
        
    def test_backward_movement_scenario(self):
        """Test scenario designed to trigger backward movement"""
        rospy.loginfo("="*60)
        rospy.loginfo("SPECIAL TEST: Backward Movement Scenario")
        rospy.loginfo("This test is designed to trigger 'Atras' command")
        rospy.loginfo("Moving can from zone 1 to zone 1 (same zone)")
        rospy.loginfo("="*60)
        
        return self.test_move_can_with_gripper(can_id=1, origin_zone=1, dest_zone=1)
        
    def test_gripper_threshold_behavior(self):
        """Test gripper behavior at different distances"""
        rospy.loginfo("="*60)
        rospy.loginfo("SPECIAL TEST: Gripper Threshold Behavior")
        rospy.loginfo("This test monitors gripper opening/closing timing")
        rospy.loginfo("="*60)
        
        return self.test_move_can_with_gripper(can_id=2, origin_zone=2, dest_zone=3)
        
    def run_comprehensive_tests(self):
        """Run a comprehensive test suite"""
        rospy.loginfo("Starting comprehensive navigation and gripper tests...")
        rospy.loginfo("Make sure ArUco markers are properly positioned!")
        
        # Wait for system to be ready
        rospy.sleep(3.0)
        
        tests = [
            # Basic can movement tests
            ("Basic Can Movement 1", lambda: self.test_move_can_with_gripper(1, 1, 2)),
            ("Basic Can Movement 2", lambda: self.test_move_can_with_gripper(2, 2, 3)),
            
            # Robot movement tests
            ("Robot Movement 1", lambda: self.test_move_to_can_with_gripper(1, 3)),
            ("Robot Movement 2", lambda: self.test_move_to_can_with_gripper(3, 1)),
            
            # Special scenario tests
            ("Backward Movement Test", self.test_backward_movement_scenario),
            ("Gripper Threshold Test", self.test_gripper_threshold_behavior),
            
            # Complex movement test
            ("Complex Movement", lambda: self.test_move_can_with_gripper(3, 3, 1)),
        ]
        
        successful_tests = 0
        total_tests = len(tests)
        
        for test_name, test_func in tests:
            rospy.loginfo("\n" + "="*80)
            rospy.loginfo("RUNNING: %s", test_name)
            rospy.loginfo("="*80)
            
            try:
                result = test_func()
                if result['success']:
                    successful_tests += 1
                    rospy.loginfo("✓ %s PASSED", test_name)
                else:
                    rospy.logwarn("✗ %s FAILED", test_name)
                    
            except Exception as e:
                rospy.logerr("✗ %s ERROR: %s", test_name, str(e))
                
            # Wait between tests
            rospy.sleep(2.0)
            
        # Print final summary
        self.print_final_summary(successful_tests, total_tests)
        
    def print_final_summary(self, successful, total):
        """Print final test summary"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("FINAL TEST SUMMARY")
        rospy.loginfo("="*80)
        rospy.loginfo("Tests passed: %d/%d (%.1f%%)", successful, total, 
                     (successful/float(total))*100 if total > 0 else 0)
        
        # Analyze specific features
        backward_tests = sum(1 for r in self.test_results if r['backward_detected'])
        gripper_tests = sum(1 for r in self.test_results if len(r['gripper_actions']) > 0)
        
        rospy.loginfo("Backward movement detected in: %d tests", backward_tests)
        rospy.loginfo("Gripper control active in: %d tests", gripper_tests)
        
        # Feature analysis
        rospy.loginfo("\nFEATURE ANALYSIS:")
        rospy.loginfo("- Backward movement ('Atras'): %s", 
                     "WORKING" if backward_tests > 0 else "NOT DETECTED")
        rospy.loginfo("- Gripper control: %s", 
                     "WORKING" if gripper_tests > 0 else "NOT DETECTED")
        rospy.loginfo("- Navigation completion: %s", 
                     "WORKING" if successful > 0 else "FAILED")
        
        rospy.loginfo("="*80)

def main():
    try:
        tester = EnhancedNavigationTester()
        
        rospy.loginfo("Enhanced Navigation Tester Ready!")
        rospy.loginfo("Available test methods:")
        rospy.loginfo("  - run_comprehensive_tests(): Full test suite")
        rospy.loginfo("  - test_move_can_with_gripper(can_id, origin, dest)")
        rospy.loginfo("  - test_move_to_can_with_gripper(origin, dest)")
        rospy.loginfo("  - test_backward_movement_scenario()")
        rospy.loginfo("  - test_gripper_threshold_behavior()")
        
        # Ask user what to do
        rospy.loginfo("\nStarting comprehensive tests in 5 seconds...")
        rospy.loginfo("Press Ctrl+C to cancel and use manual testing")
        
        try:
            rospy.sleep(5.0)
            tester.run_comprehensive_tests()
        except KeyboardInterrupt:
            rospy.loginfo("Comprehensive tests cancelled. Entering manual mode.")
            
        # Keep node alive for manual testing
        rospy.loginfo("\nNode ready for manual testing. Use the methods above.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Enhanced navigation tester shutting down")

if __name__ == '__main__':
    main() 