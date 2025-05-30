#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Test script to help identify available ArUco markers and test robot detection
"""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArUcoTester:
    def __init__(self):
        rospy.init_node("aruco_tester", anonymous=True)
        self.bridge = CvBridge()
        
        # Test different robot IDs
        self.test_robot_ids = [0, 1, 2, 3, 4, 5, 15, 17, 18, 19]
        self.current_test_id = 0
        
        # Subscribe to camera
        self.sub = rospy.Subscriber("/csi_cam_0/image_raw", Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("ArUco Tester started. Press 'n' to test next robot ID, 'q' to quit")
        rospy.loginfo("Testing robot ID: %d", self.test_robot_ids[self.current_test_id])
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))
            
            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # ArUco detection
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Draw detected markers
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                
                detected_ids = ids.flatten().tolist()
                current_robot_id = self.test_robot_ids[self.current_test_id]
                
                # Check if current test robot ID is detected
                robot_detected = current_robot_id in detected_ids
                
                # Display information
                info_text = "Detected IDs: {}".format(detected_ids)
                cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                test_text = "Testing Robot ID: {} - {}".format(
                    current_robot_id, 
                    "FOUND!" if robot_detected else "Not found"
                )
                color = (0, 255, 0) if robot_detected else (0, 0, 255)
                cv2.putText(frame, test_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # If robot is detected, show orientation info
                if robot_detected:
                    for i, id_ in enumerate(detected_ids):
                        if id_ == current_robot_id:
                            c = corners[detected_ids.index(id_)]
                            centro = tuple(np.mean(c.reshape(4, 2), axis=0).astype(int))
                            
                            # Draw robot center
                            cv2.circle(frame, centro, 10, (0, 255, 0), -1)
                            cv2.putText(frame, "ROBOT", (centro[0] + 15, centro[1]), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            
                            # Calculate and show orientation
                            orientation = self.calculate_orientation(c.reshape(4, 2))
                            if orientation is not None:
                                orient_text = "Orientation: {:.1f}°".format(orientation)
                                cv2.putText(frame, orient_text, (10, 90), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                
                                # Draw orientation arrow
                                arrow_length = 50
                                angle_rad = np.radians(orientation)
                                end_x = int(centro[0] + arrow_length * np.cos(angle_rad))
                                end_y = int(centro[1] + arrow_length * np.sin(angle_rad))
                                cv2.arrowedLine(frame, centro, (end_x, end_y), (0, 255, 0), 3)
                            break
                
                # Instructions
                cv2.putText(frame, "Press 'n' for next ID, 'q' to quit", (10, frame.shape[0] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                cv2.putText(frame, "No ArUco markers detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow("ArUco Robot ID Tester", frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('n'):
                self.current_test_id = (self.current_test_id + 1) % len(self.test_robot_ids)
                rospy.loginfo("Testing robot ID: %d", self.test_robot_ids[self.current_test_id])
            elif key == ord('q'):
                rospy.signal_shutdown("User quit")
                
        except Exception as e:
            rospy.logerr("Error in image callback: %s", str(e))
    
    def calculate_orientation(self, corners):
        """Simple orientation calculation for testing"""
        try:
            # Calculate center
            center = np.mean(corners, axis=0)
            
            # Use top edge (corners 0 and 1)
            top_center = (corners[0] + corners[1]) / 2.0
            
            # Vector from center to top
            forward_vector = top_center - center
            
            # Calculate angle
            angle_rad = np.arctan2(forward_vector[1], forward_vector[0])
            angle_deg = np.degrees(angle_rad)
            
            # Normalize to 0-360
            if angle_deg < 0:
                angle_deg += 360
                
            return angle_deg
            
        except Exception as e:
            rospy.logerr("Error calculating orientation: %s", str(e))
            return None

if __name__ == "__main__":
    try:
        tester = ArUcoTester()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows() 