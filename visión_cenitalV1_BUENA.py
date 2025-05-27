#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import os
import json
import threading
import time

class Navegacion_Robot:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)
        
        # Initialize CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # System parameters
        self.frame_size = (640, 480)
        
        # Camera calibration parameters
        self.calibration_done = False
        
        # Default camera matrix (will be updated during calibration)
        self.camera_matrix = np.array([
            [500.0, 0, self.frame_size[0]/2],
            [0, 500.0, self.frame_size[1]/2],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Default distortion coefficients
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # World points for homography calculation
        self.world_pts = np.array([
            [  600,   600],   # Bottom left
            [  600, 18800],   # Top left
            [28800,   600],   # Bottom right
            [28800, 18800],   # Top right
        ], dtype=np.float32)

        # Dictionary to store ArUco information by zone
        self.info_zonas = {
            i: {'arucos': [], 'centros': []} for i in range(1, 9)
        }
        
        # Reference ArUcos for homography (IDs 20-23)
        self.arucos_medio = {i: [] for i in range(20, 24)}
        self.centros_medio_IMG_Points = []
        
        # Current homography matrix
        self.current_homography = None

        # ROS subscriptions and publications
        self.sub = rospy.Subscriber("/csi_cam_0/image_raw", Image, self.image_callback, queue_size=1)
        
        # Topics for robot communication
        self.mover_lata_pub = rospy.Publisher("/info_camara_mover_lata", String, queue_size=1)
        self.mover_a_lata_pub = rospy.Publisher("/info_camara_mover_a_lata", String, queue_size=1)
        
        # Gripper control topic
        self.gripper_pub = rospy.Publisher("/gripper_control", String, queue_size=1) ## revisar
        
        # Subscriptions to robot messages
        self.mover_lata_sub = rospy.Subscriber("/info_robot_mover_lata", String, 
                                            lambda msg: self.process_robot_message(msg, "mover_lata"), 
                                            queue_size=1)
        self.mover_a_lata_sub = rospy.Subscriber("/info_robot_mover_a_lata", String, 
                                                lambda msg: self.process_robot_message(msg, "mover_a_lata"), 
                                                queue_size=1)

        # Robot state - ENHANCED WITH ORIENTATION
        self.robot_state = {
            'lata_actual': None,
            'almacen_origen': None,
            'almacen_destino': None,
            'accion': "Parar",
            'modo': None,
            'navegando': False,
            'gripper_state': 'closed',
            'approaching_can': False,
            'can_picked': False,
            'position': None,           # (x, y) current position
            'orientation': None,        # Current heading in degrees (0-360)
            'orientation_vector': None, # (dx, dy) unit vector of robot facing direction
            'last_valid_orientation': 0.0  # Backup orientation value
        }

        self.robot_id = 0
        
        # Distance thresholds for gripper control
        self.gripper_open_threshold = 80   # Distance to open gripper (pixels)
        self.gripper_close_threshold = 30  # Distance to close gripper (pixels)
        self.destination_threshold = 50    # Distance to consider arrived at destination
        
        # Orientation calculation parameters
        self.orientation_smoothing = 0.7   # Smoothing factor for orientation (0-1)
        self.min_orientation_change = 5.0  # Minimum change in degrees to update orientation
        
        # Calibration variables
        self.calibration_frames = []
        self.calibration_count = 0
        self.max_calibration_frames = 10
        
        # Thread safety
        self.processing_lock = threading.Lock()
        
        # Load previous calibration if available
        self.load_calibration()
        
        rospy.loginfo("Robot Navigation System initialized with orientation tracking")

    def preprocess_frame(self, frame):
        """
        Preprocesses the input frame for ArUco detection
        """
        try:
            if frame is None:
                return None
                
            # Resize if necessary
            if frame.shape[:2] != (self.frame_size[1], self.frame_size[0]):
                frame = cv2.resize(frame, self.frame_size)
            
            # Apply undistortion if camera is calibrated
            if self.calibration_done:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)
            
            # Convert to grayscale and enhance
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Reduced blur for better ArUco detection
            gray = cv2.equalizeHist(gray)
            
            return gray
            
        except Exception as e:
            rospy.logerr("Error in preprocess_frame: %s", str(e))
            return None

    def detect_arucos(self, frame):
        """
        Detects ArUco markers in the frame and categorizes them by zones
        """
        try:
            # Clear previous detections
            for zona_id in self.info_zonas:
                self.info_zonas[zona_id]['arucos'] = []
                self.info_zonas[zona_id]['centros'] = []
            
            for aruco_id in self.arucos_medio:
                self.arucos_medio[aruco_id] = []
            
            # ArUco detection
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            
            # Optimize detection parameters
            parameters.adaptiveThreshWinSizeMin = 3
            parameters.adaptiveThreshWinSizeMax = 23
            parameters.adaptiveThreshWinSizeStep = 10
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            if ids is not None:
                for i, id_ in enumerate(ids.flatten()):
                    c = corners[i]
                    centro = tuple(np.mean(c.reshape(4, 2), axis=0).astype(int))
                    
                    # Zone ArUcos (9-16 correspond to zones 1-8)
                    # Zones 1-6: Can storage zones (with cans)
                    # Zones 7-8: Empty destination zones (where robot leaves cans)
                    if 9 <= id_ <= 16:
                        zona = id_ - 8  # ID 9→Zone 1, ID 10→Zone 2, ..., ID 16→Zone 8
                        aruco_info = {
                            'aruco_id': id_,
                            'centro': centro,
                            'corners': c.reshape(4, 2).tolist()
                        }
                        
                        self.info_zonas[zona]['arucos'].append(aruco_info)
                        self.info_zonas[zona]['centros'].append(centro)
                    
                    # Reference ArUcos for homography (20-23)
                    elif 20 <= id_ <= 23:
                        self.arucos_medio[id_].append({
                            'corners': corners[i],
                            'center': centro
                        })
                    
                    # Robot ArUco (ID 15 conflicts with zone mapping!)
                    # Need to change robot ID to avoid conflict with zone 7 (ID 15)
                    elif id_ == self.robot_id:
                        # Find which zone the robot is in and add it there
                        # This is a simplified assignment - in practice, you'd determine
                        # the actual zone based on robot position within zone boundaries
                        for zona_id in range(1, 9):
                            aruco_info = {
                                'aruco_id': id_,
                                'centro': centro,
                                'corners': c.reshape(4, 2).tolist()
                            }
                            # For now, add to zone 1 (this should be improved with proper zone detection)
                            self.info_zonas[1]['arucos'].append(aruco_info)
                            self.info_zonas[1]['centros'].append(centro)
                            break
                            
            return len(ids) if ids is not None else 0
            
        except Exception as e:
            rospy.logerr("Error in detect_arucos: %s", str(e))
            return 0

    def calculate_homography(self, frame):
        """
        Calculates homography matrix using reference ArUcos
        """
        try:
            # Check if we have all 4 reference ArUcos
            available_arucos = sum(1 for aruco_id in range(20, 24) 
                                 if self.arucos_medio[aruco_id])
            
            if available_arucos < 4:
                rospy.logdebug("Need 4 reference ArUcos (20-23). Only have %d", available_arucos)
                return None
                
            # Extract centers of the 4 reference ArUcos
            pts = np.zeros((4, 2), dtype=np.float32)
            for i, aruco_id in enumerate(range(20, 24)):
                if not self.arucos_medio[aruco_id]:
                    return None
                    
                aruco_data = self.arucos_medio[aruco_id][-1]
                center = aruco_data['center']
                pts[i] = [center[0], center[1]]
                
                # Draw reference point for debugging
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, "Ref {}".format(aruco_id), 
                           (center[0], center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Calculate homography
            H, status = cv2.findHomography(pts, self.world_pts, cv2.RANSAC, 5.0)
            
            if H is not None and status.sum() >= 3:
                self.current_homography = H
                return H
            else:
                rospy.logwarn("Could not calculate good homography")
                return None
                
        except Exception as e:
            rospy.logerr("Error calculating homography: %s", str(e))
            return None

    def draw_zone_info(self, frame, zona):
        """
        Draws zone information on the frame
        """
        try:
            if zona not in self.info_zonas:
                return frame
                
            zona_info = self.info_zonas[zona]
            
            # Draw ArUco centers
            for centro in zona_info['centros']:
                cv2.circle(frame, centro, 4, (0, 255, 0), -1)
            
            # Draw ArUco labels
            for aruco in zona_info['arucos']:
                center = aruco['centro']
                label = "ID:{} Z:{}".format(aruco['aruco_id'], zona)
                cv2.putText(frame, label, (center[0] + 10, center[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            return frame
            
        except Exception as e:
            rospy.logerr("Error in draw_zone_info: %s", str(e))
            return frame

    def process_robot_message(self, msg, topic_type):
        """
        Processes messages received from robot topics
        """
        try:
            # Parse JSON message
            try:
                data = json.loads(msg.data)
                
                # Map alternative key names
                if 'alm1' in data:
                    data['origen'] = data['alm1']
                if 'alm2' in data:
                    data['destino'] = data['alm2']
                    
            except ValueError:
                # Parse key:value format
                data = {}
                for item in msg.data.split(','):
                    if ':' in item:
                        key, value = item.split(':', 1)
                        data[key.strip()] = value.strip()
            
            rospy.loginfo("Received %s message: %s", topic_type, data)
            
            # Update robot state
            with self.processing_lock:
                self.robot_state['modo'] = topic_type
                
                if topic_type == "mover_lata":
                    if 'lata' in data:
                        self.robot_state['lata_actual'] = int(data['lata'])
                    if 'origen' in data:
                        self.robot_state['almacen_origen'] = int(data['origen'])
                    if 'destino' in data:
                        self.robot_state['almacen_destino'] = int(data['destino'])
                        
                elif topic_type == "mover_a_lata":
                    if 'origen' in data:
                        self.robot_state['almacen_origen'] = int(data['origen'])
                    if 'destino' in data:
                        self.robot_state['almacen_destino'] = int(data['destino'])
                
                self.robot_state['navegando'] = True
            
            # Calculate and execute navigation
            self.execute_navigation()
            
        except Exception as e:
            rospy.logerr("Error processing robot message %s: %s", topic_type, str(e))

    def execute_navigation(self):
        """
        Calculates and executes navigation commands
        """
        try:
            with self.processing_lock:
                if not self.robot_state['navegando']:
                    return
                    
                origen_id = self.robot_state['almacen_origen']
                destino_id = self.robot_state['almacen_destino']
                
                if origen_id is None or destino_id is None:
                    rospy.logwarn("Origin or destination not defined")
                    return
                
                # Get robot position
                pos_robot = self.get_robot_position()
                if not pos_robot:
                    rospy.logwarn("Cannot locate robot")
                    action = "Parar"
                else:
                    # Get zone positions
                    pos_origen = self.get_zone_position(origen_id)
                    pos_destino = self.get_zone_position(destino_id)
                    
                    if not pos_origen or not pos_destino:
                        rospy.logwarn("Cannot get zone positions")
                        action = "Parar"
                    else:
                        # Calculate navigation direction
                        action = self.calculate_robot_direction(pos_robot, pos_origen, pos_destino)
                
                # Check gripper control based on distance to target
                self.check_gripper_control()
                
                # Update robot state
                self.robot_state['accion'] = action
                
                # Publish action
                message = "accion:{}".format(action)
                if self.robot_state['modo'] == "mover_lata":
                    self.mover_lata_pub.publish(String(data=message))
                elif self.robot_state['modo'] == "mover_a_lata":
                    self.mover_a_lata_pub.publish(String(data=message))
                
                rospy.loginfo("Published action: %s", action)
                
                # Check if navigation is complete
                if action == "Destino":
                    self.robot_state['navegando'] = False
                    # Reset gripper states for next task
                    self.robot_state['approaching_can'] = False
                    self.robot_state['can_picked'] = False
                    
                    completion_msg = "Destino"
                    if self.robot_state['modo'] == "mover_lata":
                        self.mover_lata_pub.publish(String(data=completion_msg))
                    else:
                        self.mover_a_lata_pub.publish(String(data=completion_msg))
                    rospy.loginfo("Navigation completed: %s", self.robot_state['modo'])
                    
        except Exception as e:
            rospy.logerr("Error in navigation execution: %s", str(e))

    def get_zone_position(self, zona_id):
        """
        Gets the center position of a specific zone
        """
        try:
            if zona_id not in self.info_zonas:
                return None
                
            zona = self.info_zonas[zona_id]
            centros = zona['centros']
            
            if not centros:
                return None
                
            # Calculate average center
            x_avg = sum(float(c[0]) for c in centros) / len(centros)
            y_avg = sum(float(c[1]) for c in centros) / len(centros)
            
            return (int(x_avg), int(y_avg))
            
        except Exception as e:
            rospy.logerr("Error getting zone %d position: %s", zona_id, str(e))
            return None

    def get_robot_position(self):
        """
        Gets current robot position and orientation based on robot ArUco detection
        """
        try:
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    if aruco['aruco_id'] == self.robot_id:
                        position = aruco['centro']
                        
                        # Calculate orientation from ArUco corners
                        if 'corners' in aruco:
                            orientation_data = self.calculate_robot_orientation(aruco['corners'])
                            
                            # Update robot state with position and orientation
                            with self.processing_lock:
                                self.robot_state['position'] = position
                                if orientation_data:
                                    self.robot_state['orientation'] = orientation_data['angle']
                                    self.robot_state['orientation_vector'] = orientation_data['vector']
                                    self.robot_state['last_valid_orientation'] = orientation_data['angle']
                        
                        return position
            return None
            
        except Exception as e:
            rospy.logerr("Error getting robot position: %s", str(e))
            return None

    def calculate_robot_orientation(self, corners):
        """
        Calculates robot orientation from ArUco marker corners
        
        Args:
            corners: List of 4 corner points [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
                    ArUco corners are ordered: top-left, top-right, bottom-right, bottom-left
        
        Returns:
            dict: {
                'angle': float,      # Orientation in degrees (0-360)
                'vector': tuple,     # (dx, dy) unit vector of facing direction
                'confidence': float  # Confidence in the measurement (0-1)
            }
        """
        try:
            if not corners or len(corners) != 4:
                return None
                
            # Convert to numpy array for easier manipulation
            corners_array = np.array(corners, dtype=np.float32)
            
            # Calculate the center of the marker
            center = np.mean(corners_array, axis=0)
            
            # Method 1: Use the direction from center to top edge center
            # Top edge is between corners 0 and 1 (top-left to top-right)
            top_left = corners_array[0]
            top_right = corners_array[1]
            top_center = (top_left + top_right) / 2.0
            
            # Vector from marker center to top center (this is the "forward" direction)
            forward_vector = top_center - center
            
            # Calculate angle in degrees (0 = pointing right, 90 = pointing up)
            angle_rad = np.arctan2(forward_vector[1], forward_vector[0])
            angle_deg = np.degrees(angle_rad)
            
            # Normalize to 0-360 degrees
            if angle_deg < 0:
                angle_deg += 360
                
            # Create unit vector
            magnitude = np.linalg.norm(forward_vector)
            if magnitude > 0:
                unit_vector = forward_vector / magnitude
            else:
                unit_vector = np.array([1.0, 0.0])  # Default to pointing right
            
            # Calculate confidence based on marker size and shape regularity
            confidence = self.calculate_orientation_confidence(corners_array)
            
            # Apply smoothing if we have a previous orientation
            if self.robot_state['orientation'] is not None:
                prev_angle = self.robot_state['orientation']
                angle_diff = abs(angle_deg - prev_angle)
                
                # Handle wrap-around (e.g., 359° to 1°)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                
                # Only update if change is significant or confidence is high
                if angle_diff < self.min_orientation_change and confidence < 0.8:
                    # Use smoothed value
                    smoothing = self.orientation_smoothing
                    angle_deg = prev_angle * smoothing + angle_deg * (1 - smoothing)
            
            return {
                'angle': float(angle_deg),
                'vector': (float(unit_vector[0]), float(unit_vector[1])),
                'confidence': float(confidence)
            }
            
        except Exception as e:
            rospy.logerr("Error calculating robot orientation: %s", str(e))
            return None

    def calculate_orientation_confidence(self, corners):
        """
        Calculates confidence in orientation measurement based on marker quality
        
        Args:
            corners: numpy array of 4 corner points
            
        Returns:
            float: Confidence value between 0 and 1
        """
        try:
            # Calculate side lengths
            side_lengths = []
            for i in range(4):
                p1 = corners[i]
                p2 = corners[(i + 1) % 4]
                length = np.linalg.norm(p2 - p1)
                side_lengths.append(length)
            
            # Check if marker is roughly square (good for orientation)
            mean_length = np.mean(side_lengths)
            if mean_length == 0:
                return 0.0
                
            length_variance = np.var(side_lengths) / (mean_length ** 2)
            
            # Calculate area (larger markers are more reliable)
            area = cv2.contourArea(corners)
            
            # Confidence based on:
            # 1. How square the marker is (low variance in side lengths)
            # 2. Size of the marker (larger = more reliable)
            # 3. Minimum area threshold
            
            square_confidence = max(0, 1.0 - length_variance * 10)  # Penalize non-square shapes
            size_confidence = min(1.0, area / 1000.0)  # Normalize area to 0-1
            
            overall_confidence = (square_confidence * 0.7 + size_confidence * 0.3)
            
            return max(0.0, min(1.0, overall_confidence))
            
        except Exception as e:
            rospy.logerr("Error calculating orientation confidence: %s", str(e))
            return 0.0

    def calculate_distance(self, point1, point2):
        """
        Calculates Euclidean distance between two points
        """
        return np.sqrt(float(point2[0] - point1[0])**2 + float(point2[1] - point1[1])**2)

    def control_gripper(self, action):
        """
        Controls the robot gripper
        
        Args:
            action: "open" or "close"
        """
        try:
            if action not in ["open", "close"]:
                rospy.logwarn("Invalid gripper action: %s", action)
                return
                
            # Only send command if state is changing
            if self.robot_state['gripper_state'] != action:
                message = "gripper:{}".format(action)
                self.gripper_pub.publish(String(data=message))
                self.robot_state['gripper_state'] = action
                rospy.loginfo("Gripper command sent: %s", action)
                
        except Exception as e:
            rospy.logerr("Error controlling gripper: %s", str(e))

    def get_target_can_position(self):
        """
        Gets the position of the target can based on current task
        
        Returns:
            tuple: (x, y) position of target can, or None if not found
        """
        try:
            if self.robot_state['modo'] == "mover_lata":
                # Look for the specific can ID in the origin zone
                lata_id = self.robot_state['lata_actual']
                origen_id = self.robot_state['almacen_origen']
                
                if origen_id in self.info_zonas:
                    for aruco in self.info_zonas[origen_id]['arucos']:
                        if aruco['aruco_id'] == lata_id:
                            return aruco['centro']
                            
            elif self.robot_state['modo'] == "mover_a_lata":
                # For "mover_a_lata", navigate to the destination zone center
                # Zones 7-8 are destination storage zones where cans accumulate
                destino_id = self.robot_state['almacen_destino']
                
                if destino_id in self.info_zonas:
                    # Return the zone center position for navigation
                    # This allows the robot to navigate to the zone for dropping cans
                    return self.get_zone_position(destino_id)
                            
            return None
            
        except Exception as e:
            rospy.logerr("Error getting target can position: %s", str(e))
            return None

    def check_gripper_control(self):
        """
        Checks if gripper should be opened or closed based on distance to target can
        """
        try:
            if not self.robot_state['navegando']:
                return
                
            robot_pos = self.get_robot_position()
            target_can_pos = self.get_target_can_position()
            
            if not robot_pos or not target_can_pos:
                return
                
            distance_to_can = self.calculate_distance(robot_pos, target_can_pos)
            
            # Logic for gripper control based on task mode
            if self.robot_state['modo'] == "mover_lata":
                # Moving a can: open gripper when approaching, close when close enough
                if not self.robot_state['can_picked']:
                    if distance_to_can <= self.gripper_open_threshold and self.robot_state['gripper_state'] == 'closed':
                        self.control_gripper("open")
                        self.robot_state['approaching_can'] = True
                        rospy.loginfo("Approaching can, opening gripper. Distance: %.1f", distance_to_can)
                        
                    elif distance_to_can <= self.gripper_close_threshold and self.robot_state['gripper_state'] == 'open':
                        self.control_gripper("close")
                        self.robot_state['can_picked'] = True
                        self.robot_state['approaching_can'] = False
                        rospy.loginfo("Can reached, closing gripper. Distance: %.1f", distance_to_can)
                        
            elif self.robot_state['modo'] == "mover_a_lata":
                # Moving to a destination zone: prepare gripper for can placement
                # Since zones 7-8 are storage zones, robot should arrive with gripper closed (holding can)
                # and open gripper to drop the can when close to zone center
                if distance_to_can <= self.gripper_close_threshold and self.robot_state['gripper_state'] == 'closed':
                    # Robot has arrived at destination zone with can - open gripper to drop it
                    self.control_gripper("open")
                    rospy.loginfo("Arrived at destination zone, opening gripper to drop can. Distance: %.1f", distance_to_can)
                    
        except Exception as e:
            rospy.logerr("Error in gripper control check: %s", str(e))

    def calculate_robot_direction(self, pos_actual, pos_origen, pos_destino):
        """
        Calculates the direction the robot should take using ACTUAL robot orientation
        Enhanced with proper orientation-based navigation
        """
        try:
            # Calculate distances
            dist_to_origin = self.calculate_distance(pos_actual, pos_origen)
            dist_to_destination = self.calculate_distance(pos_actual, pos_destino)
            
            # Check if we're close to destination first
            if dist_to_destination < self.destination_threshold:
                # Handle gripper control at destination
                if self.robot_state['modo'] == "mover_lata" and self.robot_state['can_picked']:
                    # Drop the can - open gripper
                    self.control_gripper("open")
                    self.robot_state['can_picked'] = False
                    rospy.loginfo("Destination reached, dropping can")
                return "Destino"
            
            # Get current robot orientation
            current_orientation = self.robot_state.get('orientation')
            orientation_vector = self.robot_state.get('orientation_vector')
            
            if current_orientation is None or orientation_vector is None:
                # Fallback to old method if orientation is not available
                rospy.logwarn("Robot orientation not available, using fallback navigation")
                return self.calculate_robot_direction_fallback(pos_actual, pos_origen, pos_destino)
            
            # Calculate desired direction vector (from current position to destination)
            desired_vector = (pos_destino[0] - pos_actual[0], pos_destino[1] - pos_actual[1])
            desired_magnitude = np.sqrt(float(desired_vector[0])**2 + float(desired_vector[1])**2)
            
            if desired_magnitude == 0:
                return "Parar"
            
            # Normalize desired direction vector
            desired_unit_vector = (desired_vector[0] / desired_magnitude, desired_vector[1] / desired_magnitude)
            
            # Calculate desired angle
            desired_angle = np.degrees(np.arctan2(desired_vector[1], desired_vector[0]))
            if desired_angle < 0:
                desired_angle += 360
            
            # Calculate angle difference between current orientation and desired direction
            angle_diff = desired_angle - current_orientation
            
            # Normalize angle difference to [-180, 180]
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            # Log orientation information for debugging
            rospy.logdebug("Robot orientation: %.1f°, Desired: %.1f°, Diff: %.1f°", 
                          current_orientation, desired_angle, angle_diff)
            
            # Decision logic based on angle difference
            angle_tolerance = 15.0  # Degrees of tolerance for "straight" movement
            large_angle_threshold = 120.0  # Threshold for backward movement
            
            if abs(angle_diff) <= angle_tolerance:
                # Robot is facing roughly the right direction - go straight
                return "Recto"
                
            elif abs(angle_diff) >= large_angle_threshold:
                # Large angle difference - it might be easier to go backward
                # Check if going backward would be more efficient
                backward_angle_diff = angle_diff + 180 if angle_diff < 0 else angle_diff - 180
                
                if abs(backward_angle_diff) < abs(angle_diff):
                    rospy.loginfo("Large angle difference (%.1f°), moving backward", angle_diff)
                    return "Atras"
                else:
                    # Still need to turn, decide direction
                    return "GirarD" if angle_diff > 0 else "GirarI"
                    
            elif angle_diff > angle_tolerance:
                # Need to turn left (counter-clockwise)
                return "GirarI"
                
            elif angle_diff < -angle_tolerance:
                # Need to turn right (clockwise)
                return "GirarD"
            
            else:
                # Should not reach here, but safety fallback
                return "Parar"
                
        except Exception as e:
            rospy.logerr("Error calculating robot direction: %s", str(e))
            return "Parar"

    def calculate_robot_direction_fallback(self, pos_actual, pos_origen, pos_destino):
        """
        Fallback navigation method when orientation is not available
        Uses the old position-based approach
        """
        try:
            rospy.logwarn("Using fallback navigation - orientation not available")
            
            # Calculate distances
            dist_to_origin = self.calculate_distance(pos_actual, pos_origen)
            dist_to_destination = self.calculate_distance(pos_actual, pos_destino)
            
            # Calculate direction vectors
            vector_to_destination = (pos_destino[0] - pos_actual[0], pos_destino[1] - pos_actual[1])
            vector_from_origin = (pos_actual[0] - pos_origen[0], pos_actual[1] - pos_origen[1])
            
            # Calculate magnitude of vector to destination
            mag_to_dest = np.sqrt(float(vector_to_destination[0])**2 + float(vector_to_destination[1])**2)
            
            # Avoid division by zero
            if mag_to_dest == 0:
                return "Parar"
            
            # Special case: if robot is very close to origin and needs to move away first
            origin_proximity_threshold = 40
            if dist_to_origin < origin_proximity_threshold:
                # Calculate if we need to move backward from origin
                dot_origin_dest = (vector_from_origin[0] * vector_to_destination[0] + 
                                  vector_from_origin[1] * vector_to_destination[1])
                
                # If destination is roughly in the opposite direction from where we came from origin
                if dot_origin_dest < 0:
                    rospy.loginfo("Moving backward from origin (fallback)")
                    return "Atras"
            
            # Normal navigation logic
            # Calculate current heading (assume robot is facing away from origin)
            mag_from_origin = np.sqrt(float(vector_from_origin[0])**2 + float(vector_from_origin[1])**2)
            
            if mag_from_origin > 0:
                # Calculate angle between current heading and desired direction
                dot_product = (vector_from_origin[0] * vector_to_destination[0] + 
                              vector_from_origin[1] * vector_to_destination[1])
                cos_angle = float(dot_product) / (mag_from_origin * mag_to_dest)
                cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp to avoid numerical errors
                
                angle_rad = np.arccos(cos_angle)
                angle_degrees = np.degrees(angle_rad)
                
                # Calculate cross product to determine turn direction
                cross_product = (vector_from_origin[0] * vector_to_destination[1] - 
                               vector_from_origin[1] * vector_to_destination[0])
                
                # Decision logic with improved thresholds
                if angle_degrees < 15:  # Small angle threshold for going straight
                    return "Recto"
                elif angle_degrees > 150:  # Large angle - need to go backward
                    return "Atras"
                elif cross_product > 0:
                    return "GirarD"  # Turn right
                else:
                    return "GirarI"  # Turn left
            else:
                # Robot is at origin, decide initial direction
                if abs(vector_to_destination[0]) > abs(vector_to_destination[1]):
                    return "GirarD" if vector_to_destination[0] > 0 else "GirarI"
                else:
                    return "Recto" if vector_to_destination[1] > 0 else "Atras"
                
        except Exception as e:
            rospy.logerr("Error in fallback direction calculation: %s", str(e))
            return "Parar"

    def save_calibration(self):
        """
        Saves calibration parameters to file
        """
        try:
            np.savez('camera_calibration.npz', 
                    camera_matrix=self.camera_matrix, 
                    dist_coeffs=self.dist_coeffs,
                    frame_size=np.array(self.frame_size))
            
            rospy.loginfo("Calibration parameters saved to 'camera_calibration.npz'")
            return True
            
        except Exception as e:
            rospy.logerr("Error saving calibration: %s", str(e))
            return False

    def load_calibration(self):
        """
        Loads calibration parameters from file
        """
        try:
            if os.path.exists('camera_calibration.npz'):
                data = np.load('camera_calibration.npz')
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                saved_size = data['frame_size']
                
                if tuple(saved_size) == self.frame_size:
                    self.calibration_done = True
                    rospy.loginfo("Calibration parameters loaded")
                    return True
                else:
                    rospy.logwarn("Frame size mismatch in calibration")
            return False
            
        except Exception as e:
            rospy.logerr("Error loading calibration: %s", str(e))
            return False

    def image_callback(self, msg):
        """
        Main image processing callback
        """
        try:
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocess frame
            preprocessed = self.preprocess_frame(frame)
            if preprocessed is None:
                return
            
            # Detect ArUcos
            num_detected = self.detect_arucos(preprocessed)
            
            # Calculate homography and create bird's eye view
            H = self.calculate_homography(frame)
            warped_frame = None
            
            if H is not None:
                warped_frame = cv2.warpPerspective(frame, H, (1000, 1000))
                
                # Draw zone information on warped frame
                for zona in range(1, 9):
                    warped_frame = self.draw_zone_info(warped_frame, zona)
                
                # Draw navigation vector if navigating
                if self.robot_state['navegando']:
                    self.draw_navigation_vector(frame, warped_frame, H)
                
                cv2.imshow("Vista Cenital", warped_frame)
            
            # Draw zone information on original frame
            frame_viz = frame.copy()
            for zona in range(1, 9):
                frame_viz = self.draw_zone_info(frame_viz, zona)
            
            # Add status information
            status_text = "Mode: {} | Action: {} | Detected: {}".format(
                self.robot_state.get('modo', 'None'),
                self.robot_state.get('accion', 'None'),
                num_detected
            )
            cv2.putText(frame_viz, status_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add gripper status
            gripper_text = "Gripper: {} | Can: {}".format(
                self.robot_state.get('gripper_state', 'unknown'),
                'Picked' if self.robot_state.get('can_picked', False) else 'Free'
            )
            cv2.putText(frame_viz, gripper_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Add orientation information
            orientation = self.robot_state.get('orientation')
            if orientation is not None:
                orientation_text = "Orientation: {:.1f}°".format(orientation)
                cv2.putText(frame_viz, orientation_text, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw orientation arrow on robot if position is known
                robot_pos = self.robot_state.get('position')
                orientation_vector = self.robot_state.get('orientation_vector')
                if robot_pos and orientation_vector:
                    # Calculate end point for orientation arrow
                    arrow_length = 50
                    end_x = int(robot_pos[0] + orientation_vector[0] * arrow_length)
                    end_y = int(robot_pos[1] + orientation_vector[1] * arrow_length)
                    
                    # Draw orientation arrow
                    cv2.arrowedLine(frame_viz, robot_pos, (end_x, end_y), (0, 255, 0), 3)
                    cv2.putText(frame_viz, "Robot Heading", 
                               (robot_pos[0] + 10, robot_pos[1] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                orientation_text = "Orientation: Unknown"
                cv2.putText(frame_viz, orientation_text, (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("ArUco Detection", frame_viz)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr("Error in image callback: %s", str(e))

    def draw_navigation_vector(self, frame, warped_frame, H):
        """
        Draws navigation vectors on both original and warped frames
        """
        try:
            pos_robot = self.get_robot_position()
            if not pos_robot:
                return
                
            destino_id = self.robot_state['almacen_destino']
            pos_destino = self.get_zone_position(destino_id)
            
            if not pos_destino:
                return
            
            # Draw arrow on original frame
            cv2.arrowedLine(frame, pos_robot, pos_destino, (0, 0, 255), 2)
            
            # Transform positions to warped space and draw arrow
            robot_pts = np.array([[pos_robot]], dtype=np.float32)
            destino_pts = np.array([[pos_destino]], dtype=np.float32)
            
            warped_robot = cv2.perspectiveTransform(robot_pts, H)[0][0]
            warped_destino = cv2.perspectiveTransform(destino_pts, H)[0][0]
            
            warped_robot = (int(warped_robot[0]), int(warped_robot[1]))
            warped_destino = (int(warped_destino[0]), int(warped_destino[1]))
            
            cv2.arrowedLine(warped_frame, warped_robot, warped_destino, (0, 0, 255), 2)
            
        except Exception as e:
            rospy.logerr("Error drawing navigation vector: %s", str(e))

    def run(self):
        """
        Main execution loop
        """
        rospy.loginfo("Starting robot navigation system...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down robot navigation system")
        finally:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        navigator = Navegacion_Robot()
        navigator.run()
    except rospy.ROSInterruptException:
        pass 