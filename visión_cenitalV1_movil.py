#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
import json
import threading
import time
import argparse

# ROS imports (optional - will work without ROS)
try:
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Bool
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError:
    print("ROS not available - running in standalone mode")
    ROS_AVAILABLE = False

class Navegacion_Robot:
    def __init__(self, camera_source="usb", mobile_ip="192.168.71.56:8080", video_file=None, use_ros=False, autonomous=True):
        """
        Initialize the robot navigation system
        
        Args:
            camera_source: "mobile", "usb", "file", or "ros"
            mobile_ip: IP address for mobile camera (required for mobile camera)
            video_file: Path to video file (required for file camera)
            use_ros: Whether to use ROS functionality
            autonomous: Run in autonomous mode (no interactive prompts)
        """
        self.camera_source = camera_source
        self.mobile_ip = mobile_ip
        self.video_file = video_file
        self.use_ros = use_ros and ROS_AVAILABLE
        self.autonomous = autonomous
        
        # Initialize ROS if available and requested
        rospy.init_node("nada", anonymous=True)
        self.bridge = CvBridge()
        
        # Camera capture object
        self.cap = None
        self.camera_active = False
        
        # System parameters
        self.frame_size = (640, 480)
        
        # Debug mode
        self.debug_mode = False
        self.last_preprocessed = None
        self.last_detections = {}
        
        # Interactive parameter adjustment
        self.interactive_mode = False
        self.show_parameter_menu = False
        
        # Image enhancement parameters (adjustable in real-time)
        self.enhancement_params = {
            'brightness': 0,        # -100 to 100
            'contrast': 1.0,        # 0.5 to 3.0
            'saturation': 1.0,      # 0.0 to 2.0
            'gamma': 1.0,           # 0.5 to 2.0
            'blur_kernel': 3,       # 1, 3, 5, 7
            'bilateral_d': 9,       # 5, 9, 15
            'bilateral_sigma': 75,  # 25, 50, 75, 100
            'morph_kernel': 3,      # 1, 3, 5, 7
            'clahe_limit': 2.0,     # 1.0 to 8.0
            'clahe_grid': 8,        # 4, 8, 16
        }
        
        # ArUco detection parameters (adjustable in real-time)
        self.aruco_params = {
            'adaptiveThreshConstant': 7,           # 3 to 15
            'adaptiveThreshWinSizeMin': 3,         # 3 to 10
            'adaptiveThreshWinSizeMax': 23,        # 15 to 50
            'adaptiveThreshWinSizeStep': 10,       # 5 to 15
            'minMarkerPerimeterRate': 0.03,        # 0.01 to 0.1
            'maxMarkerPerimeterRate': 4.0,         # 2.0 to 8.0
            'polygonalApproxAccuracyRate': 0.05,   # 0.01 to 0.1
            'minCornerDistanceRate': 0.05,         # 0.01 to 0.1
            'minDistanceToBorder': 3,              # 1 to 10
            'cornerRefinementWinSize': 5,          # 3 to 15
            'cornerRefinementMaxIterations': 30,   # 10 to 100
            'cornerRefinementMinAccuracy': 0.1,    # 0.01 to 1.0
        }
        
        # Debugging parameters
        self.debug_params = {
            'show_preprocessing': True,
            'show_detection_steps': True,
            'show_parameter_effects': True,
            'show_timing_info': True,
            'show_rejection_analysis': True,
            'verbose_logging': True,
            'save_debug_frames': False,
            'frame_counter': 0
        }
        
        # Performance tracking
        self.performance_stats = {
            'total_frames': 0,
            'successful_detections': 0,
            'detection_times': [],
            'preprocessing_times': [],
            'best_methods': {},
            'parameter_effectiveness': {},
            'calibration_count': 0
        }
        
        # Parameter calibration timing
        self.calibration_interval = 30.0  # Reduced frequency: Calibrate every 30 seconds
        self.last_calibration_time = 0
        self.calibration_running = False
        
        # Performance optimization settings
        self.performance_mode = True  # Enable aggressive optimizations
        self.skip_preprocessing = True  # Skip heavy preprocessing when possible
        self.fast_detection_only = True  # Use only fast detection method
        self.reduce_logging = True  # Minimize logging overhead
        self.frame_skip_count = 0  # Skip frames for better performance
        self.frame_skip_interval = 1  # Process every N frames (1 = all frames, 2 = every other frame)
        
        # Best parameters found during calibration
        self.best_detection_params = {
            'preprocessing_method': 'enhanced',  # Default to enhanced
            'aruco_params': None,  # Will be set to default params
            'detection_score': 0,  # Number of successful detections with these params
            'last_update': 0
        }
        
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
        # These should correspond to a reasonable bird's eye view rectangle
        # Order: [bottom-left, top-left, bottom-right, top-right] for ArUco IDs [20, 21, 22, 23]
        self.world_pts = np.array([
            [  0,   0],   # ArUco ID 20: Bottom left
            [  0, 800],   # ArUco ID 21: Top left  
            [800,   0],   # ArUco ID 22: Bottom right
            [800, 800],   # ArUco ID 23: Top right
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
        
        # Homography caching and timing
        self.homography_auto_recalc = False  # Disabled by default - only manual or on marker movement
        self.last_homography_time = 0
        self.homography_calculating = False
        self.last_reference_positions = {}  # Store last known positions of reference markers
        self.homography_position_threshold = 15.0  # Pixels - if markers move more than this, recalculate
        self.homography_valid = False
        self.homography_quality_score = 0.0
        self.homography_locked = False  # When True, only manual recalculation allowed

        # Initialize ROS topics if ROS is available
        print("🔧 Setting up ROS communication...")
        try:
            # Always set up ROS publishers and subscribers since ROS is initialized
            print("   📡 Creating ROS publishers...")
            self.mover_lata_pub = rospy.Publisher("/info_camara_mover_lata", String, queue_size=1)
            self.mover_a_lata_pub = rospy.Publisher("/info_camara_mover_a_lata", String, queue_size=1)
            self.gripper_pub = rospy.Publisher("/gripper_control", String, queue_size=1)
            
            print("   📡 Creating ROS subscribers...")
            # Subscriptions to robot messages with debug info
            print("      - Subscribing to: /info_robot_mover_lata")
            self.mover_lata_sub = rospy.Subscriber("/info_robot_mover_lata", String, 
                                                lambda msg: self.process_robot_message(msg, "mover_lata"), 
                                                queue_size=1)
            print("      - Subscribing to: /info_robot_mover_a_lata")
            self.mover_a_lata_sub = rospy.Subscriber("/info_robot_mover_a_lata", String, 
                                                    lambda msg: self.process_robot_message(msg, "mover_a_lata"), 
                                                    queue_size=1)
            
            # Camera subscriber only if using ROS camera
            if self.use_ros and self.camera_source == "ros":
                print("      - Subscribing to: /csi_cam_0/image_raw")
                self.sub = rospy.Subscriber("/csi_cam_0/image_raw", Image, self.ros_image_callback, queue_size=1)
            
            print("   ✅ ROS communication setup complete!")
            print("   📻 Waiting for robot messages on:")
            print("      - /info_robot_mover_lata")
            print("      - /info_robot_mover_a_lata")
            
            # Add a small delay to let ROS settle
            time.sleep(1)
            
            # Run diagnostic checks
            self.check_ros_topics()
            
            # Test publisher
            self.test_ros_publisher()
            
        except Exception as e:
            print("   ❌ Error setting up ROS: {}".format(str(e)))
            print("   Make sure ROS is running: roscore")

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
        
        # Load previous homography if available
        self.load_homography()
        
        # Initialize camera
        self.initialize_camera()
        
        # Autonomous mode: minimal startup info
        if self.autonomous:
            print("🤖 Navigation system initialized")
            print("   Camera source: {}".format(camera_source))
            print("   Robot ArUco ID: {}".format(self.robot_id))
            print("   Performance mode: {}".format("ENABLED" if self.performance_mode else "DISABLED"))
            if self.homography_valid:
                print("   Homography: Loaded from file")
        else:
            # Full startup information for non-autonomous mode
            print("Robot Navigation System initialized with orientation tracking")
            print("Camera source: {}".format(camera_source))
            print("Robot ArUco ID set to: {}".format(self.robot_id))
            print("🚀 PERFORMANCE MODE: {} (Press 'f' to toggle)".format("ENABLED" if self.performance_mode else "DISABLED"))
            if self.performance_mode:
                print("   • Ultra-fast processing")
                print("   • Minimal UI and logging")
                print("   • Static homography caching")
                print("   • Reduced calibration frequency")
            else:
                print("🔧 Parameter calibration: Every {:.1f} seconds".format(self.calibration_interval))
            print("🏗️ STATIC HOMOGRAPHY CACHING:")
            print("  • Homography calculated ONCE when markers detected")
            print("  • NO automatic time-based recalculation")
            print("  • Only recalculates if markers move >15px (when enabled)")
            print("  • Can be locked for 100% static operation")
            print("  • Saves/loads from file for persistence")
            print("  • Maximum performance - minimal computation")
            if self.homography_valid:
                print("   ✅ Previous homography loaded from file")

    def initialize_camera(self):
        """
        Initialize camera based on the selected source
        """
        try:
            if self.camera_source == "mobile":
                if not self.mobile_ip:
                    print("ERROR: Mobile IP not provided!")
                    print("Please provide mobile IP address (e.g., '192.168.1.100:8080')")
                    return False
                
                # Try different mobile camera URL formats
                mobile_urls = [
                    "http://{}/video".format(self.mobile_ip),
                    "http://{}/cam.mjpg".format(self.mobile_ip),
                    "http://{}/video.mjpg".format(self.mobile_ip),
                    "http://{}/mjpeg".format(self.mobile_ip),
                ]
                
                for url in mobile_urls:
                    print("Trying mobile camera URL: {}".format(url))
                    self.cap = cv2.VideoCapture(url)
                    if self.cap.isOpened():
                        print("✅ Successfully connected to mobile camera: {}".format(url))
                        self.camera_active = True
                        break
                    else:
                        self.cap.release()
                
                if not self.camera_active:
                    print("❌ Failed to connect to mobile camera")
                    print("Make sure:")
                    print("1. Your phone and computer are on the same WiFi network")
                    print("2. Camera app is running on your phone")
                    print("3. IP address is correct: {}".format(self.mobile_ip))
                    return False
                    
            elif self.camera_source == "usb":
                # Try USB cameras (0, 1, 2...)
                for i in range(3):
                    print("Trying USB camera {}...".format(i))
                    self.cap = cv2.VideoCapture(i)
                    if self.cap.isOpened():
                        print("✅ Successfully connected to USB camera {}".format(i))
                        self.camera_active = True
                        break
                    else:
                        self.cap.release()
                
                if not self.camera_active:
                    print("❌ No USB camera found")
                    return False
                    
            elif self.camera_source == "file":
                if not hasattr(self, 'video_file') or not self.video_file:
                    print("❌ Video file path not provided. Use --video-file argument.")
                    return False
                    
                if os.path.exists(self.video_file):
                    self.cap = cv2.VideoCapture(self.video_file)
                    if self.cap.isOpened():
                        print("✅ Successfully opened video file: {}".format(self.video_file))
                        self.camera_active = True
                    else:
                        print("❌ Failed to open video file")
                        return False
                else:
                    print("❌ Video file not found: {}".format(self.video_file))
                    return False
                    
            elif self.camera_source == "ros":
                if not self.use_ros:
                    print("❌ ROS not available")
                    return False
                print("✅ Using ROS camera topic")
                self.camera_active = True
                
            # Set camera properties for better performance
            if self.cap and self.camera_active and self.camera_source != "ros":
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_size[0])
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_size[1])
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
            return self.camera_active
            
        except Exception as e:
            print("Error initializing camera: {}".format(str(e)))
            return False

    def get_frame(self):
        """
        Get a frame from the camera source
        """
        try:
            if not self.camera_active:
                return None
                
            if self.camera_source == "ros":
                # For ROS, frames are handled in ros_image_callback
                return None
                
            if self.cap is None:
                return None
                
            ret, frame = self.cap.read()
            if ret:
                return frame
            else:
                print("Failed to read frame from camera")
                return None
                
        except Exception as e:
            print("Error getting frame: {}".format(str(e)))
            return None

    def ros_image_callback(self, msg):
        """
        ROS image callback (only used when camera_source is "ros")
        """
        try:
            if not self.use_ros:
                return
                
            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame)
            
        except Exception as e:
            if self.use_ros:
                rospy.logerr("Error in ROS image callback: %s", str(e))
            else:
                print("Error in ROS image callback: {}".format(str(e)))

    def preprocess_frame(self, frame):
        """
        Ultra-fast preprocessing optimized for performance
        """
        try:
            if frame is None:
                return None
                
            # Frame skipping for performance
            if self.performance_mode and self.frame_skip_interval > 1:
                self.frame_skip_count += 1
                if self.frame_skip_count % self.frame_skip_interval != 0:
                    return None  # Skip this frame
            
            # Quick resize if necessary
            if frame.shape[:2] != (self.frame_size[1], self.frame_size[0]):
                frame = cv2.resize(frame, self.frame_size)
            
            # Convert to grayscale immediately
            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame.copy()
            
            # Performance mode: minimal processing
            if self.performance_mode and self.skip_preprocessing:
                # Return only the essential grayscale frame
                return {'enhanced': gray}
            
            # Standard mode: apply only CLAHE enhancement (fastest effective method)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            
            return {'enhanced': enhanced}
            
        except Exception as e:
            if not self.reduce_logging:
                self.log_error("Error in preprocess_frame: %s", str(e))
            return None

    def detect_arucos(self, frame):
        """
        Ultra-fast ArUco detection optimized for performance
        """
        try:
            # Clear previous detections
            for zona_id in self.info_zonas:
                self.info_zonas[zona_id]['arucos'] = []
                self.info_zonas[zona_id]['centros'] = []
            
            for aruco_id in self.arucos_medio:
                self.arucos_medio[aruco_id] = []
            
            # Get preprocessed frames
            preprocessed = self.preprocess_frame(frame)
            if preprocessed is None:
                return 0
            
            # Performance mode: skip expensive calibration
            if self.performance_mode and self.fast_detection_only:
                return self.fast_detect_with_best_params(preprocessed)
            
            # Check if it's time to run parameter calibration (reduced frequency)
            current_time = time.time()
            should_calibrate = (current_time - self.last_calibration_time) >= self.calibration_interval
            
            if should_calibrate and not self.performance_mode:
                if not self.reduce_logging:
                    self.log_info("🔧 Running parameter calibration (every %.1f seconds)", self.calibration_interval)
                self.run_parameter_calibration(preprocessed)
                self.last_calibration_time = current_time
                self.performance_stats['calibration_count'] += 1
            
            # Use the best parameters found during calibration for normal detection
            total_processed = self.detect_with_best_params(preprocessed)
            
            return total_processed
            
        except Exception as e:
            if not self.reduce_logging:
                self.log_error("Error in detect_arucos: %s", str(e))
            return 0

    def fast_detect_with_best_params(self, preprocessed):
        """
        Ultra-fast ArUco detection using minimal processing
        """
        try:
            # Use only the enhanced frame
            gray_frame = preprocessed['enhanced']
            
            # ArUco dictionary
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            
            # Use optimized detection parameters
            params = cv2.aruco.DetectorParameters_create()
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 23
            params.adaptiveThreshWinSizeStep = 10
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE  # Skip corner refinement for speed
            
            # Perform detection
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=params)
            
            total_processed = 0
            
            if ids is not None and len(ids) > 0:
                # Process all detections quickly
                for i, id_ in enumerate(ids.flatten()):
                    corners_data = corners[i]
                    centro = tuple(np.mean(corners_data.reshape(4, 2), axis=0).astype(int))
                    
                    # Zone ArUcos (9-16 correspond to zones 1-8)
                    if 9 <= id_ <= 16:
                        zona = id_ - 8
                        aruco_info = {
                            'aruco_id': id_,
                            'centro': centro,
                            'corners': corners_data.reshape(4, 2).tolist(),
                            'detection_method': 'fast'
                        }
                        
                        self.info_zonas[zona]['arucos'].append(aruco_info)
                        self.info_zonas[zona]['centros'].append(centro)
                        total_processed += 1
                    
                    # Reference ArUcos for homography (20-23)
                    elif 20 <= id_ <= 23:
                        self.arucos_medio[id_].append({
                            'corners': corners_data,
                            'center': centro,
                            'detection_method': 'fast'
                        })
                        total_processed += 1
                    
                    # Robot ArUco
                    elif id_ == self.robot_id:
                        aruco_info = {
                            'aruco_id': id_,
                            'centro': centro,
                            'corners': corners_data.reshape(4, 2).tolist(),
                            'detection_method': 'fast'
                        }
                        
                        # Add to zone 1 for compatibility
                        self.info_zonas[1]['arucos'].append(aruco_info)
                        self.info_zonas[1]['centros'].append(centro)
                        total_processed += 1
                    
                    # Can ArUcos (IDs 1-8 for individual cans)
                    elif 1 <= id_ <= 8:
                        zona = ((id_ - 1) % 6) + 1
                        aruco_info = {
                            'aruco_id': id_,
                            'centro': centro,
                            'corners': corners_data.reshape(4, 2).tolist(),
                            'detection_method': 'fast',
                            'type': 'can'
                        }
                        
                        self.info_zonas[zona]['arucos'].append(aruco_info)
                        self.info_zonas[zona]['centros'].append(centro)
                        total_processed += 1
            
            return total_processed
            
        except Exception as e:
            if not self.reduce_logging:
                self.log_error("Error in fast_detect_with_best_params: %s", str(e))
            return 0

    def run_parameter_calibration(self, preprocessed):
        """
        Runs parameter calibration to find the best detection parameters
        This is a more intensive process that runs every few seconds
        """
        try:
            self.calibration_running = True
            start_time = time.time()
            
            # ArUco dictionary
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            
            # Create parameter sets for testing
            parameter_sets = []
            
            # Parameter set 1: Standard detection
            params1 = cv2.aruco.DetectorParameters_create()
            params1.adaptiveThreshWinSizeMin = 3
            params1.adaptiveThreshWinSizeMax = 23
            params1.adaptiveThreshWinSizeStep = 10
            params1.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            params1.cornerRefinementWinSize = 5
            params1.cornerRefinementMaxIterations = 30
            params1.cornerRefinementMinAccuracy = 0.1
            parameter_sets.append(("Standard", params1))
            
            # Parameter set 2: High sensitivity
            params2 = cv2.aruco.DetectorParameters_create()
            params2.adaptiveThreshWinSizeMin = 3
            params2.adaptiveThreshWinSizeMax = 53
            params2.adaptiveThreshWinSizeStep = 4
            params2.adaptiveThreshConstant = 7
            params2.minMarkerPerimeterRate = 0.03
            params2.maxMarkerPerimeterRate = 4.0
            params2.polygonalApproxAccuracyRate = 0.05
            params2.minCornerDistanceRate = 0.05
            params2.minDistanceToBorder = 3
            params2.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            parameter_sets.append(("High Sensitivity", params2))
            
            # Parameter set 3: Low threshold
            params3 = cv2.aruco.DetectorParameters_create()
            params3.adaptiveThreshWinSizeMin = 3
            params3.adaptiveThreshWinSizeMax = 23
            params3.adaptiveThreshWinSizeStep = 10
            params3.adaptiveThreshConstant = 5  # Lower threshold
            params3.minMarkerPerimeterRate = 0.01  # Allow smaller markers
            params3.maxMarkerPerimeterRate = 8.0   # Allow larger markers
            params3.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
            parameter_sets.append(("Low Threshold", params3))
            
            # Test all combinations and track results
            best_combination = None
            best_score = 0
            calibration_results = []
            
            for preprocess_name, gray_frame in preprocessed.items():
                for param_name, params in parameter_sets:
                    try:
                        corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=params)
                        
                        # Calculate score based on number of detections and quality
                        detection_count = len(ids) if ids is not None else 0
                        rejection_count = len(rejected) if rejected is not None else 0
                        
                        # Score factors: more detections is better, fewer rejections is better
                        score = detection_count * 10 - rejection_count * 0.5
                        
                        # Bonus points for detecting important markers (robot, reference)
                        if ids is not None:
                            for id_ in ids.flatten():
                                if id_ == self.robot_id:
                                    score += 20  # Robot detection is very important
                                elif 20 <= id_ <= 23:
                                    score += 15  # Reference markers are important
                                elif 9 <= id_ <= 16:
                                    score += 5   # Zone markers are useful
                        
                        calibration_results.append({
                                'preprocess': preprocess_name,
                                'params': param_name,
                            'score': score,
                            'detections': detection_count,
                            'rejections': rejection_count,
                            'aruco_params': params
                        })
                        
                        if score > best_score:
                            best_score = score
                            best_combination = {
                                'preprocessing_method': preprocess_name,
                                'aruco_params': params,
                                'detection_score': score,
                                'param_name': param_name
                            }
                            
                    except Exception as e:
                        self.log_debug("Calibration attempt failed (%s + %s): %s", preprocess_name, param_name, str(e))
                        continue
            
            # Update best parameters if we found a better combination
            if best_combination and best_score > self.best_detection_params['detection_score']:
                self.best_detection_params.update(best_combination)
                self.best_detection_params['last_update'] = time.time()
                
                self.log_info("📈 Updated best parameters: %s + %s (score: %.1f)", 
                             best_combination['preprocessing_method'], 
                             best_combination['param_name'], 
                             best_score)
            else:
                self.log_info("📊 Keeping current best parameters (score: %.1f vs %.1f)", 
                             self.best_detection_params['detection_score'], best_score)
            
            # Log calibration summary
            calibration_time = time.time() - start_time
            self.log_info("🔧 Calibration completed in %.2f seconds. Tested %d combinations.", 
                         calibration_time, len(calibration_results))
            
            self.calibration_running = False
            
        except Exception as e:
            self.log_error("Error in parameter calibration: %s", str(e))
            self.calibration_running = False

    def detect_with_best_params(self, preprocessed):
        """
        Performs ArUco detection using the best parameters found during calibration
        This is the fast method used for regular frame processing
        """
        try:
            # Get the best preprocessing method and parameters
            best_preprocess = self.best_detection_params['preprocessing_method']
            best_params = self.best_detection_params['aruco_params']
            
            # Use default parameters if none have been calibrated yet
            if best_params is None:
                best_params = cv2.aruco.DetectorParameters_create()
                best_params.adaptiveThreshWinSizeMin = 3
                best_params.adaptiveThreshWinSizeMax = 23
                best_params.adaptiveThreshWinSizeStep = 10
                best_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                self.log_info("Using default parameters (no calibration yet)")
            
            # Get the best preprocessed frame
            if best_preprocess not in preprocessed:
                # Fallback to 'enhanced' or first available
                best_preprocess = 'enhanced' if 'enhanced' in preprocessed else list(preprocessed.keys())[0]
                
            gray_frame = preprocessed[best_preprocess]
            
            # ArUco dictionary
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            
            # Perform detection
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=best_params)
            
            # Process detections
            all_detections = {}
            total_processed = 0
            
            if ids is not None and len(ids) > 0:
                # Store detections
                for i, id_ in enumerate(ids.flatten()):
                    all_detections[id_] = {
                        'corners': corners[i],
                        'method': "{} + best_params".format(best_preprocess)
                    }
                
                self.log_info("Fast detection: %d markers with %s + best_params", 
                             len(ids), best_preprocess)
                
                # Process all detections
            for id_, detection_data in all_detections.items():
                corners_data = detection_data['corners']
                centro = tuple(np.mean(corners_data.reshape(4, 2), axis=0).astype(int))
                
                # Zone ArUcos (9-16 correspond to zones 1-8)
                if 9 <= id_ <= 16:
                    zona = id_ - 8  # ID 9→Zone 1, ID 10→Zone 2, ..., ID 16→Zone 8
                    aruco_info = {
                        'aruco_id': id_,
                        'centro': centro,
                        'corners': corners_data.reshape(4, 2).tolist(),
                        'detection_method': detection_data['method']
                    }
                    
                    self.info_zonas[zona]['arucos'].append(aruco_info)
                    self.info_zonas[zona]['centros'].append(centro)
                    total_processed += 1
                    self.log_debug("Zone ArUco %d detected in zone %d at %s", id_, zona, centro)
                
                # Reference ArUcos for homography (20-23)
                elif 20 <= id_ <= 23:
                    self.arucos_medio[id_].append({
                        'corners': corners_data,
                        'center': centro,
                        'detection_method': detection_data['method']
                    })
                    total_processed += 1
                    self.log_debug("Reference ArUco %d detected at %s", id_, centro)
                
                # Robot ArUco
                elif id_ == self.robot_id:
                    self.log_info("Robot ArUco %d detected at position: %s (method: %s)", 
                                 id_, centro, detection_data['method'])
                    
                    # Store robot position globally (not tied to a specific zone)
                    aruco_info = {
                        'aruco_id': id_,
                        'centro': centro,
                        'corners': corners_data.reshape(4, 2).tolist(),
                        'detection_method': detection_data['method']
                    }
                    
                    # Add to zone 1 for compatibility (this should be improved)
                    self.info_zonas[1]['arucos'].append(aruco_info)
                    self.info_zonas[1]['centros'].append(centro)
                    total_processed += 1
                
                # Can ArUcos (IDs 1-8 for individual cans)
                elif 1 <= id_ <= 8:
                    # These are individual can markers - assign to appropriate zones
                    # For now, assign to zone based on position or predefined mapping
                    zona = ((id_ - 1) % 6) + 1  # Distribute cans across zones 1-6
                    aruco_info = {
                        'aruco_id': id_,
                        'centro': centro,
                        'corners': corners_data.reshape(4, 2).tolist(),
                        'detection_method': detection_data['method'],
                        'type': 'can'
                    }
                    
                    self.info_zonas[zona]['arucos'].append(aruco_info)
                    self.info_zonas[zona]['centros'].append(centro)
                    total_processed += 1
                    self.log_debug("Can ArUco %d detected in zone %d at %s", id_, zona, centro)
                
                else:
                    self.log_debug("Unknown ArUco ID %d detected at %s", id_, centro)
            
            else:
                self.log_warn("No ArUco markers detected with best parameters")
            
            # Store debug data
            self.last_detections = all_detections
            
            return total_processed
            
        except Exception as e:
            self.log_error("Error in detect_with_best_params: %s", str(e))
            return 0

    def calculate_homography(self, frame):
        """
        Calculates homography matrix using reference ArUcos with static caching
        Only recalculates when markers move or manual request - no time-based recalculation
        """
        try:
            current_time = time.time()
            
            # Check if we have all 4 reference ArUcos
            available_arucos = sum(1 for aruco_id in range(20, 24) 
                                 if self.arucos_medio[aruco_id])
            
            if available_arucos < 4:
                # Use log_info with throttling simulation
                if not hasattr(self, '_last_homography_warning') or current_time - self._last_homography_warning > 2:
                    self.log_info("Need 4 reference ArUcos (20-23). Only have %d", available_arucos)
                    self._last_homography_warning = current_time
                self.homography_valid = False
                return self.current_homography  # Return cached version if available
                
            # Get current reference marker positions
            current_positions = {}
            pts = np.zeros((4, 2), dtype=np.float32)
            
            for i, aruco_id in enumerate(range(20, 24)):
                if not self.arucos_medio[aruco_id]:
                    self.homography_valid = False
                    return self.current_homography
                    
                aruco_data = self.arucos_medio[aruco_id][-1]
                center = aruco_data['center']
                current_positions[aruco_id] = center
                pts[i] = [center[0], center[1]]
                
                # Draw reference point for debugging (always draw these)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, "Ref {}".format(aruco_id), 
                           (center[0], center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Check if recalculation is needed
            should_recalculate = False
            recalc_reason = ""
            
            # Reason 1: No valid homography exists
            if not self.homography_valid or self.current_homography is None:
                should_recalculate = True
                recalc_reason = "No valid homography"
                
            # Reason 2: Homography is locked - skip all automatic recalculation
            elif self.homography_locked:
                return self.current_homography
                
            # Reason 3: Reference markers moved significantly (only if auto-recalc enabled)
            elif self.homography_auto_recalc and self.last_reference_positions:
                max_movement = 0
                for aruco_id, new_pos in current_positions.items():
                    if aruco_id in self.last_reference_positions:
                        old_pos = self.last_reference_positions[aruco_id]
                        movement = np.sqrt((new_pos[0] - old_pos[0])**2 + (new_pos[1] - old_pos[1])**2)
                        max_movement = max(max_movement, movement)
                
                if max_movement > self.homography_position_threshold:
                    should_recalculate = True
                    recalc_reason = "Markers moved {:.1f} pixels".format(max_movement)
            
            # If no recalculation needed, return cached homography
            if not should_recalculate and self.homography_valid:
                return self.current_homography
            
            # Perform homography calculation
            self.homography_calculating = True
            self.log_info("🔄 Calculating homography: %s", recalc_reason)
            
            start_time = time.time()
            H, status = cv2.findHomography(pts, self.world_pts, cv2.RANSAC, 5.0)
            calc_time = time.time() - start_time
            
            # Validate homography quality
            if H is not None and status is not None:
                inliers = status.sum()
                total_points = len(status)
                inlier_ratio = float(inliers) / total_points if total_points > 0 else 0
                
                # Calculate quality score based on inliers and geometric consistency
                self.homography_quality_score = inlier_ratio
                
                # Consider homography valid if at least 75% of points are inliers
                if inlier_ratio >= 0.75:
                    self.current_homography = H
                    self.homography_valid = True
                    self.last_homography_time = current_time
                    self.last_reference_positions = current_positions.copy()
                    
                    self.log_info("✅ Homography calculated: {:.1f}% inliers, {:.2f}ms", 
                                 inlier_ratio * 100, calc_time * 1000)
                    self.log_info("🔒 Homography cached - will only recalculate if markers move >{:.1f}px", 
                                 self.homography_position_threshold)
                    
                    # Transform all ArUco positions to homography space
                    self.transform_aruco_positions_to_homography(H)
                else:
                    self.log_warn("⚠️ Poor homography quality: {:.1f}% inliers", inlier_ratio * 100)
                    # Keep the old homography if the new one is poor quality
                    if not self.homography_valid:
                        self.current_homography = H  # Better than nothing
                        self.homography_valid = True
                        self.last_homography_time = current_time
            else:
                self.log_warn("❌ Could not calculate homography")
                if self.current_homography is None:
                    self.homography_valid = False
                # Keep existing homography if calculation failed
                    
            self.homography_calculating = False
            return self.current_homography
                
        except Exception as e:
            self.log_error("Error calculating homography: %s", str(e))
            self.homography_calculating = False
            return self.current_homography  # Return cached version on error

    def get_homography_status(self):
        """
        Returns current homography status for UI display
        """
        try:
            current_time = time.time()
            time_since_calc = current_time - self.last_homography_time if self.last_homography_time > 0 else 0
            
            status = {
                'valid': self.homography_valid,
                'calculating': self.homography_calculating,
                'quality_score': self.homography_quality_score,
                'time_since_calc': time_since_calc,
                'has_matrix': self.current_homography is not None,
                'locked': self.homography_locked,
                'auto_recalc': self.homography_auto_recalc
            }
            return status
        except Exception as e:
            self.log_error("Error getting homography status: %s", str(e))
            return {'valid': False, 'calculating': False, 'quality_score': 0, 
                   'time_since_calc': 0, 'has_matrix': False, 'locked': False, 'auto_recalc': False}

    def draw_zone_info(self, frame, zona):
        """
        Draws enhanced zone information on the frame with colored markers and labels
        """
        try:
            if zona not in self.info_zonas:
                return frame
                
            zona_info = self.info_zonas[zona]
            
            # Define colors for different types of markers
            zone_color = (0, 255, 0)      # Green for zone markers
            robot_color = (0, 0, 255)     # Red for robot
            can_color = (255, 0, 0)       # Blue for cans
            reference_color = (255, 255, 0)  # Cyan for reference markers
            
            # Draw ArUco centers and labels
            for aruco in zona_info['arucos']:
                center = aruco['centro']
                aruco_id = aruco['aruco_id']
                
                # Choose color based on ArUco type
                if aruco_id == self.robot_id:
                    color = robot_color
                    marker_type = "ROBOT"
                    circle_size = 8
                elif 9 <= aruco_id <= 16:
                    color = can_color
                    marker_type = "CAN"
                    circle_size = 5
                elif 1 <= aruco_id <= 9:
                    color = zone_color
                    marker_type = "ZONE"
                    circle_size = 6
                elif 20 <= aruco_id <= 23:
                    color = reference_color
                    marker_type = "REF"
                    circle_size = 6
                else:
                    color = (128, 128, 128)  # Gray for unknown
                    marker_type = "UNK"
                    circle_size = 4
                
                # Draw filled circle
                cv2.circle(frame, center, circle_size, color, -1)
                
                # Draw outer ring for better visibility
                cv2.circle(frame, center, circle_size + 2, (255, 255, 255), 1)
                
                # Draw ArUco ID and type
                label = "{} {}".format(marker_type, aruco_id)
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                
                # Calculate label position to avoid overlap
                label_x = center[0] + 15
                label_y = center[1] - 5
                
                # Draw black background for text
                cv2.rectangle(frame, 
                             (label_x - 2, label_y - label_size[1] - 2),
                             (label_x + label_size[0] + 2, label_y + 2),
                             (0, 0, 0), -1)
                
                # Draw white text
                cv2.putText(frame, label, (label_x, label_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Draw zone number if it's a zone marker
                if marker_type == "ZONE":
                    zone_label = "Z{}".format(zona)
                    cv2.putText(frame, zone_label, (center[0] - 15, center[1] + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                
                # If it's the robot, draw orientation indicator
                if aruco_id == self.robot_id and 'corners' in aruco:
                    self.draw_robot_orientation(frame, center, aruco['corners'])

            return frame
            
        except Exception as e:
            self.log_error("Error in draw_zone_info: %s", str(e))
            return frame

    def draw_robot_orientation(self, frame, center, corners):
        """
        Draws robot orientation indicator
        """
        try:
            if len(corners) >= 4:
                # Calculate orientation vector from corners
                corners_array = np.array(corners, dtype=np.float32)
                top_left = corners_array[0]
                top_right = corners_array[1]
                top_center = (top_left + top_right) / 2.0
                
                # Draw arrow indicating robot orientation
                start_point = (int(center[0]), int(center[1]))
                end_point = (int(top_center[0]), int(top_center[1]))
                
                # Draw orientation arrow
                cv2.arrowedLine(frame, start_point, end_point, (0, 0, 255), 3, tipLength=0.3)
                
                # Draw orientation angle if available
                if self.robot_state['orientation'] is not None:
                    angle_text = "{:.0f}°".format(self.robot_state['orientation'])
                    cv2.putText(frame, angle_text, (center[0] - 20, center[1] + 35), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
        except Exception as e:
            self.log_error("Error drawing robot orientation: %s", str(e))

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
            self.log_error("Error drawing navigation vector: %s", str(e))

    def process_robot_message(self, msg, topic_type):
        """
        Processes messages received from robot topics with enhanced debugging
        """
        try:
            print("🤖 ROBOT MESSAGE RECEIVED!")
            print("   Topic type: {}".format(topic_type))
            print("   Raw message: '{}'".format(msg.data))
            print("   Message type: {}".format(type(msg.data)))
            print("   Timestamp: {}".format(time.time()))
            
            # Parse JSON message
            data = {}
            try:
                print("Mensaje recibido-1: ", msg.data)
                data = json.loads(msg.data)
                print("   ✅ Successfully parsed as JSON")
                
                # Map alternative key names and convert string values to integers
                print("   🔄 Starting conversions...")
                print("      Current data: {}".format(data))
                
                print("      Checking if 'alm1' in data...")
                if 'alm1' in data:
                    print("      ✅ 'alm1' found in data")
                    # Convert "a1" to 1, "a2" to 2, etc.
                    origen_str = data['alm1']
                    print("      origen_str = {}".format(origen_str))
                    print("      type(origen_str) = {}".format(type(origen_str)))
                    print("      isinstance(origen_str, str) = {}".format(isinstance(origen_str, str)))
                    print("      isinstance(origen_str, unicode) = {}".format(isinstance(origen_str, unicode)))
                    print("      origen_str.startswith('a') = {}".format(origen_str.startswith('a')))
                    
                    if isinstance(origen_str, (str, unicode)) and origen_str.startswith('a'):
                        print("      ✅ Conditions met, converting...")
                        try:
                            data['origen'] = int(origen_str[1:])  # Remove 'a' and convert to int
                            print("      🔄 Mapped alm1: '{}' -> origen: {}".format(origen_str, data['origen']))
                        except ValueError:
                            print("      ⚠️ Could not convert alm1 '{}' to integer".format(origen_str))
                    else:
                        print("      ❌ Conditions not met for conversion")
                else:
                    print("      ❌ 'alm1' not found in data")
                    
                print("      Checking if 'alm2' in data...")
                if 'alm2' in data:
                    print("      ✅ 'alm2' found in data")
                    # Convert "a1" to 1, "a2" to 2, etc.
                    destino_str = data['alm2']
                    if isinstance(destino_str, (str, unicode)) and destino_str.startswith('a'):
                        try:
                            data['destino'] = int(destino_str[1:])  # Remove 'a' and convert to int
                            print("      🔄 Mapped alm2: '{}' -> destino: {}".format(destino_str, data['destino']))
                        except ValueError:
                            print("      ⚠️ Could not convert alm2 '{}' to integer".format(destino_str))
                else:
                    print("      ❌ 'alm2' not found in data")
                
                print("      Checking if 'lata' in data...")
                if 'lata' in data:
                    print("      ✅ 'lata' found in data")
                    # Convert "l1" to 1, "l2" to 2, etc.
                    lata_str = data['lata']
                    print("      lata_str = {}".format(lata_str))
                    print("      type(lata_str) = {}".format(type(lata_str)))
                    print("      isinstance(lata_str, str) = {}".format(isinstance(lata_str, str)))
                    print("      isinstance(lata_str, unicode) = {}".format(isinstance(lata_str, unicode)))
                    print("      lata_str.startswith('l') = {}".format(lata_str.startswith('l')))
                    
                    if isinstance(lata_str, (str, unicode)) and lata_str.startswith('l'):
                        print("      ✅ Conditions met, converting...")
                        try:
                            data['lata_id'] = int(lata_str[1:])  # Remove 'l' and convert to int
                            print("      🔄 Mapped lata: '{}' -> lata_id: {}".format(lata_str, data['lata_id']))
                        except ValueError:
                            print("      ⚠️ Could not convert lata '{}' to integer".format(lata_str))
                    else:
                        print("      ❌ Conditions not met for conversion")
                else:
                    print("      ❌ 'lata' not found in data")
                
                print("   ✅ Conversions completed")
                
                # Show data AFTER conversions
                print("Mensaje recibido-2 (after conversions): ", data)
                print("   📋 Parsed data keys (after conversions): {}".format(list(data.keys())))
                    
            except ValueError as json_error:
                print("   ⚠️ JSON parsing failed: {}".format(str(json_error)))
                print("   🔄 Trying key:value format...")
                # Parse key:value format
                data = {}
                for item in msg.data.split(','):
                    if ':' in item:
                        key, value = item.split(':', 1)
                        data[key.strip()] = value.strip()
                print("   ✅ Parsed as key:value format")
                print("Mensaje recibido-2 (key:value format): ", data)
                print("   📋 Parsed data keys (key:value): {}".format(list(data.keys())))
                        
            self.log_info("Received %s message: %s", topic_type, data)
            
            # Update robot state
            with self.processing_lock:
                print("   🔄 Updating robot state...")
                old_mode = self.robot_state.get('modo', 'None')
                self.robot_state['modo'] = topic_type
                print("      Mode: {} -> {}".format(old_mode, topic_type))
                
                if topic_type == "mover_lata":
                    print("   📦 Processing mover_lata command...")
                    if 'lata_id' in data:
                        old_lata = self.robot_state.get('lata_actual', 'None')
                        self.robot_state['lata_actual'] = data['lata_id']
                        print("      Lata: {} -> {}".format(old_lata, self.robot_state['lata_actual']))
                    if 'origen' in data:
                        old_origen = self.robot_state.get('almacen_origen', 'None')
                        self.robot_state['almacen_origen'] = data['origen']
                        print("      Origen: {} -> {}".format(old_origen, self.robot_state['almacen_origen']))
                    if 'destino' in data:
                        old_destino = self.robot_state.get('almacen_destino', 'None')
                        self.robot_state['almacen_destino'] = data['destino']
                        print("      Destino: {} -> {}".format(old_destino, self.robot_state['almacen_destino']))
                    
                    # Get positions from transformed homography space
                    print("   🔍 Getting positions from transformed homography space...")
                    
                    transformed_positions = self.robot_state.get('transformed_positions', {})
                    lata_id = self.robot_state.get('lata_actual')
                    origen_zone = self.robot_state.get('almacen_origen')
                    destino_zone = self.robot_state.get('almacen_destino')
                    
                    target_can_position = None
                    target_destination_position = None
                    
                    # Get robot position (transformed)
                    robot_position = transformed_positions.get('robot_position')
                    if robot_position:
                        print("      🤖 Robot position (homography): {}".format(robot_position))
                        self.robot_state['robot_homography_position'] = robot_position
                    
                    # Get specific can position (transformed)
                    if lata_id is not None:
                        can_positions = transformed_positions.get('can_positions', {})
                        if lata_id in can_positions:
                            target_can_position = can_positions[lata_id]
                            print("      ✅ Found can {} position (homography): {}".format(lata_id, target_can_position))
                        else:
                            print("      ⚠️ Can {} not found in transformed positions".format(lata_id))
                            print("      Available cans: {}".format(list(can_positions.keys())))
                    
                    # Get destination zone position (transformed)
                    if destino_zone is not None:
                        zone_positions = transformed_positions.get('zone_positions', {})
                        if destino_zone in zone_positions:
                            target_destination_position = zone_positions[destino_zone]
                            print("      ✅ Found destination zone {} position (homography): {}".format(destino_zone, target_destination_position))
                        else:
                            print("      ⚠️ Destination zone {} not found in transformed positions".format(destino_zone))
                            print("      Available zones: {}".format(list(zone_positions.keys())))
                    
                    # Store positions in robot state
                    self.robot_state['target_can_position'] = target_can_position
                    self.robot_state['target_destination_position'] = target_destination_position
                    print("      📍 Target can position (homography): {}".format(target_can_position))
                    print("      📍 Target destination position (homography): {}".format(target_destination_position))
                        
                elif topic_type == "mover_a_lata":
                    print("   🎯 Processing mover_a_lata command...")
                    print("      📋 Available keys in data: {}".format(list(data.keys())))
                    print("      🔍 Checking for 'origen' key...")
                    
                    # For mover_a_lata, alm1 is typically the destination
                    if 'origen' in data:
                        print("      ✅ Found 'origen' key with value: {}".format(data['origen']))
                        old_origen = self.robot_state.get('almacen_origen', 'None')
                        self.robot_state['almacen_origen'] = data['origen']
                        print("      Origen: {} -> {}".format(old_origen, self.robot_state['almacen_origen']))
                        # Also use origen as destination for mover_a_lata
                        old_destino = self.robot_state.get('almacen_destino', 'None')
                        self.robot_state['almacen_destino'] = data['origen']
                        print("      Destino (from origen): {} -> {}".format(old_destino, self.robot_state['almacen_destino']))
                    else:
                        print("      ❌ 'origen' key NOT found in data")
                        print("      🔍 Checking for 'destino' key...")
                        
                        if 'destino' in data:
                            print("      ✅ Found 'destino' key with value: {}".format(data['destino']))
                            old_destino = self.robot_state.get('almacen_destino', 'None')
                            self.robot_state['almacen_destino'] = data['destino']
                            print("      Destino: {} -> {}".format(old_destino, self.robot_state['almacen_destino']))
                        else:
                            print("      ❌ 'destino' key NOT found in data either")
                            print("      🚨 No valid destination found for mover_a_lata!")
                    
                    # Handle lata_id if present
                    print("      🔍 Checking for 'lata_id' key...")
                    if 'lata_id' in data:
                        print("      ✅ Found 'lata_id' key with value: {}".format(data['lata_id']))
                        old_lata = self.robot_state.get('lata_actual', 'None')
                        self.robot_state['lata_actual'] = data['lata_id']
                        print("      Lata: {} -> {}".format(old_lata, self.robot_state['lata_actual']))
                    else:
                        print("      ❌ 'lata_id' key NOT found in data")
                    
                    # Get positions from transformed homography space
                    print("   🔍 Getting destination position from transformed homography space...")
                    
                    transformed_positions = self.robot_state.get('transformed_positions', {})
                    destino_zone = self.robot_state.get('almacen_destino')
                    target_destination_position = None
                    
                    # Get robot position (transformed)
                    robot_position = transformed_positions.get('robot_position')
                    if robot_position:
                        print("      🤖 Robot position (homography): {}".format(robot_position))
                        self.robot_state['robot_homography_position'] = robot_position
                    
                    # Get destination zone position (transformed)
                    if destino_zone is not None:
                        zone_positions = transformed_positions.get('zone_positions', {})
                        if destino_zone in zone_positions:
                            target_destination_position = zone_positions[destino_zone]
                            print("      ✅ Found destination zone {} position (homography): {}".format(destino_zone, target_destination_position))
                        else:
                            print("      ⚠️ Destination zone {} not found in transformed positions".format(destino_zone))
                            print("      Available zones: {}".format(list(zone_positions.keys())))
                    
                    # Store destination position in robot state
                    self.robot_state['target_destination_position'] = target_destination_position
                    self.robot_state['target_can_position'] = None  # No specific can for mover_a_lata
                    print("      📍 Target destination position (homography): {}".format(target_destination_position))
                
                old_navegando = self.robot_state.get('navegando', False)
                self.robot_state['navegando'] = True
                print("      Navegando: {} -> {}".format(old_navegando, True))
                
                print("   📊 Current robot state:")
                for key, value in self.robot_state.items():
                    if key in ['modo', 'lata_actual', 'almacen_origen', 'almacen_destino', 'navegando', 'accion']:
                        print("      {}: {}".format(key, value))
            
            print("   🚀 Starting navigation execution...")
            # Calculate and execute navigation
            self.execute_navigation()
            print("   ✅ Message processing complete!")
            print("=" * 60)
            
        except Exception as e:
            print("   ❌ ERROR processing robot message:")
            print("      Topic: {}".format(topic_type))
            print("      Message: {}".format(msg.data))
            print("      Error: {}".format(str(e)))
            print("      Error type: {}".format(type(e)))
            import traceback
            print("      Traceback:")
            traceback.print_exc()
            self.log_error("Error processing robot message %s: %s", topic_type, str(e))

    def execute_navigation(self):
        """
        Calculates and executes navigation commands using info_zonas data
        """
        try:
            with self.processing_lock:

                print("navegando1::::::::::::::")
                if not self.robot_state['navegando']:
                    return
                print("navegando2::::::::::::::")
                
                # Get target positions from robot state (set during message processing)
                target_can_position = self.robot_state.get('target_can_position')
                target_destination_position = self.robot_state.get('target_destination_position')
                
                # Get zone IDs from robot state
                origen_id = self.robot_state['almacen_origen']
                destino_id = self.robot_state['almacen_destino']
                lata_id = self.robot_state.get('lata_actual')
                
                print("origen_id: ", origen_id)
                print("destino_id: ", destino_id)
                print("lata_id: ", lata_id)
                print("target_can_position: ", target_can_position)
                print("target_destination_position: ", target_destination_position)
                print("navegando3::::::::::::::")
                
                if origen_id is None or destino_id is None:
                    print("navegando3.1::::::::::::::")
                    self.log_warn("Origin or destination not defined")
                    print("navegando3.2::::::::::::::")
                    return
                print("navegando4::::::::::::::")
                
                # Get robot position from homography space (preferred) or fallback to camera space
                pos_robot = self.robot_state.get('robot_homography_position')
                if not pos_robot:
                    # Fallback to camera space position
                    pos_robot = self.get_robot_position()
                    print("      ⚠️ Using fallback camera space robot position")
                else:
                    print("      ✅ Using homography space robot position")
                    
                print("navegando5::::::::::::::")
                print("pos_robot: ", pos_robot)
                
                if not pos_robot:
                    print("navegando6::::::::::::::")
                    self.log_warn("Cannot locate robot")
                    action = "Parar"
                else:
                    # Determine navigation target based on mode
                    if self.robot_state['modo'] == "mover_lata":
                        # For mover_lata: first go to can, then to destination
                        if target_can_position and not self.robot_state.get('can_picked', False):
                            # Robot needs to go to the can first
                            pos_target = target_can_position
                            navigation_phase = "going_to_can"
                            print("      📦 Navigation phase: Going to can at {}".format(pos_target))
                        elif target_destination_position and self.robot_state.get('can_picked', False):
                            # Robot has can, go to destination
                            pos_target = target_destination_position
                            navigation_phase = "going_to_destination"
                            print("      🎯 Navigation phase: Going to destination at {}".format(pos_target))
                        elif target_can_position:
                            # Fallback: go to can
                            pos_target = target_can_position
                            navigation_phase = "going_to_can"
                            print("      📦 Navigation phase: Fallback - Going to can at {}".format(pos_target))
                        else:
                            print("      ❌ No valid target position found for mover_lata")
                            pos_target = None
                            navigation_phase = "unknown"
                            
                    elif self.robot_state['modo'] == "mover_a_lata":
                        # For mover_a_lata: go directly to destination
                        if target_destination_position:
                            pos_target = target_destination_position
                            navigation_phase = "going_to_destination"
                            print("      🎯 Navigation phase: Going to destination at {}".format(pos_target))
                        else:
                            print("      ❌ No valid destination position found for mover_a_lata")
                            pos_target = None
                            navigation_phase = "unknown"
                    else:
                        print("      ❌ Unknown navigation mode: {}".format(self.robot_state['modo']))
                        pos_target = None
                        navigation_phase = "unknown"
                    
                    print("navegando7::::::::::::::")
                    
                    if not pos_target:
                        print("navegando8::::::::::::::")
                        self.log_warn("Cannot determine target position for navigation")
                        action = "Parar"
                    else:
                        # Calculate navigation direction to target
                        print("      🧭 Calculating direction from {} to {}".format(pos_robot, pos_target))
                        
                        # For mover_lata, we need both origin and destination for proper navigation
                        if self.robot_state['modo'] == "mover_lata":
                            # Get origin position (where the can is/was)
                            pos_origen = self.get_zone_position(origen_id)
                            if pos_origen:
                                action = self.calculate_robot_direction(pos_robot, pos_origen, pos_target)
                            else:
                                # Direct navigation to target
                                action = self.calculate_robot_direction_direct(pos_robot, pos_target)
                        else:
                            # Direct navigation to target for mover_a_lata
                            action = self.calculate_robot_direction_direct(pos_robot, pos_target)
                        
                        print("      🎮 Calculated action: {}".format(action))
                    
                    print("navegando9::::::::::::::")
                
                # Check gripper control based on distance to target
                #self.check_gripper_control()
                
                # Update robot state
                self.robot_state['accion'] = action
                print("navegando10::::::::::::::")
                
                # Publish action
                print("publicando action: {}".format(action))
                try:
                    if self.robot_state['modo'] == "mover_lata":
                        print("      📡 Publishing to mover_lata topic...")
                        self.mover_lata_pub.publish(String(data=str(action)))
                        print("      ✅ Published to mover_lata: {}".format(action))
                    elif self.robot_state['modo'] == "mover_a_lata":
                        print("      📡 Publishing to mover_a_lata topic...")
                        self.mover_a_lata_pub.publish(String(data=str(action)))
                        print("      ✅ Published to mover_a_lata: {}".format(action))
                except Exception as pub_error:
                    print("      ❌ Publishing ERROR: {}".format(str(pub_error)))
                    print("      Error type: {}".format(type(pub_error)))
                    import traceback
                    traceback.print_exc()
                    
                print("navegando11::::::::::::::")
                self.log_info("Published action: %s", action)
                print("navegando12::::::::::::::")
                
                # Check if navigation is complete
                if action == "Destino":
                    self.robot_state['navegando'] = False
                    # Reset gripper states for next task
                    self.robot_state['approaching_can'] = False
                    self.robot_state['can_picked'] = False
                    
                    completion_msg = "Destino"
                    if self.robot_state['modo'] == "mover_lata":
                        self.mover_lata_pub.publish(String(data=str(completion_msg)))
                    else:
                        self.mover_a_lata_pub.publish(String(data=str(completion_msg)))
                    self.log_info("Navigation completed: %s", self.robot_state['modo'])
                print("navegando13::::::::::::::")
                
        except Exception as e:
            self.log_error("Error in navigation execution: %s", str(e))

    def calculate_robot_direction_direct(self, pos_robot, pos_target):
        """
        Calculate direction directly from robot position to target position
        """
        try:
            # Calculate distance to target
            distance = self.calculate_distance(pos_robot, pos_target)
            
            if distance < self.destination_threshold:
                return "Destino"
            
            # Get current robot orientation
            current_orientation = self.robot_state.get('orientation')
            
            if current_orientation is None:
                # Fallback to simple direction calculation
                dx = pos_target[0] - pos_robot[0]
                dy = pos_target[1] - pos_robot[1]
                
                if abs(dx) > abs(dy):
                    return "GirarD" if dx > 0 else "GirarI"
                else:
                    return "Recto" if dy > 0 else "Atras"
            
            # Calculate desired direction
            dx = pos_target[0] - pos_robot[0]
            dy = pos_target[1] - pos_robot[1]
            desired_angle = np.degrees(np.arctan2(dy, dx))
            if desired_angle < 0:
                desired_angle += 360
            
            # Calculate angle difference
            angle_diff = desired_angle - current_orientation
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            
            # Decision logic
            angle_tolerance = 15.0
            
            if abs(angle_diff) <= angle_tolerance:
                return "Recto"
            elif angle_diff > angle_tolerance:
                return "GirarI"  # Turn left
            elif angle_diff < -angle_tolerance:
                return "GirarD"  # Turn right
            else:
                return "Parar"
                
        except Exception as e:
            self.log_error("Error in direct direction calculation: %s", str(e))
            return "Parar"

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
            self.log_error("Error getting zone %d position: %s", zona_id, str(e))
            return None

    def get_robot_position(self):
        """
        Gets current robot position and orientation based on robot ArUco detection
        """
        try:
            print("get_robot_position::::::::::::::")
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
                        print("get_robot_position2::::::::::::::")
                        return position
            print("get_robot_position3::::::::::::::")
            return None
            
        except Exception as e:
            self.log_error("Error getting robot position: %s", str(e))
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
            self.log_error("Error calculating robot orientation: %s", str(e))
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
            self.log_error("Error calculating orientation confidence: %s", str(e))
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
                self.log_warn("Invalid gripper action: %s", action)
                return
                
            # Only send command if state is changing
            if self.robot_state['gripper_state'] != action:
                message = "gripper:{}".format(action)
                self.gripper_pub.publish(String(data=message))
                self.robot_state['gripper_state'] = action
                self.log_info("Gripper command sent: %s", action)
                
        except Exception as e:
            self.log_error("Error controlling gripper: %s", str(e))

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
            self.log_error("Error getting target can position: %s", str(e))
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
                        self.log_info("Approaching can, opening gripper. Distance: %.1f", distance_to_can)
                        
                    elif distance_to_can <= self.gripper_close_threshold and self.robot_state['gripper_state'] == 'open':
                        self.control_gripper("close")
                        self.robot_state['can_picked'] = True
                        self.robot_state['approaching_can'] = False
                        self.log_info("Can reached, closing gripper. Distance: %.1f", distance_to_can)
                        
            elif self.robot_state['modo'] == "mover_a_lata":
                # Moving to a destination zone: prepare gripper for can placement
                # Since zones 7-8 are storage zones, robot should arrive with gripper closed (holding can)
                # and open gripper to drop the can when close to zone center
                if distance_to_can <= self.gripper_close_threshold and self.robot_state['gripper_state'] == 'closed':
                    # Robot has arrived at destination zone with can - open gripper to drop it
                    self.control_gripper("open")
                    self.log_info("Arrived at destination zone, opening gripper to drop can. Distance: %.1f", distance_to_can)
                    
        except Exception as e:
            self.log_error("Error in gripper control check: %s", str(e))

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
                    self.log_info("Destination reached, dropping can")
                return "Destino"
            
            # Get current robot orientation
            current_orientation = self.robot_state.get('orientation')
            orientation_vector = self.robot_state.get('orientation_vector')
            
            if current_orientation is None or orientation_vector is None:
                # Fallback to old method if orientation is not available
                self.log_warn("Robot orientation not available, using fallback navigation")
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
            self.log_debug("Robot orientation: %.1f°, Desired: %.1f°, Diff: %.1f°", 
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
                    self.log_info("Large angle difference (%.1f°), moving backward", angle_diff)
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
            self.log_error("Error calculating robot direction: %s", str(e))
            return "Parar"

    def calculate_robot_direction_fallback(self, pos_actual, pos_origen, pos_destino):
        """
        Fallback navigation method when orientation is not available
        Uses the old position-based approach
        """
        try:
            self.log_warn("Using fallback navigation - orientation not available")
            
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
                    self.log_info("Moving backward from origin (fallback)")
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
            self.log_error("Error in fallback direction calculation: %s", str(e))
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
            
            self.log_info("Calibration parameters saved to 'camera_calibration.npz'")
            return True
            
        except Exception as e:
            self.log_error("Error saving calibration: %s", str(e))
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
                    self.log_info("Calibration parameters loaded")
                    return True
                else:
                    self.log_warn("Frame size mismatch in calibration")
            return False
            
        except Exception as e:
            self.log_error("Error loading calibration: %s", str(e))
            return False

    def save_homography(self):
        """
        Saves homography matrix to file for reuse
        """
        try:
            if self.current_homography is not None and self.homography_valid:
                save_data = {
                    'homography_matrix': self.current_homography,
                    'quality_score': self.homography_quality_score,
                    'reference_positions': self.last_reference_positions,
                    'timestamp': time.time()
                }
                np.savez('homography_calibration.npz', **save_data)
                self.log_info("✅ Homography saved to 'homography_calibration.npz'")
                return True
            else:
                self.log_warn("❌ No valid homography to save")
                return False
                
        except Exception as e:
            self.log_error("Error saving homography: %s", str(e))
            return False

    def load_homography(self):
        """
        Loads homography matrix from file
        """
        try:
            if os.path.exists('homography_calibration.npz'):
                data = np.load('homography_calibration.npz', allow_pickle=True)
                
                if 'homography_matrix' in data:
                    self.current_homography = data['homography_matrix']
                    self.homography_quality_score = float(data.get('quality_score', 0))
                    self.last_reference_positions = data.get('reference_positions', {}).item() if 'reference_positions' in data else {}
                    saved_time = data.get('timestamp', 0)
                    
                    self.homography_valid = True
                    self.last_homography_time = saved_time
                    
                    age = time.time() - saved_time if saved_time > 0 else 0
                    self.log_info("✅ Homography loaded (quality: {:.1f}%, age: {:.1f}s)", 
                                 self.homography_quality_score * 100, age)
                    return True
                    
            return False
            
        except Exception as e:
            self.log_error("Error loading homography: %s", str(e))
            return False

    def process_frame(self, frame):
        """
        Enhanced frame processing with comprehensive visual markers
        """
        try:
            # Detect ArUcos
            num_detected = self.detect_arucos(frame)
            
            # Calculate homography only if needed
            H = None
            if not self.performance_mode or not self.current_homography:
                H = self.calculate_homography(frame)
            else:
                H = self.current_homography
            
            # Create enhanced visualization frame
            frame_viz = frame.copy()
            
            # Draw all detected ArUcos with enhanced visuals
            total_arucos = 0
            for zona in range(1, 9):
                total_arucos += len(self.info_zonas[zona]['arucos'])
                for aruco in self.info_zonas[zona]['arucos']:
                    center = aruco['centro']
                    aruco_id = aruco['aruco_id']
                    corners = aruco.get('corners', [])
                    
                    # Define colors and labels based on ArUco type
                    if aruco_id == self.robot_id:
                        color = (0, 0, 255)      # Red for robot
                        label = "ROBOT {}".format(aruco_id)
                        circle_size = 12
                        draw_orientation = True
                    elif 1 <= aruco_id <= 8:
                        color = (255, 0, 0)      # Blue for cans
                        label = "CAN {}".format(aruco_id)
                        circle_size = 8
                        draw_orientation = False
                    elif 9 <= aruco_id <= 16:
                        color = (0, 255, 0)      # Green for zones
                        zone_num = aruco_id - 8
                        label = "ZONE {} (ID:{})".format(zone_num, aruco_id)
                        circle_size = 8
                        draw_orientation = False
                    elif 20 <= aruco_id <= 23:
                        color = (255, 255, 0)    # Cyan for reference
                        label = "REF {}".format(aruco_id)
                        circle_size = 8
                        draw_orientation = False
                    else:
                        color = (128, 128, 128)  # Gray for unknown
                        label = "ID {}".format(aruco_id)
                        circle_size = 6
                        draw_orientation = False
                    
                    # Draw marker outline if corners available
                    if corners and len(corners) >= 4:
                        corners_array = np.array(corners, dtype=np.int32)
                        cv2.polylines(frame_viz, [corners_array], True, color, 3)
                    
                    # Draw center circle with white ring
                    cv2.circle(frame_viz, center, circle_size, color, -1)
                    cv2.circle(frame_viz, center, circle_size + 3, (255, 255, 255), 2)
                    
                    # Draw label with black background
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    label_x = center[0] + 15
                    label_y = center[1] - 10
                    
                    # Black background for text readability
                    cv2.rectangle(frame_viz, 
                                 (label_x - 5, label_y - label_size[1] - 5),
                                 (label_x + label_size[0] + 5, label_y + 5),
                                 (0, 0, 0), -1)
                    
                    # White text
                    cv2.putText(frame_viz, label, (label_x, label_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Draw orientation arrow for robot
                    if draw_orientation and corners and len(corners) >= 4:
                        self.draw_robot_orientation(frame_viz, center, corners)
            
            # Draw reference markers (might not be in zones)
            for aruco_id in range(20, 24):
                if self.arucos_medio[aruco_id]:
                    aruco_data = self.arucos_medio[aruco_id][-1]
                    center = aruco_data['center']
                    color = (255, 255, 0)  # Cyan for reference
                    
                    # Draw reference marker
                    cv2.circle(frame_viz, center, 8, color, -1)
                    cv2.circle(frame_viz, center, 11, (255, 255, 255), 2)
                    
                    # Label
                    label = "REF {}".format(aruco_id)
                    cv2.putText(frame_viz, label, (center[0] + 15, center[1] - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw comprehensive status information
            status_y = 30
            line_height = 35
            
            # Main status
            status_text = "ArUcos Detected: {} | Robot ID: {} | Mode: {}".format(
                total_arucos, self.robot_id, 
                self.robot_state.get('modo', 'Standby'))
            cv2.putText(frame_viz, status_text, (10, status_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Robot status
            if self.robot_state['navegando']:
                action_text = "Action: {} | Gripper: {}".format(
                    self.robot_state.get('accion', 'None'),
                    self.robot_state.get('gripper_state', 'unknown'))
                cv2.putText(frame_viz, action_text, (10, status_y + line_height), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                status_y += line_height
            
            # Performance mode indicator
            perf_text = "Performance Mode: {}".format(
                "ON" if self.performance_mode else "OFF")
            perf_color = (0, 255, 0) if self.performance_mode else (0, 165, 255)
            cv2.putText(frame_viz, perf_text, (10, status_y + line_height), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, perf_color, 2)
            
            # Homography status
            if H is not None:
                homo_text = "Homography: Valid | Quality: {:.0f}%".format(
                    self.homography_quality_score * 100)
                cv2.putText(frame_viz, homo_text, (10, status_y + line_height * 2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Show transformed positions count
                transformed_positions = self.robot_state.get('transformed_positions', {})
                if transformed_positions:
                    robot_pos = transformed_positions.get('robot_position')
                    can_count = len(transformed_positions.get('can_positions', {}))
                    zone_count = len(transformed_positions.get('zone_positions', {}))
                    
                    transform_text = "Transformed: Robot:{} | Cans:{} | Zones:{}".format(
                        "✓" if robot_pos else "✗", can_count, zone_count)
                    cv2.putText(frame_viz, transform_text, (10, status_y + line_height * 3), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Draw color legend
            self.draw_enhanced_legend(frame_viz)
            
            # Draw controls help
            self.draw_controls_help(frame_viz)
            
            # Show the main camera window
            cv2.imshow("Robot Navigation - ArUco Detection", frame_viz)
            
            # Create bird's eye view if homography is available and not in performance mode
            if H is not None and not self.performance_mode:
                warped_frame = cv2.warpPerspective(frame, H, (800, 800))
                
                # Draw zone information on warped frame
                for zona in range(1, 9):
                    warped_frame = self.draw_zone_info(warped_frame, zona)
                
                # Draw navigation vector if navigating
                if self.robot_state['navegando']:
                    self.draw_navigation_vector(frame_viz, warped_frame, H)
                
                cv2.imshow("Bird's Eye View", warped_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return False  # Signal to quit
            elif key == ord('f'):
                # Toggle performance mode
                self.performance_mode = not self.performance_mode
                if self.performance_mode:
                    self.log_info("🚀 PERFORMANCE MODE ENABLED - Ultra-fast processing")
                    cv2.destroyWindow("Bird's Eye View")  # Close bird's eye view
                else:
                    self.log_info("🐌 PERFORMANCE MODE DISABLED - Full features enabled")
            elif key == ord('r'):
                # Reset robot state
                self.robot_state['navegando'] = False
                self.robot_state['accion'] = "Parar"
                if not self.reduce_logging:
                    self.log_info("Robot state reset")
            elif key == ord('h') and not self.performance_mode:
                # Force homography recalculation (disabled in performance mode)
                if not self.homography_calculating:
                    self.log_info("🔄 Manually triggering homography recalculation...")
                    self.homography_valid = False
                    self.last_homography_time = 0
                    H = self.calculate_homography(frame_viz)
                    if H is not None:
                        self.log_info("✅ Homography manually recalculated")
                    else:
                        self.log_warn("❌ Failed to recalculate homography")
                else:
                    self.log_info("Homography calculation already running, please wait...")
            elif key == ord('s') and not self.performance_mode:
                # Save homography (disabled in performance mode)
                if self.save_homography():
                    self.log_info("💾 Homography saved successfully")
                else:
                    self.log_warn("❌ Failed to save homography")
            
            return True
            
        except Exception as e:
            if not self.reduce_logging:
                self.log_error("Error in process_frame: %s", str(e))
            return True

    def draw_enhanced_legend(self, frame):
        """
        Draw enhanced color legend with all marker types
        """
        try:
            # Legend position
            legend_x = frame.shape[1] - 220
            legend_y = 30
            
            # Legend items with colors
            legend_items = [
                ("🔴 ROBOT (ID {})".format(self.robot_id), (0, 0, 255)),
                ("🔵 CAN (ID 1-8)", (255, 0, 0)),
                ("🟢 ZONE (ID 9-16)", (0, 255, 0)),
                ("🟡 REF (ID 20-23)", (255, 255, 0)),
                ("⚪ UNKNOWN", (128, 128, 128))
            ]
            
            # Calculate legend size
            legend_height = len(legend_items) * 25 + 20
            
            # Draw legend background
            cv2.rectangle(frame, (legend_x - 10, legend_y - 10), 
                         (legend_x + 200, legend_y + legend_height), 
                         (0, 0, 0), -1)
            cv2.rectangle(frame, (legend_x - 10, legend_y - 10), 
                         (legend_x + 200, legend_y + legend_height), 
                         (255, 255, 255), 2)
            
            # Draw legend title
            cv2.putText(frame, "ARUCO MARKERS", (legend_x, legend_y + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Draw legend items
            for i, (label, color) in enumerate(legend_items):
                y_pos = legend_y + 30 + i * 25
                cv2.circle(frame, (legend_x + 15, y_pos), 6, color, -1)
                cv2.circle(frame, (legend_x + 15, y_pos), 8, (255, 255, 255), 1)
                cv2.putText(frame, label[2:], (legend_x + 30, y_pos + 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
        except Exception as e:
            self.log_error("Error drawing legend: %s", str(e))

    def draw_controls_help(self, frame):
        """
        Draw controls help at the bottom of the frame
        """
        try:
            # Controls text
            controls = [
                "'q' - Quit",
                "'f' - Toggle Performance Mode", 
                "'r' - Reset Robot",
                "'h' - Recalc Homography",
                "'s' - Save Homography"
            ]
            
            # Draw controls at bottom
            start_y = frame.shape[0] - 30
            for i, control in enumerate(controls):
                x_pos = 10 + i * 200
                if x_pos < frame.shape[1] - 100:
                    cv2.putText(frame, control, (x_pos, start_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            
        except Exception as e:
            self.log_error("Error drawing controls help: %s", str(e))

    def run(self):
        """
        Main execution loop
        """
        self.log_info("Starting robot navigation system...")
        self.log_info("Camera source: %s", self.camera_source)
        self.log_info("Press 'q' to quit, 'c' to capture calibration frame, 'r' to reset robot state")
        
        try:
            if self.camera_source == "ros" and self.use_ros:
                # Use ROS spin for ROS camera
                rospy.spin()
            else:
                # Use manual frame capture loop for other camera sources
                while True:
                    frame = self.get_frame()
                    if frame is not None:
                        if not self.process_frame(frame):
                            break  # Quit signal received
                    else:
                        self.log_info("No frame received, retrying...")
                        time.sleep(0.1)
                        
                    # Small delay to prevent excessive CPU usage
                    time.sleep(0.03)  # ~30 FPS
                    
        except KeyboardInterrupt:
            self.log_info("Shutting down robot navigation system")
        except Exception as e:
            self.log_error("Error in main loop: %s", str(e))
        finally:
            self.cleanup()

    def cleanup(self):
        """
        Clean up resources
        """
        try:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            self.log_info("Cleanup completed")
        except Exception as e:
            self.log_error("Error during cleanup: %s", str(e))

    def generate_aruco_markers(self, output_dir="aruco_markers"):
        """
        Generates ArUco markers for testing
        """
        try:
            import os
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            
            # Generate markers for different purposes
            marker_sets = {
                'reference': [20, 21, 22, 23],  # Reference markers for homography
                'zones': list(range(9, 17)),     # Zone markers (9-16 for zones 1-8)
                'cans': list(range(1, 9)),       # Can markers (1-8)
                'robot': [self.robot_id]         # Robot marker
            }
            
            marker_size = 200  # pixels
            
            for set_name, marker_ids in marker_sets.items():
                set_dir = os.path.join(output_dir, set_name)
                if not os.path.exists(set_dir):
                    os.makedirs(set_dir)
                
                for marker_id in marker_ids:
                    # Generate marker
                    marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
                    
                    # Add border and label
                    bordered_img = np.ones((marker_size + 60, marker_size + 60), dtype=np.uint8) * 255
                    bordered_img[30:30+marker_size, 30:30+marker_size] = marker_img
                    
                    # Add ID label
                    cv2.putText(bordered_img, "ID: {}".format(marker_id), 
                               (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                    
                    # Add purpose label
                    purpose = ""
                    if set_name == 'reference':
                        purpose = "Reference"
                    elif set_name == 'zones':
                        zone_num = marker_id - 8
                        purpose = "Zone {}".format(zone_num)
                    elif set_name == 'cans':
                        purpose = "Can {}".format(marker_id)
                    elif set_name == 'robot':
                        purpose = "Robot"
                    
                    cv2.putText(bordered_img, purpose, 
                               (10, marker_size + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    
                    # Save marker
                    filename = "aruco_{:02d}_{}.png".format(marker_id, purpose.lower().replace(" ", "_"))
                    filepath = os.path.join(set_dir, filename)
                    cv2.imwrite(filepath, bordered_img)
                    
                    self.log_info("Generated marker: %s", filepath)
            
            self.log_info("ArUco markers generated in directory: %s", output_dir)
            return True
            
        except Exception as e:
            self.log_error("Error generating ArUco markers: %s", str(e))
            return False

    def log_info(self, message, *args):
        """
        Optimized logging function
        """
        if not self.reduce_logging:
            if self.use_ros and ROS_AVAILABLE:
                rospy.loginfo(message, *args)
            else:
                if args:
                    print(message % args)
                else:
                    print(message)

    def log_warn(self, message, *args):
        """
        Optimized warning logging function
        """
        if not self.reduce_logging:
            if self.use_ros and ROS_AVAILABLE:
                rospy.logwarn(message, *args)
            else:
                if args:
                    print("WARNING: " + message % args)
                else:
                    print("WARNING: " + message)

    def log_error(self, message, *args):
        """
        Optimized error logging function
        """
        if not self.reduce_logging:
            if self.use_ros and ROS_AVAILABLE:
                rospy.logerr(message, *args)
            else:
                if args:
                    print("ERROR: " + message % args)
                else:
                    print("ERROR: " + message)

    def log_debug(self, message, *args):
        """
        Optimized debug logging function
        """
        if not self.reduce_logging and not self.performance_mode:
            if self.use_ros and ROS_AVAILABLE:
                rospy.logdebug(message, *args)
            else:
                if args:
                    print("DEBUG: " + message % args)
                else:
                    print("DEBUG: " + message)

    def check_ros_topics(self):
        """
        Check ROS topics and connections for debugging
        """
        try:
            print("🔍 ROS DIAGNOSTIC CHECK:")
            
            # Check if ROS master is running
            try:
                import rosgraph
                master = rosgraph.Master('/rostopic')
                print("   ✅ ROS Master is running")
                
                # List all topics
                topics = master.getPublishedTopics('')
                print("   📡 Available topics:")
                for topic_name, topic_type in topics:
                    print("      - {} ({})".format(topic_name, topic_type))
                    
                # Check specifically for our topics
                our_topics = ["/info_robot_mover_lata", "/info_robot_mover_a_lata"]
                found_topics = [topic[0] for topic in topics]
                
                print("   🎯 Checking for robot communication topics:")
                for topic in our_topics:
                    if topic in found_topics:
                        print("      ✅ {} - FOUND".format(topic))
                    else:
                        print("      ❌ {} - NOT FOUND".format(topic))
                        
            except Exception as e:
                print("   ❌ Cannot connect to ROS Master: {}".format(str(e)))
                print("   💡 Make sure to run: roscore")
                
        except ImportError:
            print("   ❌ rosgraph not available")
        except Exception as e:
            print("   ❌ Error in ROS diagnostic: {}".format(str(e)))

    def test_ros_publisher(self):
        """
        Test publishing a message to verify ROS communication
        """
        try:
            print("📡 Testing ROS publisher...")
            test_message = "test_from_camera_{}".format(time.time())
            
            # Test publishing
            self.mover_lata_pub.publish(String(data=test_message))
            print("   ✅ Test message published: {}".format(test_message))
            print("   📻 Robot should receive this message if listening")
            
        except Exception as e:
            print("   ❌ Error testing publisher: {}".format(str(e)))

    def transform_aruco_positions_to_homography(self, H):
        """
        Transform all detected ArUco positions to homography space (bird's eye view)
        and store them in robot state for navigation
        """
        try:
            if H is None:
                print("❌ No valid homography matrix available")
                return
            
            print("🔄 Transforming ArUco positions to homography space...")
            
            # Initialize transformed positions storage
            transformed_positions = {
                'robot_position': None,
                'can_positions': {},  # {can_id: (x, y)}
                'zone_positions': {},  # {zone_id: (x, y)}
                'reference_positions': {}  # {ref_id: (x, y)}
            }
            
            # Transform robot position (ArUco ID 0)
            robot_found = False
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    if aruco['aruco_id'] == self.robot_id:  # Robot ID = 0
                        center = aruco['centro']
                        # Transform to homography space
                        point = np.array([[center]], dtype=np.float32)
                        transformed_point = cv2.perspectiveTransform(point, H)[0][0]
                        transformed_positions['robot_position'] = (int(transformed_point[0]), int(transformed_point[1]))
                        robot_found = True
                        print("   🤖 Robot (ID {}): {} -> {}".format(
                            self.robot_id, center, transformed_positions['robot_position']))
                        break
                if robot_found:
                    break
            
            if not robot_found:
                print("   ⚠️ Robot ArUco ID {} not found".format(self.robot_id))
            
            # Transform can positions (ArUco IDs 1-8, 9-16 depending on your mapping)
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    aruco_id = aruco['aruco_id']
                    center = aruco['centro']
                    
                    # Transform position to homography space
                    point = np.array([[center]], dtype=np.float32)
                    transformed_point = cv2.perspectiveTransform(point, H)[0][0]
                    transformed_pos = (int(transformed_point[0]), int(transformed_point[1]))
                    
                    # Categorize based on ArUco ID
                    if 1 <= aruco_id <= 8:  # Individual cans
                        transformed_positions['can_positions'][aruco_id] = transformed_pos
                        print("   📦 Can (ID {}): {} -> {}".format(aruco_id, center, transformed_pos))
                    
                    elif 9 <= aruco_id <= 16:  # Zone markers (these could also be cans in your system)
                        # Map zone marker ID to zone number
                        zone_num = aruco_id - 8  # ID 9->Zone 1, ID 10->Zone 2, etc.
                        if zone_num not in transformed_positions['zone_positions']:
                            transformed_positions['zone_positions'][zone_num] = []
                        transformed_positions['zone_positions'][zone_num].append(transformed_pos)
                        print("   🏠 Zone {} (ID {}): {} -> {}".format(zone_num, aruco_id, center, transformed_pos))
            
            # Transform reference markers (ArUco IDs 20-23)
            for aruco_id in range(20, 24):
                if self.arucos_medio[aruco_id]:
                    aruco_data = self.arucos_medio[aruco_id][-1]
                    center = aruco_data['center']
                    point = np.array([[center]], dtype=np.float32)
                    transformed_point = cv2.perspectiveTransform(point, H)[0][0]
                    transformed_pos = (int(transformed_point[0]), int(transformed_point[1]))
                    transformed_positions['reference_positions'][aruco_id] = transformed_pos
                    print("   🎯 Reference (ID {}): {} -> {}".format(aruco_id, center, transformed_pos))
            
            # Calculate zone centers from multiple markers
            for zone_id, positions in transformed_positions['zone_positions'].items():
                if positions:
                    if len(positions) > 1:
                        # Average multiple markers in the same zone
                        x_avg = sum(p[0] for p in positions) / len(positions)
                        y_avg = sum(p[1] for p in positions) / len(positions)
                        transformed_positions['zone_positions'][zone_id] = (int(x_avg), int(y_avg))
                        print("   🏠 Zone {} center (from {} markers): ({}, {})".format(
                            zone_id, len(positions), int(x_avg), int(y_avg)))
                    else:
                        # Single marker in zone
                        transformed_positions['zone_positions'][zone_id] = positions[0]
                        print("   🏠 Zone {} (single marker): {}".format(zone_id, positions[0]))
            
            # Store transformed positions in robot state
            self.robot_state['transformed_positions'] = transformed_positions
            
            print("✅ Position transformation complete!")
            print("   🤖 Robot: {}".format(transformed_positions['robot_position']))
            print("   📦 Cans: {} found".format(len(transformed_positions['can_positions'])))
            print("   🏠 Zones: {} found".format(len(transformed_positions['zone_positions'])))
            print("   🎯 References: {} found".format(len(transformed_positions['reference_positions'])))
            
            return transformed_positions
            
        except Exception as e:
            self.log_error("Error transforming ArUco positions: %s", str(e))
            return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Autonomous Robot Navigation System')
    parser.add_argument('--camera', choices=['mobile', 'usb', 'file', 'ros'], default='mobile',
                       help='Camera source type (default: mobile for IP Camera app)')
    parser.add_argument('--ip', type=str, default='192.168.71.56:8080',
                       help='Mobile camera IP address (default: 192.168.71.56:8080)')
    parser.add_argument('--video-file', type=str,
                       help='Video file path (required for file camera)')
    parser.add_argument('--ros', action='store_true',
                       help='Enable ROS functionality')
    parser.add_argument('--robot-id', type=int, default=0,
                       help='Robot ArUco marker ID (default: 0)')
    parser.add_argument('--autonomous', action='store_true', default=True,
                       help='Run in autonomous mode (no interactive prompts)')
    parser.add_argument('--show-info', action='store_true',
                       help='Show startup information and controls (disabled in autonomous mode)')
    parser.add_argument('--generate-markers', action='store_true',
                       help='Generate ArUco markers and exit')
    
    args = parser.parse_args()
    
    # Generate markers if requested
    if args.generate_markers:
        print("🎯 Generating ArUco markers...")
        navigator = Navegacion_Robot(camera_source='usb', use_ros=False, autonomous=True)
        navigator.robot_id = args.robot_id
        success = navigator.generate_aruco_markers()
        if success:
            print("✅ ArUco markers generated successfully!")
            print("📁 Check the 'aruco_markers' directory")
        else:
            print("❌ Failed to generate markers")
        exit(0)
    
    # Validate required parameters for autonomous operation
    if args.camera == 'file' and not args.video_file:
        print("❌ ERROR: Video file path required. Use --video-file argument.")
        print("Example: python {} --camera file --video-file /path/to/video.mp4".format(__file__))
        exit(1)
        
    if args.camera == 'ros' and not ROS_AVAILABLE:
        print("❌ ERROR: ROS not available!")
        exit(1)

    # Show startup information only if requested (disabled in autonomous mode by default)
    if args.show_info:
        print("=" * 60)
        print("🤖 AUTONOMOUS ROBOT NAVIGATION SYSTEM")
        print("=" * 60)
        print("Camera: {}".format(args.camera))
        if args.camera == 'mobile':
            print("Mobile IP: {}".format(args.ip))
    elif args.camera == 'file':
            print("Video file: {}".format(args.video_file))

    
    try:
        # Create navigator instance
        navigator = Navegacion_Robot(
            camera_source=args.camera,
            mobile_ip="192.168.71.56:8080",
            video_file=args.video_file,
            use_ros=args.ros,
            autonomous=args.autonomous
        )
        
        # Set robot ID
        navigator.robot_id = args.robot_id
        
        # Autonomous startup message (brief)
        if args.autonomous:
            print("🚀 Starting autonomous navigation system...")
            print("   Camera: {} | IP: {} | Robot ID: {} | Performance: ENABLED".format(
                args.camera, "192.168.71.56:8080", args.robot_id))
        
        # Check if camera initialization was successful
        if not navigator.camera_active:
            print("❌ Camera initialization failed. Exiting.")
            exit(1)
        
        print("✅ System ready. Navigation active.")
        
        # Run the navigation system
        navigator.run()
        
    except KeyboardInterrupt:
        print("\n🛑 System stopped.")
    except Exception as e:
        print("❌ Critical error: {}".format(str(e)))
        exit(1)