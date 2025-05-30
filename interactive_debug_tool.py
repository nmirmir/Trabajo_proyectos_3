#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Interactive Parameter Adjustment Tool for ArUco Detection
This tool provides real-time parameter adjustment and extensive debugging
"""

import cv2
import numpy as np
import json
import time
import threading

class InteractiveDebugTool:
    def __init__(self):
        """Initialize the interactive debugging tool"""
        
        # Image enhancement parameters (adjustable in real-time)
        self.enhancement_params = {
            'brightness': 0,        # -100 to 100
            'contrast': 1.0,        # 0.5 to 3.0
            'saturation': 1.0,      # 0.0 to 2.0
            'gamma': 1.0,           # 0.5 to 2.0
            'blur_kernel': 3,       # 1, 3, 5, 7
            'bilateral_d': 9,       # 5, 9, 15
            'bilateral_sigma': 75,  # 25, 50, 75, 100
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
        
        # Debug settings
        self.debug_settings = {
            'show_preprocessing': True,
            'show_detection_steps': True,
            'show_timing_info': True,
            'verbose_logging': True,
            'save_frames': False
        }
        
        # Performance tracking
        self.stats = {
            'total_frames': 0,
            'successful_detections': 0,
            'detection_times': [],
            'best_methods': {}
        }
        
        # Camera
        self.cap = None
        self.running = False
        
    def initialize_camera(self, camera_id=0):
        """Initialize camera"""
        try:
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                print("❌ Failed to open camera {}".format(camera_id))
                return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print("✅ Camera {} initialized successfully".format(camera_id))
            return True
            
        except Exception as e:
            print("❌ Error initializing camera: {}".format(str(e)))
            return False
    
    def apply_image_enhancements(self, frame):
        """Apply adjustable image enhancements"""
        try:
            enhanced = frame.copy().astype(np.float32)
            
            # Apply brightness
            if self.enhancement_params['brightness'] != 0:
                enhanced = enhanced + self.enhancement_params['brightness']
            
            # Apply contrast
            if self.enhancement_params['contrast'] != 1.0:
                enhanced = enhanced * self.enhancement_params['contrast']
            
            # Apply gamma correction
            if self.enhancement_params['gamma'] != 1.0:
                enhanced = enhanced / 255.0
                enhanced = np.power(enhanced, self.enhancement_params['gamma'])
                enhanced = enhanced * 255.0
            
            # Clip values
            enhanced = np.clip(enhanced, 0, 255).astype(np.uint8)
            
            # Apply saturation (for color images)
            if len(frame.shape) == 3 and self.enhancement_params['saturation'] != 1.0:
                hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV).astype(np.float32)
                hsv[:, :, 1] = hsv[:, :, 1] * self.enhancement_params['saturation']
                hsv[:, :, 1] = np.clip(hsv[:, :, 1], 0, 255)
                enhanced = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
            
            return enhanced
            
        except Exception as e:
            print("❌ Error in image enhancement: {}".format(str(e)))
            return frame
    
    def preprocess_frame(self, frame):
        """Create multiple preprocessed versions of the frame"""
        try:
            # Apply enhancements first
            enhanced_frame = self.apply_image_enhancements(frame)
            
            # Convert to grayscale
            if len(enhanced_frame.shape) == 3:
                gray = cv2.cvtColor(enhanced_frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = enhanced_frame.copy()
            
            # Create different preprocessing methods
            methods = {}
            
            # Original
            methods['original'] = gray.copy()
            
            # CLAHE
            clahe = cv2.createCLAHE(
                clipLimit=self.enhancement_params['clahe_limit'],
                tileGridSize=(self.enhancement_params['clahe_grid'], self.enhancement_params['clahe_grid'])
            )
            methods['clahe'] = clahe.apply(gray)
            
            # Gaussian blur
            kernel_size = self.enhancement_params['blur_kernel']
            if kernel_size % 2 == 0:
                kernel_size += 1
            methods['blurred'] = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
            
            # Bilateral filter
            methods['bilateral'] = cv2.bilateralFilter(
                gray,
                self.enhancement_params['bilateral_d'],
                self.enhancement_params['bilateral_sigma'],
                self.enhancement_params['bilateral_sigma']
            )
            
            # Histogram equalization
            methods['hist_eq'] = cv2.equalizeHist(gray)
            
            return methods
            
        except Exception as e:
            print("❌ Error in preprocessing: {}".format(str(e)))
            return None
    
    def detect_arucos_with_params(self, gray_frame):
        """Detect ArUcos using current parameters"""
        try:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            
            # Create detector parameters
            params = cv2.aruco.DetectorParameters_create()
            params.adaptiveThreshConstant = self.aruco_params['adaptiveThreshConstant']
            params.adaptiveThreshWinSizeMin = self.aruco_params['adaptiveThreshWinSizeMin']
            params.adaptiveThreshWinSizeMax = self.aruco_params['adaptiveThreshWinSizeMax']
            params.adaptiveThreshWinSizeStep = self.aruco_params['adaptiveThreshWinSizeStep']
            params.minMarkerPerimeterRate = self.aruco_params['minMarkerPerimeterRate']
            params.maxMarkerPerimeterRate = self.aruco_params['maxMarkerPerimeterRate']
            params.polygonalApproxAccuracyRate = self.aruco_params['polygonalApproxAccuracyRate']
            params.minCornerDistanceRate = self.aruco_params['minCornerDistanceRate']
            params.minDistanceToBorder = self.aruco_params['minDistanceToBorder']
            params.cornerRefinementWinSize = self.aruco_params['cornerRefinementWinSize']
            params.cornerRefinementMaxIterations = self.aruco_params['cornerRefinementMaxIterations']
            params.cornerRefinementMinAccuracy = self.aruco_params['cornerRefinementMinAccuracy']
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            
            # Detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=params)
            
            return corners, ids, rejected
            
        except Exception as e:
            print("❌ Error in ArUco detection: {}".format(str(e)))
            return None, None, None
    
    def test_all_methods(self, frame):
        """Test detection with all preprocessing methods"""
        start_time = time.time()
        
        # Get preprocessed frames
        preprocessed = self.preprocess_frame(frame)
        if not preprocessed:
            return None
        
        results = {}
        
        # Test each preprocessing method
        for method_name, gray_frame in preprocessed.items():
            corners, ids, rejected = self.detect_arucos_with_params(gray_frame)
            
            detection_count = len(ids) if ids is not None else 0
            rejection_count = len(rejected) if rejected is not None else 0
            
            results[method_name] = {
                'corners': corners,
                'ids': ids,
                'rejected': rejected,
                'detection_count': detection_count,
                'rejection_count': rejection_count,
                'success_rate': detection_count / max(1, detection_count + rejection_count),
                'gray_frame': gray_frame
            }
            
            if self.debug_settings['verbose_logging'] and detection_count > 0:
                print("🔍 {}: {} detected, {} rejected".format(
                    method_name, detection_count, rejection_count))
        
        # Update stats
        detection_time = time.time() - start_time
        self.stats['detection_times'].append(detection_time)
        self.stats['total_frames'] += 1
        
        # Find best method
        best_method = max(results.items(), key=lambda x: x[1]['detection_count'])
        if best_method[1]['detection_count'] > 0:
            self.stats['successful_detections'] += 1
            method_name = best_method[0]
            if method_name not in self.stats['best_methods']:
                self.stats['best_methods'][method_name] = 0
            self.stats['best_methods'][method_name] += 1
        
        return results
    
    def show_parameter_menu(self):
        """Display parameter adjustment menu"""
        print("\n" + "="*80)
        print("🎛️  INTERACTIVE PARAMETER ADJUSTMENT TOOL")
        print("="*80)
        
        print("\n📸 IMAGE ENHANCEMENT PARAMETERS:")
        print("  1. Brightness:     {:+3d}     (Range: -100 to +100)".format(self.enhancement_params['brightness']))
        print("  2. Contrast:       {:.2f}     (Range: 0.5 to 3.0)".format(self.enhancement_params['contrast']))
        print("  3. Saturation:     {:.2f}     (Range: 0.0 to 2.0)".format(self.enhancement_params['saturation']))
        print("  4. Gamma:          {:.2f}     (Range: 0.5 to 2.0)".format(self.enhancement_params['gamma']))
        print("  5. Blur Kernel:    {:d}       (Values: 1, 3, 5, 7)".format(self.enhancement_params['blur_kernel']))
        print("  6. Bilateral D:    {:d}       (Values: 5, 9, 15)".format(self.enhancement_params['bilateral_d']))
        print("  7. Bilateral Sigma:{:d}       (Values: 25, 50, 75, 100)".format(self.enhancement_params['bilateral_sigma']))
        print("  8. CLAHE Limit:    {:.1f}     (Range: 1.0 to 8.0)".format(self.enhancement_params['clahe_limit']))
        print("  9. CLAHE Grid:     {:d}       (Values: 4, 8, 16)".format(self.enhancement_params['clahe_grid']))
        
        print("\n🎯 ARUCO DETECTION PARAMETERS:")
        print("  a. Adaptive Thresh Constant:    {:d}     (Range: 3 to 15)".format(self.aruco_params['adaptiveThreshConstant']))
        print("  b. Min Window Size:             {:d}     (Range: 3 to 10)".format(self.aruco_params['adaptiveThreshWinSizeMin']))
        print("  c. Max Window Size:             {:d}     (Range: 15 to 50)".format(self.aruco_params['adaptiveThreshWinSizeMax']))
        print("  d. Min Marker Perimeter Rate:   {:.3f}  (Range: 0.01 to 0.1)".format(self.aruco_params['minMarkerPerimeterRate']))
        print("  e. Max Marker Perimeter Rate:   {:.1f}   (Range: 2.0 to 8.0)".format(self.aruco_params['maxMarkerPerimeterRate']))
        print("  f. Corner Refinement Win Size:  {:d}     (Range: 3 to 15)".format(self.aruco_params['cornerRefinementWinSize']))
        
        print("\n📊 PERFORMANCE STATS:")
        if self.stats['total_frames'] > 0:
            success_rate = (self.stats['successful_detections'] / self.stats['total_frames']) * 100
            print("  Total Frames:          {:d}".format(self.stats['total_frames']))
            print("  Successful Detections: {:d} ({:.1f}%)".format(self.stats['successful_detections'], success_rate))
            
            if self.stats['detection_times']:
                avg_time = np.mean(self.stats['detection_times'][-10:]) * 1000
                print("  Avg Detection Time:    {:.1f} ms".format(avg_time))
            
            if self.stats['best_methods']:
                best_method = max(self.stats['best_methods'].items(), key=lambda x: x[1])
                print("  Best Method:           {} ({} times)".format(best_method[0], best_method[1]))
        
        print("\n🎮 CONTROLS:")
        print("  'q' - Quit")
        print("  'm' - Show this menu")
        print("  '1'-'9' - Adjust image parameters")
        print("  'a'-'f' - Adjust ArUco parameters")
        print("  'r' - Reset to defaults")
        print("  's' - Save parameters")
        print("  'l' - Load parameters")
        print("="*80)
    
    def adjust_parameter(self, key):
        """Adjust parameter based on key press"""
        try:
            if key in '123456789':
                # Image enhancement parameters
                param_map = {
                    '1': ('brightness', -100, 100, int, self.enhancement_params),
                    '2': ('contrast', 0.5, 3.0, float, self.enhancement_params),
                    '3': ('saturation', 0.0, 2.0, float, self.enhancement_params),
                    '4': ('gamma', 0.5, 2.0, float, self.enhancement_params),
                    '5': ('blur_kernel', [1, 3, 5, 7], None, int, self.enhancement_params),
                    '6': ('bilateral_d', [5, 9, 15], None, int, self.enhancement_params),
                    '7': ('bilateral_sigma', [25, 50, 75, 100], None, int, self.enhancement_params),
                    '8': ('clahe_limit', 1.0, 8.0, float, self.enhancement_params),
                    '9': ('clahe_grid', [4, 8, 16], None, int, self.enhancement_params)
                }
                
                if key in param_map:
                    param_name, min_val, max_val, param_type, param_dict = param_map[key]
                    current_val = param_dict[param_name]
                    
                    print("\n🔧 Adjusting: {} (current: {})".format(param_name, current_val))
                    
                    if isinstance(min_val, list):
                        print("Available values: {}".format(min_val))
                        try:
                            new_val = input("Enter new value: ").strip()
                            new_val = param_type(new_val)
                            if new_val in min_val:
                                param_dict[param_name] = new_val
                                print("✅ {} set to {}".format(param_name, new_val))
                            else:
                                print("❌ Invalid value")
                        except ValueError:
                            print("❌ Invalid input format")
                    else:
                        print("Range: {} to {}".format(min_val, max_val))
                        try:
                            new_val = input("Enter new value: ").strip()
                            new_val = param_type(new_val)
                            if min_val <= new_val <= max_val:
                                param_dict[param_name] = new_val
                                print("✅ {} set to {}".format(param_name, new_val))
                            else:
                                print("❌ Value out of range")
                        except ValueError:
                            print("❌ Invalid input format")
            
            elif key in 'abcdef':
                # ArUco detection parameters
                param_map = {
                    'a': ('adaptiveThreshConstant', 3, 15, int),
                    'b': ('adaptiveThreshWinSizeMin', 3, 10, int),
                    'c': ('adaptiveThreshWinSizeMax', 15, 50, int),
                    'd': ('minMarkerPerimeterRate', 0.01, 0.1, float),
                    'e': ('maxMarkerPerimeterRate', 2.0, 8.0, float),
                    'f': ('cornerRefinementWinSize', 3, 15, int)
                }
                
                if key in param_map:
                    param_name, min_val, max_val, param_type = param_map[key]
                    current_val = self.aruco_params[param_name]
                    
                    print("\n🎯 Adjusting: {} (current: {})".format(param_name, current_val))
                    print("Range: {} to {}".format(min_val, max_val))
                    
                    try:
                        new_val = input("Enter new value: ").strip()
                        new_val = param_type(new_val)
                        if min_val <= new_val <= max_val:
                            self.aruco_params[param_name] = new_val
                            print("✅ {} set to {}".format(param_name, new_val))
                        else:
                            print("❌ Value out of range")
                    except ValueError:
                        print("❌ Invalid input format")
                        
        except Exception as e:
            print("❌ Error adjusting parameter: {}".format(str(e)))
    
    def save_parameters(self, filename="debug_parameters.json"):
        """Save current parameters"""
        try:
            data = {
                'enhancement_params': self.enhancement_params,
                'aruco_params': self.aruco_params,
                'debug_settings': self.debug_settings
            }
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            print("✅ Parameters saved to {}".format(filename))
        except Exception as e:
            print("❌ Error saving: {}".format(str(e)))
    
    def load_parameters(self, filename="debug_parameters.json"):
        """Load parameters from file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            if 'enhancement_params' in data:
                self.enhancement_params.update(data['enhancement_params'])
            if 'aruco_params' in data:
                self.aruco_params.update(data['aruco_params'])
            if 'debug_settings' in data:
                self.debug_settings.update(data['debug_settings'])
            
            print("✅ Parameters loaded from {}".format(filename))
        except Exception as e:
            print("❌ Error loading: {}".format(str(e)))
    
    def reset_parameters(self):
        """Reset all parameters to defaults"""
        self.enhancement_params = {
            'brightness': 0, 'contrast': 1.0, 'saturation': 1.0, 'gamma': 1.0,
            'blur_kernel': 3, 'bilateral_d': 9, 'bilateral_sigma': 75,
            'clahe_limit': 2.0, 'clahe_grid': 8,
        }
        self.aruco_params = {
            'adaptiveThreshConstant': 7, 'adaptiveThreshWinSizeMin': 3,
            'adaptiveThreshWinSizeMax': 23, 'adaptiveThreshWinSizeStep': 10,
            'minMarkerPerimeterRate': 0.03, 'maxMarkerPerimeterRate': 4.0,
            'polygonalApproxAccuracyRate': 0.05, 'minCornerDistanceRate': 0.05,
            'minDistanceToBorder': 3, 'cornerRefinementWinSize': 5,
            'cornerRefinementMaxIterations': 30, 'cornerRefinementMinAccuracy': 0.1,
        }
        print("✅ Parameters reset to defaults")
    
    def run(self):
        """Main execution loop"""
        if not self.initialize_camera():
            return
        
        print("🎛️  Interactive ArUco Detection Debug Tool")
        print("Press 'm' for menu, 'q' to quit")
        self.show_parameter_menu()
        
        self.running = True
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Failed to read frame")
                    break
                
                # Test all detection methods
                results = self.test_all_methods(frame)
                
                if results:
                    # Find best result
                    best_method = max(results.items(), key=lambda x: x[1]['detection_count'])
                    best_name, best_result = best_method
                    
                    # Display original frame with detections
                    display_frame = frame.copy()
                    
                    if best_result['ids'] is not None and len(best_result['ids']) > 0:
                        cv2.aruco.drawDetectedMarkers(display_frame, best_result['corners'], best_result['ids'])
                        
                        # Add detection info
                        info_text = "Best: {} ({} markers)".format(best_name, best_result['detection_count'])
                        cv2.putText(display_frame, info_text, (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        ids_text = "IDs: {}".format([int(id_) for id_ in best_result['ids'].flatten()])
                        cv2.putText(display_frame, ids_text, (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        cv2.putText(display_frame, "No markers detected", (10, 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # Show parameter values
                    param_text = "B:{:+d} C:{:.1f} G:{:.1f} T:{}".format(
                        self.enhancement_params['brightness'],
                        self.enhancement_params['contrast'],
                        self.enhancement_params['gamma'],
                        self.aruco_params['adaptiveThreshConstant']
                    )
                    cv2.putText(display_frame, param_text, (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    cv2.imshow("Interactive ArUco Debug", display_frame)
                    
                    # Show preprocessing methods if enabled
                    if self.debug_settings['show_preprocessing']:
                        self.show_preprocessing_grid(results)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('m'):
                    self.show_parameter_menu()
                elif key == ord('r'):
                    self.reset_parameters()
                elif key == ord('s'):
                    self.save_parameters()
                elif key == ord('l'):
                    self.load_parameters()
                elif chr(key) in '123456789abcdef':
                    self.adjust_parameter(chr(key))
                    
        except KeyboardInterrupt:
            print("\n👋 Shutting down...")
        finally:
            self.cleanup()
    
    def show_preprocessing_grid(self, results):
        """Show grid of preprocessing methods"""
        try:
            frames = []
            for method_name, result in results.items():
                if 'gray_frame' in result:
                    frame = result['gray_frame'].copy()
                    
                    # Draw detections if any
                    if result['ids'] is not None and len(result['ids']) > 0:
                        cv2.aruco.drawDetectedMarkers(frame, result['corners'], result['ids'])
                    
                    # Add method label
                    cv2.putText(frame, "{} ({})".format(method_name, result['detection_count']), 
                               (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # Resize for grid
                    frame = cv2.resize(frame, (200, 150))
                    frames.append(frame)
            
            if len(frames) >= 4:
                # Create 2x2 grid (or more)
                top_row = np.hstack(frames[:2])
                bottom_row = np.hstack(frames[2:4])
                grid = np.vstack([top_row, bottom_row])
                
                if len(frames) > 4:
                    # Add remaining frames
                    extra_frames = frames[4:]
                    while len(extra_frames) < 2:
                        extra_frames.append(np.zeros((150, 200), dtype=np.uint8))
                    extra_row = np.hstack(extra_frames[:2])
                    grid = np.vstack([grid, extra_row])
                
                cv2.imshow("Preprocessing Methods", grid)
                
        except Exception as e:
            print("❌ Error showing preprocessing grid: {}".format(str(e)))
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("🧹 Cleanup completed")

if __name__ == "__main__":
    print("🎛️  Starting Interactive ArUco Debug Tool...")
    tool = InteractiveDebugTool()
    tool.run() 