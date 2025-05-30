#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Simple ArUco Detection Test Script
This script helps test and debug ArUco detection issues
"""

import cv2
import numpy as np
import argparse

def test_aruco_detection():
    """
    Simple test function for ArUco detection
    """
    print("🔍 ArUco Detection Test")
    print("=" * 40)
    
    # Create ArUco dictionary
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    
    # Test with different parameter sets
    parameter_sets = []
    
    # Standard parameters
    params1 = cv2.aruco.DetectorParameters_create()
    parameter_sets.append(("Standard", params1))
    
    # High sensitivity parameters
    params2 = cv2.aruco.DetectorParameters_create()
    params2.adaptiveThreshWinSizeMin = 3
    params2.adaptiveThreshWinSizeMax = 53
    params2.adaptiveThreshWinSizeStep = 4
    params2.adaptiveThreshConstant = 7
    params2.minMarkerPerimeterRate = 0.03
    params2.maxMarkerPerimeterRate = 4.0
    parameter_sets.append(("High Sensitivity", params2))
    
    # Low threshold parameters
    params3 = cv2.aruco.DetectorParameters_create()
    params3.adaptiveThreshConstant = 5
    params3.minMarkerPerimeterRate = 0.01
    params3.maxMarkerPerimeterRate = 8.0
    parameter_sets.append(("Low Threshold", params3))
    
    # Initialize camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Could not open camera")
        return
    
    print("📷 Camera opened successfully")
    print("Press 'q' to quit, 's' to save current frame")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read frame")
            break
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Test different preprocessing methods
        preprocessing_methods = {
            'original': gray,
            'enhanced': cv2.equalizeHist(gray),
            'blurred': cv2.GaussianBlur(gray, (3, 3), 0),
            'bilateral': cv2.bilateralFilter(gray, 9, 75, 75)
        }
        
        best_detection = None
        max_detections = 0
        
        # Try all combinations
        for preprocess_name, processed_frame in preprocessing_methods.items():
            for param_name, params in parameter_sets:
                try:
                    corners, ids, rejected = cv2.aruco.detectMarkers(processed_frame, aruco_dict, parameters=params)
                    
                    if ids is not None and len(ids) > max_detections:
                        max_detections = len(ids)
                        best_detection = {
                            'corners': corners,
                            'ids': ids,
                            'method': "{} + {}".format(preprocess_name, param_name),
                            'rejected': len(rejected) if rejected is not None else 0
                        }
                except Exception as e:
                    continue
        
        # Draw results on frame
        display_frame = frame.copy()
        
        if best_detection and max_detections > 0:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(display_frame, best_detection['corners'], best_detection['ids'])
            
            # Add detection info
            info_text = "Detected: {} markers | Method: {}".format(
                max_detections, best_detection['method'])
            cv2.putText(display_frame, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Show detected IDs
            ids_text = "IDs: {}".format([int(id_) for id_ in best_detection['ids'].flatten()])
            cv2.putText(display_frame, ids_text, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            print("✅ Detected {} markers: {} using {}".format(
                max_detections, 
                [int(id_) for id_ in best_detection['ids'].flatten()],
                best_detection['method']))
        else:
            cv2.putText(display_frame, "No ArUco markers detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            print("❌ No markers detected")
        
        cv2.imshow("ArUco Detection Test", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite("aruco_test_frame.jpg", frame)
            print("💾 Frame saved as 'aruco_test_frame.jpg'")
    
    cap.release()
    cv2.destroyAllWindows()
    print("👋 Test completed")

def generate_test_markers():
    """
    Generate test ArUco markers
    """
    print("🎯 Generating test ArUco markers...")
    
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    
    # Test marker IDs
    test_ids = [1, 9, 15, 20, 21, 22, 23, 25]
    marker_size = 200
    
    for marker_id in test_ids:
        # Generate marker
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
        
        # Add border and label
        bordered_img = np.ones((marker_size + 60, marker_size + 60), dtype=np.uint8) * 255
        bordered_img[30:30+marker_size, 30:30+marker_size] = marker_img
        
        # Add ID label
        cv2.putText(bordered_img, "ArUco ID: {}".format(marker_id), 
                   (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Save marker
        filename = "test_aruco_{:02d}.png".format(marker_id)
        cv2.imwrite(filename, bordered_img)
        print("✅ Generated: {}".format(filename))
    
    print("🎯 Test markers generated successfully!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ArUco Detection Test Tool')
    parser.add_argument('--generate', action='store_true', 
                       help='Generate test ArUco markers')
    parser.add_argument('--test', action='store_true', 
                       help='Run detection test with camera')
    
    args = parser.parse_args()
    
    if args.generate:
        generate_test_markers()
    elif args.test:
        test_aruco_detection()
    else:
        print("ArUco Detection Test Tool")
        print("Usage:")
        print("  python test_aruco_detection.py --generate  # Generate test markers")
        print("  python test_aruco_detection.py --test      # Test detection with camera") 