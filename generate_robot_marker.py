#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
ArUco marker generator for robot identification
"""
import cv2
import numpy as np

def generate_aruco_marker(marker_id, size=200, save_path=None):
    """
    Generate an ArUco marker
    
    Args:
        marker_id: ID of the marker (0-49 for DICT_4X4_50)
        size: Size of the marker in pixels
        save_path: Path to save the marker image
    """
    try:
        # Create ArUco dictionary
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        
        # Generate marker
        marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, size)
        
        # Add border and text for printing
        border_size = 50
        total_size = size + 2 * border_size
        
        # Create white background
        final_img = np.ones((total_size, total_size), dtype=np.uint8) * 255
        
        # Place marker in center
        start_pos = border_size
        final_img[start_pos:start_pos+size, start_pos:start_pos+size] = marker_img
        
        # Add text
        text = "ArUco ID: {}".format(marker_id)
        cv2.putText(final_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
        
        text2 = "ROBOT MARKER"
        cv2.putText(final_img, text2, (10, total_size - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 2)
        
        # Save if path provided
        if save_path:
            cv2.imwrite(save_path, final_img)
            print("✅ Marker saved to: {}".format(save_path))
        
        return final_img
        
    except Exception as e:
        print("❌ Error generating marker: {}".format(str(e)))
        return None

def main():
    print("🎯 ArUco Robot Marker Generator")
    print("=" * 40)
    
    # Generate common robot marker IDs
    robot_ids = [0, 1, 2, 15, 17, 18, 19]
    
    print("Generating robot markers for IDs: {}".format(robot_ids))
    print("\nRecommended robot ID: 0 (default)")
    print("Alternative IDs: 1, 2, 17, 18, 19")
    print("⚠️  Avoid IDs 9-16 (zones) and 20-23 (reference)")
    
    for robot_id in robot_ids:
        filename = "robot_marker_id_{}.png".format(robot_id)
        marker = generate_aruco_marker(robot_id, size=200, save_path=filename)
        
        if marker is not None:
            # Display marker
            cv2.imshow("Robot Marker ID {}".format(robot_id), marker)
            print("📄 Generated: {}".format(filename))
    
    print("\n🖨️  Printing Instructions:")
    print("1. Print the marker on white paper")
    print("2. Cut out the marker with some white border")
    print("3. Attach to the TOP of your robot")
    print("4. Ensure the marker is flat and visible from above")
    print("5. The TOP edge of the marker = robot forward direction")
    
    print("\n📐 Size Recommendations:")
    print("- Small robot: 3-5 cm marker")
    print("- Medium robot: 5-8 cm marker") 
    print("- Large robot: 8-12 cm marker")
    print("- Larger markers = better detection from distance")
    
    print("\nPress any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 