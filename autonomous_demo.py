#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Autonomous Robot Navigation System Demo

This script demonstrates how to run the autonomous robot navigation system
without any interactive prompts or menus.

Usage examples:

1. Mobile Camera (default - IP Camera app):
   python autonomous_demo.py

2. USB Camera:
   python autonomous_demo.py --camera usb

3. Mobile Camera with different IP:
   python autonomous_demo.py --camera mobile --ip 192.168.1.100:8080

4. Video File:
   python autonomous_demo.py --camera file --video-file /path/to/video.mp4

5. ROS Camera:
   python autonomous_demo.py --camera ros --ros

6. Generate ArUco markers:
   python autonomous_demo.py --generate-markers

The system will run completely autonomously with no user interaction required.
"""

import subprocess
import sys
import os

def run_autonomous_navigation(camera_type="usb", **kwargs):
    """
    Run the autonomous navigation system with specified parameters
    
    Args:
        camera_type: "usb", "mobile", "file", or "ros"
        **kwargs: Additional arguments like ip, video_file, robot_id, etc.
    """
    
    # Build command
    cmd = [sys.executable, "visión_cenitalV1_movil.py"]
    cmd.extend(["--camera", camera_type])
    cmd.append("--autonomous")  # Always run in autonomous mode
    
    # Add optional parameters
    if "ip" in kwargs:
        cmd.extend(["--ip", kwargs["ip"]])
    if "video_file" in kwargs:
        cmd.extend(["--video-file", kwargs["video_file"]])
    if "robot_id" in kwargs:
        cmd.extend(["--robot-id", str(kwargs["robot_id"])])
    if kwargs.get("ros", False):
        cmd.append("--ros")
    if kwargs.get("generate_markers", False):
        cmd.append("--generate-markers")
    if kwargs.get("show_info", False):
        cmd.append("--show-info")
    
    print("🚀 Starting autonomous robot navigation system...")
    print("Command: {}".format(" ".join(cmd)))
    print("=" * 60)
    
    try:
        # Run the navigation system
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("❌ Navigation system exited with error code: {}".format(e.returncode))
    except KeyboardInterrupt:
        print("\n🛑 Demo stopped by user.")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Autonomous Robot Navigation Demo')
    parser.add_argument('--camera', choices=['usb', 'mobile', 'file', 'ros'], 
                       default='mobile', help='Camera source (default: mobile for IP Camera app)')
    parser.add_argument('--ip', help='Mobile camera IP (e.g., 192.168.1.100:8080)')
    parser.add_argument('--video-file', help='Path to video file')
    parser.add_argument('--robot-id', type=int, default=25, help='Robot ArUco ID')
    parser.add_argument('--ros', action='store_true', help='Enable ROS')
    parser.add_argument('--generate-markers', action='store_true', 
                       help='Generate ArUco markers and exit')
    parser.add_argument('--show-info', action='store_true',
                       help='Show detailed startup information')
    
    args = parser.parse_args()
    
    # Check if main navigation file exists
    if not os.path.exists("visión_cenitalV1_movil.py"):
        print("❌ Error: visión_cenitalV1_movil.py not found!")
        print("Make sure you're running this from the correct directory.")
        return
    
    # Run with specified parameters
    kwargs = {
        "robot_id": args.robot_id,
        "ros": args.ros,
        "generate_markers": args.generate_markers,
        "show_info": args.show_info
    }
    
    if args.ip:
        kwargs["ip"] = args.ip
    if args.video_file:
        kwargs["video_file"] = args.video_file
    
    run_autonomous_navigation(args.camera, **kwargs)

if __name__ == "__main__":
    print("🤖 AUTONOMOUS ROBOT NAVIGATION DEMO")
    print("=" * 60)
    print("This demo runs the robot navigation system in fully autonomous mode.")
    print("No interactive prompts or menus will appear.")
    print("=" * 60)
    main() 