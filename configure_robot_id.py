#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Configuration script to change robot ArUco ID in the navigation system
"""
import re

def change_robot_id(new_id):
    """
    Changes the robot ID in the main navigation file
    """
    filename = "visión_cenitalV1_BUENA.py"
    
    try:
        # Read the file
        with open(filename, 'r') as f:
            content = f.read()
        
        # Find and replace the robot_id line
        pattern = r'self\.robot_id\s*=\s*\d+'
        replacement = 'self.robot_id = {}'.format(new_id)
        
        new_content = re.sub(pattern, replacement, content)
        
        # Write back to file
        with open(filename, 'w') as f:
            f.write(new_content)
        
        print("✅ Successfully changed robot ID to: {}".format(new_id))
        print("📝 Updated file: {}".format(filename))
        print("🔄 Please restart your navigation system to apply changes")
        
        return True
        
    except Exception as e:
        print("❌ Error changing robot ID: {}".format(str(e)))
        return False

def main():
    print("🤖 Robot ID Configuration Tool")
    print("=" * 40)
    print("Current robot ID in system: 0")
    print("\nCommon robot IDs to try:")
    print("  0  - Default robot ID")
    print("  1  - Alternative robot ID")
    print("  2  - Alternative robot ID")
    print("  15 - If you have an existing marker")
    print("  17 - Alternative ID")
    print("  18 - Alternative ID")
    print("  19 - Alternative ID")
    print("\nNote: Avoid IDs 9-16 (zones) and 20-23 (reference markers)")
    
    try:
        new_id = int(input("\nEnter new robot ID: "))
        
        # Validate ID
        if new_id in range(9, 17) or new_id in range(20, 24):
            print("⚠️  Warning: ID {} is reserved for zones or reference markers!".format(new_id))
            confirm = input("Continue anyway? (y/N): ")
            if confirm.lower() != 'y':
                print("❌ Cancelled")
                return
        
        if change_robot_id(new_id):
            print("\n🎯 Next steps:")
            print("1. Run the test script: python test_robot_detection.py")
            print("2. Check if your robot ArUco marker is detected")
            print("3. Restart your main navigation system")
        
    except ValueError:
        print("❌ Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        print("\n❌ Cancelled by user")

if __name__ == "__main__":
    main() 