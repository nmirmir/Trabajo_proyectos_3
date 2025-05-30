#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Simple test script to verify mobile camera connection
"""
import cv2
import sys
import time

def test_mobile_camera(ip_address):
    """
    Test mobile camera connection
    """
    print("=" * 50)
    print("📱 MOBILE CAMERA CONNECTION TEST")
    print("=" * 50)
    print("Testing IP: {}".format(ip_address))
    
    # Try different URL formats
    test_urls = [
        "http://{}/video".format(ip_address),
        "http://{}/cam.mjpg".format(ip_address),
        "http://{}/video.mjpg".format(ip_address),
        "http://{}/mjpeg".format(ip_address),
    ]
    
    for i, url in enumerate(test_urls):
        print("\n🔍 Test {}: {}".format(i+1, url))
        
        cap = cv2.VideoCapture(url)
        
        if cap.isOpened():
            print("✅ Connection successful!")
            
            # Try to read a few frames
            frame_count = 0
            start_time = time.time()
            
            for _ in range(10):
                ret, frame = cap.read()
                if ret:
                    frame_count += 1
                    if frame_count == 1:
                        print("📏 Frame size: {}x{}".format(frame.shape[1], frame.shape[0]))
                else:
                    break
                    
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            print("📊 Captured {} frames in {:.2f}s ({:.1f} FPS)".format(
                frame_count, elapsed, fps))
            
            if frame_count > 0:
                print("🎉 SUCCESS! Mobile camera is working")
                print("\nTo use with navigation system:")
                print("python visión_cenitalV1_movil.py --camera mobile --ip {}".format(ip_address))
                
                # Show a test frame
                print("\nShowing test frame... Press any key to close")
                cv2.imshow("Mobile Camera Test", frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                cap.release()
                return True
            else:
                print("❌ No frames received")
                
        else:
            print("❌ Connection failed")
            
        cap.release()
    
    print("\n❌ All connection attempts failed")
    print("\n🔧 TROUBLESHOOTING:")
    print("1. Check that your phone and computer are on the same WiFi")
    print("2. Verify the IP address is correct")
    print("3. Make sure the camera app is running on your phone")
    print("4. Try restarting the camera app")
    print("5. Check firewall settings")
    
    return False

def test_usb_camera():
    """
    Test USB camera as fallback
    """
    print("\n" + "=" * 50)
    print("🔌 USB CAMERA TEST")
    print("=" * 50)
    
    for i in range(3):
        print("Testing USB camera {}...".format(i))
        cap = cv2.VideoCapture(i)
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print("✅ USB camera {} working!".format(i))
                print("📏 Frame size: {}x{}".format(frame.shape[1], frame.shape[0]))
                
                print("Showing USB camera... Press any key to close")
                cv2.imshow("USB Camera Test", frame)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                cap.release()
                return True
            else:
                print("❌ USB camera {} found but no frames".format(i))
        else:
            print("❌ USB camera {} not found".format(i))
            
        cap.release()
    
    print("❌ No working USB cameras found")
    return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python test_mobile_camera.py <IP_ADDRESS:PORT>")
        print("Example: python test_mobile_camera.py 192.168.1.100:8080")
        sys.exit(1)
    
    ip_address = sys.argv[1]
    
    # Test mobile camera
    mobile_success = test_mobile_camera(ip_address)
    
    if not mobile_success:
        # Test USB camera as fallback
        print("\nTrying USB camera as fallback...")
        usb_success = test_usb_camera()
        
        if usb_success:
            print("\n💡 Suggestion: Use USB camera with:")
            print("python visión_cenitalV1_movil.py --camera usb")
    
    print("\n" + "=" * 50)
    print("🏁 TEST COMPLETE")
    print("=" * 50) 