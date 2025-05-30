# 📱 Mobile Camera Setup Guide

## Overview
This guide shows you how to use your mobile phone as a camera for the robot navigation system instead of ROS topics.

## 🚀 Quick Start

### 1. Install Mobile Camera App

**For Android:**
- Install **"IP Webcam"** by Pavel Khlebovich (free)
- Alternative: **"DroidCam"** 

**For iPhone:**
- Install **"EpocCam"** 
- Alternative: **"iVCam"**

### 2. Setup Network Connection
1. Connect your phone and computer to the **same WiFi network**
2. Note your phone's IP address from the app
3. The app will show something like: `http://192.168.1.100:8080`

### 3. Run the System

**Basic usage:**
```bash
python visión_cenitalV1_movil.py --camera mobile --ip 192.168.1.100:8080
```

**With custom robot ID:**
```bash
python visión_cenitalV1_movil.py --camera mobile --ip 192.168.1.100:8080 --robot-id 5
```

**With ROS enabled:**
```bash
python visión_cenitalV1_movil.py --camera mobile --ip 192.168.1.100:8080 --ros
```

## 📋 All Camera Options

### Mobile Phone Camera
```bash
python visión_cenitalV1_movil.py --camera mobile --ip YOUR_PHONE_IP:8080
```

### USB Camera
```bash
python visión_cenitalV1_movil.py --camera usb
```

### Video File
```bash
python visión_cenitalV1_movil.py --camera file
```

### ROS Topic (original)
```bash
python visión_cenitalV1_movil.py --camera ros --ros
```

## 🎮 Controls

- **'q'** - Quit the application
- **'c'** - Capture calibration frame
- **'r'** - Reset robot state

## 📱 Mobile App Configuration

### IP Webcam (Android)
1. Open the app
2. Scroll down and tap **"Start server"**
3. Note the IP address shown (e.g., `http://192.168.1.100:8080`)
4. Optional: Adjust video quality in settings for better performance

### EpocCam (iPhone)
1. Install EpocCam on iPhone and computer
2. Connect both devices to same WiFi
3. Start the app on iPhone
4. Note the IP address displayed

## 🔧 Troubleshooting

### Camera Connection Issues
1. **Check WiFi**: Ensure both devices are on the same network
2. **Firewall**: Temporarily disable firewall on computer
3. **IP Address**: Verify the IP address is correct
4. **Port**: Try different ports (8080, 8081, 4747)

### Performance Issues
1. **Lower Resolution**: Reduce camera resolution in mobile app
2. **Frame Rate**: Lower FPS in app settings
3. **Network**: Use 5GHz WiFi if available

### Common Error Messages

**"Failed to connect to mobile camera"**
- Check IP address format: `192.168.1.100:8080`
- Ensure mobile app is running
- Try different URL formats (the system tries multiple automatically)

**"No frame received"**
- Check network connection
- Restart mobile camera app
- Try different video quality settings

## 🎯 ArUco Marker Setup

The system expects these ArUco markers:
- **Reference markers**: IDs 20, 21, 22, 23 (for homography)
- **Zone markers**: IDs 9-16 (for zones 1-8)
- **Robot marker**: ID 0 (configurable with `--robot-id`)

## 📊 System Features

✅ **Works without ROS** - Standalone operation  
✅ **Multiple camera sources** - Mobile, USB, file, ROS  
✅ **Real-time ArUco detection** - Optimized parameters  
✅ **Robot orientation tracking** - Enhanced navigation  
✅ **Homography correction** - Bird's eye view  
✅ **Gripper control** - Distance-based automation  
✅ **Visual feedback** - Status display and navigation vectors  

## 🔄 Migration from ROS Version

If you're migrating from the ROS-only version:

1. **Keep ROS functionality**: Add `--ros` flag
2. **Same ArUco IDs**: No changes needed
3. **Same navigation logic**: All algorithms preserved
4. **Additional features**: Mobile camera + standalone mode

## 📞 Support

If you encounter issues:
1. Check this troubleshooting guide
2. Verify network connectivity
3. Test with USB camera first
4. Check ArUco marker visibility

## 🎉 Success Indicators

When working correctly, you should see:
- ✅ "Successfully connected to mobile camera"
- Real-time video feed in OpenCV windows
- ArUco markers detected and labeled
- Robot orientation displayed (if robot marker visible)
- Homography working (bird's eye view window) 