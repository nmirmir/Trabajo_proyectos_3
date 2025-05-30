# ArUco Detection Improvements

## 🔧 What Was Fixed

The original code had several issues with ArUco detection:

1. **Limited preprocessing**: Only basic grayscale conversion and histogram equalization
2. **Single parameter set**: Used only one set of detection parameters
3. **Poor error handling**: Limited debugging information when detection failed
4. **No detection optimization**: No attempt to try different methods when detection failed

## 🚀 Improvements Made

### 1. **Multi-Method Preprocessing**
- **Original grayscale**: Basic conversion
- **Enhanced**: Histogram equalization for better contrast
- **Blurred**: Gaussian blur to reduce noise
- **Bilateral filter**: Edge-preserving noise reduction
- **Morphological**: Closing operation to fill gaps

### 2. **Multiple Detection Parameter Sets**
- **Standard**: Default OpenCV parameters
- **High Sensitivity**: More aggressive detection for small/distant markers
- **Low Threshold**: Better detection for poor lighting conditions

### 3. **Enhanced Detection Algorithm**
- Tests all combinations of preprocessing + parameters
- Keeps track of best detection method
- Provides detailed logging of detection attempts
- Stores unique detections to avoid duplicates

### 4. **Debug Visualization**
- Press `d` to toggle debug mode
- Shows all preprocessing methods side-by-side
- Highlights which method detected which markers
- Real-time detection statistics

### 5. **Better ArUco ID Management**
- **Reference markers**: IDs 20-23 (for homography)
- **Zone markers**: IDs 9-16 (zones 1-8)
- **Can markers**: IDs 1-8 (individual cans)
- **Robot marker**: ID 25 (configurable, changed from 15 to avoid conflicts)

## 🎯 How to Use

### Basic Usage
```bash
# Run with mobile camera
python visión_cenitalV1_movil.py --camera mobile --ip 192.168.1.100:8080

# Run with USB camera
python visión_cenitalV1_movil.py --camera usb

# Set custom robot ID
python visión_cenitalV1_movil.py --camera usb --robot-id 25
```

### Generate Test Markers
```bash
# Generate all markers for testing
python visión_cenitalV1_movil.py --generate-markers

# Generate with custom robot ID
python visión_cenitalV1_movil.py --generate-markers --robot-id 25
```

### Debug Mode
1. Run the program normally
2. Press `d` to enable debug mode
3. A new window will show all preprocessing methods
4. You can see which method works best for your lighting conditions

### Test Detection Only
```bash
# Generate test markers
python test_aruco_detection.py --generate

# Test detection with camera
python test_aruco_detection.py --test
```

## 🔍 Troubleshooting ArUco Detection

### Problem: No markers detected at all

**Possible causes:**
- Poor lighting conditions
- Markers too small or too far
- Camera focus issues
- Wrong ArUco dictionary

**Solutions:**
1. **Check lighting**: Ensure good, even lighting on markers
2. **Enable debug mode**: Press `d` to see which preprocessing method works
3. **Adjust camera**: Move closer to markers or improve focus
4. **Generate test markers**: Use `--generate-markers` to create proper markers

### Problem: Intermittent detection

**Possible causes:**
- Camera shake or movement
- Varying lighting conditions
- Marker orientation issues

**Solutions:**
1. **Stabilize camera**: Use a tripod or stable mount
2. **Improve lighting**: Add consistent lighting source
3. **Check marker quality**: Ensure markers are flat and not damaged

### Problem: Wrong IDs detected

**Possible causes:**
- Marker damage or printing issues
- Similar-looking patterns in the scene
- Wrong ArUco dictionary

**Solutions:**
1. **Regenerate markers**: Use the built-in marker generator
2. **Check marker quality**: Ensure clean, high-contrast printing
3. **Remove false positives**: Clear the camera view of similar patterns

### Problem: Detection works in debug but not in main program

**Possible causes:**
- Different preprocessing being used
- Timing issues with camera capture

**Solutions:**
1. **Note which debug method works**: Check the debug window output
2. **Modify preprocessing**: Adjust the main detection to use the working method
3. **Check frame rate**: Ensure camera is providing stable frames

## 📊 Detection Statistics

The improved system provides detailed logging:

```
INFO: Best detection: 4 markers with enhanced + High Sensitivity
INFO: Detected ArUco IDs: [9, 20, 21, 25]
INFO: Processed 4 ArUco markers total
DEBUG: Zone ArUco 9 detected in zone 1 at (320, 240)
DEBUG: Reference ArUco 20 detected at (100, 100)
DEBUG: Reference ArUco 21 detected at (100, 500)
INFO: Robot ArUco 25 detected at position: (400, 300) (method: enhanced + High Sensitivity)
```

## 🎮 Controls

- `q` - Quit program
- `c` - Capture calibration frame
- `r` - Reset robot state
- `d` - **Toggle debug mode** (NEW!)

## 📁 Generated Files

### Marker Generation
When you run `--generate-markers`, it creates:
```
aruco_markers/
├── reference/          # IDs 20-23 (homography reference)
├── zones/             # IDs 9-16 (zone markers)
├── cans/              # IDs 1-8 (can markers)
└── robot/             # Robot marker (configurable ID)
```

### Test Markers
When you run `test_aruco_detection.py --generate`:
```
test_aruco_01.png      # Can marker
test_aruco_09.png      # Zone marker
test_aruco_15.png      # Old robot ID
test_aruco_20.png      # Reference marker
test_aruco_21.png      # Reference marker
test_aruco_22.png      # Reference marker
test_aruco_23.png      # Reference marker
test_aruco_25.png      # New robot marker
```

## 🔧 Advanced Configuration

### Custom Detection Parameters
You can modify the detection parameters in the code:

```python
# In detect_arucos() function
params.adaptiveThreshConstant = 7        # Lower = more sensitive
params.minMarkerPerimeterRate = 0.03     # Lower = smaller markers allowed
params.maxMarkerPerimeterRate = 4.0      # Higher = larger markers allowed
```

### Camera Settings
Adjust camera properties for better detection:

```python
# In initialize_camera() function
self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)     # Manual exposure
self.cap.set(cv2.CAP_PROP_GAIN, 0)          # Manual gain
self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)  # Brightness adjustment
```

## 📈 Performance Tips

1. **Use appropriate marker size**: Markers should be at least 50x50 pixels in the image
2. **Maintain good contrast**: Black markers on white background work best
3. **Avoid reflections**: Use matte printing, avoid glossy surfaces
4. **Stable mounting**: Reduce camera shake for consistent detection
5. **Good lighting**: Even, diffused lighting works better than harsh directional light

## 🐛 Common Issues and Fixes

### Issue: "No ArUco markers detected with any method"
- Check if markers are in the camera view
- Verify markers are from DICT_4X4_50 dictionary
- Try different lighting conditions
- Enable debug mode to see preprocessing results

### Issue: Detection is slow
- Reduce frame size in `self.frame_size`
- Limit the number of preprocessing methods tested
- Use fewer parameter sets for detection

### Issue: False positive detections
- Increase `minMarkerPerimeterRate` to filter small false positives
- Improve lighting to reduce noise
- Use corner refinement for better accuracy

### Issue: Robot orientation calculation fails
- Ensure robot marker is clearly visible
- Check that all 4 corners are detected properly
- Verify marker is not too small or distorted

This improved detection system should significantly increase the reliability of ArUco marker detection in various lighting and environmental conditions. 