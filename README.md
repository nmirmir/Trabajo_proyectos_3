# Robot Navigation System - Improved Version

## Overview
This is an improved version of the robot navigation system that uses ArUco markers for localization and provides overhead (bird's eye) view navigation capabilities.

## Key Improvements Made

### 1. **Fixed Syntax Errors**
- Fixed incomplete exception handling (`Exception as e:` instead of `Exceptio`)
- Corrected missing variable declarations
- Fixed string formatting issues for Python 2.7 compatibility

### 2. **Enhanced Image Processing**
- **Proper ROS Image Conversion**: Uses `cv_bridge` for reliable ROS Image to OpenCV conversion
- **Optimized ArUco Detection**: Improved detection parameters for better marker recognition
- **Reduced Blur**: Changed from (21,21) to (5,5) Gaussian blur for better ArUco detection
- **Better Error Handling**: Comprehensive try-catch blocks throughout the pipeline

### 3. **Improved Code Organization**
- **Modular Functions**: Split large functions into smaller, focused methods
- **Thread Safety**: Added threading locks for concurrent access to robot state
- **Cleaner Data Structures**: Simplified zone and ArUco data management
- **Better Naming**: More descriptive function and variable names

### 4. **Enhanced Navigation System**
- **Robust Direction Calculation**: Improved vector-based navigation algorithm
- **Better State Management**: Cleaner robot state tracking with navigation flags
- **Error Recovery**: Graceful handling of missing robot or zone positions
- **Visual Feedback**: Real-time navigation vectors displayed on both views

### 5. **Performance Optimizations**
- **Efficient Data Clearing**: Only clear detection data when necessary
- **Reduced Redundancy**: Eliminated duplicate calculations
- **Better Memory Management**: Proper cleanup of OpenCV windows
- **Optimized Homography**: Cache homography matrix when stable

### 6. **Enhanced Visualization**
- **Status Display**: Real-time status information on screen
- **Navigation Vectors**: Visual arrows showing robot direction
- **Zone Labels**: Clear labeling of detected ArUcos and zones
- **Dual View**: Both original and bird's eye view with synchronized information

## Installation

### Prerequisites
```bash
# Install ROS (if not already installed)
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get install ros-melodic-image-transport

# Install Python dependencies
pip install -r requirements.txt
```

### Setup
1. Clone or copy the improved code files
2. Make sure your ROS environment is sourced:
   ```bash
   source /opt/ros/melodic/setup.bash
   ```
3. Make the script executable:
   ```bash
   chmod +x visión_cenitalV1_improved.py
   ```

## Usage

### Running the System
```bash
# Start ROS core (in separate terminal)
roscore

# Run the navigation system
python visión_cenitalV1_improved.py
```

### ArUco Marker Setup
The system expects the following ArUco markers:
- **Reference markers (20-23)**: Used for homography calculation
- **Zone markers (9-14)**: Correspond to zones 1-6
- **Robot marker (15)**: Tracks robot position

### Robot Commands
Send navigation commands via ROS topics:

**Move can command:**
```bash
rostopic pub /info_robot_mover_lata std_msgs/String "data: '{\"lata\": 1, \"origen\": 1, \"destino\": 2}'"
```

**Move to can command:**
```bash
rostopic pub /info_robot_mover_a_lata std_msgs/String "data: '{\"origen\": 1, \"destino\": 2}'"
```

## Key Features

### 1. **Automatic Camera Calibration**
- Supports both chessboard and ArUco-based calibration
- Saves/loads calibration parameters automatically
- Applies distortion correction when calibrated

### 2. **Real-time Navigation**
- Calculates optimal robot direction based on current position
- Provides commands: "Recto", "GirarD", "GirarI", "Parar", "Destino"
- Visual feedback with navigation arrows

### 3. **Robust ArUco Detection**
- Optimized detection parameters for various lighting conditions
- Handles multiple ArUco types (zones, reference, robot)
- Error recovery for missing or occluded markers

### 4. **Bird's Eye View**
- Real-time homography calculation using reference markers
- Overhead view for better spatial understanding
- Synchronized visualization between original and transformed views

## Configuration

### Adjustable Parameters
```python
# Frame size
self.frame_size = (640, 480)

# Navigation threshold
destination_threshold = 50  # Distance to consider "arrived"

# World coordinates for homography
self.world_pts = np.array([
    [  600,   600],   # Bottom left
    [  600, 18800],   # Top left  
    [28800,   600],   # Bottom right
    [28800, 18800],   # Top right
], dtype=np.float32)
```

## Troubleshooting

### Common Issues

1. **No ArUco Detection**
   - Check lighting conditions
   - Verify ArUco marker quality and size
   - Adjust detection parameters if needed

2. **Poor Homography**
   - Ensure all 4 reference markers (20-23) are visible
   - Check marker placement and orientation
   - Verify world coordinate mapping

3. **Navigation Issues**
   - Confirm robot marker (ID 15) is detected
   - Check zone marker visibility
   - Verify topic names and message formats

### Debug Information
The system provides extensive logging:
- ArUco detection counts
- Navigation state changes
- Error messages with context
- Real-time status display

## Future Improvements

1. **Advanced Path Planning**: Implement A* or other pathfinding algorithms
2. **Dynamic Zone Detection**: Automatic zone boundary calculation
3. **Multi-Robot Support**: Handle multiple robots simultaneously
4. **Obstacle Avoidance**: Integrate obstacle detection and avoidance
5. **Machine Learning**: Use ML for improved marker detection in challenging conditions

## Dependencies
- ROS Melodic (or compatible)
- OpenCV 4.5+
- NumPy 1.19+
- cv_bridge
- Python 2.7 (ROS Melodic compatibility)

## License
This code is provided as-is for educational and research purposes. 