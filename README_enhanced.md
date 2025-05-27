# Robot Navigation System - Enhanced Version with Gripper Control

## Overview
This enhanced version of the robot navigation system includes advanced navigation commands and automatic gripper control based on distance thresholds. The system now supports backward movement and intelligent gripper operation for can manipulation tasks.

## New Features Added

### 1. **Enhanced Navigation Commands**
The system now supports the following navigation commands:
- **"Recto"**: Move straight forward
- **"Atras"**: Move backward (NEW)
- **"GirarD"**: Turn right
- **"GirarI"**: Turn left
- **"Parar"**: Stop movement
- **"Destino"**: Destination reached

### 2. **Automatic Gripper Control**
- **Distance-based operation**: Gripper opens/closes based on proximity to target cans
- **Task-aware logic**: Different behavior for "mover_lata" vs "mover_a_lata" modes
- **State tracking**: Monitors gripper state and can pickup status

### 3. **Enhanced Robot State Management**
```python
self.robot_state = {
    'lata_actual': None,           # ID of current can
    'almacen_origen': None,        # Origin zone
    'almacen_destino': None,       # Destination zone
    'accion': "Parar",            # Current action
    'modo': None,                 # "mover_lata" or "mover_a_lata"
    'navegando': False,           # Navigation active flag
    'gripper_state': 'closed',    # 'open' or 'closed'
    'approaching_can': False,     # Approaching target flag
    'can_picked': False          # Can pickup status
}
```

## Configuration Parameters

### Distance Thresholds
```python
self.gripper_open_threshold = 80   # Distance to open gripper (pixels)
self.gripper_close_threshold = 30  # Distance to close gripper (pixels)
self.destination_threshold = 50    # Distance to consider arrived
```

These thresholds can be adjusted based on:
- Camera resolution and field of view
- Physical robot dimensions
- Gripper mechanism specifications
- Required precision for your application

## Navigation Logic

### Backward Movement ("Atras")
The robot will move backward in these situations:
1. **Origin Proximity**: When very close to origin and destination is in opposite direction
2. **Large Angle**: When the required turn angle is > 150 degrees
3. **Initial Positioning**: When starting from origin and destination is behind

### Enhanced Direction Calculation
```python
def calculate_robot_direction(self, pos_actual, pos_origen, pos_destino):
```
- **Improved angle thresholds**: More precise turn decisions
- **Backward movement logic**: Handles complex navigation scenarios
- **Origin proximity handling**: Special logic for starting positions

## Gripper Control System

### Automatic Operation Modes

#### Mode 1: "mover_lata" (Moving a Can)
1. **Approach Phase**: 
   - Robot approaches target can
   - Gripper opens when distance ≤ 80 pixels
2. **Pickup Phase**:
   - Gripper closes when distance ≤ 30 pixels
   - Can is marked as "picked"
3. **Transport Phase**:
   - Robot navigates to destination with can
4. **Drop Phase**:
   - Gripper opens when destination is reached
   - Can is marked as "dropped"

#### Mode 2: "mover_a_lata" (Moving to a Can)
1. **Approach Phase**:
   - Robot navigates to target location
   - Gripper opens when distance ≤ 80 pixels
2. **Arrival Phase**:
   - Gripper closes when distance ≤ 30 pixels
   - Task completed

### Gripper Control Functions

#### `control_gripper(action)`
```python
def control_gripper(self, action):
    """
    Controls the robot gripper
    Args:
        action: "open" or "close"
    """
```
- Publishes commands to `/gripper_control` topic
- Prevents duplicate commands
- Updates internal gripper state

#### `get_target_can_position()`
```python
def get_target_can_position(self):
    """
    Gets the position of the target can based on current task
    Returns:
        tuple: (x, y) position of target can, or None if not found
    """
```
- Locates target can based on current mode
- Handles specific can IDs for "mover_lata"
- Finds any available can for "mover_a_lata"

#### `check_gripper_control()`
```python
def check_gripper_control(self):
    """
    Checks if gripper should be opened or closed based on distance to target can
    """
```
- Continuously monitors distance to target
- Triggers gripper actions at appropriate thresholds
- Handles different logic for each operation mode

## ROS Topics

### New Topic Added
- **`/gripper_control`** (Publisher): Sends gripper commands
  - Message format: `"gripper:open"` or `"gripper:close"`

### Existing Topics
- **`/csi_cam_0/image_raw`** (Subscriber): Camera feed
- **`/info_robot_mover_lata`** (Subscriber): Can movement commands
- **`/info_robot_mover_a_lata`** (Subscriber): Robot movement commands
- **`/info_camara_mover_lata`** (Publisher): Navigation responses for can movement
- **`/info_camara_mover_a_lata`** (Publisher): Navigation responses for robot movement

## Usage Examples

### Moving a Specific Can
```bash
# Move can ID 5 from zone 1 to zone 3
rostopic pub /info_robot_mover_lata std_msgs/String "data: '{\"lata\": 5, \"origen\": 1, \"destino\": 3}'"
```

**Expected Behavior:**
1. Robot navigates to zone 1
2. Gripper opens when approaching can 5
3. Gripper closes when reaching can 5
4. Robot transports can to zone 3
5. Gripper opens to drop can at destination

### Moving Robot to a Zone
```bash
# Move robot from zone 2 to zone 4
rostopic pub /info_robot_mover_a_lata std_msgs/String "data: '{\"origen\": 2, \"destino\": 4}'"
```

**Expected Behavior:**
1. Robot navigates from zone 2 to zone 4
2. Gripper opens when approaching zone 4
3. Gripper closes when reaching zone 4

## Visual Feedback

### Enhanced Status Display
The system now shows:
- **Navigation Mode**: Current operation type
- **Action**: Current movement command (including "Atras")
- **Detected Markers**: Number of ArUcos found
- **Gripper State**: Open/Closed status
- **Can Status**: Picked/Free status

### Navigation Visualization
- **Direction Arrows**: Show intended robot movement
- **Zone Labels**: Display ArUco IDs and zone assignments
- **Distance Indicators**: Visual feedback for gripper thresholds

## Troubleshooting

### Gripper Issues
1. **Gripper not responding**:
   - Check `/gripper_control` topic subscription
   - Verify gripper hardware connection
   - Check threshold distances

2. **Premature gripper activation**:
   - Adjust `gripper_open_threshold` value
   - Check camera calibration accuracy
   - Verify ArUco detection quality

3. **Can not being picked**:
   - Adjust `gripper_close_threshold` value
   - Check can ArUco marker visibility
   - Verify robot positioning accuracy

### Navigation Issues
1. **Robot not moving backward**:
   - Check "Atras" command implementation in robot controller
   - Verify navigation logic thresholds
   - Check origin proximity detection

2. **Erratic movement**:
   - Adjust angle thresholds in direction calculation
   - Check ArUco detection stability
   - Verify homography calculation

## Hardware Requirements

### Robot Controller
Must support the following commands:
- `accion:Recto`
- `accion:Atras` (NEW)
- `accion:GirarD`
- `accion:GirarI`
- `accion:Parar`
- `accion:Destino`

### Gripper Controller
Must support:
- `gripper:open`
- `gripper:close`

## Future Enhancements

1. **Adaptive Thresholds**: Dynamic adjustment based on can size
2. **Multi-Can Handling**: Support for multiple cans in single operation
3. **Collision Avoidance**: Enhanced navigation with obstacle detection
4. **Force Feedback**: Gripper force sensing for better pickup confirmation
5. **Path Planning**: Advanced algorithms for optimal route calculation

## Dependencies
- ROS Melodic (or compatible)
- OpenCV 3.1+ (Jetson Nano compatible)
- NumPy 1.16.6 (Python 2.7 compatible)
- cv_bridge
- Python 2.7

This enhanced system provides robust, intelligent robot navigation with automatic gripper control, making it suitable for industrial automation and research applications requiring precise can manipulation. 