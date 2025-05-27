# Robot Navigation System - 8-Zone Behavior Documentation

## System Overview

The robot navigation system manages **8 zones** with the following organization:
- **Zones 1-6**: Source zones containing cans that need to be moved
- **Zones 7-8**: Destination storage zones where cans accumulate after being moved

## Zone Configuration

### **Source Zones (1-6)**
```
Zone 1 (ArUco ID 9)  → Contains cans to be moved
Zone 2 (ArUco ID 10) → Contains cans to be moved  
Zone 3 (ArUco ID 11) → Contains cans to be moved
Zone 4 (ArUco ID 12) → Contains cans to be moved
Zone 5 (ArUco ID 13) → Contains cans to be moved
Zone 6 (ArUco ID 14) → Contains cans to be moved
```

### **Destination Storage Zones (7-8)**
```
Zone 7 (ArUco ID 15) → Accumulates moved cans
Zone 8 (ArUco ID 16) → Accumulates moved cans
```

## Robot Operation Modes

### **Mode 1: "mover_lata" (Move Specific Can)**

**Purpose**: Move a specific can from a source zone to a destination zone

**Message Format**:
```json
{
  "lata": 5,      // Specific can ArUco ID
  "origen": 2,    // Source zone (1-6)
  "destino": 7    // Destination zone (7-8)
}
```

**Robot Behavior**:
1. **Navigate to Source Zone**: Robot moves to zone specified in `origen`
2. **Locate Target Can**: Finds can with ArUco ID specified in `lata`
3. **Approach Can**: 
   - Opens gripper when within 80 pixels of target can
   - Closes gripper when within 30 pixels (picks up can)
4. **Navigate to Destination**: Moves to zone specified in `destino`
5. **Drop Can**: 
   - Opens gripper when within 50 pixels of zone center
   - Can remains in destination zone (accumulates)

**Gripper Control Logic**:
```python
# Approaching target can
if distance_to_can <= 80 and gripper_state == 'closed':
    open_gripper()  # Prepare to pick up

# Picking up can  
if distance_to_can <= 30 and gripper_state == 'open':
    close_gripper()  # Grab the can

# At destination zone
if distance_to_destination <= 50 and gripper_state == 'closed':
    open_gripper()  # Drop the can
```

### **Mode 2: "mover_a_lata" (Move to Destination Zone)**

**Purpose**: Navigate to a destination zone (typically carrying a can)

**Message Format**:
```json
{
  "origen": 3,    // Current/source zone
  "destino": 8    // Destination zone (7-8)
}
```

**Robot Behavior**:
1. **Navigate to Destination**: Robot moves to zone center specified in `destino`
2. **Drop Can**: When close to zone center (30 pixels), opens gripper to drop carried can
3. **Can Accumulation**: Dropped can remains in destination zone

**Gripper Control Logic**:
```python
# At destination zone with can
if distance_to_zone_center <= 30 and gripper_state == 'closed':
    open_gripper()  # Drop the can in storage zone
```

## Navigation Algorithm

### **Orientation-Based Navigation**
The system uses **actual robot orientation** calculated from ArUco marker corners:

1. **Orientation Detection**: 
   - Analyzes robot ArUco marker (ID 0) corners
   - Calculates heading direction (0-360°)
   - Creates orientation vector for navigation

2. **Direction Calculation**:
   - Compares current heading with desired direction
   - Determines optimal movement command
   - Supports backward movement for large angle differences

3. **Movement Commands**:
   - `"Recto"`: Move forward (angle difference < 15°)
   - `"GirarD"`: Turn right (clockwise)
   - `"GirarI"`: Turn left (counter-clockwise)  
   - `"Atras"`: Move backward (angle difference > 120°)
   - `"Parar"`: Stop
   - `"Destino"`: Arrived at destination

## Can Accumulation System

### **How Cans Accumulate in Zones 7-8**

**Initial State**:
```
Zone 7: [Zone Marker ID 15]
Zone 8: [Zone Marker ID 16]
```

**After Moving Can ID 5 to Zone 7**:
```
Zone 7: [Zone Marker ID 15, Can ID 5]
Zone 8: [Zone Marker ID 16]
```

**After Moving Can ID 3 to Zone 8**:
```
Zone 7: [Zone Marker ID 15, Can ID 5]
Zone 8: [Zone Marker ID 16, Can ID 3]
```

**After Moving Can ID 7 to Zone 7**:
```
Zone 7: [Zone Marker ID 15, Can ID 5, Can ID 7]
Zone 8: [Zone Marker ID 16, Can ID 3]
```

### **Can Detection in Destination Zones**
The system properly identifies cans vs. zone markers:
```python
# Can detection logic
if aruco_id not in [0, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23]:
    # This is a can (not zone marker, robot, or reference)
    process_can(aruco_id)
```

## Example Workflows

### **Example 1: Move Can 5 from Zone 2 to Zone 7**

**Command**: `{"lata": 5, "origen": 2, "destino": 7}`

**Step-by-Step Process**:
1. Robot receives "mover_lata" command
2. Robot navigates to Zone 2 (ArUco ID 10)
3. Robot locates Can ID 5 in Zone 2
4. Robot approaches Can ID 5:
   - Distance 80px: Opens gripper
   - Distance 30px: Closes gripper (picks up can)
5. Robot navigates to Zone 7 (ArUco ID 15)
6. Robot approaches Zone 7 center:
   - Distance 50px: Opens gripper (drops can)
7. Can ID 5 now resides in Zone 7
8. Robot publishes "estado:completado"

**Result**: Zone 7 contains Can ID 5

### **Example 2: Move Can 3 from Zone 4 to Zone 8**

**Command**: `{"lata": 3, "origen": 4, "destino": 8}`

**Result**: Zone 8 contains Can ID 3

### **Example 3: Multiple Can Accumulation**

**Sequence of Commands**:
```json
{"lata": 1, "origen": 1, "destino": 7}  // Can 1 → Zone 7
{"lata": 2, "origen": 3, "destino": 7}  // Can 2 → Zone 7  
{"lata": 4, "origen": 5, "destino": 8}  // Can 4 → Zone 8
{"lata": 6, "origen": 2, "destino": 8}  // Can 6 → Zone 8
```

**Final State**:
```
Zone 7: [Zone Marker ID 15, Can ID 1, Can ID 2]
Zone 8: [Zone Marker ID 16, Can ID 4, Can ID 6]
```

## System Benefits

### **1. Organized Storage**
- Clear separation between source (1-6) and destination (7-8) zones
- Systematic can accumulation in designated areas
- Easy tracking of moved cans

### **2. Flexible Operations**
- Can move any can from any source zone to any destination zone
- Supports multiple cans in destination zones
- No conflicts between accumulated cans

### **3. Robust Navigation**
- Orientation-based navigation for precise movement
- Automatic gripper control based on distance thresholds
- Comprehensive error handling and fallback methods

### **4. Scalable Design**
- Easy to add more zones (extend ArUco ID range)
- Configurable distance thresholds
- Modular navigation algorithms

## Monitoring and Debugging

### **Visual Feedback**
The system provides real-time visual information:
- Robot position and orientation arrow
- Zone markers and can positions
- Navigation vectors and status
- Gripper state and can pickup status

### **ROS Topics**
```
/info_camara_mover_lata     → Commands to robot (mover_lata mode)
/info_camara_mover_a_lata   → Commands to robot (mover_a_lata mode)
/gripper_control            → Gripper open/close commands
/info_robot_mover_lata      → Status from robot (mover_lata mode)
/info_robot_mover_a_lata    → Status from robot (mover_a_lata mode)
```

### **Log Messages**
```
"Approaching can, opening gripper. Distance: 75.3"
"Can reached, closing gripper. Distance: 25.1"
"Destination reached, dropping can"
"Navigation completed: mover_lata"
```

This system provides a complete solution for organized can management with accumulation in designated storage zones! 