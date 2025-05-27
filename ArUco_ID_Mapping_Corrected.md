# ArUco ID Mapping - Corrected for 8-Zone System

## Overview
This document explains the **corrected ArUco ID mapping** for the robot navigation system that supports **8 zones** (6 can storage zones + 2 empty destination zones).

## ❌ Previous Incorrect Mapping
```
IDs 9-14: Zone markers (zones 1-6) ← WRONG! Only 6 zones
ID 15: Robot marker ← CONFLICTS with zone 7!
IDs 20-23: Reference markers for homography
```

## ✅ Corrected Mapping for 8 Zones

### **Zone Markers (8 zones total)**
```
ArUco ID 9  → Zone 1 (Can storage)
ArUco ID 10 → Zone 2 (Can storage)  
ArUco ID 11 → Zone 3 (Can storage)
ArUco ID 12 → Zone 4 (Can storage)
ArUco ID 13 → Zone 5 (Can storage)
ArUco ID 14 → Zone 6 (Can storage)
ArUco ID 15 → Zone 7 (Empty destination) ← FIXED!
ArUco ID 16 → Zone 8 (Empty destination) ← ADDED!
```

### **Robot Marker**
```
ArUco ID 0 → Robot position and orientation
```

### **Reference Markers (Homography)**
```
ArUco ID 20 → Reference point 1 (Bottom left)
ArUco ID 21 → Reference point 2 (Top left)  
ArUco ID 22 → Reference point 3 (Bottom right)
ArUco ID 23 → Reference point 4 (Top right)
```

### **Can Markers**
```
ArUco IDs 1-8, 17-19, 24-49 → Individual cans
(Any ID not used for zones, robot, or reference)
```

## Zone Functionality

### **Zones 1-6: Source Can Storage Zones**
- **Purpose**: Store cans that need to be moved
- **Content**: Contains cans with various ArUco IDs
- **Robot Action**: Pick up cans from these zones

### **Zones 7-8: Destination Can Storage Zones**  
- **Purpose**: Store cans that are moved from zones 1-6
- **Content**: Initially empty, then filled with cans moved by robot
- **Robot Action**: Drop cans in these zones (cans accumulate here)

## Code Implementation

### **Zone Detection Logic**
```python
# Zone ArUcos (9-16 correspond to zones 1-8)
if 9 <= id_ <= 16:
    zona = id_ - 8  # ID 9→Zone 1, ID 10→Zone 2, ..., ID 16→Zone 8
    # Zones 1-6: Source can storage zones (with cans to be moved)
    # Zones 7-8: Destination can storage zones (where robot deposits cans)
```

### **Robot Detection Logic**
```python
# Robot ArUco (ID 0 - no conflict with zones)
elif id_ == self.robot_id:  # self.robot_id = 0
    # Process robot position and orientation
```

### **Can Detection Logic**
```python
# Exclude zone markers (9-16), reference markers (20-23), and robot marker (0)
if aruco['aruco_id'] not in [0, 9, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23]:
    # This is a can marker (can be in any zone)
```

## Physical Setup Requirements

### **Zone Marker Placement**
```
Physical Layout (overhead view):

[Zone 1] [Zone 2] [Zone 3]
  ID 9     ID 10    ID 11
  🥫🥫     🥫🥫     🥫🥫
(Source) (Source) (Source)

[Zone 4] [Zone 5] [Zone 6]  
  ID 12    ID 13    ID 14
  🥫🥫     🥫🥫     🥫🥫
(Source) (Source) (Source)

[Zone 7] [Zone 8]
  ID 15    ID 16
  📦🥫     📦🥫
(Dest.)  (Dest.)
```

### **Reference Marker Placement**
```
ID 21 ●────────────● ID 23
      │            │
      │   Camera   │
      │   Field    │
      │   of View  │
      │            │
ID 20 ●────────────● ID 22
```

### **Robot Marker**
- **ArUco ID 0** mounted on top of robot
- **Orientation**: Top edge of marker = robot forward direction
- **Size**: Large enough for reliable detection from overhead camera

## Navigation Examples

### **Example 1: Move Can from Zone 2 to Zone 7**
```json
{
  "lata": 5,      // Can with ArUco ID 5
  "origen": 2,    // Zone 2 (ArUco ID 10)
  "destino": 7    // Zone 7 (ArUco ID 15) - Destination storage
}
```

**Process**:
1. Robot navigates to Zone 2 (ArUco ID 10)
2. Finds can with ArUco ID 5
3. Picks up can
4. Navigates to Zone 7 (ArUco ID 15)
5. Drops can in destination storage zone (can stays there)

### **Example 2: Move Can from Zone 4 to Zone 8**
```json
{
  "lata": 3,      // Can with ArUco ID 3  
  "origen": 4,    // Zone 4 (ArUco ID 12)
  "destino": 8    // Zone 8 (ArUco ID 16) - Destination storage
}
```

**Result**: Zone 8 now contains can with ArUco ID 3

## Benefits of Corrected Mapping

### **1. No ID Conflicts**
- Robot ID 0 doesn't conflict with any zone
- All 8 zones have unique IDs
- Clear separation between different marker types

### **2. Logical Zone Organization**
- Zones 1-6: Source zones with cans to be moved
- Zones 7-8: Destination zones that accumulate moved cans
- Easy to understand and maintain

### **3. Scalable Design**
- Can easily add more zones (ID 17, 18, etc.)
- Can add more reference points if needed
- Flexible can ID assignment

## Migration from Old System

### **Required Changes**:
1. **Update Zone Mapping**: Change `if 9 <= id_ <= 14:` to `if 9 <= id_ <= 16:`
2. **Change Robot ID**: From 15 to 0 (already done)
3. **Update Can Detection**: Exclude IDs 0, 9-16, 20-23
4. **Physical Setup**: Add ArUco markers 15 and 16 for zones 7-8

### **Backward Compatibility**:
- Zones 1-6 remain unchanged
- Reference markers (20-23) unchanged  
- Only additions: zones 7-8 and robot ID change

## Testing Checklist

### **Zone Detection Test**:
- [ ] Zone 1 (ID 9) detected correctly
- [ ] Zone 2 (ID 10) detected correctly
- [ ] Zone 3 (ID 11) detected correctly
- [ ] Zone 4 (ID 12) detected correctly
- [ ] Zone 5 (ID 13) detected correctly
- [ ] Zone 6 (ID 14) detected correctly
- [ ] Zone 7 (ID 15) detected correctly ← NEW
- [ ] Zone 8 (ID 16) detected correctly ← NEW

### **Robot Detection Test**:
- [ ] Robot (ID 0) detected correctly
- [ ] Robot orientation calculated correctly
- [ ] No conflict with zone markers

### **Navigation Test**:
- [ ] Can move from zones 1-6 to zones 7-8
- [ ] Proper gripper control in destination zones
- [ ] Correct path planning to storage zones
- [ ] Cans accumulate properly in zones 7-8

### **Can Accumulation Test**:
- [ ] Multiple cans can be stored in zone 7
- [ ] Multiple cans can be stored in zone 8
- [ ] System tracks cans in destination zones
- [ ] No conflicts between accumulated cans

This corrected mapping ensures proper functionality for the complete 8-zone robot navigation system with can accumulation in destination zones! 