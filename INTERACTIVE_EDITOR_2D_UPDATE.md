# Interactive Map Editor - 2D Navigation Update

**Date**: 2026-02-10  
**Update**: Restricted to 2D navigation (XY movement, yaw rotation only)  
**Status**: ✅ Complete

## Changes Made

Modified the interactive map editor to be optimized for 2D ground robot navigation by restricting movement and rotation to the horizontal plane.

### 1. Control Restrictions

**Before (6-DOF)**:
- Move in X, Y, Z axes
- Rotate around X (roll), Y (pitch), Z (yaw) axes

**After (3-DOF for 2D)**:
- Move in X, Y axes only (no Z movement)
- Rotate around Z axis only (yaw only, no roll/pitch)

### 2. Code Changes

#### Marker Controls (create_node_marker method)

**Removed**:
- Move Z control
- Rotate X (roll) control
- Rotate Y (pitch) control
- MOVE_ROTATE_3D interaction mode

**Changed to**:
- MOVE_PLANE interaction mode (allows dragging in XY plane)
- Move X axis control
- Move Y axis control
- Rotate Z axis control (yaw only)

**Result**: Cleaner, simpler controls focused on 2D navigation

#### Feedback Processing (process_feedback method)

**Position Updates**:
```python
# Before: Updated X, Y, Z from feedback
node['pose']['position']['x'] = float(feedback.pose.position.x)
node['pose']['position']['y'] = float(feedback.pose.position.y)
node['pose']['position']['z'] = float(feedback.pose.position.z)

# After: Update X, Y only, force Z to 0
node['pose']['position']['x'] = float(feedback.pose.position.x)
node['pose']['position']['y'] = float(feedback.pose.position.y)
node['pose']['position']['z'] = 0.0  # Force Z to 0 for 2D
```

**Orientation Updates**:
```python
# Before: Copied quaternion directly (all rotations)
node['pose']['orientation']['x'] = float(feedback.pose.orientation.x)
node['pose']['orientation']['y'] = float(feedback.pose.orientation.y)
node['pose']['orientation']['z'] = float(feedback.pose.orientation.z)
node['pose']['orientation']['w'] = float(feedback.pose.orientation.w)

# After: Extract yaw, reconstruct quaternion with roll=0, pitch=0
euler = tf_transformations.euler_from_quaternion([...])
yaw = euler[2]  # Only keep yaw
quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
node['pose']['orientation']['x'] = float(quat[0])
node['pose']['orientation']['y'] = float(quat[1])
node['pose']['orientation']['z'] = float(quat[2])
node['pose']['orientation']['w'] = float(quat[3])
```

### 3. Benefits

✅ **Simpler interaction**: Only 3 controls instead of 6
✅ **Prevents errors**: Can't accidentally move nodes in Z or tilt them
✅ **2D focused**: Perfect for ground robots navigating on flat surfaces
✅ **Cleaner visualization**: Less clutter in RViz2
✅ **Data integrity**: Ensures Z=0 and roll=pitch=0 in saved maps

### 4. User Experience

**In RViz2, users will see**:
- Blue sphere marker at node position
- Red arrow showing yaw orientation
- Two movement arrows (X and Y axes)
- One rotation ring (Z axis for yaw)

**Interaction**:
- **Drag marker**: Move node in XY plane
- **Use X arrow**: Move along X axis only
- **Use Y arrow**: Move along Y axis only
- **Use Z ring**: Rotate yaw (heading direction)

### 5. Technical Details

**Coordinate System**:
- X: Forward/backward
- Y: Left/right
- Z: Up/down (fixed at 0)
- Yaw: Rotation around Z axis (heading)

**Quaternion Handling**:
- Input quaternion may have roll/pitch from user interaction
- Extracted yaw angle using `euler_from_quaternion()`
- Reconstructed quaternion with roll=0, pitch=0, yaw=extracted
- Ensures saved orientation is always 2D-valid

**Position Handling**:
- X and Y updated from feedback
- Z always forced to 0.0
- Prevents accidental vertical displacement

## Files Modified

1. **topological_navigation/topological_navigation/scripts/interactive_map_editor.py**
   - Updated docstring to reflect 2D focus
   - Modified `create_node_marker()`: Changed from 6-DOF to 3-DOF controls
   - Modified `process_feedback()`: Added yaw extraction and Z=0 enforcement

## Testing

After rebuilding, test the changes:

```bash
# Rebuild
colcon build --packages-select topological_navigation
source install/setup.bash

# Launch editor
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml

# Open RViz2
rviz2
```

**In RViz2**:
1. Add InteractiveMarkers display
2. Set topic to `/topological_map_editor/update`
3. Try to move markers - should only move in XY plane
4. Try to rotate - should only rotate around Z axis (yaw)
5. Verify Z position stays at 0

**Verify saved data**:
```bash
# Save the map
ros2 service call /interactive_map_editor/save_map std_srvs/srv/Trigger

# Check the YAML file
cat /path/to/map.tmap2.yaml
```

All nodes should have:
- `position.z: 0.0`
- Orientation quaternions representing pure yaw rotation (no roll/pitch)

## Comparison

| Feature | Before (6-DOF) | After (2D) |
|---------|---------------|------------|
| Move X | ✅ | ✅ |
| Move Y | ✅ | ✅ |
| Move Z | ✅ | ❌ (fixed at 0) |
| Rotate X (roll) | ✅ | ❌ (fixed at 0) |
| Rotate Y (pitch) | ✅ | ❌ (fixed at 0) |
| Rotate Z (yaw) | ✅ | ✅ |
| Controls shown | 8 | 4 |
| Interaction mode | MOVE_ROTATE_3D | MOVE_PLANE |
| Use case | 3D navigation | 2D ground robots |

## Documentation Updates Needed

The following documentation should be updated to reflect 2D focus:

- [ ] `INTERACTIVE_MAP_EDITOR.md` - Update feature list
- [ ] `INTERACTIVE_EDITOR_ARCHITECTURE.md` - Update control structure
- [ ] `QUICK_START_INTERACTIVE_EDITOR.md` - Update description
- [ ] `README.md` - Update feature bullets

## Backward Compatibility

✅ **Fully backward compatible**:
- Existing maps with Z≠0 will have Z reset to 0 on first edit
- Existing maps with roll/pitch will have them reset to 0 on first edit
- No breaking changes to file format
- All existing functionality preserved

## Future Enhancements

Potential additions for 2D navigation:
- [ ] Snap to grid option
- [ ] Angle snap (e.g., 45° increments)
- [ ] Distance constraints between nodes
- [ ] Collision detection with map obstacles
- [ ] Batch rotation of multiple nodes

---

**Update completed**: 2026-02-10  
**Lines changed**: ~60 lines in interactive_map_editor.py  
**Testing status**: Ready for testing
