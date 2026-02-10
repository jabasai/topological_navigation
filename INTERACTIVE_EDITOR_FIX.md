# Interactive Map Editor - Bug Fix

**Date**: 2026-02-10  
**Issue**: InteractiveMarkerServer API error  
**Status**: ✅ Fixed

## Problem

When launching the interactive map editor, it crashed with:

```
Error: InteractiveMarkerServer.insert() takes 2 positional arguments but 3 were given
```

## Root Cause

The ROS2 `InteractiveMarkerServer` API is different from ROS1:

**ROS1 (incorrect for ROS2)**:
```python
self.server.insert(int_marker, self.process_feedback)
```

**ROS2 (correct)**:
```python
self.server.insert(int_marker)
self.server.setCallback(int_marker.name, self.process_feedback)
```

## Solution

Changed the marker insertion code in `create_node_marker()` method:

**Before**:
```python
# Insert marker and set callback
self.server.insert(int_marker, self.process_feedback)
```

**After**:
```python
# Insert marker and set callback
self.server.insert(int_marker)
self.server.setCallback(int_marker.name, self.process_feedback)
```

## Files Modified

- `topological_navigation/topological_navigation/scripts/interactive_map_editor.py`
  - Line ~285: Split `insert()` and `setCallback()` calls

## Testing

After the fix, the editor should launch successfully:

```bash
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml
```

Expected output:
```
[INFO] [interactive_map_editor]: Loaded map from /path/to/test_simple_tmap2.yaml
[INFO] [interactive_map_editor]: Interactive Map Editor started for: /path/to/test_simple_tmap2.yaml
[INFO] [interactive_map_editor]: Loaded 5 nodes
[INFO] [interactive_map_editor]: Use RViz2 Interactive Markers to edit node positions and orientations
[INFO] [interactive_map_editor]: Call service /interactive_map_editor/save_map to save changes
```

## ROS2 API Reference

The correct ROS2 InteractiveMarkerServer API pattern:

```python
from interactive_markers import InteractiveMarkerServer

# Create server
server = InteractiveMarkerServer(node, 'server_name')

# Create marker
marker = InteractiveMarker()
marker.name = "my_marker"
# ... configure marker ...

# Insert marker (no callback parameter)
server.insert(marker)

# Set callback separately
server.setCallback(marker.name, callback_function)

# Apply changes to publish
server.applyChanges()
```

## Verification

Run these commands to verify the fix:

```bash
# 1. Rebuild
colcon build --packages-select topological_navigation
source install/setup.bash

# 2. Launch editor
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml

# 3. Check node is running
ros2 node list | grep interactive_map_editor

# 4. Check topics
ros2 topic list | grep topological_map_editor

# Expected topics:
# /topological_map_editor/feedback
# /topological_map_editor/update
```

## Status

✅ **Fixed and ready for testing**

The interactive map editor should now work correctly with ROS2.

---

**Fix applied**: 2026-02-10  
**Lines changed**: 2 lines in interactive_map_editor.py
