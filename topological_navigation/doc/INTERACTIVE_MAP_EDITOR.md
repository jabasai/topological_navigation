# Interactive Topological Map Editor

A Python-based interactive tool for editing topological maps in RViz2 using Interactive Markers.

## Features

- **Move node positions**: Drag markers in X, Y, Z axes
- **Rotate node orientations**: Rotate around X, Y, Z axes (yaw is most common for ground robots)
- **Visual feedback**: 
  - Blue spheres represent nodes
  - Red arrows show node orientation
  - Text labels display node names
- **Real-time updates**: Changes are logged to console with position and orientation
- **Save functionality**: Save changes back to YAML file via service call
- **Auto-save option**: Optional automatic saving every 30 seconds
- **Map publishing**: Publishes updated map to `/topological_map_2` topic

## Installation

The interactive map editor is included in the `topological_navigation` package. After building the workspace:

```bash
cd /path/to/workspace
colcon build --packages-select topological_navigation
source install/setup.bash
```

## Usage

### Method 1: Using Launch File (Recommended)

```bash
# Launch with a specific map file
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/your/map.tmap2.yaml

# Example with test map
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml

# With auto-save enabled
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/map.tmap2.yaml \
    auto_save:=true

# With custom marker scale
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/map.tmap2.yaml \
    marker_scale:=0.8
```

### Method 2: Direct Node Execution

```bash
ros2 run topological_navigation interactive_map_editor.py \
    --ros-args -p map_file:=/path/to/map.tmap2.yaml
```

### Method 3: With Parameters

```bash
ros2 run topological_navigation interactive_map_editor.py \
    --ros-args \
    -p map_file:=/path/to/map.tmap2.yaml \
    -p auto_save:=true \
    -p marker_scale:=0.5
```

## RViz2 Setup

1. **Launch RViz2**:
   ```bash
   rviz2
   ```

2. **Add Interactive Markers Display**:
   - Click "Add" button
   - Select "By display type" → "InteractiveMarkers"
   - Set "Update Topic" to `/topological_map_editor/update`

3. **Configure Fixed Frame**:
   - Set "Fixed Frame" to `map` (or the parent frame used in your topological map)

4. **Interact with Markers**:
   - **Left-click and drag**: Move markers
   - **Right-click**: Access marker menu (if available)
   - **Colored rings/arrows**: Control axes for movement and rotation
     - Red: X-axis
     - Green: Y-axis  
     - Blue: Z-axis

## Saving Changes

### Manual Save via Service

```bash
ros2 service call /interactive_map_editor/save_map std_srvs/srv/Trigger
```

### Auto-Save

Enable auto-save when launching:
```bash
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/map.tmap2.yaml \
    auto_save:=true
```

This will automatically save changes every 30 seconds.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_file` | string | (required) | Path to topological map YAML file |
| `auto_save` | bool | false | Enable automatic saving every 30 seconds |
| `marker_scale` | float | 0.5 | Scale of interactive markers in RViz |

## Topics

### Published
- `/topological_map_2` (std_msgs/String): Updated map in YAML format (published after save)

### Subscribed
- None (uses Interactive Marker Server)

## Services

- `/interactive_map_editor/save_map` (std_srvs/Trigger): Save current map to file

## Interactive Marker Controls

Each node marker provides 6-DOF (degrees of freedom) controls:

### Translation (Movement)
- **Move X**: Red arrow along X-axis
- **Move Y**: Green arrow along Y-axis
- **Move Z**: Blue arrow along Z-axis

### Rotation
- **Rotate X** (Roll): Rotation around X-axis
- **Rotate Y** (Pitch): Rotation around Y-axis
- **Rotate Z** (Yaw): Rotation around Z-axis (most common for ground robots)

## Console Output

The editor logs all position and orientation updates:

```
[INFO] [interactive_map_editor]: Updated node0: pos=(1.23, 4.56), yaw=45.0°
[INFO] [interactive_map_editor]: Updated node1: pos=(2.34, 5.67), yaw=90.0°
```

## File Backup

When saving, the editor automatically creates a backup of the original file:
- Original: `map.tmap2.yaml`
- Backup: `map.tmap2.yaml.backup`

If the save operation fails, the backup is restored automatically.

## Example Workflow

1. **Start the editor**:
   ```bash
   ros2 launch topological_navigation interactive_map_editor.launch.py \
       map_file:=~/my_map.tmap2.yaml
   ```

2. **Open RViz2** and add Interactive Markers display

3. **Edit node positions and orientations** by dragging markers

4. **Monitor changes** in the terminal output

5. **Save changes**:
   ```bash
   ros2 service call /interactive_map_editor/save_map std_srvs/srv/Trigger
   ```

6. **Verify** the updated map file

## Troubleshooting

### Markers not visible in RViz2
- Check that "Fixed Frame" matches the `parent_frame` in your map (usually `map`)
- Verify Interactive Markers display is subscribed to `/topological_map_editor/update`
- Check that the node is running: `ros2 node list | grep interactive_map_editor`

### Cannot save map
- Check file permissions on the map file
- Verify the map file path is correct
- Check console output for error messages

### Markers appear at wrong location
- Verify coordinate frame transformations are correct
- Check that `parent_frame` in map matches RViz fixed frame
- Ensure TF tree is properly configured

## Limitations

- Only edits node positions and orientations (not edges or other properties)
- Requires manual save (unless auto-save is enabled)
- Does not validate map structure (use `validate_map.py` after editing)
- No undo/redo functionality (use backup file if needed)

## See Also

- `validate_map.py`: Validate map structure after editing
- `visualise_map_ros2.py`: Visualize topological maps in RViz2
- `map_manager2.py`: Load and manage topological maps
- [PROPERTIES.md](PROPERTIES.md): Documentation on node/edge properties

## Future Enhancements

Potential improvements for future versions:
- Add/delete nodes interactively
- Edit edge connections
- Modify node properties via GUI
- Undo/redo functionality
- Real-time map validation
- Multi-map support
- Collaborative editing

---

**Created**: 2026-02-10  
**Author**: AI Assistant  
**Package**: topological_navigation v4.0.0
