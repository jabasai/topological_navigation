# Quick Start: Interactive Map Editor

## What You Got

A Python-based tool to edit topological maps visually in RViz2. You can:
- **Move nodes** by dragging markers
- **Rotate nodes** using rotation controls
- **Save changes** back to YAML files

## Installation

```bash
cd /path/to/workspace
colcon build --packages-select topological_navigation
source install/setup.bash
```

## Usage (3 Simple Steps)

### Step 1: Launch the Editor

```bash
# With test map
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml

# With your own map
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/your/map.tmap2.yaml
```

### Step 2: Open RViz2

In another terminal:
```bash
rviz2
```

In RViz2:
1. Set **Fixed Frame** to `map`
2. Click **Add** → **By display type** → **InteractiveMarkers**
3. Set **Update Topic** to `/topological_map_editor/update`

### Step 3: Edit and Save

- **Drag markers** to move nodes
- **Use colored rings** to rotate nodes (red=X, green=Y, blue=Z)
- **Watch console** for position updates

Save your changes:
```bash
ros2 service call /interactive_map_editor/save_map std_srvs/srv/Trigger
```

## Optional: Auto-Save

Enable automatic saving every 30 seconds:
```bash
ros2 launch topological_navigation interactive_map_editor.launch.py \
    map_file:=/path/to/map.tmap2.yaml \
    auto_save:=true
```

## Files

- **Script**: `topological_navigation/topological_navigation/scripts/interactive_map_editor.py`
- **Launch**: `topological_navigation/launch/interactive_map_editor.launch.py`
- **Full Docs**: `topological_navigation/doc/INTERACTIVE_MAP_EDITOR.md`

## Troubleshooting

**Markers not visible?**
- Check Fixed Frame is set to `map`
- Verify Interactive Markers topic is `/topological_map_editor/update`

**Can't save?**
- Check file permissions on the map file
- Look for error messages in the console

## What's Next?

After editing:
1. Validate your map: `ros2 run topological_navigation validate_map.py /path/to/map.tmap2.yaml`
2. Use it with navigation: `ros2 run topological_navigation map_manager2.py`

---

**Full documentation**: See `topological_navigation/doc/INTERACTIVE_MAP_EDITOR.md`
