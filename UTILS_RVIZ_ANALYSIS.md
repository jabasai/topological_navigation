# Topological Utils & RViz Tools Analysis

## Overview

Analysis of `topological_utils` and `topological_rviz_tools` packages to determine which scripts are used and which can be deleted.

## Executive Summary

### topological_utils
- **Total Scripts:** 43 files
- **ROS1 Scripts:** 30+ (use rospy, MongoDB)
- **ROS2/Standalone Scripts:** ~10 (no ROS dependency or minimal)
- **Recommendation:** **DELETE ENTIRE PACKAGE** - Mostly legacy ROS1 utilities for MongoDB-based map management

### topological_rviz_tools  
- **Status:** Has `COLCON_IGNORE` file (already disabled)
- **Type:** C++ RViz plugin for interactive map editing
- **ROS Version:** ROS1 (uses old RViz API)
- **Recommendation:** **KEEP BUT DISABLED** - May be useful if ported to ROS2 RViz2

---

## Detailed Analysis: topological_utils

### Package Purpose
Utility scripts for topological map management, mostly designed for MongoDB-based map storage (legacy ROS1 approach).

### Setup.py Issues
The setup.py has **BROKEN entry points** - references `topological_navigation_utils` but package is named `topological_utils`:

```python
# WRONG - package doesn't exist
'add_content.py = topological_navigation_utils.add_content:main.py'

# Should be
'add_content.py = topological_utils.scripts.add_content:main'
```

**This means NONE of the entry points work!**

### Script Categories

#### Category 1: MongoDB/ROS1 Scripts (DELETE - 30 files)
These require MongoDB and ROS1, not compatible with current ROS2 system:

```
✗ add_content.py          - Add content to MongoDB
✗ add_edge.py             - Add edge to MongoDB map
✗ add_node.py             - Add node to MongoDB map
✗ add_node_tags.py        - Add tags to MongoDB nodes
✗ check_map               - Check MongoDB map
✗ crop_map.py             - Crop MongoDB map
✗ dummy_topological_navigation.py - ROS1 dummy nav
✗ draw_predicted_map.py   - Draw predictions (ROS1)
✗ edge_length_analysis.py - Analyze edges (ROS1)
✗ evaluate_predictions.py - Evaluate predictions (ROS1)
✗ insert_empty_map.py     - Insert to MongoDB
✗ insert_map.py           - Insert to MongoDB
✗ joy_add_node.py         - Joystick node adding (ROS1)
✗ joy_add_waypoint.py     - Joystick waypoint (ROS1)
✗ list_maps               - List MongoDB maps
✗ load_yaml_map.py        - Load YAML to MongoDB (ROS1)
✗ load_json_map.py        - Load JSON to MongoDB
✗ map_collection_change.py - Change MongoDB collection
✗ map_export.py           - Export from MongoDB (ROS1)
✗ map_to_json.py          - Convert map to JSON (ROS1)
✗ map_to_yaml.py          - Convert map to YAML (ROS1)
✗ map_converter.py        - Convert maps (ROS1)
✗ migrate.py              - Migrate MongoDB maps
✗ node_rm.py              - Remove node from MongoDB
✗ node_metadata.py        - Node metadata (ROS1)
✗ print_nav_stats.py      - Print nav stats (ROS1)
✗ rm_map_from_db.py       - Remove map from MongoDB
✗ remove_node_tags.py     - Remove tags (ROS1)
✗ rename_node             - Rename node in MongoDB
✗ topological_map_update.py - Update MongoDB map (ROS1)
✗ visualise_map.py        - Visualize map (ROS1)
```

#### Category 2: Standalone/Useful Scripts (KEEP - 6 files)
These don't require ROS and work with YAML files:

```
✓ tmap_to_yaml.py         - Convert .tmap to YAML (standalone)
✓ waypoints_to_yaml_tmap.py - Convert waypoints to YAML (standalone)
✓ plot_yaml.py            - Plot YAML map (standalone)
✓ plot_yaml2.py           - Plot YAML map v2 (standalone)
✓ plot_topo_map2.py       - Plot topo map (uses rospy but could be standalone)
✓ edge_reconf_groups_to_tmap2.py - Convert reconfig to tmap2
```

#### Category 3: Utility Libraries (KEEP - 2 files)
```
✓ topological_utils/nodes.py   - Node utilities (if ROS1 removed)
✓ topological_utils/queries.py - Query utilities (if ROS1 removed)
```

#### Category 4: Unknown/Not in Entry Points (CHECK - 5 files)
```
? goal_converter.py       - Not in setup.py
? toponav_tool.py         - Not in setup.py  
? plot_topo_map.py        - Not in setup.py (ROS1)
? plot_topo_map2.py       - Not in setup.py (has rospy)
? __init__,py             - Typo file (should be __init__.py)
```

### Launch Files (DELETE - All ROS1)
```
✗ create_topological_map.launch
✗ dummy_topological_navigation.launch
✗ empty_topological_map.launch
✗ mapping.launch
✗ topological_map_edition.launch
✗ topological_prediction_test.launch
```

---

## Detailed Analysis: topological_rviz_tools

### Package Purpose
RViz plugin for interactive topological map editing (C++ Qt-based GUI).

### Status
- **COLCON_IGNORE present** - Package is already disabled
- **ROS Version:** ROS1 (uses old RViz API)
- **Language:** C++ with Qt
- **Functionality:** Interactive map editing in RViz

### Components

#### C++ Source Files (10 files)
```
- edge_controller.cpp         - Edge editing controller
- edge_property.cpp           - Edge property panel
- node_controller.cpp         - Node editing controller
- node_property.cpp           - Node property panel
- pose_property.cpp           - Pose property editor
- tag_controller.cpp          - Tag editing controller
- tag_property.cpp            - Tag property panel
- topmap_manager.cpp          - Map management
- topological_edge_tool.cpp   - Edge creation tool
- topological_map_panel.cpp   - Main RViz panel
- topological_node_tool.cpp   - Node creation tool
```

#### Python Interface (1 file)
```
- python_topmap_interface.py  - Python interface to C++ tools
```

### Why It's Disabled
1. **ROS1 RViz API** - Uses old RViz plugin API
2. **Not ported to ROS2** - Would require significant rewrite for RViz2
3. **Alternative exists** - `visualise_map_ros2.py` provides similar functionality

### Recommendation
**KEEP BUT DISABLED** - May be useful reference if someone wants to port to ROS2 RViz2 in the future.

---

## Recommendations

### Option 1: Complete Cleanup (Recommended)
**Delete topological_utils entirely:**
- 30+ ROS1/MongoDB scripts are obsolete
- Entry points are broken anyway
- 6 useful scripts can be moved to topological_navigation if needed
- Reduces maintenance burden

**Keep topological_rviz_tools disabled:**
- Already has COLCON_IGNORE
- May be useful for future ROS2 port
- Not causing any issues being disabled

### Option 2: Selective Cleanup
**Keep only useful scripts from topological_utils:**
1. Move these 6 scripts to `topological_navigation/scripts/utils/`:
   - `tmap_to_yaml.py`
   - `waypoints_to_yaml_tmap.py`
   - `plot_yaml.py`
   - `plot_yaml2.py`
   - `edge_reconf_groups_to_tmap2.py`

2. Delete the rest of topological_utils

3. Keep topological_rviz_tools disabled

### Option 3: Minimal (Keep Everything Disabled)
- Add `COLCON_IGNORE` to topological_utils
- Keep topological_rviz_tools with existing `COLCON_IGNORE`
- No deletion, just disable both packages

---

## Impact Analysis

### If topological_utils is deleted:
- ✅ No impact on core navigation (not used by navigation2.py, localisation2.py, etc.)
- ✅ No impact on ROS2 functionality
- ✅ Reduces codebase by ~43 files
- ⚠️ Lose some useful standalone plotting/conversion tools
- ⚠️ Would need to recreate if YAML conversion tools needed

### If topological_rviz_tools is deleted:
- ✅ No impact (already disabled with COLCON_IGNORE)
- ✅ Reduces codebase by ~15 files
- ⚠️ Lose reference implementation for future RViz2 port
- ⚠️ Interactive map editing would only be via visualise_map_ros2.py

---

## File Count Summary

### topological_utils
- **Scripts:** 43 files
- **Libraries:** 2 files
- **Launch files:** 6 files
- **Total:** 51 files

### topological_rviz_tools
- **C++ sources:** 11 files
- **Python scripts:** 1 file
- **Headers:** ~10 files
- **Total:** ~22 files

### Combined
- **Total files:** ~73 files
- **Recommendation:** Delete or disable all

---

## Detailed Recommendations by File

### topological_utils - DELETE List

#### High Priority Delete (ROS1 + MongoDB - 30 files)
```bash
rm topological_utils/topological_utils/scripts/add_content.py
rm topological_utils/topological_utils/scripts/add_edge.py
rm topological_utils/topological_utils/scripts/add_node.py
rm topological_utils/topological_utils/scripts/add_node_tags.py
rm topological_utils/topological_utils/scripts/check_map
rm topological_utils/topological_utils/scripts/crop_map.py
rm topological_utils/topological_utils/scripts/dummy_topological_navigation.py
rm topological_utils/topological_utils/scripts/draw_predicted_map.py
rm topological_utils/topological_utils/scripts/edge_length_analysis.py
rm topological_utils/topological_utils/scripts/evaluate_predictions.py
rm topological_utils/topological_utils/scripts/insert_empty_map.py
rm topological_utils/topological_utils/scripts/insert_map.py
rm topological_utils/topological_utils/scripts/joy_add_node.py
rm topological_utils/topological_utils/scripts/joy_add_waypoint.py
rm topological_utils/topological_utils/scripts/list_maps
rm topological_utils/topological_utils/scripts/load_yaml_map.py
rm topological_utils/topological_utils/scripts/load_json_map.py
rm topological_utils/topological_utils/scripts/map_collection_change.py
rm topological_utils/topological_utils/scripts/map_export.py
rm topological_utils/topological_utils/scripts/map_to_json.py
rm topological_utils/topological_utils/scripts/map_to_yaml.py
rm topological_utils/topological_utils/scripts/map_converter.py
rm topological_utils/topological_utils/scripts/migrate.py
rm topological_utils/topological_utils/scripts/node_rm.py
rm topological_utils/topological_utils/scripts/node_metadata.py
rm topological_utils/topological_utils/scripts/print_nav_stats.py
rm topological_utils/topological_utils/scripts/rm_map_from_db.py
rm topological_utils/topological_utils/scripts/remove_node_tags.py
rm topological_utils/topological_utils/scripts/rename_node
rm topological_utils/topological_utils/scripts/topological_map_update.py
rm topological_utils/topological_utils/scripts/visualise_map.py
rm topological_utils/topological_utils/scripts/plot_topo_map.py
```

#### Consider Keeping (Standalone - 6 files)
```bash
# These could be useful for YAML map manipulation
topological_utils/topological_utils/scripts/tmap_to_yaml.py
topological_utils/topological_utils/scripts/waypoints_to_yaml_tmap.py
topological_utils/topological_utils/scripts/plot_yaml.py
topological_utils/topological_utils/scripts/plot_yaml2.py
topological_utils/topological_utils/scripts/plot_topo_map2.py
topological_utils/topological_utils/scripts/edge_reconf_groups_to_tmap2.py
```

---

## Final Recommendation

### Immediate Action (Recommended)
1. **Add COLCON_IGNORE to topological_utils:**
   ```bash
   touch topological_utils/COLCON_IGNORE
   ```

2. **Keep topological_rviz_tools disabled** (already has COLCON_IGNORE)

3. **Document the decision** in README

### Future Action (Optional)
1. Extract 6 useful standalone scripts to topological_navigation
2. Delete both packages entirely
3. Update documentation

---

## Summary Table

| Package | Files | ROS1 | ROS2 | Standalone | Recommendation |
|---------|-------|------|------|------------|----------------|
| topological_utils | 51 | 30+ | 0 | 6 | **DELETE or DISABLE** |
| topological_rviz_tools | 22 | Yes | No | No | **KEEP DISABLED** |
| **Total** | **73** | **30+** | **0** | **6** | **Disable both** |

---

**Analysis Date:** February 10, 2026  
**Status:** Ready for cleanup decision
