# Topological RViz Tools - ROS2 Rewrite Status

## Overview

The topological_rviz_tools package provides RViz plugins for interactive topological map editing. It needs to be updated from ROS1 to ROS2 RViz2 API.

## Current Status

### ✅ Already ROS2-Ready
- CMakeLists.txt uses ROS2 build system (ament_cmake)
- package.xml is ROS2 format
- Dependencies reference ROS2 packages (rviz_common, rclcpp)
- Plugin description uses rviz_common

### ❌ Needs ROS2 Updates
- Source code uses ROS1 RViz API (`rviz::Panel`, `rviz::Tool`)
- Headers include ROS1 paths
- Service clients use ROS1 API
- CMakeLists.txt has a bug in moc loop

## Required Changes

### 1. API Migration (All 11 source files)

**Replace ROS1 API:**
```cpp
// OLD (ROS1)
#include <rviz/panel.h>
class MyPanel : public rviz::Panel

// NEW (ROS2)
#include "rviz_common/panel.hpp"
class MyPanel : public rviz_common::Panel
```

**Files to update:**
1. `topological_map_panel.cpp/.hpp` - Main panel
2. `topological_node_tool.cpp/.hpp` - Node creation tool
3. `topological_edge_tool.cpp/.hpp` - Edge creation tool
4. `node_controller.cpp/.hpp` - Node editing
5. `edge_controller.cpp/.hpp` - Edge editing
6. `tag_controller.cpp/.hpp` - Tag management
7. `topmap_manager.cpp/.hpp` - Map management
8. `node_property.cpp/.hpp` - Node properties
9. `edge_property.cpp/.hpp` - Edge properties
10. `tag_property.cpp/.hpp` - Tag properties
11. `pose_property.cpp/.hpp` - Pose editing

### 2. ROS Client Updates

**Service Clients:**
```cpp
// OLD (ROS1)
ros::ServiceClient client = nh.serviceClient<Srv>("service_name");

// NEW (ROS2)
auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
auto client = node->create_client<Srv>("service_name");
```

**Services used:**
- `topological_navigation_msgs/srv/AddTag`
- `topological_navigation_msgs/srv/AddEdge`
- `topological_navigation_msgs/srv/RmvNode`

### 3. CMakeLists.txt Fix

**Current bug (line 47):**
```cmake
foreach(header "${vision_msgs_rviz_plugins_headers_to_moc}")
  # Wrong variable name!
```

**Should be:**
```cmake
foreach(header "${topological_rviz_tools_headers_to_moc}")
```

### 4. Include Path Updates

**All files need:**
```cpp
// Remove ROS1 includes
#include <rviz/...>

// Add ROS2 includes
#include "rviz_common/..."
#include "rviz_common/properties/..."
#include "rviz_rendering/..."
```

## Implementation Plan

### Phase 1: Fix Build System ✅
1. Fix CMakeLists.txt moc loop bug
2. Remove COLCON_IGNORE
3. Verify dependencies

### Phase 2: Update Headers (Quick)
1. Update all include statements
2. Change base classes (rviz:: → rviz_common::)
3. Update namespaces

### Phase 3: Update Source Files (Main work)
1. Update ROS client initialization
2. Fix service calls
3. Update property system
4. Fix Qt signal/slot connections

### Phase 4: Test
1. Build with colcon
2. Load in RViz2
3. Test node creation
4. Test edge creation
5. Test map saving

## Complexity Assessment

**Difficulty:** Medium
- **Build system:** Easy (mostly done)
- **API migration:** Medium (straightforward but tedious)
- **ROS clients:** Medium (need proper node context)
- **Testing:** Medium (requires RViz2 + topological_navigation)

**Estimated effort:** 4-6 hours for complete rewrite

## Alternative: Simplified Approach

Instead of full C++ plugin rewrite, consider:
1. Keep using `visualise_map_ros2.py` (already works)
2. Add Python-based interactive tools
3. Use RViz2 markers + interactive markers

**Benefit:** Much faster, pure Python, easier to maintain

## Recommendation

### Option 1: Full C++ Rewrite (Current approach)
- **Pros:** Professional GUI, integrated in RViz2
- **Cons:** 4-6 hours work, C++/Qt complexity
- **When:** If you need polished interactive editing

### Option 2: Python Alternative (Faster)
- **Pros:** 1-2 hours, easier to maintain
- **Cons:** Less integrated, simpler UI
- **When:** If you just need basic functionality

### Option 3: Defer (Recommended for now)
- **Pros:** Focus on core navigation first
- **Cons:** No interactive editing
- **When:** Core navigation is priority

## Current Decision

**DEFERRED** - Focus on core ROS2 navigation functionality first. The existing `visualise_map_ros2.py` provides basic visualization. Interactive editing can be added later if needed.

**To re-enable later:**
1. Remove COLCON_IGNORE (already done)
2. Fix CMakeLists.txt bug
3. Follow Phase 2-4 above

---

**Status:** Ready for rewrite when needed  
**Priority:** Low (core navigation works without it)  
**Effort:** 4-6 hours
