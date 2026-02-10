# ROS Version Analysis and Call Graph for Topological Navigation

## Executive Summary

This document provides a comprehensive analysis of the topological_navigation codebase, categorizing scripts by ROS version (ROS1 vs ROS2) and documenting the ROS2 system call graph.

## 1. ROS Version Classification

### 1.1 ROS2 Scripts (using rclpy)

#### Core Navigation Scripts (Active in ROS2)
1. **navigation2.py** - Main topological navigation action server
2. **localisation2.py** - Topological localization node
3. **map_manager2.py** - Map loading and publishing
4. **get_simple_policy2.py** - Route planning service

#### Visualization and Tools
5. **visualise_map_ros2.py** - RViz visualization for ROS2
6. **topological_visual.py** - Topological visualization
7. **occupancy_checker.py** - Node occupancy checking
8. **topological_transform_publisher.py** - TF publisher for topological map
9. **manual_topomapping.py** - Manual map creation tool

#### Supporting Scripts
10. **param_processing.py** - Parameter handling utilities (ROS2 specific)

### 1.2 ROS1 Scripts (using rospy)

#### Legacy Navigation Scripts
1. **navigation.py** - ROS1 navigation server
2. **localisation.py** - ROS1 localization
3. **map_manager.py** - ROS1 map manager
4. **get_simple_policy.py** - ROS1 route planning

#### Prediction and Statistics
5. **topological_prediction.py** - Travel time prediction
6. **mean_based_prediction.py** - Mean-based prediction
7. **speed_based_prediction.py** - Speed-based prediction
8. **manual_edge_predictions.py** - Manual prediction configuration
9. **evaluate_top_pred.py** - Prediction evaluation

#### Utilities and Tools
10. **visualise_map.py** - ROS1 visualization
11. **visualise_map2.py** - ROS1 visualization (variant)
12. **map_publisher.py** - Map publishing utility
13. **search_route.py** - Route search utility
14. **travel_time_estimator.py** - Travel time estimation
15. **restrictions_manager.py** - Navigation restrictions
16. **reconf_at_edges_server.py** - Edge reconfiguration
17. **nav_client.py** - Navigation action client

### 1.3 Core Library Files

#### ROS2 Libraries
- **manager2.py** - Core map management (ROS2)
- **route_search2.py** - A* path planning (ROS2)
- **edge_action_manager2.py** - Edge action execution (ROS2)
- **edge_reconfigure_manager2.py** - Edge reconfiguration (ROS2)
- **topomap_marker2.py** - Map markers (ROS2)
- **policy_marker2.py** - Policy visualization (ROS2)
- **goal_builder.py** - Navigation goal construction (ROS2)
- **row_operation_handler.py** - Agricultural row operations (ROS2)

#### ROS1 Libraries
- **manager.py** - Map management (ROS1)
- **route_search.py** - Path planning (ROS1)
- **edge_action_manager.py** - Edge actions (ROS1)
- **edge_reconfigure_manager.py** - Edge reconfiguration (ROS1)
- **topological_map.py** - Map data structures (ROS1)
- **load_maps_from_yaml.py** - YAML map loader (ROS1)
- **policies.py** - Navigation policies (ROS1)
- **restrictions_impl.py** - Restrictions implementation (ROS1)
- **topomap_marker.py** - Map markers (ROS1)
- **policy_marker.py** - Policy markers (ROS1)
- **node_controller.py** - Node control (ROS1)
- **edge_controller.py** - Edge control (ROS1)
- **vertex_controller.py** - Vertex control (ROS1)
- **node_manager.py** - Node management (ROS1)
- **goto.py** - Navigation goto (ROS1)
- **edge_std.py** - Standard edge operations (ROS1)
- **marker_arrays.py** - Marker array utilities (ROS1)
- **publisher.py** - Publishing utilities (ROS1)
- **testing.py** - Testing utilities (ROS1)

#### Shared/Utility Libraries (No ROS dependency)
- **tmap_utils.py** - Map utility functions
- **point2line.py** - Geometric calculations
- **navigation_stats.py** - Navigation statistics
- **map_types.py** - Map type definitions
- **actions_bt.py** - Behavior tree action types

## 2. ROS2 System Call Graph

### 2.1 Entry Points (from setup.py)

The following scripts are registered as executable entry points in ROS2:

```
Core ROS2 Nodes:
├── navigation2.py          (topological_navigation action server)
├── localisation2.py        (topological localization)
├── map_manager2.py         (map loading and publishing)
└── get_simple_policy2.py   (route planning services)

Visualization:
├── visualise_map_ros2.py   (RViz interactive visualization)
├── topological_visual.py   (topological route visualization)
├── topomap_marker2.py      (map marker publishing)
└── policy_marker2.py       (policy visualization)

Utilities:
├── occupancy_checker.py    (node occupancy monitoring)
├── topological_transform_publisher.py (TF publishing)
├── manual_topomapping.py   (manual map creation)
└── validate_map.py         (map validation)
```

### 2.2 Core ROS2 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Topological Navigation              │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐
│  map_manager2.py │  Entry Point
└────────┬─────────┘
         │
         ├─> manager2.py (Core map management)
         │   ├─> load_maps_from_yaml.py (YAML loading)
         │   ├─> map_types.py (Type definitions)
         │   └─> tmap_utils.py (Utilities)
         │
         └─> Publishes: /topological_map_2 (String - YAML)

┌────────────────────┐
│ localisation2.py   │  Entry Point
└────────┬───────────┘
         │
         ├─> Subscribes: /topological_map_2
         ├─> Subscribes: TF (map -> base_link)
         ├─> Uses: tmap_utils.py, point2line.py
         │
         └─> Publishes:
             ├─> /current_node (String)
             ├─> /closest_node (String)
             ├─> /closest_edges (ClosestEdges)
             └─> /current_node/tag (String)

┌────────────────────┐
│  navigation2.py    │  Entry Point (Main Navigation Server)
└────────┬───────────┘
         │
         ├─> Subscribes:
         │   ├─> /topological_map_2
         │   ├─> /current_node
         │   ├─> /closest_node
         │   └─> /closest_edges
         │
         ├─> Action Servers:
         │   ├─> /topological_navigation (GotoNode)
         │   └─> /topological_navigation/execute_policy_mode
         │
         ├─> Core Dependencies:
         │   ├─> route_search2.py (A* path planning)
         │   │   └─> TopologicalRouteSearch2 class
         │   │
         │   ├─> edge_action_manager2.py (Edge execution)
         │   │   ├─> EdgeActionManager class
         │   │   ├─> Nav2 action clients:
         │   │   │   ├─> NavigateToPose
         │   │   │   ├─> NavigateThroughPoses
         │   │   │   └─> FollowWaypoints
         │   │   ├─> goal_builder.py (Goal construction)
         │   │   └─> row_operation_handler.py (Agricultural ops)
         │   │
         │   ├─> edge_reconfigure_manager2.py (Parameter updates)
         │   │   └─> param_processing.py (Parameter handling)
         │   │
         │   ├─> navigation_stats.py (Statistics tracking)
         │   └─> actions_bt.py (Behavior tree types)
         │
         └─> Publishes:
             ├─> /topological_navigation/Statistics
             ├─> /topological_navigation/Route
             ├─> /current_edge
             └─> /topological_navigation/move_action_status

┌──────────────────────────┐
│ get_simple_policy2.py    │  Entry Point
└────────┬─────────────────┘
         │
         ├─> Subscribes: /topological_map_2
         ├─> Uses: route_search2.py
         │
         └─> Services:
             ├─> /topological_navigation/get_route_to
             └─> /topological_navigation/get_route_between
```

### 2.3 Visualization Components

```
┌──────────────────────────┐
│ visualise_map_ros2.py    │  Interactive RViz Tool
└────────┬─────────────────┘
         │
         ├─> Subscribes:
         │   ├─> /topological_map_2
         │   ├─> /topological_navigation/Route
         │   └─> /current_node
         │
         ├─> Action Clients:
         │   └─> /topological_navigation (GotoNode)
         │
         └─> Publishes:
             └─> Interactive markers for map editing

┌──────────────────────────┐
│ topological_visual.py    │  Route Visualization
└────────┬─────────────────┘
         │
         ├─> Subscribes:
         │   ├─> /topological_navigation/Route
         │   └─> /topological_occupied_nodes
         │
         └─> Publishes:
             └─> /topological_route_markers (MarkerArray)

┌──────────────────────────┐
│ topomap_marker2.py       │  Map Marker Publisher
└────────┬─────────────────┘
         │
         ├─> Subscribes: /topological_map_2
         │
         └─> Publishes:
             └─> /topological_map_markers (MarkerArray)
```

### 2.4 Supporting Utilities

```
┌──────────────────────────────┐
│ occupancy_checker.py         │  Node Occupancy Monitor
└────────┬─────────────────────┘
         │
         ├─> Subscribes:
         │   ├─> /topological_map_2
         │   └─> /robot_poses (PoseArray)
         │
         └─> Publishes:
             └─> /topological_occupied_nodes

┌──────────────────────────────────────┐
│ topological_transform_publisher.py   │  TF Publisher
└────────┬─────────────────────────────┘
         │
         ├─> Subscribes: /topological_map_2
         │
         └─> Publishes: TF transforms for nodes
```

## 3. Active vs Inactive Scripts

### 3.1 Core Active ROS2 Scripts
These are essential for basic topological navigation:

1. **navigation2.py** - Main navigation server (ACTIVE)
2. **localisation2.py** - Localization (ACTIVE)
3. **map_manager2.py** - Map management (ACTIVE)
4. **get_simple_policy2.py** - Route planning (ACTIVE)

### 3.2 Active Supporting Scripts
Used for visualization and utilities:

1. **visualise_map_ros2.py** - Interactive map editing (ACTIVE)
2. **topomap_marker2.py** - Map visualization (ACTIVE)
3. **topological_visual.py** - Route visualization (ACTIVE)
4. **occupancy_checker.py** - Multi-robot support (ACTIVE)

### 3.3 Potentially Inactive/Legacy Scripts
These are registered but may not be actively used:

1. **map_manager.py** - ROS1 version (LEGACY - use map_manager2.py)
2. **navigation.py** - ROS1 version (LEGACY - use navigation2.py)
3. **visualise_map.py** - ROS1 version (LEGACY)
4. **visualise_map2.py** - ROS1 variant (LEGACY)
5. **navstats_logger.py** - Statistics logging (UTILITY)
6. **test_top_pred.py** - Testing utility (DEVELOPMENT)

## 4. Key Dependencies

### 4.1 External ROS2 Dependencies
- **nav2_msgs** - Nav2 navigation actions
- **tf2_ros** - Transform library
- **rclpy** - ROS2 Python client library
- **action_msgs** - Action message types
- **geometry_msgs** - Geometric message types

### 4.2 Internal Dependencies
- **topological_navigation_msgs** - Custom message/service/action definitions
- **manager2.py** - Core map management
- **route_search2.py** - Path planning algorithms
- **edge_action_manager2.py** - Edge execution logic

## 5. ROS2 Communication Patterns

### 5.1 Topics
```
Published:
- /topological_map_2 (String) - Map data in YAML format
- /current_node (String) - Robot's current topological node
- /closest_node (String) - Closest node to robot
- /closest_edges (ClosestEdges) - Nearest edges
- /topological_navigation/Route (TopologicalRoute) - Current route
- /topological_navigation/Statistics (NavStatistics) - Nav stats
- /current_edge (String) - Current edge being traversed

Subscribed:
- TF transforms (map -> base_link)
- /robot_poses (for multi-robot)
```

### 5.2 Services
```
- /topological_navigation/get_route_to
- /topological_navigation/get_route_between
- /topological_localisation/get_nodes_with_tag
- /topological_localisation/localise_pose
- /restrictions_manager/evaluate_edge
- /restrictions_manager/evaluate_node
```

### 5.3 Actions
```
- /topological_navigation (GotoNode)
- /topological_navigation/execute_policy_mode (ExecutePolicyMode)
- /navigate_to_pose (Nav2)
- /navigate_through_poses (Nav2)
- /follow_waypoints (Nav2)
```

## 6. Recommendations

### 6.1 For ROS2 Development
- Focus on scripts ending with `2.py` (e.g., navigation2.py, localisation2.py)
- Use manager2.py, route_search2.py, and edge_action_manager2.py as core libraries
- Leverage the flexible properties system for domain-specific metadata

### 6.2 For Code Maintenance
- Consider deprecating ROS1 scripts if ROS1 support is no longer needed
- Document which scripts are actively used in production
- Create integration tests for core ROS2 navigation flow

### 6.3 For New Features
- Extend edge_action_manager2.py for new edge action types
- Add properties to nodes/edges for new metadata
- Use the existing route_search2.py for path planning modifications

## 7. Conclusion

The topological_navigation codebase contains both ROS1 and ROS2 implementations. The ROS2 system is centered around four core nodes:
1. map_manager2.py (map loading)
2. localisation2.py (robot localization)
3. navigation2.py (navigation execution)
4. get_simple_policy2.py (route planning)

These nodes communicate via topics, services, and actions to provide topological navigation capabilities. The system integrates with Nav2 for metric navigation and supports agricultural-specific operations through specialized handlers.
