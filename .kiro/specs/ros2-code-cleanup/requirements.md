# ROS2 Code Cleanup - Requirements

## Overview
Remove all legacy ROS1 code, keeping only essential ROS2 components.

## User Stories

### 2.1 Clean Codebase
**As a** maintainer  
**I want** to remove all ROS1 legacy code  
**So that** the codebase is cleaner and ROS2-focused

**Acceptance Criteria:**
- All ROS1 scripts removed
- All ROS1 libraries removed  
- Only essential ROS2 scripts remain
- setup.py updated
- No broken imports

### 2.2 Essential ROS2 System
**As a** developer  
**I want** only essential ROS2 components  
**So that** the system is minimal and efficient

**Acceptance Criteria:**
- 4 core ROS2 nodes preserved
- Essential tools preserved
- All dependencies satisfied
- System functionality maintained

## Files to Remove (36 total)

### ROS1 Scripts (18)
navigation.py, localisation.py, map_manager.py, get_simple_policy.py,
visualise_map.py, visualise_map2.py, map_publisher.py, search_route.py,
travel_time_estimator.py, restrictions_manager.py, reconf_at_edges_server.py,
nav_client.py, navstats_logger.py, topological_prediction.py,
mean_based_prediction.py, speed_based_prediction.py,
manual_edge_predictions.py, evaluate_top_pred.py, test_top_pred.py

### ROS1 Libraries (18)
manager.py, route_search.py, edge_action_manager.py,
edge_reconfigure_manager.py, topological_map.py, policies.py,
restrictions_impl.py, topomap_marker.py, policy_marker.py,
node_controller.py, edge_controller.py, vertex_controller.py,
node_manager.py, goto.py, edge_std.py,
marker_arrays.py, publisher.py, testing.py

## Files to Keep (26 total)

### ROS2 Scripts (12)
navigation2.py, localisation2.py, map_manager2.py, get_simple_policy2.py,
visualise_map_ros2.py, topomap_marker2.py, topological_visual.py,
occupancy_checker.py, topological_transform_publisher.py,
manual_topomapping.py, validate_map.py, policy_marker2.py

### Libraries (14)
manager2.py, route_search2.py, edge_action_manager2.py,
edge_reconfigure_manager2.py, goal_builder.py, row_operation_handler.py,
topomap_marker2.py, policy_marker2.py, param_processing.py, actions_bt.py,
tmap_utils.py, point2line.py, navigation_stats.py, map_types.py

## Tasks
1. Remove ROS1 scripts (18 files)
2. Remove ROS1 libraries (18 files)
3. Update setup.py (remove 18 entry points)
4. Update documentation
5. Verify no broken imports
6. Test core navigation

## Success Criteria
- 36 files removed (58% reduction)
- 26 files remain
- No broken imports
- Core navigation works
