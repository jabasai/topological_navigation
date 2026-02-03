# Topological Navigation – Functionality & Usage Audit

Date: 2026-02-03

## Scope
This report documents the functionality of the **topological_navigation** package and provides a static “used vs. unused” scan of scripts and selected functions. The scan is conservative and based on:
- **Console entry points** in setup.py
- **Launch file node types** in launch/
- **In-package imports** (topological_navigation.* and topological_navigation.scripts.*)

> Note: Dynamic use (e.g., runtime parameter loading, plugin discovery, ROS launch overrides, or external packages importing these modules) may not be captured in a static scan.

---

## High-Level Functionality
Topological Navigation provides a ROS 2 (and legacy ROS 1) framework for navigating a robot using a **graph of named nodes and edges** instead of continuous metric coordinates. The system enables:

1. **Topological Map Management**
   - Load, publish, update, and persist topological maps (.tmap2 YAML).
   - Support both JSON-encoded map streaming (/topological_map_2) and legacy TopologicalMap messages (/topological_map).

2. **Topological Localisation**
   - Compute the **current node** and **closest node/edges** based on robot pose.
   - Optional localisation-by-topic logic (map-configured topic/value matching).

3. **Topological Route Planning**
   - A* route search over the topological graph.
   - Edge avoidance and route validation.

4. **Edge Action Execution**
   - Execute edge-specific actions (e.g., NavigateToPose, NavigateThroughPoses, row traversal, goal alignment).
   - Integrates with Nav2 action servers, behavior trees, and edge-specific parameters.

5. **Restrictions & Policies**
   - Runtime and planning restrictions via restrictions_manager.
   - Policy generation and execution (simple policy tools and prediction components).

6. **Visualization & Tooling**
   - RViz markers for nodes, edges, and map visualization.
   - CLI utilities for map validation and testing.

7. **Agricultural Row Operations**
   - Specialized row traversal and boundary handling (row operations, boundary nodes, and side-edge paths).

---

## Core Data Structures
### Topological Map (tmap2)
- YAML file format with nodes and edges.
- Nodes include pose, influence zone, edges, and a flexible properties dictionary.
- Edges include action/action_type and optional properties.

### Flexible Properties System
- `properties` is optional and namespace-friendly.
- Domain-specific metadata is safely accessed using `.get()` with defaults.

---

## Key Runtime Components (ROS 2)
### Map Manager 2
- **File**: manager2.py
- **Node**: map_manager_2
- Provides services to query and modify maps, and publishes /topological_map_2 and /topological_map (legacy).

### Localisation 2
- **File**: scripts/localisation2.py
- Computes current and closest nodes and publishes:
  - /current_node
  - /closest_node
  - /closest_edges

### Navigation 2
- **File**: scripts/navigation2.py
- Action server for topological navigation:
  - /topological_navigation (GotoNode)
  - /topological_navigation/execute_policy_mode
- Integrates route search, edge actions, and restrictions.

### Edge Action Manager 2
- **File**: edge_action_manager2.py
- Executes Nav2 actions (NavigateToPose, NavigateThroughPoses, FollowWaypoints).
- Supports row traversal and boundary checking.

### Route Search 2
- **File**: route_search2.py
- Implements A* on a topological graph.

---

## ROS 2 Usage Note (Entry Points, Topics, Actions, Services)
This note describes **ROS 2** nodes and utilities in this package. ROS 1 components are covered separately under Legacy Components.

### ROS 2 Entry Points (console scripts)
The following entry points are rclpy-based:

- **get_simple_policy2.py** – Route service for generating simple policies over a tmap2 map.
- **localisation2.py** – Topological localisation for ROS 2.
- **map_manager2.py** – Map loading/publishing and map services for tmap2.
- **navigation2.py** – Topological navigation action server.
- **topological_transform_publisher.py** – Publishes the static transform defined in tmap2.
- **manual_topomapping.py** – Manual topological map creation using joystick/odometry.
- **occupancy_checker.py** – Node pair for plan TF conversion and occupied-node detection.
- **topological_visual.py** – Visualizers for route and occupied nodes.
- **visualise_map_ros2.py** – RViz interactive marker visualization for tmap2.

> Other entry points listed in setup.py are ROS 1 or legacy utilities.

### ROS 2 Node Interfaces (Topics, Services, Actions)

#### map_manager2 (map_manager_2)
**Publishes**
- /topological_map_2 (std_msgs/String)
- /topological_map (topological_navigation_msgs/TopologicalMap) – legacy bridge if enabled

**Services (selected; all are in /topological_map_manager2/...)**
- get_topological_map (std_srvs/Trigger)
- get_tagged_nodes, get_tags, get_node_tags, get_edges_between_nodes
- write_topological_map, switch_topological_map
- add_topological_node, remove_topological_node, add_edges_between_nodes, remove_edge
- update_node_name, update_node_pose, update_node_tolerance
- modify_node_tags, add_tag_to_node, rm_tag_from_node
- update_edge, update_action
- update_node_restrictions, update_edge_restrictions
- update_fail_policy, set_node_influence_zone
- clear_topological_nodes
- *Multi-operations*: add_topological_node_multi, add_edges_between_nodes_multi, add_param_to_edge_config_multi, set_node_influence_zone_multi

#### localisation2 (TopologicalNavLoc)
**Publishes**
- closest_node (std_msgs/String)
- closest_node_distance (std_msgs/Float32)
- current_node (std_msgs/String)
- closest_edges (topological_navigation_msgs/ClosestEdges)
- current_node/tag (std_msgs/String)
- robot_navigation_area (std_msgs/String)

**Subscribes**
- /topological_map_2 (std_msgs/String)
- TF between tmap frame and base frame

**Services**
- /topological_localisation/get_nodes_with_tag (topological_navigation_msgs/GetTaggedNodes)
- /topological_localisation/localise_pose (topological_navigation_msgs/LocalisePose)

#### navigation2 (TopologicalNavServer)
**Publishes**
- topological_navigation/Statistics (topological_navigation_msgs/NavStatistics)
- topological_navigation/Route (topological_navigation_msgs/TopologicalRoute)
- current_edge (std_msgs/String)
- topological_navigation/move_action_status (std_msgs/String)

**Subscribes**
- /topological_map_2 (std_msgs/String)
- closest_node (std_msgs/String)
- closest_edges (topological_navigation_msgs/ClosestEdges)
- current_node (std_msgs/String)

**Actions (servers)**
- /topological_navigation (topological_navigation_msgs/GotoNode)
- /topological_navigation/execute_policy_mode (topological_navigation_msgs/ExecutePolicyMode)

**Services (clients)**
- /restrictions_manager/evaluate_edge (topological_navigation_msgs/EvaluateEdge)
- /restrictions_manager/evaluate_node (topological_navigation_msgs/EvaluateNode)

#### edge_action_manager2 (EdgeActionManager)
**Publishes**
- /boundary_checker (nav_msgs/Path)
- /robot_operation_current_status (std_msgs/String)
- /topological_navigation/current_destination (std_msgs/String)
- /target_edge_path (nav_msgs/Path)
- /center_node/pose (geometry_msgs/PoseStamped)
- /robot_current_behavior (robot_behavior_msg/RobotBehavior) – optional

**Subscribes**
- /odometry/global (nav_msgs/Odometry)
- closest_node (std_msgs/String)
- /robot_navigation_area (std_msgs/String)

**Actions (clients)**
- Nav2 action servers resolved from action name (e.g., navigate_to_pose, navigate_through_poses, follow_waypoints, compute_path_to_pose, compute_path_through_poses)

#### get_simple_policy2 (SearchPolicyServer)
**Subscribes**
- /topological_map_2 (std_msgs/String)
- closest_node (std_msgs/String)

**Services**
- get_simple_policy/get_route_to (topological_navigation_msgs/GetRouteTo)
- get_simple_policy/get_route_between (topological_navigation_msgs/GetRouteBetween)

#### topological_transform_publisher (TopologicalTransformPublisher)
**Subscribes**
- /topological_map_2 (std_msgs/String)

**Publishes**
- Static TF transform defined by tmap2 transformation block

#### manual_topomapping (RobotTmapping)
**Publishes**
- /tmapping_nodes (visualization_msgs/MarkerArray)

**Subscribes**
- /joy (sensor_msgs/Joy) – configurable
- /gps_base/odometry (nav_msgs/Odometry) – configurable
- /gps_base/yaw (sensor_msgs/Imu) – configurable

**Services**
- /tmapping_robot/save_waypoints (std_srvs/Trigger)
- /tmapping_robot/save_map (std_srvs/Trigger)

#### occupancy_checker
This script launches two nodes:

**PoseTransformerNode**
- Publishes: /plan_in_map_frame (geometry_msgs/PoseArray)
- Subscribes: /rownav_teb_poses (geometry_msgs/PoseArray)

**OccupancyCheckerNode**
- Publishes: /topological_navigation/occupied_node (topological_navigation_msgs/TopologicalOccupiedNode)
- Subscribes: /plan_in_map_frame (geometry_msgs/PoseArray)
- Subscribes: /topological_navigation/current_destination (std_msgs/String)

#### topological_visual
**RouteVisualiserNode**
- Publishes: topological_route_visualisation (visualization_msgs/MarkerArray)
- Subscribes: topological_navigation/Route (topological_navigation_msgs/TopologicalRoute)

**OccupancyVisualiserNode**
- Publishes: /topological_navigation/visual/occupied_node (visualization_msgs/MarkerArray)
- Subscribes: /topological_navigation/occupied_node (topological_navigation_msgs/TopologicalOccupiedNode)

#### visualise_map_ros2
**Publishes**
- topological_map_visualisation (visualization_msgs/MarkerArray)
- topological_route_visualisation (visualization_msgs/MarkerArray)

**Subscribes**
- /topological_map_2 (std_msgs/String)

**Actions (clients)**
- /topological_navigation (topological_navigation_msgs/GotoNode)

### ROS 2 Interaction Diagram (Simplified)

```
           +---------------------+
           |  map_manager2       |
           |  (map services)     |
           +----------+----------+
                  |
                  | /topological_map_2
                  v
  +----------------+   +----+-----+   +---------------------+
  | localisation2  |-->| navigation2|-->| edge_action_manager2|
  | (closest/current)  | (actions) |   | (Nav2 action clients)
  +----+-----------+   +----+-----+   +----------+----------+
     |                    |                    |
     | closest_node        | action servers     | navigate_to_pose, etc.
     v                    v                    v
  get_simple_policy2   visualise_map_ros2      Nav2 action servers

  topological_transform_publisher -> TF static transform
  occupancy_checker -> occupied_node -> topological_visual
```

---

---

## Legacy Components (ROS 1)
These are maintained for compatibility but rely on rospy, mongodb_store, and ROS 1 conventions:
- scripts/localisation.py
- scripts/navigation.py
- manager.py, edge_action_manager.py, edge_reconfigure_manager.py
- topological_map.py (explicitly marks methods as @deprecated)

---

## Launch Files (Selected)
- minimal_topological_navigation.launch
- topological_navigation_empty_map.launch
- topo_nav_local.launch / topo_nav_global.launch
- topological_map_visualise.launch
- reconf_at_edges_server.launch

These launch files wire together map_manager, localisation, navigation, restrictions, and visualization scripts.

---

## Script Usage Scan (scripts/)
Legend:
- **Entry**: console entry point in setup.py
- **Launch**: referenced in launch/
- **Import**: imported by other package modules

| Script | Entry | Launch | Import | Notes |
|---|---|---|---|---|
| actions_bt.py | No | No | Yes | Imported by navigation2/edge_action_manager2 for action definitions. |
| evaluate_top_pred.py | No | No | No | **Potentially unused** (no static references). |
| get_simple_policy.py | No | No | No | **Potentially unused** (no static references). |
| get_simple_policy2.py | Yes | No | No | Console tool (policy generation). |
| in_row_operations.py | No | No | Yes | Imported by edge_action_manager2. |
| localisation.py | No | Yes | No | ROS 1 localisation. |
| localisation2.py | Yes | No | No | ROS 2 localisation. |
| manual_edge_predictions.py | Yes | No | No | Console tool. |
| manual_topomapping.py | Yes | No | No | Console tool. |
| map_manager.py | Yes | Yes | No | ROS 1 map manager. |
| map_manager2.py | Yes | Yes | No | ROS 2 map manager. |
| map_publisher.py | Yes | No | No | Map publisher utility. |
| mean_based_prediction.py | Yes | No | No | Prediction tool. |
| nav_client.py | Yes | No | No | Simple navigation client. |
| navigation.py | Yes | Yes | No | ROS 1 navigation server. |
| navigation2.py | Yes | No | No | ROS 2 navigation server. |
| navstats_logger.py | Yes | Yes | No | Navigation statistics logger. |
| occupancy_checker.py | Yes | No | No | Console utility. |
| param_processing.py | No | No | Yes | Imported by navigation2/edge_action_manager2. |
| reconf_at_edges_server.py | Yes | Yes | No | Edge reconfigure server. |
| restrictions_manager.py | Yes | Yes | No | Restrictions manager. |
| search_route.py | Yes | No | No | Route search tool. |
| speed_based_prediction.py | Yes | Yes | No | Prediction tool. |
| topological_prediction.py | Yes | Yes | No | Prediction tool. |
| topological_transform_publisher.py | Yes | No | No | Transform publisher. |
| topological_visual.py | Yes | No | No | Visual debugging tool. |
| travel_time_estimator.py | Yes | Yes | No | Travel-time estimator. |
| validate_map.py | Yes | No | No | Map validation CLI. |
| visualise_map.py | Yes | Yes | No | Map visualisation. |
| visualise_map2.py | Yes | Yes | No | Map visualisation (tmap2). |
| visualise_map_ros2.py | Yes | No | No | ROS 2 visualisation utility. |

### Potentially Unused Scripts (No Entry/Launch/Import References)
- evaluate_top_pred.py
- get_simple_policy.py

---

## Module Usage Scan (topological_navigation/)
Notes:
- Modules below are **not imported by other package modules**, but can still be used via entry points or tests.

### Observed external usage
- load_maps_from_yaml.py is used in tests/scenario_server.py
- testing.py is used in tests/travel_time_tester.py
- policy_marker.py, topomap_marker.py, and their “2” variants are used via console entry points

### Potentially unused (no in-package imports found)
- None beyond test- or entry-point-only usage.

---

## Function-Level Observations (Conservative)
### Deprecated Legacy API
- **File**: topological_map.py
- Many methods are decorated with @deprecated and depend on ROS 1 + MongoDB.
- These are likely unused in ROS 2 workflows and retained for compatibility.

> Full unused-function detection requires deeper call-graph analysis across runtime usage and external packages, which is beyond static scanning.

---

## Recommendations
1. **Clarify ROS 1 vs ROS 2 usage** in README or a new migration note.
2. **Review potentially unused scripts** (evaluate_top_pred.py, get_simple_policy.py). Remove or document if still required.
3. **Document entry points** as a single table in README for easier discovery.
4. **Consider marking ROS 1 scripts/modules** as legacy to avoid confusion.

---

## References
- setup.py (console scripts)
- launch/ (runtime configuration)
- manager2.py, localisation2.py, navigation2.py, edge_action_manager2.py
- topological_map.py (legacy API)
