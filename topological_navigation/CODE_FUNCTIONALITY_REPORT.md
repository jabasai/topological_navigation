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
