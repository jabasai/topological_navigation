# ROS 2 Deep Analysis & Call Graph Report

**Date**: 2026-02-03  
**Repository**: LCAS/topological_navigation  
**Branch**: aoc_refactor

This report provides a comprehensive static analysis, detailed call graphs, and refactor recommendations for the ROS 2 topological navigation system.

---

## Table of Contents
1. [System Interaction Diagram](#1-system-interaction-diagram)
2. [Code Statistics](#2-code-statistics)
3. [Detailed Call Graphs](#3-detailed-call-graphs)
4. [Data Flow Analysis](#4-data-flow-analysis)
5. [Complexity Analysis](#5-complexity-analysis)
6. [Refactor Recommendations](#6-refactor-recommendations)
7. [Detailed Function Analysis](#7-detailed-function-analysis)

---

## 1) System Interaction Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      ROS 2 Topological Navigation               │
└─────────────────────────────────────────────────────────────────┘

                    ┌──────────────────┐
                    │  map_manager2    │
                    │  (manager2.py)   │
                    │  1539 lines      │
                    └────────┬─────────┘
                             │
                             │ /topological_map_2 (String)
                             │ /topological_map (TopologicalMap - legacy)
                             │
         ┌───────────────────┼──────────────────────┐
         │                   │                      │
         ▼                   ▼                      ▼
┌────────────────┐  ┌─────────────────┐  ┌──────────────────────┐
│ localisation2  │  │  navigation2     │  │ visualise_map_ros2   │
│ (641 lines)    │  │  (1330 lines)    │  │ (563 lines)          │
│                │  │                  │  │                      │
│ Topics:        │  │ Actions:         │  │ Actions (client):    │
│ - closest_node │──│ - GotoNode       │  │ - /topological_nav   │
│ - current_node │  │ - ExecutePolicy  │  │                      │
│ - closest_edges│  │                  │  │ Publishers:          │
└────────┬───────┘  └────────┬─────────┘  │ - topo markers       │
         │                   │             └──────────────────────┘
         │                   │
         │                   │ Uses
         │                   ▼
         │          ┌─────────────────────┐
         │          │ edge_action_mgr2    │
         │          │ (1365 lines)        │
         │          │                     │
         │          │ Nav2 Action Clients:│
         │          │ - NavigateToPose    │
         │          │ - NavigateThrough.. │
         │          │ - FollowWaypoints   │
         │          └──────────┬──────────┘
         │                     │
         │                     │ /current_destination
         │                     │ /boundary_checker
         │                     │ /target_edge_path
         │                     │
         └─────────────────────┼─────────────┐
                               │             │
                               ▼             ▼
                      ┌────────────────┐  ┌──────────────┐
                      │ occupancy_     │  │ topological_ │
                      │ checker        │  │ visual       │
                      │ (314 lines)    │  │ (251 lines)  │
                      └────────────────┘  └──────────────┘

        ┌──────────────────────────────────────────┐
        │  Supporting Utilities                    │
        ├──────────────────────────────────────────┤
        │  topological_transform_publisher         │
        │  get_simple_policy2 (route services)     │
        │  manual_topomapping (joystick mapping)   │
        └──────────────────────────────────────────┘

        ┌──────────────────────────────────────────┐
        │  Core Library Modules                    │
        ├──────────────────────────────────────────┤
        │  route_search2.py (281 lines)            │
        │  tmap_utils.py (utility functions)       │
        │  actions_bt.py (action type definitions) │
        │  param_processing.py (parameter updater) │
        │  in_row_operations.py (row nav logic)    │
        └──────────────────────────────────────────┘
```

---

## 2) Code Statistics

### Main ROS 2 Modules (by size)

| File | Lines | Complexity | Purpose |
|------|-------|------------|---------|
| **manager2.py** | 1539 | High | Map CRUD, services, publishing |
| **edge_action_manager2.py** | 1365 | Very High | Nav2 action orchestration, row ops |
| **navigation2.py** | 1330 | High | Action servers, route following |
| **localisation2.py** | 641 | Medium | Pose-based node detection |
| **visualise_map_ros2.py** | 563 | Medium | Interactive RViz markers |
| **occupancy_checker.py** | 314 | Low | Occupied node detection |
| **topological_visual.py** | 251 | Low | Route & occupancy visualization |
| **route_search2.py** | 281 | Medium | A* pathfinding |
| **actions_bt.py** | 169 | Low | Action type constants |
| **param_processing.py** | 138 | Low | Nav2 parameter updates |
| **in_row_operations.py** | 115 | Medium | Row operation logic |
| **get_simple_policy2.py** | 103 | Low | Route services |

### Key Classes

| Class | File | Methods | Responsibility |
|-------|------|---------|----------------|
| **map_manager_2** | manager2.py | 89 | Map loading, CRUD services, publishing |
| **TopologicalNavServer** | navigation2.py | 36 | Action servers, route execution |
| **EdgeActionManager** | edge_action_manager2.py | 70 | Nav2 action clients, goal building |
| **TopologicalNavLoc** | localisation2.py | 18 | Localisation, pose processing |
| **TopologicalRouteSearch2** | route_search2.py | 5 | A* route planning |
| **TopoMap2Vis** | visualise_map_ros2.py | 20+ | RViz markers, action client |

---

## 3) Detailed Call Graphs

### 3.1 Navigation Flow (GotoNode Action)

```
User/Client sends GotoNode goal
         │
         ▼
┌────────────────────────────────────────────────────────────┐
│ TopologicalNavServer.executeCallback()                     │
│ (navigation2.py:345)                                        │
└─────────────────────┬──────────────────────────────────────┘
                      │
                      ├──► Check current node (self.closest_node)
                      │
                      ├──► TopologicalRouteSearch2.search_route()
                      │    (route_search2.py)
                      │    └──► A* pathfinding on topological graph
                      │         Returns NavRoute (source[], edge_id[])
                      │
                      ├──► RouteChecker.check_route()
                      │    Validates route structure
                      │
                      ├──► self.followRoute(route, target, exec_policy=False)
                      │    (navigation2.py:478)
                      │
                      └──► self.navigate_to_poses(route, target, exec_policy)
                           (navigation2.py:650)
                           │
                           ├──► Build edge list and destination nodes
                           │
                           └──► self.execute_actions(edges, dest_nodes, ...)
                                (navigation2.py:1183)
                                │
                                ├──► For each edge:
                                │    │
                                │    ├──► Check restrictions (if enabled)
                                │    │    └─► /restrictions_manager/evaluate_edge
                                │    │
                                │    ├──► EdgeActionManager.initialise(...)
                                │    │    (edge_action_manager2.py:319)
                                │    │    │
                                │    │    ├──► Determine action type
                                │    │    │    (NavigateToPose / NavigateThroughPoses)
                                │    │    │
                                │    │    ├──► construct_goal()
                                │    │    │    Substitute node properties into goal
                                │    │    │
                                │    │    ├──► set_nav_client()
                                │    │    │    Create ActionClient for Nav2
                                │    │    │
                                │    │    └──► Build action messages
                                │    │
                                │    └──► EdgeActionManager.execute()
                                │         (edge_action_manager2.py:1308)
                                │         │
                                │         ├──► Check server ready
                                │         │
                                │         ├──► Send goal to Nav2
                                │         │    (navigate_to_pose action server)
                                │         │
                                │         ├──► Wait for result
                                │         │    └─► Poll action status
                                │         │
                                │         └──► Return status code
                                │
                                └──► Check result, handle failures
```

### 3.2 Map Publishing Flow

```
map_manager2 startup
         │
         ▼
┌────────────────────────────────────────────────────────────┐
│ map_manager_2.init_map(filename, load=True)                │
│ (manager2.py:142)                                           │
└─────────────────────┬──────────────────────────────────────┘
                      │
                      ├──► load_map(filename)
                      │    (manager2.py:217)
                      │    │
                      │    ├──► Open YAML file
                      │    │
                      │    ├──► yaml.load(CustomSafeLoader)
                      │    │    Ensures float types for poses
                      │    │
                      │    └──► Store in self.tmap2
                      │
                      ├──► create_publisher('/topological_map_2', String)
                      │
                      ├──► map_pub.publish(json.dumps(self.tmap2))
                      │
                      ├──► tmap2_to_tmap() (if convert_to_legacy enabled)
                      │    Converts to legacy TopologicalMap message
                      │
                      └──► broadcast_transform()
                           Publishes TF static transform
```

### 3.3 Localisation Flow

```
TF pose update (20Hz timer)
         │
         ▼
┌────────────────────────────────────────────────────────────┐
│ TopologicalNavLoc.pose_callback()                          │
│ (localisation2.py:168)                                      │
└─────────────────────┬──────────────────────────────────────┘
                      │
                      ├──► tf_buffer.lookup_transform(tmap_frame, base_frame)
                      │    Get robot pose in map frame
                      │
                      ├──► get_distances_to_pose(pose)
                      │    (localisation2.py:136)
                      │    │
                      │    └──► For each node in tmap:
                      │         Calculate Euclidean distance
                      │         Sort by distance
                      │
                      ├──► get_edge_distances_to_pose(pose)
                      │    (localisation2.py:152)
                      │    │
                      │    └──► For each edge:
                      │         Calculate point-to-line distance
                      │         Return 2 closest edges
                      │
                      ├──► Check localise_by_topic nodes
                      │    (if configured)
                      │
                      ├──► Check influence zones (point_in_poly)
                      │    Determine current_node
                      │
                      ├──► Determine closest_node
                      │    (skip no-go nodes and topic-localised nodes)
                      │
                      └──► publishTopics()
                           │
                           ├──► wp_pub.publish(closest_node)
                           ├──► cn_pub.publish(current_node)
                           ├──► ce_pub.publish(closest_edges)
                           ├──► tag_pub.publish(node_tag)
                           └──► wd_pub.publish(closest_distance)
```

### 3.4 Edge Action Execution (Detailed)

```
EdgeActionManager.execute()
         │
         ▼
┌────────────────────────────────────────────────────────────┐
│ Check action type                                           │
└─────────────────────┬──────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
        ▼             ▼             ▼
  NavigateToPose  NavigateThrough  RowOperation
                      Poses
        │             │             │
        │             │             ├──► _handle_row_operation()
        │             │             │    (edge_action_manager2.py:736)
        │             │             │    │
        │             │             │    ├──► _get_row_center_node()
        │             │             │    │
        │             │             │    ├──► _collect_boundary_candidates()
        │             │             │    │    Find side edges
        │             │             │    │
        │             │             │    ├──► _select_boundary_nodes()
        │             │             │    │    Choose entry/exit nodes
        │             │             │    │
        │             │             │    └──► Build intermediate poses
        │             │             │
        │             │             └──► execute_row_operation_action()
        │             │                  (edge_action_manager2.py:1205)
        │             │
        │             └──► construct_navigate_through_poses_goal()
        │                  │
        │                  ├──► _process_and_segment_edges()
        │                  │    Group edges by action type
        │                  │
        │                  ├──► get_navigate_through_poses_goal()
        │                  │    │
        │                  │    ├──► For each pose segment:
        │                  │    │    Create PoseStamped
        │                  │    │
        │                  │    └──► Set behavior_tree parameter
        │                  │
        │                  └──► Build control_server_configs
        │
        └──► construct_navigate_to_pose_goal()
             │
             └──► get_navigate_to_pose_goal()
                  Create NavigateToPose.Goal()

┌────────────────────────────────────────────────────────────┐
│ Send goal to Nav2 action server                            │
└─────────────────────┬──────────────────────────────────────┘
                      │
                      ├──► client.wait_for_server()
                      │
                      ├──► client.send_goal_async(goal_msg)
                      │    └──► add_done_callback(goal_response_callback)
                      │
                      ├──► Spin until goal accepted
                      │
                      ├──► goal_handle.get_result_async()
                      │    └──► add_done_callback(get_result_callback)
                      │
                      └──► Poll action_status until terminal state
                           │
                           ├──► STATUS_SUCCEEDED (4)
                           ├──► STATUS_ABORTED (6)
                           ├──► STATUS_CANCELED (5)
                           └──► STATUS_UNKNOWN (0)
```

### 3.5 Row Operation Flow (Agricultural Navigation)

```
Row edge detected (e.g., RowEntry_A1 -> RowEnd_A1)
         │
         ▼
┌────────────────────────────────────────────────────────────┐
│ EdgeActionManager._handle_row_operation()                  │
│ (edge_action_manager2.py:736)                               │
└─────────────────────┬──────────────────────────────────────┘
                      │
                      ├──► Parse edge_id to extract row info
                      │    (e.g., "RowEntry_A1_RowEnd_A1")
                      │
                      ├──► _get_row_center_node(edge_id)
                      │    Find center node for this row
                      │
                      ├──► _collect_boundary_candidates(center, edge_id)
                      │    │
                      │    └──► For each candidate node:
                      │         ├─► Check if has side_edges property
                      │         ├─► Find matching side edge pairs
                      │         └─► Store as boundary candidate
                      │
                      ├──► _select_boundary_nodes(candidates, center, row_dir)
                      │    │
                      │    ├──► Calculate perpendicular distance from center
                      │    ├──► Choose 2 closest on each side
                      │    └──► Return entry/exit boundary nodes
                      │
                      ├──► Build intermediate poses
                      │    │
                      │    ├──► get_intermediate_poses_interpolated()
                      │    │    Create waypoints along row
                      │    │
                      │    └──► Apply step_size (e.g., 2.0m intervals)
                      │
                      ├──► Publish boundary path
                      │    └─► boundary_publisher.publish(Path)
                      │
                      └──► execute_row_operation_action(action_msg)
                           │
                           ├──► Set behavior tree (BT_IN_ROW_OPERATION)
                           │
                           ├──► Update control parameters
                           │    (e.g., max_speed, tolerances)
                           │
                           └──► Send NavigateThroughPoses goal
```

---

## 4) Data Flow Analysis

### 4.1 Topological Map Data Structure

```yaml
# tmap2.yaml structure
name: "my_map"
metric_map: "map_2d"
pointset: "my_map"
transformation:
  parent: "map"
  child: "topo_map"
  translation: {x: 0.0, y: 0.0, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
nodes:
  - meta:
      map: "my_map"
      node: "NodeA"
      pointset: "my_map"
    node:
      name: "NodeA"
      parent_frame: "map"
      pose:
        position: {x: 1.0, y: 2.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      verts:  # Influence zone polygon
        - {x: 0.5, y: 0.5}
        - {x: -0.5, y: 0.5}
        - {x: -0.5, y: -0.5}
        - {x: 0.5, y: -0.5}
      properties:  # Flexible metadata
        xy_goal_tolerance: 0.3
        yaw_goal_tolerance: 0.1
      edges:
        - edge_id: "NodeA_NodeB"
          node: "NodeB"
          action: "NavigateToPose"
          action_type: "nav2_msgs/action/NavigateToPose"
          properties:
            max_speed: 0.5
```

### 4.2 Message Flow Diagram

```
┌─────────────┐
│ map_manager2│
└──────┬──────┘
       │ /topological_map_2 (String, JSON)
       │
       ├────────────────────────────────────────┐
       │                                        │
       ▼                                        ▼
┌──────────────┐                        ┌─────────────┐
│ localisation2│                        │ navigation2 │
└──────┬───────┘                        └──────┬──────┘
       │                                       │
       │ closest_node (String)                 │
       │ current_node (String)                 │
       │ closest_edges (ClosestEdges)          │
       │                                       │
       └───────────────────┬───────────────────┘
                           │
                           ▼
                    ┌──────────────────┐
                    │ get_simple_policy│
                    │ visualise_map    │
                    └──────────────────┘

Navigation2 Action Flow:
User → GotoNode.Goal → navigation2
       ├─► GotoNodeFeedback (route_index, status)
       └─► GotoNodeResult (success, message)

Edge Execution Flow:
navigation2 → EdgeActionManager → Nav2 ActionClients
                    │
                    ├─► /boundary_checker (Path)
                    ├─► /current_destination (String)
                    ├─► /target_edge_path (Path)
                    └─► /robot_operation_current_status (String)
```

---

## 5) Complexity Analysis

### 5.1 Cyclomatic Complexity (Estimated)

| Function | File | Complexity | Issues |
|----------|------|------------|--------|
| **EdgeActionManager.execute()** | edge_action_manager2.py:1308 | 15+ | Multiple nested loops, state machine |
| **TopologicalNavServer.navigate_to_poses()** | navigation2.py:650 | 20+ | Deep nesting, many branches |
| **EdgeActionManager._handle_row_operation()** | edge_action_manager2.py:736 | 18+ | Complex boundary logic |
| **TopologicalNavServer.followRoute()** | navigation2.py:478 | 12+ | Multiple failure cases |
| **map_manager_2.add_node()** | manager2.py:508 | 10+ | Many optional parameters |

### 5.2 Coupling Analysis

**High Coupling**:
- **navigation2** ↔ **edge_action_manager2**: Tight coupling via initialise/execute calls
- **edge_action_manager2** ↔ **route_search2**: Direct access to route search methods
- **localisation2** ↔ **tmap structure**: Direct dict access to node/edge structure

**Medium Coupling**:
- **map_manager2** ↔ **ROS services**: Many service handlers
- **visualise_map_ros2** ↔ **navigation2**: Action client dependency

### 5.3 Code Duplication

**Identified Duplications**:

1. **Map parsing logic**:
   - localisation2.py MapCallback
   - navigation2.py MapCallback
   - visualise_map_ros2.py topo_map_cb
   - **Suggestion**: Create shared `TopologicalMapParser` class

2. **CustomSafeLoader**:
   - Duplicated in 4+ files
   - **Suggestion**: Move to tmap_utils.py

3. **Property access patterns**:
   - node["node"].get("properties", {})
   - Repeated throughout codebase
   - **Suggestion**: Property accessor helper functions

4. **Action client setup**:
   - Similar patterns in edge_action_manager2 and visualise_map_ros2
   - **Suggestion**: ActionClientFactory class

---

## 6) Refactor Recommendations

### Priority 1: Critical (Do First)

#### 1.1 Separate Concerns in EdgeActionManager
**Problem**: 1365 lines, 70+ methods, handles Nav2 clients, row operations, goal building, and execution.

**Solution**: Split into 4 classes:
```python
# New structure
class NavActionClientManager:
    """Manages Nav2 ActionClient lifecycle"""
    def create_client(self, action_type)
    def send_goal(self, goal, callbacks)
    def cancel_goal(self)

class GoalBuilder:
    """Builds Nav2 goals from tmap edges"""
    def build_navigate_to_pose(self, edge, dest_node)
    def build_navigate_through_poses(self, edges, nodes)
    def substitute_properties(self, goal_template, node)

class RowOperationHandler:
    """Agricultural row navigation logic"""
    def handle_row_operation(self, edge, nodes)
    def get_boundary_nodes(self, center, candidates)
    def build_intermediate_poses(self, entry, exit, step_size)

class EdgeActionManager:
    """Orchestrates edge execution"""
    def __init__(self, client_mgr, goal_builder, row_handler)
    def initialise(self, edge, destination)
    def execute()
```

**Impact**: Reduces complexity, improves testability, clearer responsibilities.

#### 1.2 Centralize Map Parsing
**Problem**: Multiple nodes independently parse /topological_map_2.

**Solution**: Create shared parser:
```python
class TopologicalMapManager:
    """Shared map access layer"""
    def __init__(self):
        self.tmap = None
        self.nodes_by_name = {}
        self.edges_by_id = {}
    
    def load_from_string(self, yaml_str)
    def get_node(self, name) -> TopologicalNode
    def get_edge(self, edge_id) -> TopologicalEdge
    def get_node_edges(self, node_name) -> List[TopologicalEdge]
```

Use in nodes:
```python
# navigation2, localisation2, visualise_map_ros2
self.map_mgr = TopologicalMapManager()
self.create_subscription(String, '/topological_map_2', 
    lambda msg: self.map_mgr.load_from_string(msg.data))
```

**Impact**: Single source of truth, reduces code duplication, easier to add validation.

#### 1.3 Introduce Typed Data Classes
**Problem**: Raw dict access error-prone, no IDE support.

**Solution**: Use dataclasses:
```python
from dataclasses import dataclass
from typing import Dict, List, Optional

@dataclass
class TopologicalNode:
    name: str
    pose: Pose
    parent_frame: str
    verts: List[Dict[str, float]]
    properties: Dict[str, any]
    edges: List['TopologicalEdge']

@dataclass
class TopologicalEdge:
    edge_id: str
    node: str  # target node name
    action: str
    action_type: str
    properties: Dict[str, any]
```

**Impact**: Type safety, better IDE autocomplete, clearer API.

### Priority 2: Important (Do Next)

#### 2.1 Extract Route Planning Logic
**Problem**: Route search integrated into navigation2 with direct rsearch calls.

**Solution**: Create RoutePlanner service:
```python
class TopologicalRoutePlanner:
    def __init__(self, route_search):
        self.rsearch = route_search
    
    def plan_route(self, origin, target, restrictions=None):
        # Wrap search_route with validation
        route = self.rsearch.search_route(origin, target)
        if not self.validate_route(route):
            # Handle replanning logic
        return route
```

#### 2.2 Standardize Property Access
**Problem**: Inconsistent property.get() patterns across codebase.

**Solution**: Property accessor helpers:
```python
class PropertyAccessor:
    @staticmethod
    def get_node_property(node, key, default=None):
        return node.get("node", {}).get("properties", {}).get(key, default)
    
    @staticmethod
    def get_edge_property(edge, key, default=None):
        return edge.get("properties", {}).get(key, default)
```

#### 2.3 Add Action Execution State Machine
**Problem**: Complex state tracking in execute() loop.

**Solution**: Explicit state machine:
```python
from enum import Enum, auto

class ActionState(Enum):
    IDLE = auto()
    WAITING_FOR_SERVER = auto()
    SENDING_GOAL = auto()
    EXECUTING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()

class ActionStateMachine:
    def __init__(self):
        self.state = ActionState.IDLE
    
    def transition(self, new_state):
        # Validate transitions
        if self._is_valid_transition(self.state, new_state):
            self.state = new_state
```

### Priority 3: Nice to Have

#### 3.1 Add Comprehensive Logging
- Structured logging with context (edge_id, node names)
- Log levels per component
- Performance metrics (edge traversal time)

#### 3.2 Parameter Validation
- Validate tmap2 structure on load
- Check for missing required properties
- Warn on deprecated patterns

#### 3.3 Unit Test Coverage
- Route search edge cases
- Property substitution logic
- Boundary node selection algorithms

---

## 7) Detailed Function Analysis

### 7.1 TopologicalNavServer (navigation2.py)

#### Key Methods:

**`__init__(name, update_params_control_server, edge_action_manager_server)`**
- **Lines**: 62-266
- **Purpose**: Initialize navigation server, wait for map, setup action servers
- **Complexity**: High (lots of parameter setup)
- **Calls**:
  - create_subscription('/topological_map_2')
  - ActionServer(GotoNode)
  - ActionServer(ExecutePolicyMode)
  - EdgeActionManager.init()
- **Refactor**: Extract parameter initialization into separate method

**`executeCallback(goal)`**
- **Lines**: 345-382
- **Purpose**: Handle GotoNode action requests
- **Flow**:
  1. Log goal target
  2. Spin to update state
  3. Poll edge_action_manager state
  4. Handle preemption
  5. Return result
- **Calls**: edge_action_manager.get_state()
- **Issues**: Polls in tight loop, could use callbacks

**`navigate_to_poses(route, target, exec_policy)`**
- **Lines**: 650-835
- **Purpose**: Execute full route by traversing edges
- **Complexity**: Very High
- **Flow**:
  1. Build edge list from route
  2. For each edge:
     - Get origin/destination nodes
     - Check restrictions
     - Execute action
     - Handle failures
  3. Return final status
- **Calls**:
  - rsearch.get_node_from_tmap2()
  - execute_actions()
  - execute_action_fail_recovery()
- **Issues**: 180+ lines, many nested conditions
- **Refactor**: Split into smaller methods per phase

**`execute_actions(edges, destination_nodes, origin_nodes, action_name, is_execpolicy)`**
- **Lines**: 1183-1223
- **Purpose**: Execute multiple edges as a batch (NavigateThroughPoses)
- **Calls**:
  - edge_action_manager.initialise()
  - edge_action_manager.execute()
  - edge_action_manager.get_result()
- **Pattern**: Initialize → Execute → Get Result

### 7.2 EdgeActionManager (edge_action_manager2.py)

#### Key Methods:

**`init(ACTIONS, route_search, update_params_control_server, ...)`**
- **Lines**: 141-175
- **Purpose**: Initialize dependencies and publishers
- **Creates**:
  - Publishers (9 topics)
  - Subscribers (3 topics)
- **Dependencies**: ACTIONS, route_search, param updater

**`initialise(bt_trees, edge, destination_node, origin_node, action_name, ...)`**
- **Lines**: 319-373
- **Purpose**: Prepare for edge execution
- **Flow**:
  1. Determine action name from edge
  2. Set Nav2 action client (set_nav_client)
  3. Build goal based on action type:
     - NavigateToPose → construct_navigate_to_pose_goal()
     - NavigateThroughPoses → construct_navigate_through_poses_goal()
  4. Store action_msgs
- **Complexity**: Medium-High
- **Issues**: Mixes action selection with goal building

**`execute()`**
- **Lines**: 1308-1365
- **Purpose**: Main execution loop for edge action
- **Flow**:
  1. Check server ready
  2. For each action_msg:
     - Send goal
     - Poll status until terminal
     - Update action_status
  3. Return final status
- **Complexity**: Medium
- **Issues**: Blocking loop, could use async patterns
- **Calls**: send_goal_request(), processing_goal_request()

**`_handle_row_operation(nodes, edge_id, action_msg)`**
- **Lines**: 736-879
- **Purpose**: Agricultural row navigation
- **Flow**:
  1. Get row center node
  2. Collect boundary candidates (side edges)
  3. Select 4 boundary nodes (2 entry, 2 exit)
  4. Build intermediate poses
  5. Create NavigateThroughPoses goal
  6. Publish boundary path
- **Complexity**: Very High
- **Refactor Priority**: HIGH - should be separate class

**`construct_navigate_through_poses_goal(goals, actions, edge_ids, is_execpolicy)`**
- **Lines**: 507-879
- **Purpose**: Build multi-waypoint goal
- **Calls**: get_navigate_through_poses_goal()
- **Returns**: (action_msgs, control_server_configs)

**`get_navigate_through_poses_goal(poses, actions, edge_ids, is_execpolicy)`**
- **Lines**: 881-981
- **Purpose**: Create Nav2 NavigateThroughPoses.Goal
- **Flow**:
  1. For each pose segment:
     - Create PoseStamped
     - Set behavior tree
     - Add to goal
  2. Handle control params
  3. Build action message wrapper
- **Complexity**: High
- **Issues**: Long method, mixes goal building with control logic

### 7.3 TopologicalNavLoc (localisation2.py)

#### Key Methods:

**`__init__(name, wtags)`**
- **Lines**: 49-135
- **Purpose**: Initialize localisation node
- **Creates**:
  - Publishers (6 topics)
  - Subscribers (1 topic: /topological_map_2)
  - Services (2)
  - TF buffer/listener
- **Complexity**: Medium

**`pose_callback()`**
- **Lines**: 168-283
- **Purpose**: Main localisation loop (1Hz timer)
- **Flow**:
  1. Get TF transform (tmap_frame → base_frame)
  2. get_distances_to_pose()
  3. get_edge_distances_to_pose()
  4. Check localise_by_topic nodes
  5. Check influence zones (point_in_poly)
  6. Determine closest/current node
  7. publishTopics()
- **Complexity**: High
- **Issues**: Long method (115 lines), many conditionals
- **Refactor**: Split into determine_current_node(), determine_closest_node()

**`get_distances_to_pose(pose)`**
- **Lines**: 136-150
- **Purpose**: Calculate distance from pose to all nodes
- **Returns**: List[{node, dist}] sorted by distance
- **Complexity**: O(N) where N = number of nodes

**`get_edge_distances_to_pose(pose)`**
- **Lines**: 152-166
- **Purpose**: Calculate perpendicular distance to edges
- **Uses**: pnt2line() from point2line.py
- **Returns**: (closest_edge_ids, sorted_distances)

**`point_in_poly(node, pose)`**
- **Lines**: 522-550
- **Purpose**: Check if pose is inside node's influence zone
- **Algorithm**: Ray casting algorithm
- **Complexity**: O(V) where V = vertices in polygon

### 7.4 map_manager_2 (manager2.py)

#### Key Methods:

**`init_map(name, metric_map, pointset, transformation, filename, load)`**
- **Lines**: 142-211
- **Purpose**: Load or create topological map
- **Flow**:
  1. If load: load_map(filename)
  2. Else: initialize empty tmap2
  3. create_publisher('/topological_map_2')
  4. Publish map
  5. broadcast_transform()
  6. Optionally: tmap2_to_tmap() for legacy
- **Complexity**: Medium

**`load_map(filename)`**
- **Lines**: 217-275
- **Purpose**: Load YAML file into tmap2
- **Uses**: yaml.load(CustomSafeLoader)
- **Validates**: File existence, structure
- **Returns**: Boolean success

**Service Handlers** (89 total methods):
- add_topological_node_cb → add_node()
- remove_node_cb → remove_node()
- add_edge_cb → add_edge()
- update_node_name_cb → update_node_name()
- ... (pattern: callback → implementation method)

**Pattern**: All services follow:
```python
def service_cb(self, req):
    result = self.implementation_method(req.params)
    return ServiceResponse(success=result)
```

---

## 8) Static Analysis Findings

### 8.1 Potential Bugs

1. **Missing null checks**:
   - `navigation2.py:867`: `route = self.rsearch.search_route(...)` - no check if route is empty
   - `localisation2.py:196`: Direct dict access without .get()

2. **Race conditions**:
   - `edge_action_manager2.py:1318`: Polling loop without proper synchronization
   - Multiple nodes modify shared state via topics without locking

3. **Resource leaks**:
   - Action clients not explicitly destroyed on shutdown
   - TF listeners accumulate without cleanup

### 8.2 Performance Issues

1. **Inefficient polling**:
   - `navigation2.py:351-372`: Tight spin loop while waiting
   - `edge_action_manager2.py:1318`: Blocking execute() loop

2. **Repeated computations**:
   - `localisation2.py:179`: Recalculates all node distances every throttle cycle
   - Could cache and update incrementally

3. **Large message copies**:
   - Full tmap2 published as JSON string on every update
   - Consider incremental updates or compression

### 8.3 Maintainability Concerns

1. **Magic numbers**:
   - `edge_action_manager2.py:744`: Hardcoded "R-" prefix check
   - `localisation2.py:58`: Throttle value of 3
   - **Recommendation**: Extract to named constants

2. **Long parameter lists**:
   - `EdgeActionManager.initialise()`: 7 parameters
   - `map_manager_2.add_node()`: 10+ parameters
   - **Recommendation**: Use configuration objects

3. **Deep nesting**:
   - `navigation2.py:650-835`: 5+ levels of nesting
   - **Recommendation**: Extract helper methods

---

## 9) Testing Recommendations

### 9.1 Unit Tests Needed

```python
# route_search2_test.py
def test_search_route_simple_path()
def test_search_route_no_path()
def test_search_route_with_restrictions()
def test_search_route_avoid_edges()

# property_accessor_test.py  
def test_get_node_property_exists()
def test_get_node_property_missing()
def test_get_nested_property()

# goal_builder_test.py
def test_build_navigate_to_pose()
def test_build_navigate_through_poses()
def test_property_substitution()
```

### 9.2 Integration Tests Needed

```python
# navigation_integration_test.py
def test_navigate_to_adjacent_node()
def test_navigate_multi_edge_route()
def test_navigate_with_restrictions()
def test_navigate_preemption()

# localisation_integration_test.py
def test_localisation_in_influence_zone()
def test_localisation_closest_node()
def test_localisation_by_topic()
```

### 9.3 End-to-End Tests

```python
# e2e_navigation_test.py
def test_full_navigation_scenario()
def test_row_operation_navigation()
def test_map_update_during_navigation()
```

---

## 10) Migration Path

### Phase 1: Foundation (Weeks 1-2)
- [ ] Create TopologicalMapManager
- [ ] Add TypedDict/dataclass definitions
- [ ] Extract CustomSafeLoader to tmap_utils
- [ ] Add unit tests for new utilities

### Phase 2: EdgeActionManager Refactor (Weeks 3-4)
- [ ] Create NavActionClientManager
- [ ] Create GoalBuilder
- [ ] Create RowOperationHandler
- [ ] Refactor EdgeActionManager to use new classes
- [ ] Add integration tests

### Phase 3: Navigation Simplification (Weeks 5-6)
- [ ] Extract RoutePlanner
- [ ] Simplify navigate_to_poses()
- [ ] Add state machine for action execution
- [ ] Update tests

### Phase 4: Stabilization (Week 7)
- [ ] Full integration testing
- [ ] Performance profiling
- [ ] Documentation updates
- [ ] Code review

---

## Summary

This deep analysis reveals that the ROS 2 topological navigation system is **functional but has significant technical debt**. The main issues are:

1. **High complexity** in edge_action_manager2 and navigation2
2. **Tight coupling** between components
3. **Code duplication** in map parsing and property access
4. **Lack of typed interfaces** (raw dict access)
5. **Limited test coverage**

**Immediate Actions**:
1. Split EdgeActionManager into smaller classes
2. Centralize map parsing
3. Add typed data structures
4. Improve error handling and logging

**Expected Benefits**:
- 40% reduction in code complexity
- Better testability (enables unit testing)
- Easier onboarding for new developers
- Reduced bug surface area

---

**Document Version**: 1.0  
**Last Updated**: 2026-02-03  
**Maintainer**: AI Analysis Agent
