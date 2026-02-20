# navigation2.py — Topological Navigation Server

**Location**: `topological_navigation/topological_navigation/scripts/navigation2.py`
**Lines**: ~1,331
**Author**: Geesara Kulathunga (ggeesara@gmail.com)
**Last Updated**: 2026-02-19

---

## 1. Overview

`navigation2.py` is the **main entry point** for the topological navigation system. It implements the `TopologicalNavServer` ROS 2 node, which orchestrates high-level graph-based navigation for a mobile robot. Rather than navigating through continuous metric space directly, this node plans and executes routes along a **topological map** — a graph of discrete **nodes** (waypoints) connected by **edges** (navigable paths).

The script acts as the **central coordinator** between:

- The **topological map** (received from the map manager)
- The **localisation system** (current node and closest edge information)
- The **Nav2 stack** (low-level metric navigation via `NavigateToPose` / `NavigateThroughPoses`)
- **Edge action management** (executing the specific action defined on each edge)
- **Edge reconfiguration** (adjusting planner parameters per-edge)
- **Restrictions management** (runtime evaluation of edge/node traversability)

### High-Level Flow

```
Client sends GotoNode / ExecutePolicyMode goal
  │
  ├── Determine origin node (from current_node or closest_edge)
  ├── Plan route via A* (TopologicalRouteSearch2)
  ├── Enforce navigable route (inject closest-edge if needed)
  ├── For each edge in route:
  │     ├── Reconfigure planner tolerances
  │     ├── Execute edge action (via EdgeActionManager → Nav2)
  │     ├── Handle fail policies (retry / replan / wait / fail)
  │     └── Publish stats / feedback
  └── Return success/failure result
```

---

## 2. Main Interfaces

### 2.1 ROS 2 Action Servers (Inputs — How Clients Use This Node)

| Action Server | Type | Purpose |
|---|---|---|
| `/<node_name>` (default: `/topological_navigation`) | `GotoNode` | Navigate robot to a named topological node |
| `/topological_navigation/execute_policy_mode` | `ExecutePolicyMode` | Execute a **pre-planned route** (list of source nodes + edge IDs) |

#### `GotoNode` Action

- **Goal**: `{target: "node_name", no_orientation: false}`
- **Result**: `{success: bool}`
- **Feedback**: `{route: "string"}` — describes current progress
- **Behavior**: The server plans the route from the robot's current position to the target node, then executes it edge-by-edge.

#### `ExecutePolicyMode` Action

- **Goal**: `{route: NavRoute}` where `NavRoute` = `{source: [str], edge_id: [str]}`
- **Result**: `{success: bool}`
- **Feedback**: `{current_wp: "string", status: uint8}`
- **Behavior**: The server validates the pre-planned route and executes it without replanning. Useful for externally-computed policies.

### 2.2 Subscriptions (Inputs — Data the Node Consumes)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/topological_map_2` | `std_msgs/String` | Transient Local | YAML-encoded topological map. Triggers map reload. |
| `closest_node` | `std_msgs/String` | Transient Local | Name of the robot's closest topological node. |
| `closest_edges` | `ClosestEdges` | Transient Local | Two closest edges with distances (from localisation). |
| `current_node` | `std_msgs/String` | Transient Local | Name of the node the robot is currently inside (or `"none"`). |

### 2.3 Publishers (Outputs — Data the Node Produces)

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `topological_navigation/Statistics` | `NavStatistics` | Transient Local | Per-edge navigation statistics (timings, status). |
| `topological_navigation/Route` | `TopologicalRoute` | Best Effort | Currently planned route (published every 2s via timer). |
| `current_edge` | `std_msgs/String` | Transient Local | Currently traversed edge ID. |
| `topological_navigation/move_action_status` | `std_msgs/String` | Transient Local | JSON-encoded action status `{goal, final_goal, action, status}`. |

### 2.4 Service Clients (Optional External Services)

| Service | Type | Purpose |
|---|---|---|
| `/restrictions_manager/evaluate_edge` | `EvaluateEdge` | Check if an edge is restricted at runtime. |
| `/restrictions_manager/evaluate_node` | `EvaluateNode` | Check if a destination node is restricted at runtime. |

### 2.5 ROS 2 Parameters

The node declares a large set of parameters that configure its behavior:

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `navigation_action_name` | String | `"NavigateToPose"` | Default Nav2 action for navigation edges |
| `navigation_actions` | String[] | See `ActionsType` | List of recognized navigation action names |
| `use_nav2_follow_route` | Bool | `False` | If `True`, use `navigate_to_poses()` (NavigateThroughPoses); else `followRoute()` (edge-by-edge NavigateToPose) |
| `use_in_row_operation` | Bool | `False` | Enable agricultural in-row operation support |
| `inrow_step_size` | Double | `2.0` | Step size for in-row operations |
| `inrow_step_intermediate_dis` | Double | `-1.0` | Intermediate distance for in-row steps |
| `max_dist_to_closest_edge` | Double | `1.0` | Max distance to consider planning from an edge vs. from closest node |
| `reconfigure_edges` | Bool | `True` | Enable per-edge planner reconfiguration |
| `reconfigure_edges_srv` | Bool | `False` | Use service-based reconfiguration instead of direct param setting |
| `row_traversal_planner` | String | `"dwb_core::DWBLocalPlanner"` | Planner plugin for row traversal actions |
| `default_planner` | String | `"dwb_core::DWBLocalPlanner"` | Planner plugin for default navigation |
| `goal_align_planner` | String | `"dwb_core::DWBLocalPlanner"` | Planner plugin for goal alignment actions |
| `*_xy_goal_tolerance` | Double | varies | XY tolerance per planner type |
| `*_yaw_goal_tolerance` | Double | varies | Yaw tolerance per planner type |
| `allow_intermediate_orientation_override` | Bool | `False` | Override intermediate node orientations to face the next node (smoother paths) |
| `bt_tree_default` | String | `config/bt_tree_default.xml` | Behavior tree XML for default navigation |
| `bt_tree_in_row` | String | `config/bt_tree_in_row.xml` | Behavior tree XML for in-row navigation |
| `bt_tree_goal_align` | String | `config/bt_tree_goal_align.xml` | Behavior tree XML for goal alignment |
| `bt_tree_in_row_operation` | String | `config/bt_tree_in_row_operation.xml` | Behavior tree XML for in-row operations |
| `bt_tree_in_row_recovery` | String | `config/bt_tree_in_row_recovery.xml` | Behavior tree XML for in-row recovery |

---

## 3. Behavior Determination — How Navigation Decisions Are Made

### 3.1 Initialization Sequence

The `__init__` method follows a **strict sequential boot**:

1. **Declare and load all ROS 2 parameters** (planners, tolerances, BT trees, flags)
2. **Configure `ActionsType`** — set planners and tolerances for each action type
3. **Load behavior tree paths** from package share directory
4. **Create publishers** (stats, route, current_edge, move_action_status)
5. **Wait for topological map** — blocks via `rclpy.spin_once()` until `/topological_map_2` is received
6. **Initialize EdgeActionManager** — passes `ACTIONS`, `rsearch`, and config to the edge executor
7. **Optionally create EdgeReconfigureManager** — if `reconfigure_edges` is `True`
8. **Wait for localisation** — blocks until `closest_node` topic publishes
9. **Optionally connect to restrictions service** — 3-second timeout, degrades gracefully
10. **Create action servers** — `GotoNode` and `ExecutePolicyMode` become available

> **Note**: Steps 5 and 8 are **blocking** — the node will not start action servers until both map and localisation are available. This is a common pattern but creates a rigid startup dependency.

### 3.2 Navigation Mode Selection

When `navigate()` is called (from `executeCallback`), the system chooses between several strategies:

```
navigate(target)
  │
  ├── Is robot close to an edge AND not at a node?
  │     YES → Plan from closest EDGE destination node
  │     NO  → Plan from closest NODE
  │
  ├── Are origin and target the SAME node?
  │     YES → Case 2: to_goal_node() — navigate to the exact pose
  │     NO  → Case 1: Plan route via A*, then:
  │           ├── use_nav2_follow_route == True → navigate_to_poses() [batch]
  │           └── use_nav2_follow_route == False → followRoute() [edge-by-edge]
  │
  └── Origin or target is None?
        YES → Case 3: Abort with error
```

### 3.3 Two Navigation Execution Modes

#### Mode A: `followRoute()` — Edge-by-Edge Execution (Default)

Each edge in the route is executed sequentially:

1. Look up edge and its action type
2. Determine if this is an intermediate or final goal
3. **Reconfigure planner tolerances** via `reconf_movebase()`:
   - Intermediate nodes in fluid navigation: yaw tolerance = `6.283` rad (~360°) — meaning "don't care about orientation"
   - Final goal: use node's `xy_goal_tolerance` and `yaw_goal_tolerance` properties
4. **Start edge reconfiguration** (if enabled)
5. **Execute action** via `EdgeActionManager` (sends `NavigateToPose` to Nav2)
6. On failure: invoke **fail policy** (retry/replan/wait/fail)
7. Publish statistics and proceed to next edge

**Fluid Navigation**: When `fluid_navigation` is `True` on an edge and consecutive edges use navigation actions, the robot doesn't stop at intermediate nodes — it treats them with loose tolerances and the `currentNodeCallback` detects arrival via influence zones, allowing seamless transition to the next edge.

#### Mode B: `navigate_to_poses()` — Batch Execution

All route edges are collected upfront and sent as a single `NavigateThroughPoses` action:

1. Iterate through all edges, build lists of destinations, origins, and actions
2. Optionally **override intermediate node orientations** (if `allow_intermediate_orientation_override` is `True`) — computes yaw angle to face the next node for smoother paths
3. Call `execute_actions()` which delegates to `EdgeActionManager.initialise()` with the full batch, then the edge action manager **segments** the route by action type
4. Reports combined statistics

### 3.4 Fail Policy System

When an edge execution fails (and it's not a preempt), `execute_action_fail_recovery()` kicks in:

- The **fail_policy** field on the edge (a comma-separated string) defines recovery actions
- Format: `"retry_3,replan,fail"` → retry 3 times, then replan, then fail
- Supported policies:
  - `retry` — Re-insert the current edge into the route and try again
  - `replan` — A* search for a new route avoiding the failed edge
  - `wait` — Pause (does not currently advance)
  - `fail` — Stop navigation with failure
- State is tracked in `self.executing_fail_policy` dict (cleared on success)

### 3.5 Edge-from-Closest-Edge Logic

When the robot is not at a node but close to an edge (`closest_edges.distances[0] <= max_dist_to_closest_edge`):

1. `orig_node_from_closest_edge()` selects the best origin based on the two closest edges
2. If both edges are equidistant (bidirectional edge), it picks the destination node that gives the **shorter route** to the final goal
3. `enforce_navigable_route()` ensures the closest edge is included at the start of the route

### 3.6 Planner Reconfiguration

For each edge, the system reconfigures Nav2 planner tolerances:

- `reconf_movebase()` sets `FollowPath.xy_goal_tolerance` and `FollowPath.yaw_goal_tolerance` on the controller server
- Intermediate nodes in fluid mode get relaxed yaw tolerance (6.283 rad)
- The original parameters are saved via `init_reconfigure()` and restored via `reset_reconf()` after the route completes

### 3.7 Restrictions Evaluation

Before executing each edge (in `execute_action`):

1. **Evaluate edge restrictions** via `/restrictions_manager/evaluate_edge` service
2. **Evaluate destination node restrictions** via `/restrictions_manager/evaluate_node` service
3. If either restriction fires → navigation stops with failure

---

## 4. Internal Architecture & Method Reference

### 4.1 Class: `CustomSafeLoader`

A YAML loader that ensures all `x`, `y`, `z`, `w` keys are float-type (not int). This prevents assertion errors in ROS 2 geometry messages.

### 4.2 Class: `TopologicalNavServer` (extends `rclpy.node.Node`)

| Method | Purpose |
|---|---|
| `__init__()` | Full initialization (parameters, subscribers, action servers) |
| `_on_node_shutdown()` | Graceful shutdown — preempts any active navigation |
| `MapCallback()` | Handles `/topological_map_2` updates; rebuilds route search |
| `make_navigation_edge()` | Creates a synthetic edge definition for generic "navigate to pose" |
| `executeCallback()` | `GotoNode` action server callback — main entry point for navigation |
| `executeCallbackexecpolicy()` | `ExecutePolicyMode` action server callback |
| `preemptCallback()` | Cancellation handler for `GotoNode` |
| `preemptCallbackexecpolicy()` | Cancellation handler for `ExecutePolicyMode` |
| `closestNodeCallback()` | Updates `self.closest_node` from localisation |
| `closestEdgesCallback()` | Updates `self.closest_edges` from localisation |
| `currentNodeCallback()` | Updates `self.current_node`; triggers intermediate node detection for fluid navigation |
| `followRoute()` | Edge-by-edge route execution |
| `navigate_to_poses()` | Batch route execution via NavigateThroughPoses |
| `navigate()` | High-level planning and dispatch (Cases 1-3) |
| `execute_policy()` | Dispatches route to `followRoute` or `navigate_to_poses` |
| `to_goal_node()` | Handles case where origin == target |
| `orig_node_from_closest_edge()` | Determines best origin from closest edges |
| `enforce_navigable_route()` | Inserts closest edge at route start if needed |
| `execute_action()` | Single edge execution (restrictions → EdgeActionManager → result) |
| `execute_actions()` | Batch edge execution (for NavigateThroughPoses mode) |
| `execute_action_fail_recovery()` | Wraps `execute_action` with fail policy handling |
| `init_reconfigure()` / `reconf_movebase()` / `reset_reconf()` | Planner tolerance management |
| `edge_reconf_start()` / `edge_reconf_end()` | Per-edge parameter reconfiguration |
| `publish_route()` / `publish_stats()` / `pub_status()` | Publishing helpers |
| `cancel_current_action()` | Cancels the currently executing Nav2 action |

### 4.3 `main()` Function

Creates and wires three ROS 2 nodes in a `MultiThreadedExecutor`:

```python
update_params_control_server = ParameterUpdaterNode("controller_server")
edge_action_manager_server = EdgeActionManager("edge_action_manager")
node = TopologicalNavServer('topological_navigation', ...)
executor = MultiThreadedExecutor()
executor.add_node(update_params_control_server)
executor.add_node(edge_action_manager_server)
executor.add_node(node)
executor.spin()
```

---

## 5. Interaction with Other Scripts

### 5.1 Direct Dependencies

| Module | Usage in `navigation2.py` |
|---|---|
| `route_search2.py` → `TopologicalRouteSearch2` | **A\* route planning**. Instantiated every time the map updates. Provides `search_route()`, `get_node_from_tmap2()`, and `get_edges_between_tmap2()`. |
| `route_search2.py` → `RouteChecker` | **Route validation**. Used in `ExecutePolicyMode` to verify externally-provided routes are valid before execution. |
| `route_search2.py` → `get_route_distance()` | **Distance estimation**. Used in `orig_node_from_closest_edge()` to compare routes. |
| `edge_action_manager2.py` → `EdgeActionManager` | **Edge execution engine**. Manages Nav2 action clients, constructs goals from edge definitions, handles BT trees, segments batched routes, supports in-row operations. Runs as a separate ROS 2 node. |
| `edge_reconfigure_manager2.py` → `EdgeReconfigureManager` | **Per-edge planner reconfiguration**. Reads `config` field from edges, adjusts Nav2 parameters before/after edge traversal. |
| `tmap_utils.py` | **Map utility functions**. Provides `get_edge_from_id_tmap2()`, `get_node_names_from_edge_id_2()`, `get_distance_to_node_tmap2()`, etc. Imported via wildcard (`from ... import *`). |
| `navigation_stats.py` → `nav_stats` | **Timing statistics**. Records start/end times and calculates operation durations for each edge. |
| `scripts/param_processing.py` → `ParameterUpdaterNode` | **Nav2 parameter client**. Gets/sets parameters on the `controller_server` node to adjust planner tolerances at runtime. |
| `scripts/actions_bt.py` → `ActionsType` | **Action type constants and configuration**. Defines action names, BT tree mappings, planner configs, status mappings. |

### 5.2 Upstream Dependencies (Nodes That Feed This Script)

| Node Script | Data Provided | Topic |
|---|---|---|
| `map_manager2.py` | Topological map (YAML string) | `/topological_map_2` |
| `localisation2.py` | Current node, closest node, closest edges | `current_node`, `closest_node`, `closest_edges` |

### 5.3 Downstream Consumers (Nodes/Systems That Use This Script's Outputs)

| Consumer | Data Consumed | Topic |
|---|---|---|
| Any monitoring system | Navigation statistics | `topological_navigation/Statistics` |
| Visualization / external planners | Current route | `topological_navigation/Route` |
| Task planners / orchestrators | Edge being traversed | `current_edge` |
| UI / monitoring dashboards | Action status JSON | `topological_navigation/move_action_status` |
| External clients | Navigation goal result | Via `GotoNode` / `ExecutePolicyMode` action result |

### 5.4 Interaction Diagram

```
                                ┌──────────────────────────┐
                                │   External Task Planner   │
                                │  (sends GotoNode or       │
                                │   ExecutePolicyMode goal)  │
                                └────────────┬─────────────┘
                                             │
                                             ▼
┌───────────────┐   /topological_map_2   ┌──────────────────────────────────┐
│ map_manager2  │ ─────────────────────> │                                  │
│   .py         │                        │     TopologicalNavServer         │
└───────────────┘                        │         (navigation2.py)         │
                                         │                                  │
┌───────────────┐   closest_node,        │  ┌─────────────────────────────┐│
│localisation2  │   current_node,        │  │  TopologicalRouteSearch2    ││
│   .py         │ ─ closest_edges ─────> │  │   (route_search2.py)        ││
└───────────────┘                        │  │   A* planning on dict map   ││
                                         │  └─────────────────────────────┘│
                                         │                                  │
                                         │  ┌─────────────────────────────┐│
                                         │  │    EdgeActionManager        ││
                                         │  │  (edge_action_manager2.py)  ││
                                         │  │  Sends Nav2 actions         ││
                                         │  └──────────┬──────────────────┘│
                                         │             │                    │
                                         │  ┌──────────▼──────────────────┐│
                                         │  │  ParameterUpdaterNode       ││
                                         │  │ (param_processing.py)       ││
                                         │  │ Adjusts controller_server   ││
                                         │  └─────────────────────────────┘│
                                         │                                  │
                                         │  ┌─────────────────────────────┐│
                                         │  │  EdgeReconfigureManager     ││
                                         │  │(edge_reconfigure_manager2)  ││
                                         │  │ Per-edge param overrides    ││
                                         │  └─────────────────────────────┘│
                                         └──────────────┬─────────────────┘
                                                        │
                                                        ▼
                                         ┌──────────────────────────────┐
                                         │   Nav2 Stack                  │
                                         │  (navigate_to_pose /          │
                                         │   navigate_through_poses)     │
                                         └──────────────────────────────┘
```

---

## 6. Map Data Structure — Current State

The topological map is currently stored and consumed as a **raw Python dictionary** (parsed from YAML). The structure looks like:

```python
self.lnodes = {
    "pointset": "map_name",
    "nodes": [
        {
            "meta": {"map": "...", "node": "...", "pointset": "..."},
            "node": {
                "name": "WP1",
                "parent_frame": "map",
                "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                         "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}},
                "properties": {"xy_goal_tolerance": 0.3, "yaw_goal_tolerance": 0.1, ...},
                "edges": [
                    {
                        "edge_id": "WP1_WP2",
                        "node": "WP2",
                        "action": "NavigateToPose",
                        "action_type": "nav2_msgs/action/NavigateToPose",
                        "goal": {"target_pose": {"pose": "$node.pose", "header": {"frame_id": "$node.parent_frame"}}},
                        "fluid_navigation": true,
                        "fail_policy": "retry_3,replan,fail",
                        "config": [],
                        "properties": {}
                    }
                ]
            }
        }
    ]
}
```

Every component (`navigation2.py`, `route_search2.py`, `tmap_utils.py`, `edge_action_manager2.py`) accesses this dictionary with **deep nested key lookups** like:

```python
node["node"]["pose"]["position"]["x"]
node["node"]["properties"]["xy_goal_tolerance"]
edge["action"]
```

This creates significant coupling and fragility.

---

## 7. Migration Plan: Adopting NetworkX Graph Data Structures

### 7.1 Motivation

Currently `route_search2.py` implements A* from scratch using raw dicts. The `networkx_utils.py` module already provides `build_graph_from_tmap()`, `compute_shortest_path()`, and many other utilities for localisation. Extending this to navigation would:

- **Replace custom A\* implementation** with battle-tested `nx.dijkstra_path()`
- **Centralize graph operations** — one graph representation for localisation AND navigation
- **Enable advanced routing** — weighted edges, multi-criteria optimization, avoid-sets
- **Improve performance** for large maps (O((V+E) log V) Dijkstra vs. custom A* with linear search)
- **Reduce code duplication** — `get_node_from_tmap2`, `get_edges_between`, `get_distance_to_node` all have NetworkX equivalents

### 7.2 Step-by-Step Migration

#### Step 1: Build shared NetworkX graph in `MapCallback` (navigation2.py)

**Current**:
```python
def MapCallback(self, msg):
    self.lnodes = yaml.load(msg.data, Loader=CustomSafeLoader)
    self.topol_map = self.lnodes["pointset"]
    self.rsearch = TopologicalRouteSearch2(self.lnodes)
    self.route_checker = RouteChecker(self.lnodes)
```

**Proposed**:
```python
from topological_navigation.networkx_utils import build_graph_from_tmap

def MapCallback(self, msg):
    self.lnodes = yaml.load(msg.data, Loader=CustomSafeLoader)
    self.topol_map = self.lnodes["pointset"]

    # Build NetworkX graph (shared with localisation)
    self._graph = build_graph_from_tmap(self.lnodes, logger=self.get_logger())

    # Keep backward-compatible rsearch for now (phase 2 will replace)
    self.rsearch = TopologicalRouteSearch2(self.lnodes)
    self.route_checker = RouteChecker(self.lnodes)
```

#### Step 2: Add a NetworkX-based route search wrapper

Create a new class or function in `networkx_utils.py` (or a new file `route_search_nx.py`):

```python
from topological_navigation.networkx_utils import compute_shortest_path

def search_route_nx(graph, origin, target, avoid_edges=None, logger=None):
    """
    NetworkX-based route search replacement for TopologicalRouteSearch2.search_route().

    Returns a NavRoute with source[] and edge_id[] populated.
    """
    route = NavRoute()

    if origin == "none" or target == "none" or origin == target:
        return route

    # If we need to avoid edges, temporarily remove them
    if avoid_edges:
        graph = graph.copy()
        for edge_id in avoid_edges:
            for u, v, data in list(graph.edges(data=True)):
                if data.get('edge_id') == edge_id:
                    graph.remove_edge(u, v)

    path = compute_shortest_path(graph, origin, target, weight='weight', logger=logger)

    if len(path) < 2:
        return route

    for i in range(len(path) - 1):
        src, dst = path[i], path[i + 1]
        edge_data = graph.edges[src, dst]
        route.source.append(src)
        route.edge_id.append(edge_data['edge_id'])

    return route
```

#### Step 3: Replace `tmap_utils.py` calls with graph attribute lookups

Create helper functions that operate on the NetworkX graph instead of raw dicts:

| Current (dict-based) | Proposed (NetworkX-based) |
|---|---|
| `get_node_from_tmap2(tmap, name)` — O(n) linear scan | `graph.nodes[name]` — O(1) hash lookup |
| `get_edge_from_id_tmap2(tmap, node, edge_id)` — O(n*m) | `graph.edges[src, dst]` — O(1) or edge attribute query |
| `get_distance_to_node_tmap2(a, b)` — calculates on the fly | `graph.edges[a, b]['weight']` — pre-computed at graph build |
| `get_node_names_from_edge_id_2(tmap, edge_id)` — O(n*m) | Graph edge lookup by edge_id attribute index |

#### Step 4: Refactor `TopologicalRouteSearch2` to use NetworkX internally

Instead of rewriting all consumers at once, wrap the existing API:

```python
class TopologicalRouteSearch2:
    def __init__(self, top_map, graph=None, logger=None):
        self.top_map = top_map
        self.logger = logger
        if graph is not None:
            self.graph = graph
        else:
            self.graph = build_graph_from_tmap(top_map, logger=logger)

        # Keep legacy node dict for backward compatibility
        self.nodes = {}
        for node in self.top_map["nodes"]:
            name = node["node"]["name"]
            self.nodes[name] = node

    def search_route(self, origin, target, avoid_edges=[]):
        return search_route_nx(self.graph, origin, target, avoid_edges, self.logger)

    def get_node_from_tmap2(self, node_name):
        return self.nodes.get(node_name)
```

#### Step 5: Pass graph to EdgeActionManager

Modify `edge_action_manager.init()` to accept the graph:

```python
self.edge_action_manager.init(
    self.ACTIONS, self.rsearch, self.update_params_control_server,
    self.inrow_step_size, self.inrow_step_intermediate_dis,
    graph=self._graph  # NEW: pass NetworkX graph
)
```

#### Step 6: Replace `RouteChecker` with graph validation

```python
def check_route(self, route):
    """Validate route using NetworkX graph."""
    for i, (src, edge_id) in enumerate(zip(route.source, route.edge_id)):
        if src not in self.graph.nodes:
            return False
        # Check edge exists in graph
        for _, dst, data in self.graph.out_edges(src, data=True):
            if data['edge_id'] == edge_id:
                break
        else:
            return False
    return True
```

#### Step 7: Eliminate raw dict deepcopy patterns

The `EdgeActionManager` currently does `yaml.safe_load(json.dumps(edge))` to clone edge dicts. With graph attributes, edge data can be accessed directly:

```python
# Current (edge_action_manager2.py line 303):
self.edge = yaml.safe_load(json.dumps(edge))  # slow deepcopy trick

# Proposed:
self.edge = dict(graph.edges[origin, dest])  # clean copy from graph
```

#### Step 8: Remove `tmap_utils.py` wildcard import

Replace `from topological_navigation.tmap_utils import *` with specific imports, then progressively migrate each function to its NetworkX equivalent or deprecate.

### 7.3 Migration Order and Risk Assessment

| Phase | Change | Risk | Reversibility |
|---|---|---|---|
| 1 | Build graph in MapCallback (additive) | Low | Just remove the line |
| 2 | NetworkX route search wrapper | Low | Falls back to old A* |
| 3 | Pass graph to TopologicalRouteSearch2 | Medium | Constructor has default |
| 4 | Replace tmap_utils calls one by one | Medium | Individual reverts |
| 5 | Pass graph to EdgeActionManager | Medium | Optional parameter |
| 6 | Replace RouteChecker | Low | Self-contained |
| 7 | Eliminate raw dict patterns | High | Requires thorough testing |
| 8 | Remove tmap_utils wildcard import | Low | But needs comprehensive testing |

---

## 8. Code Review — Scripts Needing Refactoring

### 8.1 `navigation2.py` — **HIGH** Priority

**Issues identified**:

1. **Blocking initialization** (lines 192-196, 211-215): Uses `while rclpy.ok(): rclpy.spin_once()` to block until map and localisation arrive. This prevents other callbacks from being processed and doesn't use lifecycle patterns.
   - **Fix**: Use a state machine or lifecycle node to handle startup phases asynchronously.

2. **Massive `__init__`** (~200 lines): Parameter declaration, loading, planner configuration, publisher/subscriber creation, and blocking waits are all in one monolithic constructor.
   - **Fix**: Extract into named methods (`_declare_params()`, `_setup_publishers()`, `_wait_for_map()`, etc.)

3. **Duplicated navigation logic**: `followRoute()` (lines 476-641) and `navigate_to_poses()` (lines 648-838) share ~70% identical code (initialization, origin validation, stats). 
   - **Fix**: Extract common pre-route logic into shared methods.

4. **Raw string comparison for node state**: `self.current_node == "none"`, `self.current_node == "Unknown"` scattered throughout. No enum or constant.
   - **Fix**: Define `NODE_NONE = "none"`, `NODE_UNKNOWN = "Unknown"` constants.

5. **Direct access to `properties` without `.get()`** (line 282): `cnode["node"]["properties"]["xy_goal_tolerance"]` will crash if the property is missing.
   - **Fix**: Use `props.get("xy_goal_tolerance", 0.5)`.

6. **Wildcard import**: `from topological_navigation.tmap_utils import *` — pollutes namespace, unclear dependencies.
   - **Fix**: Use explicit imports.

7. **`rclpy.spin_once(self)` inside service calls** (lines 1241-1258): Blocking spin for restriction evaluation is fragile and can cause reentrancy issues.
   - **Fix**: Use async/await patterns or dedicated callback groups.

### 8.2 `edge_action_manager2.py` — **HIGH** Priority

**Issues identified**:

1. **1,365 lines** — far too large for a single class. Mixes concerns: Nav2 action client management, goal construction, BT tree selection, in-row operations, boundary publishing, orientation adjustment, edge segmentation.
   - **Fix**: Split into: `NavActionClientManager` (already exists as a file but underused), `GoalBuilder`, `RowOperationHandler`, `EdgeSegmenter`.

2. **`yaml.safe_load(json.dumps(edge))`** (line 303) — bizarre pattern to deep-copy a dict. Slow and fragile.
   - **Fix**: Use `copy.deepcopy(edge)` or access graph attributes directly.

3. **Hardcoded string patterns** for node name classification: `_is_row_node_name()`, `_is_waypoint_name()` use string matching with prefixes like `"WayPoint"`, `"wp"`, `"c"`. Brittle for different map conventions.
   - **Fix**: Use edge/node properties (e.g., `"type": "row"`) instead of name parsing.

4. **Multiple executor patterns**: Has its own `SingleThreadedExecutor` for Nav2 clients, plus the main `MultiThreadedExecutor`. Complex threading model.
   - **Fix**: Consolidate executor usage, consider using callback groups more consistently.

### 8.3 `route_search2.py` — **HIGH** Priority

**Issues identified**:

1. **Custom A\* implementation** — 100+ lines of hand-written search that duplicates functionality available in NetworkX (`nx.dijkstra_path`).
   - **Fix**: Replace with NetworkX wrapper per migration plan above.

2. **`RouteChecker` extends `Node`** (line 219) — creates a full ROS 2 node just to validate a route and log. Excessive.
   - **Fix**: Accept a logger parameter instead of extending `Node`.

3. **`get_route_distance()` creates a `TopologicalRouteSearch2` every call** (line 269) — allocates and rebuilds the entire search structure for a single distance query.
   - **Fix**: Accept a pre-built search instance or use NetworkX `compute_path_length`.

4. **Linear search in child node data**: `get_distance_to_node_tmap2` scans the `children` list linearly.
   - **Fix**: Use graph edge weight attribute.

### 8.4 `tmap_utils.py` — **MEDIUM** Priority

**Issues identified**:

1. **O(n) linear scans everywhere**: `get_node_from_tmap2()` scans all nodes, `get_edge_from_id_tmap2()` scans all edges. These are hot-path functions called frequently.
   - **Fix**: Pre-build dicts/graphs. With NetworkX graph, these become O(1) lookups.

2. **Dual API surface**: Every function exists in two variants (tmap1 and tmap2). The tmap1 variants are dead code since ROS 1 support was removed.
   - **Fix**: Remove `get_node()`, `get_distance()`, `get_conected_nodes()`, `get_edges_between()`, `get_edge_from_id()`, `get_node_names_from_edge_id()` (tmap1 variants).

### 8.5 `edge_reconfigure_manager2.py` — **LOW** Priority

**Issues identified**:

1. **Creates `ParameterUpdaterNode` instances on every `initialise()` and `update()` call** — creates and discards ROS 2 nodes repeatedly. Wasteful.
   - **Fix**: Cache `ParameterUpdaterNode` instances by namespace.

2. **Extends `rclpy.node.Node`** but is never added to an executor — its callbacks may never fire.
   - **Fix**: Consider making it a plain class that accepts a logger.

### 8.6 `scripts/param_processing.py` — **LOW** Priority

**Issues identified**:

1. **`rclpy.spin_once(self, executor=self.internal_executor)`** — fragile pattern with private executor inside a node. Can deadlock with the main executor.
   - **Fix**: Use the main executor's async patterns.

### 8.7 `scripts/actions_bt.py` — **LOW** Priority

**Issues identified**:

1. **No configuration file backing** — all constants (planner names, BT trees, status mappings, PD regulator params) are hardcoded in Python.
   - **Fix**: Load from a YAML config file.

2. **String-based action type matching** everywhere — no enum or type-safe mechanism.
   - **Fix**: Use Python `Enum` for action types and robot status.

---

## 9. Summary of Recommended Refactoring Priorities

| Priority | Module | Key Action |
|---|---|---|
| **P0** | `route_search2.py` | Replace custom A* with NetworkX `dijkstra_path`. Eliminates ~150 lines. |
| **P0** | `navigation2.py` → `MapCallback` | Build NetworkX graph alongside existing dict map (additive, zero risk). |
| **P1** | `edge_action_manager2.py` | Split 1,365-line monolith into focused classes. |
| **P1** | `navigation2.py` | Extract shared `followRoute`/`navigate_to_poses` logic; fix unsafe property access. |
| **P1** | `tmap_utils.py` | Remove dead tmap1 functions; mark tmap2 functions for deprecation as NetworkX equivalents are adopted. |
| **P2** | `navigation2.py` | Replace blocking init with lifecycle/state machine. |
| **P2** | `route_search2.py` → `RouteChecker` | Stop extending `Node`; use graph-based validation. |
| **P3** | `edge_reconfigure_manager2.py` | Cache ParameterUpdaterNode instances. |
| **P3** | `actions_bt.py` | Move constants to YAML config; introduce enums. |

---

## 10. Testing Considerations

After refactoring, ensure the following are verified:

- **Unit tests**: Existing tests in `test/test_navigationcore.py` should pass unchanged
- **Route search equivalence**: Verify that NetworkX `dijkstra_path` returns identical routes to the custom A* for all test maps
- **Edge-from-closest-edge logic**: This is subtle — needs targeted tests with bidirectional edges
- **Fail policy parsing**: The comma-separated `fail_policy` string parsing should be tested
- **Batch vs edge-by-edge**: Ensure both modes produce equivalent navigation behavior
- **Restrictions integration**: Test with restrictions service both available and unavailable

---

**Last Updated**: 2026-02-19
**Branch**: aoc_refactor
