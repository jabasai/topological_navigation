# ROS 2 System Flow Report (Topological Navigation)

Date: 2026-02-03

This document expands the system interaction diagram into a detailed ROS 2 flow description, including a **function-call flow**, and provides **refactor guidance**.

---

## 1) System Interaction Diagram (ROS 2)

```mermaid
flowchart LR
  MM[map_manager2\n(map services)] -->|/topological_map_2| LOC[localisation2\n(current/closest)]
  MM -->|/topological_map_2| NAV[navigation2\n(action server)]
  LOC -->|closest_node, current_node, closest_edges| NAV
  NAV -->|edge actions| EAM[edge_action_manager2\n(Nav2 action clients)]
  NAV -->|/topological_navigation| RVIZ[visualise_map_ros2\n(action client)]
  LOC -->|closest_node| POL[get_simple_policy2\n(route services)]
  MM -->|/topological_map_2| TTP[topological_transform_publisher\n(TF static)]
  EAM -->|/topological_navigation/current_destination| OCC[occupancy_checker]
  OCC -->|/topological_navigation/occupied_node| VIS[topological_visual]

  subgraph Nav2
    N2[navigate_to_pose\n navigate_through_poses\n follow_waypoints]
  end
  EAM -->|action clients| N2
```

---

## 2) Node Responsibilities (ROS 2)

### map_manager2
- Loads the topological map (tmap2 YAML).
- Publishes the map as /topological_map_2.
- Hosts map CRUD services under /topological_map_manager2.
- Bridges to legacy /topological_map if enabled.

**Key file**: [topological_navigation/manager2.py](topological_navigation/manager2.py)

### localisation2
- Computes closest and current node based on TF pose and influence zones.
- Publishes closest node, current node, closest edges, and node tag.

**Key file**: [topological_navigation/scripts/localisation2.py](topological_navigation/scripts/localisation2.py)

### navigation2
- Provides the action server for /topological_navigation (GotoNode).
- Plans topological routes and coordinates edge traversal.
- Orchestrates edge action execution via edge_action_manager2.

**Key file**: [topological_navigation/scripts/navigation2.py](topological_navigation/scripts/navigation2.py)

### edge_action_manager2
- Builds Nav2 goals for edge traversal.
- Sends NavigateToPose / NavigateThroughPoses / FollowWaypoints.
- Publishes additional operational status topics.

**Key file**: [topological_navigation/edge_action_manager2.py](topological_navigation/edge_action_manager2.py)

### visualise_map_ros2
- RViz interactive markers for nodes and edges.
- Can send /topological_navigation actions via GotoNode.

**Key file**: [topological_navigation/scripts/visualise_map_ros2.py](topological_navigation/scripts/visualise_map_ros2.py)

### get_simple_policy2
- Route services for policy or external planners.

**Key file**: [topological_navigation/scripts/get_simple_policy2.py](topological_navigation/scripts/get_simple_policy2.py)

### topological_transform_publisher
- Publishes a static transform from tmap2 transformation block.

**Key file**: [topological_navigation/scripts/topological_transform_publisher.py](topological_navigation/scripts/topological_transform_publisher.py)

### occupancy_checker + topological_visual
- Computes occupied nodes from route plans and visualizes occupancy + routes.

**Key files**:
- [topological_navigation/scripts/occupancy_checker.py](topological_navigation/scripts/occupancy_checker.py)
- [topological_navigation/scripts/topological_visual.py](topological_navigation/scripts/topological_visual.py)

---

## 3) Function-Call Flow (ROS 2)

> This is a **call-flow outline** (not an exhaustive call graph). Function names are shown as symbols. File references are links.

### 3.1 map_manager2 (map_manager_2)
**File**: [topological_navigation/manager2.py](topological_navigation/manager2.py)

1. `map_manager_2.__init__()`
   - Declares parameters and loads nav config.
   - Calls `advertise()` to create services.
2. `map_manager_2.init_map()`
   - Loads or initializes tmap2.
   - Publishes /topological_map_2 and optionally /topological_map.
3. Service handlers call CRUD helpers (update node/edge, tags, restrictions).

### 3.2 localisation2 (TopologicalNavLoc)
**File**: [topological_navigation/scripts/localisation2.py](topological_navigation/scripts/localisation2.py)

1. `TopologicalNavLoc.__init__()`
   - Subscribes to /topological_map_2.
   - Initializes TF buffer and periodic pose processing.
2. `MapCallback()`
   - Parses tmap2, caches nodes/edges.
3. `pose_callback()`
   - Gets TF transform.
   - Calls `get_distances_to_pose()` and `get_edge_distances_to_pose()`.
   - Publishes closest/current node and closest edges.

### 3.3 navigation2 (TopologicalNavServer)
**File**: [topological_navigation/scripts/navigation2.py](topological_navigation/scripts/navigation2.py)

1. `TopologicalNavServer.__init__()`
   - Subscribes to /topological_map_2 and waits for map.
   - Creates action servers for `GotoNode` and `ExecutePolicyMode`.
   - Subscribes to localisation topics.
   - Initializes `EdgeActionManager`.
2. `MapCallback()`
   - Updates map and `TopologicalRouteSearch2`.
3. `executeCallback()` (GotoNode)
   - Computes route via `TopologicalRouteSearch2.search_route()`.
   - Calls edge traversal logic in `EdgeActionManager`.
4. `executeCallbackexecpolicy()` (ExecutePolicyMode)
   - Similar structure, but follows policy-specific edge list.

### 3.4 edge_action_manager2 (EdgeActionManager)
**File**: [topological_navigation/edge_action_manager2.py](topological_navigation/edge_action_manager2.py)

1. `EdgeActionManager.init()`
   - Sets dependencies, QoS, and publishers.
2. `EdgeActionManager.initialise()`
   - Processes edge and destination nodes.
   - Chooses action (NavigateToPose or NavigateThroughPoses).
   - Calls `set_nav_client()` to create Nav2 `ActionClient`.
3. `EdgeActionManager.construct_goal()`
   - Substitutes properties into the action goal.
4. `EdgeActionManager.construct_navigate_to_pose_goal()` / `construct_navigate_through_poses_goal()`
   - Builds Nav2 action goals.
5. `EdgeActionManager.preempt()`
   - Cancels the active Nav2 goal when switching actions.

### 3.5 visualise_map_ros2
**File**: [topological_navigation/scripts/visualise_map_ros2.py](topological_navigation/scripts/visualise_map_ros2.py)

1. `TopoMap2Vis.__init__()`
   - Subscribes to /topological_map_2.
   - Creates GotoNode `ActionClient` for interactive markers.
2. `topo_map_cb()`
   - Loads tmap2 and builds RViz markers via `create_map_marker()`.

---

## 4) Suggested Refactor Plan (ROS 2)

### Phase 1 — Inventory & Boundaries
**Goal**: separate ROS 2 core from ROS 1/legacy utilities.
- Create a dedicated ROS 2 package module boundary (e.g., topological_navigation_ros2).
- Move ROS 1 scripts and dependencies into a legacy namespace.
- Add clear ROS 2 README sections with node entry points and topics.

### Phase 2 — API & Data Flow Cleanup
**Goal**: simplify and make dependencies explicit.
- Introduce typed dataclasses for tmap2 nodes/edges in a new module.
- Replace raw dict access with typed getters.
- Centralize map parsing in a single module (used by navigation/localisation/visualization).

### Phase 3 — Action & Edge Execution Simplification
**Goal**: reduce complexity inside edge_action_manager2.
- Split `EdgeActionManager` into smaller components:
  - `ActionClientFactory`
  - `GoalBuilder`
  - `RowOperationHandler`
- Move Nav2 action client handling into a dedicated class.

### Phase 4 — Testing & Stability
**Goal**: safer refactors.
- Add unit tests for `TopologicalRouteSearch2` and goal construction.
- Add integration tests for node orchestration (map → localisation → navigation).
- Add ROS 2 launch tests for basic navigation stack.

---

## 5) Suggested Fixes (Short-Term)

1. **Reduce duplication of map parsing**
   - Several nodes parse /topological_map_2 independently. Centralize in a shared helper.

2. **Explicit message schema validations**
   - Validate missing fields before action execution to avoid runtime KeyErrors.

3. **Parameter handling consistency**
   - Normalize parameter names and defaults across navigation2 and edge_action_manager2.

4. **Logging and error clarity**
   - Improve action server error logs to include edge id and node names.

5. **Topic naming consistency**
   - Document public topics and keep consistent namespaces under /topological_navigation/.

---

## 6) Quick Start for Refactor Work

1. Start with **map parsing consolidation** (least risky).
2. Move **ActionClient creation** into a factory class.
3. Split **EdgeActionManager** goal construction from execution.
4. Add **unit tests** for each small refactor step.

---

If you want a deeper, full call graph (function-by-function) I can generate a static analysis report next.