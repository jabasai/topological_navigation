# Topological Localisation (localisation2.py) - Technical Documentation

**Version**: 2.1  
**Date**: 2026-02-21  
**Module**: `topological_navigation.scripts.localisation2`  
**Lines of Code**: ~475  
**Test Coverage**: 22 unit tests (`test/test_localisation2.py`)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Class Reference](#3-class-reference)
4. [ROS 2 Interface](#4-ros-2-interface)
5. [Localisation Algorithm](#5-localisation-algorithm)
6. [Configuration](#6-configuration)
7. [Data Flow](#7-data-flow)
8. [Dependencies](#8-dependencies)
9. [Testing](#9-testing)
10. [Troubleshooting](#10-troubleshooting)
11. [Changelog](#11-changelog)

---

## 1. Overview

### Purpose

The **Topological Localisation** node (`TopologicalNavLoc`) determines the
robot's position within a topological map. It answers two questions every
cycle:

1. **Current node** – Which node's influence zone does the robot occupy?
2. **Closest node** – Which node is nearest by Euclidean distance?

The node subscribes to the `/topological_map_2` topic, builds an in-memory
NetworkX directed graph and a scipy KD-tree, then periodically reads the TF
transform between the map frame and the robot's base frame to perform
spatial localisation.

### Key Features

- **O(log n) nearest-neighbour queries** via KD-tree spatial indexing
- **Influence-zone membership** via ray-casting point-in-polygon
- **Topic-based localisation** for nodes detected by external sensors
- **Latched publishing** – only publishes when values change (configurable)
- **Throttled updates** – configurable update rate divisor
- **Service API** – one-shot `localise_pose` queries without TF
- **No-go node filtering** – excludes restricted nodes from results
- **Edge distance reporting** – publishes distances to the two closest edges

---

## 2. Architecture

### System Context

```
┌──────────────────────────────────────────────────────────────────┐
│                   Topological Navigation System                   │
└──────────────────────────────────────────────────────────────────┘

  /topological_map_2 ──►┌───────────────────────┐──► /current_node
  (std_msgs/String)      │  TopologicalNavLoc    │──► /closest_node
                         │  (localisation2.py)   │──► /closest_node_distance
  TF: map → base_link ─►│                       │──► /closest_edges
                         │  NetworkX DiGraph     │──► /current_node/tag
                         │  scipy KD-tree        │
                         └───────────┬───────────┘
                                     │
                          /topological_localisation/
                             localise_pose  (srv)
```

### Internal Components

```
TopologicalNavLoc
│
├── _map_callback()
│   ├── build_graph_from_tmap()     ← networkx_utils
│   ├── build_kdtree_from_graph()   ← networkx_utils
│   └── update_loc_by_topic_nx()    ← networkx_utils
│
├── _pose_callback()                ← timer (1 Hz)
│   ├── TF lookup (map → base_link)
│   ├── get_edge_distances_to_pose()
│   │   └── get_edge_distances_nx() ← networkx_utils
│   ├── determine_current_node()    ← networkx_utils
│   ├── determine_closest_node()    ← networkx_utils
│   ├── _get_node_tag()
│   └── _publish_topics()
│
├── localise_pose_cb()              ← service handler
│   ├── determine_current_node()
│   └── determine_closest_node()
│
└── _get_no_go_nodes()              ← GetTaggedNodes service
```

---

## 3. Class Reference

### `TopologicalNavLoc(Node)`

ROS 2 node that performs topological localisation.

#### Constructor

```python
TopologicalNavLoc(name: str, with_tags: bool = True)
```

| Parameter   | Type   | Default | Description                                     |
|-------------|--------|---------|--------------------------------------------------|
| `name`      | `str`  | —       | ROS 2 node name                                  |
| `with_tags` | `bool` | `True`  | Whether to query `GetTaggedNodes` for no-go nodes |

The constructor:
1. Declares and reads ROS parameters
2. Creates publishers, subscriptions, services
3. Blocks until the first topological map is received
4. Initialises the TF listener and a 1 Hz timer

#### Public Methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `get_edge_distances_to_pose` | `(pose: Pose) → tuple[list, ndarray]` | `(edge_ids, distances)` for all edges |
| `localise_pose_cb` | `(req, res) → res` | Service handler for one-shot localisation |

#### Private Methods

| Method | Description |
|--------|-------------|
| `_map_callback` | Builds graph, KD-tree, and topic-loc config from incoming map |
| `_pose_callback` | Periodic (1 Hz) TF-based localisation loop |
| `_publish_topics` | Publishes all localisation topics (with optional latching) |
| `_get_node_tag` | Returns the first tag string for a node, or `'Unknown'` |
| `_get_no_go_nodes` | Queries `GetTaggedNodes` service for restricted nodes |
| `_make_string_msg` | *(static)* Creates a `std_msgs/String` message |
| `_make_float32_msg` | *(static)* Creates a `std_msgs/Float32` message |

#### Key Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `_graph` | `nx.DiGraph \| None` | NetworkX graph built from tmap |
| `_kdtree` | `KDTree \| None` | scipy KD-tree for spatial queries |
| `_kdtree_node_names` | `list[str]` | Node names indexed in KD-tree order |
| `tmap` | `dict` | Raw topological map YAML data |
| `tmap_frame` | `str` | Parent coordinate frame from map |
| `loc_by_topic` | `list[dict]` | Active topic-based localisations |
| `nogos` | `list[str]` | No-go node names |
| `only_latched` | `bool` | Publish only on value change |
| `throttle_val` | `int` | Update rate divisor (default 3) |

---

## 4. ROS 2 Interface

### Publishers

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/closest_node` | `std_msgs/String` | Transient-local | Name of nearest node |
| `~/closest_node_distance` | `std_msgs/Float32` | Transient-local | Distance to nearest node (m) |
| `~/current_node` | `std_msgs/String` | Transient-local | Node whose zone the robot is in (or `'none'`) |
| `~/closest_edges` | `ClosestEdges` | Transient-local | IDs and distances of 2 closest edges |
| `~/current_node/tag` | `std_msgs/String` | Transient-local | Tag of current closest node |

All publishers use **transient-local durability** so late-joining subscribers
receive the most recent value.

### Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/topological_map_2` | `std_msgs/String` | Transient-local | YAML-encoded topological map |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/topological_localisation/localise_pose` | `LocalisePose` | One-shot localise a given `Pose` |

**Request**: `geometry_msgs/Pose pose`  
**Response**: `string current_node`, `string closest_node`

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `LocalisationThrottle` | `int` | `3` | Only localise every N timer ticks |
| `OnlyLatched` | `bool` | `True` | Publish only when values change |
| `base_frame` | `string` | `"base_link"` | Robot base TF frame |

---

## 5. Localisation Algorithm

### Per-Cycle Flow

```
Timer fires (1 Hz)
    │
    ▼
Look up TF: tmap_frame → base_frame
    │
    ▼
Throttle check (skip if throttle counter ≠ 0)
    │
    ▼
Graph / KD-tree ready?  ──No──► Log warning, skip
    │ Yes
    ▼
Compute closest edges (vectorised numpy)
    │
    ▼
determine_current_node()
    ├── Query KD-tree for 3 nearest nodes
    ├── For each: point_in_poly_nx() (ray-casting)
    ├── Check topic-based localisation list
    └── Return node name or 'none'
    │
    ▼
determine_closest_node()
    ├── KD-tree query for nearest node
    ├── Filter no-go nodes and topic-localised nodes
    └── Return (node_name, distance)
    │
    ▼
Resolve node tag
    │
    ▼
_publish_topics()
    ├── If only_latched: publish only changed values
    └── Else: publish all values unconditionally
```

### Current Node Detection

The "current node" is the node whose **influence zone polygon** contains
the robot's position. The algorithm uses a two-step approach:

1. **KD-tree pre-filter**: Query the 3 nearest nodes by Euclidean distance
   (O(log n) via `scipy.spatial.KDTree`)
2. **Ray-casting check**: For each candidate, test whether the robot's
   pose lies inside the node's influence zone polygon (O(m) per node,
   where m = number of polygon vertices)

This achieves **O(log n + k·m)** complexity instead of the naive O(n·m)
approach of checking all nodes.

### Closest Node Detection

The closest node is determined by KD-tree query with filtering:
- No-go nodes are excluded from results
- Topic-localised nodes are excluded (they use a separate mechanism)
- If the current node is known, it is returned as the closest node

### Topic-Based Localisation

Nodes can be configured with a `localise_by_topic` property containing
a JSON object specifying:

| Field | Description |
|-------|-------------|
| `topic` | ROS topic to subscribe to |
| `field` | Message attribute to read |
| `val` | Expected value for positive detection |
| `localise_anywhere` | Whether detection is position-independent |
| `persistency` | Number of consecutive detections required |

When the topic fires with the expected value, the node is added to
`loc_by_topic`, overriding geometric localisation. A **hysteresis**
mechanism (`persistency`) prevents flickering between topics.

---

## 6. Configuration

### Launch Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topological_navigation',
            executable='localisation2',
            name='topological_localisation',
            parameters=[{
                'LocalisationThrottle': 3,
                'OnlyLatched': True,
                'base_frame': 'base_link',
            }],
        ),
    ])
```

### Required TF

The node expects a valid TF chain from the topological map's
`transformation.child` frame (usually `"map"`) to the robot's
`base_frame` parameter (default `"base_link"`).

### Required Topics

- `/topological_map_2` must be published by the map manager before
  localisation can begin. The node blocks at startup until this topic
  is received.

---

## 7. Data Flow

### Startup Sequence

```
1. Node created
2. Parameters read (throttle, latched, base_frame)
3. Publishers, service, subscription created
4. BLOCK: spin_once() until /topological_map_2 received
5. _map_callback() fires:
   a. Parse YAML → self.tmap
   b. build_graph_from_tmap() → self._graph (NetworkX DiGraph)
   c. build_kdtree_from_graph() → self._kdtree, self._kdtree_node_names
   d. update_loc_by_topic_nx() → topic subscriber config
   e. self.rec_map = True
6. _get_no_go_nodes() → self.nogos (via GetTaggedNodes service)
7. TF listener + 1 Hz timer created
8. Normal operation begins
```

### Runtime Loop

```
Timer tick (1 Hz)
    → _pose_callback()
        → TF lookup
        → determine_current_node() + determine_closest_node()
        → _publish_topics()
```

---

## 8. Dependencies

### Python Packages

| Package | Version | Usage |
|---------|---------|-------|
| `networkx` | ≥ 2.5 | Graph data structure for topological map |
| `scipy` | ≥ 1.5 | KD-tree spatial indexing |
| `numpy` | ≥ 1.19 | Vectorised edge distance calculations |
| `pyyaml` | — | Map YAML parsing |

### ROS 2 Packages

| Package | Components Used |
|---------|-----------------|
| `rclpy` | Node, executors, callback groups, QoS |
| `geometry_msgs` | Pose |
| `std_msgs` | String, Float32 |
| `tf2_ros` | Buffer, TransformListener, TransformException |
| `topological_navigation_msgs` | ClosestEdges, LocalisePose |

### Internal Modules

| Module | Functions Used |
|--------|----------------|
| `networkx_utils` | `build_graph_from_tmap`, `build_kdtree_from_graph`, `determine_current_node`, `determine_closest_node`, `get_edge_distances_nx`, `update_loc_by_topic_nx` |
| `tmap_utils` | `get_node_from_tmap2` |
| `map_types` | `CustomSafeLoader` |

### Optional Dependencies

| Module | Condition | Fallback |
|--------|-----------|----------|
| `topological_navigation_msgs.srv.GetTaggedNodes` | Import attempted at module load | `_HAS_GET_TAGGED_NODES = False`; no-go node detection is disabled |

---

## 9. Testing

### Test File

`test/test_localisation2.py` — 22 unit tests across 7 test classes.

### Test Classes

| Class | Tests | Description |
|-------|-------|-------------|
| `TestImport` | 3 | Module importability and class existence |
| `TestMessageHelpers` | 4 | `_make_string_msg` and `_make_float32_msg` static methods |
| `TestGetNodeTag` | 3 | `_get_node_tag` tag extraction logic |
| `TestGetEdgeDistancesToPose` | 2 | `get_edge_distances_to_pose` delegates to `networkx_utils` |
| `TestPublishTopics` | 4 | Latched vs. unconditional publishing |
| `TestMapCallback` | 3 | Map reception and graph/KD-tree construction |
| `TestLocalisePoseCb` | 3 | `localise_pose_cb` service handler |

### Running Tests

```bash
# All localisation tests
cd topological_navigation
python3 -m pytest test/test_localisation2.py -v

# Single test class
python3 -m pytest test/test_localisation2.py::TestPublishTopics -v

# With coverage (requires pytest-cov)
python3 -m pytest test/test_localisation2.py --cov=topological_navigation.scripts.localisation2 -v
```

### Test Strategy

Tests use **mocked ROS 2 infrastructure** (no `rclpy.init()` required):

- `rclpy` is mocked at import time to avoid ROS middleware dependency
- `TopologicalNavLoc.__init__` is bypassed; attributes are set manually
- `networkx_utils` functions are patched where needed
- Tests exercise pure Python logic without requiring a running ROS graph

---

## 10. Troubleshooting

### Robot Not Localising

| Symptom | Cause | Fix |
|---------|-------|-----|
| "Waiting for the topological map" loops forever | Map manager not publishing | Start `map_manager.py` or verify `/topological_map_2` |
| "TF lookup failed" warnings | Missing TF chain | Check `map → base_link` transform is published |
| `current_node` always `'none'` | Robot outside all influence zones | Check influence zone polygons in map YAML |
| `closest_node` wrong | KD-tree stale after map update | Verify `_map_callback` receives updated map |

### Performance Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| High CPU on large maps | Linear search (old code) | Ensure KD-tree is built (check logs for "KD-tree built") |
| Localisation lag | Throttle too high | Reduce `LocalisationThrottle` parameter |
| Missed detections | Topic-based persistency too high | Lower `persistency` in node `localise_by_topic` property |

### Common Log Messages

| Level | Message | Meaning |
|-------|---------|---------|
| INFO | "Received the topological map" | Map successfully parsed |
| INFO | "Graph built: N nodes, M edges" | NetworkX graph constructed |
| INFO | "KD-tree built with N nodes" | Spatial index ready |
| WARN | "Localisation skipped: graph or KD-tree not ready" | Map not yet processed |
| WARN | "Service …/get_tagged_nodes unavailable" | No-go detection disabled |
| ERROR | "Failed to build the NetworkX graph" | Map YAML is malformed |
| ERROR | "Self-referencing edge …" | Map has an edge pointing to its own node |

---

## 11. Changelog

### v2.0 (2026-02-15) — Major Refactoring

**Imports**:
- Removed 14 unused imports (`sys`, `json`, `tf2_ros` (partial), `pnt2line`,
  `time`, `Bool`, `Thread`, `Event`, `MutuallyExclusiveCallbackGroup`,
  `SingleThreadedExecutor`, `nx`, `KDTree`)
- Replaced wildcard `from topological_navigation.tmap_utils import *` with
  explicit imports (`get_distance`, `get_node_from_tmap2`)
- Made `GetTaggedNodes` import conditional (`_HAS_GET_TAGGED_NODES` flag)

**Methods renamed** (PEP 8 compliance):
| Old Name | New Name |
|----------|----------|
| `MapCallback` | `_map_callback` |
| `pose_callback` | `_pose_callback` |
| `publishTopics` | `_publish_topics` |
| `get_no_go_nodes` | `_get_no_go_nodes` |
| `get_edge_vectors` | `_build_edge_vectors` |
| `Callback` | `topic_localise_callback` |
| `get_string_msgs` | `_make_string_msg` (static) |
| `get_float32_msgs` | `_make_float32_msg` (static) |

**New methods**:
- `_get_node_tag()` – extracted from `_pose_callback` for clarity

**Removed dead code**:
- `update_loc_by_topic()` method (used Python 2 `has_key()`)
- Commented-out subscriber loop
- Unused attributes: `self.node`, `self.node_poses`, `self.rate`,
  `self.service_get_tagged_done_event`, `self.subscribers`

**Bug fixes**:
- Fixed `has_key()` → `in` operator in `topic_localise_callback` (Python 3)
- Fixed "Wating" typo → "Waiting"
- Fixed `warn()` → `warning()` (consistent with Python `logging` API)
- Fixed shadowed builtin: `str` parameter → `text` in `_make_string_msg`

**Quality**:
- Added module-level and method docstrings
- Added type hints to all method signatures
- Added 35 unit tests (`test/test_localisation2.py`)

### v2.1 (2026-02-21) — Dead Code Removal

**Removed unused methods** (no production callers):
- `get_distances_to_pose()` – never called outside tests; `query_nearest_nodes`
  from `networkx_utils` covers the same need
- `_build_edge_vectors()` – legacy backward-compat helper whose outputs
  (`self.vectors_start`, `self.vectors_end`, `self.dist_edge_ids`) were never
  read; edge distances now computed by `get_edge_distances_nx`
- `point_in_poly()` – backward-compat wrapper never called by production code;
  `point_in_poly_nx` is used directly inside `networkx_utils`
- `topic_localise_callback()` – never wired to any ROS subscription

**Removed unused imports**:
- `get_distance` from `tmap_utils`
- `point_in_poly_nx`, `query_nearest_nodes` from `networkx_utils`

**Removed unused state attributes**:
- `self.persist` (only used by `topic_localise_callback`)
- `self.previous_pose` (only used by `topic_localise_callback`)
- `self.nodes_by_topic` (set but never read)

**Tests updated**:
- Removed 4 test classes (13 tests) covering deleted methods
- Updated `_make_loc_node` helper to match trimmed attribute set
- 22 tests remaining, all passing

---

**Last Updated**: 2026-02-21
