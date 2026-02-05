# Topological Map Manager 2 (manager2.py) - Technical Documentation

**Version**: 2.0  
**Date**: 2026-02-04  
**Author**: Adam Binch (abinch@sagarobotics.com)  
**Module**: `topological_navigation.manager2`  
**Lines of Code**: 1,526  

---

## Table of Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Class Diagrams](#3-class-diagrams)
4. [Method Call Graph](#4-method-call-graph)
5. [API Reference](#5-api-reference)
6. [Data Structures](#6-data-structures)
7. [Static Analysis](#7-static-analysis)
8. [Improvement Recommendations](#8-improvement-recommendations)
9. [Usage Examples](#9-usage-examples)

---

## 1. Overview

### Purpose

The **Topological Map Manager 2** (`map_manager_2` class) is a ROS 2 node responsible for:

- **Loading and storing** topological maps from YAML files
- **Managing map lifecycle**: Creating, updating, switching, and persisting maps
- **Providing ROS 2 services** for CRUD operations on nodes and edges
- **Publishing map data** to topics for consumption by navigation and visualization nodes
- **Validating map structure** to ensure consistency and integrity
- **Broadcasting TF transforms** between map coordinate frames

### Key Features

- **Comprehensive service API**: 40+ ROS 2 services for map manipulation
- **Automatic map persistence**: Optional auto-write on modifications
- **Map caching**: Cache maps in `~/.ros/topological_maps/`
- **Legacy format support**: Converts tmap2 format to legacy TopologicalMap messages
- **Validation**: Checks for duplicate nodes, missing edges, and structural integrity
- **Batch operations**: Multi-node/edge addition for efficiency
- **Tag-based queries**: Support for semantic tagging and retrieval

---

## 2. Architecture

### System Context

```
┌─────────────────────────────────────────────────────────────────┐
│                    Topological Navigation System                 │
└─────────────────────────────────────────────────────────────────┘

                    ┌──────────────────┐
                    │  map_manager_2   │
                    │  (manager2.py)   │
                    │  1526 lines      │
                    └────────┬─────────┘
                             │
                             │ Publishes
                             │
         ┌───────────────────┼──────────────────────┐
         │                   │                      │
         ▼                   ▼                      ▼
┌────────────────┐  ┌─────────────────┐  ┌──────────────────────┐
│ /topological_  │  │ TF2 Broadcaster │  │ /topological_map     │
│ map_2 (String) │  │ (Transform)     │  │ (TopologicalMap)     │
│ JSON format    │  │                 │  │ Legacy format        │
└────────────────┘  └─────────────────┘  └──────────────────────┘
         │                                          │
         │ Consumed by                              │ Consumed by
         ▼                                          ▼
┌────────────────┐                        ┌──────────────────────┐
│ navigation2.py │                        │ Legacy components    │
│ localisation2  │                        │ (if enabled)         │
│ visualise_map  │                        └──────────────────────┘
└────────────────┘

         ┌──────────────────────────────────────────┐
         │  ROS 2 Services (40+ services)           │
         ├──────────────────────────────────────────┤
         │  Query:                                   │
         │  - get_topological_map                    │
         │  - get_tagged_nodes                       │
         │  - get_tags                               │
         │  - get_node_tags                          │
         │  - get_edges_between_nodes                │
         │                                           │
         │  Modify:                                  │
         │  - add_topological_node                   │
         │  - remove_topological_node                │
         │  - add_edges_between_nodes                │
         │  - remove_edge                            │
         │  - update_node_name                       │
         │  - update_node_pose                       │
         │  - update_node_tolerance                  │
         │  - modify_node_tags                       │
         │  - add_tag_to_node                        │
         │  - rm_tag_from_node                       │
         │  - update_node_restrictions               │
         │  - update_edge_restrictions               │
         │  - update_edge                            │
         │  - update_action                          │
         │  - set_node_influence_zone                │
         │  - clear_topological_nodes                │
         │                                           │
         │  Batch:                                   │
         │  - add_topological_node_multi             │
         │  - add_edges_between_nodes_multi          │
         │  - add_param_to_edge_config_multi         │
         │  - set_node_influence_zone_multi          │
         │                                           │
         │  Persistence:                             │
         │  - write_topological_map                  │
         │  - switch_topological_map                 │
         └──────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Responsibility | Interaction |
|-----------|----------------|-------------|
| **map_manager_2** | Central map management | Loads, stores, validates, and publishes maps |
| **YAML Loader** | Parse map files | Uses CustomSafeLoader for proper float handling |
| **Service Handlers** | ROS 2 API | 40+ callback methods wrapping core operations |
| **TF2 Broadcaster** | Coordinate transforms | Publishes parent→child frame transforms |
| **Map Publisher** | Data dissemination | Publishes JSON map to `/topological_map_2` topic |
| **Validator** | Integrity checks | Detects duplicates, missing nodes, circular edges |

---

## 3. Class Diagrams

### Main Class Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                        map_manager_2                             │
├─────────────────────────────────────────────────────────────────┤
│  Inherits: rclpy.node.Node                                      │
├─────────────────────────────────────────────────────────────────┤
│  - tmap2: dict                      # Main map data structure   │
│  - name: str                        # Map name                  │
│  - metric_map: str                  # Associated metric map     │
│  - pointset: str                    # Pointset identifier       │
│  - transformation: dict             # TF parent/child frames    │
│  - filename: str                    # File path                 │
│  - loaded: bool                     # Load status               │
│  - cache_maps: bool                 # Enable caching            │
│  - auto_write: bool                 # Auto-save on changes      │
│  - map_ok: bool                     # Validation status         │
│  - names: list                      # Node name cache           │
│  - goal_mappings: dict              # Action→Goal mappings      │
│  - cache_dir: str                   # Cache directory path      │
│  - map_pub: Publisher               # Map topic publisher       │
│  - broadcaster: TransformBroadcaster # TF broadcaster           │
│  - convert_to_legacy: bool          # Legacy format support     │
├─────────────────────────────────────────────────────────────────┤
│  + __init__(advertise_srvs=True)                                │
│  + advertise()                                                   │
│  + init_map(name, metric_map, pointset, transformation, ...)    │
│  + load_map(filename)                                            │
│  + write_topological_map(filename, no_alias=False)              │
│  + update(update_time=True)                                      │
│  + broadcast_transform()                                         │
│  + create_list_of_nodes()                                        │
│  + get_instances_of_node(node_name)                             │
│  + map_check()                                                   │
│  + tmap2_to_tmap()                                               │
│                                                                  │
│  # Node Operations                                              │
│  + add_node(name, pose, localise_by_topic, verts, ...)          │
│  + remove_node(node_name, update, write_map)                    │
│  + update_node_name(node_name, new_name, update, write_map)     │
│  + update_node_waypoint(name, pose_msg, update, write_map)      │
│  + update_node_tolerance(name, new_xy, new_yaw, ...)            │
│  + generate_circle_vertices(radius, number)                     │
│  + get_new_name()                                                │
│                                                                  │
│  # Edge Operations                                              │
│  + add_edge(origin, destination, action, action_type, ...)      │
│  + add_edge_to_node(origin, destination, action, edge_id, ...)  │
│  + remove_edge(edge_name, update, write_map)                    │
│  + update_edge(edge_id, action_name, action_type, goal, ...)    │
│  + set_goal(action, action_type, _goal)                         │
│  + set_action_type(action)                                       │
│  + get_edges_between(nodea, nodeb)                              │
│                                                                  │
│  # Tag Operations                                               │
│  + modify_tag_cb(msg)                                            │
│  + add_tag_cb(msg)                                               │
│  + rm_tag_cb(msg)                                                │
│                                                                  │
│  # Restrictions Operations                                      │
│  + update_node_restrictions(node_name, restrictions_planning, ...)│
│  + update_edge_restrictions(edge_id, restrictions_planning, ...) │
│                                                                  │
│  # Configuration Operations                                     │
│  + add_param_to_edge_config(edge_id, namespace, name, value, ...)│
│  + rm_param_from_edge_config(edge_id, namespace, name, ...)     │
│  + rm_param_from_topological_map(namespace, name, ...)          │
│                                                                  │
│  # Batch Operations                                             │
│  + add_topological_nodes(data, update, write_map)               │
│  + add_edges(data, update, write_map)                           │
│  + add_params_to_edges(data, update, write_map)                 │
│  + set_influence_zones(data, update, write_map)                 │
│                                                                  │
│  # Query Service Callbacks (10)                                 │
│  + get_topological_map_cb(req)                                   │
│  + get_tagged_cb(req, res)                                       │
│  + get_tags_cb(req)                                              │
│  + get_node_tags_cb(req)                                         │
│  + get_edges_between_cb(req)                                     │
│  + ... (40+ service callbacks total)                            │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    Helper Classes                                │
├─────────────────────────────────────────────────────────────────┤
│  NoAliasDumper (yaml.SafeDumper)                                │
│  - ignore_aliases(data) -> True                                 │
│                                                                  │
│  CustomSafeLoader (yaml.SafeLoader) [from map_types]            │
│  - Ensures proper float type handling                           │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                    Utility Functions                             │
├─────────────────────────────────────────────────────────────────┤
│  pose_dist(pose1, pose2) -> float                               │
│  - Calculates Euclidean distance between two poses              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Method Call Graph

### Initialization Flow

```
main() [external]
    │
    └──► map_manager_2.__init__(advertise_srvs=True)
            │
            ├──► self.get_parameter_or("~cache_topological_maps", ...)
            ├──► self.get_parameter_or("~auto_write_topological_maps", ...)
            ├──► self.get_parameter_or("nav_config", ...)
            ├──► yaml.safe_load(navigation_goal.yaml)
            │
            └──► self.advertise()
                    │
                    ├──► self.create_service(...) × 40+ times
                    │    (Creates all ROS 2 service endpoints)
                    │
                    └──► Return
```

### Map Loading Flow

```
init_map(name, metric_map, pointset, transformation, filename, load=True)
    │
    ├──► self.load_map(filename)  [if load=True]
    │       │
    │       ├──► multiprocessing.Process(target=loader, ...)
    │       │       │
    │       │       └──► loader(filename, transporter)
    │       │               │
    │       │               ├──► open(filename, "r")
    │       │               ├──► yaml.load(..., Loader=CustomSafeLoader)
    │       │               └──► transporter["tmap2"] = loaded_data
    │       │
    │       ├──► self.tmap2 = transporter["tmap2"]
    │       ├──► Validate map type (dict expected)
    │       ├──► self.name = self.tmap2["name"]
    │       ├──► self.metric_map = self.tmap2["metric_map"]
    │       ├──► self.pointset = self.tmap2["pointset"]
    │       ├──► self.transformation = self.tmap2["transformation"]
    │       ├──► self.names = self.create_list_of_nodes()
    │       └──► self.map_check()
    │
    ├──► self.map_pub = self.create_publisher(String, '/topological_map_2', qos)
    ├──► self.map_pub.publish(json.dumps(self.tmap2))
    ├──► self.names = self.create_list_of_nodes()
    ├──► self.broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
    ├──► self.broadcast_transform()
    │
    └──► [if convert_to_legacy] self.tmap2_to_tmap()
```

### Node Addition Flow

```
add_topological_node_cb(req)
    │
    └──► add_topological_node(node_name, node_pose, add_close_nodes)
            │
            ├──► [if node_name empty] self.get_new_name()
            ├──► [if name in self.names] return False (duplicate)
            ├──► rosidl_runtime_py.message_to_ordereddict(node_pose)
            │
            ├──► [if add_close_nodes] Find nodes within dist=8.0
            │       │
            │       └──► pose_dist(pose1, pose2) for each existing node
            │
            ├──► self.add_node(name, pose)
            │       │
            │       ├──► Create node dict structure:
            │       │    {
            │       │      "meta": {map, node, pointset, tag[]},
            │       │      "node": {
            │       │        name, pose, edges[], localise_by_topic,
            │       │        verts[], properties{}, restrictions_*, parent_frame
            │       │      }
            │       │    }
            │       │
            │       ├──► [if verts="default"] self.generate_circle_vertices()
            │       ├──► [if properties="default"] Set xy/yaw tolerances from nav_config
            │       └──► self.tmap2["nodes"].append(node)
            │
            ├──► [for each close_node] self.add_edge(name, close_node, ...)
            │
            ├──► [if update] self.update()
            │       │
            │       ├──► [if update_time] Update tmap2["last_updated"]
            │       ├──► self.map_pub.publish(json.dumps(self.tmap2))
            │       ├──► self.names = self.create_list_of_nodes()
            │       ├──► self.map_check()
            │       └──► [if convert_to_legacy] self.tmap2_to_tmap()
            │
            └──► [if auto_write and write_map] self.write_topological_map(filename)
```

### Edge Addition Flow

```
add_edge_cb(req)
    │
    └──► add_edge(origin, destination, action, action_type, edge_id)
            │
            ├──► num_available, index = self.get_instances_of_node(origin)
            │
            ├──► [if num_available == 1]
            │       │
            │       ├──► self.add_edge_to_node(origin, destination, action, ...)
            │       │       │
            │       │       ├──► Create edge dict:
            │       │       │    {
            │       │       │      edge_id, node, action, action_type, goal,
            │       │       │      config[], recovery_behaviours_config,
            │       │       │      fail_policy, restrictions_*, fluid_navigation
            │       │       │    }
            │       │       │
            │       │       ├──► [if edge_id="default"] Generate: origin+"_"+destination
            │       │       ├──► [if not action_type] self.set_action_type(action)
            │       │       ├──► the_action_type, the_goal = self.set_goal(action, action_type, goal)
            │       │       │       │
            │       │       │       ├──► [if action in goal_mappings] Use cached mapping
            │       │       │       └──► [else] Load goal template from navigation_goal.yaml
            │       │       │
            │       │       └──► self.tmap2["nodes"][index]["node"]["edges"].append(edge)
            │       │
            │       ├──► [if update] self.update()
            │       └──► [if auto_write and write_map] self.write_topological_map(filename)
            │
            └──► [else] Log error: node not found or duplicate
```

### Map Validation Flow

```
map_check()
    │
    ├──► Check 1: All nodes have same pointset
    │       │
    │       └──► [if multiple pointsets] self.map_ok = False, log warning
    │
    ├──► Check 2: No duplicate node names
    │       │
    │       ├──► self.create_list_of_nodes()
    │       └──► [for each name] Count occurrences, warn if > 1
    │
    ├──► Check 3: No duplicate edges
    │       │
    │       ├──► Build edge_ids list: [origin_UUID_destination, ...]
    │       └──► [for each edge] Count occurrences, warn if > 1
    │
    ├──► Check 4: Edge destinations exist
    │       │
    │       └──► [for each edge] Check if destination in names
    │
    └──► Check 5: No self-referencing edges
            │
            └──► [for each edge] Warn if origin == destination
```

### Service Call Pattern

All service callbacks follow this pattern:

```
{operation}_cb(req) [Service Callback]
    │
    ├──► Extract parameters from req
    │
    ├──► {operation}(params, update=True, write_map=True) [Core Implementation]
    │       │
    │       ├──► Validate parameters
    │       ├──► num_available, index = self.get_instances_of_node(node_name)
    │       ├──► [if num_available == 1] Perform operation on self.tmap2
    │       ├──► [if update] self.update()
    │       └──► [if auto_write and write_map] self.write_topological_map(filename)
    │
    └──► Return response with success status
```

---

## 5. API Reference

### Core Methods

#### Initialization

**`__init__(advertise_srvs=True)`**
- **Purpose**: Initialize the map manager node
- **Parameters**:
  - `advertise_srvs` (bool): Whether to create ROS 2 service endpoints
- **Side Effects**:
  - Loads navigation goal configuration
  - Creates cache directory if needed
  - Calls `advertise()` if requested

**`advertise()`**
- **Purpose**: Create all 40+ ROS 2 service endpoints
- **Services Created**:
  - Query services: 5
  - Modification services: 20+
  - Batch operation services: 4
  - Persistence services: 2

**`init_map(name, metric_map, pointset, transformation, filename, load=True)`**
- **Purpose**: Initialize or load a topological map
- **Parameters**:
  - `name` (str): Map name
  - `metric_map` (str): Associated 2D metric map name
  - `pointset` (str): Pointset identifier
  - `transformation` (dict|str): TF transform or "default"
  - `filename` (str): Path to map YAML file
  - `load` (bool): Whether to load from file
- **Side Effects**:
  - Creates publisher on `/topological_map_2`
  - Starts TF broadcaster
  - Caches node names
- **Returns**: None

#### Map Operations

**`load_map(filename)`**
- **Purpose**: Load topological map from YAML file
- **Parameters**:
  - `filename` (str): Absolute path to `.tmap2.yaml` file
- **Side Effects**:
  - Sets `self.tmap2`, `self.name`, `self.metric_map`, etc.
  - Runs `map_check()` validation
  - Caches map if `cache_maps=True`
- **Raises**: Logs error if file not found or invalid format
- **Implementation**: Uses multiprocessing to prevent blocking

**`write_topological_map(filename, no_alias=False)`**
- **Purpose**: Save topological map to YAML file
- **Parameters**:
  - `filename` (str): Output file path
  - `no_alias` (bool): Whether to disable YAML aliases
- **Side Effects**:
  - Sorts nodes alphabetically by name
  - Writes formatted YAML to disk
- **Returns**: None

**`update(update_time=True)`**
- **Purpose**: Update map state and notify subscribers
- **Parameters**:
  - `update_time` (bool): Whether to update last_updated timestamp
- **Side Effects**:
  - Publishes map to `/topological_map_2` topic
  - Refreshes cached node names
  - Runs validation checks
  - Converts to legacy format if enabled
- **Returns**: None

**`broadcast_transform()`**
- **Purpose**: Publish TF transform between map frames
- **Side Effects**:
  - Publishes `TransformStamped` message to TF tree
- **Returns**: None

#### Node Management

**`add_node(name, pose, localise_by_topic="", verts="default", properties="default", tags=[], restrictions_planning="True", restrictions_runtime="True")`**
- **Purpose**: Add a new node to the map
- **Parameters**:
  - `name` (str): Unique node name
  - `pose` (dict): Position and orientation
  - `localise_by_topic` (str): Topic for localization override
  - `verts` (list|str): Influence zone vertices or "default"
  - `properties` (dict|str): Node properties or "default"
  - `tags` (list): Semantic tags
  - `restrictions_*` (str): Planning/runtime restrictions
- **Side Effects**:
  - Appends node to `self.tmap2["nodes"]`
- **Returns**: None

**`remove_node(node_name, update=True, write_map=True)`**
- **Purpose**: Remove a node and all associated edges
- **Parameters**:
  - `node_name` (str): Name of node to remove
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Side Effects**:
  - Removes node from map
  - Removes all edges pointing to this node
- **Returns**: bool (success)

**`update_node_name(node_name, new_name, update=True, write_map=True)`**
- **Purpose**: Rename a node and update all edge references
- **Parameters**:
  - `node_name` (str): Current node name
  - `new_name` (str): New node name
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Side Effects**:
  - Updates node name in node definition
  - Updates all edge destinations referencing this node
- **Returns**: bool (success)

**`update_node_waypoint(name, pose_msg, update=True, write_map=True)`**
- **Purpose**: Update a node's pose
- **Parameters**:
  - `name` (str): Node name
  - `pose_msg` (Pose): New pose
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Returns**: bool (success)

**`update_node_tolerance(name, new_xy, new_yaw, update=True, write_map=True)`**
- **Purpose**: Update goal tolerance properties
- **Parameters**:
  - `name` (str): Node name
  - `new_xy` (float): XY goal tolerance in meters
  - `new_yaw` (float): Yaw goal tolerance in radians
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Returns**: bool (success)

**`generate_circle_vertices(radius=0.75, number=8)`**
- **Purpose**: Generate circular influence zone vertices
- **Parameters**:
  - `radius` (float): Circle radius in meters
  - `number` (int): Number of vertices
- **Returns**: list of dicts with x, y coordinates

**`get_new_name()`**
- **Purpose**: Generate unique node name (WayPointN)
- **Returns**: str (e.g., "WayPoint42")

#### Edge Management

**`add_edge(origin, destination, action, action_type, edge_id, update=True, write_map=True)`**
- **Purpose**: Add an edge between two nodes
- **Parameters**:
  - `origin` (str): Source node name
  - `destination` (str): Target node name
  - `action` (str): Action name (e.g., "NavigateToPose")
  - `action_type` (str): ROS 2 action type
  - `edge_id` (str): Unique edge identifier
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Returns**: bool (success)

**`add_edge_to_node(origin, destination, action="", edge_id="default", config=[], recovery_behaviours_config="", action_type="", goal=None, fail_policy="fail", restrictions_planning="True", restrictions_runtime="True", fluid_navigation=True)`**
- **Purpose**: Add edge with full configuration
- **Parameters**: (comprehensive edge configuration)
- **Side Effects**:
  - Appends edge to origin node's edges list
- **Returns**: None

**`remove_edge(edge_name, update=True, write_map=True)`**
- **Purpose**: Remove an edge by edge_id
- **Parameters**:
  - `edge_name` (str): Edge ID to remove
  - `update` (bool): Whether to call `update()`
  - `write_map` (bool): Whether to auto-save
- **Returns**: bool (success)

**`update_edge(edge_id, action_name, action_type, goal, fail_policy, not_fluid, update=True, write_map=True)`**
- **Purpose**: Update edge properties
- **Returns**: bool (success)

**`set_goal(action, action_type, _goal=None)`**
- **Purpose**: Set or retrieve action goal template
- **Parameters**:
  - `action` (str): Action name
  - `action_type` (str): Action type
  - `_goal` (dict|None): Goal template or None to load default
- **Returns**: tuple (action_type, goal_dict)

**`set_action_type(action)`**
- **Purpose**: Infer action_type from action name
- **Example**: "navigate_to_pose" → "navigate_to_pose_msgs/NavigateToPoseGoal"
- **Returns**: str (action_type)

**`get_edges_between(nodea, nodeb)`**
- **Purpose**: Find all edges between two nodes (bidirectional)
- **Returns**: tuple (edges_ab[], edges_ba[])

#### Tag Management

**`modify_tag_cb(msg)`, `add_tag_cb(msg)`, `rm_tag_cb(msg)`**
- **Purpose**: Modify, add, or remove tags from nodes
- **Parameters**: Message with node list and tag
- **Returns**: Response with success status

#### Restrictions Management

**`update_node_restrictions(node_name, restrictions_planning, restrictions_runtime, update_edges, update=True, write_map=True)`**
- **Purpose**: Update node navigation restrictions
- **Parameters**:
  - `restrictions_planning` (str): Planning-time restrictions
  - `restrictions_runtime` (str): Runtime restrictions
  - `update_edges` (bool): Whether to apply to all edges

**`update_edge_restrictions(edge_id, restrictions_planning, restrictions_runtime, update=True, write_map=True)`**
- **Purpose**: Update edge navigation restrictions
- **Returns**: bool (success)

#### Configuration Management

**`add_param_to_edge_config(edge_id, namespace, name, value, value_is_string, not_reset, update=True, write_map=True)`**
- **Purpose**: Add dynamic reconfigure parameter to edge
- **Parameters**:
  - `namespace` (str): Parameter namespace
  - `name` (str): Parameter name
  - `value` (str): Parameter value
  - `value_is_string` (bool): Type hint
  - `not_reset` (bool): Whether to reset after edge
- **Returns**: bool (success)

**`rm_param_from_edge_config(edge_id, namespace, name, update=True, write_map=True)`**
- **Purpose**: Remove parameter from edge config

**`rm_param_from_topological_map(namespace, name, update=True, write_map=True)`**
- **Purpose**: Remove parameter from all edges in map

#### Batch Operations

**`add_topological_nodes(data, update=True, write_map=True)`**
- **Purpose**: Add multiple nodes efficiently
- **Parameters**:
  - `data` (list): List of node specifications
- **Returns**: bool (success)

**`add_edges(data, update=True, write_map=True)`**
- **Purpose**: Add multiple edges efficiently

**`add_params_to_edges(data, update=True, write_map=True)`**
- **Purpose**: Add parameters to multiple edges

**`set_influence_zones(data, update=True, write_map=True)`**
- **Purpose**: Set influence zones for multiple nodes

#### Utility Methods

**`get_instances_of_node(node_name)`**
- **Purpose**: Find node in map and return count/index
- **Returns**: tuple (num_available, index)

**`map_check()`**
- **Purpose**: Validate map structure
- **Checks**:
  1. All nodes have same pointset
  2. No duplicate node names
  3. No duplicate edges
  4. All edge destinations exist
  5. No self-referencing edges
- **Side Effects**: Sets `self.map_ok` flag

**`create_list_of_nodes()`**
- **Purpose**: Extract list of node names from map
- **Returns**: list of strings

**`get_time()`**
- **Purpose**: Get current timestamp for map updates
- **Returns**: str (ISO format: YYYY-MM-DD_HH-MM-SS)

**`tmap2_to_tmap()`**
- **Purpose**: Convert tmap2 to legacy TopologicalMap message

**`convert_tmap2_to_tmap(cls, tmap2, pointset, metric_map)` [classmethod]**
- **Purpose**: Static conversion utility
- **Returns**: TopologicalMap message

---

## 6. Data Structures

### Topological Map (tmap2) Structure

```yaml
name: "my_map"
metric_map: "map_2d"
pointset: "my_map"
last_updated: "2026-02-04_10-30-00"
transformation:
  parent: "map"
  child: "topo_map"
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
nodes:
  - meta:
      map: "map_2d"
      node: "NodeA"
      pointset: "my_map"
      tag: ["charging_station", "safe_zone"]
    node:
      name: "NodeA"
      parent_frame: "map"
      localise_by_topic: ""
      pose:
        position:
          x: 1.0
          y: 2.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
      verts:
        - x: 0.75
          y: 0.0
        - x: 0.53
          y: 0.53
        - x: 0.0
          y: 0.75
        - x: -0.53
          y: 0.53
        - x: -0.75
          y: 0.0
        - x: -0.53
          y: -0.53
        - x: 0.0
          y: -0.75
        - x: 0.53
          y: -0.53
      properties:
        xy_goal_tolerance: 0.3
        yaw_goal_tolerance: 0.1
        max_speed: 0.5
        custom_property: "value"
      restrictions_planning: "True"
      restrictions_runtime: "True"
      edges:
        - edge_id: "NodeA_NodeB"
          node: "NodeB"
          action: "NavigateToPose"
          action_type: "nav2_msgs/action/NavigateToPose"
          goal:
            target_pose:
              header:
                frame_id: "$node.parent_frame"
              pose: "$node.pose"
          config:
            - namespace: "/move_base"
              name: "max_vel_x"
              value: "0.5"
              reset: true
          recovery_behaviours_config: ""
          fail_policy: "fail"
          restrictions_planning: "True"
          restrictions_runtime: "True"
          fluid_navigation: true
```

### Node Dictionary Structure

```python
node = {
    "meta": {
        "map": str,          # Associated metric map
        "node": str,         # Node name (duplicate of node.name)
        "pointset": str,     # Pointset identifier
        "tag": [str, ...]    # Optional semantic tags
    },
    "node": {
        "name": str,         # Unique node identifier
        "parent_frame": str, # TF parent frame (usually "map")
        "localise_by_topic": str,  # Topic for localization override
        "pose": {
            "position": {"x": float, "y": float, "z": float},
            "orientation": {"x": float, "y": float, "z": float, "w": float}
        },
        "verts": [           # Influence zone polygon
            {"x": float, "y": float},
            ...
        ],
        "properties": {      # Flexible metadata
            "xy_goal_tolerance": float,
            "yaw_goal_tolerance": float,
            # ... additional properties
        },
        "restrictions_planning": str,  # Planning restrictions expression
        "restrictions_runtime": str,   # Runtime restrictions expression
        "edges": [           # Outgoing edges
            # ... (see Edge structure below)
        ]
    }
}
```

### Edge Dictionary Structure

```python
edge = {
    "edge_id": str,      # Unique edge identifier (typically "origin_destination")
    "node": str,         # Destination node name
    "action": str,       # Action name (e.g., "NavigateToPose")
    "action_type": str,  # ROS 2 action type (e.g., "nav2_msgs/action/NavigateToPose")
    "goal": {            # Action goal template with variable substitution
        "target_pose": {
            "header": {
                "frame_id": "$node.parent_frame"  # Substituted at runtime
            },
            "pose": "$node.pose"  # Substituted at runtime
        }
    },
    "config": [          # Dynamic reconfigure parameters
        {
            "namespace": str,
            "name": str,
            "value": str,
            "reset": bool
        },
        ...
    ],
    "recovery_behaviours_config": str,  # Recovery behavior configuration
    "fail_policy": str,  # Failure handling policy
    "restrictions_planning": str,  # Planning restrictions
    "restrictions_runtime": str,   # Runtime restrictions
    "fluid_navigation": bool       # Fluid navigation flag
}
```

---

## 7. Static Analysis

### Metrics

| Metric | Value | Assessment |
|--------|-------|------------|
| **Total Lines** | 1,526 | Large module |
| **Methods** | 78 | High method count |
| **Services** | 40+ | Comprehensive API |
| **Complexity** | High | Many responsibilities |
| **Coupling** | Medium | Depends on ROS 2, yaml, tf2 |
| **Cohesion** | Medium | Mixed responsibilities |

### Code Quality Issues

#### 1. Single Responsibility Principle Violations

**Issue**: Class handles multiple concerns
- Map loading/saving (I/O)
- Map validation
- ROS 2 service handling
- TF broadcasting
- Node/edge CRUD operations
- Tag management
- Configuration management

**Impact**: Hard to test, maintain, and extend

#### 2. Large Method Count (78 methods)

**Issue**: Class has too many public methods

**Breakdown**:
- Service callbacks: ~40
- Core operations: ~20
- Utility methods: ~18

#### 3. Service Callback Pattern Duplication

**Issue**: Every service callback follows the same pattern:
```python
def operation_cb(self, req):
    return self.operation(req.param1, req.param2, ...)
```

**Impact**: 40+ nearly identical wrapper methods

#### 4. Inconsistent Error Handling

**Issue**: Mix of:
- Return values (bool)
- Logging errors
- No exceptions raised
- Some methods have no error handling

**Example**:
```python
# Method 1: Returns bool
def add_edge(...):
    if num_available == 1:
        # ... do work
        return True
    else:
        # ... log error
        return False

# Method 2: No return value
def add_node(...):
    # ... do work
    # No return statement
```

#### 5. Parameter Inconsistency

**Issue**: Similar operations have different parameter signatures

**Example**:
```python
def add_node(name, pose, localise_by_topic="", verts="default", ...)
def update_node_waypoint(name, pose_msg, update=True, write_map=True)
```

#### 6. Magic Strings and Values

**Locations**:
- `"default"` as a sentinel value for verts and properties
- Hardcoded radius: `0.75`
- Hardcoded distance threshold: `8.0`
- Service topic strings scattered throughout

#### 7. Tight Coupling to YAML Format

**Issue**: Direct dict access throughout:
```python
node["node"]["name"]
node["meta"]["map"]
edge["goal"]["target_pose"]["pose"]
```

**Impact**: Hard to change data structure, no type safety

#### 8. Multiprocessing for Simple YAML Load

**Issue**: Uses multiprocessing.Process for YAML loading

**Rationale**: Unclear why multiprocessing is needed for file I/O

**Code**:
```python
def load_map(self, filename):
    def loader(filename, transporter):
        with open(filename, "r") as f:
            transporter["tmap2"] = yaml.load(f, Loader=CustomSafeLoader)
    
    transporter = multiprocessing.Manager().dict()
    p = multiprocessing.Process(target=loader, args=(filename, transporter))
    p.start()
    p.join()
    self.tmap2 = transporter["tmap2"]
```

#### 9. No Type Hints

**Issue**: No type annotations for parameters or return values

**Impact**: Hard to understand expected types, no IDE support

#### 10. Limited Unit Test Coverage

**Issue**: Complex logic with minimal tests

**Evidence**: No test files found in project structure

### Cyclomatic Complexity Analysis

| Method | Estimated Complexity | Issues |
|--------|---------------------|---------|
| `add_edge` | 15+ | Nested conditionals, multiple returns |
| `map_check` | 12+ | 5 validation loops with conditionals |
| `add_topological_node` | 10+ | Distance calculations, conditionals |
| `update_node_name` | 10+ | Nested loops for edge updates |
| `add_content_cb` | 8+ | JSON parsing with error handling |

### Design Patterns

**Current Patterns**:
- **Facade Pattern**: Provides simple interface to complex map operations
- **Service Wrapper Pattern**: All `*_cb()` methods wrap core operations

**Missing Patterns**:
- **Strategy Pattern**: Could be used for different validation strategies
- **Repository Pattern**: Separate data access from business logic
- **Command Pattern**: Service requests as command objects
- **Factory Pattern**: Node/edge creation

---

## 8. Improvement Recommendations

### Priority 1: Critical Refactoring

#### 1.1 Separate Concerns (Single Responsibility Principle)

**Problem**: Class has 5+ distinct responsibilities

**Solution**: Split into multiple classes

```python
# Proposed architecture

class TopologicalMapData:
    """Pure data structure with validation"""
    def __init__(self, tmap2: dict):
        self.tmap2 = tmap2
        self.validate()
    
    def validate(self):
        """Run all validation checks"""
        pass

class MapRepository:
    """Handles loading/saving maps"""
    def load(self, filename: str) -> TopologicalMapData:
        pass
    
    def save(self, map_data: TopologicalMapData, filename: str):
        pass

class NodeManager:
    """Node CRUD operations"""
    def __init__(self, map_data: TopologicalMapData):
        self.map_data = map_data
    
    def add_node(self, name, pose, **kwargs) -> bool:
        pass
    
    def remove_node(self, name) -> bool:
        pass
    
    def update_node(self, name, **updates) -> bool:
        pass

class EdgeManager:
    """Edge CRUD operations"""
    def add_edge(self, origin, dest, action, **kwargs) -> bool:
        pass
    
    def remove_edge(self, edge_id) -> bool:
        pass

class TagManager:
    """Tag operations"""
    def add_tag(self, node_names, tag) -> bool:
        pass
    
    def remove_tag(self, node_names, tag) -> bool:
        pass

class MapValidator:
    """Map validation logic"""
    def validate_structure(self, map_data) -> list[str]:
        """Returns list of validation errors"""
        pass

class MapManagerNode(rclpy.node.Node):
    """ROS 2 node coordinating all services"""
    def __init__(self):
        super().__init__('topological_map_manager_2')
        
        # Composition over inheritance
        self.repository = MapRepository()
        self.node_mgr = NodeManager(self.map_data)
        self.edge_mgr = EdgeManager(self.map_data)
        self.tag_mgr = TagManager(self.map_data)
        self.validator = MapValidator()
        
        self.advertise_services()
    
    def add_node_cb(self, req):
        """Service callback delegates to NodeManager"""
        return self.node_mgr.add_node(
            req.name, req.pose, req.add_close_nodes
        )
```

**Benefits**:
- Each class has single responsibility
- Easier to test (mock dependencies)
- Clearer code organization
- Reusable components

#### 1.2 Introduce Type Hints

**Problem**: No type information, hard to understand API

**Solution**: Add comprehensive type hints

```python
from typing import Dict, List, Tuple, Optional, Any
from geometry_msgs.msg import Pose

class NodeManager:
    def __init__(self, map_data: TopologicalMapData) -> None:
        self.map_data = map_data
    
    def add_node(
        self,
        name: str,
        pose: Dict[str, Any],
        localise_by_topic: str = "",
        verts: Union[List[Dict[str, float]], str] = "default",
        properties: Union[Dict[str, Any], str] = "default",
        tags: List[str] = [],
        update: bool = True,
        write_map: bool = True
    ) -> bool:
        """
        Add a node to the topological map.
        
        Args:
            name: Unique node identifier
            pose: Node position and orientation
            localise_by_topic: Topic for localization override
            verts: Influence zone vertices or "default" for circle
            properties: Node properties or "default" for standard
            tags: Semantic tags for the node
            update: Whether to trigger map update
            write_map: Whether to auto-save map
            
        Returns:
            True if node was added successfully, False otherwise
            
        Raises:
            ValueError: If node name already exists
        """
        pass
    
    def get_instances_of_node(
        self, 
        node_name: str
    ) -> Tuple[int, Optional[int]]:
        """
        Find node in map.
        
        Args:
            node_name: Node name to search for
            
        Returns:
            Tuple of (count, index) where index is None if count != 1
        """
        pass
```

**Benefits**:
- IDE autocomplete and type checking
- Self-documenting code
- Catches type errors at development time
- Easier refactoring

#### 1.3 Use Typed Data Classes

**Problem**: Raw dict access throughout, no structure validation

**Solution**: Use dataclasses for map structures

```python
from dataclasses import dataclass, field
from typing import List, Dict, Any

@dataclass
class TopologicalPose:
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # x, y, z, w

@dataclass
class TopologicalEdge:
    edge_id: str
    node: str  # destination
    action: str
    action_type: str
    goal: Dict[str, Any]
    config: List[Dict[str, Any]] = field(default_factory=list)
    recovery_behaviours_config: str = ""
    fail_policy: str = "fail"
    restrictions_planning: str = "True"
    restrictions_runtime: str = "True"
    fluid_navigation: bool = True

@dataclass
class TopologicalNode:
    name: str
    pose: TopologicalPose
    parent_frame: str
    verts: List[Dict[str, float]]
    properties: Dict[str, Any]
    edges: List[TopologicalEdge] = field(default_factory=list)
    localise_by_topic: str = ""
    restrictions_planning: str = "True"
    restrictions_runtime: str = "True"

@dataclass
class TopologicalMap:
    name: str
    metric_map: str
    pointset: str
    transformation: Dict[str, Any]
    nodes: List[TopologicalNode] = field(default_factory=list)
    last_updated: str = ""
```

**Benefits**:
- Type safety
- Default values
- Validation at construction
- IDE support for fields
- Easy serialization/deserialization

#### 1.4 Eliminate Service Callback Duplication

**Problem**: 40+ nearly identical wrapper methods

**Solution**: Use decorator pattern or dynamic service creation

```python
from functools import wraps
from typing import Callable, Any

def ros_service(
    service_type: type,
    topic: str
) -> Callable:
    """
    Decorator to automatically create ROS 2 service from method.
    
    Usage:
        @ros_service(tn_srv.AddNode, '/topological_map_manager2/add_topological_node')
        def add_topological_node(self, req):
            return self.node_mgr.add_node(req.name, req.pose, req.add_close_nodes)
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(self, req):
            try:
                return func(self, req)
            except Exception as e:
                self.get_logger().error(f"Service {topic} failed: {e}")
                # Return appropriate failure response
                return service_type.Response(success=False, message=str(e))
        
        # Store metadata for service creation
        wrapper._ros_service_type = service_type
        wrapper._ros_service_topic = topic
        return wrapper
    return decorator

class MapManagerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('topological_map_manager_2')
        self.advertise_services()
    
    def advertise_services(self):
        """Automatically create services from decorated methods"""
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if hasattr(attr, '_ros_service_type'):
                self.create_service(
                    attr._ros_service_type,
                    attr._ros_service_topic,
                    attr
                )
    
    @ros_service(tn_srv.AddNode, '/topological_map_manager2/add_topological_node')
    def add_topological_node_cb(self, req):
        return self.node_mgr.add_node(req.name, req.pose, req.add_close_nodes)
    
    @ros_service(tn_srv.RmvNode, '/topological_map_manager2/remove_topological_node')
    def remove_node_cb(self, req):
        return self.node_mgr.remove_node(req.name)
```

**Benefits**:
- DRY principle
- Consistent error handling
- Easier to add new services
- Less boilerplate code

### Priority 2: Code Quality

#### 2.1 Replace Magic Strings with Constants

**Problem**: Hardcoded strings throughout

**Solution**: Define constants

```python
class MapConstants:
    """Constants for topological map management"""
    
    # Default values
    DEFAULT_VERTS = "default"
    DEFAULT_PROPERTIES = "default"
    DEFAULT_EDGE_ID = "default"
    DEFAULT_INFLUENCE_RADIUS = 0.75
    DEFAULT_INFLUENCE_VERTICES = 8
    DEFAULT_CLOSE_NODE_DISTANCE = 8.0
    
    # Frame IDs
    DEFAULT_PARENT_FRAME = "map"
    DEFAULT_CHILD_FRAME = "topo_map"
    
    # Topic names
    TOPIC_MAP = '/topological_map_2'
    TOPIC_LEGACY_MAP = '/topological_map'
    
    # Service namespaces
    SERVICE_NS = '/topological_map_manager2/'
    
    # Fail policies
    FAIL_POLICY_FAIL = "fail"
    FAIL_POLICY_RETRY = "retry"
    
    # Restrictions
    RESTRICTIONS_TRUE = "True"
    RESTRICTIONS_FALSE = "False"

# Usage
verts = MapConstants.DEFAULT_VERTS
radius = MapConstants.DEFAULT_INFLUENCE_RADIUS
```

#### 2.2 Improve Error Handling

**Problem**: Inconsistent error handling

**Solution**: Define exception hierarchy and use consistently

```python
class MapManagerError(Exception):
    """Base exception for map manager errors"""
    pass

class NodeNotFoundError(MapManagerError):
    """Raised when node doesn't exist"""
    pass

class DuplicateNodeError(MapManagerError):
    """Raised when node already exists"""
    pass

class EdgeNotFoundError(MapManagerError):
    """Raised when edge doesn't exist"""
    pass

class MapValidationError(MapManagerError):
    """Raised when map validation fails"""
    def __init__(self, errors: List[str]):
        self.errors = errors
        super().__init__(f"Map validation failed: {errors}")

# Usage
class NodeManager:
    def add_node(self, name: str, **kwargs) -> None:
        if name in self.map_data.get_node_names():
            raise DuplicateNodeError(f"Node '{name}' already exists")
        
        # Add node logic
        pass
    
    def remove_node(self, name: str) -> None:
        num_available, _ = self.map_data.get_instances_of_node(name)
        if num_available == 0:
            raise NodeNotFoundError(f"Node '{name}' not found")
        if num_available > 1:
            raise MapManagerError(f"Multiple instances of node '{name}' found")
        
        # Remove node logic
        pass
```

#### 2.3 Add Comprehensive Logging

**Problem**: Limited logging context

**Solution**: Structured logging with context

```python
class NodeManager:
    def __init__(self, map_data: TopologicalMapData, logger):
        self.map_data = map_data
        self.logger = logger
    
    def add_node(self, name: str, pose: Dict, **kwargs) -> bool:
        self.logger.info(
            "[NodeManager] Adding node",
            extra={
                "node_name": name,
                "position": f"({pose['position']['x']:.2f}, {pose['position']['y']:.2f})",
                "tags": kwargs.get("tags", [])
            }
        )
        
        try:
            # ... add node logic
            self.logger.info(f"[NodeManager] Successfully added node '{name}'")
            return True
        except Exception as e:
            self.logger.error(
                f"[NodeManager] Failed to add node '{name}'",
                extra={"error": str(e)},
                exc_info=True
            )
            return False
```

#### 2.4 Add Unit Tests

**Problem**: No test coverage

**Solution**: Create comprehensive test suite

```python
# test_node_manager.py
import unittest
from topological_navigation.map_manager import NodeManager, TopologicalMapData

class TestNodeManager(unittest.TestCase):
    def setUp(self):
        self.map_data = TopologicalMapData.create_empty("test_map")
        self.node_mgr = NodeManager(self.map_data)
    
    def test_add_node_success(self):
        """Test adding a valid node"""
        result = self.node_mgr.add_node(
            "NodeA",
            {"position": {"x": 1.0, "y": 2.0, "z": 0.0},
             "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
        )
        self.assertTrue(result)
        self.assertIn("NodeA", self.map_data.get_node_names())
    
    def test_add_duplicate_node_fails(self):
        """Test adding duplicate node raises error"""
        self.node_mgr.add_node("NodeA", self.get_default_pose())
        
        with self.assertRaises(DuplicateNodeError):
            self.node_mgr.add_node("NodeA", self.get_default_pose())
    
    def test_remove_nonexistent_node_fails(self):
        """Test removing non-existent node raises error"""
        with self.assertRaises(NodeNotFoundError):
            self.node_mgr.remove_node("NonExistentNode")
    
    def test_generate_circle_vertices(self):
        """Test circle vertex generation"""
        verts = self.node_mgr.generate_circle_vertices(radius=1.0, number=4)
        self.assertEqual(len(verts), 4)
        # Vertices should be on unit circle
        for v in verts:
            dist = (v["x"]**2 + v["y"]**2)**0.5
            self.assertAlmostEqual(dist, 1.0, places=5)
```

### Priority 3: Performance

#### 3.1 Remove Multiprocessing for YAML Load

**Problem**: Unnecessary complexity and overhead

**Solution**: Use synchronous loading

```python
def load_map(self, filename: str) -> None:
    """Load topological map from YAML file."""
    self.loaded = False
    self.get_logger().info(f"Loading Topological Map {filename}")
    
    try:
        with open(filename, "r") as f:
            self.tmap2 = yaml.load(f, Loader=CustomSafeLoader)
    except FileNotFoundError:
        self.get_logger().error(f"Map file not found: {filename}")
        raise
    except yaml.YAMLError as e:
        self.get_logger().error(f"YAML parsing error: {e}")
        raise
    
    # Validate map type
    if not isinstance(self.tmap2, dict):
        raise ValueError(f"Expected dict, got {type(self.tmap2)}")
    
    self.loaded = True
    self.name = self.tmap2["name"]
    self.metric_map = self.tmap2["metric_map"]
    self.pointset = self.tmap2["pointset"]
    self.transformation = self.tmap2["transformation"]
    self.names = self.create_list_of_nodes()
    self.map_check()
    
    if self.cache_maps:
        self.cache_map(filename)
```

#### 3.2 Cache Node Lookups

**Problem**: Repeated linear searches through node list

**Solution**: Maintain index dictionary

```python
class TopologicalMapData:
    def __init__(self, tmap2: dict):
        self.tmap2 = tmap2
        self._node_index: Dict[str, int] = {}
        self._rebuild_index()
    
    def _rebuild_index(self):
        """Build node name → index mapping"""
        self._node_index.clear()
        for i, node in enumerate(self.tmap2["nodes"]):
            name = node["node"]["name"]
            if name in self._node_index:
                # Duplicate detected
                pass
            self._node_index[name] = i
    
    def get_node(self, name: str) -> Optional[dict]:
        """O(1) node lookup"""
        idx = self._node_index.get(name)
        if idx is not None:
            return self.tmap2["nodes"][idx]
        return None
    
    def add_node(self, node: dict):
        """Add node and update index"""
        name = node["node"]["name"]
        self.tmap2["nodes"].append(node)
        self._node_index[name] = len(self.tmap2["nodes"]) - 1
    
    def remove_node(self, name: str):
        """Remove node and rebuild index"""
        idx = self._node_index.get(name)
        if idx is not None:
            del self.tmap2["nodes"][idx]
            self._rebuild_index()  # Indices shifted, rebuild
```

### Priority 4: API Improvements

#### 4.1 Consistent Return Values

**Problem**: Mix of bool returns, None returns, exceptions

**Solution**: Use Result pattern or consistent exceptions

```python
from dataclasses import dataclass
from typing import Generic, TypeVar, Union

T = TypeVar('T')

@dataclass
class Success(Generic[T]):
    value: T
    
@dataclass
class Failure:
    error: str

Result = Union[Success[T], Failure]

class NodeManager:
    def add_node(self, name: str, **kwargs) -> Result[str]:
        """
        Add node to map.
        
        Returns:
            Success with node name if successful
            Failure with error message if failed
        """
        try:
            if name in self.map_data.get_node_names():
                return Failure(f"Node '{name}' already exists")
            
            # Add node logic
            return Success(name)
        except Exception as e:
            return Failure(str(e))

# Usage
result = node_mgr.add_node("NodeA", pose=...)
if isinstance(result, Success):
    print(f"Added node: {result.value}")
else:
    print(f"Failed: {result.error}")
```

#### 4.2 Builder Pattern for Complex Objects

**Problem**: Methods with 10+ parameters

**Solution**: Use builder pattern

```python
class NodeBuilder:
    """Builder for constructing topological nodes"""
    
    def __init__(self, name: str):
        self.name = name
        self._pose = None
        self._verts = []
        self._properties = {}
        self._tags = []
        self._localise_by_topic = ""
        self._restrictions_planning = "True"
        self._restrictions_runtime = "True"
    
    def with_pose(self, x: float, y: float, theta: float = 0.0) -> 'NodeBuilder':
        """Set node pose"""
        self._pose = {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": self._quaternion_from_yaw(theta)
        }
        return self
    
    def with_circular_influence_zone(
        self, 
        radius: float = 0.75, 
        num_vertices: int = 8
    ) -> 'NodeBuilder':
        """Generate circular influence zone"""
        self._verts = self._generate_circle_vertices(radius, num_vertices)
        return self
    
    def with_property(self, key: str, value: Any) -> 'NodeBuilder':
        """Add a property"""
        self._properties[key] = value
        return self
    
    def with_tags(self, *tags: str) -> 'NodeBuilder':
        """Add semantic tags"""
        self._tags.extend(tags)
        return self
    
    def build(self) -> TopologicalNode:
        """Build the node"""
        if self._pose is None:
            raise ValueError("Node pose must be set")
        
        return TopologicalNode(
            name=self.name,
            pose=TopologicalPose(**self._pose),
            verts=self._verts,
            properties=self._properties,
            tags=self._tags,
            localise_by_topic=self._localise_by_topic,
            restrictions_planning=self._restrictions_planning,
            restrictions_runtime=self._restrictions_runtime
        )

# Usage
node = (NodeBuilder("NodeA")
        .with_pose(1.0, 2.0, 0.0)
        .with_circular_influence_zone(radius=1.0)
        .with_property("xy_goal_tolerance", 0.3)
        .with_property("yaw_goal_tolerance", 0.1)
        .with_tags("charging_station", "safe_zone")
        .build())
```

---

## 9. Usage Examples

### Basic Usage

```python
import rclpy
from topological_navigation.manager2 import map_manager_2

def main():
    rclpy.init()
    
    # Create manager
    manager = map_manager_2(advertise_srvs=True)
    
    # Load existing map
    manager.init_map(
        name="warehouse_map",
        metric_map="warehouse_floor1",
        pointset="warehouse_map",
        filename="/path/to/warehouse.tmap2.yaml",
        load=True
    )
    
    # Spin node
    rclpy.spin(manager)
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a New Map

```python
# Create empty map
manager.init_map(
    name="new_map",
    metric_map="map_2d",
    pointset="new_map",
    filename="/path/to/new_map.tmap2.yaml",
    load=False  # Don't load, create new
)

# Add nodes
from geometry_msgs.msg import Pose

pose1 = Pose()
pose1.position.x = 1.0
pose1.position.y = 2.0
pose1.orientation.w = 1.0

manager.add_topological_node("NodeA", pose1, add_close_nodes=False)

pose2 = Pose()
pose2.position.x = 5.0
pose2.position.y = 2.0
pose2.orientation.w = 1.0

manager.add_topological_node("NodeB", pose2, add_close_nodes=True)  # Auto-creates edge to NodeA

# Save map
manager.write_topological_map("/path/to/new_map.tmap2.yaml")
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import topological_navigation_msgs.srv as tn_srv

class MapClient(Node):
    def __init__(self):
        super().__init__('map_client')
        
        # Create service client
        self.get_map_client = self.create_client(
            Trigger,
            '/topological_map_manager2/get_topological_map'
        )
        
        # Wait for service
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for map service...')
    
    def get_map(self):
        """Retrieve current topological map"""
        req = Trigger.Request()
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            import json
            map_data = json.loads(future.result().message)
            return map_data
        else:
            self.get_logger().error("Failed to get map")
            return None
```

### Programmatic Node Addition

```python
# Add node with custom properties
manager.add_node(
    name="ChargingStation",
    pose={
        "position": {"x": 10.0, "y": 5.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    verts=[
        {"x": 1.0, "y": 1.0},
        {"x": -1.0, "y": 1.0},
        {"x": -1.0, "y": -1.0},
        {"x": 1.0, "y": -1.0}
    ],
    properties={
        "xy_goal_tolerance": 0.5,
        "yaw_goal_tolerance": 0.2,
        "charging_power": 100,
        "max_vehicles": 2
    },
    tags=["charging", "priority"],
    update=True,
    write_map=True
)
```

### Batch Operations

```python
# Add multiple nodes efficiently
nodes_data = [
    {
        "name": "WP1",
        "pose": {"position": {"x": 1.0, "y": 1.0, "z": 0.0}, ...},
        "add_close_nodes": False
    },
    {
        "name": "WP2",
        "pose": {"position": {"x": 2.0, "y": 1.0, "z": 0.0}, ...},
        "add_close_nodes": False
    },
    # ... more nodes
]

manager.add_topological_nodes(nodes_data, update=True, write_map=True)

# Add multiple edges
edges_data = [
    {
        "origin": "WP1",
        "destination": "WP2",
        "action": "NavigateToPose",
        "action_type": "nav2_msgs/action/NavigateToPose"
    },
    # ... more edges
]

manager.add_edges(edges_data, update=True, write_map=True)
```

---

## Summary

The **map_manager_2** class is a comprehensive topological map management system with a rich ROS 2 service API. While functional, it suffers from:

1. **Monolithic design** - Too many responsibilities in one class
2. **Code duplication** - 40+ similar service wrappers
3. **Lack of type safety** - No type hints, raw dict access
4. **Inconsistent error handling** - Mix of returns, logs, exceptions
5. **Performance issues** - Linear searches, unnecessary multiprocessing

**Recommended refactoring path**:

1. **Phase 1**: Add type hints and dataclasses (low risk, high value)
2. **Phase 2**: Extract managers (NodeManager, EdgeManager, etc.)
3. **Phase 3**: Eliminate service callback duplication
4. **Phase 4**: Add comprehensive tests
5. **Phase 5**: Performance optimizations (caching, indexing)

With these improvements, the codebase would be:
- **More maintainable**: Clear separation of concerns
- **More testable**: Isolated components with clear interfaces
- **More robust**: Comprehensive error handling and validation
- **More performant**: Optimized data access patterns
- **More developer-friendly**: Type hints, documentation, consistent API

---

**End of Documentation**
