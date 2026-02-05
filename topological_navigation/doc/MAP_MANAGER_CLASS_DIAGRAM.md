# Map Manager 2 - Class Interaction Diagram & Method Documentation

## System Architecture Overview

The `map_manager_2` class is a ROS 2 node that manages topological maps for robot navigation. It acts as the main interface between ROS services and the underlying data model, providing comprehensive map manipulation and query capabilities.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         ROS 2 Ecosystem                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │
│  │ ROS Services │  │  Publishers  │  │ TF Broadcast │                  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                  │
└─────────┼──────────────────┼──────────────────┼───────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        map_manager_2                                     │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                    Core Components                              │    │
│  │  • model: TopologicalMapModel (data storage & validation)      │    │
│  │  • goal_mappings: Action goal configurations                   │    │
│  │  • broadcaster: TF2 transform broadcaster                      │    │
│  │  • cache_dir: Map caching directory                            │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                   Service Interface (35+ services)             │    │
│  │  • Map Retrieval & Switching                                   │    │
│  │  • Node CRUD Operations                                        │    │
│  │  • Edge CRUD Operations                                        │    │
│  │  • Tag Management                                              │    │
│  │  • Configuration Updates                                       │    │
│  │  • Batch Operations                                            │    │
│  └────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    TopologicalMapModel                                   │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │  • tmap: dict (YAML/JSON structure)                            │    │
│  │  • schema: JSON Schema for validation                          │    │
│  │  • filename: Current map file path                             │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                                                                          │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                Core Model Operations                            │    │
│  │  • load() / save()                                             │    │
│  │  • validate()                                                  │    │
│  │  • add_node() / remove_node()                                  │    │
│  │  • add_edge() / remove_edge()                                  │    │
│  │  • get_node() / get_node_index()                               │    │
│  └────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Class Hierarchy

```
rclpy.node.Node
    ↑
    │ (inherits)
    │
map_manager_2
    │
    │ (composition)
    │
    ├─→ TopologicalMapModel (data model)
    ├─→ tf2_ros.TransformBroadcaster (TF publishing)
    └─→ goal_mappings: dict (action configurations)
```

---

## Data Flow Diagram

```
Service Request → Service Callback → Core Method → Model Update → Validation
                                          │              │
                                          ├─→ update() ──┤
                                          │              ▼
                                          └─→ write_topological_map()
                                                         │
                                                         ▼
                                                   YAML File (persistent)
```

---

## Complete Method Reference

### 1. INITIALIZATION & SETUP

#### `__init__(self, advertise_srvs=True)`
**Purpose**: Initializes the map manager node with all necessary components.

**Parameters**:
- `advertise_srvs` (bool): Whether to advertise ROS services immediately

**Operations**:
1. Loads ROS parameters:
   - `cache_topological_maps`: Enable/disable map caching
   - `auto_write_topological_maps`: Auto-save on modifications
   - `nav_config`: Navigation goal configuration file path
2. Locates schema file for validation (`tmap-schema.yaml`)
3. Creates cache directory (`~/.ros/topological_maps`)
4. Loads navigation goal configurations from YAML
5. Initializes empty `TopologicalMapModel` with schema validation
6. Optionally advertises all ROS services

**Use Case**: Called once when node starts up

---

#### `advertise(self)`
**Purpose**: Creates and advertises all ROS 2 service endpoints (35 services total).

**Service Categories**:

**A. Map Retrieval Services** (Read-only):
- `/topological_map_manager2/get_topological_map` - Returns complete map as JSON
- `/topological_map_manager2/get_tagged_nodes` - Query nodes by tag
- `/topological_map_manager2/get_tags` - List all available tags
- `/topological_map_manager2/get_node_tags` - Get tags for specific node
- `/topological_map_manager2/get_edges_between_nodes` - Find edges between two nodes

**B. Map File Operations**:
- `/topological_map_manager2/write_topological_map` - Save map to disk
- `/topological_map_manager2/switch_topological_map` - Load different map

**C. Node Operations**:
- `/topological_map_manager2/add_topological_node` - Add single node
- `/topological_map_manager2/add_topological_node_multi` - Batch add nodes
- `/topological_map_manager2/remove_topological_node` - Delete node
- `/topological_map_manager2/update_node_name` - Rename node
- `/topological_map_manager2/update_node_pose` - Change node position
- `/topological_map_manager2/update_node_tolerance` - Update goal tolerances
- `/topological_map_manager2/clear_topological_nodes` - Remove all nodes

**D. Edge Operations**:
- `/topological_map_manager2/add_edges_between_nodes` - Add single edge
- `/topological_map_manager2/add_edges_between_nodes_multi` - Batch add edges
- `/topological_map_manager2/remove_edge` - Delete edge
- `/topological_map_manager2/update_edge` - Modify edge properties
- `/topological_map_manager2/update_action` - Update action for all matching edges

**E. Tag Management**:
- `/topological_map_manager2/add_tag_to_node` - Add tag to nodes
- `/topological_map_manager2/rm_tag_from_node` - Remove tag from nodes
- `/topological_map_manager2/modify_node_tags` - Change existing tag

**F. Configuration Management**:
- `/topological_map_manager2/add_param_to_edge_config` - Add parameter to edge
- `/topological_map_manager2/add_param_to_edge_config_multi` - Batch add parameters
- `/topological_map_manager2/rm_param_from_edge_config` - Remove parameter from edge
- `/topological_map_manager2/rm_param_from_topological_map` - Remove parameter globally

**G. Advanced Features**:
- `/topological_map_manager2/update_node_restrictions` - Set planning/runtime restrictions
- `/topological_map_manager2/update_edge_restrictions` - Set edge restrictions
- `/topological_map_manager2/update_fail_policy` - Set global failure policy
- `/topological_map_manager2/set_node_influence_zone` - Define node spatial zone
- `/topological_map_manager2/set_node_influence_zone_multi` - Batch set zones
- `/topological_map_manager2/add_datum` - Set GPS datum
- `/topological_map_manager2/add_content_to_node` - Add semantic content

---

### 2. MAP INITIALIZATION & LOADING

#### `init_map(self, name, metric_map, pointset, transformation, filename, load)`
**Purpose**: Initializes or loads a topological map with specified parameters.

**Parameters**:
- `name` (str): Map identifier
- `metric_map` (str): Reference metric map frame (e.g., "map_2d")
- `pointset` (str): Point cloud set identifier
- `transformation` (dict): TF transform between metric and topological frames
- `filename` (str): Path to map file
- `load` (bool): Whether to load from file or create new

**Operations**:
1. Sets map metadata (name, metric_map, pointset, transformation)
2. Determines filename (uses cache if not specified)
3. If `load=True`: calls `load_map()`
4. If `load=False`: initializes empty map structure
5. Declares ROS parameters for map identification
6. Creates publishers:
   - `/topological_map_2` (String): JSON representation of map
7. Initializes TF broadcaster
8. Broadcasts transformation
9. Creates node name list for quick lookup

**Default Transformation**:
```python
{
    "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
    "child": "topo_map",
    "parent": "map"
}
```

**Use Case**: Called after `__init__()` to prepare map for operations

---

#### `load_map(self, filename)`
**Purpose**: Loads a topological map from a YAML file with validation.

**Parameters**:
- `filename` (str): Absolute path to `.tmap2` file

**Operations**:
1. Delegates to `model.load(filename)` which:
   - Reads YAML file using `CustomSafeLoader`
   - Validates against JSON schema
   - Checks logical consistency (duplicate names, invalid edges)
2. Syncs local properties from loaded map:
   - `self.name`, `self.metric_map`, `self.pointset`, `self.transformation`
3. Declares ROS parameters for loaded map
4. Creates node name index
5. Optionally caches map to `~/.ros/topological_maps/`

**Error Handling**:
- Logs error and sets `self.loaded = False` on failure
- Raises `MapValidationError` for invalid maps

**Use Case**: Loading existing maps at startup or switching maps

---

#### `write_topological_map(self, filename, no_alias=False)`
**Purpose**: Persists the current map to a YAML file.

**Parameters**:
- `filename` (str): Target file path
- `no_alias` (bool): Disable YAML anchors/aliases for readability

**Operations**:
1. Sorts nodes alphabetically for deterministic output
2. Serializes map to YAML using `NoAliasDumper` or `safe_dump`
3. Writes to file

**Use Case**: Manual saves or auto-save after modifications

---

### 3. MAP PUBLICATION & UPDATES

#### `update(self, update_time=True)`
**Purpose**: Publishes map changes to ROS topics and updates internal state.

**Parameters**:
- `update_time` (bool): Whether to update `last_updated` timestamp

**Operations**:
1. Updates `meta.last_updated` timestamp
2. Publishes JSON map to `/topological_map_2` topic
3. Regenerates node name list
4. (Legacy) Could publish to old format topic

**Frequency**: Called after every map modification

---

#### `broadcast_transform(self)`
**Purpose**: Publishes the TF transform between metric and topological frames.

**Operations**:
1. Extracts translation and rotation from `self.transformation`
2. Creates `TransformStamped` message
3. Broadcasts transform via `tf2_ros.TransformBroadcaster`

**Transform Purpose**: Allows conversion between map coordinates and topological node coordinates

**Frequency**: Called once at initialization

---

#### `create_list_of_nodes(self)`
**Purpose**: Generates sorted list of all node names for quick lookup.

**Returns**: `list[str]` - Alphabetically sorted node names

**Use Case**: Used by update() and for name validation

---

### 4. SERVICE CALLBACKS - MAP RETRIEVAL

#### `get_topological_map_cb(self, req)`
**Purpose**: Returns entire map as JSON string.

**Service**: `std_srvs.srv.Trigger`

**Returns**: 
```python
response.success = True
response.message = json.dumps(self.model.tmap)
```

**Use Case**: Map visualization, external analysis tools

---

#### `switch_topological_map_cb(self, req)`
**Purpose**: Switches to a different topological map file.

**Service**: `topological_navigation_msgs.srv.WriteTopologicalMap`

**Parameters**:
- `req.filename` (str): New map filename

**Operations**:
1. Updates ROS parameter `topological_map2_filename`
2. Constructs full path from parameter `topological_map2_path`
3. Calls `load_map()` to load new map
4. Calls `update(False)` to publish without timestamp change
5. Re-broadcasts transform (may have changed)

**Returns**: `(success: bool, message: str)` - JSON map if successful

**Use Case**: Multi-map environments, dynamic map switching

---

#### `get_tagged_cb(self, req, res)`
**Purpose**: Finds all nodes with a specific tag.

**Service**: `topological_navigation_msgs.srv.GetTaggedNodes`

**Parameters**:
- `req.tag` (str): Tag to search for

**Returns**: 
- `res.nodes` (list[str]): Node names containing the tag

**Algorithm**:
```python
for node in map["nodes"]:
    if tag in node["meta"]["tag"]:
        result.append(node["node"]["name"])
```

**Use Case**: Semantic queries like "find all charging stations" (tag: "charging")

---

#### `get_tags_cb(self, req)`
**Purpose**: Returns set of all unique tags in the map.

**Returns**: `list[str]` - Unique tag list

**Use Case**: Tag discovery, UI generation

---

#### `get_node_tags_cb(self, req)`
**Purpose**: Retrieves all tags for a specific node.

**Parameters**:
- `req.node_name` (str): Target node

**Returns**: `(success: bool, tags: list[str])`

**Use Case**: Node introspection, behavior planning

---

#### `get_edges_between_cb(self, req)`
**Purpose**: Finds all edges between two nodes (bidirectional).

**Parameters**:
- `req.nodea` (str): First node
- `req.nodeb` (str): Second node

**Returns**: `(ab: list[str], ba: list[str])` - Edge IDs in each direction

**Algorithm**:
```python
ab = [edge["edge_id"] for edge in nodeA["edges"] if edge["node"] == nodeB]
ba = [edge["edge_id"] for edge in nodeB["edges"] if edge["node"] == nodeA]
```

**Use Case**: Path planning, connectivity analysis

---

### 5. SERVICE CALLBACKS - NODE OPERATIONS

#### `add_topological_node_cb(self, req)` → `add_topological_node(...)`
**Purpose**: Adds a new node to the map.

**Parameters**:
- `node_name` (str): Unique node identifier
- `node_pose` (Pose): Spatial position (x, y, z, orientation)
- `add_close_nodes` (bool): Auto-create edges to nearby nodes
- `dist` (float): Distance threshold for "close" nodes (default 8.0m)
- `update` (bool): Call update() after operation
- `write_map` (bool): Auto-save if enabled

**Operations**:
1. Generates unique name if not provided
2. Converts ROS Pose to dictionary
3. Calls `model.add_node()` with:
   - Default properties: `xy_goal_tolerance=0.3`, `yaw_goal_tolerance=0.1`
   - Default verts: Circular boundary (radius 0.75m, 8 vertices)
   - Restrictions: `"True"` (no restrictions)
4. If `add_close_nodes=True`:
   - Scans all existing nodes
   - Calculates Euclidean distance: `sqrt((x1-x2)² + (y1-y2)²)`
   - Creates bidirectional edges to nodes within `dist` (excluding "ChargingPoint")
   - Uses default action "move_base"
5. Calls `update()` and optionally `write_topological_map()`

**Error Handling**:
- Returns `False` if node name exists (`DuplicateError`)

**Use Case**: Manual map building, waypoint creation during robot operation

---

#### `remove_node_cb(self, req)` → `remove_node(...)`
**Purpose**: Deletes a node and all edges referencing it.

**Parameters**:
- `node_name` (str): Node to remove

**Operations**:
1. Calls `model.remove_node()` which:
   - Removes node from `tmap["nodes"]` list
   - Scans all other nodes and removes edges pointing to deleted node
2. Updates map and optionally saves

**Error Handling**:
- Returns `False` if node not found (`NodeNotFoundError`)

**Use Case**: Map cleanup, removing obsolete waypoints

---

#### `update_node_name_cb(self, req)` → `update_node_name(...)`
**Purpose**: Renames a node and updates all edge references.

**Parameters**:
- `node_name` (str): Current name
- `new_name` (str): New name

**Operations**:
1. Validates `new_name` doesn't already exist
2. Scans entire map to update:
   - Edge IDs containing old name: `oldname_dest` → `newname_dest`
   - Edge `node` field pointing to renamed node
   - Node's `meta.node` and `node.name` fields
3. Updates and saves

**Edge ID Update Logic**:
```python
if n["node"]["name"] == old_name:
    edge["edge_id"] = new_name + "_" + edge["node"]
if edge["node"] == old_name:
    edge["node"] = new_name
    edge["edge_id"] = n["node"]["name"] + "_" + new_name
```

**Use Case**: Map reorganization, semantic naming improvements

---

#### `update_node_waypoint_cb(self, req)` → `update_node_waypoint(...)`
**Purpose**: Changes a node's spatial position.

**Parameters**:
- `name` (str): Node to update
- `pose_msg` (Pose): New position

**Operations**:
1. Converts ROS Pose to dict
2. Calls `model.update_node_pose()`
3. Updates and saves

**Use Case**: Correcting GPS drift, refining map accuracy

---

#### `update_node_tolerance_cb(self, req)` → `update_node_tolerance(...)`
**Purpose**: Adjusts goal tolerance parameters for navigation.

**Parameters**:
- `node_name` (str): Target node
- `xy_tolerance` (float): Position tolerance in meters
- `yaw_tolerance` (float): Orientation tolerance in radians

**Operations**:
1. Finds node
2. Ensures `node["node"]["properties"]` dict exists
3. Sets `xy_goal_tolerance` and `yaw_goal_tolerance`

**Use Case**: Tuning navigation behavior (tight vs. loose positioning)

---

#### `clear_nodes_cb(self, req)` → `clear_nodes()`
**Purpose**: Removes ALL nodes from the map.

**Operations**:
1. Sets `model.tmap["nodes"] = []`
2. Updates and saves

**Warning**: Destructive operation - no undo!

**Use Case**: Starting fresh map from scratch

---

### 6. SERVICE CALLBACKS - EDGE OPERATIONS

#### `add_edge_cb(self, req)` → `add_edge(...)`
**Purpose**: Creates a directed edge between two nodes.

**Parameters**:
- `origin` (str): Source node name
- `destination` (str): Target node name
- `action` (str): Action name (e.g., "move_base", "undock")
- `action_type` (str): ROS action type (e.g., "move_base_msgs/MoveBaseGoal")
- `edge_id` (str): Optional custom edge ID
- `update` (bool): Update map after operation
- `write_map` (bool): Save map after operation

**Operations**:
1. Resolves action type and goal configuration via `set_goal()`:
   - Checks cached `goal_mappings`
   - Attempts to load from ROS parameter `~{action_type}`
   - Falls back to package config file `{package}/config/{goal_def}.yaml`
   - Uses default move_base goal if all fail
2. Calls `model.add_edge()` to create edge with:
   - Auto-generated or provided edge_id
   - Action configuration
   - Default fail_policy: "fail"
   - Default fluid_navigation: True
3. Post-processes to inject resolved `goal` into edge
4. Updates and saves

**Edge ID Generation**:
- Format: `origin_destination` or `origin_destination_NNN` if conflict

**Error Handling**:
- Returns `False` if origin or destination node not found

**Use Case**: Building navigation graph, defining traversable paths

---

#### `remove_edge_cb(self, req)` → `remove_edge(...)`
**Purpose**: Deletes an edge by its ID.

**Parameters**:
- `edge_id` (str): Unique edge identifier

**Operations**:
1. Calls `model.remove_edge()` which scans all nodes for matching edge_id
2. Updates and saves

**Error Handling**:
- Returns `False` if edge not found (`EdgeNotFoundError`)

**Use Case**: Removing invalid/blocked paths

---

#### `update_edge_cb(self, req)` → `update_edge(...)`
**Purpose**: Modifies properties of an existing edge.

**Parameters**:
- `edge_id` (str): Target edge
- `action_name` (str): New action name
- `action_type` (str): New action type
- `goal` (str): JSON goal configuration
- `fail_policy` (str): Failure handling policy
- `not_fluid` (bool): Disable fluid navigation

**Operations**:
1. Finds edge via `get_node_names_from_edge_id_2()`
2. Updates fields if provided:
   - `action`, `action_type`, `goal` (parsed from JSON)
   - If `action_type` provided without `goal`, resolves via `set_goal()`
   - `fail_policy`
   - `fluid_navigation` (inverted logic: `not_fluid=True` → `False`)

**Goal Resolution**: Supports both explicit JSON goal or auto-resolution

**Use Case**: Tuning navigation behavior, changing actions

---

#### `update_action_cb(self, req)` → `update_action(...)`
**Purpose**: Batch updates all edges using a specific action.

**Parameters**:
- `action_name` (str): Action to search for
- `action_type` (str): New action type for all matches
- `goal` (str): New goal configuration

**Operations**:
1. Scans all nodes and all edges
2. Updates edges where `edge["action"] == action_name`
3. Updates action_type and/or goal

**Use Case**: Global behavior changes (e.g., upgrading all move_base actions)

---

### 7. SERVICE CALLBACKS - TAG MANAGEMENT

#### `modify_tag_cb(self, msg)` → Changes existing tag
**Parameters**:
- `node` (list[str]): Nodes to modify
- `tag` (str): Tag to replace
- `new_tag` (str): Replacement tag

**Operations**:
1. For each specified node:
   - Finds `tag` in `node["meta"]["tag"]` list
   - Replaces with `new_tag`

**Returns**: `(success: bool, meta: str)` - Updated metadata

---

#### `add_tag_cb(self, msg)` → Adds tag to nodes
**Parameters**:
- `node` (list[str]): Target nodes
- `tag` (str): Tag to add

**Operations**:
1. For each node:
   - Ensures `node["meta"]["tag"]` list exists
   - Appends tag if not already present

**Use Case**: Semantic labeling (e.g., "loading_zone", "safety_critical")

---

#### `rm_tag_cb(self, msg)` → Removes tag from nodes
**Parameters**:
- `node` (list[str]): Target nodes
- `tag` (str): Tag to remove

**Operations**:
1. Removes `tag` from each node's tag list

---

### 8. SERVICE CALLBACKS - CONFIGURATION MANAGEMENT

#### `add_param_to_edge_config_cb(self, req)` → `add_param_to_edge_config(...)`
**Purpose**: Adds/updates a ROS parameter override for a specific edge.

**Parameters**:
- `edge_id` (str): Target edge
- `namespace` (str): Parameter namespace (e.g., "/move_base")
- `name` (str): Parameter name (e.g., "max_vel_x")
- `value` (str): Parameter value
- `value_is_string` (bool): Whether to treat value as string
- `not_reset` (bool): Whether to persist after traversal (reset=False)

**Operations**:
1. Parses `value`:
   - If `value_is_string=False`, attempts `eval()` to convert to native type
2. Constructs parameter dict:
   ```python
   {"namespace": ns, "name": n, "value": v, "reset": not not_reset}
   ```
3. Finds edge and removes any existing param with same namespace+name
4. Appends new parameter to edge's `config` list

**Use Case**: Per-edge navigation tuning (slower speed in tight areas)

---

#### `rm_param_from_edge_config_cb(self, req)` → `rm_param_from_edge_config(...)`
**Purpose**: Removes a parameter from an edge's configuration.

**Parameters**:
- `edge_id`, `namespace`, `name`

**Operations**:
1. Filters edge's `config` list to remove matching parameter

---

#### `rm_param_from_topological_map_cb(self, req)` → `rm_param_from_topological_map(...)`
**Purpose**: Globally removes a parameter from ALL edges.

**Parameters**:
- `namespace`, `name`

**Operations**:
1. Scans all nodes, all edges
2. Removes matching parameter from each edge's config

**Use Case**: Cleaning up deprecated parameters

---

### 9. SERVICE CALLBACKS - ADVANCED FEATURES

#### `update_node_restrictions_cb(self, req)` → `update_node_restrictions(...)`
**Purpose**: Sets access restrictions for a node (planning/runtime).

**Parameters**:
- `node_name` (str): Target node
- `restrictions_planning` (str): Planning-time constraint expression
- `restrictions_runtime` (str): Runtime constraint expression
- `update_edges` (bool): Also update connected edges

**Operations**:
1. Sets `node["node"]["restrictions_planning"]` and `restrictions_runtime`
2. If `update_edges=True`:
   - Finds all edges from/to this node
   - Applies `restrictions_planning` to each edge

**Restriction Format**: Boolean expressions (e.g., `"time >= 8 AND time <= 17"`)

**Use Case**: Time-based access (night restrictions), robot-type restrictions

---

#### `update_edge_restrictions_cb(self, req)` → `update_edge_restrictions(...)`
**Purpose**: Sets restrictions on a specific edge.

**Parameters**:
- `edge_id`, `restrictions_planning`, `restrictions_runtime`

**Operations**:
1. Finds edge and sets restriction fields

---

#### `update_fail_policy_cb(self, req)` → `update_fail_policy(...)`
**Purpose**: Sets failure policy for ALL edges globally.

**Parameters**:
- `fail_policy` (str): Policy name (e.g., "fail", "retry", "continue")

**Operations**:
1. Scans all edges
2. Sets `edge["fail_policy"] = fail_policy`

**Use Case**: Global behavior changes for robust navigation

---

#### `set_influence_zone_cb(self, req)` → `set_influence_zone(...)`
**Purpose**: Defines a polygonal influence zone for a node.

**Parameters**:
- `node_name` (str): Target node
- `vertices_x` (list[float]): X coordinates
- `vertices_y` (list[float]): Y coordinates

**Operations**:
1. Validates vertices (minimum 3 points, equal array lengths)
2. Constructs vertex list: `[{"x": x, "y": y}, ...]`
3. Sets `node["node"]["verts"] = verts`

**Use Case**: Defining restricted areas, collision zones, docking zones

---

#### `add_datum_cb(self, req)` → `add_datum(...)`
**Purpose**: Sets GPS datum (reference point) for geo-referenced maps.

**Parameters**:
- `latitude` (float): Reference latitude
- `longitude` (float): Reference longitude

**Operations**:
1. Sets `model.tmap["meta"]["datum_latitude"]` and `datum_longitude`

**Use Case**: Outdoor navigation, GPS integration

---

#### `add_content_cb(self, req)` → Adds semantic content to node
**Purpose**: Attaches semantic metadata (what the node contains).

**Parameters**:
- `node` (str): Target node
- `content` (str): JSON content (category + name)

**Operations**:
1. Parses JSON content
2. Validates format: `{"category": "...", "name": "..."}`
3. Appends to `node["meta"]["contains"]` list

**Example Content**:
```json
{
  "category": "object",
  "name": "charging_station_A"
}
```

**Use Case**: Semantic mapping, task planning

---

### 10. BATCH OPERATIONS

#### `add_topological_nodes_cb(self, req)` → `add_topological_nodes(...)`
**Purpose**: Adds multiple nodes in a single transaction.

**Parameters**:
- `data` (list[AddNodeRequest]): Array of node specifications

**Operations**:
1. Iterates through data, calling `add_topological_node()` with:
   - `add_close_nodes=False` (manual edges)
   - `update=False` (defer update)
   - `write_map=False` (defer save)
2. After all nodes added, calls `update()` and `write_topological_map()` once

**Performance**: ~10-100x faster than individual service calls

**Use Case**: Importing large maps, programmatic map generation

---

#### `add_edges_cb(self, req)` → `add_edges(...)`
**Purpose**: Batch adds edges.

**Parameters**:
- `data` (list[AddEdgeRequest])

**Operations**: Similar batch pattern with deferred update/save

---

#### `add_params_to_edges_cb(self, req)` → `add_params_to_edges(...)`
**Purpose**: Batch adds parameters to multiple edges.

---

#### `set_influence_zones_cb(self, req)` → `set_influence_zones(...)`
**Purpose**: Batch sets influence zones.

---

### 11. UTILITY METHODS

#### `set_goal(self, action, action_type, _goal=None)`
**Purpose**: Resolves action goal configuration from various sources.

**Resolution Priority**:
1. Cached in `self.goal_mappings`
2. Provided `_goal` parameter
3. ROS parameter `~{action_type}`
4. Package config file: `{package}/config/{goal_def}.yaml`
5. Default move_base goal from `navigation_goal.yaml`

**Returns**: `(action_type: str, goal: dict)`

**Caching**: Stores resolved goals in `self.goal_mappings` for performance

---

#### `get_new_edge_id(self, origin, destination)`
**Purpose**: Generates edge ID (legacy/placeholder).

**Note**: In refactored version, Model handles auto-generation

---

#### Helper Function: `pose_dist(pose1, pose2)`
**Purpose**: Calculates 2D Euclidean distance between poses.

**Formula**: `sqrt((x1-x2)² + (y1-y2)²)`

**Use Case**: Used by `add_topological_node()` for close node detection

---

## TopologicalMapModel Class Reference

### Core Data Structure

```python
tmap = {
    "meta": {
        "last_updated": "2026-02-05_10-30-45",
        "datum_latitude": 52.12345,  # Optional
        "datum_longitude": -0.98765   # Optional
    },
    "name": "warehouse_map",
    "metric_map": "map_2d",
    "pointset": "warehouse_map",
    "transformation": {
        "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "child": "topo_map",
        "parent": "map"
    },
    "nodes": [
        {
            "meta": {
                "map": "map_2d",
                "node": "WayPoint1",
                "pointset": "warehouse_map",
                "tag": ["loading_zone", "priority_high"],  # Optional
                "contains": [  # Optional semantic content
                    {"category": "object", "name": "pallet_rack"}
                ]
            },
            "node": {
                "name": "WayPoint1",
                "pose": {
                    "position": {"x": 10.5, "y": 5.2, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                },
                "edges": [
                    {
                        "edge_id": "WayPoint1_WayPoint2",
                        "node": "WayPoint2",
                        "action": "move_base",
                        "action_type": "move_base_msgs/MoveBaseGoal",
                        "goal": {...},
                        "config": [
                            {
                                "namespace": "/move_base",
                                "name": "max_vel_x",
                                "value": 0.5,
                                "reset": true
                            }
                        ],
                        "fail_policy": "fail",
                        "fluid_navigation": true,
                        "restrictions_planning": "True",
                        "restrictions_runtime": "True"
                    }
                ],
                "localise_by_topic": "",
                "parent_frame": "map",
                "properties": {
                    "xy_goal_tolerance": 0.3,
                    "yaw_goal_tolerance": 0.1
                },
                "verts": [
                    {"x": 0.53, "y": 0.53},
                    {"x": -0.53, "y": 0.53},
                    {"x": -0.53, "y": -0.53},
                    {"x": 0.53, "y": -0.53}
                ],
                "restrictions_planning": "True",
                "restrictions_runtime": "True"
            }
        }
    ]
}
```

### Model Methods

#### `load(self, filename)` / `save(self, filename, no_alias=False)`
**Purpose**: Persistent storage with validation

---

#### `validate(self)`
**Purpose**: Two-phase validation:
1. **Schema Validation**: JSON Schema compliance via `jsonschema.validate()`
2. **Logical Validation**: 
   - Single pointset consistency
   - Unique node names
   - Valid edge destinations
   - No dangling references

**Raises**: `MapValidationError` with detailed message

---

#### `add_node(self, name, pose, properties, verts, ...)`
**Purpose**: Creates node with validation.

**Raises**: `DuplicateError` if name exists

**Defaults**:
- `properties`: xy_tolerance=0.3, yaw_tolerance=0.1
- `verts`: Circle with radius 0.75m, 8 vertices
- `restrictions`: "True" (no restrictions)

---

#### `remove_node(self, node_name)`
**Purpose**: Deletes node and cascades to edges.

**Cascade Logic**: Removes all edges from other nodes pointing to deleted node

---

#### `add_edge(self, origin, destination, action_type, ...)`
**Purpose**: Creates directed edge with auto-ID generation.

**ID Generation**: `origin_destination` or `origin_destination_NNN` if conflict

**Returns**: `final_edge_id` (str)

---

#### `remove_edge(self, edge_id)`
**Purpose**: Deletes edge by ID (scans all nodes).

---

#### `get_node(self, node_name)` / `get_node_index(self, node_name)`
**Purpose**: Node lookup by name.

**Returns**: Node dict or None / index or -1

---

## Common Workflows

### Workflow 1: Building a New Map

```
1. manager = map_manager_2()
2. manager.init_map(name="factory_floor", load=False)
3. manager.add_topological_node("Entrance", pose1, add_close_nodes=False)
4. manager.add_topological_node("WorkStation_A", pose2, add_close_nodes=False)
5. manager.add_topological_node("WorkStation_B", pose3, add_close_nodes=False)
6. manager.add_edge("Entrance", "WorkStation_A", "move_base", ...)
7. manager.add_edge("Entrance", "WorkStation_B", "move_base", ...)
8. manager.write_topological_map("/path/to/factory_floor.tmap2")
```

### Workflow 2: Loading and Modifying Existing Map

```
1. manager.init_map(filename="/maps/warehouse.tmap2", load=True)
2. manager.update_node_waypoint("Dock_1", new_pose)  # Correct GPS drift
3. manager.add_tag_cb(nodes=["Dock_1", "Dock_2"], tag="maintenance_required")
4. manager.update_fail_policy("retry")  # Global change
5. manager.write_topological_map()  # Save changes
```

### Workflow 3: Batch Import from External Source

```
1. nodes = [create_node_msg(...) for waypoint in gps_data]
2. manager.add_topological_nodes(nodes)
3. edges = generate_connectivity_graph(nodes)
4. manager.add_edges(edges)
5. manager.write_topological_map()
```

---

## Error Handling Strategy

### Exception Types
- `MapValidationError`: Schema or logical validation failure
- `NodeNotFoundError`: Node name doesn't exist
- `EdgeNotFoundError`: Edge ID doesn't exist
- `DuplicateError`: Attempting to create node with existing name

### Service Response Pattern
```python
try:
    result = operation()
    return True, "Success message"
except CustomError as e:
    logger.error(str(e))
    return False, str(e)
```

---

## Configuration Parameters

### ROS Parameters
- `cache_topological_maps` (bool): Cache loaded maps to `~/.ros/topological_maps/`
- `auto_write_topological_maps` (bool): Automatically save after modifications
- `nav_config` (str): Path to `navigation_goal.yaml`
- `topological_map2_name` (str): Current map name
- `topological_map2_filename` (str): Current map filename
- `topological_map2_path` (str): Current map directory

### File Locations
- **Schema**: `{package}/config/tmap-schema.yaml`
- **Nav Config**: `{package}/config/navigation_goal.yaml`
- **Cache Dir**: `~/.ros/topological_maps/`
- **Map Files**: `.tmap2` extension (YAML format)

---

## Performance Considerations

### Batch Operations
- **Single adds**: ~10-50ms per operation (I/O overhead)
- **Batch adds**: ~1-5ms per item (single I/O operation)
- **Recommendation**: Use `_multi` services for >10 items

### Caching
- `goal_mappings` caches action goal configurations
- `self.names` provides O(1) node name lookup
- Model validation runs only on load, not every update

### Auto-write Tradeoff
- **Enabled**: Prevents data loss but adds I/O overhead
- **Disabled**: Faster operations but requires manual save

---

## Thread Safety & ROS 2 Integration

### Single-threaded Operation
- ROS 2 node runs in single executor
- Service callbacks execute sequentially
- No explicit locking needed

### TF Broadcasting
- Transform broadcast in main thread
- Published once at initialization
- Re-broadcast after map switch

### Topic Publishing
- `/topological_map_2`: Transient local QoS (late joiners receive)
- Published after every `update()` call

---

## Validation & Safety

### Pre-operation Checks
- Node existence before add (prevents duplicates)
- Destination node existence before add_edge
- Edge ID existence before remove_edge

### Post-operation Validation
- Optional: Call `model.validate()` after batch operations
- Schema validation catches structural errors
- Logical validation catches semantic errors

### Atomic Operations
- Model updates are NOT transactional
- Partial failures possible in batch operations
- Recommendation: Validate input before batch operations

---

## Extension Points

### Custom Actions
1. Add action configuration to `navigation_goal.yaml`
2. Create goal YAML in package config directory
3. Use standard `add_edge()` with new action name

### Custom Tags
- Tags are free-form strings
- No pre-defined vocabulary
- Convention: `snake_case` recommended

### Custom Node Properties
- Add to `node["node"]["properties"]` dict
- No schema constraints (flexible)
- Access via `model.get_node()`

---

## Legacy Support

### tmap vs tmap2 Format
- tmap2: YAML-based, flexible schema
- tmap: ROS 1 message format (legacy)
- Conversion: `convert_to_legacy` parameter (disabled by default)

---

## Summary Table: Service → Method Mapping

| Service Name | Callback Method | Core Method | Model Method |
|--------------|----------------|-------------|--------------|
| `get_topological_map` | `get_topological_map_cb` | - | - |
| `switch_topological_map` | `switch_topological_map_cb` | `load_map` | `model.load()` |
| `add_topological_node` | `add_topological_node_cb` | `add_topological_node` | `model.add_node()` |
| `remove_topological_node` | `remove_node_cb` | `remove_node` | `model.remove_node()` |
| `add_edges_between_nodes` | `add_edge_cb` | `add_edge` | `model.add_edge()` |
| `remove_edge` | `remove_edge_cb` | `remove_edge` | `model.remove_edge()` |
| `update_node_name` | `update_node_name_cb` | `update_node_name` | Manual update |
| `update_node_pose` | `update_node_waypoint_cb` | `update_node_waypoint` | `model.update_node_pose()` |
| `write_topological_map` | `write_topological_map_cb` | `write_topological_map` | `model.save()` |

---

## Diagram Legend

```
┌─────────┐
│ Class   │ = Component/Module
└─────────┘

    │
    ├──→  = Composition/Dependency
    │
    ▼     = Data Flow

• Bullet = Attribute/Property
```

---

## Document Metadata

- **Generated**: 2026-02-05
- **Map Manager Version**: Refactored 2026-02-05
- **ROS 2 Distribution**: Humble
- **Total Services**: 35
- **Total Methods**: 75+
- **Total Lines of Code**: ~1100
