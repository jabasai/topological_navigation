# Flexible Node and Edge Properties System

This document describes the flexible properties system for topological maps, which allows application-specific metadata to be attached to both nodes and edges without requiring schema modifications.

## Overview

The topological map schema supports optional `properties` dictionaries for both nodes and edges. These properties enable domain-specific customisation while maintaining backwards compatibility with existing maps.

## Node Properties

Node properties are defined in `nodes[].node.properties` as a YAML dictionary. The schema allows any key-value pairs.

### Default Properties

The following properties are commonly used for navigation control:

| Property | Type | Description |
|----------|------|-------------|
| `xy_goal_tolerance` | float | XY position tolerance for goal reaching (metres) |
| `yaw_goal_tolerance` | float | Yaw orientation tolerance for goal reaching (radians) |

### Example Custom Properties

| Property | Type | Description |
|----------|------|-------------|
| `row` | integer | Row identifier (e.g., for agricultural polytunnel scenarios) |
| `semantics` | string | Semantic meaning of the node (e.g., "charging_station", "inspection_point") |
| `zone` | string | Operational zone designation |
| `access_level` | string | Permission level required for access |
| `capacity` | integer | Maximum number of robots that can occupy the node |

### Node Properties Example

```yaml
nodes:
- meta:
    map: riseholme
    node: ChargingStation1
    pointset: riseholme
  node:
    name: ChargingStation1
    parent_frame: map
    pose:
      position: {x: 10.0, y: 5.0, z: 0.0}
      orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
    properties:
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.1
      semantics: "charging_station"
      row: 3
      zone: "A"
      capacity: 2
    edges: []
```

## Edge Properties

Edge properties are defined in `nodes[].node.edges[].properties` as a YAML dictionary. The schema allows any key-value pairs.

### Example Edge Properties

| Property | Type | Description |
|----------|------|-------------|
| `max_speed` | float | Maximum traversal speed (m/s) |
| `priority` | integer | Preference weight for path planning (higher = more preferred) |
| `width` | float | Physical width of the traversable path (metres) |
| `surface_type` | string | Terrain classification (e.g., "concrete", "grass", "gravel") |
| `bidirectional` | boolean | Whether the edge can be traversed in both directions |
| `weather_restrictions` | list | Conditions under which edge should not be used |

### Edge Properties Example

```yaml
edges:
- edge_id: ChargingStation1_WayPoint2
  node: WayPoint2
  action: NavigateToPose
  action_type: geometry_msgs/PoseStamped
  properties:
    max_speed: 0.5
    priority: 10
    surface_type: "concrete"
    bidirectional: true
    weather_restrictions: ["heavy_rain", "snow"]
  # ... other edge fields
```

## Backwards Compatibility

The `properties` field is optional for both nodes and edges. Existing topological maps without properties remain valid and will continue to work without modification.

When properties are not specified:
- Node properties default to standard navigation tolerances if needed by the navigation system
- Edge properties are simply absent (empty dictionary)

## Usage Guidelines

### Naming Conventions

- Use `snake_case` for property names
- Use descriptive, domain-appropriate names
- Avoid abbreviations unless they are widely understood

### Using Namespaces for Properties

To organise properties by functional area or application, use **namespaces**. A namespace is simply a nested dictionary that groups related properties together. This approach:

- Prevents naming conflicts between different systems
- Makes it clear which application or module owns each property
- Improves readability and maintainability

#### Namespace Structure

```yaml
properties:
  namespace_name:
    property1: value1
    property2: value2
```

#### Common Namespace Examples

| Namespace | Purpose |
|-----------|---------|
| `navigation` | Core navigation parameters (tolerances, speeds) |
| `fleet_management` | Multi-robot coordination properties |
| `logistics` | Warehouse/delivery application properties |
| `agriculture` | Agricultural robotics properties |
| `safety` | Safety-related constraints and limits |
| `semantics` | Semantic labels and classifications |

#### Namespaced Node Properties Example

```yaml
node:
  name: ChargingStation1
  properties:
    navigation:
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.1
    fleet_management:
      capacity: 2
      priority_zone: true
      queue_enabled: true
    semantics:
      type: "charging_station"
      zone: "A"
    logistics:
      pickup_point: false
      dropoff_point: false
```

#### Namespaced Edge Properties Example

```yaml
edges:
- edge_id: ChargingStation1_WayPoint2
  node: WayPoint2
  properties:
    navigation:
      max_speed: 0.5
      bidirectional: true
    safety:
      width: 1.2
      restricted_hours: ["22:00-06:00"]
    logistics:
      priority: 10
      surface_type: "concrete"
```

#### Accessing Namespaced Properties in Code

```python
# Safe access to namespaced node properties
node_props = node["node"].get("properties", {})

# Access fleet_management namespace
fleet_props = node_props.get("fleet_management", {})
capacity = fleet_props.get("capacity", 1)

# Access navigation namespace
nav_props = node_props.get("navigation", {})
xy_tolerance = nav_props.get("xy_goal_tolerance", 0.3)

# Access namespaced edge properties
edge_props = edge.get("properties", {})
safety_props = edge_props.get("safety", {})
max_width = safety_props.get("width")
```

#### Guidelines for Namespace Usage

1. **Use namespaces for application-specific properties**: If your properties are only used by a specific application or module, group them under a namespace.
2. **Keep core navigation properties at root level**: For backwards compatibility, standard navigation properties like `xy_goal_tolerance` can remain at the root level.
3. **Document your namespaces**: If you introduce a new namespace, document its purpose and expected properties.
4. **Avoid deep nesting**: One level of namespace nesting is usually sufficient. Avoid creating deeply nested structures.
5. By convention, ROS packages should use their package name as the namespace. E.g. a package called "topfleets_coordinator" should use this as the namespace for properties it uses.

### Type Flexibility

Properties support various data types:
- **Strings**: `"charging_station"`, `"concrete"`
- **Numbers**: `0.5`, `10`, `3.14`
- **Booleans**: `true`, `false`
- **Lists**: `["heavy_rain", "snow"]`
- **Nested Objects**: `{min: 0.1, max: 1.0}`

### Application-Specific Properties

Applications can define their own property schemas and document them appropriately. The topological navigation system will safely ignore properties it does not recognise, allowing different applications to coexist on the same map.

### Accessing Properties in Code

When accessing properties programmatically, always check for property existence before use:

```python
# Safe access to node properties
node_props = node["node"].get("properties", {})
xy_tolerance = node_props.get("xy_goal_tolerance", 0.3)  # Default to 0.3
semantics = node_props.get("semantics")  # Returns None if not present

# Safe access to edge properties
edge_props = edge.get("properties", {})
max_speed = edge_props.get("max_speed")  # Returns None if not present
if max_speed is not None:
    # Use max_speed for navigation control
    pass
```

## Schema Reference

The properties fields are defined in `config/tmap-schema.yaml`:

```yaml
# Node properties (at nodes[].node.properties)
properties:
  type: object
  additionalProperties: true
  description: Flexible dictionary of application-specific node properties

# Edge properties (at nodes[].node.edges[].properties)
properties:
  type: object
  additionalProperties: true
  description: Flexible dictionary of application-specific edge properties
```

## Validating Topological Maps

The `validate_map.py` script can be used to validate topological map YAML files against the schema:

```bash
# Basic validation
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml

# With verbose output
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml -v

# With custom schema file
ros2 run topological_navigation validate_map.py my_map.tmap2.yaml --schema custom_schema.yaml
```

The validator will:
- Check the map structure against the JSON schema
- Report any missing required fields
- Warn about duplicate node names
- Warn about edges pointing to non-existent nodes

## Related Resources

- [Topological Map Schema](../config/tmap-schema.yaml)
- [Node Template](../config/template_node_2.yaml)
- [Edge Template](../config/template_edge.yaml)
