# Fluid Navigation Parameter

## Overview

The `fluid_navigation` parameter is a boolean flag on topological map edges that controls whether the robot stops at intermediate waypoints or flows smoothly through them during navigation.

## What It Does

### When `fluid_navigation: true` (Default)

The robot will **pass through intermediate waypoints smoothly** without stopping, as long as:
- Both the current edge and next edge use navigation actions (e.g., `move_base`, `human_aware_navigation`)
- The waypoint is an intermediate node (not the final destination)

**Behavior:**
- ✅ Smooth, continuous motion through waypoint sequences
- ✅ Ignores exact orientation of intermediate waypoints
- ✅ More efficient for long routes with many waypoints
- ✅ Better for fluid motion in corridors or rows

**Example Use Case:**
```
Node A → Node B → Node C → Node D (goal)
```
With `fluid_navigation: true`, the robot smoothly flows through B and C without stopping, only stopping at D.

### When `fluid_navigation: false`

The robot will **stop and align precisely** at each waypoint.

**Behavior:**
- ✅ Robot stops at every waypoint
- ✅ Robot aligns to exact pose (position + orientation) of the waypoint
- ✅ Useful when action needs to be performed at the waypoint
- ✅ Necessary for non-navigation actions (e.g., docking, scanning)

**Example Use Case:**
```
Node A → Node B (inspection point) → Node C (goal)
```
With `fluid_navigation: false` on the A→B edge, the robot stops precisely at B before continuing.

## When to Use Each Setting

| Use `fluid_navigation: true` | Use `fluid_navigation: false` |
|------------------------------|-------------------------------|
| Long sequences of navigation waypoints | Waypoint requires precise positioning |
| Corridor or row following | Action needs to be performed at waypoint |
| Intermediate waypoints are just path guides | Next action is not a navigation action |
| Want smooth, efficient motion | Need exact orientation at waypoint |
| Both edges use navigation actions | Safety-critical positioning required |

## Implementation Details

### In the Navigation Code

The navigation controller ([navigation2.py](../topological_navigation/scripts/navigation2.py)) checks this flag:

```python
# Line 583: Check if we should ignore intermediate node orientation
if rindex < len(route.edge_id) - 1 and \
   a1 in self.navigation_actions and \
   a in self.navigation_actions and \
   self.fluid_navigation:
    # Ignore orientation of intermediate waypoint
    self.reconf_movebase(cedg, cnode, True)
else:
    # Respect exact pose of waypoint
    self.reconf_movebase(cedg, cnode, False)
```

Additionally, when the robot reaches an intermediate node:

```python
# Lines 468-476: Goal completion logic
if (self.current_node == self.current_target and
    self._target != self.current_target and
    self.next_action in self.navigation_actions and
    self.current_action in self.navigation_actions and
    self.fluid_navigation):
    self.get_logger().info("Intermediate node reached: {}".format(self.current_node))
    self.goal_reached = True  # Continue to next edge
```

### In the Map Manager

When creating or updating edges:

```python
# Default value when creating new edges
edge["fluid_navigation"] = True

# Update via service (note: inverted logic)
# not_fluid=True → fluid_navigation=False (stop at waypoint)
# not_fluid=False → fluid_navigation=True (flow through)
```

## Configuration Examples

### Example 1: Smooth Row Navigation

```yaml
edges:
  - edge_id: "row_start_to_wp1"
    node: "waypoint_1"
    action: "move_base"
    fluid_navigation: true  # Flow through smoothly
    
  - edge_id: "wp1_to_wp2"
    node: "waypoint_2"
    action: "move_base"
    fluid_navigation: true  # Continue flowing
    
  - edge_id: "wp2_to_row_end"
    node: "row_end"
    action: "move_base"
    fluid_navigation: false  # Stop at end for next operation
```

### Example 2: Inspection at Waypoints

```yaml
edges:
  - edge_id: "start_to_inspection_1"
    node: "inspection_point_1"
    action: "move_base"
    fluid_navigation: false  # Must stop for inspection
    
  - edge_id: "inspection_1_to_inspection_2"
    node: "inspection_point_2"
    action: "move_base"
    fluid_navigation: false  # Must stop for inspection
```

### Example 3: Mixed Actions

```yaml
edges:
  - edge_id: "A_to_B"
    node: "B"
    action: "move_base"
    fluid_navigation: true  # Can flow if next action is also navigation
    
  - edge_id: "B_to_C"
    node: "C"
    action: "dock"  # Non-navigation action
    fluid_navigation: false  # Automatically stops (not a navigation action)
```

## Setting the Parameter

### Via Service Call

```python
# Update edge with fluid navigation disabled
self.update_edge(
    edge_id="node_a_node_b",
    action_name="move_base",
    action_type="geometry_msgs/PoseStamped",
    goal=None,
    fail_policy="fail",
    not_fluid=True  # This sets fluid_navigation=False
)
```

### In YAML Map Files

```yaml
nodes:
  - node:
      name: "waypoint_1"
      edges:
        - edge_id: "waypoint_1_waypoint_2"
          node: "waypoint_2"
          action: "move_base"
          fluid_navigation: true  # Default
```

### Via Python API

```python
# Access and modify directly
node = self.model.get_node("waypoint_1")
for edge in node["node"]["edges"]:
    if edge["edge_id"] == "waypoint_1_waypoint_2":
        edge["fluid_navigation"] = True  # or False
```

## Default Behavior

- **Default value:** `true` (fluid navigation enabled)
- **Set when:** Edge is created via `add_edge()` or loaded from file
- **Affects:** Only intermediate waypoints during navigation action sequences
- **Does not affect:** Final destination (robot always stops at goal)

## Troubleshooting

### Problem: Robot Not Stopping at Waypoints

**Check:**
1. Is `fluid_navigation: true` on the edges?
2. Are both current and next actions navigation actions?
3. Is the waypoint an intermediate node (not the final goal)?

**Solution:** Set `fluid_navigation: false` on edges where stopping is required.

### Problem: Robot Jerky Motion Between Waypoints

**Check:**
1. Is `fluid_navigation: false` when it should be `true`?
2. Are waypoints too close together?

**Solution:** Enable `fluid_navigation: true` for smoother motion through waypoint sequences.

### Problem: Robot Ignores Waypoint Orientation

**Check:**
1. Is `fluid_navigation: true`?
2. This is expected behavior for intermediate waypoints!

**Solution:** If orientation matters, set `fluid_navigation: false`.

## Related Parameters

- **`no_orientation`**: Global flag to ignore all waypoint orientations
- **`nav_from_closest_edge`**: Controls whether robot navigates to closest edge start
- **Navigation actions**: List of action types considered as "navigation" (affects fluid navigation logic)

## See Also

- [Manager2 Documentation](../doc/MANAGER2_DOCUMENTATION.md)
- [Navigation2 Script](../topological_navigation/scripts/navigation2.py)
- [Edge Schema](../config/template_edge.yaml)
