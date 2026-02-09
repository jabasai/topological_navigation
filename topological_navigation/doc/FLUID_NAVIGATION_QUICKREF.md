# Fluid Navigation - Quick Reference Card

## TL;DR

**`fluid_navigation`** controls whether robots stop at intermediate waypoints during navigation.

| Setting | Robot Behavior | Use When |
|---------|---------------|----------|
| `true` (default) | 🏃 Flows through intermediate waypoints | Efficient path following |
| `false` | 🛑 Stops at each waypoint | Precise positioning needed |

## Quick Decision Guide

```
Do you need the robot to stop at this waypoint?
│
├─ NO → fluid_navigation: true  (flows through)
│         ✓ Faster navigation
│         ✓ Smoother motion
│         ✓ Good for corridors/rows
│
└─ YES → fluid_navigation: false (stops)
          ✓ Precise positioning
          ✓ Perform action at waypoint
          ✓ Required orientation
```

## Common Examples

### ✅ Use `fluid_navigation: true` when:
- Waypoints are just path guides through a corridor
- Long sequences of navigation waypoints  
- You want efficient, smooth motion
- Example: `Row start → WP1 → WP2 → WP3 → Row end`

### ❌ Use `fluid_navigation: false` when:
- Robot needs to perform an action (scan, pick, inspect)
- Exact pose/orientation is critical
- Next action is non-navigation (dock, charge)
- Safety requires stopping
- Example: `Entrance → Checkpoint (stop) → Storage`

## Code Snippets

### Check current value:
```python
edge["fluid_navigation"]  # True or False
```

### Set via service:
```python
update_edge(edge_id="A_B", not_fluid=True)  # Stop at B
update_edge(edge_id="A_B", not_fluid=False) # Flow through B
```

### In YAML map:
```yaml
edges:
  - edge_id: "A_B"
    fluid_navigation: true   # Default, flows through
```

## Important Notes

⚠️ **Inverted Logic in API**: 
- `not_fluid=True` → `fluid_navigation=False` (stops)
- `not_fluid=False` → `fluid_navigation=True` (flows)

✅ **Always Active**:
- Only affects intermediate waypoints
- Final destination always causes robot to stop
- Both edges must be navigation actions for effect

## See Full Documentation
- [FLUID_NAVIGATION.md](FLUID_NAVIGATION.md) - Complete guide
- [FLUID_NAVIGATION_VISUAL.md](FLUID_NAVIGATION_VISUAL.md) - Visual examples
