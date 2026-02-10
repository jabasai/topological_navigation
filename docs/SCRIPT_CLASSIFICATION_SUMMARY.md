# Topological Navigation - Complete Script Classification

## Quick Reference: ROS1 vs ROS2 Scripts

### ROS2 Scripts (Active - Use These)

#### Core Navigation (Essential)
| Script | Purpose | Status |
|--------|---------|--------|
| `navigation2.py` | Main topological navigation action server | **ACTIVE** |
| `localisation2.py` | Topological localization node | **ACTIVE** |
| `map_manager2.py` | Map loading and publishing | **ACTIVE** |
| `get_simple_policy2.py` | Route planning services | **ACTIVE** |

#### Visualization & Tools
| Script | Purpose | Status |
|--------|---------|--------|
| `visualise_map_ros2.py` | Interactive RViz map editor | ACTIVE |
| `topological_visual.py` | Route visualization | ACTIVE |
| `topomap_marker2.py` | Map marker publisher | ACTIVE |
| `policy_marker2.py` | Policy visualization | ACTIVE |
| `occupancy_checker.py` | Multi-robot node occupancy | ACTIVE |
| `topological_transform_publisher.py` | TF publisher for map | ACTIVE |
| `manual_topomapping.py` | Manual map creation tool | UTILITY |
| `validate_map.py` | Map validation utility | UTILITY |

### ROS1 Scripts (Legacy - Avoid for New Development)

#### Core Navigation (Legacy)
| Script | ROS2 Equivalent | Status |
|--------|-----------------|--------|
| `navigation.py` | `navigation2.py` | LEGACY |
| `localisation.py` | `localisation2.py` | LEGACY |
| `map_manager.py` | `map_manager2.py` | LEGACY |
| `get_simple_policy.py` | `get_simple_policy2.py` | LEGACY |

#### Prediction & Statistics (ROS1 Only)
| Script | Purpose | Status |
|--------|---------|--------|
| `topological_prediction.py` | Travel time prediction | LEGACY |
| `mean_based_prediction.py` | Mean-based prediction | LEGACY |
| `speed_based_prediction.py` | Speed-based prediction | LEGACY |
| `manual_edge_predictions.py` | Manual prediction config | LEGACY |
| `evaluate_top_pred.py` | Prediction evaluation | LEGACY |
| `travel_time_estimator.py` | Travel time estimation | LEGACY |
| `navstats_logger.py` | Statistics logging | LEGACY |
| `test_top_pred.py` | Prediction testing | LEGACY |

#### Utilities & Tools (ROS1)
| Script | Purpose | Status |
|--------|---------|--------|
| `visualise_map.py` | ROS1 visualization | LEGACY |
| `visualise_map2.py` | ROS1 visualization variant | LEGACY |
| `map_publisher.py` | Map publishing utility | LEGACY |
| `search_route.py` | Route search utility | LEGACY |
| `restrictions_manager.py` | Navigation restrictions | LEGACY |
| `reconf_at_edges_server.py` | Edge reconfiguration | LEGACY |
| `nav_client.py` | Navigation action client | LEGACY |

### Core Library Files

#### ROS2 Libraries (Use These)
| File | Purpose | Dependencies |
|------|---------|--------------|
| `manager2.py` | Core map management | rclpy, tf2_ros |
| `route_search2.py` | A* path planning | rclpy |
| `edge_action_manager2.py` | Edge action execution | rclpy, Nav2 |
| `edge_reconfigure_manager2.py` | Parameter reconfiguration | rclpy |
| `topomap_marker2.py` | Map marker generation | rclpy |
| `policy_marker2.py` | Policy visualization | rclpy |
| `goal_builder.py` | Navigation goal construction | rclpy |
| `row_operation_handler.py` | Agricultural operations | rclpy |
| `param_processing.py` | Parameter handling | rclpy |
| `actions_bt.py` | Behavior tree action types | None |

#### ROS1 Libraries (Legacy)
| File | ROS2 Equivalent | Status |
|------|-----------------|--------|
| `manager.py` | `manager2.py` | LEGACY |
| `route_search.py` | `route_search2.py` | LEGACY |
| `edge_action_manager.py` | `edge_action_manager2.py` | LEGACY |
| `edge_reconfigure_manager.py` | `edge_reconfigure_manager2.py` | LEGACY |
| `topological_map.py` | Integrated in `manager2.py` | LEGACY |
| `load_maps_from_yaml.py` | Used by both | SHARED |
| `policies.py` | N/A | LEGACY |
| `restrictions_impl.py` | N/A | LEGACY |
| `topomap_marker.py` | `topomap_marker2.py` | LEGACY |
| `policy_marker.py` | `policy_marker2.py` | LEGACY |
| `node_controller.py` | N/A | LEGACY |
| `edge_controller.py` | N/A | LEGACY |
| `vertex_controller.py` | N/A | LEGACY |
| `node_manager.py` | N/A | LEGACY |
| `goto.py` | N/A | LEGACY |
| `edge_std.py` | N/A | LEGACY |
| `marker_arrays.py` | N/A | LEGACY |
| `publisher.py` | N/A | LEGACY |
| `testing.py` | N/A | LEGACY |

#### Shared Utilities (No ROS Dependency)
| File | Purpose | Used By |
|------|---------|---------|
| `tmap_utils.py` | Map utility functions | Both ROS1 & ROS2 |
| `point2line.py` | Geometric calculations | Both ROS1 & ROS2 |
| `navigation_stats.py` | Navigation statistics | Both ROS1 & ROS2 |
| `map_types.py` | Map type definitions | Both ROS1 & ROS2 |

## Setup.py Entry Points Analysis

### Registered Entry Points (27 total)

```python
entry_points={
    'console_scripts': [
        # ROS2 Core (4)
        'navigation2.py',
        'localisation2.py',
        'map_manager2.py',
        'get_simple_policy2.py',
        
        # ROS2 Visualization (5)
        'visualise_map_ros2.py',
        'topological_visual.py',
        'topomap_marker2.py',
        'policy_marker2.py',
        'occupancy_checker.py',
        
        # ROS2 Utilities (3)
        'topological_transform_publisher.py',
        'manual_topomapping.py',
        'validate_map.py',
        
        # ROS1 Legacy (15)
        'navigation.py',
        'localisation.py',  # Note: No .py in actual entry
        'map_manager.py',
        'get_simple_policy.py',  # Note: No .py in actual entry
        'visualise_map.py',
        'visualise_map2.py',
        'map_publisher.py',
        'search_route.py',
        'travel_time_estimator.py',
        'restrictions_manager.py',
        'reconf_at_edges_server.py',
        'nav_client.py',
        'navstats_logger.py',
        'topological_prediction.py',
        'mean_based_prediction.py',
        'speed_based_prediction.py',
        'manual_edge_predictions.py',
        'test_top_pred.py',
    ],
}
```

## Dependency Graph

### ROS2 Core Dependencies

```
navigation2.py
├── manager2.py
├── route_search2.py
│   └── tmap_utils.py
├── edge_action_manager2.py
│   ├── goal_builder.py
│   ├── row_operation_handler.py
│   └── actions_bt.py
├── edge_reconfigure_manager2.py
│   └── param_processing.py
├── navigation_stats.py
└── actions_bt.py

localisation2.py
├── tmap_utils.py
├── point2line.py
├── map_types.py
└── actions_bt.py

map_manager2.py
└── manager2.py
    ├── load_maps_from_yaml.py
    ├── map_types.py
    └── tmap_utils.py

get_simple_policy2.py
└── route_search2.py
    └── tmap_utils.py
```

## Migration Guide: ROS1 to ROS2

### Direct Replacements
| ROS1 Script | ROS2 Script | Notes |
|-------------|-------------|-------|
| `navigation.py` | `navigation2.py` | Full feature parity |
| `localisation.py` | `localisation2.py` | Full feature parity |
| `map_manager.py` | `map_manager2.py` | Full feature parity |
| `get_simple_policy.py` | `get_simple_policy2.py` | Full feature parity |
| `visualise_map.py` | `visualise_map_ros2.py` | Enhanced features |
| `topomap_marker.py` | `topomap_marker2.py` | Full feature parity |
| `policy_marker.py` | `policy_marker2.py` | Full feature parity |

### No Direct ROS2 Equivalent
These ROS1 scripts don't have ROS2 equivalents yet:
- Prediction system (`topological_prediction.py`, `mean_based_prediction.py`, etc.)
- `restrictions_manager.py` (restrictions checked inline in ROS2)
- `reconf_at_edges_server.py` (integrated into `edge_reconfigure_manager2.py`)

## Recommended Minimal ROS2 Setup

For basic topological navigation, you need:

1. **map_manager2.py** - Load and publish your map
2. **localisation2.py** - Localize robot in topological space
3. **navigation2.py** - Execute navigation goals
4. **Nav2 stack** - Metric navigation (external dependency)

Optional but recommended:
- **visualise_map_ros2.py** - Interactive map editing in RViz
- **topomap_marker2.py** - Visualize map in RViz
- **get_simple_policy2.py** - Route planning services

## File Count Summary

| Category | ROS2 | ROS1 | Shared | Total |
|----------|------|------|--------|-------|
| Scripts | 12 | 18 | 0 | 30 |
| Libraries | 10 | 18 | 4 | 32 |
| **Total** | **22** | **36** | **4** | **62** |

## Conclusion

The codebase contains both ROS1 and ROS2 implementations. For new development:
- **Use ROS2 scripts** (those ending in `2.py` or explicitly ROS2)
- **Focus on the 4 core nodes**: navigation2, localisation2, map_manager2, get_simple_policy2
- **Leverage ROS2 libraries**: manager2, route_search2, edge_action_manager2

The ROS1 code is maintained for backward compatibility but should not be used for new features.
