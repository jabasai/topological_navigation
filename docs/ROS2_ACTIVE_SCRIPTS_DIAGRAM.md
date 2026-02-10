# ROS2 Active Scripts - Visual Diagram

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ROS2 Topological Navigation System                        │
│                         (Active Components Only)                             │
└─────────────────────────────────────────────────────────────────────────────┘

                              ┌──────────────────┐
                              │   YAML Map File  │
                              │  (*.tmap2.yaml)  │
                              └────────┬─────────┘
                                       │
                                       ▼
                    ┌──────────────────────────────────┐
                    │     map_manager2.py              │
                    │  (Map Loading & Publishing)      │
                    │                                  │
                    │  Dependencies:                   │
                    │  • manager2.py                   │
                    │  • load_maps_from_yaml.py        │
                    │  • map_types.py                  │
                    └────────┬─────────────────────────┘
                             │
                             │ Publishes
                             ▼
                    /topological_map_2 (String/YAML)
                             │
                             │ Subscribes
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐   ┌────────────────┐   ┌──────────────────┐
│localisation2  │   │ navigation2.py │   │get_simple_policy2│
│     .py       │   │                │   │      .py         │
│               │   │ Main Nav Server│   │                  │
│ Localization  │   │                │   │ Route Planning   │
│               │   │ Dependencies:  │   │                  │
│ Dependencies: │   │ • route_search2│   │ Dependencies:    │
│ • tmap_utils  │   │ • edge_action_ │   │ • route_search2  │
│ • point2line  │   │   manager2     │   │ • tmap_utils     │
│ • map_types   │   │ • edge_reconf_ │   │                  │
│               │   │   manager2     │   │                  │
│               │   │ • goal_builder │   │                  │
│               │   │ • navigation_  │   │                  │
│               │   │   stats        │   │                  │
│               │   │ • actions_bt   │   │                  │
└───────┬───────┘   └────────┬───────┘   └────────┬─────────┘
        │                    │                     │
        │ Publishes          │ Publishes           │ Provides
        ▼                    ▼                     ▼
┌───────────────┐   ┌────────────────┐   ┌──────────────────┐
│/current_node  │   │/topological_   │   │Services:         │
│/closest_node  │   │ navigation/    │   │• get_route_to    │
│/closest_edges │   │ Route          │   │• get_route_      │
│/current_node/ │   │/current_edge   │   │  between         │
│ tag           │   │/topological_   │   │                  │
│               │   │ navigation/    │   │                  │
│               │   │ Statistics     │   │                  │
└───────────────┘   └────────┬───────┘   └──────────────────┘
                             │
                             │ Action Server
                             ▼
                    ┌────────────────────┐
                    │ /topological_      │
                    │  navigation        │
                    │ (GotoNode Action)  │
                    │                    │
                    │ /topological_      │
                    │  navigation/       │
                    │  execute_policy_   │
                    │  mode              │
                    └────────┬───────────┘
                             │
                             │ Calls
                             ▼
                    ┌────────────────────┐
                    │   Nav2 Stack       │
                    │ (External System)  │
                    │                    │
                    │ Actions:           │
                    │ • NavigateToPose   │
                    │ • NavigateThrough  │
                    │   Poses            │
                    │ • FollowWaypoints  │
                    └────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                         Visualization Components                             │
└─────────────────────────────────────────────────────────────────────────────┘

        ┌────────────────────┐         ┌────────────────────┐
        │visualise_map_ros2  │         │ topomap_marker2.py │
        │      .py           │         │                    │
        │                    │         │  Map Markers       │
        │ Interactive RViz   │         │                    │
        │ Map Editor         │         │ Subscribes:        │
        │                    │         │ • /topological_    │
        │ Subscribes:        │         │   map_2            │
        │ • /topological_    │         │                    │
        │   map_2            │         │ Publishes:         │
        │ • /topological_    │         │ • /topological_    │
        │   navigation/Route │         │   map_markers      │
        │ • /current_node    │         │                    │
        │                    │         └────────────────────┘
        │ Action Client:     │
        │ • /topological_    │         ┌────────────────────┐
        │   navigation       │         │topological_visual  │
        │                    │         │      .py           │
        │ Publishes:         │         │                    │
        │ • Interactive      │         │ Route Visualization│
        │   Markers          │         │                    │
        └────────────────────┘         │ Subscribes:        │
                                       │ • /topological_    │
        ┌────────────────────┐         │   navigation/Route │
        │ policy_marker2.py  │         │ • /topological_    │
        │                    │         │   occupied_nodes   │
        │ Policy Markers     │         │                    │
        │                    │         │ Publishes:         │
        │ Subscribes:        │         │ • /topological_    │
        │ • /topological_    │         │   route_markers    │
        │   map_2            │         │                    │
        │                    │         └────────────────────┘
        │ Publishes:         │
        │ • Policy markers   │
        └────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                         Supporting Utilities                                 │
└─────────────────────────────────────────────────────────────────────────────┘

        ┌────────────────────┐         ┌────────────────────┐
        │occupancy_checker   │         │topological_        │
        │      .py           │         │ transform_         │
        │                    │         │ publisher.py       │
        │ Multi-Robot        │         │                    │
        │ Occupancy          │         │ TF Publisher       │
        │                    │         │                    │
        │ Subscribes:        │         │ Subscribes:        │
        │ • /topological_    │         │ • /topological_    │
        │   map_2            │         │   map_2            │
        │ • /robot_poses     │         │                    │
        │                    │         │ Publishes:         │
        │ Publishes:         │         │ • TF transforms    │
        │ • /topological_    │         │   for nodes        │
        │   occupied_nodes   │         │                    │
        └────────────────────┘         └────────────────────┘

        ┌────────────────────┐         ┌────────────────────┐
        │manual_topomapping  │         │ validate_map.py    │
        │      .py           │         │                    │
        │                    │         │ Map Validation     │
        │ Manual Map         │         │                    │
        │ Creation Tool      │         │ Validates YAML     │
        │                    │         │ against schema     │
        │ Subscribes:        │         │                    │
        │ • /odom            │         │ Standalone utility │
        │                    │         │                    │
        │ Creates map files  │         └────────────────────┘
        └────────────────────┘
```

## Core Library Dependencies

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ROS2 Core Libraries                                  │
└─────────────────────────────────────────────────────────────────────────────┘

manager2.py (58KB)
├── Core map management
├── Map loading and validation
├── Node/edge indexing
└── Service providers

route_search2.py
├── A* path planning algorithm
├── Cost calculation
├── Restriction checking
└── Route optimization

edge_action_manager2.py (67KB - Complex)
├── Edge action execution
├── Nav2 action client management
├── Row operation handling
├── Goal construction
└── Failure recovery

edge_reconfigure_manager2.py
├── Parameter reconfiguration
├── Nav2 parameter updates
└── Edge-specific settings

goal_builder.py
├── Navigation goal construction
├── Property application
├── Behavior tree selection
└── Pose transformation

row_operation_handler.py
├── Agricultural row operations
├── Boundary node detection
├── Waypoint generation
└── Row-specific navigation

param_processing.py
├── ROS2 parameter handling
├── Parameter service clients
└── Dynamic reconfiguration

actions_bt.py
├── Behavior tree action types
├── Action type definitions
└── Planner configuration

navigation_stats.py
├── Navigation statistics
├── Performance tracking
└── Logging utilities
```

## Shared Utilities (No ROS Dependency)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Shared Utility Libraries                             │
└─────────────────────────────────────────────────────────────────────────────┘

tmap_utils.py
├── Map utility functions
├── Node/edge lookup
├── Distance calculations
└── Map manipulation

point2line.py
├── Geometric calculations
├── Point-to-line distance
└── Spatial algorithms

map_types.py
├── Type definitions
├── CustomSafeLoader
└── Data structures

load_maps_from_yaml.py
├── YAML map loading
├── File parsing
└── Map deserialization
```

## Data Flow: Typical Navigation Scenario

```
1. System Startup
   ├─> map_manager2.py loads map
   ├─> localisation2.py starts localizing
   └─> navigation2.py waits for map and localization

2. User Sends Navigation Goal
   └─> /topological_navigation action (GotoNode)
       └─> navigation2.py receives goal

3. Route Planning
   └─> route_search2.py
       └─> A* search from current to target node
           └─> Returns ordered list of nodes and edges

4. Route Execution
   └─> For each edge in route:
       ├─> edge_reconfigure_manager2.py updates parameters
       ├─> edge_action_manager2.py executes edge action
       │   └─> goal_builder.py constructs Nav2 goal
       │       └─> Sends to Nav2 action server
       │           └─> Robot moves
       └─> localisation2.py updates current node

5. Goal Reached
   └─> navigation2.py returns success
       └─> Client receives result
```

## Minimal Working System

To run basic topological navigation, you need these 4 nodes:

```
┌──────────────────┐
│ map_manager2.py  │  Load map from YAML
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│localisation2.py  │  Localize robot
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ navigation2.py   │  Execute navigation
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│   Nav2 Stack     │  Metric navigation
└──────────────────┘
```

Plus optional visualization:
- `visualise_map_ros2.py` for interactive editing
- `topomap_marker2.py` for map visualization
- `topological_visual.py` for route visualization

## Script Usage Frequency

```
┌─────────────────────────────────────────────────────────────────┐
│                    Usage Classification                          │
└─────────────────────────────────────────────────────────────────┘

ESSENTIAL (Always Running):
├─> map_manager2.py          ████████████████████ 100%
├─> localisation2.py         ████████████████████ 100%
└─> navigation2.py           ████████████████████ 100%

COMMON (Often Running):
├─> visualise_map_ros2.py    ████████████████     80%
├─> topomap_marker2.py       ████████████████     80%
├─> topological_visual.py    ████████████         60%
└─> get_simple_policy2.py    ████████████         60%

OPTIONAL (Situational):
├─> occupancy_checker.py     ████████             40%
├─> topological_transform_   ████                 20%
│   publisher.py
└─> policy_marker2.py        ████                 20%

UTILITY (Development/Setup):
├─> manual_topomapping.py    ██                   10%
└─> validate_map.py          ██                   10%
```

## Summary

**Total Active ROS2 Scripts: 12**
- 4 Core navigation nodes (essential)
- 4 Visualization tools (common)
- 2 Supporting utilities (optional)
- 2 Development utilities (rare)

**Total Active ROS2 Libraries: 10**
- Core libraries used by navigation system
- Shared utilities with no ROS dependency

All other scripts (18 ROS1 scripts) are legacy and should not be used for new development.
