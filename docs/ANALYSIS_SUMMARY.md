# Topological Navigation - ROS Version Analysis Summary

## Overview

This analysis provides a complete breakdown of the topological_navigation codebase, categorizing all scripts by ROS version and documenting the ROS2 system architecture.

## Key Findings

### 1. Codebase Composition

| Category | Count | Percentage |
|----------|-------|------------|
| ROS2 Scripts | 12 | 40% |
| ROS1 Scripts | 18 | 60% |
| **Total Scripts** | **30** | **100%** |

| Category | Count | Percentage |
|----------|-------|------------|
| ROS2 Libraries | 10 | 31% |
| ROS1 Libraries | 18 | 56% |
| Shared Libraries | 4 | 13% |
| **Total Libraries** | **32** | **100%** |

### 2. ROS2 Core System (4 Essential Nodes)

The ROS2 topological navigation system is built around 4 core nodes:

1. **map_manager2.py** - Loads and publishes topological maps
2. **localisation2.py** - Localizes robot in topological space
3. **navigation2.py** - Main navigation action server
4. **get_simple_policy2.py** - Route planning services

### 3. Active vs Legacy Code

**Active ROS2 Code:**
- 12 executable scripts
- 10 core libraries
- 4 shared utilities
- **Total: 26 active files**

**Legacy ROS1 Code:**
- 18 executable scripts
- 18 core libraries
- **Total: 36 legacy files**

**Recommendation:** Focus development on ROS2 code. ROS1 code is maintained for backward compatibility only.

## Document Index

This analysis consists of 4 comprehensive documents:

### 1. ROS_VERSION_ANALYSIS.md
**Purpose:** Complete classification of all scripts and libraries by ROS version

**Contents:**
- ROS2 vs ROS1 script categorization
- Core library classification
- System architecture overview
- Communication patterns (topics, services, actions)
- Active vs inactive script identification
- Development recommendations

**Use this for:** Understanding which files are ROS1 vs ROS2

### 2. ROS2_CALL_GRAPH.md
**Purpose:** Detailed call graph showing how ROS2 components interact

**Contents:**
- System initialization flow
- Navigation execution flow
- Localization process
- Route planning algorithm
- Edge action execution
- Map management
- Parameter reconfiguration
- Complete data flow diagrams
- Key function calls
- Error handling

**Use this for:** Understanding how the ROS2 system works internally

### 3. SCRIPT_CLASSIFICATION_SUMMARY.md
**Purpose:** Quick reference guide for script classification

**Contents:**
- Quick reference tables (ROS1 vs ROS2)
- Setup.py entry points analysis
- Dependency graphs
- Migration guide (ROS1 to ROS2)
- Minimal setup requirements
- File count summary

**Use this for:** Quick lookups and migration planning

### 4. ROS2_ACTIVE_SCRIPTS_DIAGRAM.md
**Purpose:** Visual diagrams of active ROS2 components

**Contents:**
- System architecture diagram
- Visualization components
- Supporting utilities
- Core library dependencies
- Data flow diagrams
- Minimal working system
- Usage frequency classification

**Use this for:** Visual understanding of the system

## Quick Start Guide

### For New Developers

1. **Read first:** `SCRIPT_CLASSIFICATION_SUMMARY.md` - Get oriented
2. **Read second:** `ROS2_ACTIVE_SCRIPTS_DIAGRAM.md` - See the big picture
3. **Deep dive:** `ROS2_CALL_GRAPH.md` - Understand the details
4. **Reference:** `ROS_VERSION_ANALYSIS.md` - Complete information

### For Existing Developers

1. **Migration:** Use `SCRIPT_CLASSIFICATION_SUMMARY.md` for ROS1→ROS2 mapping
2. **Debugging:** Use `ROS2_CALL_GRAPH.md` to trace execution flow
3. **Architecture:** Use `ROS2_ACTIVE_SCRIPTS_DIAGRAM.md` for system overview

## Key Insights

### 1. Dual ROS Support

The codebase maintains both ROS1 and ROS2 implementations:
- ROS2 scripts typically end with `2.py` (e.g., `navigation2.py`)
- ROS1 scripts use original names (e.g., `navigation.py`)
- Core functionality has feature parity between versions

### 2. ROS2 System Architecture

```
YAML Map → map_manager2 → /topological_map_2
                              ↓
                    ┌─────────┼─────────┐
                    ↓         ↓         ↓
              localisation  navigation  get_simple_policy2
                    ↓         ↓
              /current_node   ↓
              /closest_node   ↓
                         Nav2 Stack
```

### 3. Core Dependencies

**ROS2 Navigation depends on:**
- `manager2.py` - Map management
- `route_search2.py` - A* path planning
- `edge_action_manager2.py` - Edge execution (complex, 67KB)
- `goal_builder.py` - Goal construction
- `navigation_stats.py` - Statistics
- `actions_bt.py` - Action types

**External dependencies:**
- Nav2 stack (NavigateToPose, NavigateThroughPoses, FollowWaypoints)
- TF2 (coordinate transforms)
- ROS2 action/service infrastructure

### 4. Missing ROS2 Features

Some ROS1 features don't have ROS2 equivalents yet:
- Travel time prediction system
- Standalone restrictions manager (integrated into navigation2)
- Some visualization variants

### 5. Agricultural Robotics Support

The system includes specialized support for agricultural operations:
- `row_operation_handler.py` - Row navigation
- Boundary node detection
- Roboflow integration for vision
- Specialized behavior trees for in-row operations

## Recommendations

### For Development

1. **Use ROS2 exclusively** for new features
2. **Focus on the 4 core nodes** for navigation work
3. **Extend edge_action_manager2.py** for new action types
4. **Use properties system** for domain-specific metadata
5. **Follow existing patterns** in route_search2.py for planning

### For Maintenance

1. **Consider deprecating ROS1** if no longer needed
2. **Document active vs inactive** scripts clearly
3. **Create integration tests** for core navigation flow
4. **Monitor edge_action_manager2.py** (complex, 67KB)

### For Deployment

**Minimal system requires:**
- map_manager2.py
- localisation2.py
- navigation2.py
- Nav2 stack

**Recommended additions:**
- visualise_map_ros2.py (for debugging)
- topomap_marker2.py (for visualization)
- get_simple_policy2.py (for route planning services)

## File Locations

All analysis documents are in the repository root:

```
topological_navigation/
├── ANALYSIS_SUMMARY.md                  (This file)
├── ROS_VERSION_ANALYSIS.md              (Complete classification)
├── ROS2_CALL_GRAPH.md                   (Detailed call graph)
├── SCRIPT_CLASSIFICATION_SUMMARY.md     (Quick reference)
└── ROS2_ACTIVE_SCRIPTS_DIAGRAM.md       (Visual diagrams)
```

## Statistics

### Code Complexity

| Component | Lines | Complexity |
|-----------|-------|------------|
| edge_action_manager2.py | 1,363 | High |
| manager2.py | 1,200+ | High |
| navigation2.py | 1,331 | High |
| route_search2.py | 500+ | Medium |
| localisation2.py | 600+ | Medium |

### Topic/Service/Action Count

**Topics Published:** 10+
- /topological_map_2
- /current_node
- /closest_node
- /closest_edges
- /topological_navigation/Route
- /topological_navigation/Statistics
- /current_edge
- And more...

**Services Provided:** 6+
- Route planning services
- Localization services
- Map management services

**Actions Provided:** 2
- /topological_navigation (GotoNode)
- /topological_navigation/execute_policy_mode

**Actions Used (Nav2):** 3+
- NavigateToPose
- NavigateThroughPoses
- FollowWaypoints

## Conclusion

The topological_navigation codebase is a mature system with both ROS1 and ROS2 support. The ROS2 implementation is feature-complete and production-ready, centered around 4 core nodes that provide topological navigation capabilities integrated with Nav2 for metric navigation.

For new development, focus exclusively on ROS2 components. The system is well-architected with clear separation of concerns:
- Map management (manager2.py)
- Localization (localisation2.py)
- Navigation execution (navigation2.py)
- Route planning (route_search2.py)
- Edge actions (edge_action_manager2.py)

The flexible properties system allows domain-specific customization without schema changes, making it suitable for various applications including agricultural robotics.

## Next Steps

1. **Review** the 4 analysis documents based on your needs
2. **Identify** which scripts you need for your use case
3. **Focus** on ROS2 components for development
4. **Leverage** the existing architecture for new features
5. **Consider** deprecating ROS1 code if not needed

---

**Analysis Date:** February 10, 2026  
**Codebase Version:** topological_navigation v3.0.5  
**ROS2 Distribution:** Humble/Iron compatible
