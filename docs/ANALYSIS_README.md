# ROS Version Analysis - Documentation Guide

## What is This?

This is a comprehensive analysis of the topological_navigation codebase that:
1. **Categorizes all scripts** by ROS version (ROS1 vs ROS2)
2. **Documents the ROS2 system** with detailed call graphs
3. **Identifies active vs legacy code**
4. **Provides visual diagrams** of system architecture

## Quick Navigation

### 📋 Start Here
**[ANALYSIS_SUMMARY.md](ANALYSIS_SUMMARY.md)** - Overview and guide to all documents

### 📚 Main Documents

| Document | Purpose | When to Use |
|----------|---------|-------------|
| **[ROS_VERSION_ANALYSIS.md](ROS_VERSION_ANALYSIS.md)** | Complete classification of all files | Need to know if a file is ROS1 or ROS2 |
| **[ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md)** | Detailed execution flow and call graphs | Understanding how the system works |
| **[SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md)** | Quick reference tables and migration guide | Quick lookups and planning |
| **[ROS2_ACTIVE_SCRIPTS_DIAGRAM.md](ROS2_ACTIVE_SCRIPTS_DIAGRAM.md)** | Visual system architecture | Need visual understanding |

## Common Questions

### "Which scripts should I use for ROS2?"

**Answer:** Use scripts ending in `2.py`:
- `navigation2.py` (not `navigation.py`)
- `localisation2.py` (not `localisation.py`)
- `map_manager2.py` (not `map_manager.py`)
- `get_simple_policy2.py` (not `get_simple_policy.py`)

See: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md)

### "What's the minimal setup for ROS2 navigation?"

**Answer:** You need 4 nodes:
1. `map_manager2.py` - Load map
2. `localisation2.py` - Localize robot
3. `navigation2.py` - Execute navigation
4. Nav2 stack (external)

See: [ROS2_ACTIVE_SCRIPTS_DIAGRAM.md](ROS2_ACTIVE_SCRIPTS_DIAGRAM.md) → "Minimal Working System"

### "How does navigation execution work?"

**Answer:** Follow the flow:
1. User sends GotoNode action
2. navigation2.py plans route (route_search2.py)
3. For each edge: execute action (edge_action_manager2.py)
4. edge_action_manager2.py calls Nav2
5. Robot moves, localisation2.py updates position

See: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) → "Navigation Execution Flow"

### "Which files are ROS1 legacy code?"

**Answer:** 18 scripts and 18 libraries are ROS1 only:
- Scripts without `2.py` suffix (e.g., `navigation.py`)
- Prediction system scripts
- Old visualization scripts

See: [ROS_VERSION_ANALYSIS.md](ROS_VERSION_ANALYSIS.md) → "ROS1 Scripts"

### "What are the core ROS2 libraries?"

**Answer:** 10 core libraries:
- `manager2.py` - Map management
- `route_search2.py` - Path planning
- `edge_action_manager2.py` - Edge execution
- `goal_builder.py` - Goal construction
- And 6 more...

See: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md) → "Core Library Files"

### "How do I migrate from ROS1 to ROS2?"

**Answer:** Direct replacements exist:
- `navigation.py` → `navigation2.py`
- `localisation.py` → `localisation2.py`
- `map_manager.py` → `map_manager2.py`

See: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md) → "Migration Guide"

## Document Structure

```
Analysis Documents
│
├── ANALYSIS_SUMMARY.md
│   └── Overview of all findings and document guide
│
├── ROS_VERSION_ANALYSIS.md
│   ├── ROS2 Scripts (12 scripts)
│   ├── ROS1 Scripts (18 scripts)
│   ├── Core Libraries (ROS2: 10, ROS1: 18, Shared: 4)
│   ├── System Architecture
│   ├── Communication Patterns
│   └── Recommendations
│
├── ROS2_CALL_GRAPH.md
│   ├── System Initialization Flow
│   ├── Navigation Execution Flow
│   ├── Localization Flow
│   ├── Route Planning Flow
│   ├── Edge Action Execution Flow
│   ├── Map Management Flow
│   ├── Parameter Reconfiguration Flow
│   ├── Data Flow Summary
│   ├── Key Function Calls
│   └── Error Handling Flow
│
├── SCRIPT_CLASSIFICATION_SUMMARY.md
│   ├── Quick Reference Tables
│   ├── Setup.py Entry Points Analysis
│   ├── Dependency Graphs
│   ├── Migration Guide (ROS1 → ROS2)
│   ├── Minimal Setup Requirements
│   └── File Count Summary
│
└── ROS2_ACTIVE_SCRIPTS_DIAGRAM.md
    ├── System Architecture Diagram
    ├── Visualization Components
    ├── Supporting Utilities
    ├── Core Library Dependencies
    ├── Data Flow Diagrams
    ├── Minimal Working System
    └── Usage Frequency Classification
```

## Reading Paths

### Path 1: New Developer (Never seen this code)
1. Read: [ANALYSIS_SUMMARY.md](ANALYSIS_SUMMARY.md) - Get oriented (5 min)
2. Read: [ROS2_ACTIVE_SCRIPTS_DIAGRAM.md](ROS2_ACTIVE_SCRIPTS_DIAGRAM.md) - See the system (10 min)
3. Read: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) - Understand details (30 min)
4. Reference: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md) - As needed

### Path 2: Migrating from ROS1
1. Read: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md) → "Migration Guide" (5 min)
2. Read: [ROS_VERSION_ANALYSIS.md](ROS_VERSION_ANALYSIS.md) → "ROS2 System Architecture" (10 min)
3. Reference: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) - For implementation details

### Path 3: Debugging Navigation Issues
1. Read: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) → "Navigation Execution Flow" (10 min)
2. Read: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) → "Error Handling Flow" (5 min)
3. Reference: [ROS2_ACTIVE_SCRIPTS_DIAGRAM.md](ROS2_ACTIVE_SCRIPTS_DIAGRAM.md) → "Data Flow"

### Path 4: Adding New Features
1. Read: [ROS_VERSION_ANALYSIS.md](ROS_VERSION_ANALYSIS.md) → "Recommendations" (5 min)
2. Read: [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md) → Relevant flow section (15 min)
3. Reference: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md) → "Dependency Graphs"

### Path 5: Quick Lookup
1. Go to: [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md)
2. Use tables for quick reference
3. Check "Direct Replacements" for ROS1→ROS2 mapping

## Key Statistics

| Metric | Value |
|--------|-------|
| Total Scripts | 30 |
| ROS2 Scripts | 12 (40%) |
| ROS1 Scripts | 18 (60%) |
| Total Libraries | 32 |
| ROS2 Libraries | 10 (31%) |
| ROS1 Libraries | 18 (56%) |
| Shared Libraries | 4 (13%) |
| Core ROS2 Nodes | 4 |
| Topics Published | 10+ |
| Services Provided | 6+ |
| Actions Provided | 2 |

## Visual Overview

```
┌─────────────────────────────────────────────────────────────┐
│              Topological Navigation System                   │
│                    (ROS2 Components)                         │
└─────────────────────────────────────────────────────────────┘

                    YAML Map File
                         │
                         ▼
                  map_manager2.py
                         │
                         ▼
                /topological_map_2
                         │
        ┌────────────────┼────────────────┐
        ▼                ▼                ▼
  localisation2    navigation2    get_simple_policy2
        │                │                │
        ▼                ▼                ▼
  /current_node    Action Server    Route Services
  /closest_node         │
                        ▼
                   Nav2 Stack
                        │
                        ▼
                  Robot Movement
```

## Contact & Contribution

This analysis was generated to help developers understand the topological_navigation codebase structure and identify which components are active in ROS2.

**For questions about:**
- **ROS1 vs ROS2 classification** → See [ROS_VERSION_ANALYSIS.md](ROS_VERSION_ANALYSIS.md)
- **How the system works** → See [ROS2_CALL_GRAPH.md](ROS2_CALL_GRAPH.md)
- **Quick lookups** → See [SCRIPT_CLASSIFICATION_SUMMARY.md](SCRIPT_CLASSIFICATION_SUMMARY.md)
- **Visual diagrams** → See [ROS2_ACTIVE_SCRIPTS_DIAGRAM.md](ROS2_ACTIVE_SCRIPTS_DIAGRAM.md)

## Updates

This analysis is based on:
- **Date:** February 10, 2026
- **Version:** topological_navigation v3.0.5
- **ROS2 Distribution:** Humble/Iron compatible
- **Branch:** Current working branch

If the codebase changes significantly, this analysis should be updated.

---

**Start with:** [ANALYSIS_SUMMARY.md](ANALYSIS_SUMMARY.md)
