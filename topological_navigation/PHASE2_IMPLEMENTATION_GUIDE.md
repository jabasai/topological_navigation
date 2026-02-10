# Phase 2 Implementation Guide

**Status**: Ready to Start  
**Branch**: aoc_refactor  
**PR**: #242  
**Estimated Duration**: 1-2 weeks

---

## Phase 2 Overview

**Goal**: Refactor EdgeActionManager2 (1365 lines) into modular components using new infrastructure from Phase 1.

**Expected Outcome**: 
- EdgeActionManager2 reduced to ~300-400 lines
- 3 new focused classes with single responsibilities
- Improved testability
- Better code reusability

---

## Task 1: Create GoalBuilder Class

**File**: `topological_navigation/goal_builder.py`  
**Estimated Size**: 250-300 lines  
**Dependencies**: map_types.py, tmap_utils.py

### Responsibilities
- Build Nav2 action goals from topological edges
- Substitute node properties into goal templates
- Handle action-type-specific goal construction

### Key Methods

```python
class GoalBuilder:
    """
    Builds Nav2 action goals from topological map edges.
    Handles goal construction for different action types.
    """
    
    def __init__(self, node):
        """Initialize with ROS 2 node for logging"""
        self.node = node
        self.logger = node.get_logger()
    
    def build_navigate_to_pose_goal(
        self, 
        edge: TopologicalEdge,
        destination_node: TopologicalNode,
        origin_node: TopologicalNode
    ) -> NavigateToPose.Goal:
        """
        Build NavigateToPose goal for single waypoint navigation.
        
        Args:
            edge: Edge being traversed
            destination_node: Target node
            origin_node: Source node
            
        Returns:
            NavigateToPose.Goal with proper parameters
        """
        # Extract properties from nodes
        # Create PoseStamped for destination
        # Apply behavioral parameters (tolerances, etc.)
        # Return goal
    
    def build_navigate_through_poses_goal(
        self,
        edges: List[TopologicalEdge],
        destination_nodes: List[TopologicalNode],
        origin_nodes: List[TopologicalNode]
    ) -> NavigateThroughPoses.Goal:
        """
        Build NavigateThroughPoses goal for multi-waypoint path.
        
        Args:
            edges: List of edges to traverse
            destination_nodes: Target nodes in order
            origin_nodes: Source nodes in order
            
        Returns:
            NavigateThroughPoses.Goal with waypoint list
        """
        # Segment edges by action type
        # Create PoseStamped list for each segment
        # Apply control parameters
        # Return goal
    
    def substitute_properties(
        self,
        goal: Any,
        source_node: TopologicalNode,
        dest_node: TopologicalNode,
        edge: TopologicalEdge
    ) -> Any:
        """
        Substitute node/edge properties into goal.
        
        Properties applied in priority order:
        1. Edge properties (highest priority)
        2. Destination node properties
        3. Source node properties
        4. Default values (lowest priority)
        
        Args:
            goal: Goal message to modify
            source_node: Origin node
            dest_node: Destination node
            edge: Edge being traversed
            
        Returns:
            Modified goal with properties applied
        """
        # Apply xy_goal_tolerance
        # Apply yaw_goal_tolerance
        # Apply max_speed
        # Apply custom properties
```

### Testing Strategy
- Unit tests for each goal type
- Property substitution tests
- Validation of goal message structure
- Integration test with real maps

### Extract From EdgeActionManager2
Lines ~500-880 contain goal building logic:
- `construct_navigate_to_pose_goal()`
- `get_navigate_to_pose_goal()`
- `construct_navigate_through_poses_goal()`
- `get_navigate_through_poses_goal()`
- Property substitution code

---

## Task 2: Create RowOperationHandler Class

**File**: `topological_navigation/row_operation_handler.py`  
**Estimated Size**: 200-250 lines  
**Dependencies**: map_types.py, tmap_utils.py, goal_builder.py

### Responsibilities
- Handle agricultural row navigation
- Determine boundary nodes (entry/exit points)
- Build intermediate poses for row traversal
- Create row operation action messages

### Key Methods

```python
class RowOperationHandler:
    """
    Handles agricultural row operation navigation.
    Manages boundary detection and intermediate waypoint generation.
    """
    
    def __init__(self, node, goal_builder: GoalBuilder):
        """
        Initialize row operation handler.
        
        Args:
            node: ROS 2 node for logging
            goal_builder: GoalBuilder instance for goal construction
        """
        self.node = node
        self.goal_builder = goal_builder
        self.logger = node.get_logger()
    
    def handle_row_operation(
        self,
        edge: TopologicalEdge,
        topological_map: TopologicalMap,
        origin_node: TopologicalNode,
        destination_node: TopologicalNode
    ) -> NavigateThroughPoses.Goal:
        """
        Handle row operation navigation.
        
        Args:
            edge: Row operation edge (e.g., "RowEntry_A1 -> RowEnd_A1")
            topological_map: Full topological map
            origin_node: Starting node
            destination_node: Ending node
            
        Returns:
            NavigateThroughPoses goal with intermediate poses
        """
        # Parse row information from edge
        # Find row center node
        # Collect boundary candidates
        # Select optimal boundary nodes
        # Build intermediate poses
        # Create NavigateThroughPoses goal
    
    def get_boundary_nodes(
        self,
        center_node: TopologicalNode,
        candidate_nodes: List[TopologicalNode],
        row_direction: str = "forward"
    ) -> tuple:
        """
        Select optimal boundary nodes for row entry/exit.
        
        Returns 2 entry and 2 exit nodes from candidates.
        Uses side_edges property to identify parallel paths.
        
        Args:
            center_node: Center node of the row
            candidate_nodes: List of potential boundary nodes
            row_direction: "forward" or "backward"
            
        Returns:
            (entry_nodes, exit_nodes) - tuples of 2 nodes each
        """
        # Filter candidates by side_edges property
        # Calculate perpendicular distance from center
        # Sort by distance
        # Return 2 closest on each side
    
    def build_intermediate_poses(
        self,
        entry_node: TopologicalNode,
        exit_node: TopologicalNode,
        step_size: float = 2.0,
        interpolation_method: str = "linear"
    ) -> List[PoseStamped]:
        """
        Build intermediate waypoints for row traversal.
        
        Args:
            entry_node: Row entry node
            exit_node: Row exit node
            step_size: Distance between waypoints (meters)
            interpolation_method: "linear" or "spline"
            
        Returns:
            List of PoseStamped for intermediate poses
        """
        # Get entry and exit poses
        # Interpolate between them
        # Create waypoints at step_size intervals
        # Apply orientation for row alignment
    
    def collect_boundary_candidates(
        self,
        center_node: TopologicalNode,
        edge_id: str,
        topological_map: TopologicalMap
    ) -> List[TopologicalNode]:
        """
        Collect nodes that could serve as boundaries.
        
        Looks for nodes with side_edges property that match the row.
        
        Args:
            center_node: Center node of the row
            edge_id: Edge ID to match
            topological_map: Full topological map
            
        Returns:
            List of candidate boundary nodes
        """
        # Find all nodes with side_edges property
        # Filter for matching row
        # Return sorted by distance from center
```

### Testing Strategy
- Unit tests for boundary node selection
- Unit tests for intermediate pose generation
- Integration tests with agricultural maps
- Field testing with row operation scenarios

### Extract From EdgeActionManager2
Lines ~730-1200 contain row operation logic:
- `_handle_row_operation()`
- `_get_row_center_node()`
- `_collect_boundary_candidates()`
- `_select_boundary_nodes()`
- `get_intermediate_poses_interpolated()`
- `execute_row_operation_action()`

---

## Task 3: Refactor EdgeActionManager2

**File**: `topological_navigation/edge_action_manager2.py`  
**Target Size**: 300-400 lines (from 1365)  
**Dependencies**: nav_action_client_manager.py, goal_builder.py, row_operation_handler.py

### Refactored Architecture

```
EdgeActionManager2
├── Compose NavActionClientManager
│   └── Handles all Nav2 ActionClient lifecycle
├── Compose GoalBuilder
│   └── Handles all goal construction
├── Compose RowOperationHandler
│   └── Handles row operations
└── Core methods:
    ├── initialise() - Setup for edge execution
    ├── execute() - Execute the prepared edge
    ├── preempt() - Cancel execution
    └── get_result() - Get execution result
```

### Refactoring Steps

#### Step 1: Remove Duplicated NavActionClientManager Code
- Delete: `set_nav_client()`, `init_nav_client()`, server checking code
- Replace with: `self.client_manager.create_client(action_type)`

#### Step 2: Remove GoalBuilder Code
- Delete: `construct_navigate_to_pose_goal()`, `get_navigate_to_pose_goal()`, etc.
- Replace with: `self.goal_builder.build_navigate_to_pose_goal(edge, dest, src)`

#### Step 3: Remove RowOperationHandler Code
- Delete: `_handle_row_operation()`, `_get_row_center_node()`, etc.
- Replace with: `self.row_handler.handle_row_operation(edge, tmap, src, dest)`

#### Step 4: Simplify execute() Method
- Before: 60+ lines with nested action client code
- After: 15-20 lines orchestrating component calls

**Before**:
```python
def execute(self):
    # 60 lines of action client management
    # 20 lines of goal construction
    # 40 lines of status checking
    # Total: 120 lines
```

**After**:
```python
def execute(self):
    """Execute prepared edge action"""
    # Setup (2 lines)
    if not self.client_manager.wait_for_server(self.action_type):
        return GoalStatus.STATUS_FAILED
    
    # Execute (3 lines)
    if not self.client_manager.send_goal(self.action_type, self.goal_msg):
        return GoalStatus.STATUS_FAILED
    
    # Wait and return (3 lines)
    status, _ = self.client_manager.wait_for_result()
    return status
```

### Key Simplifications

#### Remove 300+ lines of boilerplate
```python
# OLD: Complex callback setup
def _internal_feedback_callback(self, feedback_msg):
    self.goal_status = GoalStatus.STATUS_EXECUTING
    # ... update state

def _internal_result_callback(self, future):
    # ... extract result, update state, signal complete

# NEW: Delegated to NavActionClientManager
# No change needed - handled by composed class
```

#### Consolidate action type handling
```python
# OLD: Separate methods for each action type
if self.action_name == "NavigateToPose":
    goal = self.construct_navigate_to_pose_goal()
elif self.action_name == "NavigateThroughPoses":
    goal = self.construct_navigate_through_poses_goal()
elif self.action_name == "RowOperation":
    goal = self._handle_row_operation()
# ... more conditions

# NEW: Delegated to specialized builders
if self.action_name == "RowOperation":
    goal = self.row_handler.handle_row_operation(...)
else:
    goal = self.goal_builder.build_goal(...)
```

### Testing Strategy

```python
# Unit tests
test_edge_action_manager2.py
├── TestInitialize
│   ├── test_initialise_navigate_to_pose
│   ├── test_initialise_navigate_through_poses
│   └── test_initialise_row_operation
├── TestExecute
│   ├── test_execute_success
│   ├── test_execute_failure
│   └── test_execute_timeout
└── TestResult
    ├── test_get_result_succeeded
    ├── test_get_result_failed
    └── test_get_result_cancelled

# Integration tests
test_edge_action_integration.py
├── TestNavigationFlow
│   ├── test_single_edge_traversal
│   ├── test_multi_edge_route
│   └── test_edge_failure_recovery
├── TestRowOperationFlow
│   ├── test_row_entry_exit
│   └── test_row_with_obstacles
└── TestPropertySubstitution
    ├── test_property_from_edge
    ├── test_property_from_node
    └── test_property_precedence
```

### Code Review Checklist

- [ ] All methods from old EdgeActionManager2 are either removed, refactored, or delegated
- [ ] No code duplication between classes
- [ ] Each class has single responsibility
- [ ] Public API remains backward compatible
- [ ] All unit tests pass
- [ ] Integration tests pass
- [ ] Code follows PEP 8
- [ ] Docstrings updated
- [ ] Type hints added where applicable

---

## Implementation Order

### Week 1
1. **Monday-Tuesday**: Create GoalBuilder class
   - Extract goal building logic
   - Write unit tests
   - Verify all goal types work

2. **Wednesday-Thursday**: Create RowOperationHandler class
   - Extract row operation logic
   - Write unit tests
   - Test with sample maps

3. **Friday**: Code review and refinement
   - Peer review GoalBuilder
   - Peer review RowOperationHandler
   - Fix issues found

### Week 2
1. **Monday-Tuesday**: Refactor EdgeActionManager2
   - Remove extracted code
   - Integrate new classes
   - Update methods to use new classes

2. **Wednesday**: Testing
   - Run full test suite
   - Integration testing
   - Performance validation

3. **Thursday-Friday**: Documentation and Polish
   - Update docstrings
   - Add examples
   - Create migration guide
   - Prepare for merge

---

## Files to Create/Modify

### New Files
- `goal_builder.py` - 250-300 lines
- `row_operation_handler.py` - 200-250 lines
- `test_goal_builder.py` - 200+ lines
- `test_row_operation_handler.py` - 200+ lines
- `test_edge_action_manager2_refactored.py` - 300+ lines

### Modified Files
- `edge_action_manager2.py` - Remove 900+ lines, integrate new classes
- `__init__.py` - Export new classes

### Documentation
- Update docstrings in refactored classes
- Add usage examples to README
- Update PROPERTIES.md if needed

---

## Success Criteria

✅ **Code Quality**
- No code duplication
- Each class has single responsibility  
- Cyclomatic complexity < 10 for any method
- All methods < 50 lines

✅ **Testing**
- 90%+ code coverage
- All existing tests pass
- New unit tests for all public methods
- Integration tests for navigation flows

✅ **Performance**
- No regression in action execution time
- No memory leaks in long-term operation
- Same latency as original implementation

✅ **Backward Compatibility**
- Public API unchanged
- All existing map files compatible
- Gradual migration path for internal users

✅ **Documentation**
- All public methods documented
- Usage examples provided
- Migration guide for internal users
- Architecture diagrams updated

---

## Additional Resources

### Reference Implementation
See Phase 1 implementations:
- `map_types.py` - 270 lines, typed data structures
- `nav_action_client_manager.py` - 319 lines, separated concerns

### Related Code
- `edge_action_manager2.py` - Current implementation (1365 lines)
- `ROS2_DEEP_ANALYSIS.md` - Detailed function analysis
- `REFACTOR_SUMMARY.md` - Phase 1 summary

### Testing Tools
- pytest - Unit testing
- launch_pytest - Integration testing  
- colcon test - Full test suite
- coverage - Code coverage analysis

---

## Questions/Clarifications

Before starting Phase 2, confirm:
1. ✅ Are all Phase 1 changes merged?
2. ✅ Should backward compatibility be maintained? (Answer: Yes, for 1 release)
3. ✅ Any specific performance requirements? (Answer: No regression)
4. ✅ Should ROS 1 compatibility be considered? (Answer: No, ROS 2 only)

---

**Status**: ✅ Ready to Start Phase 2  
**Estimated Effort**: 10-15 hours  
**Target Completion**: Next 1-2 weeks  
**Approval**: Waiting for confirmation to proceed
