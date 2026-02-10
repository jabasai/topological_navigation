# ROS2 Topological Navigation - Detailed Call Graph

## System Overview

This document provides a detailed call graph showing how ROS2 components interact in the topological_navigation system.

## 1. System Initialization Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    System Startup Sequence                       │
└─────────────────────────────────────────────────────────────────┘

1. map_manager2.py starts
   └─> Loads YAML map file
       └─> manager2.py.init_map()
           ├─> load_maps_from_yaml.load_map_from_yaml()
           ├─> Validates against schema
           └─> Publishes to /topological_map_2

2. localisation2.py starts
   └─> Waits for /topological_map_2
       └─> Subscribes to TF (map -> base_link)
           └─> Publishes localization topics

3. navigation2.py starts
   └─> Waits for /topological_map_2
       └─> Waits for localization topics
           └─> Initializes action servers
               └─> Ready for navigation goals

4. get_simple_policy2.py starts
   └─> Waits for /topological_map_2
       └─> Provides route planning services
```

## 2. Navigation Execution Flow

```
┌─────────────────────────────────────────────────────────────────┐
│              Navigation Goal Execution (GotoNode)                │
└─────────────────────────────────────────────────────────────────┘

User/Client sends GotoNode action goal
         │
         ▼
┌────────────────────────┐
│  navigation2.py        │
│  executeCallback()     │
└────────┬───────────────┘
         │
         ├─> 1. Validate goal
         │
         ├─> 2. Plan route
         │   └─> route_search2.py
         │       └─> TopologicalRouteSearch2.search_route()
         │           ├─> A* algorithm
         │           ├─> Check restrictions
         │           └─> Return ordered node/edge list
         │
         ├─> 3. Execute route
         │   └─> followRoute()
         │       │
         │       ├─> For each edge in route:
         │       │   │
         │       │   ├─> Get edge data from map
         │       │   │
         │       │   ├─> Check if reconfiguration needed
         │       │   │   └─> edge_reconfigure_manager2.py
         │       │   │       └─> param_processing.py
         │       │   │           └─> Update Nav2 parameters
         │       │   │
         │       │   ├─> Execute edge action
         │       │   │   └─> edge_action_manager2.py
         │       │   │       └─> execute_action()
         │       │   │           │
         │       │   │           ├─> Determine action type:
         │       │   │           │   ├─> NavigateToPose
         │       │   │           │   ├─> NavigateThroughPoses
         │       │   │           │   ├─> RowOperation
         │       │   │           │   └─> Custom actions
         │       │   │           │
         │       │   │           ├─> Build goal
         │       │   │           │   └─> goal_builder.py
         │       │   │           │       └─> build_nav_goal()
         │       │   │           │           ├─> Extract node pose
         │       │   │           │           ├─> Apply properties
         │       │   │           │           └─> Create Nav2 goal
         │       │   │           │
         │       │   │           ├─> Send to Nav2
         │       │   │           │   └─> Nav2 Action Client
         │       │   │           │       ├─> /navigate_to_pose
         │       │   │           │       ├─> /navigate_through_poses
         │       │   │           │       └─> /follow_waypoints
         │       │   │           │
         │       │   │           └─> Monitor execution
         │       │   │               ├─> Feedback callback
         │       │   │               ├─> Result callback
         │       │   │               └─> Handle failures
         │       │   │
         │       │   └─> Wait for edge completion
         │       │
         │       └─> Route complete
         │
         └─> 4. Return result
             └─> GotoNode.Result()
```

## 3. Localization Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Localization Process                          │
└─────────────────────────────────────────────────────────────────┘

┌────────────────────────┐
│  localisation2.py      │
│  pose_callback()       │  (Called at 20 Hz)
└────────┬───────────────┘
         │
         ├─> 1. Get TF transform (map -> base_link)
         │   └─> tf2_ros.Buffer.lookup_transform()
         │
         ├─> 2. Calculate distances to all nodes
         │   └─> get_distances_to_pose()
         │       └─> For each node:
         │           └─> get_distance_node_pose_from_tmap2()
         │               └─> Euclidean distance calculation
         │
         ├─> 3. Calculate distances to all edges
         │   └─> get_edge_distances_to_pose()
         │       └─> point2line.pnt2line()
         │           └─> Point-to-line segment distance
         │
         ├─> 4. Determine current node
         │   └─> Check if robot is within influence zone
         │       └─> point_in_poly()
         │           └─> Ray casting algorithm
         │
         ├─> 5. Determine closest node
         │   └─> Find nearest node (excluding no-go nodes)
         │
         └─> 6. Publish localization
             ├─> /current_node
             ├─> /closest_node
             ├─> /closest_edges
             └─> /current_node/tag
```

## 4. Route Planning Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Route Planning (A* Search)                    │
└─────────────────────────────────────────────────────────────────┘

Client calls GetRouteTo service
         │
         ▼
┌────────────────────────┐
│ get_simple_policy2.py  │
│ get_route_to_cb()      │
└────────┬───────────────┘
         │
         └─> route_search2.py
             └─> TopologicalRouteSearch2.search_route()
                 │
                 ├─> 1. Initialize
                 │   ├─> Create open list (priority queue)
                 │   ├─> Create closed list
                 │   └─> Add start node
                 │
                 ├─> 2. A* Loop
                 │   │
                 │   ├─> Pop node with lowest f-score
                 │   │
                 │   ├─> If goal reached: reconstruct path
                 │   │
                 │   ├─> For each neighbor:
                 │   │   │
                 │   │   ├─> Check if edge is blocked
                 │   │   │   └─> Call restrictions service
                 │   │   │
                 │   │   ├─> Calculate g-score
                 │   │   │   └─> get_edge_cost()
                 │   │   │       ├─> Base cost (distance)
                 │   │   │       └─> Property-based modifiers
                 │   │   │
                 │   │   ├─> Calculate h-score (heuristic)
                 │   │   │   └─> Euclidean distance to goal
                 │   │   │
                 │   │   ├─> Calculate f-score = g + h
                 │   │   │
                 │   │   └─> Add to open list if better path
                 │   │
                 │   └─> Repeat until goal found or no path
                 │
                 └─> 3. Return route
                     └─> NavRoute message
                         ├─> source[] (node names)
                         ├─> edge_id[] (edge IDs)
                         └─> cost (total path cost)
```

## 5. Edge Action Execution Flow

```
┌─────────────────────────────────────────────────────────────────┐
│              Edge Action Manager Execution                       │
└─────────────────────────────────────────────────────────────────┘

┌────────────────────────────┐
│ edge_action_manager2.py    │
│ execute_action()           │
└────────┬───────────────────┘
         │
         ├─> 1. Determine action type
         │   └─> edge["action"]
         │       ├─> "NavigateToPose"
         │       ├─> "NavigateThroughPoses"
         │       ├─> "RowOperation"
         │       └─> Custom actions
         │
         ├─> 2. Build navigation goal
         │   └─> goal_builder.py
         │       └─> build_nav_goal()
         │           │
         │           ├─> Extract target node
         │           │   └─> rsearch.get_node_from_tmap2()
         │           │
         │           ├─> Get node pose
         │           │   └─> node["node"]["pose"]
         │           │
         │           ├─> Apply edge properties
         │           │   └─> edge["properties"]
         │           │       ├─> xy_goal_tolerance
         │           │       ├─> yaw_goal_tolerance
         │           │       ├─> max_speed
         │           │       └─> Custom parameters
         │           │
         │           ├─> Select behavior tree
         │           │   └─> Based on action type
         │           │       ├─> bt_tree_default.xml
         │           │       ├─> bt_tree_in_row.xml
         │           │       └─> bt_tree_goal_align.xml
         │           │
         │           └─> Create Nav2 goal message
         │               └─> NavigateToPose.Goal()
         │
         ├─> 3. Handle special cases
         │   │
         │   ├─> Row Operations
         │   │   └─> row_operation_handler.py
         │   │       └─> handle_row_operation()
         │   │           ├─> Get boundary nodes
         │   │           ├─> Generate intermediate waypoints
         │   │           ├─> Apply row-specific parameters
         │   │           └─> Create NavigateThroughPoses goal
         │   │
         │   └─> Goal Alignment
         │       └─> Apply strict orientation tolerance
         │
         ├─> 4. Send goal to Nav2
         │   └─> Nav2 Action Client
         │       ├─> wait_for_server()
         │       ├─> send_goal_async()
         │       └─> Register callbacks
         │           ├─> feedback_callback()
         │           ├─> goal_response_callback()
         │           └─> result_callback()
         │
         ├─> 5. Monitor execution
         │   │
         │   ├─> Process feedback
         │   │   └─> Update navigation state
         │   │
         │   ├─> Check for cancellation
         │   │   └─> Cancel Nav2 goal if requested
         │   │
         │   └─> Handle failures
         │       └─> Retry logic or fail
         │
         └─> 6. Return result
             └─> Success/Failure status
```

## 6. Map Management Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Map Loading and Publishing                    │
└─────────────────────────────────────────────────────────────────┘

┌────────────────────────┐
│  map_manager2.py       │
│  main()                │
└────────┬───────────────┘
         │
         └─> manager2.py
             └─> map_manager_2.init_map()
                 │
                 ├─> 1. Load map from file
                 │   └─> load_maps_from_yaml.py
                 │       └─> load_map_from_yaml()
                 │           ├─> Open YAML file
                 │           ├─> Parse with CustomSafeLoader
                 │           │   └─> Ensure float types for poses
                 │           └─> Return map dictionary
                 │
                 ├─> 2. Validate map structure
                 │   └─> Check required fields
                 │       ├─> nodes[]
                 │       ├─> transformation
                 │       └─> pointset name
                 │
                 ├─> 3. Process map data
                 │   ├─> Build node index
                 │   ├─> Build edge index
                 │   └─> Calculate influence zones
                 │
                 ├─> 4. Publish map
                 │   └─> /topological_map_2
                 │       └─> String (YAML format)
                 │           └─> Latched topic
                 │
                 └─> 5. Provide services
                     ├─> /topological_map_manager2/get_node
                     ├─> /topological_map_manager2/get_edges
                     └─> /topological_map_manager2/update_node
```

## 7. Parameter Reconfiguration Flow

```
┌─────────────────────────────────────────────────────────────────┐
│              Edge Reconfiguration Process                        │
└─────────────────────────────────────────────────────────────────┘

Navigation needs to reconfigure Nav2 parameters
         │
         ▼
┌────────────────────────────────┐
│ edge_reconfigure_manager2.py   │
│ reconfigure_edge()             │
└────────┬───────────────────────┘
         │
         ├─> 1. Extract edge properties
         │   └─> edge["properties"]
         │       ├─> xy_goal_tolerance
         │       ├─> yaw_goal_tolerance
         │       ├─> max_speed
         │       └─> planner_id
         │
         ├─> 2. Map to Nav2 parameters
         │   └─> Create parameter dictionary
         │       ├─> "FollowPath.xy_goal_tolerance"
         │       ├─> "FollowPath.yaw_goal_tolerance"
         │       └─> "controller_server.max_vel_x"
         │
         └─> 3. Update parameters
             └─> param_processing.py
                 └─> ParameterUpdaterNode.set_params()
                     ├─> Create SetParameters request
                     ├─> Call Nav2 parameter service
                     └─> Wait for confirmation
```

## 8. Data Flow Summary

```
┌─────────────────────────────────────────────────────────────────┐
│                    Complete Data Flow                            │
└─────────────────────────────────────────────────────────────────┘

YAML Map File
     │
     ▼
map_manager2.py ──────> /topological_map_2 (String/YAML)
                              │
                              ├──────> localisation2.py
                              │            │
                              │            ├──> /current_node
                              │            ├──> /closest_node
                              │            └──> /closest_edges
                              │
                              ├──────> navigation2.py
                              │            │
                              │            ├──> route_search2.py
                              │            │        └──> Plan route
                              │            │
                              │            ├──> edge_action_manager2.py
                              │            │        │
                              │            │        ├──> goal_builder.py
                              │            │        │
                              │            │        └──> Nav2 Actions
                              │            │                 │
                              │            │                 ├──> /navigate_to_pose
                              │            │                 ├──> /navigate_through_poses
                              │            │                 └──> /follow_waypoints
                              │            │
                              │            └──> /topological_navigation/Route
                              │
                              └──────> get_simple_policy2.py
                                           └──> Route planning services

TF (map -> base_link) ──────> localisation2.py
                                    └──> Determine current/closest node

User/Client ──────> /topological_navigation (Action)
                         └──> navigation2.py
                                  └──> Execute navigation
```

## 9. Key Function Calls

### navigation2.py Key Methods
```python
executeCallback(goal)
    └─> navigate(target)
        └─> followRoute(route, target, exec_policy)
            ├─> execute_action(edge, node)
            │   └─> edge_action_manager.execute_action()
            └─> monitor_navigation()
```

### edge_action_manager2.py Key Methods
```python
execute_action(edge_data, target_node)
    ├─> determine_action_type()
    ├─> build_navigation_goal()
    │   └─> goal_builder.build_nav_goal()
    ├─> send_nav2_goal()
    └─> wait_for_result()
```

### route_search2.py Key Methods
```python
TopologicalRouteSearch2.search_route(origin, target)
    ├─> initialize_search()
    ├─> a_star_loop()
    │   ├─> get_neighbors()
    │   ├─> calculate_costs()
    │   └─> update_open_list()
    └─> reconstruct_path()
```

### localisation2.py Key Methods
```python
pose_callback()
    ├─> get_distances_to_pose()
    ├─> get_edge_distances_to_pose()
    ├─> determine_current_node()
    │   └─> point_in_poly()
    └─> publishTopics()
```

## 10. Thread and Callback Groups

```
navigation2.py uses MultiThreadedExecutor with callback groups:

├─> callback_group_map (ReentrantCallbackGroup)
│   └─> /topological_map_2 subscription
│
├─> callback_group_gotonode (ReentrantCallbackGroup)
│   └─> /topological_navigation action server
│
└─> callback_group_policy (ReentrantCallbackGroup)
    └─> /execute_policy_mode action server

This allows concurrent handling of:
- Map updates
- Multiple navigation goals
- Policy execution
```

## 11. Error Handling Flow

```
Navigation Failure
     │
     ├─> Edge action fails
     │   └─> edge_action_manager2.py
     │       ├─> Check retry count
     │       ├─> If retries available: retry
     │       └─> If no retries: return failure
     │
     ├─> Route blocked
     │   └─> navigation2.py
     │       ├─> Replan route
     │       │   └─> route_search2.py
     │       └─> Continue with new route
     │
     └─> Goal unreachable
         └─> Return failure to client
             └─> GotoNode.Result(success=False)
```

This call graph provides a comprehensive view of how the ROS2 topological navigation system operates, from initialization through navigation execution.
