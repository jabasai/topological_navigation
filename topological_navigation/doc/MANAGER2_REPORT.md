# Functionality and Complexity Report: `manager2.py`

**File Location:** `topological_navigation/topological_navigation/manager2.py`
**Class:** `map_manager_2`

## 1. Overview
The `manager2.py` file defines the `map_manager_2` class, which is a ROS 2 node responsible for managing topological maps. It acts as the central server for map data, providing services to query, modify, and persist the map.

## 2. Usages
This class is a core component used in:

1.  **Main Execution Node**:
    -   `topological_navigation/scripts/map_manager2.py`: Instantiates `map_manager_2` to run the Map Manager ROS node.

2.  **Utilities**:

## 3. Functionality Analysis

The `map_manager_2` class performs the following key functions:

### 3.1 Map Lifecycle Management
-   **Loading**: Loads `.tmap2` (YAML) files using a separate process (`multiprocessing`) to avoid blocking the main thread during parsing.
-   **Storage**: Maintains the map in memory as a nested dictionary structure (`self.tmap2`).
-   **Persistence**: Writes the current map state back to YAML format upon request or automatic updates (`write_topological_map`), optionally handling YAML aliases.

### 3.2 ROS Interface
-   **Publishing**: Publishes the full map JSON to `/topological_map_2` and broadcasts TF transforms for the map frame.
-   **Services**: Exposes roughly 25+ ROS services for external interaction, categorized into:
    -   **Queries**: `get_topological_map`, `get_tagged_nodes`, `get_tags`, `get_edges_between_nodes`, etc.
    -   **Modifications**: `add_topological_node`, `remove_topological_node`, `add_edges_between_nodes`, `update_node_pose`, `update_edge`, etc.
    -   **Batch Operations**: `add_topological_node_multi`, `add_edges_between_nodes_multi`.

### 3.3 Core Logic
-   **Validation**: Implements `map_check` to verify consistency (duplicate names, invalid edge destinations, multiple pointsets).
-   **Geometry**: Generates default node footprints (`generate_circle_vertices`) and calculates distances.
-   **Synchronization**: The `update()` method triggers a map publish and optional write-to-disk, ensuring subscribers receive the latest state.

## 4. Complexity Report

### 4.1 Metrics
-   **Lines of Code**: ~1400 lines.
-   **Methods**: Very high number of methods (~50), largely due to the 1-to-1 mapping between ROS services and callback handlers.
-   **Cognitive Load**: High. The class mixes ROS communication logic, file I/O, data validation, and geometric helper functions.

### 4.2 Structural Issues
-   **State Management**: The map state is a raw dictionary (`self.tmap2`). Operations on this dictionary are spread throughout the class, making it hard to track where and how state changes.
-   **Concurrency**: Uses `multiprocessing` for loading but otherwise runs deeply synchronous logic in service callbacks. If `write_topological_map` is slow (disk I/O), it blocks the ROS executor for that callback.
-   **Coupling**: Tightly coupled to `topological_navigation_msgs` and `tf2_ros`.

## 5. Suggestions for Improvement

1.  **Separation of Concerns (Refactoring)**:
    -   Extract the map data model into a standalone class (e.g., `TopologicalMapModel`). This class should handle the dictionary manipulation, validation, and file I/O.
    -   The `map_manager_2` ROS node should only act as an interface layer (Controller) that translates ROS service requests into method calls on the Model.

2.  **Data Validation**:
    -   Replace manual `map_check` logic with a robust schema validator (e.g., `jsonschema`), leveraging the `tmap-schema.yaml` already present in the config.

3.  **Modern Python Practices**:
    -   Add **Type Hinting** to all methods to improve code clarity and enable static analysis.
    -   Use `pathlib` for file path manipulations instead of `os.path`.

4.  **Error Handling**:
    -   Replace checks like `if num_available == 1` with proper exception handling. Using exceptions for "node not found" would be cleaner than returning boolean/string tuples.

5.  **Testing**:
    -   The extraction of the Map Model (Suggestion 1) would allow for unit testing the map logic without needing a running ROS system. Currently, testing requires spinning up the node.

6.  **Async/Await**:
    -   For file I/O operations (like writing the map), consider using asynchronous calls or offloading to a worker thread to prevent blocking the ROS event loop, although `multiprocessing` is currently used for loading.

## 6. Refactoring Implementation Status (February 2026)

The suggestions outlined above have been implemented to modernize the codebase and improve maintainability.

### 6.1 Architecture Changes
-   **MVC Pattern Implemented**: The monolithic `manager2.py` has been split into:
    -   **Model (`tmap_model.py`)**: A pure Python class handling topological map data, schema validation, persistence, and logical operations. It has no ROS dependencies.
    -   **Controller (`manager2.py`)**: A streamlined ROS 2 node that wraps the Model. It handles services, publishers, and parameters, delegating all business logic to the Model.
-   **Legacy Preservation**: The original implementation was renamed to `manager2_legacy.py` to ensure a safe fallback if needed.

### 6.2 Key Improvements
-   **Schema Validation**: The `TopologicalMapModel` now enforces the structure defined in `config/tmap-schema.yaml` using the `jsonschema` library. This ensures map integrity upon loading and modification.
-   **Unit Testing**: A new test suite `test/test_tmap_model.py` was created to verify the Model logic isolated from ROS.
    -   Tests cover: Initialization, Node/Edge addition, Duplicate handling, Validation scenarios, and File I/O.
    -   Status: **Passing** (as of Feb 5, 2026).
-   **Exception Handling**: The Model uses custom exceptions (`MapValidationError`, `NodeNotFoundError`, `DuplicateError`) which the Controller catches to return appropriate ROS service responses.

### 6.3 Future Work
-   **Type Hinting**: While the structure is improved, full type checking (mypy) could be added to the new Model class.
-   **Async I/O**: File writing is still synchronous in the Model; offloading this in the Controller for large maps remains a potential optimization.
