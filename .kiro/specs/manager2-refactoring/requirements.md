# Requirements: Self-Contained Map Manager Refactoring

## Overview
Refactor the `manager2.py` script to be a self-contained, standalone ROS 2 node that embeds all dependencies (schema validation, map model) and publishes the schema as a latched topic.

## User Stories

### 1. Self-Contained Script
**As a** developer  
**I want** the map manager to be self-contained without external dependencies  
**So that** I can understand and maintain the code in a single file

**Acceptance Criteria:**
- 1.1 The script embeds the YAML schema definition inline
- 1.2 The script includes all map validation logic (no dependency on `tmap_model.py`)
- 1.3 The script includes all YAML loading logic (no dependency on `map_types.py`)
- 1.4 The script can run standalone with only standard ROS 2 and Python dependencies
- 1.5 All helper functions are defined within the script

### 2. Schema Validation
**As a** map manager  
**I want** to validate topological maps against the embedded schema  
**So that** I can ensure map data integrity

**Acceptance Criteria:**
- 2.1 Schema validation occurs when loading a map file
- 2.2 Schema validation errors provide clear, actionable messages
- 2.3 Validation checks for duplicate node names
- 2.4 Validation checks for edges pointing to non-existent nodes
- 2.5 Validation checks for consistent pointset across all nodes

### 3. Schema Publication
**As a** ROS 2 system  
**I want** the schema published as a latched topic  
**So that** other nodes can validate maps independently

**Acceptance Criteria:**
- 3.1 Schema is published to `/topological_map_schema` topic as `std_msgs/String`
- 3.2 Topic uses TRANSIENT_LOCAL durability (latched behavior)
- 3.3 Schema is published once during node initialization
- 3.4 Schema is published as JSON string format

### 4. Core Services
**As a** ROS 2 client  
**I want** essential map management services  
**So that** I can interact with the topological map

**Acceptance Criteria:**
- 4.1 Service `/topological_map_manager2/get_topological_map` returns current map
- 4.2 Service `/topological_map_manager2/write_topological_map` saves map to file
- 4.3 Service `/topological_map_manager2/switch_topological_map` loads a different map
- 4.4 All services return success/failure status with descriptive messages

### 5. Map Publication
**As a** navigation system  
**I want** the topological map published to a topic  
**So that** I can access map data for navigation

**Acceptance Criteria:**
- 5.1 Map is published to `/topological_map_2` as `std_msgs/String` (JSON format)
- 5.2 Topic uses TRANSIENT_LOCAL durability for late subscribers
- 5.3 Map is published after loading and after any modifications
- 5.4 Published map includes all nodes, edges, and metadata

### 6. Test Coverage
**As a** developer  
**I want** comprehensive tests for the map manager  
**So that** I can verify functionality and prevent regressions

**Acceptance Criteria:**
- 6.1 Test loads the test map file successfully
- 6.2 Test validates map against embedded schema
- 6.3 Test calls `get_topological_map` service and verifies response
- 6.4 Test calls `write_topological_map` service and verifies file creation
- 6.5 Test calls `switch_topological_map` service and verifies map change
- 6.6 Test verifies schema publication on `/topological_map_schema` topic
- 6.7 Test verifies map publication on `/topological_map_2` topic
- 6.8 Test validates schema validation catches invalid maps

## Non-Functional Requirements

### Performance
- Map loading should complete within 5 seconds for maps with up to 1000 nodes
- Schema validation should complete within 1 second

### Maintainability
- Code should be well-documented with docstrings
- Complex logic should have inline comments
- Follow ROS 2 and Python best practices

### Compatibility
- Must work with ROS 2 Humble and Iron
- Must use Python 3.8+
- Must follow existing ROS 2 message/service interfaces

## Out of Scope
- Advanced map editing operations (add/remove nodes/edges)
- Interactive map editor integration
- Map caching functionality
- Auto-write functionality
- Complex TF broadcasting (keep simple if needed)

## Dependencies
- ROS 2 (rclpy)
- PyYAML
- jsonschema
- std_msgs, std_srvs
- topological_navigation_msgs

## Success Metrics
- Script is under 800 lines of code
- All tests pass
- No external Python module dependencies (except standard libraries and ROS 2)
- Schema and map topics are published correctly
- All three core services work as expected
