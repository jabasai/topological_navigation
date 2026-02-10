# Topological Navigation Manager2 Unit Tests

## Overview

This directory contains comprehensive unit tests for the topological map manager2 module, which handles topological maps for robot navigation in ROS 2.

## Test Coverage

The test suite covers the following functionality:

### Core Functions
- **pose_dist**: Distance calculation between poses
- **map initialization**: Creating new maps and loading existing ones
- **map persistence**: Loading and writing YAML map files

### Node Operations
- Adding nodes to the topological map
- Removing nodes from the map
- Updating node names
- Updating node waypoint positions
- Setting node tolerance parameters
- Setting influence zones (node boundaries)

### Edge Operations
- Adding edges between nodes
- Removing edges
- Updating edge properties (action, action_type, goal)
- Updating edge restrictions (planning and runtime)
- Updating fail policies
- Setting fluid navigation behavior

### Map Operations
- Broadcasting TF transformations
- Adding GPS datum coordinates
- Clearing all nodes from map
- Batch operations (multiple nodes/edges at once)

### Service Callbacks
- Getting topological map data
- Switching between different maps
- All CRUD operations via service interfaces

## Running the Tests

### Prerequisites

Ensure you have the following installed:
- ROS 2 (Humble or later)
- Python 3.8+
- Required Python packages: `rclpy`, `pyyaml`, `unittest`

### Run All Tests

From the workspace root:

```bash
cd /home/ros/aoc_strawberry_scenario_ws
source install/setup.bash

# Run tests directly
python3 src/aoc_strawberry_scenario/contrib/topological_navigation/topological_navigation/test/test_manager2.py
```

### Run with ROS 2 Test Framework

If integrated with colcon:

```bash
cd /home/ros/aoc_strawberry_scenario_ws
colcon test --packages-select topological_navigation
colcon test-result --verbose
```

### Run Specific Test Classes

```bash
# Run only pose distance tests
python3 -m unittest test_manager2.TestPoseDistance

# Run only map manager tests
python3 -m unittest test_manager2.TestMapManager2

# Run specific test method
python3 -m unittest test_manager2.TestMapManager2.test_add_node
```

### Run with Verbose Output

```bash
python3 test_manager2.py -v
```

## Test Structure

### TestPoseDistance
Tests the utility function for calculating Euclidean distance between poses.

### TestMapManager2
Main test class covering:
- Initialization and configuration
- Map loading and saving
- Node and edge CRUD operations
- Transformation broadcasting
- Map validation

### TestServiceCallbacks
Tests ROS 2 service callback functions.

## Writing New Tests

When adding new functionality to manager2.py, follow these guidelines:

1. **Create a test method** in the appropriate test class:
   ```python
   def test_new_feature(self):
       """Test description"""
       manager = self.create_manager()
       # Test implementation
       self.assertTrue(result)
   ```

2. **Use setUp and tearDown** for test fixtures:
   - `setUp()`: Creates temporary directories and test data
   - `tearDown()`: Cleans up temporary files

3. **Mock external dependencies** when needed:
   ```python
   with patch('module.function') as mock_func:
       mock_func.return_value = expected_value
       # Test code
   ```

4. **Test both success and failure cases**:
   - Valid inputs should succeed
   - Invalid inputs should fail gracefully
   - Edge cases should be handled

## Test Data

Tests use temporary directories and dynamically generated YAML map files. The test setup creates:

- **Minimal test maps**: Simple maps with one or more nodes
- **Nav configuration**: Default navigation goal configurations
- **Temporary directories**: Automatically cleaned up after tests

## Continuous Integration

These tests can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run unit tests
  run: |
    source install/setup.bash
    python3 src/*/test/test_manager2.py
```

## Troubleshooting

### ROS 2 Not Initialized Error
Ensure `rclpy.init()` is called before creating nodes. The test suite handles this in `setUpClass`.

### Import Errors
Make sure the topological_navigation package is built and sourced:
```bash
colcon build --packages-select topological_navigation
source install/setup.bash
```

### File Permission Errors
Tests create temporary directories. Ensure write permissions in `/tmp`.

### Timeout Errors
Some tests may take longer on slower systems. Increase timeout values if needed.

## Code Coverage

To generate code coverage reports:

```bash
# Install coverage tool
pip3 install coverage

# Run tests with coverage
coverage run -m unittest test_manager2
coverage report
coverage html  # Generates HTML report in htmlcov/
```

## Contributing

When contributing tests:

1. Ensure all tests pass before submitting
2. Add tests for new features
3. Maintain test coverage above 80%
4. Follow existing naming conventions
5. Document complex test scenarios

## Additional Resources

- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [Python unittest documentation](https://docs.python.org/3/library/unittest.html)
- [Topological Navigation Wiki](https://github.com/LCAS/topological_navigation/wiki)

## License

This test suite is part of the topological_navigation package and follows the same license.
