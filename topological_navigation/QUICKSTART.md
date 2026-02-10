# Quick Start Guide - Topological Map Manager2

## 🚀 Getting Started in 5 Minutes

### Step 1: Test the Installation

```bash
# Show help to verify installation
ros2 run topological_navigation map_manager2.py --help
```

### Step 2: Run with Test Map

```bash
# Terminal 1: Start the manager with test map
ros2 run topological_navigation map_manager2.py --test

# Terminal 2: Check if it's running
ros2 service list | grep topological_map_manager2

# Terminal 3: Get the current map
ros2 service call /topological_map_manager2/get_topological_map std_srvs/srv/Trigger
```

### Step 3: Try Adding a Node

```bash
# Add a new node to the map
ros2 service call /topological_map_manager2/add_topological_node \
  topological_navigation_msgs/srv/AddNode \
  "{
    name: 'waypoint1',
    pose: {
      position: {x: 5.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }"
```

### Step 4: Save Your Map

```bash
# Save the modified map
ros2 service call /topological_map_manager2/write_topological_map \
  topological_navigation_msgs/srv/WriteTopologicalMap \
  "{filename: '/tmp/my_new_map.yaml'}"
```

---

## 🧪 Run Unit Tests

```bash
cd /home/ros/aoc_strawberry_scenario_ws
source install/setup.bash

# Run all tests
python3 src/aoc_strawberry_scenario/contrib/topological_navigation/topological_navigation/test/test_manager2.py

# Or run with unittest
python3 -m unittest discover -s src/aoc_strawberry_scenario/contrib/topological_navigation/topological_navigation/test -p "test_*.py" -v
```

---

## 📚 Common Use Cases

### Use Case 1: Load Your Own Map

```bash
ros2 run topological_navigation map_manager2.py /path/to/your/map.yaml
```

### Use Case 2: Create a New Map from Scratch

```bash
# Start with empty map
ros2 run topological_navigation map_manager2.py -n my_new_map.yaml

# Add nodes via services (see examples above)
# Save when done
```

### Use Case 3: Debug Mode

```bash
# Run with verbose logging
ros2 run topological_navigation map_manager2.py -v --test
```

### Use Case 4: Programmatic Usage in Python

```python
#!/usr/bin/env python3
import rclpy
from topological_navigation.manager2 import map_manager_2

def main():
    rclpy.init()
    
    # Create manager
    manager = map_manager_2(advertise_srvs=True)
    
    # Load existing map or create new one
    manager.init_map(filepath='/path/to/map.yaml', load=True)
    
    # Add a node
    pose = {
        'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }
    success = manager.add_topological_node('node1', pose, add_close_nodes=False)
    
    if success:
        print("Node added successfully!")
        manager.write_topological_map('/path/to/output.yaml')
    
    # Keep node running
    rclpy.spin(manager)
    
    # Cleanup
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔧 Available Services

Once the manager is running, these services are available:

### Query Services:
- `get_topological_map` - Get the entire map as JSON
- `get_tagged_nodes` - Get nodes with specific tags
- `get_tags` - List all available tags
- `get_node_tags` - Get tags for a specific node
- `get_edges_between_nodes` - Get edges between two nodes

### Node Operations:
- `add_topological_node` - Add a new node
- `remove_topological_node` - Remove a node
- `update_node_name` - Rename a node
- `update_node_pose` - Update node position
- `update_node_tolerance` - Update goal tolerances
- `set_node_influence_zone` - Set node boundary vertices

### Edge Operations:
- `add_edges_between_nodes` - Add edge between nodes
- `remove_edge` - Remove an edge
- `update_edge` - Update edge properties
- `update_action` - Update action for all edges
- `update_edge_restrictions` - Update edge restrictions

### Map Operations:
- `write_topological_map` - Save map to file
- `switch_topological_map` - Switch to different map
- `clear_topological_nodes` - Remove all nodes
- `add_datum` - Set GPS coordinates

### Batch Operations:
- `add_topological_node_multi` - Add multiple nodes
- `add_edges_between_nodes_multi` - Add multiple edges
- `add_param_to_edge_config_multi` - Configure multiple edges
- `set_node_influence_zone_multi` - Set multiple influence zones

---

## 🐛 Troubleshooting

### Issue: "No module named 'topological_navigation'"

**Solution:**
```bash
cd /home/ros/aoc_strawberry_scenario_ws
colcon build --packages-select topological_navigation
source install/setup.bash
```

### Issue: "File not found" when loading map

**Solution:**
- Check the file path is correct
- Use absolute paths, not relative
- Verify file has `.yaml` or `.yml` extension

### Issue: Service calls fail

**Solution:**
1. Check manager is running: `ros2 node list | grep topological`
2. Verify services: `ros2 service list | grep manager2`
3. Check message format: `ros2 interface show <srv_type>`

### Issue: Tests fail

**Solution:**
```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check dependencies
pip3 install pyyaml

# Run with verbose output
python3 test/test_manager2.py -v
```

---

## 📖 More Information

- **Full Help**: `ros2 run topological_navigation map_manager2.py --help`
- **Usage Examples**: `python3 examples/manager2_usage_examples.py`
- **Test Documentation**: `cat test/README.md`
- **Refactoring Summary**: `cat REFACTORING_SUMMARY.md`

---

## 💡 Tips

1. **Always use absolute paths** for map files
2. **Test with the default map** first (`--test` flag)
3. **Use verbose mode** (`-v`) when debugging
4. **Save your map frequently** via write_topological_map service
5. **Validate your map** - the manager validates against the schema automatically
6. **Check logs** for detailed error messages
7. **Use tab completion** in terminals for service names

---

## ✅ Checklist

Before using in production:

- [ ] Tested with your map file
- [ ] Verified all required nodes are present
- [ ] Checked edge connections are correct
- [ ] Validated transformation parameters
- [ ] Tested service calls work as expected
- [ ] Saved a backup of your map file
- [ ] Ran unit tests successfully

---

## 🎯 Next Steps

1. **Create your own map** using the manager
2. **Run the unit tests** to ensure everything works
3. **Explore the examples** to learn more features
4. **Integrate with your navigation stack**
5. **Read the full documentation** for advanced features

---

## 📞 Support

For issues or questions:
- Check the test documentation: `test/README.md`
- Review examples: `examples/manager2_usage_examples.py`
- Read the code: `manager2.py` has comprehensive docstrings
- Check ROS 2 logs: `ros2 run topological_navigation map_manager2.py -v`

**Happy Mapping! 🗺️**
