# Parameter Server Usage for topological_map_manager_2

## Overview

The `topological_map_manager_2` node now implements a proper ROS 2 parameter server pattern with:
- All parameters declared in `__init__` 
- Dynamic parameter updates via callback
- Parameter validation and logging

## Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cache_topological_maps` | bool | `false` | Cache topological maps to `~/.ros/topological_maps` |
| `auto_write_topological_maps` | bool | `false` | Automatically write map changes to disk |
| `nav_config` | string | `{pkg}/config/navigation_goal.yaml` | Path to navigation goal configuration |
| `topological_map2_name` | string | `""` | Name of the loaded topological map |
| `topological_map2_filename` | string | `""` | Filename of the loaded map |
| `topological_map2_path` | string | `""` | Path to the loaded map file |

## Usage

### 1. Launch with Parameter File

```bash
ros2 run topological_navigation topological_map_manager_2 --ros-args --params-file /path/to/manager2_params.yaml
```

### 2. Launch with Individual Parameters

```bash
ros2 run topological_navigation topological_map_manager_2 \
    --ros-args \
    -p cache_topological_maps:=true \
    -p auto_write_topological_maps:=true
```

### 3. Update Parameters at Runtime

Using command line:
```bash
ros2 param set /topological_map_manager_2 cache_topological_maps true
ros2 param set /topological_map_manager_2 auto_write_topological_maps false
```

Using Python:
```python
from rclpy.parameter import Parameter

# Set a single parameter
node.set_parameters([Parameter('cache_topological_maps', Parameter.Type.BOOL, True)])

# Set multiple parameters
node.set_parameters([
    Parameter('cache_topological_maps', Parameter.Type.BOOL, True),
    Parameter('auto_write_topological_maps', Parameter.Type.BOOL, False)
])
```

### 4. Get Parameters

```bash
# List all parameters
ros2 param list /topological_map_manager_2

# Get specific parameter
ros2 param get /topological_map_manager_2 cache_topological_maps

# Dump all parameters
ros2 param dump /topological_map_manager_2
```

## Parameter Callback Behavior

When a parameter is updated, the `parameters_callback` method:
1. Validates the new parameter value
2. Updates the corresponding instance variable
3. Logs the change
4. Performs any necessary reconfiguration (e.g., reloading nav_config)
5. Returns a `SetParametersResult` indicating success/failure

### Special Cases

- **`nav_config`**: When updated, the node automatically reloads the navigation goal configuration from the new file
- **`topological_map2_*`**: These are informational parameters set by the node when maps are loaded

## Example Parameter File

See [config/manager2_params.yaml](../config/manager2_params.yaml) for a template.

```yaml
topological_map_manager_2:
  ros__parameters:
    cache_topological_maps: false
    auto_write_topological_maps: false
    nav_config: ""
    topological_map2_name: ""
    topological_map2_filename: ""
    topological_map2_path: ""
```

## Integration in Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('topological_navigation'),
        'config',
        'manager2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='topological_navigation',
            executable='topological_map_manager_2',
            name='topological_map_manager_2',
            parameters=[config]
        )
    ])
```

## Implementation Details

The parameter server pattern is implemented with:

```python
def __init__(self):
    # 1. Declare all parameters with defaults
    self.declare_parameter('cache_topological_maps', False)
    self.declare_parameter('auto_write_topological_maps', False)
    # ... etc
    
    # 2. Register callback
    self.add_on_set_parameters_callback(self.parameters_callback)
    
    # 3. Get initial values
    self.cache_maps = self.get_parameter('cache_topological_maps').value
    # ... etc

def parameters_callback(self, params):
    """Handle parameter updates"""
    from rcl_interfaces.msg import SetParametersResult
    
    for param in params:
        if param.name == 'cache_topological_maps':
            self.cache_maps = param.value
            self.get_logger().info(f'Parameter updated to: {param.value}')
        # ... handle other parameters
    
    return SetParametersResult(successful=True)
```

This pattern ensures:
- ✅ All parameters are declared at node initialization
- ✅ Parameters can be set via launch files, CLI, or programmatically  
- ✅ Dynamic updates are handled with validation
- ✅ Parameter changes are logged
- ✅ Compatible with ROS 2 parameter tools (`ros2 param`)
