#!/usr/bin/env python3
"""
Launch file for Interactive Topological Map Editor

Usage:
    ros2 launch topological_navigation interactive_map_editor.launch.py \
        map_file:=/path/to/map.tmap2.yaml

Example:
    ros2 launch topological_navigation interactive_map_editor.launch.py \
        map_file:=$(ros2 pkg prefix topological_navigation)/share/topological_navigation/config/test_simple_tmap2.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to topological map YAML file (.tmap2.yaml)'
    )
    
    auto_save_arg = DeclareLaunchArgument(
        'auto_save',
        default_value='false',
        description='Enable automatic saving every 30 seconds'
    )
    
    marker_scale_arg = DeclareLaunchArgument(
        'marker_scale',
        default_value='0.5',
        description='Scale of interactive markers in RViz'
    )
    
    # Interactive map editor node
    editor_node = Node(
        package='topological_navigation',
        executable='interactive_map_editor.py',
        name='interactive_map_editor',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_file'),
            'auto_save': LaunchConfiguration('auto_save'),
            'marker_scale': LaunchConfiguration('marker_scale'),
        }]
    )
    
    return LaunchDescription([
        map_file_arg,
        auto_save_arg,
        marker_scale_arg,
        editor_node,
    ])
