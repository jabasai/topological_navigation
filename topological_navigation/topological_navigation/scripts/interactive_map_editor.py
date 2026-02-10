#!/usr/bin/env python3
"""
Interactive Topological Map Editor for RViz2

Allows interactive editing of topological map nodes for 2D navigation:
- Move node positions in X and Y (dragging markers)
- Rotate node orientations around Z axis (yaw only)
- Save changes back to YAML file

Designed for 2D ground robot navigation - Z position is fixed at 0,
and only yaw rotation is allowed (roll and pitch are kept at 0).

Author: AI Assistant
Date: 2026-02-10
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger
import yaml
import sys
import os
from copy import deepcopy
import tf_transformations

class InteractiveMapEditor(Node):
    """
    Interactive editor for topological maps using RViz2 Interactive Markers
    """
    
    def __init__(self):
        super().__init__('interactive_map_editor')
        
        # Parameters
        self.declare_parameter('map_file', '')
        self.declare_parameter('auto_save', False)
        self.declare_parameter('marker_scale', 0.5)
        
        self.map_file = self.get_parameter('map_file').value
        self.auto_save = self.get_parameter('auto_save').value
        self.marker_scale = self.get_parameter('marker_scale').value
        
        if not self.map_file:
            self.get_logger().error('No map_file parameter provided!')
            self.get_logger().info('Usage: ros2 run topological_navigation interactive_map_editor.py --ros-args -p map_file:=/path/to/map.yaml')
            sys.exit(1)
        
        # Load map
        self.tmap = None
        self.load_map()
        
        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'topological_map_editor')
        
        # QoS for publishers
        self.latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # Publisher for map updates
        self.map_pub = self.create_publisher(String, '/topological_map_2', qos_profile=self.latching_qos)
        
        # Create interactive markers for all nodes
        self.create_interactive_markers()
        
        # Timer for periodic save (if auto_save enabled)
        if self.auto_save:
            self.save_timer = self.create_timer(30.0, self.save_map)  # Save every 30 seconds
        
        self.get_logger().info(f'Interactive Map Editor started for: {self.map_file}')
        self.get_logger().info(f'Loaded {len(self.tmap["nodes"])} nodes')
        self.get_logger().info('Use RViz2 Interactive Markers to edit node positions and orientations')
        self.get_logger().info('Call service /interactive_map_editor/save_map to save changes')
        
        # Service to save map
        self.create_service(
            Trigger,
            f'{self.get_name()}/save_map',
            self.save_map_service
        )
    
    def load_map(self):
        """Load topological map from YAML file"""
        try:
            with open(self.map_file, 'r') as f:
                self.tmap = yaml.safe_load(f)
            self.get_logger().info(f'Loaded map from {self.map_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            sys.exit(1)
    
    def save_map(self):
        """Save topological map to YAML file"""
        try:
            # Backup original file
            backup_file = self.map_file + '.backup'
            if os.path.exists(self.map_file):
                os.rename(self.map_file, backup_file)
            
            # Save updated map
            with open(self.map_file, 'w') as f:
                yaml.dump(self.tmap, f, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f'Saved map to {self.map_file}')
            
            # Publish updated map
            self.publish_map()
            
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            # Restore backup if save failed
            if os.path.exists(backup_file):
                os.rename(backup_file, self.map_file)
    
    def save_map_service(self, request, response):
        """Service callback to save map"""
        self.save_map()
        response.success = True
        response.message = f'Map saved to {self.map_file}'
        return response
    
    def publish_map(self):
        """Publish map to /topological_map_2 topic"""
        msg = String()
        msg.data = yaml.dump(self.tmap, default_flow_style=False, sort_keys=False)
        self.map_pub.publish(msg)
        self.get_logger().info('Published updated map to /topological_map_2')
    
    def create_interactive_markers(self):
        """Create interactive markers for all nodes in the map"""
        for node_data in self.tmap['nodes']:
            node = node_data['node']
            self.create_node_marker(node)
        
        self.server.applyChanges()
    
    def create_node_marker(self, node):
        """Create an interactive marker for a single node"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = node.get('parent_frame', 'map')
        int_marker.name = node['name']
        int_marker.description = f"Node: {node['name']}"
        
        # Set pose
        int_marker.pose.position.x = float(node['pose']['position']['x'])
        int_marker.pose.position.y = float(node['pose']['position']['y'])
        int_marker.pose.position.z = float(node['pose']['position'].get('z', 0.0))
        int_marker.pose.orientation.x = float(node['pose']['orientation']['x'])
        int_marker.pose.orientation.y = float(node['pose']['orientation']['y'])
        int_marker.pose.orientation.z = float(node['pose']['orientation']['z'])
        int_marker.pose.orientation.w = float(node['pose']['orientation']['w'])
        
        # Create a sphere marker for the node
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Create control for the marker - MOVE_PLANE in XY (horizontal plane)
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker_control.orientation.w = 1.0
        marker_control.orientation.x = 0.0
        marker_control.orientation.y = 1.0  # Point up (Z axis) for XY plane movement
        marker_control.orientation.z = 0.0
        int_marker.controls.append(marker_control)
        
        # Add 2D controls (move in X, Y and rotate around Z only)
        
        # Move X
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Move Y
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        # Rotate Z (yaw - for ground robots)
        # Orientation points along Z axis (up) for rotation around Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0  # Point along Z axis (up)
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        # Add arrow showing orientation
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = self.marker_scale * 1.5  # Length
        arrow.scale.y = self.marker_scale * 0.2  # Width
        arrow.scale.z = self.marker_scale * 0.2  # Height
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        
        arrow_control = InteractiveMarkerControl()
        arrow_control.always_visible = True
        arrow_control.markers.append(arrow)
        int_marker.controls.append(arrow_control)
        
        # Add text label
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.text = node['name']
        text.scale.z = self.marker_scale * 0.5
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.pose.position.z = self.marker_scale * 0.8
        
        text_control = InteractiveMarkerControl()
        text_control.always_visible = True
        text_control.markers.append(text)
        int_marker.controls.append(text_control)
        
        # Insert marker and set callback
        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
    
    def process_feedback(self, feedback):
        """Process feedback from interactive marker"""
        node_name = feedback.marker_name
        
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update node pose in map
            for node_data in self.tmap['nodes']:
                if node_data['node']['name'] == node_name:
                    node = node_data['node']
                    
                    # Update position (X, Y only - keep Z at 0 for 2D navigation)
                    node['pose']['position']['x'] = float(feedback.pose.position.x)
                    node['pose']['position']['y'] = float(feedback.pose.position.y)
                    node['pose']['position']['z'] = 0.0  # Force Z to 0 for 2D
                    
                    # Update orientation (yaw only - keep roll and pitch at 0)
                    # Extract yaw from the feedback quaternion
                    euler = tf_transformations.euler_from_quaternion([
                        feedback.pose.orientation.x,
                        feedback.pose.orientation.y,
                        feedback.pose.orientation.z,
                        feedback.pose.orientation.w
                    ])
                    yaw = euler[2]  # Only keep yaw
                    
                    # Convert back to quaternion with roll=0, pitch=0, yaw=yaw
                    quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                    node['pose']['orientation']['x'] = float(quat[0])
                    node['pose']['orientation']['y'] = float(quat[1])
                    node['pose']['orientation']['z'] = float(quat[2])
                    node['pose']['orientation']['w'] = float(quat[3])
                    
                    # Log the update
                    yaw_deg = yaw * 180.0 / 3.14159
                    
                    self.get_logger().info(
                        f'Updated {node_name}: '
                        f'pos=({feedback.pose.position.x:.2f}, {feedback.pose.position.y:.2f}), '
                        f'yaw={yaw_deg:.1f}°'
                    )
                    
                    break
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.get_logger().info(f'Menu select for {node_name}')
        
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        editor = InteractiveMapEditor()
        rclpy.spin(editor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
