#!/usr/bin/env python3
"""Unified topological map visualiser with interactive editing.

Provides a single ROS 2 node that:
- Subscribes to ``/topological_map_2`` and renders nodes, edges, zones, and
  labels as RViz ``MarkerArray`` messages.
- Optionally loads a map directly from a YAML file for editing (``map_file``
  parameter).
- Creates interactive markers for every node, allowing the user to drag
  positions (X/Y) and rotate yaw in RViz.
- Saves the modified map back to YAML and re-publishes it on
  ``/topological_map_2`` so that downstream nodes pick up changes
  immediately.

Usage
-----
**Live visualisation only** (subscribes to map topic)::

    ros2 run topological_navigation_visual topological_map_visualiser.py

**Interactive editing from file**::

    ros2 run topological_navigation_visual topological_map_visualiser.py \\
        --ros-args -p map_file:=/path/to/map.tmap2.yaml

**Save** (from another terminal)::

    ros2 service call /topological_map_visualiser/save_map std_srvs/srv/Trigger

"""

import math
import os
import sys
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

import yaml
import tf_transformations

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
from interactive_markers import InteractiveMarkerServer

from topological_navigation.map_types import CustomSafeLoader
import topological_navigation.tmap_utils as tmap_utils

# ──────────────────────────────────────────────────────────────────
#  Colour palette
# ──────────────────────────────────────────────────────────────────
_PALETTE = [
    [0.2, 0.2, 0.7],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [1.0, 1.0, 0.0],
    [1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0],
    [1.0, 1.0, 1.0],
]


def _colour(index: int):
    """Return an RGB list from the palette, wrapping around."""
    return _PALETTE[index % len(_PALETTE)]


def _node2pose(pose_dict) -> Pose:
    """Convert a tmap2 pose dict to a ``geometry_msgs/Pose``."""
    p = Pose()
    p.position.x = float(pose_dict['position']['x'])
    p.position.y = float(pose_dict['position']['y'])
    p.position.z = float(pose_dict['position']['z'])
    p.orientation.w = float(pose_dict['orientation']['w'])
    p.orientation.x = float(pose_dict['orientation']['x'])
    p.orientation.y = float(pose_dict['orientation']['y'])
    p.orientation.z = float(pose_dict['orientation']['z'])
    return p


def _get_node(nodes_list, name):
    """Look up a node dict by name from the tmap2 node list."""
    for entry in nodes_list:
        if entry['node']['name'] == name:
            return entry['node']
    return None


# ══════════════════════════════════════════════════════════════════
#  Main visualiser node
# ══════════════════════════════════════════════════════════════════
class TopologicalMapVisualiser(Node):
    """Unified map visualiser with optional interactive editing."""

    def __init__(self):
        super().__init__('topological_map_visualiser')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('map_file', '')
        self.declare_parameter('auto_save', False)
        self.declare_parameter('marker_scale', 0.5)
        self.declare_parameter('edit_mode', True)

        self.map_file: str = self.get_parameter('map_file').value
        self.auto_save: bool = self.get_parameter('auto_save').value
        self.marker_scale: float = self.get_parameter('marker_scale').value
        self.edit_mode: bool = self.get_parameter('edit_mode').value

        # ── State ────────────────────────────────────────────────
        self.tmap = None
        self._map_dirty = False

        # ── QoS ──────────────────────────────────────────────────
        self._latching_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._cb_group = ReentrantCallbackGroup()

        # ── Publishers ───────────────────────────────────────────
        self.map_marker_pub = self.create_publisher(
            MarkerArray,
            'topological_map_visualisation',
            qos_profile=self._latching_qos,
        )
        self.map_topic_pub = self.create_publisher(
            String,
            '/topological_map_2',
            qos_profile=self._latching_qos,
        )

        # ── Interactive-marker server (for dragging nodes) ──────
        self._im_server = InteractiveMarkerServer(
            self, 'topological_map_editor'
        )

        # ── Load map from file or subscribe to topic ─────────────
        if self.map_file:
            self._load_map_from_file()
            self._publish_map_topic()
            self._rebuild_visualisation()
        else:
            self.get_logger().info(
                'No map_file parameter — subscribing to /topological_map_2'
            )

        # Always subscribe so we pick up external updates too
        self.create_subscription(
            String,
            '/topological_map_2',
            self._map_topic_cb,
            qos_profile=self._latching_qos,
            callback_group=self._cb_group,
        )

        # ── Save service ─────────────────────────────────────────
        self.create_service(
            Trigger,
            f'{self.get_name()}/save_map',
            self._save_map_service,
            callback_group=self._cb_group,
        )

        # ── Auto-save timer ──────────────────────────────────────
        if self.auto_save and self.map_file:
            self.create_timer(30.0, self._auto_save_cb)

        self.get_logger().info('Topological map visualiser started')
        if self.map_file:
            self.get_logger().info(f'  map_file : {self.map_file}')
            self.get_logger().info(f'  edit_mode: {self.edit_mode}')
            self.get_logger().info(f'  auto_save: {self.auto_save}')
        self.get_logger().info(
            'Call /<node>/save_map service to persist changes'
        )

    # ──────────────────────────────────────────────────────────────
    #  Map loading / saving
    # ──────────────────────────────────────────────────────────────
    def _load_map_from_file(self):
        """Load a tmap2 YAML file into ``self.tmap``."""
        try:
            with open(self.map_file, 'r') as fh:
                self.tmap = yaml.load(fh, Loader=CustomSafeLoader)
            self.get_logger().info(
                f'Loaded map with {len(self.tmap.get("nodes", []))} nodes '
                f'from {self.map_file}'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to load map file: {exc}')
            sys.exit(1)

    def save_map(self):
        """Save current map to the YAML file (with backup)."""
        if not self.map_file:
            self.get_logger().warn('No map_file set — cannot save')
            return False
        try:
            backup = self.map_file + '.backup'
            if os.path.exists(self.map_file):
                if os.path.exists(backup):
                    os.remove(backup)
                os.rename(self.map_file, backup)

            with open(self.map_file, 'w') as fh:
                yaml.dump(
                    self.tmap,
                    fh,
                    default_flow_style=False,
                    sort_keys=False,
                )
            self.get_logger().info(f'Map saved to {self.map_file}')
            self._map_dirty = False
            self._publish_map_topic()
            return True

        except Exception as exc:
            self.get_logger().error(f'Save failed: {exc}')
            backup = self.map_file + '.backup'
            if os.path.exists(backup):
                os.rename(backup, self.map_file)
            return False

    # ── Callbacks ────────────────────────────────────────────────
    def _map_topic_cb(self, msg: String):
        """Handle updates from the ``/topological_map_2`` topic."""
        incoming = yaml.load(msg.data, Loader=CustomSafeLoader)
        if incoming == self.tmap:
            return  # no change, avoid flicker
        self.tmap = incoming
        self.get_logger().info('Received updated map from topic')
        self._rebuild_visualisation()

    def _save_map_service(self, request, response):
        ok = self.save_map()
        response.success = ok
        response.message = (
            f'Map saved to {self.map_file}' if ok else 'Save failed'
        )
        return response

    def _auto_save_cb(self):
        if self._map_dirty:
            self.get_logger().info('Auto-saving map…')
            self.save_map()

    # ──────────────────────────────────────────────────────────────
    #  Publishing helpers
    # ──────────────────────────────────────────────────────────────
    def _publish_map_topic(self):
        """Publish the in-memory map to ``/topological_map_2``."""
        if self.tmap is None:
            return
        msg = String()
        msg.data = yaml.dump(
            self.tmap, default_flow_style=False, sort_keys=False
        )
        self.map_topic_pub.publish(msg)
        self.get_logger().info('Published map to /topological_map_2')

    # ──────────────────────────────────────────────────────────────
    #  Build / rebuild all visualisation markers
    # ──────────────────────────────────────────────────────────────
    def _rebuild_visualisation(self):
        """Re-create marker array + interactive markers from ``self.tmap``."""
        if self.tmap is None:
            return

        nodes = self.tmap.get('nodes', [])
        marker_array = MarkerArray()
        actions_seen: list = []
        idn = 0

        for entry in nodes:
            node = entry['node']

            # Collect unique edge actions for the legend
            for edge in node.get('edges', []):
                act = edge.get('action', '')
                if act and act not in actions_seen:
                    actions_seen.append(act)

            # Node sphere
            marker_array.markers.append(self._mk_node(node, idn))
            idn += 1
            # Name label
            marker_array.markers.append(self._mk_name(node, idn))
            idn += 1
            # Influence zone
            if node.get('verts'):
                marker_array.markers.append(self._mk_zone(node, idn))
                idn += 1
            # Edges
            for edge in node.get('edges', []):
                m = self._mk_edge(node, edge, actions_seen)
                if m is not None:
                    m.id = idn
                    marker_array.markers.append(m)
                    idn += 1

        # Legend
        for row, action_name in enumerate(actions_seen):
            marker_array.markers.append(
                self._mk_legend(action_name, row, actions_seen, idn)
            )
            idn += 1

        self.map_marker_pub.publish(marker_array)

        # Interactive editor markers
        if self.edit_mode:
            self._rebuild_interactive_markers(nodes)

        self.get_logger().info(
            f'Visualisation published ({len(nodes)} nodes, '
            f'{idn} markers)'
        )

    # ──────────────────────────────────────────────────────────────
    #  Interactive marker layer
    # ──────────────────────────────────────────────────────────────
    def _rebuild_interactive_markers(self, nodes):
        """Create / update interactive markers for every node."""
        # Clear existing markers
        self._im_server.clear()

        for entry in nodes:
            node = entry['node']
            self._create_edit_marker(node)

        self._im_server.applyChanges()

    def _create_edit_marker(self, node: dict):
        """Insert one interactive marker for *node*."""
        scale = self.marker_scale

        im = InteractiveMarker()
        im.header.frame_id = node.get('parent_frame', 'map')
        im.name = node['name']
        im.description = ''

        pose = _node2pose(node['pose'])
        im.pose = pose

        # ── Visible sphere ──────────────────────────────────────
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = scale
        sphere.scale.y = scale
        sphere.scale.z = scale
        sphere.color.r = 0.0
        sphere.color.g = 0.5
        sphere.color.b = 1.0
        sphere.color.a = 0.8

        vis_ctrl = InteractiveMarkerControl()
        vis_ctrl.always_visible = True
        vis_ctrl.markers.append(sphere)
        vis_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        vis_ctrl.orientation.w = 1.0
        vis_ctrl.orientation.y = 1.0  # XY plane
        im.controls.append(vis_ctrl)

        # ── Move X ──────────────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.x = 1.0
        ctrl.name = 'move_x'
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(ctrl)

        # ── Move Y ──────────────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.z = 1.0
        ctrl.name = 'move_y'
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(ctrl)

        # ── Rotate Z (yaw) ─────────────────────────────────────
        ctrl = InteractiveMarkerControl()
        ctrl.orientation.w = 1.0
        ctrl.orientation.y = 1.0
        ctrl.name = 'rotate_z'
        ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(ctrl)

        # ── Arrow showing orientation ───────────────────────────
        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = scale * 1.5
        arrow.scale.y = scale * 0.2
        arrow.scale.z = scale * 0.2
        arrow.color.r = 1.0
        arrow.color.a = 1.0

        arrow_ctrl = InteractiveMarkerControl()
        arrow_ctrl.always_visible = True
        arrow_ctrl.markers.append(arrow)
        im.controls.append(arrow_ctrl)

        # ── Text label ──────────────────────────────────────────
        txt = Marker()
        txt.type = Marker.TEXT_VIEW_FACING
        txt.text = node['name']
        txt.scale.z = scale * 0.5
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.position.z = scale * 0.8

        txt_ctrl = InteractiveMarkerControl()
        txt_ctrl.always_visible = True
        txt_ctrl.markers.append(txt)
        im.controls.append(txt_ctrl)

        self._im_server.insert(im)
        self._im_server.setCallback(im.name, self._im_feedback)

    def _im_feedback(self, feedback: InteractiveMarkerFeedback):
        """Handle interactive-marker drag / rotate events."""
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            self._im_server.applyChanges()
            return

        name = feedback.marker_name
        for entry in self.tmap.get('nodes', []):
            if entry['node']['name'] != name:
                continue

            node = entry['node']
            node['pose']['position']['x'] = float(
                feedback.pose.position.x
            )
            node['pose']['position']['y'] = float(
                feedback.pose.position.y
            )
            node['pose']['position']['z'] = 0.0

            euler = tf_transformations.euler_from_quaternion([
                feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w,
            ])
            yaw = euler[2]
            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
            node['pose']['orientation']['x'] = float(quat[0])
            node['pose']['orientation']['y'] = float(quat[1])
            node['pose']['orientation']['z'] = float(quat[2])
            node['pose']['orientation']['w'] = float(quat[3])

            self._map_dirty = True
            yaw_deg = math.degrees(yaw)
            self.get_logger().debug(
                f'{name}: pos=({feedback.pose.position.x:.2f}, '
                f'{feedback.pose.position.y:.2f}), yaw={yaw_deg:.1f}°'
            )
            break

        # Refresh the static markers (edges, zones) to reflect the move
        self._rebuild_static_markers()
        self._im_server.applyChanges()

    def _rebuild_static_markers(self):
        """Re-publish static MarkerArray only (no interactive markers)."""
        if self.tmap is None:
            return

        nodes = self.tmap.get('nodes', [])
        marker_array = MarkerArray()
        actions_seen: list = []
        idn = 0

        for entry in nodes:
            node = entry['node']
            for edge in node.get('edges', []):
                act = edge.get('action', '')
                if act and act not in actions_seen:
                    actions_seen.append(act)

            marker_array.markers.append(self._mk_node(node, idn))
            idn += 1
            marker_array.markers.append(self._mk_name(node, idn))
            idn += 1
            if node.get('verts'):
                marker_array.markers.append(self._mk_zone(node, idn))
                idn += 1
            for edge in node.get('edges', []):
                m = self._mk_edge(node, edge, actions_seen)
                if m is not None:
                    m.id = idn
                    marker_array.markers.append(m)
                    idn += 1

        for row, action_name in enumerate(actions_seen):
            marker_array.markers.append(
                self._mk_legend(action_name, row, actions_seen, idn)
            )
            idn += 1

        self.map_marker_pub.publish(marker_array)

    # ──────────────────────────────────────────────────────────────
    #  Marker factory helpers
    # ──────────────────────────────────────────────────────────────
    def _mk_node(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.SPHERE
        s = self.marker_scale
        m.scale.x = s * 0.4
        m.scale.y = s * 0.4
        m.scale.z = s * 0.4
        m.color.a = 0.4
        m.color.r = 0.2
        m.color.g = 0.2
        m.color.b = 0.7
        m.pose = _node2pose(node['pose'])
        m.pose.position.z += 0.1
        m.ns = '/nodes'
        return m

    def _mk_name(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.TEXT_VIEW_FACING
        m.text = node['name']
        m.pose = _node2pose(node['pose'])
        m.pose.position.z += 0.25
        m.scale.z = self.marker_scale * 0.24
        m.color.a = 0.9
        m.color.r = 0.3
        m.color.g = 0.3
        m.color.b = 0.3
        m.ns = '/names'
        return m

    def _mk_zone(self, node: dict, idn: int) -> Marker:
        m = Marker()
        m.id = idn
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.LINE_STRIP
        m.pose.orientation.w = 1.0
        m.scale.x = self.marker_scale * 0.2
        m.color.a = 0.8
        m.color.r = 0.7
        m.color.g = 0.1
        m.color.b = 0.2

        px = float(node['pose']['position']['x'])
        py = float(node['pose']['position']['y'])
        pz = float(node['pose']['position']['z'])

        for v in node['verts']:
            pt = Point()
            pt.x = px + float(v['x'])
            pt.y = py + float(v['y'])
            pt.z = pz
            m.points.append(pt)

        # Close the polygon
        first = node['verts'][0]
        pt = Point()
        pt.x = px + float(first['x'])
        pt.y = py + float(first['y'])
        pt.z = pz
        m.points.append(pt)
        m.ns = '/zones'
        return m

    def _mk_edge(
        self, node: dict, edge: dict, actions: list
    ) -> Marker | None:
        to_node = _get_node(self.tmap['nodes'], edge['node'])
        if to_node is None:
            self.get_logger().warn(
                f"Edge target node '{edge['node']}' not found"
            )
            return None

        action = edge.get('action', '')
        col_idx = actions.index(action) if action in actions else 0
        col = _colour(col_idx)

        m = Marker()
        m.header.frame_id = node.get('parent_frame', 'map')
        m.type = Marker.LINE_LIST

        v1 = _node2pose(node['pose']).position
        v1.z += 0.1
        v2 = _node2pose(to_node['pose']).position
        v2.z += 0.1

        m.pose.orientation.w = 1.0
        m.scale.x = self.marker_scale * 0.2
        m.color.a = 0.5
        m.color.r = float(col[0])
        m.color.g = float(col[1])
        m.color.b = float(col[2])
        m.points.append(v1)
        m.points.append(v2)
        m.ns = '/edges'
        return m

    def _mk_legend(
        self, action: str, row: int, actions: list, idn: int
    ) -> Marker:
        col_idx = actions.index(action) if action in actions else 0
        col = _colour(col_idx)
        m = Marker()
        m.id = idn
        m.header.frame_id = 'map'
        m.type = Marker.TEXT_VIEW_FACING
        m.text = action
        m.pose.position.x = 1.0
        m.pose.position.y = 0.18 * row
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.z = self.marker_scale * 0.3
        m.color.a = 1.0
        m.color.r = float(col[0])
        m.color.g = float(col[1])
        m.color.b = float(col[2])
        m.ns = '/legend'
        return m


# ══════════════════════════════════════════════════════════════════
#  Entry-point
# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = TopologicalMapVisualiser()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down topological map visualiser')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
