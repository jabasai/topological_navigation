#!/usr/bin/env python3
"""Policy visualisation node.

Subscribes to ``/topological_map_2`` for the current map and to
``mdp_plan_exec/current_policy_mode`` for the active policy.  Publishes
arrow markers on ``~/vis`` to show policy edges.
"""

import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler

from topological_navigation.map_types import CustomSafeLoader
from topological_navigation.route_search2 import TopologicalRouteSearch2
from topological_navigation.tmap_utils import get_edge_from_id_tmap2

from topological_navigation_msgs.msg import NavRoute


class PoliciesVis(Node):
    """Visualise the current topological policy as arrows in RViz."""

    def __init__(self):
        super().__init__('topological_policy_markers')
        self.lnodes = None
        self.rsearch = None
        self.policy = MarkerArray()

        self.marker_pub = self.create_publisher(MarkerArray, '~/vis', 10)
        self.marker_pub.publish(self.policy)

        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            String, '/topological_map_2', self._map_cb, qos
        )
        self.create_subscription(
            NavRoute, 'mdp_plan_exec/current_policy_mode',
            self._policy_cb, qos,
        )
        self.get_logger().info('Policy visualiser started')

    def _map_cb(self, msg: String):
        self.lnodes = yaml.load(msg.data, Loader=CustomSafeLoader)
        self.rsearch = TopologicalRouteSearch2(self.lnodes)
        self.get_logger().info('Map received for policy visualisation')

    def _policy_cb(self, msg: NavRoute):
        if self.lnodes is None:
            return

        self.policy = MarkerArray()
        added_sources = []

        for i in range(len(msg.source)):
            source = msg.source[i]
            edge_id = msg.edge_id[i]

            ori = self.rsearch.get_node_from_tmap2(source)
            edge_data = get_edge_from_id_tmap2(
                self.lnodes, source, edge_id
            )
            if edge_data is None:
                continue

            target = self.rsearch.get_node_from_tmap2(edge_data['node'])
            if target is None:
                continue

            added_sources.append(source)
            colour = (
                [0.1, 0.1, 0.9]
                if edge_data['node'] in added_sources
                else [0.9, 0.1, 0.1]
            )
            self.policy.markers.append(
                self._arrow(
                    ori['node']['pose'], target['node']['pose'], colour
                )
            )

        for idx, m in enumerate(self.policy.markers):
            m.id = idx
        self.marker_pub.publish(self.policy)

    @staticmethod
    def _arrow(pose1_dict, pose2_dict, colour) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.ARROW

        p1x = float(pose1_dict['position']['x'])
        p1y = float(pose1_dict['position']['y'])
        p1z = float(pose1_dict['position']['z'])
        p2x = float(pose2_dict['position']['x'])
        p2y = float(pose2_dict['position']['y'])

        angle = math.atan2(p2y - p1y, p2x - p1x)
        qat = quaternion_from_euler(0, 0, angle)

        pose = Pose()
        pose.position.x = p1x
        pose.position.y = p1y
        pose.position.z = p1z
        pose.orientation.w = qat[3]
        pose.orientation.x = qat[0]
        pose.orientation.y = qat[1]
        pose.orientation.z = qat[2]

        r = math.hypot(p2y - p1y, p2x - p1x)
        m.scale.x = r
        m.scale.y = 0.15
        m.scale.z = 0.15
        m.color.a = 0.95
        m.color.r = float(colour[0])
        m.color.g = float(colour[1])
        m.color.b = float(colour[2])
        m.pose = pose
        return m


def main(args=None):
    rclpy.init(args=args)
    node = PoliciesVis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
