#!/usr/bin/env python
"""Topological localisation node for ROS 2.

Determines the robot's current topological node and closest node by
subscribing to TF transforms and the topological map topic.  Uses a
NetworkX directed graph and a KD-tree spatial index for efficient
O(log n) nearest-neighbour queries.

Publishers:
    ~/closest_node          (std_msgs/String)   - name of the closest node
    ~/closest_node_distance (std_msgs/Float32)  - distance to closest node
    ~/current_node          (std_msgs/String)   - node whose influence zone
                                                   the robot currently occupies
    ~/closest_edges         (topological_navigation_msgs/ClosestEdges)
    ~/current_node/tag      (std_msgs/String)   - tag of the current node

Services:
    /topological_localisation/localise_pose (LocalisePose)

Subscriptions:
    /topological_map_2      (std_msgs/String)   - YAML-encoded topological map
"""

import numpy as np
import rclpy
import yaml

from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from topological_navigation.map_types import CustomSafeLoader
from topological_navigation.networkx_utils import (
    build_graph_from_tmap,
    build_kdtree_from_graph,
    determine_closest_node,
    determine_current_node,
    get_edge_distances_nx,
    update_loc_by_topic_nx,
)
from topological_navigation_msgs.msg import ClosestEdges
from topological_navigation_msgs.srv import LocalisePose

try:
    from topological_navigation_msgs.srv import GetTaggedNodes
    _HAS_GET_TAGGED_NODES = True
except ImportError:
    _HAS_GET_TAGGED_NODES = False

class TopologicalNavLoc(rclpy.node.Node):
    """ROS 2 node for topological localisation.

    Determines which topological node the robot currently occupies
    (influence-zone check) and which node is closest (KD-tree query).
    Publishes the results on latched topics.
    """

    def __init__(self, name: str, with_tags: bool = True):
        super().__init__(name)

        # -- ROS parameters --------------------------------------------------
        self.declare_parameter('LocalisationThrottle', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('OnlyLatched', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('base_frame', rclpy.Parameter.Type.STRING)

        self.throttle_val = self.get_parameter_or(
            "LocalisationThrottle",
            Parameter('int', Parameter.Type.INTEGER, 1),
        ).value
        self.only_latched = self.get_parameter_or(
            "OnlyLatched",
            Parameter('bool', Parameter.Type.BOOL, True),
        ).value
        self.base_frame = self.get_parameter_or(
            "base_frame",
            Parameter('str', Parameter.Type.STRING, "base_link"),
        ).value

        # -- Internal state ---------------------------------------------------
        self.throttle = self.throttle_val
        self.wpstr = "Unknown"
        self.closest_dist = 1e6 - 1
        self.cnstr = "Unknown"
        self.nodetag = "Unknown"
        self.closest_edge_ids: list = []
        self.closest_edge_dists: list = []

        # NetworkX graph and KD-tree data structures
        self._graph = None
        self._kdtree = None
        self._kdtree_node_names: list = []

        self.with_tags = with_tags

        # -- QoS profile (transient-local for late-joining subscribers) -------
        self.qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -- Publishers -------------------------------------------------------
        self.wp_pub = self.create_publisher(String, 'closest_node', qos_profile=self.qos)
        self.wd_pub = self.create_publisher(Float32, 'closest_node_distance', qos_profile=self.qos)
        self.cn_pub = self.create_publisher(String, 'current_node', qos_profile=self.qos)
        self.ce_pub = self.create_publisher(ClosestEdges, 'closest_edges', qos_profile=self.qos)
        self.tag_pub = self.create_publisher(String, 'current_node/tag', qos_profile=self.qos)

        # -- Localisation state -----------------------------------------------
        self.rec_map = False
        self.set_nogos = False
        self.loc_by_topic: list = []

        self.current_pose = Pose()

        # -- Callback groups --------------------------------------------------
        self._cb_group_localise = ReentrantCallbackGroup()
        self._cb_group_map = ReentrantCallbackGroup()

        # -- Service ----------------------------------------------------------
        self.loc_pos_srv = self.create_service(
            LocalisePose,
            '/topological_localisation/localise_pose',
            self.localise_pose_cb,
            callback_group=self._cb_group_localise,
        )

        # -- Subscription -----------------------------------------------------
        self.create_subscription(
            String,
            '/topological_map_2',
            self._map_callback,
            qos_profile=self.qos,
            callback_group=self._cb_group_map,
        )

        # -- Wait for the first map -------------------------------------------
        self.get_logger().info("Waiting for the topological map on /topological_map_2 ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.rec_map:
                if self.with_tags:
                    self.nogos = self._get_no_go_nodes()
                    self.set_nogos = True
                else:
                    self.nogos = []
                self.get_logger().info(f"No-go nodes: {self.nogos}")
                self.get_logger().info(f"Localise-by-topic nodes: {self.names_by_topic}")
                self.get_logger().info(
                    f"Listening for TF: {self.tmap_frame} -> {self.base_frame}"
                )
                break
            self.get_logger().warning("Still waiting for the topological map ...")

        # -- TF listener & periodic callback ----------------------------------
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._pose_callback)

    def get_edge_distances_to_pose(self, pose: Pose):
        """Return edge-ID list and distance array for all edges relative to *pose*.

        Uses the NetworkX graph for vectorised edge distance calculations.

        Returns:
            Tuple ``(edge_ids, distances)`` - both may be empty if the graph
            is unavailable.
        """
        if self._graph is None:
            self.get_logger().warning(
                "Cannot compute edge distances: graph not yet available"
            )
            return [], np.array([])

        return get_edge_distances_nx(self._graph, pose, logger=self.get_logger())

    # -----------------------------------------------------------------
    # Periodic TF-based localisation
    # -----------------------------------------------------------------

    def _pose_callback(self):
        """Look up the TF transform and localise the robot in the topological map."""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.tmap_frame, self.base_frame, rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warning(
                f"TF lookup failed ({self.tmap_frame} -> {self.base_frame}): {ex}"
            )
            return

        msg = Pose()
        msg.position.x = trans.transform.translation.x
        msg.position.y = trans.transform.translation.y
        msg.position.z = trans.transform.translation.z
        msg.orientation.x = trans.transform.rotation.x
        msg.orientation.y = trans.transform.rotation.y
        msg.orientation.z = trans.transform.rotation.z
        msg.orientation.w = trans.transform.rotation.w
        self.current_pose = msg

        if self.throttle % self.throttle_val != 0:
            self.throttle += 1
            return

        if self._graph is None or self._kdtree is None:
            self.get_logger().warning(
                "Localisation skipped: graph or KD-tree not ready"
            )
            self.throttle += 1
            return

        # Current node (inside influence zone)
        currentstr = determine_current_node(
            self._graph, self._kdtree, self._kdtree_node_names,
            msg, self.loc_by_topic, self.nogos,
        )

        # Closest node by distance
        closeststr, closest_dist = determine_closest_node(
            self._kdtree, self._kdtree_node_names, self._graph,
            currentstr, self.nogos, self.names_by_topic, msg,
        )

        # Closest edges (computed from the *current* pose)
        closest_edges, edge_dists = self.get_edge_distances_to_pose(msg)
        if len(closest_edges) > 1:
            closest_edges = closest_edges[:2]
            edge_dists = edge_dists[:2]

        # Resolve node tag
        nodetag = self._get_node_tag(closeststr)

        # Publish
        closest_dist = float(np.round(closest_dist, 3))
        self._publish_topics(
            closeststr, closest_dist, currentstr,
            closest_edges, list(np.round(edge_dists, 3)), nodetag,
        )
        self.throttle = 1
        

    # -----------------------------------------------------------------
    # Tag helper
    # -----------------------------------------------------------------

    def _get_node_tag(self, node_name: str) -> str:
        """Return the first tag string for *node_name*, or ``'Unknown'``.

        Uses the NetworkX graph ``meta`` attribute instead of a linear
        YAML lookup.
        """
        if self._graph is None or node_name not in self._graph.nodes:
            return 'Unknown'
        meta = self._graph.nodes[node_name].get('meta', {})
        try:
            return meta['tag'][0]
        except (KeyError, IndexError, TypeError):
            return 'Unknown'

    # -----------------------------------------------------------------
    # Message construction helpers
    # -----------------------------------------------------------------

    @staticmethod
    def _make_string_msg(text: str) -> String:
        msg = String()
        msg.data = text
        return msg

    @staticmethod
    def _make_float32_msg(value: float) -> Float32:
        msg = Float32()
        msg.data = value
        return msg

    # -----------------------------------------------------------------
    # Topic publishing
    # -----------------------------------------------------------------

    def _publish_topics(
        self,
        wpstr: str,
        closest_dist: float,
        cnstr: str,
        closest_edge_ids: list,
        closest_edge_dists: list,
        nodetag: str = 'Unknown',
    ):
        """Publish localisation results, optionally in latched mode."""

        def _pub_edges(edge_ids, edge_dists):
            msg = ClosestEdges()
            msg.edge_ids = edge_ids
            msg.distances = edge_dists
            self.ce_pub.publish(msg)

        if len(set(closest_edge_dists)) == 1:
            closest_edge_ids.sort()

        if self.only_latched:
            if self.wpstr != wpstr:
                self.wp_pub.publish(self._make_string_msg(wpstr))
            if self.closest_dist != closest_dist:
                self.wd_pub.publish(self._make_float32_msg(closest_dist))
            if self.cnstr != cnstr:
                self.cn_pub.publish(self._make_string_msg(cnstr))
            if self.nodetag != nodetag:
                self.tag_pub.publish(self._make_string_msg(nodetag))
            if (self.closest_edge_ids != closest_edge_ids
                    or self.closest_edge_dists != closest_edge_dists):
                _pub_edges(closest_edge_ids, closest_edge_dists)
        else:
            self.wp_pub.publish(self._make_string_msg(wpstr))
            self.wd_pub.publish(self._make_float32_msg(closest_dist))
            self.cn_pub.publish(self._make_string_msg(cnstr))
            self.tag_pub.publish(self._make_string_msg(nodetag))
            _pub_edges(closest_edge_ids, closest_edge_dists)

        self.wpstr = wpstr
        self.closest_dist = closest_dist
        self.cnstr = cnstr
        self.nodetag = nodetag
        self.closest_edge_ids = closest_edge_ids
        self.closest_edge_dists = closest_edge_dists

        self.get_logger().info(
            f"Published: closest_node='{wpstr}', closest_dist={closest_dist}, "
            f"current_node='{cnstr}', nodetag='{nodetag}', "
            f"closest_edges={closest_edge_ids} (dists: {closest_edge_dists})"
        )

    # -----------------------------------------------------------------
    # Map reception
    # -----------------------------------------------------------------

    def _map_callback(self, msg):
        """Handle incoming topological map - build graph and KD-tree."""

        self.names_by_topic: list = []
        self.nogos: list = []

        self.tmap = yaml.load(msg.data, Loader=CustomSafeLoader)
        self.tmap_frame = self.tmap["transformation"]["topological_frame_id"]
        self.get_logger().info("Received the topological map")

        # Build NetworkX graph
        self._graph = build_graph_from_tmap(self.tmap, logger=self.get_logger())
        if self._graph is None:
            self.get_logger().error("Failed to build the NetworkX graph – aborting map load")
            return
        self.get_logger().info(
            f"Graph built: {self._graph.number_of_nodes()} nodes, "
            f"{self._graph.number_of_edges()} edges"
        )

        # Build KD-tree
        self._kdtree, self._kdtree_node_names = build_kdtree_from_graph(
            self._graph, logger=self.get_logger(),
        )
        if self._kdtree is None:
            self.get_logger().error("Failed to build KD-tree – aborting map load")
            return
        self.get_logger().info(
            f"KD-tree built with {len(self._kdtree_node_names)} nodes"
        )

        # Topic-based localisation config
        _, self.names_by_topic = update_loc_by_topic_nx(
            self._graph, logger=self.get_logger(),
        )

        self.rec_map = True

    # -----------------------------------------------------------------
    # Localise-pose service
    # -----------------------------------------------------------------

    def localise_pose_cb(self, req, res):
        """Service callback: localise a given pose in the topological map."""
        if self._graph is None or self._kdtree is None:
            self.get_logger().warning(
                "localise_pose service called before map is ready"
            )
            res.current_node = 'none'
            res.closest_node = 'none'
            return res

        currentstr = determine_current_node(
            self._graph, self._kdtree, self._kdtree_node_names,
            req.pose, [], self.nogos,  # no topic-based loc for one-shot queries
        )
        closeststr, _ = determine_closest_node(
            self._kdtree, self._kdtree_node_names, self._graph,
            currentstr, self.nogos, self.names_by_topic, req.pose,
        )

        res.current_node = currentstr
        res.closest_node = closeststr
        return res

    # -----------------------------------------------------------------
    # No-go nodes
    # -----------------------------------------------------------------

    def _get_no_go_nodes(self) -> list:
        """Query the map manager for 'no-go' tagged nodes.

        Returns an empty list when the GetTaggedNodes service type is
        unavailable (e.g. not defined in topological_navigation_msgs).
        """
        if not _HAS_GET_TAGGED_NODES:
            self.get_logger().info(
                "GetTaggedNodes service not available in this build; "
                "no-go nodes disabled"
            )
            return []

        cli = self.create_client(
            GetTaggedNodes,
            '/topological_map_manager2/get_tagged_nodes',
        )
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning(
                "Service /topological_map_manager2/get_tagged_nodes unavailable; "
                "assuming no no-go nodes"
            )
            return []

        future = cli.call_async(GetTaggedNodes.Request())
        rclpy.spin_until_future_complete(self, future)
        return list(future.result().nodes)


# =====================================================================
# Entry point
# =====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = TopologicalNavLoc('topological_localisation', with_tags=True)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down localisation node")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


