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
    point_in_poly_nx,
    query_nearest_nodes,
    update_loc_by_topic_nx,
)
from topological_navigation.tmap_utils import get_distance, get_node_from_tmap2
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
            Parameter('int', Parameter.Type.INTEGER, 3),
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
        self.current_closest_node_name = ""

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
        self.force_check = True
        self.rec_map = False
        self.set_nogos = False
        self.loc_by_topic: list = []
        self.persist: dict = {}

        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.previous_pose.position.x = 1000.0  # large initial value to trigger first check

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

    # -----------------------------------------------------------------
    # Distance helpers
    # -----------------------------------------------------------------

    def get_distances_to_pose(self, pose: Pose) -> list:
        """Return distance from every waypoint to *pose*, sorted nearest-first.

        Uses the KD-tree for O(log n) spatial queries.

        Returns:
            List of dicts ``{'node': <node_dict>, 'dist': <float>}``.
        """
        if self._kdtree is None or not self._kdtree_node_names:
            return []

        k = len(self._kdtree_node_names)
        nearest_nodes = query_nearest_nodes(
            self._kdtree, self._kdtree_node_names, pose, k=k,
        )

        distances = []
        for info in nearest_nodes:
            node_data = get_node_from_tmap2(self.tmap, info['node'])
            if node_data:
                distances.append({'node': node_data, 'dist': info['dist']})
        return distances

    def get_edge_distances_to_pose(self, pose: Pose):
        """Return edge-ID list and distance array for all edges relative to *pose*.

        Uses the NetworkX graph for vectorised edge distance calculations.

        Returns:
            Tuple ``(edge_ids, distances)`` – both may be empty if the graph
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

        # Closest edges
        closest_edges, edge_dists = self.get_edge_distances_to_pose(msg)
        if len(closest_edges) > 1:
            closest_edges = closest_edges[:2]
            edge_dists = edge_dists[:2]

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

        # Update force_check flag
        if currentstr != 'none':
            self.current_closest_node_name = currentstr
            self.force_check = False
        else:
            self.force_check = True

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
        """Return the first tag string for *node_name*, or ``'Unknown'``."""
        node = get_node_from_tmap2(self.tmap, node_name)
        if node is None:
            self.get_logger().warning(
                f"Node '{node_name}' not found in the topological map"
            )
            return 'Unknown'
        try:
            return node['meta']['tag'][0]
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
        


# -----------------------------------------------------------------
    # Map reception
    # -----------------------------------------------------------------

    def _map_callback(self, msg):
        """Handle incoming topological map – build graph and KD-tree."""
        if self.rec_map:
            return  # already processed

        self.names_by_topic: list = []
        self.nodes_by_topic: list = []
        self.nogos: list = []

        self.tmap = yaml.load(msg.data, Loader=CustomSafeLoader)
        self.tmap_frame = self.tmap["transformation"]["child"]
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
        self.nodes_by_topic, self.names_by_topic = update_loc_by_topic_nx(
            self._graph, logger=self.get_logger(),
        )

        # Edge vectors (kept for backward compatibility)
        self._build_edge_vectors()

        self.rec_map = True

    # -----------------------------------------------------------------
    # Edge vectors  (backward-compatible helper)
    # -----------------------------------------------------------------

    def _build_edge_vectors(self):
        """Pre-compute start/end vectors for every edge (used by legacy code)."""
        node_poses = {
            n["node"]["name"]: n["node"]["pose"] for n in self.tmap["nodes"]
        }
        self.dist_edge_ids: list = []
        vectors_start: list = []
        vectors_end: list = []

        for node in self.tmap["nodes"]:
            name = node["node"]["name"]
            orig = node_poses[name]
            start = [orig["position"]["x"], orig["position"]["y"], 0]

            for edge in node["node"]["edges"]:
                if name == edge["node"]:
                    self.get_logger().error(
                        f"Self-referencing edge '{edge['edge_id']}' on node '{name}'"
                    )
                    continue
                dest = node_poses[edge["node"]]
                self.dist_edge_ids.append(edge["edge_id"])
                vectors_start.append(start)
                vectors_end.append([dest["position"]["x"], dest["position"]["y"], 0])

        self.vectors_start = np.array(vectors_start)
        self.vectors_end = np.array(vectors_end)

    # -----------------------------------------------------------------
    # Topic-based localisation callback
    # -----------------------------------------------------------------

    def topic_localise_callback(self, msg, item):
        """Update ``loc_by_topic`` when a subscribed topic fires.

        Only re-evaluates when the robot has moved more than 0.10 m since
        the last detection.
        """
        if self.force_check:
            dist = 1.0
        else:
            dist = get_distance(self.current_pose, self.previous_pose)

        if dist <= 0.10:
            return

        val = getattr(msg, item['field'])

        if val == item['val']:
            if item['name'] in self.persist:
                if self.persist[item['name']] < item['persistency']:
                    self.persist[item['name']] += 1
            else:
                self.persist[item['name']] = 0

            active_names = [x['name'] for x in self.loc_by_topic]
            if (item['name'] not in active_names
                    and self.persist[item['name']] < item['persistency']):
                self.loc_by_topic.append(item)
                self.previous_pose = self.current_pose
        else:
            self.persist.pop(item['name'], None)
            active = [x for x in self.loc_by_topic if x['name'] == item['name']]
            for entry in active:
                self.loc_by_topic.remove(entry)
            if active:
                self.previous_pose = self.current_pose

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

    # -----------------------------------------------------------------
    # Point-in-polygon (backward-compatible wrapper)
    # -----------------------------------------------------------------

    def point_in_poly(self, node, pose) -> bool:
        """Check whether *pose* lies inside *node*'s influence zone.

        Args:
            node: Node data dictionary from the tmap (``{'node': {'name': ...}}``)
            pose: ``geometry_msgs.msg.Pose``

        Returns:
            ``True`` if inside the influence zone, ``False`` otherwise.
        """
        if isinstance(node, dict) and 'node' in node and 'name' in node['node']:
            node_name = node['node']['name']
        else:
            return False

        if self._graph is None:
            return False

        return point_in_poly_nx(self._graph, node_name, pose)


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


