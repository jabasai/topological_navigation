#!/usr/bin/env python
"""Topological Navigation Server -- Self-Contained Implementation.

Reimplements the topological navigation server using:
- **NetworkX** graph data structures for route planning (A*)
- **Explicit state machine** for predictable navigation behaviour
- **Action merging** of consecutive same-type edges into segments
- **Multi-pose segment execution** via NavigateThroughPoses
- **Intermediate orientation ignored** (identity quaternion for drive-through)
- **Boundary polygon publishing** for row_traversal corridors
- **Built-in Nav2 action execution** -- no external EdgeActionManager

Edge actions (read from the topomap, both naming conventions accepted):
    NavigateToPose / navigate_to_pose  -- standard Nav2 point-to-point
    GoalAlign      / goal_align        -- precision alignment at goal pose
    RowTraversal   / row_traversal     -- agricultural row navigation

ROS 2 interfaces:
    Action servers:
        /<node_name>                                     (GotoNode)
        /topological_navigation/execute_policy_mode      (ExecutePolicyMode)
    Subscriptions:
        /topological_map_2, closest_node, closest_edges, current_node
    Publishers:
        topological_navigation/Statistics, topological_navigation/Route,
        current_edge, /boundary_checker (PolygonStamped),
        /robot_operation_current_status (String),
        topological_navigation/move_action_status (String)

Last Updated: 2026-02-21
"""

import json
import os

import yaml

import rclpy
import rclpy.node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy import Parameter
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

from topological_navigation.edge_reconfigure_manager2 import (
    EdgeReconfigureManager,
)
from topological_navigation.navigation_graph import (
    ACTION_TO_STATE,
    NavState,
    NavStateMachine,
    compute_row_boundary_polygon,
    get_route_distance,
    get_route_edges,
    merge_action_segments,
    normalize_action_name,
    plan_route,
)
from topological_navigation.navigation_stats import nav_stats
from topological_navigation.networkx_utils import build_graph_from_tmap
from topological_navigation.tmap_utils import (
    get_edge_from_id_tmap2,
    get_node_from_tmap2,
)
from topological_navigation_msgs.action import ExecutePolicyMode, GotoNode
from topological_navigation_msgs.msg import (
    ClosestEdges,
    ExecutePolicyModeFeedback,
    GotoNodeFeedback,
    NavStatistics,
    TopologicalRoute,
)
from topological_navigation_msgs.srv import EvaluateEdge, EvaluateNode


# =====================================================================
# Status mapping (GoalStatus int -> human-readable string)
# =====================================================================

_STATUS_MAP = {
    GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
    GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
    GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
}


def _status_str(code: int) -> str:
    """Convert a GoalStatus integer to a readable string."""
    return _STATUS_MAP.get(code, "STATUS_UNKNOWN(%d)" % code)


# =====================================================================
# YAML Loader
# =====================================================================


class _FloatSafeLoader(yaml.SafeLoader):
    """YAML loader that coerces pose coordinate values to ``float``."""

    def construct_mapping(self, node, deep=False):
        mapping = super().construct_mapping(node, deep=deep)
        for key in ('x', 'y', 'z', 'w'):
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])
        return mapping


# =====================================================================
# Main Navigation Server
# =====================================================================


class TopologicalNavServer(rclpy.node.Node):
    """Self-contained topological navigation server.

    Owns its Nav2 ActionClient internally -- no external
    EdgeActionManager node required.
    """

    # -----------------------------------------------------------------
    # Construction
    # -----------------------------------------------------------------

    def __init__(self, name):
        super().__init__(name)
        rclpy.get_default_context().on_shutdown(self._on_node_shutdown)

        # -- State machine -----------------------------------------------
        self._sm = NavStateMachine(logger=self.get_logger())
        self._sm.transition(NavState.WAITING_FOR_MAP)

        # -- Navigation runtime state ------------------------------------
        self._cancelled = False
        self._preempted = False
        self._goal_reached = False
        self._navigation_activated = False
        self._no_orientation = False
        self._target = "none"
        self._current_target = "none"

        # -- Map data ----------------------------------------------------
        self._tmap = None
        self._graph = None
        self._topol_map = ""

        # -- Localisation ------------------------------------------------
        self._current_node = "Unknown"
        self._closest_node = "Unknown"
        self._closest_edges = ClosestEdges()

        # -- Nav2 action client ------------------------------------------
        self._nav2_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav2_executor = SingleThreadedExecutor()
        self._nav2_client = ActionClient(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
            callback_group=self._nav2_cb_group,
        )
        self._goal_handle = None
        self._action_status = GoalStatus.STATUS_UNKNOWN

        # -- Stats -------------------------------------------------------
        self._stat = None

        # -- Parameters --------------------------------------------------
        self._declare_parameters()
        self._load_parameters()
        self._load_bt_trees()

        # -- QoS profiles ------------------------------------------------
        self._latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._best_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Publishers --------------------------------------------------
        self._stats_pub = self.create_publisher(
            NavStatistics,
            "topological_navigation/Statistics",
            qos_profile=self._latch_qos,
        )
        self._route_pub = self.create_publisher(
            TopologicalRoute,
            "topological_navigation/Route",
            qos_profile=self._best_qos,
        )
        self._route_timer = self.create_timer(
            2.0, self._route_pub_timer_cb,
        )
        self._stroute = None
        self._cur_edge_pub = self.create_publisher(
            String, "current_edge", qos_profile=self._latch_qos,
        )
        self._move_status_pub = self.create_publisher(
            String,
            "topological_navigation/move_action_status",
            qos_profile=self._latch_qos,
        )
        self._boundary_pub = self.create_publisher(
            PolygonStamped,
            "/boundary_checker",
            qos_profile=self._latch_qos,
        )
        self._status_pub = self.create_publisher(
            String,
            "/robot_operation_current_status",
            qos_profile=self._latch_qos,
        )

        # -- Map subscription (blocking wait) ----------------------------
        self._map_received = False
        self._loc_received = False
        self._cb_group_map = ReentrantCallbackGroup()
        self.create_subscription(
            String,
            '/topological_map_2',
            self._map_callback,
            callback_group=self._cb_group_map,
            qos_profile=self._latch_qos,
        )

        self.get_logger().info(
            "[INIT] Waiting for topological map on /topological_map_2 ...",
        )
        self._publish_status("WAITING_FOR_MAP")

        while rclpy.ok() and not self._map_received:
            rclpy.spin_once(self)

        self.get_logger().info(
            "[INIT] Map received: '%s' -- %d nodes, %d edges"
            % (
                self._topol_map,
                self._graph.number_of_nodes(),
                self._graph.number_of_edges(),
            ),
        )

        # -- Edge reconfigure --------------------------------------------
        self._edge_reconfigure_enabled = self.get_parameter_or(
            "reconfigure_edges",
            Parameter('b', Parameter.Type.BOOL, True),
        ).value
        self._srv_edge_reconfigure = self.get_parameter_or(
            "reconfigure_edges_srv",
            Parameter('b', Parameter.Type.BOOL, False),
        ).value
        if self._edge_reconfigure_enabled:
            self._edge_reconf_mgr = EdgeReconfigureManager()
        else:
            self._edge_reconf_mgr = None
            self.get_logger().warning(
                "[INIT] Edge reconfiguration is disabled",
            )

        # -- Localisation subscription (blocking wait) -------------------
        self._sm.transition(NavState.WAITING_FOR_LOCALISATION)
        self._publish_status("WAITING_FOR_LOCALISATION")

        self.create_subscription(
            String,
            'closest_node',
            self._closest_node_cb,
            qos_profile=self._latch_qos,
        )

        self.get_logger().info(
            "[INIT] Waiting for localisation on 'closest_node' ...",
        )
        while rclpy.ok() and not self._loc_received:
            rclpy.spin_once(self)

        self.get_logger().info(
            "[INIT] Localisation received. Closest node: %s"
            % self._closest_node,
        )

        self.create_subscription(
            ClosestEdges,
            'closest_edges',
            self._closest_edges_cb,
            qos_profile=self._latch_qos,
        )
        self.create_subscription(
            String,
            'current_node',
            self._current_node_cb,
            qos_profile=self._latch_qos,
        )

        # -- Restrictions (optional, non-blocking) -----------------------
        self._using_restrictions = False
        try:
            self._eval_edge_srv = self.create_client(
                EvaluateEdge, '/restrictions_manager/evaluate_edge',
            )
            if self._eval_edge_srv.wait_for_service(timeout_sec=3.0):
                self._eval_node_srv = self.create_client(
                    EvaluateNode, '/restrictions_manager/evaluate_node',
                )
                self._using_restrictions = True
                self.get_logger().info(
                    "[INIT] Restrictions services available",
                )
            else:
                self.get_logger().warning(
                    "[INIT] Restrictions services unavailable (timeout)",
                )
        except Exception as exc:
            self.get_logger().error(
                "[INIT] Error probing restrictions services: %s" % exc,
            )

        # -- Fail-policy runtime state -----------------------------------
        self._fail_policy_state = {}

        # -- Action servers ----------------------------------------------
        self._sm.transition(NavState.READY)
        self._publish_status("READY")

        cb_goto = ReentrantCallbackGroup()
        cb_policy = ReentrantCallbackGroup()

        self.get_logger().info("[INIT] Creating GotoNode action server ...")
        self._as_goto = ActionServer(
            self,
            GotoNode,
            "/" + name,
            execute_callback=self._execute_goto_cb,
            cancel_callback=self._cancel_goto_cb,
            callback_group=cb_goto,
        )
        self._goto_fb_pub = self.create_publisher(
            GotoNodeFeedback,
            "/" + name + "/feedback",
            qos_profile=self._latch_qos,
        )

        self.get_logger().info(
            "[INIT] Creating ExecutePolicyMode action server ...",
        )
        self._as_policy = ActionServer(
            self,
            ExecutePolicyMode,
            "/topological_navigation/execute_policy_mode",
            execute_callback=self._execute_policy_cb,
            cancel_callback=self._cancel_policy_cb,
            callback_group=cb_policy,
        )
        self._policy_fb_pub = self.create_publisher(
            ExecutePolicyModeFeedback,
            "topological_navigation/execute_policy_mode/feedback",
            qos_profile=self._latch_qos,
        )

        self.get_logger().info(
            "[INIT] Topological navigation server READY. "
            "Map='%s', Nodes=%d, Edges=%d"
            % (
                self._topol_map,
                self._graph.number_of_nodes(),
                self._graph.number_of_edges(),
            ),
        )

    # =================================================================
    # Parameters
    # =================================================================

    def _declare_parameters(self):
        """Declare all ROS 2 parameters."""
        for name, ptype in [
            ('max_dist_to_closest_edge', Parameter.Type.DOUBLE),
            ('reconfigure_edges', Parameter.Type.BOOL),
            ('reconfigure_edges_srv', Parameter.Type.BOOL),
            ('default_boundary_left', Parameter.Type.DOUBLE),
            ('default_boundary_right', Parameter.Type.DOUBLE),
            ('bt_tree_default', Parameter.Type.STRING),
            ('bt_tree_in_row', Parameter.Type.STRING),
            ('bt_tree_goal_align', Parameter.Type.STRING),
        ]:
            self.declare_parameter(name, ptype)

    def _load_parameters(self):
        """Read parameter values with defaults."""

        def _p(name, ptype, default):
            return self.get_parameter_or(
                name, Parameter('_', ptype, default),
            ).value

        self._max_dist_to_closest_edge = _p(
            'max_dist_to_closest_edge', Parameter.Type.DOUBLE, 1.0,
        )
        self._default_boundary_left = _p(
            'default_boundary_left', Parameter.Type.DOUBLE, 0.5,
        )
        self._default_boundary_right = _p(
            'default_boundary_right', Parameter.Type.DOUBLE, 0.5,
        )

    def _load_bt_trees(self):
        """Resolve behaviour-tree XML paths for each action type."""
        cfg = os.path.join(
            get_package_share_directory('topological_navigation'),
            'config',
        )

        bt_map = {
            'NavigateToPose': ('bt_tree_default.xml', 'bt_tree_default'),
            'row_traversal': ('bt_tree_in_row.xml', 'bt_tree_in_row'),
            'goal_align': ('bt_tree_goal_align.xml', 'bt_tree_goal_align'),
        }
        self._bt_trees = {}
        for action, (fname, param) in bt_map.items():
            default = os.path.join(cfg, fname)
            self._bt_trees[action] = self.get_parameter_or(
                param, Parameter('s', Parameter.Type.STRING, default),
            ).value
            self.get_logger().info(
                "[INIT] BT '%s': %s" % (action, self._bt_trees[action]),
            )

    # =================================================================
    # Lifecycle
    # =================================================================

    def _on_node_shutdown(self):
        """Graceful shutdown: cancel any active goal."""
        self.get_logger().info("[SHUTDOWN] Tearing down navigation server")
        if self._navigation_activated:
            self._preempted = True
            self._cancel_nav2_goal(timeout_sec=2.0)

    # =================================================================
    # Topic callbacks
    # =================================================================

    def _map_callback(self, msg):
        """Handle ``/topological_map_2`` updates -- rebuild graph."""
        try:
            self._tmap = yaml.load(msg.data, Loader=_FloatSafeLoader)
            self._topol_map = self._tmap.get("pointset", "unknown")

            self._graph = build_graph_from_tmap(
                self._tmap, logger=self.get_logger(),
            )
            if self._graph is None:
                self.get_logger().error(
                    "[MAP] Failed to build graph from topomap",
                )
                return

            self._map_received = True

            self.get_logger().info(
                "[MAP] Updated: '%s' -- %d nodes, %d edges"
                % (
                    self._topol_map,
                    self._graph.number_of_nodes(),
                    self._graph.number_of_edges(),
                ),
            )
        except Exception as exc:
            self.get_logger().error(
                "[MAP] Error processing topological map: %s" % exc,
            )

    def _closest_node_cb(self, msg):
        self._loc_received = True
        self._closest_node = msg.data

    def _closest_edges_cb(self, msg):
        self._closest_edges = msg

    def _current_node_cb(self, msg):
        if self._current_node != msg.data:
            self._current_node = msg.data
            if msg.data != "none":
                self.get_logger().info(
                    "[LOC] Entered node: %s" % self._current_node,
                )
                if (
                    self._navigation_activated
                    and self._current_node == self._current_target
                ):
                    self.get_logger().info(
                        "[LOC] Reached intermediate target: %s"
                        % self._current_target,
                    )
                    self._goal_reached = True

    def _route_pub_timer_cb(self):
        if self._stroute and self._stroute.nodes:
            self._route_pub.publish(self._stroute)

    # =================================================================
    # Publishers
    # =================================================================

    def _publish_status(self, state_str):
        """Publish current state on ``/robot_operation_current_status``."""
        msg = String()
        msg.data = state_str
        self._status_pub.publish(msg)

    def _publish_boundary(self, polygon_pts, frame_id="map"):
        """Publish ``PolygonStamped`` on ``/boundary_checker``."""
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        for x, y in polygon_pts:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)
        self._boundary_pub.publish(msg)
        self.get_logger().info(
            "[BOUNDARY] Published %d-point polygon (frame=%s)"
            % (len(polygon_pts), frame_id),
        )

    def _publish_empty_boundary(self, frame_id="map"):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self._boundary_pub.publish(msg)

    def _publish_route(self, route_nodes):
        stroute = TopologicalRoute()
        for n in route_nodes:
            stroute.nodes.append(n)
        self._stroute = stroute
        self._route_pub.publish(stroute)

    def _publish_stats(self):
        if self._stat is None:
            return
        s = self._stat
        pubst = NavStatistics()
        pubst.edge_id = s.edge_id
        pubst.status = s.status
        pubst.origin = s.origin
        pubst.target = s.target
        pubst.topological_map = s.topological_map
        pubst.time_to_waypoint = float(s.time_to_wp)
        pubst.operation_time = s.operation_time
        pubst.date_started = s.get_start_time_str()
        pubst.date_at_node = s.date_at_node.strftime(
            "%A, %B %d %Y, at %H:%M:%S hours",
        )
        pubst.date_finished = s.get_finish_time_str()
        self._stats_pub.publish(pubst)

    def _publish_current_edge(self, edge_id):
        msg = String()
        msg.data = (
            "%s--%s" % (edge_id, self._topol_map)
            if edge_id != "none"
            else "none"
        )
        self._cur_edge_pub.publish(msg)

    def _publish_move_status(self, goal_node, action, status_str):
        d = {
            "goal": goal_node,
            "final_goal": self._target,
            "action": action.upper(),
            "status": status_str,
        }
        msg = String()
        msg.data = json.dumps(d)
        self._move_status_pub.publish(msg)

    # =================================================================
    # Nav2 Goal construction and execution
    # =================================================================

    def _build_pose_stamped(self, node_dict, ignore_orientation=False):
        """Build ``PoseStamped`` from a topomap node dict.

        Args:
            node_dict: A node entry from the topomap YAML,
                e.g. ``get_node_from_tmap2(tmap, name)``.
            ignore_orientation: If True, set identity quaternion
                (0,0,0,1) so Nav2 does not enforce orientation at
                this waypoint.

        Returns:
            ``PoseStamped`` with the node's pose and parent frame.
        """
        nd = node_dict["node"]
        pose = nd["pose"]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = nd.get("parent_frame", "map")
        ps.pose.position.x = float(pose["position"]["x"])
        ps.pose.position.y = float(pose["position"]["y"])
        ps.pose.position.z = float(pose["position"]["z"])
        if ignore_orientation:
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
        else:
            ps.pose.orientation.x = float(pose["orientation"]["x"])
            ps.pose.orientation.y = float(pose["orientation"]["y"])
            ps.pose.orientation.z = float(pose["orientation"]["z"])
            ps.pose.orientation.w = float(pose["orientation"]["w"])
        return ps

    def _build_pose_from_graph(self, node_name, ignore_orientation=False):
        """Build ``PoseStamped`` from the NetworkX graph node attributes.

        Uses the graph's node attributes directly (x, y, z,
        orientation, parent_frame) rather than looking up from the
        raw YAML dict.

        Args:
            node_name: Name of the node in the graph.
            ignore_orientation: If True, set identity quaternion.

        Returns:
            ``PoseStamped`` or None if node not in graph.
        """
        if node_name not in self._graph:
            return None
        attrs = self._graph.nodes[node_name]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = attrs.get("parent_frame", "map")
        ps.pose.position.x = float(attrs.get("x", 0.0))
        ps.pose.position.y = float(attrs.get("y", 0.0))
        ps.pose.position.z = float(attrs.get("z", 0.0))
        if ignore_orientation:
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
        else:
            ori = attrs.get(
                "orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            )
            ps.pose.orientation.x = float(ori.get("x", 0.0))
            ps.pose.orientation.y = float(ori.get("y", 0.0))
            ps.pose.orientation.z = float(ori.get("z", 0.0))
            ps.pose.orientation.w = float(ori.get("w", 1.0))
        return ps

    def _build_segment_goal(self, segment, is_final_segment):
        """Build a ``NavigateThroughPoses.Goal`` for an entire segment.

        Collects all waypoints (target nodes) in the segment and sends
        them as a single multi-pose goal to Nav2. Intermediate
        waypoints use identity orientation so Nav2 does not enforce
        heading at mid-route nodes; only the final waypoint keeps
        the real orientation from the topological map.

        Args:
            segment: An ``ActionSegment`` containing one or more edges.
            is_final_segment: True if this is the last segment in the
                route (affects orientation of the final waypoint).

        Returns:
            ``NavigateThroughPoses.Goal``
        """
        action = segment.action_type
        goal = NavigateThroughPoses.Goal()

        n_edges = segment.num_edges
        for ei, edata in enumerate(segment.edge_data):
            tgt = edata['target']
            is_last = (ei == n_edges - 1)

            # Intermediate waypoints: ignore orientation so the robot
            # drives through without stopping to rotate.
            # Final waypoint: keep orientation unless no_orientation
            # was requested.
            ignore_ori = not is_last or (
                is_last and self._no_orientation and is_final_segment
            )

            ps = self._build_pose_from_graph(tgt, ignore_orientation=ignore_ori)
            if ps is None:
                # Fallback to YAML lookup
                dest_node = get_node_from_tmap2(self._tmap, tgt)
                if dest_node:
                    ps = self._build_pose_stamped(
                        dest_node, ignore_orientation=ignore_ori,
                    )
            if ps is not None:
                goal.poses.append(ps)

        # Select BT tree for this action type
        bt = self._bt_trees.get(action) or self._bt_trees.get(
            normalize_action_name(action), '',
        )
        if bt:
            goal.behavior_tree = bt

        self.get_logger().info(
            "[GOAL] %s segment: %d poses, BT=%s"
            % (action, len(goal.poses), bt or "default"),
        )
        return goal

    def _send_nav2_goal(self, goal):
        """Send goal to Nav2 and block until result.

        Returns:
            ``GoalStatus`` integer.
        """
        if not self._nav2_client.server_is_ready():
            self.get_logger().info(
                "[NAV2] Waiting for /navigate_through_poses server ...",
            )
            if not self._nav2_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(
                    "[NAV2] Action server not available",
                )
                return GoalStatus.STATUS_ABORTED

        # Send goal
        send_future = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback_cb,
        )

        # Wait for acceptance
        while rclpy.ok():
            try:
                rclpy.spin_once(
                    self, executor=self._nav2_executor, timeout_sec=0.5,
                )
                if send_future.done():
                    break
            except Exception as exc:
                self.get_logger().error(
                    "[NAV2] send_goal_async error: %s" % exc,
                )
                return GoalStatus.STATUS_ABORTED

        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("[NAV2] Goal REJECTED by server")
            return GoalStatus.STATUS_ABORTED

        self.get_logger().info("[NAV2] Goal ACCEPTED")

        # Wait for result
        result_future = self._goal_handle.get_result_async()

        while rclpy.ok():
            if self._preempted or self._cancelled:
                self._cancel_nav2_goal(timeout_sec=2.0)
                return GoalStatus.STATUS_CANCELED
            try:
                rclpy.spin_once(
                    self, executor=self._nav2_executor, timeout_sec=1.0,
                )
                if result_future.done():
                    result = result_future.result()
                    self._action_status = result.status
                    self.get_logger().info(
                        "[NAV2] Goal finished: %s"
                        % _status_str(result.status),
                    )
                    return result.status
            except Exception as exc:
                self.get_logger().error(
                    "[NAV2] processing error: %s" % exc,
                )
                return GoalStatus.STATUS_ABORTED

        return GoalStatus.STATUS_ABORTED

    def _nav2_feedback_cb(self, feedback_msg):
        """Handle Nav2 feedback (currently just updates status)."""
        self._action_status = GoalStatus.STATUS_EXECUTING

    def _cancel_nav2_goal(self, timeout_sec=2.0):
        """Cancel the current Nav2 goal if one is active."""
        if self._goal_handle is None:
            return
        try:
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(
                self, cancel_future, timeout_sec=timeout_sec,
            )
            self.get_logger().info("[NAV2] Goal cancel sent")
        except Exception as exc:
            self.get_logger().error(
                "[NAV2] Error cancelling goal: %s" % exc,
            )
        finally:
            self._goal_handle = None

    # =================================================================
    # Action-server callbacks
    # =================================================================

    def _execute_goto_cb(self, goal_handle):
        """``GotoNode`` action callback -- plan and follow route."""
        target = goal_handle.request.target
        self.get_logger().info(
            "=" * 70
            + "\n[GOTO] target='%s', no_orientation=%s"
            % (target, goal_handle.request.no_orientation),
        )

        self._cancel_current_navigation()

        self._navigation_activated = True
        self._cancelled = False
        self._preempted = False
        self._no_orientation = goal_handle.request.no_orientation
        self._fail_policy_state = {}

        fb = GotoNodeFeedback()
        fb.route = "Planning..."
        self._goto_fb_pub.publish(fb)

        success = self._navigate(target)

        self._navigation_activated = False
        result = GotoNode.Result()
        result.success = success

        if success:
            goal_handle.succeed()
            self.get_logger().info(
                "[GOTO] Navigation to '%s' SUCCEEDED" % target,
            )
        else:
            goal_handle.abort()
            self.get_logger().warning(
                "[GOTO] Navigation to '%s' %s"
                % (
                    target,
                    "CANCELLED" if self._preempted else "FAILED",
                ),
            )

        if self._sm.is_terminal():
            self._sm.reset()

        return result

    def _execute_policy_cb(self, goal_handle):
        """``ExecutePolicyMode`` action -- follow provided route."""
        self.get_logger().info(
            "=" * 70
            + "\n[POLICY] Execute-policy goal received",
        )

        self._cancel_current_navigation()

        self._navigation_activated = True
        self._cancelled = False
        self._preempted = False
        self._fail_policy_state = {}

        route = goal_handle.request.route

        # Validate
        if (
            len(route.source) < 1
            or len(route.source) != len(route.edge_id)
        ):
            self.get_logger().error(
                "[POLICY] Invalid route: source/edge_id mismatch "
                "(%d vs %d)" % (len(route.source), len(route.edge_id)),
            )
            self._navigation_activated = False
            goal_handle.succeed()
            return ExecutePolicyMode.Result(success=False)

        if (
            route.source[0] != self._current_node
            and route.source[0] != self._closest_node
        ):
            self.get_logger().error(
                "[POLICY] Route starts at '%s' but robot at '%s' "
                "(closest: '%s')"
                % (
                    route.source[0],
                    self._current_node,
                    self._closest_node,
                ),
            )
            self._navigation_activated = False
            goal_handle.succeed()
            return ExecutePolicyMode.Result(success=False)

        # Build route-node list
        route_nodes = list(route.source)
        if route.edge_id:
            last_edge = get_edge_from_id_tmap2(
                self._tmap, route.source[-1], route.edge_id[-1],
            )
            if last_edge:
                final_target = last_edge["node"]
                if not route_nodes or route_nodes[-1] != final_target:
                    route_nodes.append(final_target)

        target = route_nodes[-1]
        self.get_logger().info(
            "[POLICY] Route: %s" % " -> ".join(route_nodes),
        )
        self._publish_route(route_nodes)

        success = self._execute_route(route_nodes, target)

        self._navigation_activated = False
        if self._sm.is_terminal():
            self._sm.reset()

        goal_handle.succeed()
        self.get_logger().info(
            "[POLICY] %s" % ("SUCCEEDED" if success else "FAILED"),
        )
        return ExecutePolicyMode.Result(success=success)

    def _cancel_goto_cb(self, goal_handle):
        self.get_logger().warning("[GOTO] Cancel requested")
        self._preempted = True
        self._cancel_nav2_goal(timeout_sec=2.0)

    def _cancel_policy_cb(self, goal_handle):
        self.get_logger().warning("[POLICY] Cancel requested")
        self._preempted = True
        self._cancel_nav2_goal(timeout_sec=2.0)

    # =================================================================
    # Core navigation logic
    # =================================================================

    def _navigate(self, target):
        """Plan route from current position and execute.

        Returns ``True`` on success.
        """
        if self._cancelled:
            return False

        self._sm.transition(NavState.PLANNING)
        self._publish_status("PLANNING")
        self._target = target

        if target not in self._graph:
            self.get_logger().error(
                "[NAV] Target '%s' not in map" % target,
            )
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        origin = self._determine_origin(target)
        if origin is None:
            self.get_logger().error("[NAV] Cannot determine origin node")
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        self.get_logger().info(
            "[NAV] Route planning: '%s' -> '%s'" % (origin, target),
        )

        if origin == target:
            self.get_logger().info("[NAV] Already at target node")
            self._sm.transition(NavState.SUCCEEDED)
            self._publish_status("SUCCEEDED")
            return True

        route_nodes = plan_route(
            self._graph, origin, target, logger=self.get_logger(),
        )
        if not route_nodes or len(route_nodes) < 2:
            self.get_logger().error(
                "[NAV] No route from '%s' to '%s'" % (origin, target),
            )
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")
            return False

        self.get_logger().info(
            "[NAV] Route: %s (%d nodes)"
            % (" -> ".join(route_nodes), len(route_nodes)),
        )
        self._publish_route(route_nodes)

        success = self._execute_route(route_nodes, target)

        if not success and not self._cancelled and not self._preempted:
            self._sm.transition(NavState.FAILED)
            self._publish_status("FAILED")

        return success

    def _determine_origin(self, target):
        """Determine best origin node.

        Priority: current_node > closest-edge endpoint > closest_node.
        """
        if self._current_node not in ("none", "Unknown"):
            self.get_logger().info(
                "[NAV] Origin: current node '%s'" % self._current_node,
            )
            return self._current_node

        if (
            self._closest_edges.distances
            and self._closest_edges.distances[0]
            <= self._max_dist_to_closest_edge
        ):
            eids = self._closest_edges.edge_ids
            origin = self._origin_from_closest_edge(
                target,
                eids[0] if eids else None,
                eids[1] if len(eids) > 1 else None,
            )
            if origin:
                self.get_logger().info(
                    "[NAV] Origin: closest-edge endpoint '%s'" % origin,
                )
                return origin

        if self._closest_node != "Unknown":
            self.get_logger().info(
                "[NAV] Origin: closest node '%s'" % self._closest_node,
            )
            return self._closest_node

        return None

    def _origin_from_closest_edge(self, target, eid1, eid2):
        """Pick origin from closest edge(s)."""

        def _src_of(eid):
            if not eid:
                return None
            for u, _v, d in self._graph.edges(data=True):
                if d.get('edge_id') == eid:
                    return u
            return None

        src1 = _src_of(eid1)
        if src1 is None:
            return self._closest_node

        if eid2 and len(self._closest_edges.distances) > 1:
            src2 = _src_of(eid2)
            if (
                src2
                and self._closest_edges.distances[0]
                == self._closest_edges.distances[1]
            ):
                r1 = plan_route(self._graph, src1, target)
                r2 = plan_route(self._graph, src2, target)
                d1 = (
                    get_route_distance(self._graph, r1)
                    if r1
                    else float('inf')
                )
                d2 = (
                    get_route_distance(self._graph, r2)
                    if r2
                    else float('inf')
                )
                return src1 if d1 <= d2 else src2

        return src1

    # =================================================================
    # Route execution
    # =================================================================

    def _execute_route(self, route_nodes, target):
        """Execute a planned route, segment by segment.

        1. Extract route edges from the NetworkX graph.
        2. Merge consecutive same-action-type edges into segments.
        3. Execute each segment sequentially.
        4. For row_traversal segments, publish the boundary polygon.
        """
        self._navigation_activated = True

        route_edges = get_route_edges(self._graph, route_nodes)
        if not route_edges:
            self.get_logger().error("[EXEC] No edges found along route")
            return False

        segments = merge_action_segments(route_edges)

        self.get_logger().info(
            "[EXEC] %d edges -> %d segment(s):"
            % (len(route_edges), len(segments)),
        )
        for i, seg in enumerate(segments):
            self.get_logger().info(
                "  [%d] %s x%d: %s -> %s"
                % (
                    i,
                    seg.action_type,
                    seg.num_edges,
                    seg.first_source,
                    seg.last_target,
                ),
            )

        for seg_idx, segment in enumerate(segments):
            if self._cancelled or self._preempted:
                self.get_logger().warning("[EXEC] Cancelled/preempted")
                self._sm.transition(NavState.CANCELLED)
                self._publish_status("CANCELLED")
                return False

            is_final = seg_idx == len(segments) - 1
            ok = self._execute_segment(
                segment, is_final, seg_idx, len(segments),
            )

            if not ok:
                if self._cancelled or self._preempted:
                    self._sm.transition(NavState.CANCELLED)
                    self._publish_status("CANCELLED")
                    return False

                recovered = self._attempt_recovery(
                    segment, route_nodes, target, seg_idx,
                )
                if not recovered:
                    self.get_logger().error(
                        "[EXEC] Segment %d (%s) failed permanently"
                        % (seg_idx, segment.action_type),
                    )
                    return False

        self._sm.transition(NavState.SUCCEEDED)
        self._publish_status("SUCCEEDED")
        self._publish_empty_boundary()
        self.get_logger().info(
            "[EXEC] Route to '%s' completed successfully" % target,
        )
        return True

    def _execute_segment(self, segment, is_final, seg_idx, total):
        """Execute one action segment as a single Nav2 multi-pose goal.

        All waypoints in the segment are sent at once via
        ``NavigateThroughPoses``. Intermediate waypoints use identity
        orientation so the robot drives through without stopping to
        rotate; only the final waypoint keeps its map orientation.

        For ``row_traversal`` segments the boundary polygon is
        published before sending the goal.

        Pre-flight checks (restrictions, reconfigure) are performed
        for every edge in the segment before the goal is dispatched.
        After the Nav2 goal completes, per-edge statistics are
        published retroactively.
        """
        action = segment.action_type
        exec_state = ACTION_TO_STATE.get(
            action, NavState.EXECUTING_NAVIGATE_TO_POSE,
        )

        self._sm.transition(exec_state)
        self._publish_status(exec_state.value)

        self.get_logger().info(
            "[SEG %d/%d] %s x%d: %s -> %s"
            % (
                seg_idx + 1,
                total,
                action,
                segment.num_edges,
                segment.first_source,
                segment.last_target,
            ),
        )

        # row_traversal: compute & publish boundary polygon
        if action == "row_traversal":
            self._handle_row_boundary(segment)
        else:
            self._publish_empty_boundary()

        # ----- Pre-flight: validate all edges before sending goal -----
        edge_dicts = []
        for ei, edata in enumerate(segment.edge_data):
            if self._cancelled or self._preempted:
                return False

            src = edata['source']
            tgt = edata['target']
            edge_id = edata.get('edge_id', '')

            edge_dict = get_edge_from_id_tmap2(
                self._tmap, src, edge_id,
            )

            if not edge_dict:
                self.get_logger().error(
                    "  Edge '%s' (%s->%s): lookup failed"
                    % (edge_id, src, tgt),
                )
                self._stat = nav_stats(
                    src, tgt, self._topol_map, edge_id,
                )
                self._stat.set_ended(self._current_node)
                self._stat.status = "failed"
                self._publish_stats()
                return False

            # Restrictions check
            if not self._check_restrictions(edge_id, tgt):
                self._stat = nav_stats(
                    src, tgt, self._topol_map, edge_id,
                )
                self._stat.set_ended(self._current_node)
                self._stat.status = "restricted"
                self._publish_stats()
                return False

            edge_dicts.append(edge_dict)

        # ----- Edge reconfigure (pre) using first edge -----
        first_edge_dict = edge_dicts[0] if edge_dicts else None
        if (
            first_edge_dict
            and self._edge_reconf_mgr
            and not self._srv_edge_reconfigure
        ):
            self._edge_reconf_mgr.register_edge(first_edge_dict)
            if self._edge_reconf_mgr.active:
                self._edge_reconf_mgr.initialise()
                self._edge_reconf_mgr.reconfigure()

        # ----- Build multi-pose goal for the whole segment -----
        self._current_target = segment.last_target
        all_edge_ids = " -> ".join(
            e.get('edge_id', '?') for e in segment.edge_data
        )
        self._publish_current_edge(
            segment.edge_ids[0] if segment.edge_ids else "none",
        )

        self.get_logger().info(
            "  Sending %d-waypoint %s goal: %s"
            % (segment.num_edges, action, all_edge_ids),
        )

        nav2_goal = self._build_segment_goal(segment, is_final)
        self._stat = nav_stats(
            segment.first_source or "?",
            segment.last_target or "?",
            self._topol_map,
            segment.edge_ids[0] if segment.edge_ids else "",
        )
        status = self._send_nav2_goal(nav2_goal)

        status_str = _status_str(status)
        self._publish_move_status(
            segment.last_target or "?", action, status_str,
        )

        # ----- Edge reconfigure (post-reset) -----
        if (
            self._edge_reconf_mgr
            and not self._srv_edge_reconfigure
            and self._edge_reconf_mgr.active
        ):
            self._edge_reconf_mgr._reset()

        # ----- Evaluate result -----
        self._stat.set_ended(self._current_node)
        if (
            status == GoalStatus.STATUS_SUCCEEDED
            or self._goal_reached
        ):
            self._stat.status = "success"
            self._publish_stats()
            self.get_logger().info(
                "  Segment OK: %s -> %s (%.1fs)"
                % (
                    segment.first_source,
                    segment.last_target,
                    self._stat.operation_time,
                ),
            )
            self._goal_reached = False
        else:
            self._stat.status = "failed"
            self._publish_stats()
            if status == GoalStatus.STATUS_CANCELED:
                self._preempted = True
            self.get_logger().warning(
                "  Segment FAILED: %s -> %s (status=%s)"
                % (
                    segment.first_source,
                    segment.last_target,
                    status_str,
                ),
            )
            return False

        self._publish_current_edge("none")
        return True

    # =================================================================
    # RowTraversal boundary
    # =================================================================

    def _handle_row_boundary(self, segment):
        """Compute and publish boundary polygon for row_traversal."""
        frame_id = self._graph.nodes.get(
            segment.first_source, {},
        ).get('parent_frame', 'map')

        poly = compute_row_boundary_polygon(
            self._graph,
            segment,
            default_left=self._default_boundary_left,
            default_right=self._default_boundary_right,
        )

        if poly:
            self._publish_boundary(poly, frame_id)
            props = segment.edge_data[0].get('properties', {})
            left = props.get(
                'boundary_left', self._default_boundary_left,
            )
            right = props.get(
                'boundary_right', self._default_boundary_right,
            )
            self.get_logger().info(
                "[BOUNDARY] row_traversal corridor: "
                "left=%.2fm, right=%.2fm, edges=%d"
                % (left, right, segment.num_edges),
            )
        else:
            self.get_logger().warning(
                "[BOUNDARY] Could not compute row boundary polygon",
            )
            self._publish_empty_boundary(frame_id)

    # =================================================================
    # Restrictions
    # =================================================================

    def _check_restrictions(self, edge_id, target_node):
        """Check edge/node restrictions. True = navigation allowed."""
        if not self._using_restrictions:
            return True
        try:
            req = EvaluateEdge.Request()
            req.edge = edge_id
            req.runtime = True
            fut = self._eval_edge_srv.call_async(req)
            rclpy.spin_until_future_complete(
                self, fut, timeout_sec=3.0,
            )
            if fut.done():
                r = fut.result()
                if r and r.success and r.evaluation:
                    self.get_logger().warning(
                        "[RESTRICT] Edge '%s' restricted" % edge_id,
                    )
                    return False

            req2 = EvaluateNode.Request()
            req2.node = target_node
            req2.runtime = True
            fut2 = self._eval_node_srv.call_async(req2)
            rclpy.spin_until_future_complete(
                self, fut2, timeout_sec=3.0,
            )
            if fut2.done():
                r2 = fut2.result()
                if r2 and r2.success and r2.evaluation:
                    self.get_logger().warning(
                        "[RESTRICT] Node '%s' restricted" % target_node,
                    )
                    return False
        except Exception as exc:
            self.get_logger().error(
                "[RESTRICT] Error checking restrictions: %s" % exc,
            )
        return True

    # =================================================================
    # Recovery / fail policy
    # =================================================================

    def _attempt_recovery(self, segment, route_nodes, target, seg_idx):
        """Execute fail-policy recovery for a failed segment.

        Supported policies (comma-separated on edge,
        e.g. ``retry_3,replan,fail``):
            retry  -- re-execute the failed segment
            replan -- A* avoiding the failed edge(s), then execute
            fail   -- stop navigation
        """
        self._sm.transition(NavState.RECOVERING)
        self._publish_status("RECOVERING")

        src0 = segment.source_nodes[0] if segment.source_nodes else None
        eid0 = segment.edge_ids[0] if segment.edge_ids else None
        raw_edge = None
        if src0 and eid0:
            raw_edge = get_edge_from_id_tmap2(self._tmap, src0, eid0)
        policy_str = (raw_edge or {}).get('fail_policy', 'fail')

        policies = []
        for part in policy_str.split(','):
            tokens = part.strip().split('_')
            act = tokens[0]
            count = 1
            if len(tokens) > 1 and tokens[1].isdigit():
                count = int(tokens[1])
            policies.extend([act] * count)

        edge_key = eid0 or 'unknown'
        if edge_key not in self._fail_policy_state:
            self._fail_policy_state[edge_key] = 0
        idx = self._fail_policy_state[edge_key]

        while idx < len(policies):
            pol = policies[idx]
            self._fail_policy_state[edge_key] = idx + 1

            self.get_logger().info(
                "[RECOVER] Policy '%s' (%d/%d)"
                % (pol, idx + 1, len(policies)),
            )

            if pol == "retry":
                ok = self._execute_segment(
                    segment, seg_idx == 0, seg_idx, 1,
                )
                if ok:
                    self._fail_policy_state.pop(edge_key, None)
                    return True
                idx += 1

            elif pol == "replan":
                origin = (
                    self._current_node
                    if self._current_node not in ("none", "Unknown")
                    else self._closest_node
                )
                avoid = list(segment.edge_ids)
                new_route = plan_route(
                    self._graph,
                    origin,
                    target,
                    avoid_edges=avoid,
                    logger=self.get_logger(),
                )
                if new_route and len(new_route) >= 2:
                    self.get_logger().info(
                        "[RECOVER] Replanned: %s"
                        % " -> ".join(new_route),
                    )
                    ok = self._execute_route(new_route, target)
                    if ok:
                        self._fail_policy_state.pop(edge_key, None)
                        return True
                else:
                    self.get_logger().warning(
                        "[RECOVER] Replan failed -- no route",
                    )
                idx += 1

            elif pol == "fail":
                self.get_logger().warning(
                    "[RECOVER] 'fail' policy -- stopping navigation",
                )
                self._fail_policy_state.pop(edge_key, None)
                return False

            else:
                self.get_logger().warning(
                    "[RECOVER] Unknown policy '%s', skipping" % pol,
                )
                idx += 1

        self._fail_policy_state.pop(edge_key, None)
        return False

    # =================================================================
    # Helpers
    # =================================================================

    def _cancel_current_navigation(self):
        """Cancel any active navigation."""
        if self._navigation_activated:
            self.get_logger().info(
                "[CANCEL] Stopping current navigation",
            )
            self._cancel_nav2_goal(timeout_sec=2.0)
            self._cancelled = True
            self._navigation_activated = False


# =====================================================================
# Entry point
# =====================================================================


def main():
    """Launch the topological navigation server."""
    rclpy.init(args=None)

    node = TopologicalNavServer('topological_navigation')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # EdgeReconfigureManager is a separate node if enabled
    if node._edge_reconf_mgr is not None:
        executor.add_node(node._edge_reconf_mgr)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[SHUTDOWN] Keyboard interrupt")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
