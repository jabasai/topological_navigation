#!/usr/bin/env python
"""
Goal Builder for EdgeActionManager2

This module provides a dedicated class for constructing ROS 2 action goals.
It extracts goal construction logic from EdgeActionManager2 to improve
modularity, testability, and reusability.

Created: 2026-02-03
Author: AI Coding Agent
"""

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import (
    NavigateToPose,
    NavigateThroughPoses,
    FollowWaypoints,
)
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.node import Node
import numpy as np
import math


class TopoNavEdgeActionMsg:
    """Container for topological navigation edge action messages."""

    def __init__(self):
        """Initialize the edge action message container."""
        self.action = None
        self.nav_goal = None
        self.side_edges = {}
        self.target_frame_id = "map"
        self.control_plugin_params = {}

    def setAction(self, action):
        """Set the action type."""
        self.action = action

    def setNavGoal(self, nav_goal):
        """Set the navigation goal."""
        self.nav_goal = nav_goal

    def setSideEdges(self, side_edges, target_frame_id):
        """Set side edges for row operations."""
        self.side_edges = side_edges
        self.target_frame_id = target_frame_id

    def setControlPluginParams(self, control_plugin_params):
        """Set control plugin parameters."""
        self.control_plugin_params = control_plugin_params

    def getAction(self):
        """Get the action type."""
        return self.action

    def getNavGoal(self):
        """Get the navigation goal."""
        return self.nav_goal

    def getBoundary(self):
        """Get boundary path from side edges."""
        from nav_msgs.msg import Path

        path = Path()
        header = Header()
        header.frame_id = self.target_frame_id
        path.header = header
        if len(self.side_edges) > 0:
            for key, val in self.side_edges.items():
                for pose in val:
                    path.poses.append(pose)
        return path

    def getTargetFrameId(self):
        """Get the target frame ID."""
        return self.target_frame_id


class GoalBuilder:
    """
    Builds ROS 2 action goals for navigation.

    This class is responsible for constructing NavigateToPose and
    NavigateThroughPoses goals with appropriate parameters and
    behavior tree configurations.
    """

    def __init__(self, node: Node, actions, bt_trees: dict = None):
        """
        Initialize the GoalBuilder.

        Args:
            node: ROS 2 node for logging
            actions: Actions configuration object with action type constants
            bt_trees: Dictionary of behavior tree configurations, defaults to empty dict
        """
        self.node = node
        self.actions = actions
        self.bt_trees = bt_trees if bt_trees is not None else {}

    def construct_navigate_to_pose_goal(self, goal_dict):
        """
        Construct a NavigateToPose goal from goal dictionary.

        Args:
            goal_dict: Dictionary with structure:
                {
                    "target_pose": {
                        "header": {"frame_id": "map"},
                        "pose": {
                            "position": {"x": ..., "y": ..., "z": ...},
                            "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}
                        }
                    }
                }

        Returns:
            List of TopoNavEdgeActionMsg objects
        """
        frame_id = goal_dict["target_pose"]["header"]["frame_id"]
        pose = goal_dict["target_pose"]["pose"]
        return self.get_navigate_to_pose_goal(frame_id, pose)

    def get_navigate_to_pose_goal(self, frame_id: str, goal_pose: dict):
        """
        Create a NavigateToPose goal.

        Args:
            frame_id: Target frame ID (e.g., "map")
            goal_pose: Pose dictionary with position and orientation

        Returns:
            List of TopoNavEdgeActionMsg objects containing NavigateToPose goal
        """
        nav_goal = NavigateToPose.Goal()
        target_pose = self._create_pose_stamped_msg(frame_id, goal_pose)
        nav_goal.pose = target_pose

        # Apply behavior tree if available
        if self.actions.NAVIGATE_TO_POSE in self.bt_trees:
            nav_goal.behavior_tree = self.bt_trees[self.actions.NAVIGATE_TO_POSE]
            self.node.get_logger().info(
                "NavigateToPose BT: {}".format(
                    self.bt_trees[self.actions.NAVIGATE_TO_POSE]
                )
            )

        action_msg = TopoNavEdgeActionMsg()
        action_msg.setAction(self.actions.NAVIGATE_TO_POSE)
        action_msg.setNavGoal(nav_goal)

        return [action_msg]

    def construct_navigate_through_poses_goal(
        self, goals, actions, edge_ids, is_execpolicy=False
    ):
        """
        Construct a NavigateThroughPoses goal from multiple goal dictionaries.

        Args:
            goals: Dictionary of goals indexed by segment
            actions: List of action names for each segment
            edge_ids: List of edge IDs for each segment
            is_execpolicy: Whether to use execute policy mode (relaxed yaw tolerance)

        Returns:
            Tuple of (action_msgs, control_server_configs)
        """
        return self.get_navigate_through_poses_goal(
            goals, actions, edge_ids, is_execpolicy=is_execpolicy
        )

    def get_navigate_through_poses_goal(
        self, poses_dict, actions, edge_ids, is_execpolicy=False
    ):
        """
        Create a NavigateThroughPoses goal.

        Args:
            poses_dict: Dictionary of pose lists indexed by segment
            actions: List of action names for each segment
            edge_ids: List of edge IDs for each segment
            is_execpolicy: Whether to use execute policy mode

        Returns:
            Tuple of (action_msgs list, control_server_configs dict)
        """
        control_server_configs = {}
        action_msgs = []

        for seg_i, nodes in poses_dict.items():
            nav_goal = NavigateThroughPoses.Goal()
            nav_goal.poses = []

            # Convert nodes to poses
            for node in nodes:
                target_pose_data = node.get("target_pose", {})
                frame_id = target_pose_data.get("header", {}).get("frame_id", "map")
                pose = target_pose_data.get("pose", {})
                pose_msg = self._create_pose_stamped_msg(frame_id, pose)
                nav_goal.poses.append(pose_msg)

            # Apply behavior tree if available
            if self.actions.NAVIGATE_THROUGH_POSES in self.bt_trees:
                nav_goal.behavior_tree = self.bt_trees[
                    self.actions.NAVIGATE_THROUGH_POSES
                ]
                self.node.get_logger().info(
                    "NavigateThroughPoses BT: {}".format(
                        self.bt_trees[self.actions.NAVIGATE_THROUGH_POSES]
                    )
                )

            action_msg = TopoNavEdgeActionMsg()
            action_msg.setAction(self.actions.NAVIGATE_THROUGH_POSES)
            action_msg.setNavGoal(nav_goal)
            action_msgs.append(action_msg)

        return action_msgs, control_server_configs

    def _create_pose_stamped_msg(self, frame_id: str, pose_dict: dict) -> PoseStamped:
        """
        Create a PoseStamped message from pose dictionary.

        Args:
            frame_id: Frame ID for the pose
            pose_dict: Dictionary with 'position' and 'orientation' keys

        Returns:
            PoseStamped message
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp.sec = 0
        pose_msg.header.stamp.nanosec = 0

        # Extract position
        if "position" in pose_dict:
            pos = pose_dict["position"]
            pose_msg.pose.position.x = float(pos.get("x", 0.0))
            pose_msg.pose.position.y = float(pos.get("y", 0.0))
            pose_msg.pose.position.z = float(pos.get("z", 0.0))

        # Extract orientation
        if "orientation" in pose_dict:
            ori = pose_dict["orientation"]
            pose_msg.pose.orientation.x = float(ori.get("x", 0.0))
            pose_msg.pose.orientation.y = float(ori.get("y", 0.0))
            pose_msg.pose.orientation.z = float(ori.get("z", 0.0))
            pose_msg.pose.orientation.w = float(ori.get("w", 1.0))
        else:
            # Default orientation (identity quaternion)
            pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def substitute_properties_into_goal(self, goal, properties: dict):
        """
        Substitute properties from node/edge into a goal.

        Args:
            goal: Navigation goal to modify
            properties: Property dictionary from node/edge

        Returns:
            Modified goal object
        """
        # Extract relevant properties
        if "xy_goal_tolerance" in properties:
            goal.xy_goal_tolerance = float(properties["xy_goal_tolerance"])
        if "yaw_goal_tolerance" in properties:
            goal.yaw_goal_tolerance = float(properties["yaw_goal_tolerance"])

        return goal

    def check_edges_area_same(self, side_edges: dict) -> bool:
        """
        Check if all side edges are at approximately the same location.

        Args:
            side_edges: Dictionary of side edge poses

        Returns:
            True if all edges are at the same position (within 0.001 m)
        """
        edge_poses = []
        if len(side_edges) >= 2:
            for key, val in side_edges.items():
                for pose in val:
                    edge_poses.append(
                        np.array([pose.pose.position.x, pose.pose.position.y])
                    )
                if len(edge_poses) == 2:
                    if np.linalg.norm(edge_poses[0] - edge_poses[1]) < 0.001:
                        return True
        return False

    def check_target_is_same(self, node1: dict, node2: dict) -> bool:
        """
        Check if two nodes have the same target position.

        Args:
            node1: First node dictionary
            node2: Second node dictionary

        Returns:
            True if nodes are at approximately the same position (within 0.001 m)
        """
        target1 = np.array(
            [
                node1["pose"]["position"]["x"],
                node1["pose"]["position"]["y"],
            ]
        )
        target2 = np.array(
            [
                node2["pose"]["position"]["x"],
                node2["pose"]["position"]["y"],
            ]
        )
        return np.linalg.norm(target1 - target2) < 0.001

    def extract_number_from_tag(self, s: str) -> float:
        """
        Extract numeric value from tag string.

        Args:
            s: Tag string like "p-123-abc" or "p123"

        Returns:
            First numeric component as float
        """
        try:
            # Remove leading letters
            numeric_part = ""
            for i, char in enumerate(s):
                if char.isdigit() or (char == '-' and i > 0) or char == '.':
                    numeric_part = s[i:]
                    break
            
            if numeric_part:
                # Remove any non-numeric part after the number
                result = ""
                for char in numeric_part:
                    if char.isdigit() or char == '.':
                        result += char
                    elif result:  # If we already have a number, stop
                        break
                    else:  # Skip leading non-digits
                        continue
                
                if result:
                    return float(result)
        except (IndexError, ValueError):
            pass
        return 0.0

    def yaw_from_quaternion(self, orientation: dict) -> float:
        """
        Convert quaternion to yaw angle.

        Args:
            orientation: Dictionary with x, y, z, w components

        Returns:
            Yaw angle in radians
        """
        x = float(orientation.get("x", 0.0))
        y = float(orientation.get("y", 0.0))
        z = float(orientation.get("z", 0.0))
        w = float(orientation.get("w", 1.0))

        # Extract yaw from quaternion
        sin_roll = 2 * (w * x + y * z)
        cos_roll = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2 * (w * y - z * x)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
        pitch = math.asin(sin_pitch)

        sin_yaw = 2 * (w * z + x * y)
        cos_yaw = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        return yaw
