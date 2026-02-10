#!/usr/bin/env python
"""
Row Operation Handler for EdgeActionManager2

This module provides a dedicated class for handling agricultural row operations.
It extracts row operation logic from EdgeActionManager2 to improve modularity,
testability, and reusability.

Created: 2026-02-03
Author: AI Coding Agent
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from rclpy.node import Node
import numpy as np
import math
from typing import List, Tuple, Optional, Dict


class RowOperationHandler:
    """
    Handles agricultural row navigation operations.

    This class manages row boundary detection, center node calculation,
    and intermediate waypoint generation for in-row navigation.
    """

    def __init__(self, node: Node, route_search, boundary_publisher=None):
        """
        Initialize the RowOperationHandler.

        Args:
            node: ROS 2 node for logging
            route_search: TopologicalRouteSearch2 instance for map queries
            boundary_publisher: ROS 2 publisher for boundary paths
        """
        self.node = node
        self.route_search = route_search
        self.boundary_publisher = boundary_publisher
        self.selected_edges = {}

    def get_row_center_node(self, edge_id: str) -> Tuple[Optional[dict], Optional[str], Optional[str]]:
        """
        Parse edge_id to find the row center node.

        Expected edge_id format: "<center_tag>_RowEnd_<end_tag>"

        Args:
            edge_id: Edge ID string

        Returns:
            Tuple of (center_node_dict, tag_id, target_row_edge_id) or (None, None, None) on failure
        """
        try:
            parts = edge_id.split("_RowEnd_")
            if len(parts) != 2:
                self.node.get_logger().error(
                    f"Invalid row edge format: {edge_id}. Expected '<center>_RowEnd_<end>'"
                )
                return None, None, None

            center_tag = parts[0]
            end_tag = parts[1]

            # Extract numeric ID from tag
            center_id = self._extract_numeric_id(center_tag)
            end_id = self._extract_numeric_id(end_tag)

            if center_id is None or end_id is None:
                self.node.get_logger().error(
                    f"Could not extract numeric IDs from edge_id: {edge_id}"
                )
                return None, None, None

            target_row_edge_id = center_tag

            # Fetch the center node from topological map
            center_node = self.route_search.get_node_from_tmap2(target_row_edge_id)
            if not center_node:
                self.node.get_logger().error(
                    f"Center node not found in topological map: {target_row_edge_id}"
                )
                return None, None, None

            return center_node, center_id, target_row_edge_id

        except Exception as e:
            self.node.get_logger().error(
                f"[get_row_center_node] Exception: {e}"
            )
            return None, None, None

    def collect_boundary_candidates(
        self, center_node: dict, target_row_edge_id: str
    ) -> List[str]:
        """
        Collect boundary node candidates for row operations.

        Args:
            center_node: Center node dictionary
            target_row_edge_id: ID of the target row edge

        Returns:
            List of boundary node IDs
        """
        candidates = []
        try:
            # Get edges from center node
            edges = center_node.get("node", {}).get("edges", [])

            for edge in edges:
                target_node_id = edge.get("node", "")
                edge_id = edge.get("edge_id", "")

                # Look for side edges (boundary candidates)
                if self._is_side_edge(edge_id, target_row_edge_id):
                    candidates.append(target_node_id)

            self.node.get_logger().info(
                f"[collect_boundary_candidates] Found {len(candidates)} candidates: {candidates}"
            )

        except Exception as e:
            self.node.get_logger().error(
                f"[collect_boundary_candidates] Exception: {e}"
            )

        return candidates

    def select_boundary_nodes(
        self,
        candidates: List[str],
        center_xy: np.ndarray,
        row_direction: np.ndarray,
        tag_id: str,
    ) -> List[str]:
        """
        Select boundary nodes based on row geometry.

        Args:
            candidates: List of candidate node IDs
            center_xy: Center position as [x, y] array
            row_direction: Row direction as unit vector [dx, dy]
            tag_id: Tag ID for disambiguation

        Returns:
            List of selected boundary node IDs
        """
        selected = []
        perp_direction = np.array([-row_direction[1], row_direction[0]])

        try:
            left_candidates = []
            right_candidates = []

            # Classify candidates as left or right of center
            for candidate_id in candidates:
                node = self.route_search.get_node_from_tmap2(candidate_id)
                if not node or "node" not in node:
                    continue

                node_pos = node["node"]["pose"]["position"]
                node_xy = np.array([node_pos["x"], node_pos["y"]])
                offset = node_xy - center_xy

                # Project onto perpendicular direction
                dot_product = np.dot(offset, perp_direction)

                if dot_product < -0.01:
                    left_candidates.append((candidate_id, abs(dot_product)))
                elif dot_product > 0.01:
                    right_candidates.append((candidate_id, abs(dot_product)))

            # Select closest from each side
            if left_candidates:
                left_candidates.sort(key=lambda x: x[1])
                selected.append(left_candidates[0][0])

            if right_candidates:
                right_candidates.sort(key=lambda x: x[1])
                selected.append(right_candidates[0][0])

            self.node.get_logger().info(
                f"[select_boundary_nodes] Selected {len(selected)} boundary nodes: {selected}"
            )

        except Exception as e:
            self.node.get_logger().error(
                f"[select_boundary_nodes] Exception: {e}"
            )

        return selected

    def get_intermediate_poses_interpolated(
        self,
        selected_edges: Dict[str, List[PoseStamped]],
        center_node: dict,
        last_goal: dict,
        step_size: float = 0.5,
    ) -> List[PoseStamped]:
        """
        Generate interpolated intermediate poses for row navigation.

        Args:
            selected_edges: Dictionary of selected edge poses
            center_node: Center node dictionary
            last_goal: Final goal for the row
            step_size: Step size for interpolation

        Returns:
            List of interpolated PoseStamped messages
        """
        interpolated_poses = []

        try:
            center_pos = center_node["node"]["pose"]["position"]
            center_xy = np.array([center_pos["x"], center_pos["y"]])

            last_pos = last_goal.get("target_pose", {}).get("pose", {}).get("position", {})
            last_xy = np.array([last_pos.get("x", 0.0), last_pos.get("y", 0.0)])

            # Generate interpolated poses along the row
            distance = np.linalg.norm(last_xy - center_xy)
            if distance > 0:
                direction = (last_xy - center_xy) / distance
                num_steps = max(2, int(distance / step_size))

                for i in range(1, num_steps):
                    t = i / num_steps
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = center_xy[0] + direction[0] * distance * t
                    pose.pose.position.y = center_xy[1] + direction[1] * distance * t
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    interpolated_poses.append(pose)

            self.node.get_logger().info(
                f"[get_intermediate_poses_interpolated] Generated {len(interpolated_poses)} poses"
            )

        except Exception as e:
            self.node.get_logger().error(
                f"[get_intermediate_poses_interpolated] Exception: {e}"
            )

        return interpolated_poses

    def publish_boundary_path(
        self, selected_edges: Dict[str, List[PoseStamped]], frame_id: str = "map"
    ):
        """
        Publish boundary path to ROS 2 topic.

        Args:
            selected_edges: Dictionary of selected edge poses
            frame_id: Frame ID for the path
        """
        if not self.boundary_publisher:
            return

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0

        for edge_id, poses in selected_edges.items():
            path.poses.extend(poses)

        self.boundary_publisher.publish(path)
        self.node.get_logger().info(
            f"Published boundary path with {len(path.poses)} poses"
        )

    def _extract_numeric_id(self, tag: str) -> Optional[str]:
        """
        Extract numeric ID from tag string.

        Args:
            tag: Tag string like "p123" or "p-123"

        Returns:
            Numeric ID as string, or None if extraction fails
        """
        try:
            # Handle format like "p123" or "p-123"
            numeric_part = tag.lstrip("pP")
            # Remove leading dash if present
            if numeric_part.startswith("-"):
                numeric_part = numeric_part[1:]
            numeric_part = numeric_part.split("-")[0] if "-" in numeric_part else numeric_part
            if numeric_part and numeric_part.replace(".", "", 1).isdigit():
                return numeric_part
        except Exception:
            pass
        return None

    def _is_side_edge(self, edge_id: str, center_edge_id: str) -> bool:
        """
        Determine if an edge is a side edge (boundary edge).

        Args:
            edge_id: Edge ID to check
            center_edge_id: Center row edge ID

        Returns:
            True if this is a side/boundary edge
        """
        # Side edges have different patterns (e.g., side prefix or specific naming)
        return (
            "side" in edge_id.lower()
            or "boundary" in edge_id.lower()
            or edge_id != center_edge_id
        )

    def _is_row_node_name(self, name: str) -> bool:
        """
        Check if node name indicates a row navigation node.

        Args:
            name: Node name to check

        Returns:
            True if this appears to be a row node
        """
        return (
            "row" in name.lower()
            or "end" in name.lower()
            or "entry" in name.lower()
        )

    def validate_row_operation(
        self,
        center_node: Optional[dict],
        boundary_candidates: List[str],
        selected_boundaries: List[str],
    ) -> bool:
        """
        Validate that row operation can proceed.

        Args:
            center_node: Center node dictionary or None
            boundary_candidates: List of boundary candidates
            selected_boundaries: List of selected boundaries

        Returns:
            True if row operation parameters are valid
        """
        if center_node is None:
            self.node.get_logger().warn("No center node found for row operation")
            return False

        if not boundary_candidates:
            self.node.get_logger().warn("No boundary candidates found for row operation")
            return False

        if not selected_boundaries:
            self.node.get_logger().warn("No boundaries selected for row operation")
            return False

        return True

    def set_pose_yaw(self, pose_stamped: PoseStamped, yaw: float):
        """
        Set the yaw angle of a pose.

        Args:
            pose_stamped: PoseStamped message to modify
            yaw: Yaw angle in radians
        """
        # Convert yaw to quaternion
        x = 0.0
        y = 0.0
        z = math.sin(yaw / 2.0)
        w = math.cos(yaw / 2.0)

        pose_stamped.pose.orientation.x = x
        pose_stamped.pose.orientation.y = y
        pose_stamped.pose.orientation.z = z
        pose_stamped.pose.orientation.w = w
