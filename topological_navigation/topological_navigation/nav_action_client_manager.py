#!/usr/bin/env python3
"""
Nav2 Action Client Manager
Manages Nav2 ActionClient lifecycle and goal execution.
"""

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from typing import Dict, Optional, Callable, Any
from threading import Event


class NavActionClientManager:
    """
    Manages Nav2 action clients and handles goal execution.
    Separates action client management from goal construction logic.
    """
    
    def __init__(self, node):
        """
        Initialize action client manager.
        
        Args:
            node: ROS 2 node instance
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Action clients dictionary
        self._clients: Dict[str, ActionClient] = {}
        
        # Current goal state
        self._current_goal_handle = None
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self._goal_result = None
        self._goal_complete_event = Event()
        
        # Callbacks
        self._feedback_callback: Optional[Callable] = None
        self._result_callback: Optional[Callable] = None
    
    def create_client(self, action_type: str) -> ActionClient:
        """
        Create or get existing action client for specified action type.
        
        Args:
            action_type: Action type name ('NavigateToPose', 'NavigateThroughPoses', etc.)
            
        Returns:
            ActionClient instance
        """
        # Return existing client if already created
        if action_type in self._clients:
            return self._clients[action_type]
        
        # Map action type to ROS 2 action and topic
        action_map = {
            'NavigateToPose': (NavigateToPose, '/navigate_to_pose'),
            'NavigateThroughPoses': (NavigateThroughPoses, '/navigate_through_poses'),
            'FollowWaypoints': (FollowWaypoints, '/follow_waypoints'),
        }
        
        if action_type not in action_map:
            self.logger.error(f"Unknown action type: {action_type}")
            return None
        
        action_class, topic = action_map[action_type]
        
        # Create new action client
        client = ActionClient(
            self.node,
            action_class,
            topic
        )
        
        self._clients[action_type] = client
        self.logger.info(f"Created action client for {action_type} on {topic}")
        
        return client
    
    def wait_for_server(self, action_type: str, timeout_sec: float = 5.0) -> bool:
        """
        Wait for action server to be available.
        
        Args:
            action_type: Action type name
            timeout_sec: Timeout in seconds
            
        Returns:
            True if server is available, False otherwise
        """
        client = self.create_client(action_type)
        if client is None:
            return False
        
        self.logger.info(f"Waiting for {action_type} action server...")
        server_ready = client.wait_for_server(timeout_sec=timeout_sec)
        
        if server_ready:
            self.logger.info(f"{action_type} action server is ready")
        else:
            self.logger.warn(f"{action_type} action server not available after {timeout_sec}s")
        
        return server_ready
    
    def send_goal(self, action_type: str, goal_msg, 
                  feedback_callback: Optional[Callable] = None) -> bool:
        """
        Send goal to action server.
        
        Args:
            action_type: Action type name
            goal_msg: Goal message to send
            feedback_callback: Optional callback for feedback
            
        Returns:
            True if goal was accepted, False otherwise
        """
        client = self._clients.get(action_type)
        if client is None:
            self.logger.error(f"No client for {action_type}. Call create_client() first.")
            return False
        
        # Reset state
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self._goal_result = None
        self._goal_complete_event.clear()
        self._feedback_callback = feedback_callback
        
        # Send goal
        self.logger.info(f"Sending goal to {action_type}")
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self._internal_feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
        
        if not send_goal_future.done():
            self.logger.error("Goal send timeout")
            return False
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.logger.warn("Goal was rejected by action server")
            return False
        
        self.logger.info("Goal accepted by action server")
        self._current_goal_handle = goal_handle
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._internal_result_callback)
        
        return True
    
    def wait_for_result(self, timeout_sec: Optional[float] = None) -> tuple:
        """
        Wait for action to complete and return result.
        
        Args:
            timeout_sec: Optional timeout in seconds
            
        Returns:
            Tuple of (status, result)
        """
        if timeout_sec:
            completed = self._goal_complete_event.wait(timeout=timeout_sec)
            if not completed:
                self.logger.warn(f"Action did not complete within {timeout_sec}s")
                return (GoalStatus.STATUS_UNKNOWN, None)
        else:
            self._goal_complete_event.wait()
        
        return (self._goal_status, self._goal_result)
    
    def cancel_goal(self) -> bool:
        """
        Cancel the current goal.
        
        Returns:
            True if cancel request was sent, False otherwise
        """
        if self._current_goal_handle is None:
            self.logger.warn("No active goal to cancel")
            return False
        
        self.logger.info("Canceling current goal")
        cancel_future = self._current_goal_handle.cancel_goal_async()
        
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=5.0)
        
        if cancel_future.done():
            self.logger.info("Goal cancelled successfully")
            return True
        else:
            self.logger.error("Failed to cancel goal")
            return False
    
    def get_status(self) -> int:
        """
        Get current goal status.
        
        Returns:
            GoalStatus constant
        """
        return self._goal_status
    
    def is_active(self) -> bool:
        """
        Check if there is an active goal.
        
        Returns:
            True if goal is active, False otherwise
        """
        return self._goal_status in [
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING
        ]
    
    def is_complete(self) -> bool:
        """
        Check if goal has reached a terminal state.
        
        Returns:
            True if goal is complete, False otherwise
        """
        return self._goal_status in [
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED
        ]
    
    def _internal_feedback_callback(self, feedback_msg):
        """Internal feedback callback that forwards to user callback"""
        self._goal_status = GoalStatus.STATUS_EXECUTING
        
        if self._feedback_callback:
            self._feedback_callback(feedback_msg.feedback)
    
    def _internal_result_callback(self, future):
        """Internal result callback"""
        try:
            result = future.result()
            self._goal_status = result.status
            self._goal_result = result.result
            
            status_names = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED"
            }
            status_name = status_names.get(result.status, f"UNKNOWN({result.status})")
            self.logger.info(f"Goal completed with status: {status_name}")
            
        except Exception as e:
            self.logger.error(f"Exception in result callback: {e}")
            self._goal_status = GoalStatus.STATUS_ABORTED
        finally:
            self._goal_complete_event.set()
    
    def destroy(self):
        """Clean up action clients"""
        for action_type, client in self._clients.items():
            self.logger.info(f"Destroying action client for {action_type}")
            client.destroy()
        self._clients.clear()
