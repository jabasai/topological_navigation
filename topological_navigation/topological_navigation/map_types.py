#!/usr/bin/env python3
"""
Typed data structures for topological maps.
"""

import yaml

# ===== YAML Loader =====

class CustomSafeLoader(yaml.SafeLoader):
    """
    Custom YAML loader that ensures poses and translations are float-type.
    ROS 2 messages (Vector3, Pose, etc.) have assertions for float-type [x,y,z,w] keys.
    """
    def construct_mapping(self, node, deep=False):
        mapping = super().construct_mapping(node, deep=deep)
        
        # Convert int to float for pose/vector keys
        for key in ['x', 'y', 'z', 'w', 'yaw_goal_tolerance', 'xy_goal_tolerance']:
            if key in mapping and isinstance(mapping[key], int):
                mapping[key] = float(mapping[key])
        
        return mapping