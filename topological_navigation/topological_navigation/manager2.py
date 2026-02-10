#!/usr/bin/env python3
"""
Created on Tue Sep 29 16:06:36 2020
Refactored on 2026-02-05

@author: Adam Binch (abinch@sagarobotics.com)
@maintainer: Ibrahim Hroob (ihroob@lincoln.ac.uk)
"""

#########################################################################################################
import os, sys, json, yaml, math, importlib

import rclpy, tf2_ros, std_msgs.msg, rosidl_runtime_py
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Pose # Kept Pose 

from std_srvs.srv import Trigger, Empty
import topological_navigation_msgs.srv as tn_srv

from topological_navigation.tmap_model import *


def pose_dist(pose1, pose2):
    return math.sqrt((pose1["position"]["x"] - pose2["position"]["x"])**2 + (pose1["position"]["y"] - pose2["position"]["y"])**2)

#########################################################################################################

class map_manager_2(rclpy.node.Node):

    def __init__(self, advertise_srvs=True):
        super().__init__('topological_map_manager_2')

        package_path = get_package_share_directory('topological_navigation')
        nav_config_default = str(os.path.join(package_path, 'config', 'navigation_goal.yaml'))

        # Schema path for validation
        self.schema_path = str(os.path.join(package_path, 'config', 'tmap-schema.yaml'))

        # Declare all parameters with defaults
        self.declare_parameter('cache_topological_maps', False)
        self.declare_parameter('auto_write_topological_maps', False)
        self.declare_parameter('nav_config', nav_config_default)
        self.declare_parameter('topological_map2_name', '')
        self.declare_parameter('topological_map2_filename', '')
        self.declare_parameter('topological_map2_path', '')

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Get parameter values
        self.cache_maps = self.get_parameter('cache_topological_maps').value
        self.auto_write = self.get_parameter('auto_write_topological_maps').value
        self.nav_config = self.get_parameter('nav_config').value
        self.topomap2_name = self.get_parameter('topological_map2_name').value
        self.topomap2_path = self.get_parameter('topological_map2_path').value
        self.topomap2_filename = self.get_parameter('topological_map2_filename').value

        self.get_logger().info("cache_topological_maps: {}".format(self.cache_maps))
        self.get_logger().info("auto_write_topological_maps: {}".format(self.auto_write))
        self.get_logger().info("nav config file: {}".format(self.nav_config))
        self.get_logger().info("topological_map2_name: {}".format(self.topomap2_name))
        self.get_logger().info("topological_map2_path: {}".format(self.topomap2_path))
        self.get_logger().info("topological_map2_filename: {}".format(self.topomap2_filename))
        self.get_logger().info("schema file: {}".format(self.schema_path))

        self.cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "topological_maps")
        if not os.path.exists(self.cache_dir):
            os.mkdir(self.cache_dir)
        self.get_logger().info(f"Cache directory: {self.cache_dir}")

        '''
        action_goal_cache is a dict to store resolved action_type and goal configurations 
        for actions. This allows us to avoid repeated file reads and parsing for the same 
        action types, improving efficiency when adding multiple edges with the same action.
        '''
        self.action_goal_cache  = {} 

        # Load default NavigateToPose configuration from nav_config parameter
        with open(self.nav_config, "r") as f:
            self.navigate_to_pose_config  = yaml.safe_load(f)["topological_navigation/navigation_goal"]
            self.get_logger().info(f"Loaded default NavigateToPose config: {self.navigate_to_pose_config }")

        # Initialize Model (empty)
        self.model = TopologicalMapModel(schema_path=self.schema_path, logger=self.get_logger())

        # Advertise services
        if advertise_srvs:
            self.advertise()

        # Create publisher for the topological map with transient local durability to ensure late subscribers get the latest map
        qos = QoSProfile(depth=10, 
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(std_msgs.msg.String, '/topological_map_2', qos)


    def parameters_callback(self, params):
        """
        Callback for parameter updates
        """
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'cache_topological_maps':
                self.cache_maps = param.value
                self.get_logger().info(f'Parameter cache_topological_maps updated to: {param.value}')
            
            elif param.name == 'auto_write_topological_maps':
                self.auto_write = param.value
                self.get_logger().info(f'Parameter auto_write_topological_maps updated to: {param.value}')
            
            elif param.name == 'nav_config':
                self.nav_config = param.value
                self.get_logger().info(f'Parameter nav_config updated to: {param.value}')
                # Reload navigation goal configuration
                try:
                    with open(self.nav_config, "r") as f:
                        self.navigate_to_pose_config  = yaml.safe_load(f)["topological_navigation/navigation_goal"]
                        self.get_logger().info(f"Reloaded MoveBaseGoal config: {self.navigate_to_pose_config }")
                except Exception as e:
                    self.get_logger().error(f"Failed to reload nav_config: {e}")
                    return SetParametersResult(successful=False, reason=str(e))
            
            elif param.name == 'topological_map2_name':
                # update the map name in the model if it exists
                if "tmap" in self.model.__dict__:
                    self.model.tmap["name"] = param.value
                self.get_logger().info(f'Parameter topological_map2_name updated to: {param.value}')
                        
            elif param.name == 'topological_map2_path':
                self.get_logger().info(f'Parameter topological_map2_path updated to: {param.value}')

            elif param.name == 'topological_map2_filename':
                self.get_logger().info(f'Parameter topological_map2_filename updated to: {param.value}')
                # Optionally, we could auto-switch maps on filename change, but for now we just log it.
                # To auto-switch, we would call self.switch_topological_map_cb with the new filename.

        return SetParametersResult(successful=True)


    def advertise(self):

        self.get_logger().info("Advertising services...")

        # Services that retrieve information from the map
        self.get_map_srv = self.create_service(Trigger, '/topological_map_manager2/get_topological_map', self.get_topological_map_cb)

        # Services that modify the map
        self.write_map_srv = self.create_service(tn_srv.WriteTopologicalMap, '/topological_map_manager2/write_topological_map', self.write_topological_map_cb)
        self.switch_map_srv = self.create_service(tn_srv.WriteTopologicalMap, '/topological_map_manager2/switch_topological_map', self.switch_topological_map_cb)


    def init_map(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", filepath=None, load=True):

        if transformation == "default":
            self.transformation = {}
            self.transformation["rotation"] = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
            self.transformation["translation"] = {"x": 0.0, "y": 0.0, "z": 0.0}
            self.transformation["child"] = "topo_map"
            self.transformation["parent"] = "map"
        else:
            self.transformation = transformation

        if load:
            self.load_map(filepath)
        else:
            # Initialize empty map in model
            self.model.tmap["name"] = name
            self.model.tmap["metric_map"] = metric_map
            self.model.tmap["pointset"] = pointset
            self.model.tmap["transformation"] = self.transformation
            self.model.tmap["nodes"] = []

        self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.model.tmap)))

        self.names = self.create_list_of_nodes()

        self.broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self.broadcast_transform()


    def load_map(self, filename=None):
        if filename is None:
            filename = os.path.join(self.topomap2_path, self.topomap2_filename)
        self.get_logger().info("Loading Topological Map {} ...".format(filename))
        
        try:
            self.model.load(filename)
            # Sync local properties from loaded map
            self.name = self.model.tmap.get("name", "new_map")
            self.metric_map = self.model.tmap.get("metric_map", "map_2d")
            self.pointset = self.model.tmap.get("pointset", "new_map")
            self.transformation = self.model.tmap.get("transformation", {})
            self.names = self.create_list_of_nodes()

            self.set_parameters([rclpy.parameter.Parameter('topological_map2_name', rclpy.Parameter.Type.STRING, self.pointset)])
            self.get_logger().info("Done")
            
            if self.cache_maps:
                self.get_logger().info("Caching the map...")
                self.write_topological_map(os.path.join(self.cache_dir, os.path.basename(filename)), no_alias=True)

        except Exception as e:
             self.get_logger().error(f"Failed to load map: {e}")


    def write_topological_map(self, filename, no_alias=False):
        try:
            self.model.save(filename, no_alias)
        except Exception as e:
            self.get_logger().error(f"Failed to write map: {e}")


    def update(self, update_time=True):

        if update_time:
            self.model.tmap["meta"]["last_updated"] = self.model._get_time()

        # validate the map before publishing
        self.model.validate()

        self.map_pub.publish(std_msgs.msg.String(data=json.dumps(self.model.tmap)))
        self.names = self.create_list_of_nodes()


    def broadcast_transform(self):

        trans, rot = Vector3(), Quaternion()
        rosidl_runtime_py.set_message_fields(trans, self.transformation.get("translation", {}))
        rosidl_runtime_py.set_message_fields(rot, self.transformation.get("rotation", {}))

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.transformation.get("parent", "map")
        msg.child_frame_id = self.transformation.get("child", "topo_map")
        msg.transform.translation = trans
        msg.transform.rotation = rot

        self.broadcaster.sendTransform(msg)


    def create_list_of_nodes(self):
        names = []
        if "nodes" in self.model.tmap:
            names = [node["node"]["name"] for node in self.model.tmap["nodes"]]
            names.sort()
        return names


    def get_topological_map_cb(self, req, res):
        """
        Returns the topological map
        """
        self.get_logger().info("[SRV] Topological map requested, sending map...")
        res.success = True
        res.message = json.dumps(self.model.tmap)
        return res


    def switch_topological_map_cb(self, req, res):
        """
        Changes the topological map
        """
        self.set_parameters([rclpy.parameter.Parameter('topological_map2_filename', rclpy.Parameter.Type.STRING, req.filename)])
        path = self.get_parameter("topological_map2_path").value

        # if filename is just a name without path, assume it's in the topological_map2_path directory
        if os.path.isabs(req.filename) or path == "":
            self.filename = req.filename
        else:
            self.filename = os.path.join(path, req.filename)

        # print stack for debugging
        self.get_logger().info(f"Switching to map: {self.filename}")

        self.load_map(self.filename)
        self.update(False)
        self.broadcast_transform()

        res.success = True
        res.message = json.dumps(self.model.tmap)
        return res


    def write_topological_map_cb(self, req, res):
        """
        Saves the topological map to a yaml file
        """
        filename = req.filename
        if not filename:
            path = self.get_parameter("topological_map2_path").value
            fname = self.get_parameter("topological_map2_filename").value
            filename = path + "/" + fname

        try:
            self.write_topological_map(filename, req.no_alias)
            res.success = True
            res.message = f"Writing map to {filename}"
        except Exception as e:
            res.success = False
            res.message = str(e)

        return res


    def set_goal(self, action, action_type, _goal=None):
        if action in self.action_goal_cache  and action_type == self.action_goal_cache [action]["action_type"]:
            goal = self.action_goal_cache [action]["goal"]
        else:
            if _goal is not None:
                goal = _goal
            else:
                try:
                    package = action_type.split("/")[0]
                    goal_def = action_type.split("/")[1]

                    # Check if there's a custom parameter for this action type
                    param_name = action_type.replace('/', '_')
                    if self.has_parameter(param_name):
                        _file = self.get_parameter(param_name).value
                    else:
                        _file = ""
                    
                    if not _file:
                        package_object = importlib.import_module(package)
                        _file = os.path.join(package_object.__path__[0], '..', 'config', f"{goal_def}.yaml")
                    with open(_file, "r") as f:
                        goal = yaml.safe_load(f)
                except:
                    action_type = self.navigate_to_pose_config ["action_type"]
                    goal = self.navigate_to_pose_config ["goal"]

            self.action_goal_cache [action] = {"action_type": action_type, "goal": goal}

        return action_type, goal


#########################################################################################################
def usage():
    """
    Display usage information for the topological map manager.
    """
    print("\n" + "="*80)
    print(" Topological Map Manager 2 - ROS 2 Node")
    print("="*80)
    print("\nDESCRIPTION:")
    print("  Publishes and manages topological maps for robot navigation.")
    print("  Provides services for adding/removing nodes, edges, and map manipulation.")
    print("\nUSAGE:")
    print("  ros2 run topological_navigation map_manager2.py [OPTIONS] [MAP_FILE]")
    print("\nOPTIONS:")
    print("  -h, --help              Show this help message and exit")
    print("  -n, --new MAP_FILE      Create a new empty map with the specified filename")
    print("  -t, --test              Load the default test map (test_simple_tmap2.yaml)")
    print("  -v, --verbose           Enable verbose logging output")
    print("\nARGUMENTS:")
    print("  MAP_FILE                Path to the topological map YAML file to load")
    print("                          If not specified, loads the default test map")
    print("\nEXAMPLES:")
    print("  # Load an existing map")
    print("  ros2 run topological_navigation map_manager2.py my_map.yaml")
    print("")
    print("  # Create a new empty map")
    print("  ros2 run topological_navigation map_manager2.py -n new_map.yaml")
    print("")
    print("  # Load default test map")
    print("  ros2 run topological_navigation map_manager2.py --test")
    print("")
    print("  # Load map with absolute path")
    print("  ros2 run topological_navigation map_manager2.py /path/to/map.yaml")
    print("\nSERVICES:")
    print("  See ROS 2 services list for available map management operations:")
    print("  ros2 service list | grep topological_map_manager2")
    print("\nNOTES:")
    print("  - Map files must be in YAML format conforming to tmap2 schema")
    print("  - The node will broadcast a TF transform from map to topological_map frame")
    print("  - Maps can be modified via ROS 2 services during runtime")
    print("="*80 + "\n")


def parse_arguments():
    """
    Parse command line arguments for the map manager.
    
    Returns:
        tuple: (map_file, load, verbose) where:
            - map_file (str): Path to the map file
            - load (bool): True to load existing map, False to create new
            - verbose (bool): Enable verbose logging
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Topological Map Manager 2 - Manages topological maps for robot navigation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s my_map.yaml              Load an existing map
  %(prog)s -n new_map.yaml          Create a new empty map
  %(prog)s --test                   Load the default test map
  %(prog)s -v my_map.yaml           Load map with verbose logging
        """
    )
    
    parser.add_argument(
        'map_file',
        nargs='?',
        default=None,
        help='Path to the topological map YAML file'
    )
    
    parser.add_argument(
        '-n', '--new',
        action='store_true',
        help='Create a new empty map instead of loading existing one'
    )
    
    parser.add_argument(
        '-t', '--test',
        action='store_true',
        help='Load the default test map (test_simple_tmap2.yaml)'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose logging output'
    )
    
    args = parser.parse_args()
    
    # Determine map file
    if args.test or (args.map_file is None and not args.new):
        # Load default test map
        try:
            package_path = get_package_share_directory('topological_navigation')
            map_file = os.path.join(package_path, 'config', 'test_simple_tmap2.yaml')
            if not args.test and args.map_file is None:
                print(f"No map specified, loading default test map: {map_file}")
        except Exception as e:
            print(f"Error: Could not find default test map: {e}")
            sys.exit(1)
    elif args.new:
        if args.map_file is None:
            print("Error: --new requires a map filename")
            parser.print_help()
            sys.exit(1)
        map_file = args.map_file
    else:
        map_file = args.map_file
    
    # Validate map file path
    if not args.new and map_file:
        if not os.path.exists(map_file):
            print(f"Error: Map file not found: {map_file}")
            sys.exit(1)
        if not map_file.endswith(('.yaml', '.yml')):
            print(f"Warning: Map file should have .yaml or .yml extension: {map_file}")
    
    load = not args.new
    
    return map_file, load, args.verbose


# add main function to run the node standalone for testing
def main(args=None):
    """
    Main entry point for the topological map manager node.
    
    Args:
        args: Optional ROS 2 arguments
        
    Returns:
        int: Exit code (0 for success, non-zero for failure)
    """
    try:
        # Parse command line arguments
        map_file, load, verbose = parse_arguments()
        
        # Initialize ROS 2
        rclpy.init(args=args)
        
        # Create manager node
        manager = map_manager_2(advertise_srvs=True)
        
        # Set log level if verbose
        if verbose:
            manager.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            manager.get_logger().info("Verbose logging enabled")
        
        # Display startup information
        manager.get_logger().info("="*80)
        manager.get_logger().info("Topological Map Manager 2 - Starting")
        manager.get_logger().info("="*80)
        
        if load:
            manager.get_logger().info(f"Loading map: {map_file}")
        else:
            manager.get_logger().info(f"Creating new map: {map_file}")
        
        # Initialize map
        try:
            manager.init_map(filepath=map_file, load=load)
            manager.get_logger().info(f"Map initialized successfully")
            manager.get_logger().info(f"  Name: {manager.model.tmap.get('name', 'N/A')}")
            manager.get_logger().info(f"  Nodes: {len(manager.model.tmap.get('nodes', []))}")
            manager.get_logger().info(f"  Metric map: {manager.model.tmap.get('metric_map', 'N/A')}")
        except Exception as e:
            manager.get_logger().error(f"Failed to initialize map: {e}")
            manager.destroy_node()
            rclpy.shutdown()
            return 1
        
        manager.get_logger().info("="*80)
        manager.get_logger().info("Services advertised. Node is ready.")
        manager.get_logger().info("Use 'ros2 service list | grep topological_map_manager2' to see available services")
        manager.get_logger().info("="*80)
        
        # Spin node
        try:
            rclpy.spin(manager)
        except KeyboardInterrupt:
            manager.get_logger().info("Keyboard interrupt received, shutting down...")
        except Exception as e:
            manager.get_logger().error(f"Error during node execution: {e}")
            return 1
        finally:
            # Clean shutdown
            manager.get_logger().info("Shutting down Topological Map Manager 2")
            manager.destroy_node()
            rclpy.shutdown()
        
        return 0
        
    except Exception as e:
        print(f"Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
#########################################################################################################