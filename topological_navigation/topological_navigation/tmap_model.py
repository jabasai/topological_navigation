import yaml
import math
import datetime
import logging
import jsonschema

from topological_navigation.map_types import CustomSafeLoader

# Helper for YAML dumping
class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

class MapValidationError(Exception):
    pass

class NodeNotFoundError(Exception):
    pass

class EdgeNotFoundError(Exception):
    pass

class DuplicateError(Exception):
    pass

class TopologicalMapModel:
    """
    Represents the data model for a Topological Map (tmap2).
    Handles loading, saving, validation, and manipulation of the map data.
    """
    def __init__(self, filename=None, schema_path=None, logger=None):
        self.tmap = {}
        self.filename = filename
        self.schema = None
        self.logger = logger or logging.getLogger(__name__)

        if schema_path:
            try:
                # add timer to log loading time
                with open(schema_path, 'r') as f:
                    self.schema = yaml.safe_load(f)
                self.logger.info(f"Loaded schema from {schema_path} successfully.")
            except Exception as e:
                self.logger.error(f"Failed to load schema from {schema_path}: {e}")

        # Initialize empty map structure
        self.tmap = {
            "meta": {"last_updated": self._get_time()},
            "metric_map": "map_2d",
            "name": "new_map",
            "pointset": "new_map",
            "transformation": {
                "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "child": "topo_map",
                "parent": "map"
            },
            "nodes": []
        }

    def _get_time(self):
        # Schema expects DD-MM-YYYY_HH-MM-SS format
        return datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')

    def load(self, filename):
        """Loads the map from a YAML file."""
        self.filename = filename
        self.logger.info(f"Loading map from {filename}")
        
        try:
            # Add timer to log loading time
            start_time = datetime.datetime.now()
            
            with open(self.filename, "r") as f:
                loaded_map = yaml.load(f, Loader=CustomSafeLoader)

            if not isinstance(loaded_map, dict):
                 raise MapValidationError(f"Loaded map must be a dictionary, got {type(loaded_map)}")

            self.tmap = loaded_map
            self.validate()
            self.logger.info("Map loaded and validated successfully.")

            end_time = datetime.datetime.now()
            load_validate_time = (end_time - start_time).total_seconds()
            self.logger.info(f"Map loaded and validated in {load_validate_time:.3f} seconds.")

            return True
        except Exception as e:
            self.logger.error(f"Failed to load map: {e}")
            raise

    def save(self, filename=None, no_alias=False):
        """Saves the map to a YAML file."""
        target_file = filename or self.filename
        if not target_file:
            raise ValueError("No filename specified for saving.")

        self.logger.info(f"Saving map to {target_file}")
        
        # Sort nodes by name for deterministic output
        if "nodes" in self.tmap:
             self.tmap["nodes"].sort(key=lambda node: node["node"]["name"])

        try:
            if no_alias:
                yml = yaml.dump(self.tmap, default_flow_style=False, Dumper=NoAliasDumper)
            else:
                yml = yaml.safe_dump(self.tmap, default_flow_style=False)

            with open(target_file, "w") as fh:
                fh.write(str(yml))
            
            self.logger.info("Map saved successfully.")
        except Exception as e:
            self.logger.error(f"Failed to save map: {e}")
            raise

    def validate(self):
        """Validates the map against the schema and logical constraints."""
        # 1. Schema Validation
        if self.schema:
            try:
                jsonschema.validate(instance=self.tmap, schema=self.schema)
            except jsonschema.exceptions.ValidationError as e:
                raise MapValidationError(f"Schema validation failed: {e.message} at {e.path}")

        # 2. Logical Validation (ported from map_check)
        self._check_consistency()

    def _check_consistency(self):
        if "nodes" not in self.tmap:
            raise MapValidationError("Map must contain 'nodes' key.")
             
        # Check pointset consistency
        pointsets = {node["meta"]["pointset"] for node in self.tmap["nodes"]}
        if len(pointsets) > 1:
            raise MapValidationError(f"Multiple pointsets found: {pointsets}")

        # Check duplicate node names
        names = [node["node"]["name"] for node in self.tmap["nodes"]]
        if len(names) != len(set(names)):
             # Find duplicate
             seen = set()
             dupes = [x for x in names if x in seen or seen.add(x)]
             raise MapValidationError(f"Duplicate node names found: {dupes}")

        # Check edges
        sep = "_"
        # Note: manager2 used a UUID separator which is weird for checking uniqueness across runs, 
        # but internal uniqueness is what matters. 
        # We will check if (origin, destination) pairs are unique.
        
        edges_set = set()
        for node in self.tmap["nodes"]:
            origin = node["node"]["name"]
            for edge in node["node"]["edges"]:
                dest = edge["node"]
                edge_id = edge["edge_id"]
                
                # Check destination exists
                if dest not in names:
                    raise MapValidationError(f"Edge from {origin} points to non-existent node {dest}")
                
                # Check self-loop
                if origin == dest:
                    self.logger.warning(f"Self-loop detected on node {origin} (edge {edge_id})")
                    # raise MapValidationError? Original code just warned.

                # Check edge ID uniqueness logic could be complex if IDs are manually set
                # But typically we care about Origin->Dest uniqueness or ID uniqueness
    
    def get_node(self, node_name):
        """Retrieves a node dict by name."""
        for node in self.tmap.get("nodes", []):
            if node["node"]["name"] == node_name:
                return node
        return None

    def get_node_index(self, node_name):
        for i, node in enumerate(self.tmap.get("nodes", [])):
            if node["node"]["name"] == node_name:
                return i
        return -1

    def add_node(self, name, pose, properties=None, verts=None, dist=8.0, 
                 restrictions_planning="True", restrictions_runtime="True"):
        
        if self.get_node(name):
            raise DuplicateError(f"Node {name} already exists.")

        node_data = {
            "meta": {
                "map": self.tmap.get("metric_map", "map_2d"),
                "node": name,
                "pointset": self.tmap.get("pointset", "new_map")
            },
            "node": {
                "name": name,
                "pose": pose,
                "edges": [],
                "localise_by_topic": "",
                "parent_frame": self.tmap.get("transformation", {}).get("parent", "map"),
                "properties": properties or {
                    "xy_goal_tolerance": 0.3, 
                    "yaw_goal_tolerance": 0.1
                },
                "verts": verts or self._generate_circle_vertices(),
                "restrictions_planning": restrictions_planning,
                "restrictions_runtime": restrictions_runtime
            }
        }
        self.tmap["nodes"].append(node_data)
        self.tmap["meta"]["last_updated"] = self._get_time()
        
    def remove_node(self, node_name):
        idx = self.get_node_index(node_name)
        if idx == -1:
            raise NodeNotFoundError(f"Node {node_name} not found.")
        
        # Remove the node
        del self.tmap["nodes"][idx]
        
        # Remove edges pointing to this node
        for node in self.tmap["nodes"]:
            original_edges = node["node"]["edges"]
            node["node"]["edges"] = [e for e in original_edges if e["node"] != node_name]

        self.tmap["meta"]["last_updated"] = self._get_time()

    def add_edge(self, origin, destination, action_type, config=None, action_name="move_base", edge_id=None):
        origin_node = self.get_node(origin)
        if not origin_node:
            raise NodeNotFoundError(f"Origin node {origin} not found")
        
        dest_node = self.get_node(destination)
        if not dest_node:
            raise NodeNotFoundError(f"Destination node {destination} not found")

        # Check if edge exists
        existing_ids = [e["edge_id"] for e in origin_node["node"]["edges"]]
        
        final_edge_id = edge_id
        if not final_edge_id or final_edge_id in existing_ids:
            # Auto-generate ID if conflict or missing
            test = 0
            base_id = f"{origin}_{destination}"
            final_edge_id = base_id
            while final_edge_id in existing_ids:
                final_edge_id = f"{base_id}_{test:03d}"
                test += 1
        
        # Construct edge
        # Note: goal logic is complex in manager2 (loading from file etc). 
        # The Model should ideally accept the goal object, not resolving it from ROS params.
        # We will assume 'config' and 'goal' are passed in ready-to-go or simplified.
        
        new_edge = {
            "edge_id": final_edge_id,
            "node": destination,
            "action": action_name,
            "action_type": action_type,
            "config": config or [],
            "fail_policy": "fail",
            "fluid_navigation": True,  # True: robot flows through intermediate waypoints, False: stops at each waypoint
            "restrictions_planning": "True",
            "restrictions_runtime": "True"
        }
        
        origin_node["node"]["edges"].append(new_edge)
        self.tmap["meta"]["last_updated"] = self._get_time()
        return final_edge_id

    def remove_edge(self, edge_id):
        # Scan all nodes to find and remove the edge
        found = False
        for node in self.tmap["nodes"]:
            edges = node["node"]["edges"]
            for i, edge in enumerate(edges):
                if edge["edge_id"] == edge_id:
                    del edges[i]
                    found = True
                    break # Assuming unique edge IDs map-wide?
            if found: break # Optimization if unique
        
        if not found:
             raise EdgeNotFoundError(f"Edge {edge_id} not found")
        
        self.tmap["meta"]["last_updated"] = self._get_time()


    def update_node_pose(self, node_name, pose):
        node = self.get_node(node_name)
        if not node:
            raise NodeNotFoundError(f"Node {node_name} not found")
        node["node"]["pose"] = pose
        self.tmap["meta"]["last_updated"] = self._get_time()

    def _generate_circle_vertices(self, radius=0.75, number=8):
        separation_angle = 2 * math.pi / number
        current_angle = separation_angle / 2
        points = []
        for i in range(0, number):
            points.append({"x": math.cos(current_angle) * radius, "y": math.sin(current_angle) * radius})
            current_angle += separation_angle
        return points

    def get_new_name(self):
        names = [node["node"]["name"] for node in self.tmap["nodes"]]
        namesnum = []
        for i in names:
            if i.startswith('WayPoint'):
                try:
                    nam = i.replace('WayPoint', '')
                    namesnum.append(int(nam))
                except ValueError:
                    pass
        namesnum.sort()
        if namesnum:
            return 'WayPoint%d'%(namesnum[-1]+1)
        else:
            return 'WayPoint1'
