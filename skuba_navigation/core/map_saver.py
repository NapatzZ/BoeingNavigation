#!/usr/bin/env python3
"""
Serializes the current SLAM Toolbox map to disk. Used by map_save_cli's
interactive `ros2 run skuba_navigation map_save_cli` helper.

Matches the maps/ layout convention (maps/<category>/<name>.{pgm,yaml,data,
posegraph}, e.g. maps/30_skuba_home/skuba_home.pgm) instead of inventing an
extra per-map subfolder: `category` is the existing (or new) category folder
under `maps_root`, and the map files are written directly into it.
"""
import subprocess
import os

class MapSaver:
    def __init__(self, map_name="skuba_map", category="", maps_root=None):
        self.map_name = map_name
        # expanduser: a literal "~" typed into a save-path prompt is not
        # shell-expanded by input() — without this it creates a folder
        # named "~" instead of resolving to $HOME.
        maps_root = os.path.expanduser(maps_root) if maps_root else os.getcwd()
        self.save_directory = os.path.join(maps_root, category) if category else maps_root
        self.map_path = os.path.join(self.save_directory, self.map_name)

    def run_command(self, command):
        """Execute a command and return success status"""
        try:
            subprocess.run(command, check=True, capture_output=True, timeout=30)
            return True
        except (subprocess.SubprocessError, OSError):
            return False

    def save_nav2_map(self):
        """Save map using nav2_map_server"""
        command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self.map_path]
        return self.run_command(command)

    def serialize_slam_map(self):
        """Serialize SLAM toolbox map"""
        command = [
            "ros2", "service", "call", "/slam_toolbox/serialize_map", 
            "slam_toolbox/srv/SerializePoseGraph", f"{{filename: '{self.map_path}'}}"
        ]
        return self.run_command(command)

    def save_maps(self):
        """Save both maps"""
        print(f"Saving maps: {self.map_name}")
        
        os.makedirs(self.save_directory, exist_ok=True)
        
        nav2_success = self.save_nav2_map()
        slam_success = self.serialize_slam_map()
        
        if nav2_success and slam_success:
            print("Maps saved successfully")
            return True
        elif nav2_success:
            print("Nav2 map saved, SLAM serialization failed")
            return False
        elif slam_success:
            print("Nav2 map failed, SLAM map serialized")
            return False
        else:
            print("Both operations failed")
            return False