#!/usr/bin/env python3
"""
usage : ros2 run skuba_navigation map_saver --name kuy_gu_dum --dir some/dir/path

The map is saved into its own subfolder: <dir>/<name>/<name>.{pgm,yaml,data,posegraph}
"""
import subprocess
import sys
import os
import argparse

class MapSaver:
    def __init__(self, map_name="skuba_map", save_directory=None):
        self.map_name = map_name
        base_directory = save_directory or os.getcwd()
        # Each map gets its own subfolder: <dir>/<name>/<name>.{pgm,yaml,data,posegraph}
        self.save_directory = os.path.join(base_directory, self.map_name)
        self.map_path = os.path.join(self.save_directory, self.map_name)
        
    def run_command(self, command):
        """Execute a command and return success status"""
        try:
            subprocess.run(command, check=True, capture_output=True, timeout=30)
            return True
        except:
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

def main():
    parser = argparse.ArgumentParser(description="Save ROS2 maps")
    parser.add_argument("--name", "-n", default="skuba_map", help="Map name")
    parser.add_argument("--dir", "-d", help="Save directory")
    
    args = parser.parse_args()
    
    map_saver = MapSaver(args.name, args.dir)
    success = map_saver.save_maps()
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()