#!/usr/bin/env python3
"""
Interactive helper to run alongside nav2_slam_bringup_launch.

While SLAM is running, press Enter to save the current map. You will be
prompted for a map name (default: skuba_map) and a save directory
(default: /root/skuba_ws/src/skuba_navigation/maps). The map is
written into its own subfolder <dir>/<name>/<name>.{pgm,yaml,data,posegraph}.

usage: ros2 run skuba_navigation map_save_cli
"""
from skuba_navigation.map_saver import MapSaver

DEFAULT_DIR = "/root/skuba_ws/src/skuba_navigation/maps"


def main():
    print("Mapping in progress.")
    print("Press Enter to save the map, or Ctrl+C to quit without saving.")

    while True:
        try:
            input("\nPress Enter to save map...")
        except (KeyboardInterrupt, EOFError):
            print("\nExiting without saving.")
            break

        map_name = input("Map name [skuba_map]: ").strip() or "skuba_map"
        save_dir = input(f"Save directory [{DEFAULT_DIR}]: ").strip() or DEFAULT_DIR

        map_saver = MapSaver(map_name, save_dir)
        success = map_saver.save_maps()

        if success:
            print(f"Map '{map_name}' saved to {save_dir}")
        else:
            print("Map save failed, see messages above.")

        again = input("Save another map? [y/N]: ").strip().lower()
        if again != "y":
            break


if __name__ == "__main__":
    main()
