#!/usr/bin/env python3
"""
Interactive helper to run alongside nav2_slam_bringup_launch.

While SLAM is running, press Enter to save the current map. You will be
prompted for a map name (default: skuba_map), an optional category folder
(e.g. "30_skuba_home" — matches the maps/<category>/<name>.* layout; leave
blank to save straight into the maps root) and the maps root directory
(default: /root/skuba_ws/src/skuba_navigation/maps).

usage: ros2 run skuba_navigation map_save_cli
"""
from ..core.map_saver import MapSaver

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
        category = input("Category folder, e.g. 30_skuba_home [none]: ").strip()
        maps_root = input(f"Maps root [{DEFAULT_DIR}]: ").strip() or DEFAULT_DIR

        map_saver = MapSaver(map_name, category, maps_root)
        success = map_saver.save_maps()

        if success:
            print(f"Map '{map_name}' saved to {map_saver.save_directory}")
        else:
            print("Map save failed, see messages above.")

        again = input("Save another map? [y/N]: ").strip().lower()
        if again != "y":
            break


if __name__ == "__main__":
    main()
