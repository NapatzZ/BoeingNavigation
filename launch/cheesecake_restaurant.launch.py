#!/usr/bin/env python3
"""
Cheesecake navigation for the restaurant: a plain white map and no AMCL.

The restaurant map has free floor and no walls, so AMCL has nothing to
match the laser against. As in skuba_cheesecake_navigation, the robot trusts
its odometry instead: map_server loads the white map and a fixed
map -> odom transform makes the `map` frame equal to `odom`. The `map`
frame exists from the start, with no initial pose to send.

Everything else is cheesecake_nav.launch.py, so its arguments (sim,
use_rviz, launch_bringup, ...) work here too:

    ros2 launch skuba_navigation cheesecake_restaurant.launch.py

In skuba_robot the hardware container is already up:

    ros2 launch skuba_navigation cheesecake_restaurant.launch.py \
        launch_bringup:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory("skuba_navigation")
    sim = LaunchConfiguration("sim", default="false")
    use_sim_time = {"use_sim_time": ParameterValue(sim, value_type=bool)}

    # Declared before the nav launch: launch arguments are global, so the
    # nav launch (and the nav2 launch inside it) find these set and keep them.
    args = [
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                nav_share, "maps", "20_restaurant", "resturant_map_long.yaml"
            ),
            description="White map yaml for map_server",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_share, "params", "nav2_params.yaml"),
            description="nav2 parameters file",
        ),
        # No AMCL: nav2_bringup_launch.py skips its localization launch.
        DeclareLaunchArgument("use_localization", default_value="False"),
    ]
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "cheesecake_nav.launch.py")
        )
    )

    # What the skipped localization launch gave, minus AMCL: the map and
    # the map frame.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": LaunchConfiguration("map")},
            use_sim_time,
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"autostart": True, "node_names": ["map_server"]},
            use_sim_time,
        ],
    )
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    return LaunchDescription([*args, nav, map_server, map_lifecycle, map_to_odom])
