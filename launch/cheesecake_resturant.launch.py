#!/usr/bin/env python3
"""Full Cheesecake navigation stack for the restaurant map, with real AMCL.

nav2_bringup_launch.py is launched with its default use_localization:=True, so
its stock localization_launch.py runs AMCL against the loaded map — robot pose
is corrected by lidar scan-matching, not derived purely from odometry
integration.

Brings up, in order:
  1. hardware + complete-robot state publisher  (skuba_bringup)
  2. nav2 navigation stack + AMCL localization    (nav2_bringup_launch.py)
  3. RViz                                         (nav2_bringup rviz)
  4. service nodes: nav_to_pose, save_location, door_detector,
     costmap_reconfig_server, following_server

    ros2 launch skuba_navigation cheesecake_resturant.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = "skuba_navigation"
    bringup_pkg = "skuba_bringup"
    nav2_bringup_pkg = "nav2_bringup"

    nav_share = get_package_share_directory(nav_pkg)
    bringup_share = get_package_share_directory(bringup_pkg)

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            nav_share, "maps", "20_restaurant", "resturant_map_long.yaml"
        ),
        description="Full path to the map yaml file to load",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_share, "params", "nav2_params.yaml"),
        description="nav2 parameters file",
    )

    # 1. Hardware + complete-robot state publisher (kobuki, lidar, teleop, RSP)
    # launch_robot_state_publisher:=false -- the manipulation stack's
    # move_group.launch.py already runs its own robot_state_publisher from the
    # same URDF (plus the live arm joints this one never gets). Leaving both
    # on puts two identically-named robot_state_publisher nodes on /tf at
    # once, which was diagnosed as the cause of move_to_tf intermittently
    # resolving targets to the wrong place (TEAM2-011, 2026-08-13).
    cheesecake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "cheesecake_bringup.launch.py")
        ),
        launch_arguments={
            "launch_robot_state_publisher": "false",
        }.items(),
    )

    # 2. nav2 navigation stack + AMCL localization (use_localization defaults True)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "nav2_bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "slam": "False",
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    # 3. RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(nav2_bringup_pkg),
                "launch",
                "rviz_launch.py",
            )
        )
    )

    # 4. Service nodes
    nav_to_pose = Node(
        package=nav_pkg, executable="nav_to_pose", name="nav_to_pose", output="screen"
    )
    save_location = Node(
        package=nav_pkg,
        executable="save_location",
        name="save_location",
        output="screen",
    )
    door_detector = Node(
        package=nav_pkg,
        executable="door_detector",
        name="door_detector",
        output="screen",
    )
    costmap_reconfig_server = Node(
        package=nav_pkg,
        executable="costmap_reconfig_server",
        name="costmap_reconfig_server",
        output="screen",
    )
    following_server = Node(
        package=nav_pkg,
        executable="following_server",
        name="following_server",
        output="screen",
    )

    return LaunchDescription(
        [
            map_arg,
            params_arg,
            # hardware first
            cheesecake_bringup,
            # nav2 (+ AMCL localization) after base/lidar/RSP are up
            TimerAction(period=5.0, actions=[nav2]),
            # rviz + service nodes after nav2 is up
            TimerAction(
                period=12.0,
                actions=[
                    rviz,
                    nav_to_pose,
                    save_location,
                    door_detector,
                    costmap_reconfig_server,
                    following_server,
                ],
            ),
        ]
    )
