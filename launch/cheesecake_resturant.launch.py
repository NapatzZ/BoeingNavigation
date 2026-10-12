#!/usr/bin/env python3
"""
Full Cheesecake navigation stack for the restaurant map, with real AMCL.

nav2_bringup_launch.py is launched with its default use_localization:=True, so
its stock localization_launch.py runs AMCL against the loaded map — robot pose
is corrected by lidar scan-matching, not derived purely from odometry
integration.

Brings up, in order:
  1. hardware + complete-robot state publisher  (skuba_bringup)
  2. nav2 navigation stack + AMCL localization    (nav2_bringup_launch.py)
  3. RViz                                         (nav2_bringup rviz)
  4. service nodes: navigation_server, save_location, door_detector,
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

    # 1. Hardware + complete-robot state publisher (skuba_bringup: kobuki, lidar, teleop, RSP)
    # skuba_bringup owns the one robot_state_publisher (whole URDF); the
    # manipulation stack runs with_bringup:=true and starts none, only
    # publishing the arm / pan-tilt /joint_states. Two at once was the cause
    # of move_to_tf resolving targets to the wrong place (TEAM2-011, 2026-08-13).
    cheesecake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "bringup.launch.py")),
        launch_arguments={
            # The xbox pad goes through twist_mux (joystick lane, priority 100)
            # instead of straight onto /cmd_vel, so the e_stop lock stops it too.
            "teleop_cmd_vel_topic": "cmd_vel_joy",
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
        ),
        launch_arguments={
            "rviz_config": os.path.join(nav_share, "rviz", "cheesecake_nav.rviz"),
        }.items(),
    )

    # 4. Service nodes
    navigation_server = Node(
        package=nav_pkg,
        executable="navigation_server",
        name="navigation_server",
        output="screen",
        parameters=[os.path.join(nav_share, "params", "navigation_server.yaml")],
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
                    navigation_server,
                    save_location,
                    door_detector,
                    costmap_reconfig_server,
                    following_server,
                ],
            ),
        ]
    )
