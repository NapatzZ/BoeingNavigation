#!/usr/bin/env python3
"""
Cheesecake mapping session on the real robot: robot base + SLAM in one launch.

Brings up, in order:
  1. hardware + complete-robot state publisher  (skuba_bringup: kobuki, lidar, xbox teleop, RSP)
  2. SLAM Toolbox (mapping mode) + nav2 + twist_mux + e-stop + RViz (always on,
     on the robot's display)
                                                  (nav2_slam_bringup_launch.py)

Drive the robot with the xbox pad (or click Nav2 goals in RViz), then save the
map from a second shell:

    ros2 launch skuba_navigation cheesecake_slam.launch.py
    ros2 run skuba_navigation map_save_cli

nav2_slam_bringup_launch.py alone is still the mapping stack WITHOUT the base,
for when skuba_bringup is already running somewhere else.

In Gazebo (skuba_bringup sim:=true brings the simulated robot up instead of
the drivers; everything runs on use_sim_time):

    ros2 launch skuba_navigation cheesecake_slam.launch.py sim:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory("skuba_navigation")
    bringup_share = get_package_share_directory("skuba_bringup")

    sim = LaunchConfiguration("sim")
    args = [
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Gazebo robot (skuba_bringup sim:=true) instead "
            "of the hardware; everything on use_sim_time.",
        ),
        DeclareLaunchArgument(
            "sim_gui",
            default_value="true",
            description="With sim:=true: open the Gazebo window.",
        ),
        DeclareLaunchArgument(
            "launch_robot_state_publisher",
            default_value="true",
            description="skuba_bringup's robot_state_publisher, the only one "
            "on the robot (manipulation runs with_bringup:=true).",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(nav_share, "params", "slam_params.yaml"),
            description="SLAM Toolbox parameters file (mapping mode)",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz (on the robot's own display). false in a "
            "container with no display.",
        ),
        DeclareLaunchArgument(
            "launch_bringup",
            default_value="true",
            description="Include skuba_bringup. false when the robot (or the "
            "Gazebo robot) is already up in its own container (skuba_robot).",
        ),
    ]

    # 1. Hardware + complete-robot state publisher
    cheesecake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "bringup.launch.py")),
        launch_arguments={
            "sim": sim,
            "sim_gui": LaunchConfiguration("sim_gui"),
            "launch_robot_state_publisher": LaunchConfiguration(
                "launch_robot_state_publisher"
            ),
            # Mapping is lidar-only.
            "launch_camera": "true",
            # The xbox pad goes through twist_mux (joystick lane, priority 100)
            # instead of straight onto /cmd_vel, so the e_stop lock stops it too.
            "teleop_cmd_vel_topic": "cmd_vel_joy",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_bringup")),
    )

    # 2. SLAM Toolbox + nav2 + twist_mux + e-stop + RViz
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "nav2_slam_bringup_launch.py")
        ),
        launch_arguments={
            # Pass explicitly: launch configurations are global, and a caller's
            # own `slam` would otherwise leak in.
            "slam": "True",
            "use_sim_time": sim,
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            # On by default: it pops up on the robot's own display (DISPLAY=:1).
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    return LaunchDescription(
        [
            *args,
            # hardware first
            cheesecake_bringup,
            # SLAM after the base/lidar/RSP are up, so its first scans have TF
            TimerAction(period=5.0, actions=[slam]),
        ]
    )
