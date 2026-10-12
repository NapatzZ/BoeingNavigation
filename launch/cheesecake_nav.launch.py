#!/usr/bin/env python3
"""
Full Cheesecake navigation stack on the real robot.

Brings up, in order:
  1. hardware + complete-robot state publisher  (skuba_bringup)
  2. nav2                                         (map + params)
  3. RViz                                         (nav2_bringup rviz)
  4. service nodes: navigation_server, save_location, door_detector,
     costmap_reconfig_server

The person-follower is NOT here: it lives in the skuba-follower container (subsystem
`follow`), started only by the Follow-Me task.

    ros2 launch skuba_navigation cheesecake_nav.launch.py
    ros2 launch skuba_navigation cheesecake_nav.launch.py map:=/path/to/map.yaml

In Gazebo (skuba_bringup sim:=true brings the simulated robot up instead of
the drivers; everything runs on use_sim_time):

    ros2 launch skuba_navigation cheesecake_nav.launch.py sim:=true map:=<map.yaml>

Navigation only, against a robot another container already brought up
(skuba_robot: the hardware container runs skuba_bringup or Gazebo):

    ros2 launch skuba_navigation cheesecake_nav.launch.py launch_bringup:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_pkg = "skuba_navigation"
    bringup_pkg = "skuba_bringup"
    nav2_bringup_pkg = "nav2_bringup"

    nav_share = get_package_share_directory(nav_pkg)
    bringup_share = get_package_share_directory(bringup_pkg)

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            nav_share, "maps", "30_skuba_home", "skuba_home.yaml"
        ),
        description="Map yaml for nav2 map_server",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_share, "params", "rcj_param.yaml"),
        description="nav2 parameters file",
    )
    sim = LaunchConfiguration("sim")
    sim_args = [
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
            "use_rviz", default_value="true", description="Start RViz"
        ),
        DeclareLaunchArgument(
            "launch_bringup",
            default_value="true",
            description="Include skuba_bringup. false when the robot (or the "
            "Gazebo robot) is already up in its own container and this one only "
            "runs navigation on the same ROS_DOMAIN_ID (skuba_robot).",
        ),
    ]
    use_sim_time = {"use_sim_time": ParameterValue(sim, value_type=bool)}

    # 1. Hardware + complete-robot state publisher (skuba_bringup: kobuki, lidar, teleop, RSP)
    # skuba_bringup owns the one robot_state_publisher (whole URDF); the
    # manipulation stack runs with_bringup:=true and starts none, only
    # publishing the arm / pan-tilt /joint_states. Two at once was the cause
    # of move_to_tf resolving targets to the wrong place (TEAM2-011, 2026-08-13).
    cheesecake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "bringup.launch.py")),
        launch_arguments={
            "sim": sim,
            "sim_gui": LaunchConfiguration("sim_gui"),
            "launch_robot_state_publisher": LaunchConfiguration(
                "launch_robot_state_publisher"
            ),
            # The xbox pad goes through twist_mux (joystick lane, priority 100)
            # instead of straight onto /cmd_vel, so the e_stop lock stops it too.
            "teleop_cmd_vel_topic": "cmd_vel_joy",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_bringup")),
    )

    # 2. nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "nav2_bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": sim,
            # nav2_bringup_launch.py evaluates `slam` in a PythonExpression, so
            # it must be Python's False -- and pass it explicitly, since launch
            # configurations are global and a caller's own `slam` would leak in.
            "slam": "False",
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
        launch_arguments={"use_sim_time": sim}.items(),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # 4. Service nodes
    navigation_server = Node(
        package=nav_pkg,
        executable="navigation_server",
        name="navigation_server",
        output="screen",
        parameters=[
            os.path.join(nav_share, "params", "navigation_server.yaml"),
            use_sim_time,
        ],
    )
    save_location = Node(
        package=nav_pkg,
        executable="save_location",
        name="save_location",
        output="screen",
        parameters=[use_sim_time],
    )
    door_detector = Node(
        package=nav_pkg,
        executable="door_detector",
        name="door_detector",
        output="screen",
        parameters=[use_sim_time],
    )
    costmap_reconfig_server = Node(
        package=nav_pkg,
        executable="costmap_reconfig_server",
        name="costmap_reconfig_server",
        output="screen",
        parameters=[use_sim_time],
    )

    # following_server was launched here. It is gone deliberately.
    #
    # It advertised /skuba/cheesecake/nav/following -- the SAME service the CNN follower
    # in the skuba-follower container advertises. Both were running, so whichever
    # registered last silently won the name, and nothing anywhere told you which one was
    # actually answering a Follow-Me call.
    #
    # The follower is now its own subsystem (`follow` in system_manifest.yaml), brought up
    # only by the Follow-Me task. The service keeps its /nav/ name so callers do not change.

    return LaunchDescription(
        [
            map_arg,
            params_arg,
            *sim_args,
            # hardware first
            cheesecake_bringup,
            # nav2 after the base/lidar/RSP are up
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
                ],
            ),
        ]
    )
