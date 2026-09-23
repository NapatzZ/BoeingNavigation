#!/usr/bin/env python3
"""Full Cheesecake navigation stack on the real robot.

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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = 'skuba_navigation'
    bringup_pkg = 'skuba_bringup'
    nav2_bringup_pkg = 'nav2_bringup'

    nav_share = get_package_share_directory(nav_pkg)
    bringup_share = get_package_share_directory(bringup_pkg)

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_share, 'maps', '30_skuba_home', 'skuba_home.yaml'),
        description='Map yaml for nav2 map_server')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(nav_share, 'params', 'rcj_param.yaml'),
        description='nav2 parameters file')

    # 1. Hardware + complete-robot state publisher (skuba_bringup: kobuki, lidar, teleop, RSP)
    # launch_robot_state_publisher:=false -- the manipulation stack's
    # move_group.launch.py already runs its own robot_state_publisher from the
    # same URDF (plus the live arm joints this one never gets). Leaving both
    # on puts two identically-named robot_state_publisher nodes on /tf at
    # once, which was diagnosed as the cause of move_to_tf intermittently
    # resolving targets to the wrong place (TEAM2-011, 2026-08-13).
    cheesecake_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'bringup.launch.py')
        ),
        launch_arguments={
            'launch_robot_state_publisher': 'false',
            # skuba_bringup starts the RealSense by default; the old
            # cheesecake_bringup never did, and nav does not need it.
            'launch_camera': 'false',
        }.items(),
    )

    # 2. nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'nav2_bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # 3. RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_bringup_pkg), 'launch', 'rviz_launch.py')
        )
    )

    # 4. Service nodes
    navigation_server = Node(
        package=nav_pkg, executable='navigation_server', name='navigation_server',
        parameters=[os.path.join(nav_share, 'params', 'navigation_server.yaml')],
        output='screen')
    save_location = Node(
        package=nav_pkg, executable='save_location', name='save_location', output='screen')
    door_detector = Node(
        package=nav_pkg, executable='door_detector', name='door_detector', output='screen')
    costmap_reconfig_server = Node(
        package=nav_pkg, executable='costmap_reconfig_server',
        name='costmap_reconfig_server', output='screen')

    # following_server was launched here. It is gone deliberately.
    #
    # It advertised /skuba/cheesecake/nav/following -- the SAME service the CNN follower
    # in the skuba-follower container advertises. Both were running, so whichever
    # registered last silently won the name, and nothing anywhere told you which one was
    # actually answering a Follow-Me call.
    #
    # The follower is now its own subsystem (`follow` in system_manifest.yaml), brought up
    # only by the Follow-Me task. The service keeps its /nav/ name so callers do not change.

    return LaunchDescription([
        map_arg,
        params_arg,
        # hardware first
        cheesecake_bringup,
        # nav2 after the base/lidar/RSP are up
        TimerAction(period=5.0, actions=[nav2]),
        # rviz + service nodes after nav2 is up
        TimerAction(period=12.0, actions=[
            rviz,
            navigation_server,
            save_location,
            door_detector,
            costmap_reconfig_server,
        ]),
    ])
