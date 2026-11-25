import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # base_link -> lidar_link (from URDF: lidar_joint origin)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_link',
            arguments=['-8.2e-5', '0.0015', '0.159', '1.5708', '0', '0', 'base_link', 'lidar_link'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # lidar_link -> laser (from URDF: laser_joint origin, undoes lidar_link roll)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_laser',
            arguments=['0', '0', '0', '-1.5708', '0', '0', 'lidar_link', 'laser'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])