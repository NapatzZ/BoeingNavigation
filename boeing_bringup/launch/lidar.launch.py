import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('boeing_bringup'), 'config', 'sllidar_a3.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/rplidar',
            description='RPLIDAR serial device'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[params, {'serial_port': LaunchConfiguration('serial_port')}],
        ),
    ])
