import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('boeing_bringup'), 'config', 'kobuki_node.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device_port', default_value='/dev/kobuki',
            description='Kobuki serial device'),
        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            output='both',
            parameters=[params, {'device_port': LaunchConfiguration('device_port')}],
            remappings=[('commands/velocity', '/cmd_vel')],
        ),
    ])
