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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # odom -> base_link (kobuki_node publish_tf disabled)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='kobuki_node',
            executable='kobuki_ros_node',
            output='both',
            parameters=[params, {'device_port': LaunchConfiguration('device_port')}],
            remappings=[('commands/velocity', '/cmd_vel')],
        ),
    ])
