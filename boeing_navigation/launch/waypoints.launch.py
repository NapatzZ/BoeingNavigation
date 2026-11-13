from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='/ros_ws/src/boeing_navigation/maps/waypoints.yaml',
            description='YAML file where named waypoints are stored. '
                        'Waypoints belong to ONE map: use one file per map.'),
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='boeing_navigation',
            executable='waypoint_manager',
            name='waypoint_manager',
            output='screen',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'global_frame': LaunchConfiguration('global_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])
