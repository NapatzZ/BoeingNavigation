import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('boeing_description')
    urdf_file = os.path.join(pkg, 'urdf', 'boeing.urdf')
    rviz_config = os.path.join(pkg, 'rviz', 'urdf.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
