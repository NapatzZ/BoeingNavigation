import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('boeing_bringup'), 'launch')

    def include(name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=(args or {}).items())

    return LaunchDescription([
        DeclareLaunchArgument('kobuki_port', default_value='/dev/kobuki'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),
        include('description.launch.py'),
        include('base.launch.py', {'device_port': LaunchConfiguration('kobuki_port')}),
        include('lidar.launch.py', {'serial_port': LaunchConfiguration('lidar_port')}),
    ])
