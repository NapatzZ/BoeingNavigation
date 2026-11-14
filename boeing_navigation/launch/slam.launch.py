   import os

   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       pkg = get_package_share_directory('boeing_navigation')
       slam_toolbox = get_package_share_directory('slam_toolbox')

       static_tf_base_to_base_link = Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           name='base_to_base_link_tf',
           arguments=['0', '0', '0', '0', '0', '0', 'base', 'base_link'],
       )

       return LaunchDescription([
           DeclareLaunchArgument('use_sim_time', default_value='false'),
           DeclareLaunchArgument(
               'slam_params_file',
               default_value=os.path.join(pkg, 'config', 'slam_toolbox.yaml')),
           static_tf_base_to_base_link,
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                   os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')),
               launch_arguments={
                   'use_sim_time': LaunchConfiguration('use_sim_time'),
                   'slam_params_file': LaunchConfiguration('slam_params_file'),
               }.items()),
       ])
