import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    nav_share = get_package_share_directory("skuba_navigation")
    params_path = os.path.join(nav_share, "params", "nav2_params.yaml")
    slam_params_path = os.path.join(nav_share, "params", "slam_params.yaml")
    twist_mux_params_path = os.path.join(nav_share, "params", "twist_mux.yaml")
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )

    # twist_mux is an optional external package. When it is missing the whole
    # launch aborted; skip just its node so the rest of the stack still comes up.
    try:
        get_package_share_directory("twist_mux")
        have_twist_mux = True
    except PackageNotFoundError:
        have_twist_mux = False

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    publish_laser_tf = LaunchConfiguration("publish_laser_tf")
    use_rviz = LaunchConfiguration("use_rviz")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )

    declare_map_name_cmd = DeclareLaunchArgument(
        "map_name",
        default_value="",
        description="Map name for SLAM Toolbox deserialize_map service",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=params_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_path,
        description="Full path to the SLAM Toolbox parameters file (mapping mode)",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_publish_laser_tf_cmd = DeclareLaunchArgument(
        "publish_laser_tf",
        default_value="true",
        description="Publish the measured static base_link->laser. Set false once "
        "skuba_description's laser_joint is corrected, so only one source publishes it.",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Start RViz"
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            # Include SLAM Toolbox's own online async launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                condition=IfCondition(slam),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params_file,
                }.items(),
            ),
            # Navigation launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            # RViz launch. use_rviz:=false for headless runs: rviz_launch.py
            # shuts the whole launch down when RViz exits (e.g. no display).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rviz_launch.py")
                ),
                condition=IfCondition(use_rviz),
            ),
            # On by default: this is the measured laser mount, while
            # skuba_description's laser_joint is currently wrong. When both
            # publish, the laser frame flickers between them -- fix the URDF,
            # then launch with publish_laser_tf:=false.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_tf",
                arguments=[
                    "0",
                    "0.002",
                    "0.179",
                    "3.145920",
                    "0",
                    "0",
                    "base_link",
                    "laser",
                ],
                output="screen",
                condition=IfCondition(publish_laser_tf),
            ),
            Node(
                package="skuba_navigation",
                executable="filter_lidar",
                name="filter_lidar",
                output="screen",
            ),
            Node(
                package="skuba_navigation",
                executable="emergency_stop",
                name="emergency_stop",
                output="screen",
            ),
            # twist_mux: arbitrates cmd_vel_joy / cmd_vel_key / cmd_vel_nav
            # (+ e_stop lock) into the single /cmd_vel the base drives, so
            # teleop during a mapping session actually reaches the robot and
            # can be software-e-stopped.
            *(
                [
                    Node(
                        package="twist_mux",
                        executable="twist_mux",
                        name="twist_mux",
                        output="screen",
                        parameters=[str(twist_mux_params_path)],
                        remappings=[("cmd_vel_out", "cmd_vel")],
                    ),
                ]
                if have_twist_mux
                else []
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_name_cmd)  # Changed from declare_map_yaml_cmd
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_publish_laser_tf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
