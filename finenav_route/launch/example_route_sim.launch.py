import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route_pkg_dir = get_package_share_directory("finenav_route")
    demo_pkg_dir = get_package_share_directory("finenav_demo")
    simulator_pkg_dir = get_package_share_directory("finenav_simulator")

    requested_graph_dir = (
        "/home/slam/FineNav_ws/src/FineNav-Engine/"
        "finenav_route/config/graphs"
    )
    default_graph_dir = requested_graph_dir if os.path.isdir(
        requested_graph_dir
    ) else os.path.join(route_pkg_dir, "config", "graphs")

    simulator_launch_file = os.path.join(
        simulator_pkg_dir,
        "launch",
        "complete",
        "finenav_simulator.launch.py",
    )

    mppi_params_file = os.path.join(
        demo_pkg_dir,
        "config",
        "route_mppi_params.yaml",
    )
    finenav_params_file = os.path.join(
        demo_pkg_dir,
        "config",
        "route_finenav_params.yaml",
    )

    declare_graph_dir = DeclareLaunchArgument(
        "graph_dir",
        default_value=default_graph_dir,
        description="Directory containing route graph JSON files.",
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="empty.world",
        description="Gazebo world file name.",
    )
    declare_lio_type = DeclareLaunchArgument(
        "lio_type",
        default_value="gazebo_odom",
        description="Localization source for simulation.",
    )
    declare_use_current_pose_as_start = DeclareLaunchArgument(
        "use_current_pose_as_start",
        default_value="true",
        description=(
            "If true, the bridge ignores start_node_id and starts from the "
            "nearest route node to the current robot pose."
        ),
    )

    start_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulator_launch_file),
        launch_arguments={
            "use_sim_time": "true",
            "lio_type": LaunchConfiguration("lio_type"),
            "world_name": LaunchConfiguration("world_name"),
            "publish_map_tf": "true",
        }.items(),
    )

    start_route_finenav = Node(
        package="finenav_demo",
        executable="route_finenav",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            mppi_params_file,
            finenav_params_file,
        ],
    )

    start_compute_path_bridge = Node(
        package="finenav_route",
        executable="compute_path_bridge_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "action_name": "/compute_path",
            "navigator_action_name": "/route_finenav/navigate_to_pose",
            "navigator_behavior_tree": "RouteNavigateToPose",
            "graph_dir": LaunchConfiguration("graph_dir"),
            "path_topic": "/route_finenav/line_path",
            "nav_state_topic": "/nav_state",
            "use_current_pose_as_start": LaunchConfiguration("use_current_pose_as_start"),
            "connector_density": 0.05,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_graph_dir)
    ld.add_action(declare_world_name)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_use_current_pose_as_start)
    ld.add_action(start_simulator)
    ld.add_action(start_route_finenav)
    ld.add_action(start_compute_path_bridge)

    return ld
