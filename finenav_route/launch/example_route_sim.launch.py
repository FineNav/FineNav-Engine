import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route_pkg_dir = get_package_share_directory("finenav_route")
    demo_pkg_dir = get_package_share_directory("finenav_demo")
    simulator_pkg_dir = get_package_share_directory("finenav_simulator")

    requested_graph_file = (
        "/home/slam/FineNav_ws/src/FineNav-Engine/"
        "finenav_route/test/test_graphs/rectangle_centered.json"
    )
    default_graph_file = requested_graph_file if os.path.isfile(requested_graph_file) else os.path.join(
        route_pkg_dir,
        "test",
        "test_graphs",
        "rectangle_centered.json",
    )

    simulator_launch_file = os.path.join(
        simulator_pkg_dir,
        "launch",
        "complete",
        "finenav_simulator.launch.py",
    )

    mppi_params_file = os.path.join(
        demo_pkg_dir,
        "config",
        "mppi_params.yaml",
    )
    finenav_params_file = os.path.join(
        demo_pkg_dir,
        "config",
        "finenav_params.yaml",
    )

    declare_graph_file = DeclareLaunchArgument(
        "graph_file",
        default_value=default_graph_file,
        description="Route graph JSON file to send in the ComputePath goal.",
    )
    declare_start_node_id = DeclareLaunchArgument(
        "start_node_id",
        default_value="1",
        description="Fallback start node id. The bridge uses current pose as start by default.",
    )
    declare_goal_node_id = DeclareLaunchArgument(
        "goal_node_id",
        default_value="5",
        description="Goal node id in the route graph.",
    )
    declare_planner_id = DeclareLaunchArgument(
        "planner_id",
        default_value="finenav_mppi_controller",
        description="Planner id forwarded to the MPPI control layer.",
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="flat_small_scale_maze.world",
        description="Gazebo world file name.",
    )
    declare_lio_type = DeclareLaunchArgument(
        "lio_type",
        default_value="gazebo_odom",
        description="Localization source for simulation.",
    )
    declare_send_goal_delay = DeclareLaunchArgument(
        "send_goal_delay",
        default_value="8.0",
        description="Seconds to wait before sending the route ComputePath goal.",
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

    start_hello_finenav = Node(
        package="finenav_demo",
        executable="hello_finenav",
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
            "controller_action_name": "/hello_finenav/mppi_layer",
            "path_topic": "/hello_finenav/line_path",
            "nav_state_topic": "/nav_state",
            "use_current_pose_as_start": LaunchConfiguration("use_current_pose_as_start"),
            "connector_density": 0.05,
        }],
    )

    send_route_goal = TimerAction(
        period=LaunchConfiguration("send_goal_delay"),
        actions=[
            Node(
                package="finenav_route",
                executable="send_compute_path_goal.py",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "action_name": "/compute_path",
                    "graph_file": LaunchConfiguration("graph_file"),
                    "start_node_id": LaunchConfiguration("start_node_id"),
                    "goal_node_id": LaunchConfiguration("goal_node_id"),
                    "planner_id": LaunchConfiguration("planner_id"),
                }],
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_graph_file)
    ld.add_action(declare_start_node_id)
    ld.add_action(declare_goal_node_id)
    ld.add_action(declare_planner_id)
    ld.add_action(declare_world_name)
    ld.add_action(declare_lio_type)
    ld.add_action(declare_send_goal_delay)
    ld.add_action(declare_use_current_pose_as_start)
    ld.add_action(start_simulator)
    ld.add_action(start_hello_finenav)
    ld.add_action(start_compute_path_bridge)
    ld.add_action(send_route_goal)

    return ld
