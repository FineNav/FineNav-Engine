import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    build_tf_args,
    generate_robot_sdf,
    generate_robot_urdf_from_sdf,
    load_robot_sdf_config_from_string,
    normalize_robot_urdf_for_rviz,
)


def _launch_setup(context):
    pkg = get_package_share_directory("finenav_simulator")

    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xacro")
    lidar_support_mesh_uri = f"file://{os.path.join(pkg, 'resource', 'robot', 'meshes', 'lidar_support_tri_prism.obj')}"
    bridge_config = os.path.join(pkg, "config", "ros_gz_bridge.yaml")

    robot_name = "fine_robot"
    world_name = LaunchConfiguration("world_name").perform(context)

    robot_xml = generate_robot_sdf(
        sdf_xmacro_path,
        mappings={"lidar_support_mesh_uri": lidar_support_mesh_uri},
    )
    robot_urdf_xml = normalize_robot_urdf_for_rviz(generate_robot_urdf_from_sdf(robot_xml))
    sdf_config = load_robot_sdf_config_from_string(robot_xml)

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": bridge_config}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_description": robot_urdf_xml,
        }],
    )

    imu_sensor_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_sensor_frame_alias",
        arguments=build_tf_args(
            "imu",
            f"{robot_name}/{sdf_config['imu_parent_link']}/{sdf_config['imu_sensor_name']}",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    simulator_helper = Node(
        package="finenav_simulator",
        executable="simulator_helper_node",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "input_odom_topic": PythonExpression(["'/Odometry' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else '/odom'"]),
            "path_topic": "/path",
            "max_path_points": 2000,
            "publish_tf": PythonExpression(["'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"]),
            "path_odom_frame": "odom",
            "path_base_frame": "base_link",
            "enable_base_footprint": True,
            "base_footprint_frame": "base_footprint",
            "footprint_ground_offset": 0.0,
            "teleop_output_topic": "/cmd_vel",
            "world_name": "default",
            "enable_path_publisher": True,
            "enable_identity_map_to_odom": PythonExpression([
                "'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"
            ]),
        }],
    )

    return [
        robot_ign_bridge,
        robot_state_publisher,
        imu_sensor_alias,
        simulator_helper,
    ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "lio_type",
            default_value="fast_lio",
            description="Choose localization chain: fast_lio or gazebo_odom",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_name",
            default_value="flat_small_scale_maze.world",
            description="World file name under finenav_simulator/resource/worlds",
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
