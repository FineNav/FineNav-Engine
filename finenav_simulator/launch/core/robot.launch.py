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

    robot_xml = generate_robot_sdf(
        sdf_xmacro_path,
        mappings={"lidar_support_mesh_uri": lidar_support_mesh_uri},
    )
    robot_urdf_xml = normalize_robot_urdf_for_rviz(generate_robot_urdf_from_sdf(robot_xml))
    sdf_config = load_robot_sdf_config_from_string(robot_xml)

    # The one bridge for the whole simulation: /clock plus the robot sensor and
    # control rules.  All ROS topic names in the config file are absolute, so no
    # namespace is needed.
    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
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

    lio_type = LaunchConfiguration("lio_type").perform(context)
    publish_map_tf = LaunchConfiguration("publish_map_tf").perform(context)
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context)

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
            # map->odom identity: ground-truth modes always publish it; fast_lio
            # only when the caller explicitly asks for a map frame in dynamic mode.
            "enable_identity_map_to_odom":
                lio_type != "fast_lio" or (publish_map_tf == "true" and navigation_mode == "dynamic"),
            # Sensor frames arrive from Gazebo under SDF-scoped names; the
            # aliases connect them to the plain URDF link names the ROS
            # ecosystem uses (see SensorFrameAliasPublisher).
            "imu_alias_child_frame": f"{robot_name}/{sdf_config['imu_parent_link']}/{sdf_config['imu_sensor_name']}",
            "lidar_alias_child_frame": f"{robot_name}/{sdf_config['lidar_parent_link']}/{sdf_config['lidar_sensor_name']}",
        }],
    )

    return [
        robot_ign_bridge,
        robot_state_publisher,
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
    ld.add_action(
        DeclareLaunchArgument(
            "publish_map_tf",
            default_value="false",
            description="Enable publishing map to odom TF transform",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="dynamic",
            description="Choose the mode of navigation: dynamic or static",
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
