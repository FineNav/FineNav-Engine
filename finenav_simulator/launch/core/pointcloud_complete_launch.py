import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    generate_robot_sdf,
    load_robot_sdf_config_from_string,
)


def generate_launch_description():
    pkg = get_package_share_directory("finenav_simulator")
    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xacro")
    lidar_support_mesh_uri = f"file://{os.path.join(pkg, 'resource', 'robot', 'meshes', 'lidar_support_tri_prism.obj')}"
    robot_xml = generate_robot_sdf(
        sdf_xmacro_path,
        mappings={"lidar_support_mesh_uri": lidar_support_mesh_uri},
    )
    sdf_config = load_robot_sdf_config_from_string(robot_xml)

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # The lidar_frame -> SDF-scoped sensor frame alias used to be published
    # here; it now comes from simulator_helper_node (SensorFrameAliasPublisher).
    converter_node = Node(
        package='finenav_simulator',
        executable='point_cloud_converter_node',
        name='point_cloud_converter',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'pcd_topic': '/livox/lidar'},
            # Realistic-lidar output for external localization algorithms
            # (ring, per-point time, self-filter), transformed to base_link.
            {'output_topic': '/lidar/points_filtered'},
            {'output_frame_id': 'base_link'},
            {'transform_output_to_frame': True},
            {'vertical_samples': sdf_config['lidar_vertical_samples']},
            {'horizontal_samples': sdf_config['lidar_horizontal_samples']},
            {'vertical_min_angle': sdf_config['lidar_vertical_min_angle']},
            {'vertical_max_angle': sdf_config['lidar_vertical_max_angle']},
            {'horizontal_min_angle': sdf_config['lidar_horizontal_min_angle']},
            {'horizontal_max_angle': sdf_config['lidar_horizontal_max_angle']},
            {'scan_period': sdf_config['lidar_scan_period']},
            {'synthesize_time': True},
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        converter_node,
    ])
