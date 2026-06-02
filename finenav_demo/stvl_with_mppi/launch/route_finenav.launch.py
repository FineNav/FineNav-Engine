#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo')
    finenav_route_pkg_dir = get_package_share_directory('finenav_route')

    mppi_params_file = os.path.join(
        finenav_demo_pkg_dir, 'config', 'mppi_params.yaml')
    finenav_params_file = os.path.join(
        finenav_demo_pkg_dir, 'config', 'finenav_params.yaml')
    obs_cov_file = os.path.join(
        finenav_demo_pkg_dir, 'config', 'obs_cov.yaml')

    default_graph_dir = os.path.join(
        finenav_route_pkg_dir, 'config', 'graphs', 'yunjing')

    declare_graph_dir = DeclareLaunchArgument(
        'graph_dir',
        default_value=default_graph_dir,
        description='Directory containing route graph JSON files.')

    declare_use_current_pose_as_start = DeclareLaunchArgument(
        'use_current_pose_as_start',
        default_value='true',
        description='If true, bridge finds nearest route node to current pose as start.')

    start_route_finenav = Node(
        package='finenav_demo',
        executable='route_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            mppi_params_file,
            finenav_params_file,
            obs_cov_file,
        ],
    )

    start_compute_path_bridge = Node(
        package='finenav_route',
        executable='compute_path_bridge_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'action_name': '/compute_path',
            'navigator_action_name': '/route_finenav/navigate_to_pose',
            'navigator_behavior_tree': 'RouteNavigateToPose',
            'graph_dir': LaunchConfiguration('graph_dir'),
            'path_topic': '/route_finenav/line_path',
            'nav_state_topic': '/nav_state',
            'use_current_pose_as_start': LaunchConfiguration('use_current_pose_as_start'),
            'connector_density': 0.05,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_graph_dir)
    ld.add_action(declare_use_current_pose_as_start)
    ld.add_action(start_route_finenav)
    ld.add_action(start_compute_path_bridge)

    return ld
