#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    finenav_demo_pkg_dir = get_package_share_directory('finenav_demo')
    mppi_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'mppi_params.yaml'
    )
    finenav_params_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'finenav_params.yaml'
    )
    localizer_cov_file = os.path.join(
        finenav_demo_pkg_dir,
        'config',
        'localizer_covariances.yaml'
    )

    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator')
    finenav_simulator_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'finenav_simulator.launch.py'
    )

    start_finenav_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(finenav_simulator_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lio_type': 'static',
            'world_name': 'flat_small_scale_maze.world',
            'publish_map_tf': 'true',
        }.items()
    )

    start_hello_finenav = Node(
        package='finenav_demo',
        executable='hello_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            mppi_params_file,
            finenav_params_file,
            localizer_cov_file,
        ]
    )

    start_goal_pose_bridge = Node(
        package='finenav_demo',
        executable='goal_pose_bridge_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    # ld.add_action(start_finenav_simulator)
    ld.add_action(start_hello_finenav)
    ld.add_action(start_goal_pose_bridge)

    return ld