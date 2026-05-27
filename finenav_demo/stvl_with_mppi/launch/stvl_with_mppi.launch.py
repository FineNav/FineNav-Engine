#  Copyright (c) 2026.
#  IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
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

    finenav_simulator_pkg_dir = get_package_share_directory('finenav_simulator')
    
    # Only Gazebo and robot model infrastructure
    simulator_infrastructure_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'simulator_infrastructure.launch.py'
    )

    start_finenav_simulator_infra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulator_infrastructure_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'lio_type': 'static',
            'world_name': 'flat_small_scale_maze.world',
        }.items()
    )

    # Navigation Stack launch file (managed by manager)
    navigation_stack_launch = '/home/steven/FineNav-Engine/finenav_demo/stvl_with_mppi/launch/navigation_stack.launch.py'

    # Navigation Manager Node
    nav_manager_script = '/home/steven/FineNav-Engine/finenav_demo/stvl_with_mppi/src/nav_manager.py'
    
    start_nav_manager = Node(
        executable='python3',
        arguments=[nav_manager_script],
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'launch_file': navigation_stack_launch}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(start_finenav_simulator_infra)
    ld.add_action(start_nav_manager)

    return ld