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
    
    # This includes FAST-LIO, etc.
    slam_and_localization_launch_file = os.path.join(
        finenav_simulator_pkg_dir,
        'launch',
        'complete',
        'slam_and_localization.launch.py'
    )

    start_slam_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_and_localization_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'lio_type': 'static', # As in the original stvl_with_mppi.launch.py
            'publish_map_tf': 'true',
        }.items()
    )

    start_hello_finenav = Node(
        package='finenav_demo',
        executable='hello_finenav',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            mppi_params_file,
            finenav_params_file,
        ]
    )

    start_goal_pose_bridge = Node(
        package='finenav_demo',
        executable='goal_pose_bridge_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Also include the adapter we created
    adapter_script = '/home/steven/FineNav-Engine/finenav_demo/stvl_with_mppi/src/adapter.py'
    start_adapter = Node(
        executable='python3',
        arguments=[adapter_script],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld = LaunchDescription()
    ld.add_action(start_slam_and_localization)
    ld.add_action(start_hello_finenav)
    ld.add_action(start_goal_pose_bridge)
    ld.add_action(start_adapter)

    return ld
