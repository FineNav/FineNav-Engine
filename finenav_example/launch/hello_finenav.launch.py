from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = (
        Path(get_package_share_directory("finenav_example"))
        / "config"
        / "hello_finenav.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="finenav_example",
                executable="hello_finenav",
                output="screen",
                parameters=[str(config_file)],
            )
        ]
    )
