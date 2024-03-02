import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("mmpsu_v2"), "config", "mmpsu_v2_config.yaml")
    return LaunchDescription(
        [
            Node(
                package="mmpsu_v2",
                namespace="mmpsu",
                executable="mmpsu_v2_node",
                name="mmpsu_v2_node",
                parameters=[config],
            )
        ]
    )
