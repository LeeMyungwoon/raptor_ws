import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("raptor_imu")

    vn200_node = Node(
        package="raptor_imu",
        executable="vn200_node",
        output="screen",
        parameters=[os.path.join(config_dir, "config", "vn200_imu.yaml")],
    )

    ld = LaunchDescription()
    ld.add_action(vn200_node)

    return ld
