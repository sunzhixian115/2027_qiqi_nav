import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    default_config = os.path.join(
        get_package_share_directory("odin_nav_tools"), "config", "odin_extrinsics.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="odin_nav_tools",
            executable="odin_static_tf.py",
            name="odin_static_tf",
            output="screen",
            parameters=[config_file]),
    ])
