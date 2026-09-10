from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_synthetic_data = LaunchConfiguration("use_synthetic_data")
    odin_cloud = "/odin1/cloud_slam"
    odin_odom = "/odin1/odometry"
    remaps = [
        ("registered_scan", odin_cloud),
        ("lidar_odometry", odin_odom),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            description="Existing Nav2 parameter file used by terrain nodes."),
        DeclareLaunchArgument(
            "use_synthetic_data",
            default_value="true",
            description="Publish Odin-format synthetic data when hardware is not connected."),
        Node(
            package="odin_nav_tools",
            executable="odin_synthetic_source",
            output="screen",
            condition=IfCondition(use_synthetic_data)),
        Node(
            package="odin_nav_tools",
            executable="odin_cloud_compatibility_test",
            output="screen"),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrain_analysis",
            output="screen",
            parameters=[params_file],
            remappings=remaps),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name="terrain_analysis_ext",
            output="screen",
            parameters=[params_file],
            remappings=remaps),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            output="screen",
            parameters=[
                params_file,
                {
                    # Test data flow without asserting a physical vehicle mounting transform.
                    "lidar_frame": "odin1_base_link",
                    "base_frame": "odin1_base_link",
                    "robot_base_frame": "odin1_base_link",
                },
            ],
            remappings=remaps),
    ])
