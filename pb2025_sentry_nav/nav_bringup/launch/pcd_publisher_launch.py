import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('pb2025_nav_bringup')
    return LaunchDescription([
        Node(
            package='pb2025_nav_bringup',
            executable='pcd_publisher',
            name='pcd_publisher',
            output='screen',
            parameters=[]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_dir, 'rviz', 'pcd_display.rviz')],
            output='screen'
        )
    ])
