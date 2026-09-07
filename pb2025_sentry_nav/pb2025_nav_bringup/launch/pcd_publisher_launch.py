from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            arguments=['-d', '/home/y/nav/blue/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/pcd_display.rviz'],
            output='screen'
        )
    ])