from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    serial_dir = get_package_share_directory('serial_communication')
    red_points_file = os.path.join(serial_dir, 'config', 'red_points.yaml')

    # 雷达检测已禁用：当前工作空间没有 radar_bringup 包。
    # radar_bringup_dir = get_package_share_directory('radar_bringup')
    # radar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(radar_bringup_dir, 'launch', 'mid360_detect.launch.py')
    #     ),
    #     launch_arguments={'use_rviz': 'false'}.items()
    # )

    # 优先使用workspace的libserial.so，避免与系统libserial-dev冲突
    serial_lib_dir = os.path.join(
        os.path.dirname(os.path.dirname(serial_dir)), 'lib'
    )

    nodes = [
        SetEnvironmentVariable('LD_LIBRARY_PATH', serial_lib_dir + ':' + os.environ.get('LD_LIBRARY_PATH', '')),
        Node(
            package='serial_communication',
            executable='red',
            name='referee_system_node',
            output='screen',
            parameters=[red_points_file]
        ),
        # Node(
        #     package='serial_communication',
        #     executable='nav_goal_guard',
        #     name='nav_goal_guard',
        #     output='screen',
        #     parameters=[{'goal_timeout': 1.0}]
        # ),
    ]

    # region_monitor 和 chase 包可能没编译，有就加
    try:
        region_dir = get_package_share_directory('pb2025_region_monitor')
        red_region_file = os.path.join(region_dir, 'config', 'red.yaml')
        nodes.append(Node(
            package='pb2025_region_monitor',
            executable='region_monitor_node',
            name='region_monitor_node',
            output='screen',
            parameters=[red_region_file]
        ))
    except Exception:
        pass

    try:
        chase_dir = get_package_share_directory('chase')
        chase_params = os.path.join(chase_dir, 'config', 'chase.yaml')
        nodes.append(Node(
            package='chase',
            executable='calculate_attack_pose',
            name='chase',
            output='screen',
            parameters=[chase_params],
        ))
    except Exception:
        pass

    return LaunchDescription(nodes)
