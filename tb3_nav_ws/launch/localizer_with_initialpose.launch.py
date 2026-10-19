"""Run the 3D PCD ICP localizer and the RViz /initialpose service bridge."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('pcd_map', description='Absolute path of the saved PCD map'),
        Node(
            package='localizer', executable='localizer_node',
            namespace='pcd_localizer', output='screen',
            parameters=[{
                'config_path': str(Path(get_package_share_directory('localizer')) / 'config/localizer.yaml'),
                'startup_pcd_path': LaunchConfiguration('pcd_map'),
                'use_sim_time': True,
            }],
        ),
        ExecuteProcess(
            cmd=['python3', str(root / 'tools/initialpose_to_pcd.py'),
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', ['pcd_path:=', LaunchConfiguration('pcd_map')]],
            output='screen',
        ),
    ])
