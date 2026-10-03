"""Run the 3D PCD ICP localizer and the RViz /initialpose service bridge."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        Node(
            package='localizer', executable='localizer_node',
            namespace='pcd_localizer', output='screen',
            parameters=[{
                'config_path': str(root / 'src/FASTLIO2_ROS2_reference/localizer/config/localizer.yaml'),
                'startup_pcd_path': str(root.parent / 'fast_lio2_ws/maps/tb3_fast_lio2.pcd'),
                'use_sim_time': True,
            }],
        ),
        ExecuteProcess(
            cmd=['python3', str(root / 'tools/initialpose_to_pcd.py'),
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),
    ])
