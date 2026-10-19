"""Navigation-only FAST-LIO odometry; no global map saving or mapping RViz."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             parameters=[str(ws / 'config/tb3_gazebo_nav.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
