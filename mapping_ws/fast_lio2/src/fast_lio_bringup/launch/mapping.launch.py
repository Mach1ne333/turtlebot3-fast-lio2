"""FAST-LIO2 integration: installed config, explicit output map path."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value='tb3_gazebo_instant.yaml'),
        DeclareLaunchArgument('map_file_path',
                              description='PCD output path; create its parent directory first'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             parameters=[PathJoinSubstitution([
                 str(share / 'config'), LaunchConfiguration('config_file')]),
                 {'use_sim_time': LaunchConfiguration('use_sim_time'),
                  'map_file_path': LaunchConfiguration('map_file_path')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/lio_rviz.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
