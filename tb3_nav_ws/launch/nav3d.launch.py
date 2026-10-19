"""PCD-derived Nav2 planner + live 3D costmap, without AMCL or wheel pose.

Requires the separate PCD ICP localizer to be valid. This launch intentionally
uses navigation_launch.py, NOT bringup_launch.py (which also launches AMCL).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ws = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    param = os.path.join(ws, 'config', 'nav2_3d.yaml')
    derived_map = os.path.join(ws, 'maps', 'tb3_lio_ground.yaml')
    nav = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=derived_map,
                              description='PCD-derived planning map YAML, aligned with the localization PCD'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lio_body_to_base',
             arguments=['--x', '0.032', '--y', '0', '--z', '-0.078',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'body', '--child-frame-id', 'base_footprint'],
             output='screen'),
        ExecuteProcess(cmd=['python3', os.path.join(ws, 'tools', 'lio_nav_odom.py')],
                       output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': True}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_pcd_map',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server']}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav),
            launch_arguments={'params_file': param, 'use_sim_time': 'true',
                              'autostart': 'true', 'use_composition': 'False'}.items()),
    ])
