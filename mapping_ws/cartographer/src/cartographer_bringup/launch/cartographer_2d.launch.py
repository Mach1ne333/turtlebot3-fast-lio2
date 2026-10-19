"""Mapping only: use the separately running 2D simulator and its odom TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('cartographer_bringup'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', str(share / 'config'),
                   '-configuration_basename', 'cartographer_2d.lua'],
        remappings=[('scan', '/scan'), ('odom', '/odom')])
    grid = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        remappings=[('map', '/map')])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        RegisterEventHandler(OnProcessExit(
            target_action=cartographer,
            on_exit=[EmitEvent(event=Shutdown(reason='Cartographer exited'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=grid,
            on_exit=[EmitEvent(event=Shutdown(reason='Occupancy grid publisher exited'))])),
        cartographer,
        grid,
        Node(package='rviz2', executable='rviz2', name='mapping_rviz',
             arguments=['-d', str(share / 'rviz/cartographer_2d.rviz')],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
