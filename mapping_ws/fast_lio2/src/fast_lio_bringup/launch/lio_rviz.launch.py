"""Start the display-only height filter together with FAST-LIO's RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio_bringup'))
    display_filter = Node(
        package='fast_lio_bringup', executable='filter_map_display.py',
        arguments=['--ros-args', '-p', ['min_z:=', LaunchConfiguration('min_z')]],
        output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', name='lio_mapping_rviz',
        arguments=['-d', str(share / 'rviz/tb3_fast_lio2.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument(
            'min_z', default_value='-0.25',
            description='Display cutoff in camera_init (m); not ground-relative'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use false for a saved PCD without a simulator/bag clock'),
        RegisterEventHandler(OnProcessExit(
            target_action=display_filter,
            on_exit=[EmitEvent(event=Shutdown(reason='Display filter exited'))])),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
        display_filter,
        rviz,
    ])
