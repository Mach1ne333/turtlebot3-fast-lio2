"""One-terminal FAST-LIO2 + 3D PCD ICP + gated Nav2 Humble bringup.

Gazebo and RViz stay separate. Nav2 does not start until RViz initialpose
causes successful ICP and both the LIO and map TF are fresh. No AMCL.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ws = Path(__file__).resolve().parents[1]
    lio_cfg = Path(get_package_share_directory('fast_lio')) / 'config' / 'tb3_gazebo_nav.yaml'
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ws / 'launch/localizer_with_initialpose.launch.py')))
    readiness_gate = ExecuteProcess(
        cmd=['python3', str(ws / 'tools/wait_for_pcd_localization.py'),
             '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )

    def when_gate_exits(event, context):
        if event.returncode == 0:
            return [
                LogInfo(msg='PCD ICP valid and TF fresh: automatically starting Nav2 map/planner/controller'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(ws / 'launch/nav3d.launch.py'))),
            ]
        return [LogInfo(msg='Localization gate exited without a valid pose; Nav2 will NOT start')]

    return LaunchDescription([
        Node(package='fast_lio', executable='fastlio_mapping',
             output='screen', parameters=[str(lio_cfg), {'use_sim_time': True}]),
        localizer_launch,
        readiness_gate,
        RegisterEventHandler(OnProcessExit(target_action=readiness_gate,
                                           on_exit=when_gate_exits)),
    ])
