"""One-terminal FAST-LIO2 + 3D PCD ICP + gated Nav2 Humble bringup.

Gazebo and RViz stay separate. Nav2 does not start until RViz initialpose
causes successful ICP and both the LIO and map TF are fresh. No AMCL.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ws = Path(__file__).resolve().parents[1]
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ws / 'launch/lio_odometry.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ws / 'launch/localizer_with_initialpose.launch.py')),
        launch_arguments={'pcd_map': LaunchConfiguration('pcd_map')}.items())
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
                    PythonLaunchDescriptionSource(str(ws / 'launch/nav3d.launch.py')),
                    launch_arguments={'map': LaunchConfiguration('map')}.items()),
            ]
        return [LogInfo(msg='Localization gate exited without a valid pose; Nav2 will NOT start')]

    return LaunchDescription([
        DeclareLaunchArgument('pcd_map', description='Absolute path of the saved PCD map'),
        DeclareLaunchArgument('map', default_value=str(ws / 'maps/tb3_lio_ground.yaml'),
                              description='Planning map YAML derived from the same PCD'),
        lio_launch,
        localizer_launch,
        readiness_gate,
        RegisterEventHandler(OnProcessExit(target_action=readiness_gate,
                                           on_exit=when_gate_exits)),
    ])
