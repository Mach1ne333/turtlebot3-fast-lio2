# Copyright 2026 lhd
# Licensed under the Apache License, Version 2.0.
"""AMCL + Nav2 only: start the 2D Gazebo robot separately, supply your own map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


WS = Path(__file__).resolve().parents[1]


def _require_file(path, label):
    if not str(path).strip():
        raise RuntimeError(f'{label} is required; supply map:=/path/to/map.yaml')
    candidate = Path(path).expanduser().resolve()
    if not candidate.is_file():
        raise RuntimeError(f'{label} does not exist: {candidate}')
    return candidate


def _validate_map_assets(path):
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise RuntimeError(f'Map YAML has no image path: {path}')
    _require_file(path.parent / data['image'], 'map image')


def _validate_fast_planner_chain(data):
    # Retained from the original 2D integration: reject incompatible overrides.
    controller = data['controller_server']['ros__parameters']
    plugin = controller['FollowPath']
    smoother = data['velocity_smoother']['ros__parameters']
    period = 1.0 / controller['controller_frequency']
    if not 0 < plugin['final_check_reserve'] < plugin['budget'] < period:
        raise RuntimeError('Fast Planner budget must fit the control period')
    delay = 1.0 / smoother['smoothing_frequency'] + smoother['odom_duration']
    if plugin['reaction_time'] < period + plugin['budget'] + delay:
        raise RuntimeError('reaction_time does not cover the configured output chain')
    if (smoother['feedback'] != 'CLOSED_LOOP' or smoother['scale_velocities'] or
            not 0 < smoother['velocity_timeout'] <= 0.2 or
            smoother['max_accel'] != [plugin['accel_v'], 0.0, plugin['accel_w']] or
            smoother['max_decel'] != [-plugin['decel_v'], 0.0, -plugin['decel_w']] or
            smoother['max_velocity'] != [plugin['max_v'], 0.0, plugin['max_w']]):
        raise RuntimeError('Velocity smoother does not match the motion safety model')


def _select_params(mode, override=''):
    plugins = {'dwb': 'dwb_core::DWBLocalPlanner',
               'fast_planner': 'fast_planner_nav2::FastPlannerController'}
    if mode not in plugins:
        raise RuntimeError(f'Unsupported controller_mode: {mode}')
    path = _require_file(override or WS / 'config/2d' / f'nav2_{mode}.yaml',
                         'Nav2 parameter file')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    controller = data['controller_server']['ros__parameters']
    if (controller['controller_plugins'] != ['FollowPath'] or
            controller['FollowPath']['plugin'] != plugins[mode]):
        raise RuntimeError('params_file does not match controller_mode')
    if mode == 'fast_planner':
        _validate_fast_planner_chain(data)
    return path


def _setup(context):
    if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
        raise RuntimeError('This simulation entry requires use_sim_time:=true')
    params = _select_params(LaunchConfiguration('controller_mode').perform(context),
                            LaunchConfiguration('params_file').perform(context))
    map_path = _require_file(LaunchConfiguration('map').perform(context), 'map YAML')
    _validate_map_assets(map_path)
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/bringup_launch.py')),
            launch_arguments={
                'map': str(map_path), 'params_file': str(params),
                'use_sim_time': 'true', 'slam': 'False',
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': 'False', 'use_respawn': 'False',
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(nav2 / 'rviz/nav2_default_view.rviz')],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Required user map YAML; no default example map'),
        DeclareLaunchArgument('controller_mode', default_value='dwb',
                              choices=['dwb', 'fast_planner']),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Optional YAML matching controller_mode'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
