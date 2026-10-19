"""Shared 2D/3D Burger simulation for the tb3_sim package."""

import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _prepare_ekf_model(model):
    """Adapt only the drive outputs, never modify the official SDF on disk."""
    tree = ET.parse(model)
    drives = tree.findall(".//plugin[@filename='libgazebo_ros_diff_drive.so']")
    if len(drives) != 1:
        raise RuntimeError('2D EKF requires exactly one Gazebo differential drive plugin')
    drive = drives[0]
    flag = drive.find('publish_odom_tf')
    if flag is None:
        flag = ET.SubElement(drive, 'publish_odom_tf')
    flag.text = 'false'
    ros = drive.find('ros')
    if ros is None:
        ros = ET.SubElement(drive, 'ros')
    # ROS 2 gazebo_ros_diff_drive publishes on 'odom'; use a remap, not the
    # legacy odometry_topic SDF field, which is ignored by some ROS 2 versions.
    for remap in list(ros.findall('remapping')):
        if (remap.text or '').split(':=')[0].strip() in ('odom', '/odom'):
            ros.remove(remap)
    ET.SubElement(ros, 'remapping').text = 'odom:=/wheel/odom'
    with tempfile.NamedTemporaryFile(prefix='tb3_ekf_', suffix='.sdf', delete=False) as stream:
        # Humble spawn_entity passes Unicode to lxml, which rejects encoding
        # declarations; the SDF fragment must omit the XML declaration.
        tree.write(stream, encoding='utf-8', xml_declaration=False)
        return Path(stream.name)


def _floor_height(world_path, x, y):
    """Read the designated floor/ramp, not obstacles, to choose a spawn height."""
    world = ET.parse(world_path).getroot().find('world')
    height = 0.0
    floor = world.find("model[@name='shiyanlou_upper_floor']")
    if floor is not None:
        for link in floor.findall('link'):
            if not link.get('name', '').startswith('floor_'):
                continue
            px, py, pz, *_ = map(float, link.findtext('pose').split())
            sx, sy, sz = map(float, link.findtext('collision/geometry/box/size').split())
            if abs(x - px) <= sx / 2 and abs(y - py) <= sy / 2:
                height = max(height, pz + sz / 2)
    ramp = world.find("model[@name='shiyanlou_entrance_ramp']")
    if ramp is not None:
        px, py, pz, roll, pitch, yaw = map(float, ramp.findtext('pose').split())
        if abs(roll) > 1e-9:
            raise RuntimeError('Spawn height calculation expects the ramp to have zero roll')
        length, width, thickness = map(float, ramp.findtext('link[@name="deck"]/collision/geometry/box/size').split())
        cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        nx, ny, nz = cy * sp, sy * sp, cp
        top_x, top_y, top_z = px + nx * thickness / 2, py + ny * thickness / 2, pz + nz * thickness / 2
        z = top_z - (nx * (x - top_x) + ny * (y - top_y)) / nz
        dx, dy, dz = x - px, y - py, z - pz
        local_x = cy * cp * dx + sy * cp * dy - sp * dz
        local_y = -sy * dx + cy * dy
        if abs(local_x) <= length / 2 + 1e-8 and abs(local_y) <= width / 2:
            height = max(height, z)
    return height


def _prepare_runtime_world(world_path):
    """Avoid reloading saved robots in addition to the launch-selected robot."""
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    saved = [m for m in world.findall('model') if m.get('name') in ('burger', 'burger_lidar3d')]
    if not saved:
        return world_path
    for model in saved:
        world.remove(model)
    for state in world.findall('state'):
        for model in list(state.findall('model')):
            if model.get('name') in ('burger', 'burger_lidar3d'):
                state.remove(model)
    with tempfile.NamedTemporaryFile(prefix='tb3_world_', suffix='.world', delete=False) as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=False)
        return Path(stream.name)


def _setup(context):
    custom = Path(get_package_share_directory('tb3_sim'))
    stock = Path(get_package_share_directory('turtlebot3_gazebo'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    scene = LaunchConfiguration('scene').perform(context)
    mode = LaunchConfiguration('robot_mode').perform(context)
    worlds = {
        'shiyanlou_third': custom / 'gazebo_building_editor_models/shiyanlou/shiyanlou_third.world',
        'shiyanlou_quarter': custom / 'gazebo_building_editor_models/shiyanlou/shiyanlou_quarter.world',
        'shiyanlou': custom / 'gazebo_building_editor_models/shiyanlou/shiyanlou.world',
        'turtlebot3_world': stock / 'worlds/turtlebot3_world.world',
    }
    variants = {'3d': 'model.sdf', '3d-light': 'model_light.sdf', '3d-nav': 'model_nav.sdf'}
    if scene not in worlds or mode not in ('2d', *variants):
        raise RuntimeError(f'Unsupported scene or robot_mode: {scene}, {mode}')
    if mode == '2d':
        model = stock / 'models/turtlebot3_burger/model.sdf'
        urdf = stock / 'urdf/turtlebot3_burger.urdf'
    else:
        model = custom / 'models/turtlebot3_burger_lidar3d' / variants[mode]
        urdf = custom / 'urdf/turtlebot3_burger_lidar3d.urdf'
    override = LaunchConfiguration('model_file').perform(context)
    model = Path(override).expanduser().resolve() if override else model
    for resource in (worlds[scene], model, urdf):
        if not resource.is_file():
            raise RuntimeError(f'Simulation resource missing: {resource}')
    # Official meshes are external dependencies; no online model lookup needed.
    model_paths = [str(stock / 'models'), str(custom / 'models'),
                   str(custom / 'gazebo_building_editor_models')]
    model_paths.append(os.environ.get('GAZEBO_MODEL_PATH', ''))
    os.environ['GAZEBO_MODEL_PATH'] = os.pathsep.join(filter(None, model_paths))
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
    sim_time = LaunchConfiguration('use_sim_time')
    ekf_option = LaunchConfiguration('use_ekf').perform(context).lower()
    enable_ekf = mode == '2d' and ekf_option in ('auto', 'true')
    if ekf_option == 'true' and mode != '2d':
        raise RuntimeError('use_ekf:=true is only supported with robot_mode:=2d')
    if enable_ekf:
        # Fail before spawning anything if the optional runtime is unavailable.
        get_package_share_directory('robot_localization')
        model = _prepare_ekf_model(model)
    spawn_z = LaunchConfiguration('z_pose').perform(context)
    if spawn_z == 'auto':
        x = float(LaunchConfiguration('x_pose').perform(context))
        y = float(LaunchConfiguration('y_pose').perform(context))
        spawn_z = str(_floor_height(worlds[scene], x, y) + 0.01)
    if not math.isfinite(float(spawn_z)):
        raise RuntimeError('z_pose must be auto or a finite height in metres')
    runtime_world = _prepare_runtime_world(worlds[scene])
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'burger' if mode == '2d' else 'burger_lidar3d',
                   '-file', str(model), '-x', LaunchConfiguration('x_pose'),
                   '-y', LaunchConfiguration('y_pose'),
                   '-Y', LaunchConfiguration('yaw_pose'), '-z', spawn_z, '-timeout', '90'],
        output='screen')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzserver.launch.py')),
            launch_arguments={'world': str(runtime_world)}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch/gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': sim_time,
                          'robot_description': urdf.read_text(encoding='utf-8')}],
             output='screen'),
    ]
    if runtime_world != worlds[scene]:
        def cleanup_world(event, context):
            runtime_world.unlink(missing_ok=True)
            return []
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=cleanup_world)))
    if enable_ekf:
        def cleanup(event, context):
            model.unlink(missing_ok=True)
            return []

        # The spawn process reads the file once; also clean up on early shutdown.
        actions.extend([
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=cleanup)),
            RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
            Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
                 parameters=[str(custom / 'config/ekf_2d.yaml'),
                             {'use_sim_time': sim_time}],
                 remappings=[('odometry/filtered', '/odom')], output='screen'),
        ])
    actions.append(spawn)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='shiyanlou',
                              choices=['shiyanlou',
                                       'shiyanlou_third', 'turtlebot3_world']),
        # Direct launch uses the 3D robot default.
        DeclareLaunchArgument('robot_mode', default_value='3d',
                              choices=['2d', '3d', '3d-light', '3d-nav']),
        DeclareLaunchArgument('use_ekf', default_value='false',
                              choices=['auto', 'true', 'false'],
                              description='auto: fuse IMU + odom for 2d only; false: raw wheel odom'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('model_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='0.4'),
        DeclareLaunchArgument('z_pose', default_value='auto',
                              description='auto: spawn on the floor/ramp at x/y; or an explicit world Z'),
        DeclareLaunchArgument('yaw_pose', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
