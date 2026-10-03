"""Launch the independent 3D-LiDAR Burger in the stock TurtleBot3 world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    custom = get_package_share_directory('turtlebot3_lio_sim')
    stock = get_package_share_directory('turtlebot3_gazebo')
    gazebo = get_package_share_directory('gazebo_ros')
    world = os.path.join(stock, 'worlds', 'turtlebot3_world.world')
    model = os.path.join(custom, 'models', 'turtlebot3_burger_lidar3d', 'model.sdf')
    urdf = os.path.join(custom, 'urdf', 'turtlebot3_burger_lidar3d.urdf')
    with open(urdf, encoding='utf-8') as stream:
        robot_description = stream.read()

    sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('model_file', default_value=model),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': sim_time, 'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'burger_lidar3d', '-file', LaunchConfiguration('model_file'),
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-z', '0.01', '-timeout', '90',
            ],
            output='screen',
        ),
    ])
