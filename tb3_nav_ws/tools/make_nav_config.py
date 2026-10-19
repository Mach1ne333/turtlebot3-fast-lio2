#!/usr/bin/env python3
"""Generate an independent Humble Nav2 config from the tested TB3 Burger config.

No AMCL is launched. PCD localizer provides map->camera_init, Fast-LIO provides
camera_init->body, static bridge provides body->base_footprint. Nav2 planning
still uses the PCD-derived occupancy map; live 3D cloud handles obstacles.
"""
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[2]
source = root / 'turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/humble/burger.yaml'
target = Path(__file__).resolve().parents[1] / 'config/nav2_3d.yaml'
p = yaml.safe_load(source.read_text())
# No AMCL or LaserScan; do not silently use Gazebo's /odom.
p.pop('amcl', None)
p.pop('amcl_map_client', None)
p.pop('amcl_rclcpp_node', None)
p['bt_navigator']['ros__parameters'].update(robot_base_frame='base_footprint', odom_topic='/lio/nav_odom')
c = p['controller_server']['ros__parameters']
c.update(odom_topic='/lio/nav_odom', controller_frequency=5.0)
f = c['FollowPath']
f.update(max_vel_x=0.08, max_speed_xy=0.08, max_vel_theta=0.45,
         acc_lim_x=0.35, decel_lim_x=-0.35, acc_lim_theta=0.8,
         decel_lim_theta=-0.8, vx_samples=10, vtheta_samples=20,
         sim_time=1.0, transform_tolerance=0.6,
         trans_stopped_velocity=0.02)
c['general_goal_checker']['xy_goal_tolerance'] = 0.12
c['progress_checker']['required_movement_radius'] = 0.10
c['progress_checker']['movement_time_allowance'] = 18.0
for which in ('local_costmap', 'global_costmap'):
    q = p[which][which]['ros__parameters']
    q['robot_base_frame'] = 'base_footprint'
    q['robot_radius'] = 0.115
    q['update_frequency'] = 3.0 if which == 'local_costmap' else 1.0
    q['publish_frequency'] = 1.0
    q['plugins'] = (['voxel_layer', 'inflation_layer'] if which == 'local_costmap'
                    else ['static_layer', 'inflation_layer'])
    q.pop('obstacle_layer', None)
    q['voxel_layer'] = {
        'plugin': 'nav2_costmap_2d::VoxelLayer', 'enabled': True,
        'publish_voxel_map': False, 'origin_z': -0.20,
        'z_resolution': 0.10, 'z_voxels': 16,
        'min_obstacle_height': 0.07, 'max_obstacle_height': 1.0,
        'mark_threshold': 0, 'observation_sources': 'lidar3d',
        'lidar3d': {
            'topic': '/points_raw', 'data_type': 'PointCloud2',
            'clearing': True, 'marking': True,
            'min_obstacle_height': 0.07, 'max_obstacle_height': 1.0,
            'raytrace_min_range': 0.20, 'raytrace_max_range': 3.5,
            'obstacle_min_range': 0.20, 'obstacle_max_range': 3.0,
        },
    } if which == 'local_costmap' else None
    if which != 'local_costmap':
        q.pop('voxel_layer', None)
    q['inflation_layer']['inflation_radius'] = 0.24
    q['inflation_layer']['cost_scaling_factor'] = 5.0
    if which == 'local_costmap':
        q['global_frame'] = 'camera_init'
        q['width'] = q['height'] = 3
    else:
        q['track_unknown_space'] = True
        q['static_layer']['map_subscribe_transient_local'] = True
r = p.get('recoveries_server', p.get('behavior_server'))
if r:
    q = r['ros__parameters']
    q.update(global_frame='camera_init', robot_base_frame='base_footprint')
# Older TB3 param file includes obsolete keys; navigation_launch.py loads only
# the nodes it knows, and Nav2 Humble ignores optional unused keys.
target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(yaml.safe_dump(p, sort_keys=False, allow_unicode=True))
print(f'Wrote {target}; no changes to TurtleBot3 official files')
