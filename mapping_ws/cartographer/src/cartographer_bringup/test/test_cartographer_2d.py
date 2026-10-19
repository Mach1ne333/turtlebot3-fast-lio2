"""Configuration contracts; runtime Lua parsing is checked with Cartographer."""
import importlib.util
from pathlib import Path
import unittest

from launch_ros.actions import Node
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class CartographerConfiguration(unittest.TestCase):
    def test_tf_ownership_and_inputs(self):
        lua = (PACKAGE / 'config/cartographer_2d.lua').read_text()
        for setting in ['published_frame = "odom"', 'tracking_frame = "base_footprint"',
                        'provide_odom_frame = false', 'use_odometry = true',
                        'num_laser_scans = 1', 'num_point_clouds = 0',
                        'TRAJECTORY_BUILDER_2D.use_imu_data = false',
                        'TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1']:
            self.assertIn(setting, lua)

    def test_range_and_grid(self):
        lua = (PACKAGE / 'config/cartographer_2d.lua').read_text()
        for setting in ['TRAJECTORY_BUILDER_2D.min_range = 0.12',
                        'TRAJECTORY_BUILDER_2D.max_range = 3.5',
                        'TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.5',
                        'TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05']:
            self.assertIn(setting, lua)

    def test_launch_is_mapping_only(self):
        spec = importlib.util.spec_from_file_location(
            'mapping_ws', PACKAGE / 'launch/cartographer_2d.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        nodes = [a for a in module.generate_launch_description().entities if isinstance(a, Node)]
        executables = {n.node_executable for n in nodes}
        self.assertEqual(executables, {'cartographer_node', 'cartographer_occupancy_grid_node', 'rviz2'})

    def test_clean_rviz(self):
        cfg = yaml.safe_load((PACKAGE / 'rviz/cartographer_2d.rviz').read_text())
        manager = cfg['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        topics = {d['Topic']['Value'] for d in manager['Displays'] if 'Topic' in d}
        self.assertEqual(topics, {'/map', '/scan'})


if __name__ == '__main__':
    unittest.main()
