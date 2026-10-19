"""Integration uses package resources and caller-supplied external map data."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


class MappingLaunchTest(unittest.TestCase):
    def test_installed_configuration_and_explicit_map(self):
        spec = importlib.util.spec_from_file_location('mapping_launch', PACKAGE / 'launch/mapping.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        relocated_share = Path('/different repository/install/share/fast_lio_bringup')
        with patch.object(module, 'get_package_share_directory', return_value=str(relocated_share)), \
                patch.object(module, 'Node', wraps=Node) as nodes:
            actions = module.generate_launch_description().entities
        self.assertEqual(nodes.call_count, 1)
        node = nodes.call_args.kwargs
        self.assertEqual((node['package'], node['executable']), ('fast_lio', 'fastlio_mapping'))
        context = LaunchContext()
        context.launch_configurations.update(
            config_file='tb3_gazebo_instant.yaml', map_file_path='/external maps/test.pcd')
        self.assertEqual(node['parameters'][0].perform(context),
                         str(relocated_share / 'config/tb3_gazebo_instant.yaml'))
        self.assertEqual(node['parameters'][1]['map_file_path'].perform(context),
                         '/external maps/test.pcd')
        declarations = {a.name: a for a in actions if isinstance(a, DeclareLaunchArgument)}
        self.assertIsNone(declarations['map_file_path'].default_value)

    def test_mapping_inputs_and_map_output_are_enabled(self):
        config = yaml.safe_load((PACKAGE / 'config/tb3_gazebo_instant.yaml').read_text())
        params = config['/**']['ros__parameters']
        self.assertEqual(params['common']['lid_topic'], '/points_raw')
        self.assertEqual(params['common']['imu_topic'], '/imu')
        self.assertEqual(params['preprocess']['lidar_type'], 5)
        self.assertTrue(params['publish']['map_en'])
        self.assertTrue(params['publish']['scan_bodyframe_pub_en'])
        self.assertTrue(params['pcd_save']['pcd_save_en'])


if __name__ == '__main__':
    unittest.main()
