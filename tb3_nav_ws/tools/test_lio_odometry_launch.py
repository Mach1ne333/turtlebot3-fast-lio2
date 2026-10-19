"""Source-level navigation LIO wiring checks; no nodes, PCD writes or build."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

WS = Path(__file__).resolve().parents[1]


def load_launch(name):
    spec = importlib.util.spec_from_file_location(name, WS / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NavigationLioTest(unittest.TestCase):
    def test_navigation_configuration_does_not_save_maps(self):
        config = yaml.safe_load((WS / 'config/tb3_gazebo_nav.yaml').read_text())
        params = config['/**']['ros__parameters']
        self.assertEqual(params['common']['lid_topic'], '/points_raw')
        self.assertEqual(params['common']['imu_topic'], '/imu')
        self.assertEqual(params['preprocess']['lidar_type'], 5)
        self.assertTrue(params['publish']['scan_publish_en'])
        self.assertTrue(params['publish']['scan_bodyframe_pub_en'])
        self.assertFalse(params['publish']['map_en'])
        self.assertFalse(params['publish']['path_en'])
        self.assertFalse(params['pcd_save']['pcd_save_en'])
        self.assertNotIn('map_file_path', params)

    def test_lio_launch_uses_navigation_owned_configuration(self):
        module = load_launch('lio_odometry')
        with patch.object(module, 'Node', wraps=Node) as nodes:
            actions = module.generate_launch_description().entities
        self.assertEqual(nodes.call_count, 1)
        node = nodes.call_args.kwargs
        self.assertEqual((node['package'], node['executable']), ('fast_lio', 'fastlio_mapping'))
        self.assertEqual(node['parameters'][0], str(WS / 'config/tb3_gazebo_nav.yaml'))
        self.assertEqual(set(node['parameters'][1]), {'use_sim_time'})
        context = LaunchContext()
        context.launch_configurations['use_sim_time'] = 'true'
        self.assertEqual(node['parameters'][1]['use_sim_time'].perform(context), 'true')
        declarations = {a.name for a in actions if isinstance(a, DeclareLaunchArgument)}
        self.assertEqual(declarations, {'use_sim_time'})

    def test_combined_navigation_does_not_include_mapping_bringup(self):
        module = load_launch('navigation_all')
        with patch.object(module, 'PythonLaunchDescriptionSource',
                          wraps=PythonLaunchDescriptionSource) as sources, \
                patch.object(module, 'IncludeLaunchDescription',
                             wraps=IncludeLaunchDescription) as includes:
            module.generate_launch_description()
        self.assertEqual(sources.call_args_list[0].args[0],
                         str(WS / 'launch/lio_odometry.launch.py'))
        self.assertEqual(dict(includes.call_args_list[0].kwargs['launch_arguments']),
                         {'use_sim_time': 'true'})
        self.assertEqual(sources.call_args_list[1].args[0],
                         str(WS / 'launch/localizer_with_initialpose.launch.py'))

    def test_selected_map_reaches_nav2_only_after_successful_gate(self):
        module = load_launch('navigation_all')
        with patch.object(module, 'OnProcessExit', wraps=OnProcessExit) as handlers:
            actions = module.generate_launch_description().entities
        self.assertIn('map', {a.name for a in actions if isinstance(a, DeclareLaunchArgument)})
        callback = handlers.call_args.kwargs['on_exit']
        context = LaunchContext()
        context.launch_configurations['map'] = '/custom maps/shiyanlou_ground.yaml'
        with patch.object(module, 'IncludeLaunchDescription', wraps=IncludeLaunchDescription) as includes:
            callback(SimpleNamespace(returncode=0), context)
        selected = dict(includes.call_args.kwargs['launch_arguments'])['map']
        self.assertEqual(selected.perform(context), '/custom maps/shiyanlou_ground.yaml')
        with patch.object(module, 'IncludeLaunchDescription', wraps=IncludeLaunchDescription) as includes:
            callback(SimpleNamespace(returncode=1), context)
        includes.assert_not_called()

    def test_map_server_uses_selected_yaml(self):
        module = load_launch('nav3d')
        with patch.object(module, 'get_package_share_directory', return_value='/nav2/share'), \
                patch.object(module, 'Node', wraps=Node) as nodes:
            actions = module.generate_launch_description().entities
        declarations = {a.name for a in actions if isinstance(a, DeclareLaunchArgument)}
        self.assertIn('map', declarations)
        server = next(call.kwargs for call in nodes.call_args_list
                      if call.kwargs.get('name') == 'map_server')
        context = LaunchContext()
        context.launch_configurations['map'] = '/custom maps/shiyanlou_ground.yaml'
        self.assertEqual(server['parameters'][0]['yaml_filename'].perform(context),
                         '/custom maps/shiyanlou_ground.yaml')


if __name__ == '__main__':
    unittest.main()
