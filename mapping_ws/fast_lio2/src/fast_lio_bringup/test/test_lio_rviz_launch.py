"""Installed-resource and display-only launch contracts; no simulator needed."""
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location(
        'lio_rviz_launch', PACKAGE / 'launch/lio_rviz.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LioRvizLaunchTest(unittest.TestCase):
    def test_launcher_only_runs_display_processes(self):
        module = load_launch()
        relocated_share = Path('/relocated install/share/fast_lio_bringup')
        with patch.object(module, 'get_package_share_directory', return_value=str(relocated_share)), \
                patch.object(module, 'Node', wraps=Node) as nodes:
            actions = module.generate_launch_description().entities
        self.assertEqual([(n.kwargs['package'], n.kwargs['executable'])
                          for n in nodes.call_args_list],
                         [('fast_lio_bringup', 'filter_map_display.py'), ('rviz2', 'rviz2')])
        context = LaunchContext()
        context.launch_configurations['min_z'] = '0.2'
        cutoff = nodes.call_args_list[0].kwargs['arguments'][-1]
        self.assertEqual(perform_substitutions(context, normalize_to_list_of_substitutions(cutoff)), 'min_z:=0.2')
        self.assertEqual(nodes.call_args_list[1].kwargs['arguments'],
                         ['-d', str(relocated_share / 'rviz/tb3_fast_lio2.rviz')])
        declarations = [a for a in actions if isinstance(a, DeclareLaunchArgument)]
        self.assertEqual([a.name for a in declarations], ['min_z', 'use_sim_time'])
        # Preserve the user's pre-migration launch cutoff, not an older test's 0.05.
        self.assertEqual(perform_substitutions(context, declarations[0].default_value), '-0.25')
        self.assertEqual(sum(isinstance(a, RegisterEventHandler) for a in actions), 2)

    def test_rviz_defaults_to_filtered_map(self):
        config = yaml.safe_load((PACKAGE / 'rviz/tb3_fast_lio2.rviz').read_text())['Visualization Manager']
        self.assertEqual(config['Global Options']['Fixed Frame'], 'camera_init')
        points = [d for d in config['Displays']
                  if d['Class'] == 'rviz_default_plugins/PointCloud2']
        enabled = [d for d in points if d['Enabled']]
        self.assertEqual(len(enabled), 1)
        self.assertEqual(enabled[0]['Topic']['Value'], '/Laser_map_display')
        self.assertEqual(enabled[0]['Topic']['Reliability Policy'], 'Reliable')
        self.assertFalse(next(d for d in points
                              if d['Topic']['Value'] == '/cloud_registered')['Enabled'])

    def test_shell_uses_package_launch_and_forwards_arguments(self):
        script = (PACKAGE.parents[1] / 'start-lio-rviz.sh').read_text()
        self.assertIn('exec ros2 launch fast_lio_bringup lio_rviz.launch.py "$@"', script)
        self.assertTrue(os.access(PACKAGE / 'scripts/filter_map_display.py', os.X_OK))


if __name__ == '__main__':
    unittest.main()
