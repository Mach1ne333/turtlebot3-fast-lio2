"""Small configuration checks; run with system Python in the repository overlay."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('simulation', PACKAGE / 'launch/tb3_sim.launch.py')
SIM = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SIM)


class EkfConfiguration(unittest.TestCase):
    def test_only_drive_outputs_change(self):
        stock = Path(get_package_share_directory('turtlebot3_gazebo'))
        original = stock / 'models/turtlebot3_burger/model.sdf'
        before = original.read_bytes()
        generated = SIM._prepare_ekf_model(original)
        try:
            # Humble spawn_entity parses the file as Unicode via lxml.
            self.assertFalse(generated.read_text().startswith('<?xml'))
            root = ET.parse(generated)
            drive = root.find(".//plugin[@filename='libgazebo_ros_diff_drive.so']")
            self.assertEqual(drive.findtext('publish_odom_tf'), 'false')
            self.assertIn('odom:=/wheel/odom', [r.text for r in drive.findall('ros/remapping')])
            # Reverse the intended edits and compare the remaining model structure.
            drive.find('publish_odom_tf').text = 'true'
            for remap in list(drive.findall('ros/remapping')):
                if remap.text == 'odom:=/wheel/odom':
                    drive.find('ros').remove(remap)

            def structure(node):
                return (node.tag, node.attrib, (node.text or '').strip(),
                        [structure(child) for child in node])
            self.assertEqual(structure(root.getroot()), structure(ET.fromstring(before)))
            self.assertEqual(original.read_bytes(), before)
        finally:
            generated.unlink(missing_ok=True)

    def test_mode_selection_and_rollback(self):
        for mode, option, enabled in [('2d', 'auto', True), ('2d', 'false', False),
                                       ('3d', 'auto', False), ('3d-nav', 'auto', False)]:
            context = LaunchContext()
            context.launch_configurations.update(
                scene='shiyanlou_third', robot_mode=mode, model_file='', use_ekf=option,
                use_sim_time='true', x_pose='-2', y_pose='0.4', z_pose='auto', yaw_pose='0', gui='false')
            with patch.object(SIM, '_prepare_ekf_model', return_value=Path('/tmp/ekf_test.sdf')) as prepare:
                actions = SIM._setup(context)
                self.assertEqual(prepare.called, enabled)
                self.assertEqual(len(actions), 7 if enabled else 4)

    def test_fusion_fields(self):
        cfg = yaml.safe_load((PACKAGE / 'config/ekf_2d.yaml').read_text())['ekf_filter_node']['ros__parameters']
        self.assertEqual(cfg['world_frame'], 'odom')
        self.assertEqual(cfg['base_link_frame'], 'base_footprint')
        self.assertTrue(cfg['two_d_mode'] and cfg['publish_tf'])
        self.assertEqual([i for i, v in enumerate(cfg['odom0_config']) if v], [6, 7])
        self.assertEqual([i for i, v in enumerate(cfg['imu0_config']) if v], [11])
        self.assertEqual(len(cfg['odom0_config']), 15)
        self.assertEqual(len(cfg['imu0_config']), 15)
        self.assertEqual(cfg['odom0'], '/wheel/odom')
        self.assertEqual(cfg['imu0'], '/imu')


if __name__ == '__main__':
    unittest.main()
