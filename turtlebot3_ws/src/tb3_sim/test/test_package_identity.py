"""Catch partial ROS package/launch renames before attempting Gazebo startup."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]
NAME = ET.parse(PACKAGE / 'package.xml').getroot().findtext('name')


class PackageIdentity(unittest.TestCase):
    def test_manifest_and_cmake_agree(self):
        self.assertEqual(NAME, 'tb3_sim')
        project = re.search(r'project\(\s*([^\s)]+)', (PACKAGE / 'CMakeLists.txt').read_text())
        self.assertIsNotNone(project)
        self.assertEqual(project.group(1), NAME)

    def test_installed_resources_are_discoverable(self):
        share = Path(get_package_share_directory(NAME))
        self.assertEqual(ET.parse(share / 'package.xml').getroot().findtext('name'), NAME)
        for resource in ('launch/tb3_sim.launch.py', 'rviz/tb3_lidar3d.rviz',
                         'urdf/turtlebot3_burger_lidar3d.urdf', 'config/ekf_2d.yaml'):
            with self.subTest(resource=resource):
                self.assertTrue((share / resource).is_file())

    def test_launcher_and_build_selector_use_current_package(self):
        workspace = PACKAGE.parents[1]
        launcher = (workspace / 'start-world.sh').read_text()
        self.assertIn(f'exec ros2 launch {NAME} tb3_sim.launch.py', launcher)
        build = (workspace.parent / 'scripts/build-2d.sh').read_text()
        self.assertIn(f'--packages-select {NAME}', build)


if __name__ == '__main__':
    unittest.main()
