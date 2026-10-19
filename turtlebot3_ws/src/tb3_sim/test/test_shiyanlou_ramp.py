"""Geometry and launch regression checks for the 0.5 m raised building."""
import hashlib
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE.parents[2] / 'gazebo_building_editor_models/shiyanlou/shiyanlou.world'


def pose(element):
    return tuple(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))


def size(link):
    return tuple(map(float, link.findtext('collision/geometry/box/size').split()))


def contains(link, x, y):
    px, py, *_ = pose(link)
    sx, sy, _ = size(link)
    return abs(x - px) <= sx / 2 + 1e-9 and abs(y - py) <= sy / 2 + 1e-9


class RaisedBuildingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(WORLD).getroot().find('world')
        cls.floor = cls.world.find("model[@name='shiyanlou_upper_floor']")
        cls.ramp = cls.world.find("model[@name='shiyanlou_entrance_ramp']")
        spec = importlib.util.spec_from_file_location('sim_ramp_launch', PACKAGE / 'launch/tb3_sim.launch.py')
        cls.launch = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.launch)

    def test_walls_and_saved_state_are_raised(self):
        model = self.world.find("model[@name='shiyanlou_quarter']")
        self.assertEqual(len(model.findall('link')), 26)
        self.assertEqual(pose(model)[2], 0.5)
        state = self.world.find("state/model[@name='shiyanlou_quarter']")
        self.assertEqual(pose(state)[2], 0.5)
        for link in model.findall('link'):
            self.assertEqual(pose(link)[2], 0)
            self.assertEqual(link.findtext('collision/geometry/box/size'),
                             link.findtext('visual/geometry/box/size'))
            self.assertEqual(size(link)[2], 2.5)
            self.assertEqual(pose(state.find(f"link[@name='{link.get('name')}']"))[2], 0.5)

    def test_floor_and_ramp_have_matching_visual_collision(self):
        for model in (self.floor, self.ramp):
            self.assertEqual(model.findtext('static'), 'true')
            for link in model.findall('link'):
                self.assertEqual(link.findtext('collision/geometry/box/size'),
                                 link.findtext('visual/geometry/box/size'))
        for link in self.floor.findall('link'):
            if link.get('name').startswith('floor_'):
                self.assertAlmostEqual(pose(link)[2] + size(link)[2] / 2, .5)

    def test_all_wall_footprints_are_supported(self):
        building = self.world.find("model[@name='shiyanlou_quarter']")
        mx, my, *_ = pose(building)
        floors = [l for l in self.floor.findall('link') if l.get('name').startswith('floor_')]
        for wall in building.findall('link'):
            px, py, _, _, _, yaw = pose(wall)
            sx, sy, _ = size(wall)
            for a in (-sx / 2, 0, sx / 2):
                for b in (-sy / 2, 0, sy / 2):
                    x = mx + px + math.cos(yaw) * a - math.sin(yaw) * b
                    y = my + py + math.sin(yaw) * a + math.cos(yaw) * b
                    self.assertTrue(any(contains(f, x, y) for f in floors), (wall.get('name'), x, y))

    def test_ramp_top_ends_match_both_levels(self):
        px, py, pz, _, pitch, yaw = pose(self.ramp)
        length, width, thickness = size(self.ramp.find("link[@name='deck']"))
        ends = []
        for local_x in (-length / 2, length / 2):
            dx = math.cos(pitch) * local_x + math.sin(pitch) * thickness / 2
            ends.append((px + math.cos(yaw) * dx, py + math.sin(yaw) * dx,
                         pz - math.sin(pitch) * local_x + math.cos(pitch) * thickness / 2))
        ends.sort(key=lambda p: p[2])
        self.assertAlmostEqual(ends[0][2], 0, places=8)
        self.assertAlmostEqual(ends[1][2], .5, places=8)
        self.assertAlmostEqual(ends[1][1] - ends[0][1], 1.5, places=8)
        self.assertAlmostEqual(ends[0][1], -3.0236, places=8)
        self.assertAlmostEqual(ends[1][1], -1.5236, places=8)
        self.assertAlmostEqual(px, 3.522745)
        self.assertAlmostEqual(width, .6, places=6)
        self.assertLess(width, .62)

    def test_green_central_room_is_lower_and_former_ramp_side_is_floored(self):
        for x, y in ((-5.8, .4), (-4, -2), (-2, .4), (3.9, -3), (3, -2)):
            self.assertEqual(self.launch._floor_height(WORLD, x, y), 0, (x, y))
        for x, y in ((7.5, .5), (4.7, .5), (4.7, 1.8), (3.522745, 1.8), (3.522745, .0141)):
            self.assertEqual(self.launch._floor_height(WORLD, x, y), .5, (x, y))
        for y, height in ((-3.0236, 0), (-2.2736, .25), (-1.5236, .5)):
            self.assertAlmostEqual(self.launch._floor_height(WORLD, 3.522745, y), height, places=5)

    def test_only_ramp_mouth_is_unguarded(self):
        floors = [l for l in self.floor.findall('link') if l.get('name').startswith('floor_')]
        guards = [l for l in self.floor.findall('link') if l.get('name').startswith('guard_')]
        self.assertEqual(len(guards), 13)
        for link in floors:
            x, y, *_ = pose(link)
            sx, sy, _ = size(link)
            for axis, normal, half in ((0, -1, sx / 2), (0, 1, sx / 2),
                                       (1, -1, sy / 2), (1, 1, sy / 2)):
                other_half = sy / 2 if axis == 0 else sx / 2
                samples = math.ceil(2 * other_half / .05)
                for i in range(samples + 1):
                    point = [x, y]
                    point[axis] += normal * half
                    point[1-axis] += -other_half + 2 * other_half * i / samples
                    outside = point.copy()
                    outside[axis] += normal * .001
                    if any(contains(f, *outside) for f in floors):
                        continue
                    opening = abs(point[1] + 1.5236) < 1e-6 and 3.222745 < point[0] < 3.822745
                    if not opening:
                        self.assertTrue(any(contains(g, *point) for g in guards), point)

    def test_launch_automatically_spawns_on_the_selected_level(self):
        for x, y, expected in ((-2, .4, .01), (7.5, .5, .51), (3.522745, -2.2736, .26)):
            context = LaunchContext()
            context.launch_configurations.update(
                scene='shiyanlou', robot_mode='3d-nav', model_file='', use_ekf='false',
                use_sim_time='true', x_pose=str(x), y_pose=str(y), z_pose='auto', yaw_pose='0', gui='false')
            with patch.object(self.launch, '_prepare_runtime_world', return_value=WORLD), \
                    patch.object(self.launch, 'Node', wraps=Node) as nodes:
                self.launch._setup(context)
            spawn = next(c.kwargs for c in nodes.call_args_list if c.kwargs['executable'] == 'spawn_entity.py')
            arguments = spawn['arguments']
            self.assertAlmostEqual(float(arguments[arguments.index('-z') + 1]), expected, places=5)

    def test_runtime_world_avoids_duplicate_saved_robot_without_changing_source(self):
        before = hashlib.sha256(WORLD.read_bytes()).digest()
        temporary = self.launch._prepare_runtime_world(WORLD)
        self.assertNotEqual(temporary, WORLD)
        try:
            runtime = ET.parse(temporary).getroot().find('world')
            self.assertIsNone(runtime.find("model[@name='burger_lidar3d']"))
            self.assertIsNone(runtime.find("state/model[@name='burger_lidar3d']"))
            self.assertIsNotNone(runtime.find("model[@name='shiyanlou_upper_floor']"))
            self.assertIsNotNone(runtime.find("model[@name='unit_box_0_clone']"))
        finally:
            temporary.unlink()
        self.assertEqual(hashlib.sha256(WORLD.read_bytes()).digest(), before)


if __name__ == '__main__':
    unittest.main()
