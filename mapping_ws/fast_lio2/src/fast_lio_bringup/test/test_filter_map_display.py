"""Display crop contracts, independent of the simulator and workspace name."""
from array import array
import importlib.util
import math
from pathlib import Path
import struct
import unittest

from sensor_msgs.msg import PointCloud2, PointField

spec = importlib.util.spec_from_file_location(
    'filter_map_display', Path(__file__).resolve().parents[1] / 'scripts/filter_map_display.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
crop_above_z = module.crop_above_z


class DisplayCropTest(unittest.TestCase):
    @staticmethod
    def cloud(big_endian=False):
        msg = PointCloud2()
        msg.header.frame_id = 'camera_init'
        msg.header.stamp.sec = 42
        msg.height = 2
        msg.width = 3
        msg.point_step = 16
        msg.row_step = 52  # Four bytes of row padding, not point data.
        msg.is_bigendian = big_endian
        msg.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                      for name, offset, kind in [('x', 0, PointField.FLOAT32),
                                                 ('y', 4, PointField.FLOAT32),
                                                 ('z', 8, PointField.FLOAT32),
                                                 ('intensity', 12, PointField.UINT16)]]
        records = [struct.pack(('>' if big_endian else '<') + 'fffH2s',
                               1., 2., z, i, b'xy')
                   for i, z in enumerate([-.08, .04, .1, math.nan, .5, 1.])]
        msg.data = array('B', b''.join(records[:3]) + b'PAD!' +
                         b''.join(records[3:]) + b'PAD!')
        return msg, records

    def test_fields_padding_and_source_preserved(self):
        for endian in (False, True):
            with self.subTest(big_endian=endian):
                cloud, records = self.cloud(endian)
                original = bytes(cloud.data)
                cropped = crop_above_z(cloud, .05)
                self.assertEqual(bytes(cropped.data), b''.join(records[i] for i in [2, 4, 5]))
                self.assertEqual((cropped.height, cropped.width, cropped.row_step), (1, 3, 48))
                self.assertEqual(cropped.fields, cloud.fields)
                self.assertEqual(cropped.header, cloud.header)
                self.assertEqual(cropped.is_bigendian, endian)
                self.assertEqual(bytes(cloud.data), original)

    def test_empty_output_and_input(self):
        cloud, _ = self.cloud()
        cropped = crop_above_z(cloud, 100.)
        self.assertEqual((cropped.width, cropped.row_step, len(cropped.data)), (0, 0, 0))
        self.assertEqual(crop_above_z(PointCloud2(), .05).width, 0)

    def test_invalid_fields_and_cutoff(self):
        cloud, _ = self.cloud()
        with self.assertRaises(ValueError):
            crop_above_z(cloud, math.nan)
        cloud.fields[2].datatype = PointField.INT32
        with self.assertRaises(ValueError):
            crop_above_z(cloud, .05)


if __name__ == '__main__':
    unittest.main()
