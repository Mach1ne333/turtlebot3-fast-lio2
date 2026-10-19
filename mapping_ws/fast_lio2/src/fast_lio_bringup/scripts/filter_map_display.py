#!/usr/bin/python3
"""Publish a height-cropped display copy; never alter FAST-LIO inputs/maps.

Run from the repository root after sourcing ROS:
  ros2 run fast_lio_bringup filter_map_display.py --ros-args -p min_z:=0.05
Then set RViz's map PointCloud2 Topic to /Laser_map_display.
min_z is an absolute height in the incoming cloud frame, NOT ground-relative.
Low walls/obstacles below the cutoff are hidden too; this is not segmentation.
"""

from array import array
import math
import signal

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def crop_above_z(cloud, min_z):
    """Keep z >= min_z, preserving header, all fields, and point record bytes."""
    if not math.isfinite(min_z):
        raise ValueError('min_z must be finite')
    result = PointCloud2()
    result.header = cloud.header
    result.fields = cloud.fields
    result.height = 1
    result.point_step = cloud.point_step
    result.is_bigendian = cloud.is_bigendian
    result.is_dense = cloud.is_dense
    if cloud.width == 0 or cloud.height == 0:
        return result
    field = next((f for f in cloud.fields if f.name == 'z'), None)
    formats = {PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
    if field is None or field.count != 1 or field.datatype not in formats:
        raise ValueError('Expected one FLOAT32/FLOAT64 z field')
    dtype = np.dtype(('>' if cloud.is_bigendian else '<') + formats[field.datatype])
    if field.offset < 0 or field.offset + dtype.itemsize > cloud.point_step:
        raise ValueError('z field is outside the point record')
    if cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('row_step is shorter than a row of points')
    z = np.ndarray(
        (cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
        offset=field.offset, strides=(cloud.row_step, cloud.point_step))
    records = np.ndarray(
        (cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
        buffer=cloud.data, strides=(cloud.row_step, cloud.point_step, 1))
    selected = records[np.isfinite(z) & (z >= min_z)]
    result.width = len(selected)
    result.row_step = result.width * result.point_step
    result.data = array('B', selected.tobytes())
    return result


class MapDisplayHeightFilter(Node):
    def __init__(self):
        super().__init__('map_display_height_filter')
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/Laser_map_display')
        self.declare_parameter('min_z', 0.05)
        source = self.get_parameter('input_topic').value
        target = self.get_parameter('output_topic').value
        if self.resolve_topic_name(source) == self.resolve_topic_name(target):
            raise ValueError('Input and display output topics must differ')
        if not math.isfinite(self.get_parameter('min_z').value):
            raise ValueError('min_z must be finite')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.publisher = self.create_publisher(PointCloud2, target, 1)
        self.subscription = self.create_subscription(
            PointCloud2, source, self.on_cloud,
            # /Laser_map is a reliable, large accumulated map, not a sensor stream.
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(
            f'Display only: {source} -> {target}; z >= '
            f'{self.get_parameter("min_z").value} m in the input cloud frame')

    @staticmethod
    def validate_parameters(parameters):
        for parameter in parameters:
            if parameter.name in ('input_topic', 'output_topic'):
                return SetParametersResult(
                    successful=False, reason='Restart the display filter to change topics')
            if parameter.name == 'min_z' and not math.isfinite(parameter.value):
                return SetParametersResult(successful=False, reason='min_z must be finite')
        return SetParametersResult(successful=True)

    def on_cloud(self, cloud):
        try:
            filtered = crop_above_z(cloud, self.get_parameter('min_z').value)
        except (ValueError, TypeError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        self.publisher.publish(filtered)


def main():
    rclpy.init()
    node = MapDisplayHeightFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward Ctrl+C after the terminal already delivered it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
