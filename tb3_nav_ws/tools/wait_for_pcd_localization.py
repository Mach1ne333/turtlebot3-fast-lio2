#!/usr/bin/env python3
"""Exit 0 only after 3D ICP is valid and LIO / map TF data are recent.

Launched by navigation_all.launch.py; Nav2 starts only on a successful exit.
A loaded PCD or an RViz click alone is insufficient.
"""
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from interface.srv import IsValid


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizationGate(Node):
    def __init__(self):
        super().__init__('nav3d_localization_gate')
        self.clock_stamp = None
        self.lio_stamp = None
        self.map_stamp = None
        self.valid = False
        self.pending = None
        self.good_checks = 0
        self.ready = False
        self.last_notice = 0.0
        self.client = self.create_client(IsValid, '/pcd_localizer/relocalize_check')
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_clock = self.create_subscription(
            Clock, '/clock', lambda m: setattr(self, 'clock_stamp', seconds(m.clock)), sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/Odometry', lambda m: setattr(self, 'lio_stamp', seconds(m.header.stamp)), 10)
        self.sub_tf = self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.timer = self.create_timer(0.8, self.check)
        self.get_logger().info('Waiting for RViz 2D Pose Estimate and valid 3D PCD ICP alignment')

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'camera_init':
                self.map_stamp = seconds(t.header.stamp)

    def check(self):
        if self.pending is None and self.client.service_is_ready():
            self.pending = self.client.call_async(IsValid.Request(code=0))
        if self.pending is not None and self.pending.done():
            try:
                self.valid = bool(self.pending.result().valid)
            except Exception as exc:
                self.get_logger().warning(f'ICP validity check failed: {exc}')
                self.valid = False
            self.pending = None
        now = self.clock_stamp
        fresh = now is not None and all(
            stamp is not None and -0.2 < now - stamp < 1.5
            for stamp in (self.lio_stamp, self.map_stamp))
        if self.valid and fresh:
            self.good_checks += 1
            if self.good_checks >= 2:
                self.get_logger().info('ICP VALID and LIO/map TF FRESH: releasing Nav2 startup gate')
                self.ready = True
        else:
            self.good_checks = 0
            if time.monotonic() - self.last_notice > 8:
                self.get_logger().info('Nav2 held: waiting for ICP validity and fresh /Odometry + map TF')
                self.last_notice = time.monotonic()


def main():
    rclpy.init()
    node = LocalizationGate()
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    ready = node.ready
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if ready else 2)


if __name__ == '__main__':
    main()
