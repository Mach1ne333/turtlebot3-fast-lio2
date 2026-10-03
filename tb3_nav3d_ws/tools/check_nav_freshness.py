#!/usr/bin/env python3
"""Reject stale LIO/map TF before Nav2 starts or a test declares success."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = Node('nav3d_freshness_probe')
    stamps = {}
    keep = []
    keep.append(node.create_subscription(
        Clock, '/clock', lambda msg: stamps.__setitem__('clock', sec(msg.clock)),
        QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
    keep.append(node.create_subscription(
        Odometry, '/Odometry', lambda msg: stamps.__setitem__('lio', sec(msg.header.stamp)), 10))

    def on_tf(msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'camera_init':
                stamps['map_tf'] = sec(tf.header.stamp)
    keep.append(node.create_subscription(TFMessage, '/tf', on_tf, 10))
    end = time.monotonic() + 4
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    now = stamps.get('clock')
    ok = bool(now) and all(k in stamps and -0.2 < now - stamps[k] < 1.5
                           for k in ('lio', 'map_tf'))
    print('current_sim_time=', now,
          'LIO_age_sim_s=', round(now - stamps['lio'], 3) if now and 'lio' in stamps else 'missing',
          'map_TF_age_sim_s=', round(now - stamps['map_tf'], 3) if now and 'map_tf' in stamps else 'missing',
          'fresh=', ok, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
