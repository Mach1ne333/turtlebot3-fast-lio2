#!/usr/bin/env python3
"""Passive test-only summary of LIO pose, Gazebo pose and issued velocity commands."""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

rclpy.init()
node = Node('nav3d_motion_observer')
seen = {}
counts = {}
max_cmd = 0.0
subs = []


def odom_cb(name):
    def cb(msg):
        pos = msg.pose.pose.position
        if name not in seen:
            seen[name] = [[pos.x, pos.y], [pos.x, pos.y]]
        else:
            seen[name][1] = [pos.x, pos.y]
        counts[name] = counts.get(name, 0) + 1
    return cb


for topic in ('/lio/nav_odom', '/Odometry', '/odom', '/gazebo/odom'):
    subs.append(node.create_subscription(Odometry, topic, odom_cb(topic),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))


def cmd_cb(msg):
    global max_cmd
    max_cmd = max(max_cmd, abs(msg.linear.x), abs(msg.angular.z))
    counts['/cmd_vel'] = counts.get('/cmd_vel', 0) + 1


subs.append(node.create_subscription(Twist, '/cmd_vel', cmd_cb, 10))
end = time.monotonic() + float(sys.argv[1] if len(sys.argv) > 1 else '90')
try:
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    for topic, poses in seen.items():
        print(topic, 'messages=', counts.get(topic, 0), 'start=', poses[0], 'end=', poses[1], flush=True)
    print('cmd_vel_messages=', counts.get('/cmd_vel', 0), 'max_abs_command=', max_cmd, flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
