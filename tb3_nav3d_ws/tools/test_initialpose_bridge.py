#!/usr/bin/env python3
"""Isolated fake-service check of RViz click -> PCD ICP request, no Gazebo."""
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize, IsValid

rclpy.init()
node = Node('fake_pcd_service', namespace='pcd_localizer')
rviz = Node('fake_rviz_tool')
requests = []


def on_reloc(req, res):
    requests.append(req)
    res.success = True
    res.message = 'fake load only'
    return res


def on_valid(req, res):
    res.valid = bool(requests)
    return res


services = [node.create_service(Relocalize, 'relocalize', on_reloc),
            node.create_service(IsValid, 'relocalize_check', on_valid)]
pub = rviz.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
bridge = subprocess.Popen(['python3', str(Path(__file__).resolve().with_name('initialpose_to_pcd.py')),
                            '--ros-args', '-p', 'use_sim_time:=false'], env=os.environ.copy())
try:
    end = time.monotonic() + 16
    while pub.get_subscription_count() < 1 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.15)
    if pub.get_subscription_count() < 1:
        raise RuntimeError('Adapter never subscribed to /initialpose')
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x = 1.0
    pose.pose.pose.position.y = 2.0
    pose.pose.pose.orientation.z = math.sin(math.pi / 4)
    pose.pose.pose.orientation.w = math.cos(math.pi / 4)
    pub.publish(pose)
    while not requests and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.15)
    if not requests:
        raise RuntimeError('Adapter did not call 3D PCD relocalization service')
    req = requests[0]
    assert req.pcd_path.endswith('/fast_lio2_ws/maps/tb3_fast_lio2.pcd')
    assert abs(req.x - 1.0) < 1e-4
    assert abs(req.y - (2.0 - 0.032)) < 1e-4
    assert abs(req.z) < 1e-4
    assert abs(req.yaw - math.pi / 2) < 1e-4
    print(f'PASS RViz base pose (1.0,2.0,90deg) -> body ICP guess ({req.x:.3f},{req.y:.3f},z={req.z:.3f},yaw={req.yaw:.3f})', flush=True)
finally:
    bridge.terminate()
    try:
        bridge.wait(timeout=5)
    except subprocess.TimeoutExpired:
        bridge.kill()
    node.destroy_node()
    rviz.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
