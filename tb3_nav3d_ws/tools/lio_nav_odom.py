#!/usr/bin/env python3
"""Adapt Fast-LIO2 body pose to Nav2 base_footprint odom, without Gazebo pose.

Fast-LIO2's /Odometry has an IMU-body pose but does not populate twist. Nav2
requires body-frame velocity for controller smoothing, so estimate twist from
successive timestamped LIO poses. This node publishes no TF.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class NavOdomBridge(Node):
    def __init__(self):
        super().__init__('lio_nav_odom')
        self.previous = None
        self.publisher = self.create_publisher(Odometry, '/lio/nav_odom', 10)
        self.subscription = self.create_subscription(Odometry, '/Odometry', self.cb, 10)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        sin_y = 2.0 * (q.w * q.z + q.x * q.y)
        cos_y = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_y, cos_y)
        # body is IMU origin; inverse of URDF base_footprint->imu_link
        # (-0.032, 0, 0.078). Use full 3D rotation for the lever arm.
        offset = (0.032, 0.0, -0.078)
        ux = q.w * offset[0] + q.y * offset[2] - q.z * offset[1]
        uy = q.w * offset[1] + q.z * offset[0] - q.x * offset[2]
        uz = q.w * offset[2] + q.x * offset[1] - q.y * offset[0]
        px = msg.pose.pose.position.x + offset[0] + 2 * (q.y * uz - q.z * uy)
        py = msg.pose.pose.position.y + offset[1] + 2 * (q.z * ux - q.x * uz)
        pz = msg.pose.pose.position.z + offset[2] + 2 * (q.x * uy - q.y * ux)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = pz
        out.pose.pose.orientation = q
        out.pose.covariance = msg.pose.covariance
        if self.previous:
            old_t, old_x, old_y, old_yaw = self.previous
            dt = t - old_t
            if 0 < dt < 1.0:
                vx, vy = (px - old_x) / dt, (py - old_y) / dt
                out.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
                out.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
                dyaw = math.atan2(math.sin(yaw - old_yaw), math.cos(yaw - old_yaw))
                out.twist.twist.angular.z = dyaw / dt
        out.twist.covariance[0] = out.twist.covariance[7] = 0.05
        out.twist.covariance[35] = 0.05
        self.previous = (t, px, py, yaw)
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = NavOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
