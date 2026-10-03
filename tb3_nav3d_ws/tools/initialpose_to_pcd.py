#!/usr/bin/env python3
"""Bridge RViz 2D Pose Estimate to ICP relocalization against the saved 3D PCD.

/initialpose is an approximate base_footprint pose, NOT a claim of successful
localization. Never publish a map TF here; only the ICP localizer may do that.
Flat Gazebo world only: the clicked pose has no Z information, so assume the
IMU body is at its original PCD map elevation (default z=0).
"""
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize, IsValid


class InitialPoseToPCD(Node):
    def __init__(self):
        super().__init__('initialpose_to_pcd')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pcd_path', str(Path(__file__).resolve().parents[2] / 'fast_lio2_ws/maps/tb3_fast_lio2.pcd'))
        self.declare_parameter('body_z_on_flat_ground', 0.0)
        self.relocalize = self.create_client(Relocalize, '/pcd_localizer/relocalize')
        self.check = self.create_client(IsValid, '/pcd_localizer/relocalize_check')
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_pose, 10)
        self.pending = None
        self.checking = None
        self.deadline = None
        self.generation = 0
        self.timer = self.create_timer(0.7, self.check_alignment)
        self.get_logger().info('RViz 2D Pose Estimate -> 3D PCD ICP; waiting on /initialpose')

    def on_pose(self, msg):
        expected = self.get_parameter('map_frame').value
        if msg.header.frame_id.lstrip('/') != expected:
            self.get_logger().error(f'Initial pose frame {msg.header.frame_id!r}; set RViz Fixed Frame to {expected!r}')
            return
        if self.pending is not None and not self.pending.done():
            self.get_logger().warning('Previous PCD load pending; wait before clicking again')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error('Initial pose contains non-finite coordinates')
            return
        qnorm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if qnorm < 1e-6:
            self.get_logger().error('Initial orientation is invalid')
            return
        x, y, z, w = (q.x/qnorm, q.y/qnorm, q.z/qnorm, q.w/qnorm)
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        path = Path(self.get_parameter('pcd_path').value)
        if not path.is_file() or path.stat().st_size == 0:
            self.get_logger().error(f'Missing/nonempty PCD required: {path}')
            return
        if not self.relocalize.service_is_ready():
            self.get_logger().error('PCD localizer service not ready; start start-localizer.sh first, then click again')
            return
        req = Relocalize.Request()
        req.pcd_path = str(path)
        # The click represents base_footprint. URDF says base_footprint->imu_link
        # is (-0.032, 0, 0.078); FAST-LIO body is the IMU origin.
        req.x = float(p.x - 0.032*math.cos(yaw))
        req.y = float(p.y - 0.032*math.sin(yaw))
        req.z = float(self.get_parameter('body_z_on_flat_ground').value)
        req.yaw = float(yaw)
        req.pitch = 0.0
        req.roll = 0.0
        self.generation += 1
        self.checking = None
        self.deadline = None
        self.get_logger().info(f'Loading PCD and proposing IMU body initial guess x={req.x:.2f}, y={req.y:.2f}, z={req.z:.2f}, yaw={yaw:.2f} rad')
        self.pending = self.relocalize.call_async(req)
        gen = self.generation
        self.pending.add_done_callback(lambda future: self.on_loaded(future, gen))

    def on_loaded(self, future, gen):
        if gen != self.generation:
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'PCD load service failed: {exc}')
            return
        if not result.success:
            self.get_logger().error(f'PCD could not be loaded: {result.message}')
            return
        self.deadline = self.get_clock().now().nanoseconds / 1e9 + 18.0
        self.get_logger().info('PCD loaded, waiting for ICP; load success is NOT localization success')

    def check_alignment(self):
        if self.deadline is None:
            return
        if self.get_clock().now().nanoseconds / 1e9 > self.deadline:
            self.get_logger().warning('ICP has not converged in 18 simulated seconds; click a better guess, or check point cloud/map overlap')
            self.deadline = None
            return
        if self.checking is None and self.check.service_is_ready():
            self.checking = self.check.call_async(IsValid.Request(code=0))
            gen = self.generation
            self.checking.add_done_callback(lambda future: self.on_checked(future, gen))

    def on_checked(self, future, gen):
        if gen != self.generation or self.deadline is None:
            return
        self.checking = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'ICP check service error: {exc}')
            return
        if result.valid:
            self.get_logger().info('ICP 3D PCD localization VALID; map->camera_init TF is available')
            self.deadline = None


def main():
    rclpy.init()
    node = InitialPoseToPCD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
