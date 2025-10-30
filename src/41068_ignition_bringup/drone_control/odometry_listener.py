#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import threading

class OdometryListener(Node):
    _lock = threading.Lock()
    _latest = None  # [x, y, z, roll, pitch, yaw]

    def __init__(self):
        super().__init__('odometry_listener')
        self.subscription = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.get_logger().info("Odometry node started. Listening to /odom...")

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        # Quaternion → Euler (roll, pitch, yaw)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with OdometryListener._lock:
            OdometryListener._latest = [x, y, z, roll, pitch, yaw]

    @staticmethod
    def update():
        with OdometryListener._lock:
            return OdometryListener._latest.copy() if OdometryListener._latest else None


def main(args=None):
    rclpy.init(args=args)
    node = OdometryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
