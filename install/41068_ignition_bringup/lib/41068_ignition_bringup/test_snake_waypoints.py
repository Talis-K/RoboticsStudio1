#!/usr/bin/env python3
# Publishes snake (right-angle) waypoints as geometry_msgs/PoseArray,
# prints them to terminal, and also subscribes to /odom to
# print and republish the current pose on /current_pose.

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
)
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry


def generate_snake_right_angles(min_x=0.0, max_x=5.0,
                                min_y=0.0, max_y=5.0,
                                step_y=0.5, z=0.5):
    """Generate waypoints in a right-angle snake pattern."""
    wps = []
    y = min_y
    to_right = True
    wps.append((min_x, y, z))

    while y <= max_y:
        if to_right:
            wps.append((max_x, y, z))
            if y + step_y <= max_y:
                wps.append((max_x, y + step_y, z))
            y += step_y
            to_right = False
        else:
            wps.append((min_x, y, z))
            if y + step_y <= max_y:
                wps.append((min_x, y + step_y, z))
            y += step_y
            to_right = True
    return wps


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # QoS for waypoints (latched)
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        q.history = QoSHistoryPolicy.KEEP_LAST

        # Publishers
        self.wp_pub = self.create_publisher(PoseArray, 'snake_waypoints', q)
        self.pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)

        # State for periodic printing (starts at zeros if no odom yet)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Subscriber to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        print("Subscribed to /odom. Printing current pose every second and republishing on /current_pose.")

        # Periodic terminal print of current pose
        self.create_timer(1.0, self.print_pose)

        # Generate and print waypoints
        coords = generate_snake_right_angles(
            min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, step_y=0.5, z=0.5
        )
        print("\nGenerated waypoints (x, y, z):")
        print(coords)  # compact array
        for i, (x, y, z) in enumerate(coords):
            print(f"{i:02d}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # Build PoseArray
        self.wp_msg = PoseArray()
        self.wp_msg.header.frame_id = 'map'
        for (x, y, z) in coords:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.w = 1.0
            self.wp_msg.poses.append(p)

        # Publish waypoints now and every 2 seconds
        self._publish_waypoints()
        self.create_timer(2.0, self._publish_waypoints)
        self.get_logger().info(
            f"Publishing {len(self.wp_msg.poses)} waypoints on /snake_waypoints"
        )

    def _publish_waypoints(self):
        self.wp_msg.header.stamp = self.get_clock().now().to_msg()
        self.wp_pub.publish(self.wp_msg)

    def odom_callback(self, msg: Odometry):
        # Update local cache for printing
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y
        self.current_z = pos.z

        # Republish as PoseStamped on /current_pose
        ps = PoseStamped()
        ps.header = msg.header          # preserve frame_id and timestamp from /odom
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

    def print_pose(self):
        print(f"Current drone pose -> x: {self.current_x:.2f}, y: {self.current_y:.2f}, z: {self.current_z:.2f}")


def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
