#!/usr/bin/env python3
# Publishes snake (right-angle) waypoints as geometry_msgs/PoseArray
# and also prints them to the terminal as an array.

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
)
from geometry_msgs.msg import PoseArray, Pose


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

        # Latched QoS so late subscribers still receive the last message
        q = QoSProfile(depth=1)
        q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        q.reliability = QoSReliabilityPolicy.RELIABLE
        q.history = QoSHistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(PoseArray, 'snake_waypoints', q)

        # Generate waypoints
        coords = generate_snake_right_angles(
            min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0, step_y=0.5, z=0.5
        )

        # Print them in array form to terminal
        print("\nGenerated waypoints (x, y, z):")
        print(coords)  # compact array
        for i, (x, y, z) in enumerate(coords):
            print(f"{i:02d}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # Build PoseArray
        self.msg = PoseArray()
        self.msg.header.frame_id = 'map'
        for (x, y, z) in coords:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.w = 1.0
            self.msg.poses.append(p)

        # Publish now and every 2 seconds
        self._publish()
        self.create_timer(2.0, self._publish)
        self.get_logger().info(
            f"Publishing {len(self.msg.poses)} waypoints on /snake_waypoints"
        )

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
