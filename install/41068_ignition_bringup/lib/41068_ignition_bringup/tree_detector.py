#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score
import struct
from time import time

class TreeDetector(Node):
    def __init__(self):
        super().__init__('tree_detector_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/filtered_clusters',
            self.cluster_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.tree_counter = 0
        self.detected_trees = []  # List of (x, y, timestamp)
        self.get_logger().info('TreeDetector initialized. Analyzing /filtered_clusters for trees.')

        self.curvature_threshold = 0.9  # R^2 <0.9 = curved tree

    def cluster_callback(self, msg: PointCloud2):
        x = []
        y = []
        offset = 0
        for _ in range(msg.width):
            x_val = struct.unpack('f', msg.data[offset:offset+4])[0]
            y_val = struct.unpack('f', msg.data[offset+4:offset+8])[0]
            x.append(x_val)
            y.append(y_val)
            offset += 12

        x = np.array(x)
        y = np.array(y)
        if len(x) < 3:
            self.get_logger().info('Cluster too small (<3 points), ignoring.')
            return

        # --- Circle fitting (Kasa method) ---
        A = np.column_stack([2*x, 2*y, np.ones_like(x)])
        b = x**2 + y**2
        try:
            sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            xc, yc, c = sol
            R = np.sqrt(c + xc**2 + yc**2)
            curvature = 1.0 / R if R > 1e-6 else float('inf')
        except np.linalg.LinAlgError:
            self.get_logger().warn('Circle fit failed, skipping cluster.')
            return

        angular_span = np.max(np.arctan2(y, x)) - np.min(np.arctan2(y, x))
        self.get_logger().info(
            f'Cluster analysis: {len(x)} points, angular span ~{angular_span:.2f} rad, '
            f'curvature = {curvature:.3f} (radius {R:.2f} m)'
        )

        # --- Decision based on curvature ---
        curvature_threshold = 0.5  # tune: higher = only strongly curved accepted
        is_tree = curvature > curvature_threshold

        centroid_x = np.mean(x)
        centroid_y = np.mean(y)

        # Deduplication
        is_new_tree = True
        current_time = time()
        for tree_x, tree_y, timestamp in self.detected_trees:
            dist = np.sqrt((centroid_x - tree_x)**2 + (centroid_y - tree_y)**2)
            if dist < 0.5 and (current_time - timestamp) < 10.0:
                is_new_tree = False
                break

        if is_tree and is_new_tree:
            self.tree_counter += 1
            self.detected_trees.append((centroid_x, centroid_y, current_time))
            self.get_logger().warn(
                f'Another tree found, tree count now {self.tree_counter}'
            )
            self.publish_marker(centroid_x, centroid_y, True)
        else:
            self.get_logger().info(
                f'Cluster not a tree (curvature {curvature:.3f}, radius {R:.2f})'
            )
            self.publish_marker(centroid_x, centroid_y, False)

    def publish_marker(self, x, y, is_tree):
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        if is_tree:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.pose.position = Point(x=x, y=y, z=0.0)
        marker.pose.orientation.w = 1.0
        marker.id = self.tree_counter if is_tree else -self.tree_counter
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    tree_detector = TreeDetector()
    rclpy.spin(tree_detector)
    tree_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()