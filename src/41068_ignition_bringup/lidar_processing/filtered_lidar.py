#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import logging
import math

class FilteredLidar(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')
        
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.publisher = self.create_publisher(Float32MultiArray, '/clusters', 10)
        self.publish_geometry = self.create_publisher(Float32MultiArray, '/circle_geometry', 10)
        
        # Cluster parameters
        self.min_cluster_size = 6
        self.min_point_dist = 0.07
        self.min_centroid_dist = 0.5
        
        # Odometry state
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}  # Global frame
        
        # Detected clusters
        self.clusters = []
        
        self.logger.info('FilteredLidar initialized with incremental cluster merging and odometry transformation.')

    def odom_callback(self, msg: Odometry):
        # Extract position
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges <= 2.5)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) < self.min_cluster_size:
            return
        
        # Convert to local Cartesian coordinates
        x_local = valid_ranges * np.cos(valid_angles)
        y_local = valid_ranges * np.sin(valid_angles)
        points_local = np.vstack((x_local, y_local)).T
        
        # Transform points to global frame using current odometry
        cos_yaw = math.cos(self.current_pose['yaw'])
        sin_yaw = math.sin(self.current_pose['yaw'])
        points_global = np.zeros_like(points_local)
        for i, (x, y) in enumerate(points_local):
            # Rotation
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            # Translation
            points_global[i, 0] = x_rot + self.current_pose['x']
            points_global[i, 1] = y_rot + self.current_pose['y']
        
        self.detect_clusters(points_global, valid_angles, valid_ranges)

    def detect_clusters(self, points, angles, ranges):
        current_cluster_indices = [0]
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i-1])
            if dist <= self.min_point_dist:
                current_cluster_indices.append(i)
            else:
                self.process_cluster(current_cluster_indices, points)
                current_cluster_indices = [i]
        self.process_cluster(current_cluster_indices, points)

    def process_cluster(self, cluster_indices, points):
        if len(cluster_indices) < self.min_cluster_size:
            return

        cluster_points = points[cluster_indices]
        cx, cy, radius = self.fit_circle(cluster_points)
        new_centroid = np.array([cx, cy])

        # Check if the new cluster is too close to existing clusters
        for cluster in self.clusters:
            old_centroid = np.array(cluster['centroid'])
            if np.linalg.norm(new_centroid - old_centroid) <= self.min_centroid_dist:
                # Too close, ignore this cluster
                return

        # Add new cluster
        self.clusters.append({
            'centroid': (cx, cy),
            'radius': radius,
            'points': cluster_points,
            'published': False
        })

        # Publish cluster points
        msg = Float32MultiArray()
        msg.data = cluster_points.flatten().tolist()
        self.publisher.publish(msg)
        self.logger.info(f'New cluster published: center=({cx:.2f}, {cy:.2f}), '
                        f'radius={radius:.2f}, points={len(cluster_points)}')

        # Mark as published
        self.clusters[-1]['published'] = True


    def fit_circle(self, points: np.ndarray):
        x = points[:, 0]
        y = points[:, 1]
        A = np.c_[2*x, 2*y, np.ones(points.shape[0])]
        b = x**2 + y**2
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy = c[0], c[1]
        radius = np.sqrt(c[2] + cx**2 + cy**2)
        
        geom = Float32MultiArray()
        geom.data = [cx, cy, radius]
        self.publish_geometry.publish(geom)
        
        return cx, cy, radius

def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
