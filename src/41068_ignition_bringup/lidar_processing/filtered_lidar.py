#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import logging

class FilteredLidar(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')
        
        # Set up logging to terminal
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Initialize subscriber to /scan topic
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # Publishers
        self.publisher = self.create_publisher(Float32MultiArray, '/clusters', 10)
        self.publish_geometry = self.create_publisher(Float32MultiArray, '/circle_geometry', 10)
        
        # Parameters for cluster detection
        self.min_cluster_size = 5      # Minimum number of points in a cluster
        self.sigma = 0.1               # Noise tolerance (meters)
        self.duplicate_threshold = 0.30  # 30 cm threshold for new centroid acceptance
        
        # Store detected clusters: [{'centroid': (x, y), 'points': np.ndarray}]
        self.clusters = []
        
        self.logger.info('FilteredLidar initialized. Clustering /scan to /clusters with circle fitting.')

    def lidar_callback(self, msg):
        # Extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid ranges (e.g., nan, inf)
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges <= 20.0)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) < self.min_cluster_size:
            self.logger.info('Not enough valid points to form a cluster.')
            return
        
        # Convert to Cartesian coordinates (x, y)
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.vstack((x, y)).T  # Shape: (N, 2)
        
        # Detect clusters
        self.detect_clusters(points, valid_angles, valid_ranges)

    def detect_clusters(self, points, angles, ranges):
        current_cluster = [0]
        
        for i in range(1, len(points)):
            # Calculate adaptive threshold
            r1 = ranges[i - 1]
            r2 = ranges[i]
            delta_a = angles[i] - angles[i - 1]
            adaptive_threshold = 2 * min(r1, r2) * np.sin(delta_a / 2) + self.sigma
            
            # Calculate Euclidean distance between consecutive points
            dist = np.linalg.norm(points[i] - points[i - 1])
            
            if dist <= adaptive_threshold:
                current_cluster.append(i)
            else:
                self.process_cluster(current_cluster, points)
                current_cluster = [i]
        
        # Process the last cluster
        self.process_cluster(current_cluster, points)

    def process_cluster(self, current_cluster, points):
        if len(current_cluster) >= self.min_cluster_size:
            cluster_points = points[current_cluster]

            # Fit a circle to get accurate centroid + radius
            cx, cy, radius = self.fit_circle(cluster_points)
            new_centroid = np.array([cx, cy])

            # Check if this centroid is far enough from existing ones
            for cluster in self.clusters:
                dist = np.linalg.norm(new_centroid - np.array(cluster['centroid']))
                if dist <= self.duplicate_threshold:
                    return  # Too close therefore skip

            # Store new cluster
            self.clusters.append({
                'centroid': (cx, cy),
                'radius' : radius,
                'points': cluster_points
            })

            # Publish cluster points
            msg = Float32MultiArray()
            msg.data = cluster_points.flatten().tolist()
            self.publisher.publish(msg)

            self.logger.info(f'New cluster: center=({cx:.2f}, {cy:.2f}), '
                             f'radius={radius:.2f}, points={len(cluster_points)}')

    def fit_circle(self, points: np.ndarray):
        """
        Fit a circle to 2D points using least-squares.
        Returns (center_x, center_y, radius).
        """
        x = points[:, 0]
        y = points[:, 1]
        A = np.c_[2*x, 2*y, np.ones(points.shape[0])]
        b = x**2 + y**2
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        center_x, center_y = c[0], c[1]
        radius = np.sqrt(c[2] + center_x**2 + center_y**2)

        # Publish geometry (center and radius)
        geom = Float32MultiArray()
        geom.data = [center_x, center_y, radius]
        self.publish_geometry.publish(geom)

        return center_x, center_y, radius

def main(args=None):
    rclpy.init(args=args)
    filtered_lidar = FilteredLidar()
    rclpy.spin(filtered_lidar)
    filtered_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
