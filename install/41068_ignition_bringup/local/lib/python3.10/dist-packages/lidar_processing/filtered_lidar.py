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
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Initialize publisher for /clusters topic
        self.publisher = self.create_publisher(Float32MultiArray, '/clusters', 10)
        
        # Parameters for cluster detection
        self.min_cluster_size = 5  # Minimum number of points in a cluster
        self.sigma = 0.1   # Noise tolerance (meters); adjust if needed (e.g., 0.03-0.1 based on lidar noise)
        self.duplicate_threshold = 0.20  # 10 cm for duplicate check
        
        # Store unique cluster locations (avg_x, avg_y)
        self.cluster_locations = []  # List of tuples (avg_x, avg_y)
        
        self.logger.info('FilteredLidar initialized. Clustering /scan to /clusters with point coordinates.')

    def lidar_callback(self, msg):
        # Extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid ranges (e.g., nan, inf)
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges > 0)
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
        current_cluster = [0]  # Start with the first point
        
        for i in range(1, len(points)):
            # Calculate adaptive threshold
            r1 = ranges[i-1]
            r2 = ranges[i]
            delta_a = angles[i] - angles[i-1]
            adaptive_threshold = 2 * min(r1, r2) * np.sin(delta_a / 2) + self.sigma
            
            # Calculate Euclidean distance between consecutive points
            dist = np.linalg.norm(points[i] - points[i-1])
            
            if dist <= adaptive_threshold:
                current_cluster.append(i)
            else:
                self.process_cluster(current_cluster, points)
                current_cluster = [i]
        
        # Check the final cluster
        self.process_cluster(current_cluster, points)

    def process_cluster(self, current_cluster, points):
        if len(current_cluster) >= self.min_cluster_size:
            # Calculate average x and y for the cluster
            cluster_points = points[current_cluster]
            avg_x = np.mean(cluster_points[:, 0])
            avg_y = np.mean(cluster_points[:, 1])
            location = (avg_x, avg_y)
            
            # Check if this location is too close to any existing cluster
            is_duplicate = False
            for existing_loc in self.cluster_locations:
                dist_to_existing = np.linalg.norm(np.array(location) - np.array(existing_loc))
                if dist_to_existing <= self.duplicate_threshold:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                # Add to unique locations
                self.cluster_locations.append(location)
                
                # Prepare message with all point coordinates (x, y pairs)
                coords = cluster_points.flatten()  # Flatten to [x1, y1, x2, y2, ...]
                msg = Float32MultiArray()
                msg.data = coords.tolist()
                
                # Publish the coordinates for this cluster
                self.publisher.publish(msg)
                
                # Log the cluster with location
                self.logger.info(f'Cluster found with {len(current_cluster)} points at location ({avg_x:.2f}, {avg_y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    filtered_lidar = FilteredLidar()
    rclpy.spin(filtered_lidar)
    filtered_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()