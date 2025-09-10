#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import logging

class TreeDetection(Node):
    def __init__(self):
        super().__init__('tree_detection_node')
        
        # Set up logging to terminal
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)
        
        # Initialize subscriber to /clusters topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/clusters',
            self.cluster_callback,
            10
        )
        
        self.logger.info('TreeDetection initialized. Subscribing to /clusters to estimate tree radii.')

    def cluster_callback(self, msg):
        # Unflatten the coordinate data into x, y pairs
        coords = np.array(msg.data)
        if len(coords) < 2:  # Need at least one x, y pair
            self.logger.info('Invalid cluster data received.')
            return
        
        # Reshape into [n_points, 2] (x, y)
        num_points = len(coords) // 2
        points = coords.reshape(num_points, 2)
        
        # Calculate centroid
        centroid_x = np.mean(points[:, 0])
        centroid_y = np.mean(points[:, 1])
        centroid = np.array([centroid_x, centroid_y])
        
        # Estimate radius as average distance from centroid to points
        distances = np.linalg.norm(points - centroid, axis=1)
        radius = np.mean(distances) if distances.size > 0 else 0.0
        
        # Log the radius
        self.logger.info(f'Tree cluster detected with radius {radius:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    tree_detection = TreeDetection()
    rclpy.spin(tree_detection)
    tree_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()