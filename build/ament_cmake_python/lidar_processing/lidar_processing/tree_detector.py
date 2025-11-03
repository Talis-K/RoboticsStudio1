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
        return center_x, center_y, radius

    def cluster_callback(self, msg):
        coords = np.array(msg.data)
        if len(coords) < 6:  # need at least 3 points to fit a circle
            self.logger.info('Invalid cluster data received.')
            return
        
        # Reshape into [n_points, 2] (x, y)
        num_points = len(coords) // 2
        points = coords.reshape(num_points, 2)
        
        try:
            # Fit circle
            cx, cy, radius = self.fit_circle(points)
        except Exception as e:
            self.logger.warning(f'Circle fitting failed: {e}')
            return
        
        # Classify cluster as tree or not
        if radius < 0.5:  # 50 cm threshold
            self.logger.info(
                f'Tree cluster detected with radius {radius:.2f} m at centroid ({cx:.2f}, {cy:.2f})'
            )
        else:
            self.logger.info(
                f'Cluster is not a tree, radius {radius:.2f} m at centroid ({cx:.2f}, {cy:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)
    tree_detection = TreeDetection()
    rclpy.spin(tree_detection)
    tree_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
