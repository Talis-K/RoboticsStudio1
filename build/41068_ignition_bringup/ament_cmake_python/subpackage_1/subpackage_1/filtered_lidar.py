#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class FilteredLidar(Node):
    """
    Filter class to cluster potential trees from raw lidar data.
    Converts to 2D points, groups consecutive points ≤0.1m Euclidean distance (tweaked for occlusion).
    Minimum 3 points per cluster (reduced for small trees).
    Publishes each cluster as a PointCloud2 message.
    Processes every 5th scan. Logs cluster counts and sizes.
    """
    def __init__(self):
        super().__init__('filtered_lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, '/filtered_clusters', 10)
        self.scan_count = 0
        self.get_logger().info('FilteredLidar initialized. Clustering /scan to /filtered_clusters.')

    def lidar_callback(self, msg: LaserScan):
        self.scan_count += 1
        if self.scan_count % 5 != 0:
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        if len(valid_ranges) < 3:  # Reduced min for testing
            self.get_logger().info('No valid points for clustering.')
            return

        clusters = []
        current_cluster = {'indices': [], 'ranges': [], 'angles': []}

        for i in range(len(valid_ranges) - 1):
            idx = np.where(angles == valid_angles[i])[0][0]
            x1 = valid_ranges[i] * np.cos(valid_angles[i])
            y1 = valid_ranges[i] * np.sin(valid_angles[i])
            x2 = valid_ranges[i + 1] * np.cos(valid_angles[i + 1])
            y2 = valid_ranges[i + 1] * np.sin(valid_angles[i + 1])
            dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if dist <= 0.1:  # Increased to 10cm for occlusion/gaps
                current_cluster['indices'].append(idx)
                current_cluster['ranges'].append(valid_ranges[i])
                current_cluster['angles'].append(valid_angles[i])
            else:
                if len(current_cluster['indices']) >= 3:  # Reduced min
                    clusters.append(current_cluster)
                current_cluster = {'indices': [idx], 'ranges': [valid_ranges[i]], 'angles': [valid_angles[i]]}

        if len(current_cluster['indices']) >= 3:
            clusters.append(current_cluster)

        if clusters:
            for i, cluster in enumerate(clusters):
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                points = [(r * np.cos(a), r * np.sin(a), 0.0) for r, a in zip(cluster['ranges'], cluster['angles'])]
                cloud_data = struct.pack(f'{len(points)}f' * 3, *[coord for point in points for coord in point])
                cloud_msg = PointCloud2()
                cloud_msg.header = msg.header
                cloud_msg.height = 1
                cloud_msg.width = len(points)
                cloud_msg.fields = fields
                cloud_msg.is_bigendian = False
                cloud_msg.point_step = 12
                cloud_msg.row_step = 12 * len(points)
                cloud_msg.data = cloud_data
                cloud_msg.is_dense = True
                self.publisher.publish(cloud_msg)
                self.get_logger().info(f'Published cluster {i+1}: {len(points)} points')
            self.get_logger().info(f'Cluster found and published: {len(clusters)} clusters')
        else:
            self.get_logger().info('Published 0 clusters (no valid clusters found).')

def main(args=None):
    rclpy.init(args=args)
    filtered_lidar = FilteredLidar()
    rclpy.spin(filtered_lidar)
    filtered_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()