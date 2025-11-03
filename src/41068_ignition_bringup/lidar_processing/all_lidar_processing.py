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

        # Logging setup (more detailed)
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        self.logger = logging.getLogger(__name__)

        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cluster_pub = self.create_publisher(Float32MultiArray, '/clusters', 10)
        self.geometry_pub = self.create_publisher(Float32MultiArray, '/circle_geometry', 10)
        self.people_pub = self.create_publisher(Float32MultiArray, '/people', 10)

        # Parameters
        self.min_cluster_size = 6
        self.min_point_dist = 0.07
        self.min_centroid_dist = 0.5
        self.human_leg_dist = 0.4

        # Odometry
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Data
        self.clusters = []
        self.people = []
        self.scan_index = 0

        self.logger.info('FilteredLidar initialized with full debug output.')

    def odom_callback(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg: LaserScan):
        self.scan_index += 1
        self.logger.info(f'--- Scan #{self.scan_index} ---')

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges <= 2.5)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        self.logger.info(f'Valid lidar points: {len(ranges)}')

        if len(ranges) < self.min_cluster_size:
            self.logger.info('Too few valid points; skipping scan.')
            return

        # Convert to Cartesian
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)
        points_local = np.vstack((x_local, y_local)).T

        # Transform to global
        cos_yaw = math.cos(self.current_pose['yaw'])
        sin_yaw = math.sin(self.current_pose['yaw'])
        points_global = np.zeros_like(points_local)

        for i, (x, y) in enumerate(points_local):
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            points_global[i] = [x_rot + self.current_pose['x'], y_rot + self.current_pose['y']]

        self.detect_clusters(points_global)

    def detect_clusters(self, points):
        current_cluster = [0]
        scan_clusters = []

        self.logger.info(f'Detecting clusters in {len(points)} points.')

        # Compute inter-point distances for debugging
        dists = [np.linalg.norm(points[i] - points[i - 1]) for i in range(1, len(points))]
        if len(dists) > 0:
            self.logger.info(f'Inter-point distances: min={min(dists):.3f}, max={max(dists):.3f}, mean={np.mean(dists):.3f}')
        else:
            self.logger.info('No inter-point distances (not enough points).')

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist <= self.min_point_dist:
                current_cluster.append(i)
            else:
                cluster = self.process_cluster(current_cluster, points)
                if cluster:
                    scan_clusters.append(cluster)
                else:
                    self.logger.debug(f'Cluster ending at index {i-1} rejected.')
                current_cluster = [i]

        cluster = self.process_cluster(current_cluster, points)
        if cluster:
            scan_clusters.append(cluster)
        else:
            self.logger.debug('Final cluster rejected.')

        if scan_clusters:
            self.logger.info(f'Scan #{self.scan_index} found {len(scan_clusters)} new clusters.')
            self.detect_people(scan_clusters)
        else:
            self.logger.info(f'Scan #{self.scan_index} found 0 clusters.')
            self.logger.info(f'Reasons may include:')
            self.logger.info(f' - Cluster size < {self.min_cluster_size}')
            self.logger.info(f' - Inter-point distance > {self.min_point_dist}')
            self.logger.info(f' - Near existing cluster (duplicate rejection)')


    def process_cluster(self, indices, points):
        if len(indices) < self.min_cluster_size:
            self.logger.debug(f'Skipping small cluster with {len(indices)} points (< {self.min_cluster_size}).')
            return None

        cluster_points = points[indices]
        cx, cy, radius = self.fit_circle(cluster_points)
        centroid = np.array([cx, cy])

        for cluster in self.clusters:
            dist_to_existing = np.linalg.norm(centroid - np.array(cluster['centroid']))
            if dist_to_existing <= self.min_centroid_dist:
                self.logger.debug(f'Cluster near existing one (Δ={dist_to_existing:.2f} < {self.min_centroid_dist}), rejecting.')
                return None

        new_cluster = {
            'centroid': (cx, cy),
            'radius': radius,
            'points': cluster_points,
            'scan_index': self.scan_index
        }
        self.clusters.append(new_cluster)

        msg = Float32MultiArray()
        msg.data = cluster_points.flatten().tolist()
        self.cluster_pub.publish(msg)
        self.logger.info(f'Cluster added #{len(self.clusters)}: ({cx:.2f}, {cy:.2f}), r={radius:.2f}, pts={len(cluster_points)})')

        return new_cluster


    def fit_circle(self, points):
        x, y = points[:, 0], points[:, 1]
        A = np.c_[2*x, 2*y, np.ones(len(points))]
        b = x**2 + y**2
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy = c[0], c[1]
        r = np.sqrt(c[2] + cx**2 + cy**2)
        return cx, cy, r

    def detect_people(self, scan_clusters):
        self.logger.info(f'Checking for human pairs among {len(scan_clusters)} clusters...')
        for i in range(len(scan_clusters)):
            for j in range(i + 1, len(scan_clusters)):
                c1 = np.array(scan_clusters[i]['centroid'])
                c2 = np.array(scan_clusters[j]['centroid'])
                dist = np.linalg.norm(c1 - c2)

                self.logger.debug(f'Cluster pair dist={dist:.2f}m')

                if dist <= self.human_leg_dist:
                    new_centroid = (c1 + c2) / 2.0
                    self.logger.info(f'Human legs detected (Δ={dist:.2f}m). Centroid=({new_centroid[0]:.2f},{new_centroid[1]:.2f})')
                    self.identify_human(new_centroid, [scan_clusters[i], scan_clusters[j]])

    def identify_human(self, centroid, leg_clusters):
        centroid = np.array(centroid)

        # Check for duplicate people
        for person in self.people:
            if np.linalg.norm(centroid - np.array(person['centroid'])) <= self.human_leg_dist:
                self.logger.info('Already known human; skipping new entry.')
                return

        removed_clusters = []
        for cluster in list(self.clusters):
            if np.linalg.norm(centroid - np.array(cluster['centroid'])) <= self.human_leg_dist:
                removed_clusters.append(cluster)
                self.clusters.remove(cluster)
                self.logger.info(f'Removed old cluster #{cluster["scan_index"]} (matched human).')

        merged_points = np.vstack([leg_clusters[0]['points'], leg_clusters[1]['points']])
        if removed_clusters:
            for rc in removed_clusters:
                merged_points = np.vstack([merged_points, rc['points']])

        new_person = {
            'centroid': tuple(centroid),
            'points': merged_points,
            'scan_index': self.scan_index
        }
        self.people.append(new_person)

        msg = Float32MultiArray()
        msg.data = [centroid[0], centroid[1]] + merged_points.flatten().tolist()
        self.people_pub.publish(msg)
        self.logger.info(f'Human added (scan {self.scan_index}): ({centroid[0]:.2f},{centroid[1]:.2f}), total_pts={len(merged_points)}')
        self.logger.info(f'Total humans tracked: {len(self.people)}')


def main(args=None):
    rclpy.init(args=args)
    node = FilteredLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
