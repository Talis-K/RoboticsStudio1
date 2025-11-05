#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import logging
import math


class LidarDetection(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')

        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.tree_pub = self.create_publisher(Float32MultiArray, '/clusters', 10)
        self.geometry_pub = self.create_publisher(Float32MultiArray, '/obj_geometry', 10)
        self.people_pub = self.create_publisher(Float32MultiArray, '/people', 10)

        # Parameters
        self.min_cluster_size = 5
        self.max_point_dist = 0.13
        self.min_centroid_dist = 1
        self.human_leg_dist = 0.3

        # Odometry
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        # Data structures
        self.Trees = []   # All clusters seen
        self.people = []     # Detected humans
        self.geometries = [] # all published geometries
        self.scan_index = 0  # Incremented each scan

        self.get_logger().info('LidarDetection initialized with persistent cluster/human tracking.')

    # ===============================
    # Odometry
    # ===============================
    def odom_callback(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    # ===============================
    # Lidar callback
    # ===============================
    def lidar_callback(self, msg: LaserScan):
        self.scan_index += 1

        
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges <= 1.5)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        if len(ranges) < self.min_cluster_size:
            return

        # Convert to Cartesian coordinates (local frame)
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)
        points_local = np.vstack((x_local, y_local)).T

        # Transform to global frame
        cos_yaw = math.cos(self.current_pose['yaw'])
        sin_yaw = math.sin(self.current_pose['yaw'])
        points_global = np.zeros_like(points_local)

        for i, (x, y) in enumerate(points_local):
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            points_global[i] = [x_rot + self.current_pose['x'], y_rot + self.current_pose['y']]

        # Cluster detection
        self.detect_clusters(points_global)

    # ===============================
    # Cluster detection
    # ===============================
    def detect_clusters(self, points):
        """
        Detect clusters in a single LIDAR scan, including wrap-around at the scan boundary.
        """
        if len(points) == 0:
            return

        # Start the first cluster
        current_cluster = [0]
        scan_clusters = []

        # Loop through points in order
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist <= self.max_point_dist:
                current_cluster.append(i)
            else:
                if len(current_cluster) >= self.min_cluster_size:
                    scan_clusters.append(self.save_cluster(current_cluster, points))
                current_cluster = [i]

        # Add the last cluster from the loop
        if len(current_cluster) >= self.min_cluster_size:
            scan_clusters.append(self.save_cluster(current_cluster, points))

        if len(scan_clusters) >= 2:
            first_cluster = scan_clusters[0]
            last_cluster = scan_clusters[-1]
            dist_wrap = np.linalg.norm(first_cluster['points'][0] - last_cluster['points'][-1])
            if dist_wrap <= self.max_point_dist:
                # Merge last and first clusters
                merged_points = np.vstack((last_cluster['points'], first_cluster['points']))
                cx, cy, radius = self.fit_circle(merged_points)
                merged_cluster = {
                    'centroid': (cx, cy),
                    'radius': radius,
                    'points': merged_points,
                    'scan_index': self.scan_index
                }
                # Replace clusters
                scan_clusters = [merged_cluster] + scan_clusters[1:-1]

        # Process the detected clusters
        self.process_clusters(scan_clusters)


    # ===============================
    # Save cluster
    # ===============================   
    def save_cluster(self, indices, points):
        cluster_points = points[indices]
        cx, cy, radius = self.fit_circle(cluster_points)
        return {
            'centroid': (cx, cy),
            'radius': radius,
            'points': cluster_points,
            'scan_index': self.scan_index
        }

    # ===============================
    # Fit circle
    # ===============================
    def fit_circle(self, points):
        x, y = points[:, 0], points[:, 1]
        A = np.c_[2*x, 2*y, np.ones(len(points))]
        b = x**2 + y**2
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy = c[0], c[1]
        r = np.sqrt(c[2] + cx**2 + cy**2)

        return cx, cy, r

    # ===============================
    # Process clusters
    # ===============================
    def process_clusters(self, scan_clusters):
        humans, used_indices = self.find_humans_in_scan(scan_clusters)

        self.update_people_and_clusters(humans, scan_clusters, used_indices)

        self._publish_geometries_live()
    
    def find_humans_in_scan(self, scan_clusters):
        """Identify human candidates from pairs of close clusters in the same scan."""
        humans = []
        used_indices = set()

        for i in range(len(scan_clusters)):
            if i in used_indices:
                continue

            c1 = np.array(scan_clusters[i]['centroid'])
            for j in range(i + 1, len(scan_clusters)):
                if j in used_indices:
                    continue

                c2 = np.array(scan_clusters[j]['centroid'])
                dist = np.linalg.norm(c1 - c2)

                if dist <= self.human_leg_dist:
                    # Found two clusters close enough → person
                    human_centroid = ((c1 + c2) / 2).tolist()
                    combined_points = np.vstack((scan_clusters[i]['points'], scan_clusters[j]['points']))

                    humans.append({
                        'centroid': human_centroid,
                        'points': combined_points,
                        'scan_index': self.scan_index,
                        'source_clusters': [i, j],
                        'source_centroids': [scan_clusters[i]['centroid'], scan_clusters[j]['centroid']],
                        'source_distance': dist
                    })

                    used_indices.update([i, j])
                    break  # prevent one cluster forming multiple humans

        return humans, used_indices

    # -----------------------------------------------------------------------

    def update_people_and_clusters(self, humans, scan_clusters, used_indices):
        """Update global lists based on detected humans and remaining clusters."""

        # -----------------------------------------------------------------------
        # Process humans (first)
        # -----------------------------------------------------------------------
        for human_idx, human in enumerate(humans):
            hc = np.array(human['centroid'])
            is_existing_human = False

            # Check if it's a previously seen person
            for person_idx, person in enumerate(self.people):
                pc = np.array(person['centroid'])
                dist = np.linalg.norm(hc - pc)
                if dist <= self.min_centroid_dist:
                    is_existing_human = True
                    break

            if is_existing_human:
                continue

            # Check if it matches any existing cluster
            matched_cluster = None
            for cluster_idx, tree in enumerate(self.Trees):
                cc = np.array(tree['centroid'])
                dist = np.linalg.norm(hc - cc)
                if dist <= self.min_centroid_dist:
                    self.get_logger().info(f"→ Matches existing cluster at{tree['centroid']}, will promote to person.")
                    matched_cluster = tree
                    break

            # Combine points if matched
            if matched_cluster:
                self.Trees.remove(matched_cluster)
                combined_points = np.vstack((human['points'], matched_cluster['points']))
            else:
                combined_points = human['points']

            centroid_np = np.array(human['centroid'])
            dists = np.linalg.norm(combined_points - centroid_np, axis=1)
            max_radius = np.max(dists)

            # Add new human
            self.people.append({
                'centroid': human['centroid'],
                'radius': max_radius,
                'points': combined_points,
                'scan_index': self.scan_index
            })

            msg = Float32MultiArray()
            msg.data = combined_points.flatten().tolist()
            self.people_pub.publish(msg)

            self._publish_geometries_live()

            # Log details of the formed human
            self.get_logger().info(
                f"Published new human: "
                f"Centroid: ({human['centroid'][0]:.2f}, {human['centroid'][1]:.2f}),"
                f"radius= {max_radius}."
                f"Total points: {len(combined_points)},"
                f"Formed from clusters {human.get('source_clusters', 'unknown')},"
                f"scan_index={tree['scan_index']}"
            )
        
        #procces remaining clusters
        for idx, tree in enumerate(scan_clusters):

            if idx in used_indices:
                continue

            cc = np.array(tree['centroid'])
            is_duplicate = False

            # --- Check overlap with humans ---
            for person_idx, person in enumerate(self.people):
                pc = np.array(person['centroid'])
                dist = np.linalg.norm(cc - pc)
                if dist <= self.min_centroid_dist:
                    is_duplicate = True
                    break
            if is_duplicate:
                continue

            # --- Check overlap with existing clusters ---
            for existing_idx, existing in enumerate(self.Trees):
                ec = np.array(existing['centroid'])
                dist = np.linalg.norm(cc - ec)
                if dist <= self.min_centroid_dist:
                    # Merge points from both clusters
                    merged_points = np.vstack((existing['points'], tree['points']))
                    cx, cy, r = self.fit_circle(merged_points)

                    # Update the existing cluster in place
                    existing['points'] = merged_points
                    existing['centroid'] = (cx, cy)
                    existing['radius'] = r
                    existing['scan_index'] = self.scan_index

                    # Update geometry list to stay in sync with Trees
                    if existing_idx < len(self.geometries):
                        self.geometries[existing_idx] = [cx, cy, r]
                    else:
                        # This should only happen if Trees grew faster than geometries (e.g. first frame)
                        self.geometries.append([cx, cy, r])

                    is_duplicate = True
                    break



            # --- If not duplicate, add cluster ---
            if not is_duplicate:
                self.Trees.append(tree)
                msg = Float32MultiArray()
                msg.data = tree['points'].flatten().tolist()
                self.tree_pub.publish(msg)
                self.get_logger().info(
                    f"Published new tree: "
                    f"centroid=({tree['centroid'][0]:.2f}, {tree['centroid'][1]:.2f}), "
                    f"radius={tree['radius']:.3f},"
                    F"Total points={len(tree['points'])}, "
                    f"scan_index={tree['scan_index']}"
                )
                self._publish_geometries_live()


    def _publish_geometries_live(self):
            """Send every object (tree + person) as a Float32MultiArray right now."""
            self.geometries = []                

            for obj in self.Trees + self.people:    
                cx, cy = obj['centroid']
                r      = obj['radius']
                self.geometries.append([cx, cy, r])

                msg = Float32MultiArray()
                msg.data = [cx, cy, r]
                self.geometry_pub.publish(msg)

            self.get_logger().debug(f"Published {len(self.geometries)} live geometries")
        # ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()