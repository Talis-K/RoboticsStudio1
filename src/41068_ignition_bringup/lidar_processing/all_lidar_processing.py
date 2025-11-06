#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np


class LidarDetection(Node):
    def __init__(self):
        super().__init__('filtered_lidar_node')

        # Subscribers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.tree_pub = self.create_publisher(Float32MultiArray, '/trees', 10) # publishes detected trees,
        self.geometry_pub = self.create_publisher(Float32MultiArray, '/obj_geometry', 10) # publishes centroid and radius of detected objects 
        self.people_pub = self.create_publisher(Float32MultiArray, '/people', 10) # publishes detected people
        self.tree_count_pub = self.create_publisher(Int32, '/tree_count', 10) # publishes number of detected trees
        self.people_count_pub = self.create_publisher(Int32, '/people_count', 10) # publishes number of detected people 
        self.stump_pub = self.create_publisher(Float32MultiArray, '/stumps', 10)       # [x,y,r] one per msg (live)
        self.stump_count_pub = self.create_publisher(Int32, '/stump_count', 10)        # count per scan

        # Parameters
        self.min_cluster_size = 5 # minimum number of points to identify a cluster
        self.max_point_dist = 0.13 # maximum distance consecutive points can be from one another to not break the chain of points
        self.min_centroid_dist = 1 # minimum distance cluster centroids must be apart from one anotehr to identify as new cluster
        self.human_leg_dist = 0.3 # maximum distance two cluster can be apart from one anotehr to identify as human


        #Paramters for Tree Stump
        self.stump_radius_min = 0.12   # m  (small posts/rocks will be < this)
        self.stump_radius_max = 0.45   # m  (most trunk cut stumps < 0.45)
        self.stump_min_points = 12     # tighter than generic cluster to ensure shape quality
        self.stump_max_mean_resid = 0.03  # m, mean absolute residual from circle fit
        self.stump_max_std_resid  = 0.025 # m, residual std dev
        self.stump_min_roundness  = 0.75  # unitless, 1.0 is perfectly round

        # Odometry
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0} # initial odom with zeros

        # Data structures
        self.trees = []   # ADetected trees
        self.people = []     # Detected humans
        self.geometries = [] # all published geometries
        self.scan_index = 0  # Incremented each scan
        self.stumps = []  # detected stumps


        self.get_logger().info('LidarDetection initialized with persistent cluster/human tracking.') #initialisation log

    # ===============================
    # Odometry
    # ===============================
    def odom_callback(self, msg: Odometry): # updates current_pose with the odometry reading from /odom
        self.current_pose['x'] = msg.pose.pose.position.x # update x
        self.current_pose['y'] = msg.pose.pose.position.y # update y

        q = msg.pose.pose.orientation # calculate and update yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.current_pose['yaw'] = np.arctan2(siny_cosp, cosy_cosp)

   # ===============================
    # Stump Detetcion
    # ===============================

    def _circle_quality(self, points, cx, cy, r):
        """
        Returns (mean_abs_resid, std_resid, roundness) for how 'disk-like' a cluster is.
        - roundness ~ 1.0 means distances from centroid are very close to r.
        """
        if len(points) < 3:
            return 1e9, 1e9, 0.0
        d = np.linalg.norm(points - np.array([cx, cy]), axis=1)
        resid = np.abs(d - r)
        mean_abs = float(np.mean(resid))
        std = float(np.std(resid))
        # Roundness: 1 - normalized variance of distances (clamped to [0,1])
        # Use r as scale to keep it size-invariant; add small epsilon to avoid div/0
        eps = 1e-6 + abs(r)
        roundness = max(0.0, min(1.0, 1.0 - (np.var(d) / (eps**2))))
        return mean_abs, std, roundness

    # ===============================
    # Lidar callback
    # ===============================
    def lidar_callback(self, msg: LaserScan):
        self.scan_index += 1 # increase scan index

        ranges = np.array(msg.ranges) # convert to numpy
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) # Generate evenly spaced angles from angle_min to angle_max, matching the number of ranges
        valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges) & (ranges <= 1.5)
        ranges = ranges[valid_mask] # only ranges 0 < ranges <= 1.5
        angles = angles[valid_mask] # same indices as valid ranges

        if len(ranges) < self.min_cluster_size: # if not enough range for a cluster skip scan
            return

        # Convert to Cartesian coordinates (local frame)
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)
        points_local = np.vstack((x_local, y_local)).T

        # Transform to global frame
        cos_yaw = np.cos(self.current_pose['yaw'])
        sin_yaw = np.sin(self.current_pose['yaw'])
        points_global = np.zeros_like(points_local)

        for i, (x, y) in enumerate(points_local):
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            points_global[i] = [x_rot + self.current_pose['x'], y_rot + self.current_pose['y']] # Rotate to global orientation then add drone translation

        # Cluster detection
        self.detect_clusters(points_global)

        # Publish tree and people counts after detection
        tree_count_msg = Int32()
        tree_count_msg.data = len(self.trees)
        self.tree_count_pub.publish(tree_count_msg)

        people_count_msg = Int32()
        people_count_msg.data = len(self.people)
        self.people_count_pub.publish(people_count_msg)

        stump_count_msg = Int32()
        stump_count_msg.data = len(self.stumps)
        self.stump_count_pub.publish(stump_count_msg)


    # ===============================
    # Cluster detection
    # ===============================
    def detect_clusters(self, points):
        """
        Detect clusters in a single LIDAR scan, including wrap-around at the scan boundary.
        """

        # Start the first cluster
        current_cluster = [0] # List of point indices in the current cluster
        scan_clusters   = [] # Final list of valid clusters (after filtering)

        # Loop through points in order
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1]) # Compute Euclidean distance between current point and previous point
            if dist <= self.max_point_dist: # If points are close enough they belong to the same cluster
                current_cluster.append(i) # Add current point index to the ongoing cluster
            else: # Otherwise, gap detected, end current cluster
                if len(current_cluster) >= self.min_cluster_size: # Only save cluster if it has enough points
                    scan_clusters.append(self.save_cluster(current_cluster, points)) # calculates and adds values for clusters to array
                current_cluster = [i] # Start a new cluster with the current point

        # Add the last cluster from the loop incase there is no gap at the end
        if len(current_cluster) >= self.min_cluster_size:
            scan_clusters.append(self.save_cluster(current_cluster, points))

        #Making sure that we wrap around from 720 to 0
        if len(scan_clusters) >= 2: # only matters if there are at least 2 clusters
            first_cluster = scan_clusters[0] # first cluster
            last_cluster = scan_clusters[-1] # last cluster
            dist_wrap = np.linalg.norm(first_cluster['points'][0] - last_cluster['points'][-1]) # distance bewteen first point of first cluster and last point of last cluster
            if dist_wrap <= self.max_point_dist: # if less than max point dist then they are the same object
                # Merge last and first clusters
                merged_points = np.vstack((last_cluster['points'], first_cluster['points']))
                cx, cy, radius = self.fit_circle(merged_points)
                mean_abs, std, roundness = self._circle_quality(merged_points, cx, cy, radius)
                merged_cluster = {
                    'centroid': (cx, cy),
                    'radius': radius,
                    'points': merged_points,
                    'scan_index': self.scan_index,
                    'quality': {'mean_abs': mean_abs, 'std': std, 'roundness': roundness, 'n': len(merged_points)}
                }
                # Replace clusters
                scan_clusters = [merged_cluster] + scan_clusters[1:-1]

        humans, used_indices = self.find_humans_in_scan(scan_clusters) # Look for humans

        self.update_people_and_clusters(humans, scan_clusters, used_indices) # check agains current detected objects for duplicates

        self._publish_geometries_live() # publish newest list of obsticales to avoid


    # ===============================
    # Save cluster
    # ===============================   
    def save_cluster(self, indices, points):
        cluster_points = points[indices] #cluster points are the points from the scan at indices
        cx, cy, radius = self.fit_circle(cluster_points) # calulate values
        mean_abs, std, roundness = self._circle_quality(cluster_points, cx, cy, radius)
        return { # return values
            'centroid': (cx, cy),
            'radius': radius,
            'points': cluster_points,
            'scan_index': self.scan_index,
            'quality': {
                'mean_abs': mean_abs,
                'std': std,
                'roundness': roundness,
                'n': len(cluster_points)
        } }

    # ===============================
    # Fit circle
    # ===============================
    def fit_circle(self, points):
        """
        Fit a circle to a set of 2D points using least-squares
        """
        x, y = points[:, 0], points[:, 1] # Extract x and y coordinates from points array
        A = np.c_[2*x, 2*y, np.ones(len(points))] # Build design matrix A: [2x, 2y, 1] for each point
        b = x**2 + y**2   # Right-hand side: x� + y� for each point
        c, _, _, _ = np.linalg.lstsq(A, b, rcond=None) # Solve A*c = b � c = [a, b, d] in circle eq: x�+y� + ax + by + d = 0
        cx, cy = c[0], c[1]  # Circle center x = -a/2, y = -b/2 � but we use raw c[0], c[1]
        r = np.sqrt(c[2] + cx**2 + cy**2)  # Radius = sqrt(d + cx� + cy�) � from completing the square
        return cx, cy, r # Return center (cx, cy) and radius
        
    
    def find_humans_in_scan(self, scan_clusters):
        """Identify human candidates from pairs of close clusters in the same scan."""
        humans = [] # List to store detected humans
        used_indices = set() # Track which clusters are already used

        for i in range(len(scan_clusters)):# Loop over every cluster
            if i in used_indices:  # Skip if this cluster is already part of a human
                continue   # Go to next cluster
            c1 = np.array(scan_clusters[i]['centroid']) # Get centroid of cluster i
            for j in range(i + 1, len(scan_clusters)): # Loop over clusters after i (avoid duplicates)
                if j in used_indices: # Skip if cluster j is already used
                    continue # Go to next j
                c2 = np.array(scan_clusters[j]['centroid']) # Get centroid of cluster j
                dist = np.linalg.norm(c1 - c2) # Distance between two centroids
                if dist <= self.human_leg_dist: # If within distance then identify as human
                    human_centroid = ((c1 + c2) / 2).tolist()  # Midpoint = human center
                    combined_points = np.vstack((scan_clusters[i]['points'], scan_clusters[j]['points']))  # Merge points
                    humans.append({  # Save human data
                        'centroid': human_centroid,      
                        'points': combined_points,      
                        'scan_index': self.scan_index,   
                        'source_clusters': [i, j], 
                        'source_centroids': [scan_clusters[i]['centroid'], scan_clusters[j]['centroid']],  # Leg centers
                        'source_distance': dist  
                    })
                    used_indices.update([i, j]) # Mark both clusters as used
                    break   
        return humans, used_indices # Return humans and used cluster list

    # -----------------------------------------------------------------------

    def update_people_and_clusters(self, humans, scan_clusters, used_indices):
        """Update global lists based on detected humans and remaining clusters."""
        for human in humans: # Loop through each new human from this scan
            hc = np.array(human['centroid'])  # Human centroid as numpy array
            is_existing_human = False # Flag: is this person already in self.people?
            # Check if it's a previously seen person
            for person in self.people:  # Loop through known people
                pc = np.array(person['centroid']) # Known person centroid
                dist = np.linalg.norm(hc - pc)  # Distance between new and old
                if dist <= self.min_centroid_dist: # within min dist then same person
                    is_existing_human = True # Mark as existing
                    break # Stop checking
            if is_existing_human:  # If already known, skip
                continue # Go to next human
            # Check if human matches any existing cluster
            matched_cluster = None # Will hold tree to promote
            for tree in self.trees:# Loop through stored trees
                cc = np.array(tree['centroid'])# Tree centroid
                dist = np.linalg.norm(hc - cc)# Distance to tree
                if dist <= self.min_centroid_dist: # within min dist then promote tree to person
                    self.get_logger().info(f"Human at ({human['centroid'][0]}, {human['centroid'][1]}) matches existing cluster at {tree['centroid']}, will promote to person.")
                    matched_cluster = tree # Save reference
                    break  # Stop checking
            # Combine points if matched
            if matched_cluster: # If promoting a tree
                self.trees.remove(matched_cluster)# Remove from trees
                combined_points = np.vstack((human['points'], matched_cluster['points']))  # Merge points
            else: # No match therefore new human
                combined_points = human['points'] # Use only leg points
            centroid_np = np.array(human['centroid'])# Human center as array
            dists = np.linalg.norm(combined_points - centroid_np, axis=1) # Distance from center to each point
            max_radius = np.max(dists) # Farthest point = radius, for avoidance purposes
            # Add new human
            self.people.append({  # Save to global people list
                'centroid': human['centroid'],                      
                'radius': max_radius,                         
                'points': combined_points,                           
                'scan_index': self.scan_index             
            })

            # Log details of the formed human
            self.get_logger().info(
                f"Published new human: "
                f"Centroid: ({human['centroid'][0]:.2f}, {human['centroid'][1]:.2f}), "
                f"radius= {max_radius:.3f}. "
                f"Total points: {len(combined_points)}, "
                f"Formed from clusters {human.get('source_clusters', 'unknown')}, "
                f"scan_index={self.scan_index}"
            )
        # Process remaining clusters
        for idx, tree in enumerate(scan_clusters): # Loop through all clusters from this scan
            if idx in used_indices: # Skip if used in a human
                continue # Next cluster
            cc = np.array(tree['centroid']) # Current cluster center
            is_duplicate = False # Flag: already exists?

            # --- Check overlap with humans ---
            for person_idx, person in enumerate(self.people):# Loop through known people
                pc = np.array(person['centroid'])   # Person center
                dist = np.linalg.norm(cc - pc)  # Distance to person
                if dist <= self.min_centroid_dist: # Too close then ignore
                    is_duplicate = True  # Mark as duplicate
                    break # Stop checking
            if is_duplicate: # If too close to a person
                continue # Skip this cluster
            # --- Check overlap with existing clusters ---
            for existing_idx, existing in enumerate(self.trees): # Loop through stored trees
                ec = np.array(existing['centroid']) # Existing tree center
                dist = np.linalg.norm(cc - ec) # Distance between trees
                if dist <= self.min_centroid_dist:  # if close then merge for more detailed point cloud

                    # Merge points from both clusters
                    merged_points = np.vstack((existing['points'], tree['points']))  # Combine points
                    cx, cy, r = self.fit_circle(merged_points) # Refit circle
                    # Update the existing cluster in place
                    existing['points'] = merged_points # Update points
                    existing['centroid'] = (cx, cy) # Update center
                    existing['radius'] = r   # Update radius
                    existing['scan_index'] = tree['scan_index'] # keep same scan

                    # Update geometry list to stay in sync with Trees
                    if existing_idx < len(self.geometries): # If geometry exists
                        self.geometries[existing_idx] = [cx, cy, r] # Update it
                    else:  # Rare: geometries list too short
                        self.geometries.append([cx, cy, r]) # Add new
                    is_duplicate = True  # Mark as merged
                    break  # Stop checking

            if self._is_stump(tree):
                # Avoid duplicates with existing stumps
                is_dup_stump = False
                for existing in self.stumps:
                    if np.linalg.norm(np.array(tree['centroid']) - np.array(existing['centroid'])) <= self.min_centroid_dist:
                        # Merge for better estimate
                        merged_points = np.vstack((existing['points'], tree['points']))
                        cx, cy, r = self.fit_circle(merged_points)
                        mean_abs, std, roundness = self._circle_quality(merged_points, cx, cy, r)
                        existing.update({
                            'points': merged_points,
                            'centroid': (cx, cy),
                            'radius': r,
                            'scan_index': tree['scan_index'],
                            'quality': {'mean_abs': mean_abs, 'std': std, 'roundness': roundness, 'n': len(merged_points)}
                        })
                        is_dup_stump = True
                        break
                if not is_dup_stump:
                    self.stumps.append(tree)
                    # publish one stump as [cx, cy, r]
                    msg = Float32MultiArray()
                    cx, cy = tree['centroid']
                    msg.data = [float(cx), float(cy), float(tree['radius'])]
                    self.stump_pub.publish(msg)
                    self.get_logger().info(
                        f"Published new stump: centroid=({cx:.2f}, {cy:.2f}), r={tree['radius']:.3f}, "
                        f"n={tree['quality']['n']}, mean_abs={tree['quality']['mean_abs']:.3f}, "
                        f"std={tree['quality']['std']:.3f}, roundness={tree['quality']['roundness']:.2f}"
                    )



            # --- If not duplicate, add cluster ---
            if not is_duplicate: # New tree
                self.trees.append(tree) # Save to global trees
                msg = Float32MultiArray() # Create message
                msg.data = tree['points'].flatten().tolist() # Flatten points
                self.tree_pub.publish(msg)  # Publish raw points
                self.get_logger().info(
                    f"Published new tree: "
                    f"centroid=({tree['centroid'][0]:.2f}, {tree['centroid'][1]:.2f}), "
                    f"radius={tree['radius']:.3f}, "
                    f"Total points={len(tree['points'])}, "
                    f"scan_index={tree['scan_index']}"
                )

  
    def _is_stump(self, cluster) -> bool:
        """Decide if a single cluster is a stump by size and circularity quality."""
        r = float(cluster['radius'])
        q = cluster.get('quality', {})
        n = int(q.get('n', 0))
        if n < self.stump_min_points:
            return False
        if not (self.stump_radius_min <= r <= self.stump_radius_max):
            return False
        mean_abs = float(q.get('mean_abs', 1e9))
        std = float(q.get('std', 1e9))
        roundness = float(q.get('roundness', 0.0))
        if mean_abs > self.stump_max_mean_resid: 
            return False
        if std > self.stump_max_std_resid: 
            return False
        if roundness < self.stump_min_roundness:
            return False
        return True





    def _publish_geometries_live(self):
        """Send every object (tree + person) as a Float32MultiArray right now."""
        self.geometries = []# Clear old geometry list
        for obj in self.trees + self.people + self.stumps:# Loop through all trees and people
            cx, cy = obj['centroid'] # Get center x, y
            r = obj['radius']# Get radius
            self.geometries.append([cx, cy, r])# Save [cx, cy, r] to list
            msg = Float32MultiArray() # Create new message
            msg.data = [cx, cy, r] # Pack center and radius
            self.geometry_pub.publish(msg) # Publish one object

    # ------------------------------------------------------------------
    def end(self):
        """
        Print everything and publish raw point clouds at shutdown.
        """
        # ---- 1. TREES ----
        print("\n--- Detected Trees ---") # Header
        for i, tree in enumerate(self.trees):   # Loop through all stored trees
            print(f"Tree {i}:") # Tree number
            print(f" Centroid : {tree['centroid']}") # Center position
            print(f" Radius   : {tree['radius']:.3f}") # Radius
            print(f" Points   : {len(tree['points'])}") # Number of points
            print(f" Scan idx : {tree['scan_index']}")# Which scan
            # publish tree point cloud
            msg = Float32MultiArray() # New message
            msg.data = tree['points'].flatten().tolist() # Flatten 2D points to 1D list
            self.tree_pub.publish(msg) # Send raw points

        # ---- 2. PEOPLE ----
        print("\n--- Detected People ---") # Header
        for i, person in enumerate(self.people): # Loop through all people
            print(f"Person {i}:") # Person number
            print(f" Centroid : {person['centroid']}") # Center
            print(f" Radius   : {person['radius']:.3f}") # Radius
            print(f" Points   : {len(person['points'])}") # Points
            print(f" Scan idx : {person['scan_index']}") # Scan index
            # publish person point cloud
            msg = Float32MultiArray() # New message
            msg.data = person['points'].flatten().tolist()  # Flatten points
            self.people_pub.publish(msg) # Send raw points

        # ---- 3. GEOMETRIES ----
        print("\n--- Object Avoidance ---") # Header
        for i, geom in enumerate(self.geometries):  # Loop through saved geometries
            cx, cy, r = geom  # Unpack center and radius
            print(f"Geometry {i}:") # Geometry number
            print(f" Centroid : {cx:.2f}, {cy:.2f}") # Center
            print(f" Radius   : {r:.4f}") # Radius

    # ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)                
    node = LidarDetection()        
    rclpy.spin(node)               
    node.end()                  
    node.destroy_node()    
    rclpy.shutdown()    

if __name__ == '__main__':
    main()   