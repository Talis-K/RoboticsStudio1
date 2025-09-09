#!/usr/bin/env python3
# Shebang line: Tells the system to execute this file using Python 3 interpreter.

import rclpy
# Import rclpy: The ROS Client Library for Python, used for creating nodes, publishers, subscribers, etc.

from rclpy.node import Node
# Import Node class from rclpy: Base class for creating ROS 2 nodes.

from sensor_msgs.msg import LaserScan
# Import LaserScan message type: Standard ROS message for 2D lidar data (ranges, angles, etc.).

from std_msgs.msg import Bool
# Import Bool message type: Simple true/false message for detection output.

import numpy as np
# Import NumPy: Library for numerical operations, used here for array manipulation and clustering.

class TreeDetector(Node):
    """
    A class to detect trees from filtered lidar data.
    Subscribes to /filtered_scan, checks for tree patterns (e.g., clusters of close ranges), publishes Bool to /tree_detected.
    Logs detection and increments a counter.
    """
    def __init__(self):
        # Constructor for TreeDetector class.
        super().__init__('tree_detector_node')  # Call parent Node constructor with node name.
        self.subscription = self.create_subscription(
            # Create a subscription to listen for messages.
            LaserScan,  # Message type to subscribe to.
            '/filtered_scan',  # Topic to subscribe to (filtered lidar data).
            self.lidar_callback,  # Callback function to call when message received.
            10  # QoS depth (buffer size for incoming messages).
        )
        self.publisher = self.create_publisher(
            # Create a publisher to send detection messages.
            Bool,  # Message type to publish.
            '/tree_detected',  # Topic to publish to.
            10  # QoS depth.
        )
        self.tree_counter = 0  # Initialize counter for detected trees (resets on restart; tweak to persist if needed).
        self.timer = self.create_timer(1/3.0, self.timer_callback)  # Timer for 3 Hz logging (every ~0.33s).
        self.last_detection_time = self.get_clock().now()  # Track last callback time for no-detection log.
        self.get_logger().info('TreeDetector initialized. Detecting from /filtered_scan.')

        # Detection params (tweak these for your simulation environment)
        self.min_tree_distance = 0.5  # Min range to consider an obstacle (m) - increase to ignore close noise, decrease for sensitivity.
        self.max_tree_distance = 5.0  # Max range for tree detection (m) - decrease if trees are closer, increase for farther.
        self.min_cluster_size = 5  # Min number of consecutive points in a cluster to be a "tree" - lower for small trees, higher for larger.
        self.tree_angle_threshold = 0.2  # Max angle span for a cluster to be a "tree" (radians, ~11.5 degrees) - adjust based on lidar resolution/tree width.

    def lidar_callback(self, msg: LaserScan):
        """
        Callback to detect trees:
        - Find clusters of consecutive ranges within min/max distance.
        - If cluster size/width matches "tree" pattern, detect as true.
        - Publish Bool (true if any tree detected).
        """
        ranges = np.array(msg.ranges)  # Convert list of ranges to NumPy array.
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)  # Generate angle array for the scan.

        # Step 1: Mask ranges that could be obstacles (within thresholds).
        obstacle_mask = (ranges > self.min_tree_distance) & (ranges < self.max_tree_distance)

        # Step 2: Find cluster starts and ends using diff (robust to full-mask).
        diff_mask = np.diff(obstacle_mask.astype(int))  # Diff to find transitions (1: start, -1: end).
        cluster_starts = np.where(diff_mask == 1)[0] + 1  # Starts after transition to True.
        cluster_ends = np.where(diff_mask == -1)[0] + 1  # Ends after transition to False.

        # Handle full-mask case (all obstacles: one cluster from 0 to len-1).
        if np.all(obstacle_mask):
            cluster_starts = [0]
            cluster_ends = [len(ranges)]

        # Handle wrap-around if first and last are True (circular scan).
        if obstacle_mask[0] and obstacle_mask[-1]:
            cluster_starts = np.insert(cluster_starts, 0, cluster_starts[-1] - len(ranges))  # Adjust for wrap.
            cluster_ends = np.append(cluster_ends, cluster_ends[0] + len(ranges))  # Adjust for wrap.
            # Note: This is simplified; for precise, calculate combined span.

        is_tree_detected = False  # Flag for detection in this scan.
        num_clusters = len(cluster_starts)  # Count clusters for logging.

        for i in range(len(cluster_starts)):
            start = cluster_starts[i]
            end = cluster_ends[i] if i < len(cluster_ends) else len(ranges)  # Safe end index.
            if end <= start:  # Skip invalid clusters.
                continue

            cluster_size = end - start  # Number of points in cluster.
            cluster_angle_span = angles[end - 1] - angles[start]  # Angle width of cluster (last to first).

            # Tree detection logic: Check if cluster matches tree pattern.
            if cluster_size >= self.min_cluster_size and cluster_angle_span <= self.tree_angle_threshold:
                is_tree_detected = True
                break  # Detect if at least one tree-like cluster.

        # Publish detection result.
        detection_msg = Bool()  # Create Bool message.
        detection_msg.data = is_tree_detected  # Set true/false.
        self.publisher.publish(detection_msg)  # Publish to /tree_detected.

        # Update last time for timer (even if no detection).
        self.last_detection_time = self.get_clock().now()

    def timer_callback(self):
        """
        Timer callback to log at 3 Hz.
        Logs detection with counter if detected, or "No tree detected" if none in last period.
        """
        current_time = self.get_clock().now()  # Get current time.
        time_since_last = (current_time - self.last_detection_time).nanoseconds / 1e9  # Seconds since last callback.
        is_tree_detected = time_since_last < 0.33  # Approximate if detection happened recently (since 3 Hz = 0.33s).

        if is_tree_detected:
            self.tree_counter += 1  # Increment counter on detection.
            self.get_logger().warn(f'Tree detected! Total trees detected: {self.tree_counter}')  # Log with counter.
        else:
            self.get_logger().info('No tree detected.')  # Log if no detection.

def main(args=None):
    # Main entry point to run the node.
    rclpy.init(args=args)  # Initialize ROS 2 context.
    tree_detector = TreeDetector()  # Create instance of the node.
    rclpy.spin(tree_detector)  # Keep node alive, processing callbacks.
    tree_detector.destroy_node()  # Clean up node resources.
    rclpy.shutdown()  # Shut down ROS 2 context.

if __name__ == '__main__':
    # Run main if file executed directly.
    main()