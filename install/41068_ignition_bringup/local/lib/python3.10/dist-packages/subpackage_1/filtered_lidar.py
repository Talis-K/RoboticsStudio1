#!/usr/bin/env python3
# Shebang line: Tells the system to execute this file using Python 3 interpreter.

import rclpy
# Import rclpy: The ROS Client Library for Python, used for creating nodes, publishers, subscribers, etc.

from rclpy.node import Node
# Import Node class from rclpy: Base class for creating ROS 2 nodes.

from sensor_msgs.msg import LaserScan
# Import LaserScan message type: Standard ROS message for 2D lidar data (ranges, angles, etc.).

import numpy as np
# Import NumPy: Library for numerical operations, used here for array manipulation and filtering.

from scipy.signal import medfilt
# Import medfilt from SciPy: Median filter function to smooth data and reduce noise.

class FilteredLidar(Node):
    """
    A class to filter noise from raw lidar data and publish cleaned LaserScan.
    Subscribes to /scan, removes inf/NaN, applies median filter, publishes to /filtered_scan.
    Logs raw and filtered points every 10s.
    """
    def __init__(self):
        # Constructor for FilteredLidar class.
        super().__init__('filtered_lidar_node')  # Call parent Node constructor with node name.
        self.subscription = self.create_subscription(
            # Create a subscription to listen for messages.
            LaserScan,  # Message type to subscribe to.
            '/scan',  # Topic to subscribe to (raw lidar data).
            self.lidar_callback,  # Callback function to call when message received.
            10  # QoS depth (buffer size for incoming messages).
        )
        self.publisher = self.create_publisher(
            # Create a publisher to send filtered messages.
            LaserScan,  # Message type to publish.
            '/filtered_scan',  # Topic to publish to.
            10  # QoS depth.
        )
        self.timer = self.create_timer(10.0, self.timer_callback)  # Create timer to call function every 10 seconds.
        self.last_raw_ranges = None  # Variable to store last raw ranges for logging.
        self.last_filtered_ranges = None  # Variable to store last filtered ranges for logging.
        self.publish_count = 0  # Counter to track number of publishes for reduced logging.
        self.get_logger().info('FilteredLidar initialized. Filtering /scan to /filtered_scan.')  # Log initialization message.

    def lidar_callback(self, msg: LaserScan):
        """
        Callback to filter noise:
        - Replace inf/NaN with max_range.
        - Apply median filter (kernel_size=5) to smooth outliers/noise.
        - Publish filtered LaserScan.
        """
        ranges = np.array(msg.ranges)  # Convert list of ranges to NumPy array for easier processing.
        intensities = np.array(msg.intensities)  # Convert intensities to NumPy array (preserve if used).

        # Store for timer logging (copy to avoid modifying originals).
        self.last_raw_ranges = ranges.copy()
        self.last_filtered_ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max, neginf=msg.range_max)  # Replace NaN/inf with max_range.
        self.last_filtered_ranges = medfilt(self.last_filtered_ranges, kernel_size=5)  # Apply median filter; tweak kernel_size for more/less smoothing (odd number recommended).

        # Create new LaserScan message with filtered data.
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header  # Copy header (timestamp, frame_id).
        filtered_msg.angle_min = msg.angle_min  # Copy min angle.
        filtered_msg.angle_max = msg.angle_max  # Copy max angle.
        filtered_msg.angle_increment = msg.angle_increment  # Copy angle step.
        filtered_msg.time_increment = msg.time_increment  # Copy time per ray.
        filtered_msg.scan_time = msg.scan_time  # Copy full scan time.
        filtered_msg.range_min = msg.range_min  # Copy min valid range.
        filtered_msg.range_max = msg.range_max  # Copy max valid range.
        filtered_msg.ranges = self.last_filtered_ranges.tolist()  # Convert back to list for msg.
        filtered_msg.intensities = intensities.tolist()  # Preserve intensities.

        self.publisher.publish(filtered_msg)  # Publish the filtered message.
        self.publish_count += 1  # Increment publish counter.
        if self.publish_count % 10 == 0:  # Log every 10th publish to reduce spam; tweak % 10 for frequency.
            self.get_logger().info(f'Published filtered lidar data (count: {self.publish_count}).')

    def timer_callback(self):
        """
        Timer callback to log the first 10 raw and filtered points every 10s.
        Tweak sample_size to log more/less points.
        """
        if self.last_raw_ranges is not None and self.last_filtered_ranges is not None:
            sample_size = min(10, len(self.last_raw_ranges))  # Limit to 10 or actual size if smaller.
            raw_sample = self.last_raw_ranges[:sample_size]  # Slice first N raw ranges.
            filtered_sample = self.last_filtered_ranges[:sample_size]  # Slice first N filtered ranges.
            self.get_logger().info(f'Raw points (first {sample_size}): {raw_sample}')  # Log raw sample.
            self.get_logger().info(f'Filtered points (first {sample_size}): {filtered_sample}')  # Log filtered sample.
        else:
            self.get_logger().info('No lidar data received yet.')  # Log if no data yet.

def main(args=None):
    # Main entry point to run the node.
    rclpy.init(args=args)  # Initialize ROS 2 context.
    filtered_lidar = FilteredLidar()  # Create instance of the node.
    rclpy.spin(filtered_lidar)  # Keep node alive, processing callbacks.
    filtered_lidar.destroy_node()  # Clean up node resources.
    rclpy.shutdown()  # Shut down ROS 2 context.

if __name__ == '__main__':
    # Run main if file executed directly.
    main()