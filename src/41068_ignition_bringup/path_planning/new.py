#!/usr/bin/env python3
# obstacle_bypass_planner.py
# - Subscribes: /object_geometry  (geometry_msgs/Vector3: x=cx, y=cy, z=radius)
# - Publishes : /bypass_path      (nav_msgs/Path)
# - Prints    : Path coordinates as (x, y, z) tuples, identical format to test_snake_waypoints.py
# - Clearance: R = object_radius + 0.15

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Float32MultiArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class ObstacleBypassPlanner(Node):
    def __init__(self):
        super().__init__('obstacle_bypass_planner')

        self.get_logger().info('Object avoidance') #initialisation log

        self.buffer_dist = 0.1

        #current waypoint
        self.current_waypoint = [0,0] #stores current waypoint

        self.current_x = 0
        self.current_y = 0

        # I/O
        self.sub_obj = self.create_subscription(Float32MultiArray, '/obj_geometry', self.object_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_currentwaypoint = self.create_subscription(Float32MultiArray, '/current_waypoint', self.new_waypoint_callback, 10)
        self.pub_avoidance = self.create_publisher(Float32MultiArray, '/avoidance_waypoints', 10)

    # ---------------- Callbacks ----------------

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def object_callback(self, msg: Float32MultiArray):

        data = msg.data
        num_objects = len(data) // 3

        for i in range(num_objects):
            centroid_x = data[i * 3]
            centroid_y = data[i * 3 + 1]
            radius = data[i * 3 + 2]
            collision, avoidance_waypoints = self.path_intersects_circle(centroid_x, centroid_y, radius)
            if collision:
                print('COLLISION DETECTED --------------------------------------------------------------------------------------------------------------------')
                msg = Float32MultiArray()
                msg.data = [float(x) for pair in avoidance_waypoints for x in pair]
                self.pub_avoidance.publish(msg)
                
    def new_waypoint_callback(self, msg: Float32MultiArray):
        """
        Updates the planner's current waypoint whenever a new waypoint is published
        to the /current_waypoint topic.
        """

        self.current_waypoint = [float(msg.data[0]), float(msg.data[1])]

        # Log old and new waypoints
        old_wp = getattr(self, "current_waypoint", None)
        self.get_logger().info(
            f"Current waypoint updated from ({old_wp[0]:.2f}, {old_wp[1]:.2f}) "
            f"to ({self.current_waypoint[0]:.2f}, {self.current_waypoint[1]:.2f})."
        )

    def path_intersects_circle(self, centroid_x, centroid_y, radius, n_points=30):
        """
        Generate n_points evenly spaced between current position and current waypoint.
        Check if any of those points are within 'radius' distance of (centroid_x, centroid_y).
        """
        collision = False

        # Generate path samples
        path_x = np.linspace(self.current_x, self.current_waypoint[0], n_points)
        path_y = np.linspace(self.current_y, self.current_waypoint[1], n_points)

        for i in range(n_points):
            dist = np.hypot(path_x[i] - centroid_x, path_y[i] - centroid_y)
            if dist <= radius:
                print(f"Collision detected at path point ({path_x[i]:.2f}, {path_y[i]:.2f}) | distance={dist:.3f}")
                collision = True
                avoidance_waypoints = self.generate_avoidance(centroid_x, centroid_y, radius)
                return collision, avoidance_waypoints

        # If no collision found
        return collision, []


    def generate_avoidance(self, centroid_x, centroid_y, radius):
        avoidance_waypoints = []

        avoidance_distance = radius + self.buffer_dist

        avoidance_waypoints.append([centroid_x+avoidance_distance, centroid_y])

        return avoidance_waypoints


def main():
    rclpy.init()
    node = ObstacleBypassPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()