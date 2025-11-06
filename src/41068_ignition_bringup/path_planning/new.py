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

        # Parameters
        self.declare_parameter('use_odom_start', True)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('z_height', 0.5)  # match snake_waypoints altitude

        self.declare_parameter('corridor_margin', 0.30)
        self.corridor_margin = float(self.get_parameter('corridor_margin').value)


        self.use_odom_start = bool(self.get_parameter('use_odom_start').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.z_height = float(self.get_parameter('z_height').value)

        # Fixed safety margin (m)
        self.safety_margin = 0.15

        #current waypoint
        self.current_waypoint = [0,0] #stores curretn waypoint

        self.current_x = 0
        self.current_y = 0

        # I/O
        self.sub_obj = self.create_subscription(Float32MultiArray, '/object_geometry', self.object_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_currentwaypoint = self.create_subscription(Float32MultiArray, '/current_waypoint', self.new_waypoint_callback, 10)
        self.pub_avoidance = self.create_publisher(Float32MultiArray, '/avoidance_waypoints', 10)

        self.get_logger().info('ObstacleBypassPlanner ready. Waiting for /object_geometry...')

    # ---------------- Callbacks ----------------

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def object_callback(self, msg: Float32MultiArray):

        for object in msg:
            centroid_x, centroid_y, radius = object[0], object[1], object[2]
            collision, avoidane_waypoints = self.path_intersects_circle(centroid_x, centroid_y, radius)
            if collision:
                msg = Float32MultiArray
                msg.data = avoidane_waypoints
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
        collision = False
        path_x = np.linspace(self.current_x, self.current_waypoint[0], n_points)
        path_y = np.linspace(self.current_y, self.current_waypoint[1], n_points)
        for i in len(path_x):
            dist = np.hypot(path_x - centroid_x, path_y - centroid_y)
            if dist <= radius:
                collision = True
                avoidance_waypoints = self.generate_avoidance
                return collision, avoidance_waypoints

    def generate_avoidance(centroid_x, centroid_y, radius):
        avoidance_waypoints = []

        avoidance_distance = radius + 0.1

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