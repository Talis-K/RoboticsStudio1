#!/usr/bin/env python3
# obstacle_bypass_planner.py
# - Subscribes: /object_geometry  (geometry_msgs/Vector3: x=cx, y=cy, z=radius)
# - Publishes : /bypass_path      (nav_msgs/Path)
# - Prints    : Path coordinates as (x, y, z) tuples, identical format to test_snake_waypoints.py
# - Clearance: R = object_radius + 0.15

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Float32MultiArray


def angle_wrap(a: float) -> float:
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def angle_diff(a: float, b: float) -> float:
    return angle_wrap(a - b)

# Plans path around object from "object_geometry" and publishes as "nav_msgs/Path"
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
        self.current_waypoint = [0,0]

        # State
        self.have_odom = False
        self.odom_xy = (self.start_x, self.start_y)

        # I/O
        self.sub_obj = self.create_subscription(Vector3, '/object_geometry', self.on_object, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.sub_currentwaypoint = self.create_subscription(Float32MultiArray, '/current_waypoint', self.new_waypoint_callback, 10)
        self.pub_path = self.create_publisher(Path, '/bypass_path', 10)

        self.get_logger().info('ObstacleBypassPlanner ready. Waiting for /object_geometry...')

    # ---------------- Callbacks ----------------

    def on_odom(self, msg: Odometry):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.have_odom = True

    def on_object(self, msg: Vector3):
        cx, cy, r_obj = msg.x, msg.y, msg.z

        # Start/goal
        if self.use_odom_start and self.have_odom:
            sx, sy = self.odom_xy
        else:
            sx, sy = self.start_x, self.start_y
        gx, gy = self.goal_x, self.goal_y

        # Clearance radius = object radius + 0.15 m
        R = r_obj + self.safety_margin
        self.get_logger().info(f'Clearance radius R = {R:.3f} m (object_radius + 0.15)')

        #Only plan a bypass if the obstacle actually blocks the straight path corridor
        if not self.obstacle_blocks_path(sx, sy, gx, gy, cx, cy, R, self.corridor_margin):
            self.get_logger().info(  
                f'Object at ({cx:.2f},{cy:.2f}) does not block the path — ignoring.' 
            )  
            return 

        # Tangents from start and goal to circle C(cx,cy,R)
        ok_s, Ts_p, Ts_m, phis_p, phis_m = self.tangent_points(sx, sy, cx, cy, R)
        ok_g, Tg_p, Tg_m, phig_p, phig_m = self.tangent_points(gx, gy, cx, cy, R)

        if not ok_s or not ok_g:
            self.get_logger().warn('Start or goal inside clearance circle. Adjust positions.')
            return

        # Option 1: CCounter clockwise path
        L1 = math.dist((sx, sy), Ts_p) + math.dist((gx, gy), Tg_p)
        A1 = self.arc_length(phis_p, phig_p, R, ccw=True)
        tot1 = L1 + A1

        # Option 2: Clockwise path
        L2 = math.dist((sx, sy), Ts_m) + math.dist((gx, gy), Tg_m)
        A2 = self.arc_length(phis_m, phig_m, R, ccw=False)
        tot2 = L2 + A2

        if tot1 <= tot2:
            side, Ts, Tg, phis, phig, ccw = 'CCW', Ts_p, Tg_p, phis_p, phig_p, True
        else:
            side, Ts, Tg, phis, phig, ccw = 'CW', Ts_m, Tg_m, phis_m, phig_m, False

        self.get_logger().info(f'Chosen side: {side}')

        # Build Path: start -> Ts -> sampled arc -> Tg -> goal
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        # Store (x, y, z) for terminal printing
        detour_xyz = []

        def push_pose(x, y):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.z_height
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            detour_xyz.append((x, y, self.z_height))

        # Compose full path
        push_pose(sx, sy)
        push_pose(Ts[0], Ts[1])
        for x, y in self.sample_arc(cx, cy, R, phis, phig, ccw, n=5):
            push_pose(x, y)
        push_pose(Tg[0], Tg[1])
        push_pose(gx, gy)

        # Publish path
        self.pub_path.publish(path)
        self.get_logger().info(f'Published /bypass_path with {len(path.poses)} poses')

        # -------- Terminal output (match test_snake_waypoints.py) --------
        print("\n[UPDATED PATH] New detour generated:")
        print("Bypass coordinates (x, y, z):")
        print(detour_xyz)
        for i, (x, y, z) in enumerate(detour_xyz):
            print(f"{i:02d}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        print("")  # spacing

    def new_waypoint_callback(self, msg: Float32MultiArray):
        """
        Updates the planner's current waypoint whenever a new waypoint is published
        to the /current_waypoint topic.
        """

        new_wp = [float(msg.data[0]), float(msg.data[1])]

        # Log old and new waypoints
        old_wp = getattr(self, "current_waypoint", None)
        self.get_logger().info(
            f"Current waypoint updated from ({old_wp[0]:.2f}, {old_wp[1]:.2f}) "
            f"to ({new_wp[0]:.2f}, {new_wp[1]:.2f})."
        )

        # Update the variable
        self.current_waypoint = new_wp

    # ---------------- Geometry helpers ----------------

    # Returns True if the clearance circle intersects the start→goal segment corridor
    def obstacle_blocks_path(self, sx, sy, gx, gy, cx, cy, R, margin):
        seg_dx, seg_dy = gx - sx, gy - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        # Degenerate segment case: treat as blocked only if start is inside the circle
        if seg_len < 1e-6:
            return math.hypot(cx - sx, cy - sy) <= (R + margin)

        # Perpendicular distance from circle centre to the infinite line through S→G
        # |(gy - sy)*cx - (gx - sx)*cy + gx*sy - gy*sx| / ||S→G||
        num = abs(seg_dy * cx - seg_dx * cy + gx * sy - gy * sx)
        dist_line = num / seg_len

        # Check whether the perpendicular foot lies within the segment bounds
        dot1 = (cx - sx) * seg_dx + (cy - sy) * seg_dy
        dot2 = (cx - gx) * (-seg_dx) + (cy - gy) * (-seg_dy)
        in_segment = (dot1 >= 0.0) and (dot2 >= 0.0)

        # Must be close to the segment AND intersect the clearance radius (+ margin)
        return in_segment and (dist_line <= (R + margin))

    def tangent_points(self, px, py, cx, cy, R):
        dx, dy = px - cx, py - cy
        d = math.hypot(dx, dy)
        if d <= R:
            return False, None, None, None, None
        theta = math.atan2(dy, dx)
        delta = math.acos(R / d)  # 0..pi/2
        phi_plus  = theta + delta   # CCW tangent contact
        phi_minus = theta - delta   # CW  tangent contact
        T_plus  = (cx + R * math.cos(phi_plus),  cy + R * math.sin(phi_plus))
        T_minus = (cx + R * math.cos(phi_minus), cy + R * math.sin(phi_minus))
        return True, T_plus, T_minus, phi_plus, phi_minus

    def arc_length(self, phi1, phi2, R, ccw=True):
        dphi = angle_diff(phi2, phi1)  # [-pi, pi)
        if ccw:
            if dphi < 0:
                dphi += 2.0 * math.pi
        else:
            if dphi > 0:
                dphi -= 2.0 * math.pi
            dphi = -dphi
        return R * abs(dphi)

    def sample_arc(self, cx, cy, R, phi1, phi2, ccw, n=40):
        pts = []
        dphi = angle_diff(phi2, phi1)
        if ccw and dphi < 0:
            dphi += 2.0 * math.pi
        if not ccw and dphi > 0:
            dphi -= 2.0 * math.pi
        step = dphi / (n + 1)
        for k in range(1, n + 1):
            a = phi1 + k * step
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a)))
        return pts


def main():
    rclpy.init()
    node = ObstacleBypassPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
