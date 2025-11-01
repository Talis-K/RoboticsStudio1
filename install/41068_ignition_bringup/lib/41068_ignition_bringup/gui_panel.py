#!/usr/bin/env python3
"""
Mission Console GUI (ROS 2, Tkinter)
- iOS-like light theme
- Camera (auto raw/compressed)
- LiDAR (LaserScan or PointCloud2) stabilised via TF2 into 'odom'
- Clustering (eps/min_pts)
- Robot marker from Odometry (XY + yaw) and altitude bar
- E-STOP publish + subscribe
- Extra mission/health: Battery, GPS, IMU (RPY), Flight mode
- Waypoints (Path or PoseArray), cut-tree detections (PoseArray)
- Breadcrumb trail, next-waypoint dashed leg, tree pins
"""

import io
import math
import time
import queue
import threading
from typing import Optional, Tuple, List

import numpy as np
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from PIL import Image as PILImage
from PIL import ImageTk

# ROS messages
from sensor_msgs.msg import Image, LaserScan, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import BatteryState, Imu, NavSatFix

# TF2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

RAW_IMAGE_TYPE  = 'sensor_msgs/msg/Image'
COMP_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
SUPPORTED_RAW   = {'rgb8','bgr8','mono8','rgba8','bgra8'}


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """2D yaw from quaternion."""
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)


# ------------------------------ ROS NODE ------------------------------
class GuiNode(Node):
    def __init__(self, msg_queue: queue.Queue, img_queue: queue.Queue):
        super().__init__('gui_panel_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('estop_topic', '/e_stop')
        self.declare_parameter('max_altitude', 10.0)

        # Extra (optional) topics for drone missions
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('flight_mode_topic', '/flight_mode')

        # Waypoints & detections
        self.declare_parameter('waypoints_topic', '')          # subscribe to Path and/or PoseArray
        self.declare_parameter('detections_topic', '/trees/cut')

        self.msg_queue = msg_queue
        self.img_queue = img_queue

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_best_effort)
        self.get_logger().info(f"[GUI] scan: {scan_topic}")

        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_best_effort)
            self.get_logger().info(f"[GUI] cloud: {cloud_topic}")

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(f"[GUI] odom: {odom_topic}")

        # E-STOP pub + sub
        etopic = self.get_parameter('estop_topic').get_parameter_value().string_value
        self.estop_pub = self.create_publisher(Bool, etopic, 10)
        self._estop = False
        self.create_subscription(Bool, etopic, self.on_estop_msg, 10)

        # Camera discovery
        self._img_sub = None
        self._cam_type: Optional[str] = None
        self._requested_image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._discovery_timer = self.create_timer(1.0, self._ensure_camera_subscription)
        self._ensure_camera_subscription(initial=True)

        # Telemetry / pose
        self.altitude_m: Optional[float] = None
        self.position_xy: Optional[Tuple[float, float]] = None
        self.yaw_rad: Optional[float] = None

        # Mission/health state
        self.battery_pct: Optional[float] = None
        self.flight_mode: Optional[str] = None
        self.gps_fix: Optional[NavSatFix] = None
        self.imu_rpy: Optional[Tuple[float, float, float]] = None

        self.breadcrumb: List[Tuple[float, float]] = []
        self.breadcrumb_max = 200

        self.waypoints_xy: List[Tuple[float, float]] = []
        self.next_wp_idx: int = 0

        self.tree_positions_xy: List[Tuple[float, float]] = []

        # LiDAR prefilter
        self._scan_keep_every = 2
        self._scan_min_valid = 0.03

        # TF2 buffer/listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Time sanity
        self._last_scan_stamp_ns: Optional[int] = None

        # Subscribe to extra topics
        self._subscribe_extras()

    # ---- extra subscriptions ----
    def _subscribe_extras(self):
        bt = self.get_parameter('battery_topic').get_parameter_value().string_value
        self.create_subscription(BatteryState, bt, self.on_battery, 10)

        gt = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.create_subscription(NavSatFix, gt, self.on_gps, 10)

        it = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.create_subscription(Imu, it, self.on_imu, 10)

        fmt = self.get_parameter('flight_mode_topic').get_parameter_value().string_value
        self.create_subscription(String, fmt, self.on_flight_mode, 10)

        # Waypoints: specific topic if provided; otherwise scan and subscribe to any Path / PoseArray with likely names
        wpt = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        if wpt:
            # It's safe to create both; only the matching type will deliver messages
            self.create_subscription(Path, wpt, self.on_path, 10)
            self.create_subscription(PoseArray, wpt, self.on_posearray_waypoints, 10)
        else:
            for name, types in self.get_topic_names_and_types():
                if 'nav_msgs/msg/Path' in types:
                    self.create_subscription(Path, name, self.on_path, 10)
                if 'geometry_msgs/msg/PoseArray' in types and 'waypoint' in name.lower():
                    self.create_subscription(PoseArray, name, self.on_posearray_waypoints, 10)

        # Tree detections (PoseArray of world coords)
        dt = self.get_parameter('detections_topic').get_parameter_value().string_value
        if dt:
            self.create_subscription(PoseArray, dt, self.on_tree_detections, 10)

    # ---- E-STOP ----
    def engage_estop(self):
        if not self._estop:
            self._estop = True
            try:
                self.estop_pub.publish(Bool(data=True))
            except Exception:
                pass
            self.get_logger().warn("[GUI] EMERGENCY STOP ENGAGED")

    def reset_estop(self):
        if self._estop:
            self._estop = False
            try:
                self.estop_pub.publish(Bool(data=False))
            except Exception:
                pass
            self.get_logger().info("[GUI] E-stop reset")

    def estop_active(self) -> bool:
        return self._estop

    def on_estop_msg(self, msg: Bool):
        self._estop = bool(msg.data)

    # ---- Camera discovery ----
    def _ensure_camera_subscription(self, initial: bool = False):
        if self._img_sub is not None:
            if self._discovery_timer:
                self._discovery_timer.cancel()
                self._discovery_timer = None
            return

        cam_topic, cam_type = self._resolve_camera_topic(self._requested_image_topic)
        if cam_topic is None:
            if initial:
                self.get_logger().info("[GUI] Waiting for camera topic&")
            return

        if cam_type == 'raw':
            self._img_sub = self.create_subscription(Image, cam_topic, self.on_image_raw, 10)
        else:
            self._img_sub = self.create_subscription(CompressedImage, cam_topic, self.on_image_compressed, 10)
        self._cam_type = cam_type
        self.get_logger().info(f"[GUI] camera: {cam_topic} ({cam_type})")

    def _resolve_camera_topic(self, requested: str) -> Tuple[Optional[str], Optional[str]]:
        if requested:
            for n, ts in self.get_topic_names_and_types():
                if n == requested and (RAW_IMAGE_TYPE in ts or COMP_IMAGE_TYPE in ts):
                    return (requested, 'raw') if RAW_IMAGE_TYPE in ts else (requested, 'compressed')
            self.get_logger().warn(f"[GUI] image topic not found yet: {requested}")
            return None, None

        topics_and_types = self.get_topic_names_and_types()
        for n, ts in topics_and_types:
            if RAW_IMAGE_TYPE in ts:
                return n, 'raw'
        for n, ts in topics_and_types:
            if COMP_IMAGE_TYPE in ts:
                return n, 'compressed'
        return None, None

    # ---- Callbacks ----
    def on_scan(self, msg: LaserScan):
        if self._estop:
            return

        # Drop out-of-order timestamps
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_scan_stamp_ns is not None and stamp_ns < self._last_scan_stamp_ns:
            return
        self._last_scan_stamp_ns = stamp_ns

        # Build points in scan frame (decimated & speckle-filtered)
        pts = []
        ang = msg.angle_min
        k = max(1, int(self._scan_keep_every))
        for i, r in enumerate(msg.ranges):
            if i % k != 0:
                ang += msg.angle_increment
                continue
            if (msg.range_min < r < msg.range_max and math.isfinite(r) and r > self._scan_min_valid):
                pts.append((r * math.cos(ang), r * math.sin(ang)))
            ang += msg.angle_increment

        # Transform to odom (stable world frame)
        src_frame = msg.header.frame_id or 'laser'
        target_frame = 'odom'  # change to 'map' if that's your fixed world frame
        try:
            tfm = self.tf_buffer.lookup_transform(
                target_frame, src_frame, msg.header.stamp, timeout=Duration(seconds=0.05)
            )
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            q = tfm.transform.rotation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            pts_world = []
            for x, y in pts:
                X = cos_y*x - sin_y*y + tx
                Y = sin_y*x + cos_y*y + ty
                pts_world.append((X, Y))
            pts = pts_world
            frame_used = target_frame
        except (LookupException, ConnectivityException, ExtrapolationException):
            frame_used = src_frame  # fallback: still show points

        try:
            self.msg_queue.put({
                'src': 'scan',
                'stamp': f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}",
                'frame': frame_used,
                'n_total': len(pts),
                'points_xy': pts
            }, block=False)
        except queue.Full:
            pass

    def on_cloud(self, msg: PointCloud2):
        if self._estop:
            return
        pts = []
        for p in pc2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
            pts.append((float(p[0]), float(p[1])))
        try:
            self.msg_queue.put({
                'src': 'cloud',
                'stamp': f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}",
                'frame': msg.header.frame_id,
                'n_total': len(pts),
                'points_xy': pts
            }, block=False)
        except queue.Full:
            pass

    def on_image_raw(self, msg: Image):
        if self._estop:
            return
        try:
            if msg.encoding not in SUPPORTED_RAW:
                self.get_logger().warn(f"raw image encoding not supported: {msg.encoding}")
                return
            w, h = msg.width, msg.height
            img = PILImage.frombytes('RGB' if 'rgb' in msg.encoding else 'L', (w, h), bytes(msg.data))
            if msg.encoding in ('bgr8', 'bgra8'):
                img = PILImage.fromarray(np.array(img)[:, :, ::-1])
            self.img_queue.put_nowait(img)
        except Exception as e:
            self.get_logger().error(f"[GUI] raw image decode failed: {e}")

    def on_image_compressed(self, msg: CompressedImage):
        if self._estop:
            return
        try:
            bio = io.BytesIO(bytes(msg.data))
            img = PILImage.open(bio).convert('RGB')
            self.img_queue.put_nowait(img)
        except Exception as e:
            self.get_logger().error(f"[GUI] compressed image decode failed: {e}")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.altitude_m = float(p.z)
        self.position_xy = (float(p.x), float(p.y))
        q = msg.pose.pose.orientation
        self.yaw_rad = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # Breadcrumb trail
        if self.position_xy is not None:
            self.breadcrumb.append(self.position_xy)
            if len(self.breadcrumb) > self.breadcrumb_max:
                self.breadcrumb.pop(0)

    # ---- extras callbacks ----
    def on_battery(self, msg: BatteryState):
        if msg.percentage is not None and math.isfinite(msg.percentage):
            self.battery_pct = max(0.0, min(1.0, float(msg.percentage)))
        else:
            self.battery_pct = None

    def on_flight_mode(self, msg: String):
        self.flight_mode = msg.data.strip()

    def on_gps(self, msg: NavSatFix):
        self.gps_fix = msg

    def on_imu(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_rpy = (roll, pitch, yaw)

    def on_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waypoints_xy = pts
        if self.position_xy and pts:
            rx, ry = self.position_xy
            self.next_wp_idx = min(range(len(pts)), key=lambda i: (rx - pts[i][0]) ** 2 + (ry - pts[i][1]) ** 2)

    def on_posearray_waypoints(self, msg: PoseArray):
        pts = [(p.position.x, p.position.y) for p in msg.poses]
        self.waypoints_xy = pts
        if self.position_xy and pts:
            rx, ry = self.position_xy
            self.next_wp_idx = min(range(len(pts)), key=lambda i: (rx - pts[i][0]) ** 2 + (ry - pts[i][1]) ** 2)

    def on_tree_detections(self, msg: PoseArray):
        self.tree_positions_xy = [(p.position.x, p.position.y) for p in msg.poses]


# ------------------------------ GUI ------------------------------
class App:
    def __init__(self, node: GuiNode, q_scan: queue.Queue, q_img: queue.Queue):
        self.node = node
        self.q_scan = q_scan
        self.q_img = q_img

        self.root = tk.Tk()
        self.root.title("Mission Console")
        self.root.geometry("1280x880")
        self.root.minsize(1100, 760)
        self._apply_ios_theme()

        # Header
        header = ttk.Frame(self.root, padding=(16, 12, 16, 8), style="Header.TFrame")
        header.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(header, text="Mission Console", style="Header.TLabel").pack(side=tk.LEFT)
        self.fps_lbl = ttk.Label(header, text="", style="HeaderSub.TLabel")
        self.fps_lbl.pack(side=tk.RIGHT)

        # Body
        body = ttk.Frame(self.root, padding=(16, 8, 16, 16))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=3)
        body.columnconfigure(2, weight=0)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=0)

        # Camera
        cam_card = ttk.LabelFrame(body, text="Camera", padding=10, style="Card.TLabelframe")
        cam_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.cam_label = ttk.Label(cam_card, text="(waiting for image)", anchor="center")
        self.cam_label.pack(fill="both", expand=True)

        # LiDAR
        lidar_card = ttk.LabelFrame(body, text="LiDAR (top-down)", padding=10, style="Card.TLabelframe")
        lidar_card.grid(row=0, column=1, sticky="nsew")
        self.canvas = tk.Canvas(lidar_card, width=520, height=540, bg="#FFFFFF", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Controls
        controls = ttk.Frame(lidar_card)
        controls.pack(fill="x", pady=(8, 0))
        self.show_points_var = tk.BooleanVar(value=True)
        self.show_clusters_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="Points", variable=self.show_points_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(controls, text="Clusters", variable=self.show_clusters_var).grid(row=0, column=1, sticky="w", padx=(12, 0))
        ttk.Label(controls, text="eps (m):").grid(row=0, column=2, sticky="e", padx=(16, 4))
        self.eps_var = tk.DoubleVar(value=0.30)
        ttk.Spinbox(controls, from_=0.05, to=2.0, increment=0.05, width=5, textvariable=self.eps_var).grid(row=0, column=3, sticky="w")
        ttk.Label(controls, text="min pts:").grid(row=0, column=4, sticky="e", padx=(12, 4))
        self.minpts_var = tk.IntVar(value=4)
        ttk.Spinbox(controls, from_=1, to=40, increment=1, width=5, textvariable=self.minpts_var).grid(row=0, column=5, sticky="w")

        # Safety & Telemetry
        side = ttk.LabelFrame(body, text="Safety & Telemetry", padding=12, style="Card.TLabelframe")
        side.grid(row=0, column=2, sticky="ns", padx=(8, 0))

        self.estop_btn = tk.Button(
            side, text="EMERGENCY\nSTOP",
            font=("Helvetica Neue", 18, "bold"),
            width=12, height=3, bg="#FF3B30", fg="white", bd=0,
            activebackground="#FF453A", activeforeground="white",
            cursor="hand2", command=self.on_estop_press
        )
        self.estop_btn.pack(fill="x", pady=(0, 8))
        self.reset_btn = tk.Button(
            side, text="Reset", font=("Helvetica Neue", 12, "bold"),
            bg="#E5E5EA", fg="#111111", bd=0, cursor="hand2",
            activebackground="#D1D1D6", activeforeground="#111111",
            command=self.on_estop_reset
        )
        self.reset_btn.pack(fill="x")
        self.status_lbl = ttk.Label(side, text="Status: NORMAL")
        self.status_lbl.pack(fill="x", pady=(10, 14))

        # Telemetry readouts
        telem = ttk.Frame(side)
        telem.pack(fill="x")
        ttk.Label(telem, text="Altitude:").grid(row=0, column=0, sticky="e")
        self.alt_bar = ttk.Progressbar(telem, mode="determinate", length=120, maximum=self._max_altitude())
        self.alt_bar.grid(row=0, column=1, sticky="w", padx=(6, 0))
        self.lbl_alt = ttk.Label(telem, text=" m"); self.lbl_alt.grid(row=0, column=2, sticky="w", padx=(6, 0))

        ttk.Label(telem, text="Position:").grid(row=1, column=0, sticky="e", pady=(6, 0))
        self.lbl_xy = ttk.Label(telem, text=""); self.lbl_xy.grid(row=1, column=1, columnspan=2, sticky="w", padx=(6, 0), pady=(6, 0))

        ttk.Label(telem, text="Yaw:").grid(row=2, column=0, sticky="e", pady=(6, 0))
        self.lbl_yaw = ttk.Label(telem, text=""); self.lbl_yaw.grid(row=2, column=1, columnspan=2, sticky="w", padx=(6, 0), pady=(6, 0))

        # Battery
        ttk.Label(telem, text="Battery:").grid(row=3, column=0, sticky="e", pady=(6, 0))
        self.batt_bar = ttk.Progressbar(telem, mode="determinate", length=120, maximum=100)
        self.batt_bar.grid(row=3, column=1, sticky="w", padx=(6, 0))
        self.lbl_batt = ttk.Label(telem, text=""); self.lbl_batt.grid(row=3, column=2, sticky="w", padx=(6, 0))

        # GPS
        ttk.Label(telem, text="GPS:").grid(row=4, column=0, sticky="e", pady=(6, 0))
        self.lbl_gps = ttk.Label(telem, text=""); self.lbl_gps.grid(row=4, column=1, columnspan=2, sticky="w", padx=(6, 0))

        # Flight mode
        ttk.Label(telem, text="Mode:").grid(row=5, column=0, sticky="e", pady=(6, 0))
        self.lbl_mode = ttk.Label(telem, text=""); self.lbl_mode.grid(row=5, column=1, columnspan=2, sticky="w", padx=(6, 0))

        # IMU RPY
        ttk.Label(telem, text="IMU RPY:").grid(row=6, column=0, sticky="e", pady=(6, 0))
        self.lbl_rpy = ttk.Label(telem, text=""); self.lbl_rpy.grid(row=6, column=1, columnspan=2, sticky="w", padx=(6, 0))

        # Waypoints & Detections summary
        wp_card = ttk.LabelFrame(body, text="Mission (Waypoints & Trees)", padding=12, style="Card.TLabelframe")
        wp_card.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        wp_card.columnconfigure(0, weight=1)
        wp_card.columnconfigure(1, weight=1)
        wp_card.columnconfigure(2, weight=1)

        self.lbl_wp = ttk.Label(wp_card, text="Waypoints: 0  |  Next:   |  ETA: ")
        self.lbl_wp.grid(row=0, column=0, sticky="w")

        self.lbl_trees = ttk.Label(wp_card, text="Cut-tree detections: 0  |  Last: ")
        self.lbl_trees.grid(row=0, column=1, sticky="w")

        self.mission_cmd_pub = None
        try:
            self.mission_cmd_pub = self.node.create_publisher(String, '/mission/command', 10)
            cmd_row = ttk.Frame(wp_card); cmd_row.grid(row=0, column=2, sticky="e")
            ttk.Button(cmd_row, text="Pause", command=lambda: self._send_cmd('pause')).pack(side=tk.LEFT, padx=4)
            ttk.Button(cmd_row, text="Resume", command=lambda: self._send_cmd('resume')).pack(side=tk.LEFT, padx=4)
            ttk.Button(cmd_row, text="RTL", command=lambda: self._send_cmd('rtl')).pack(side=tk.LEFT, padx=4)
            ttk.Button(cmd_row, text="Land", command=lambda: self._send_cmd('land')).pack(side=tk.LEFT, padx=4)
        except Exception:
            pass

        # Internal state
        self._last_photo = None
        self._cam_max_w = 900
        self._cam_max_h = 560
        self._last_draw_ts = 0.0
        self._max_fps = 20.0          # rate limit drawing
        self._static_bounds = None     # cache to avoid redrawing grid
        self._fixed_view = True
        self._fixed_range_m = 8.0
        self._bounds_ema = None
        self._bounds_alpha = 0.2

        # Pollers
        self.root.after(50, self.poll_img)
        self.root.after(80, self.poll_scan)
        self.root.after(150, self.poll_altitude)
        self.root.bind("<space>", lambda e: self.on_estop_press())
        self.root.bind("<r>",      lambda e: self.on_estop_reset())

    # ---- iOS-like Theme ----
    def _apply_ios_theme(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        bg   = '#F2F2F7'  # grouped background
        card = '#FFFFFF'
        fg   = '#1C1C1E'
        sub  = '#8E8E93'
        acc  = '#0A84FF'
        self.root.configure(bg=bg)
        style.configure('.', background=bg, foreground=fg, font=("Helvetica Neue", 11))
        style.configure('Header.TFrame', background=bg)
        style.configure('Header.TLabel', background=bg, foreground=fg, font=("Helvetica Neue", 18, 'bold'))
        style.configure('HeaderSub.TLabel', background=bg, foreground=sub)
        style.configure('Footer.TLabel', background=bg, foreground=sub)
        style.configure('Card.TLabelframe', background=card, foreground=fg, relief='flat', borderwidth=0)
        style.configure('Card.TLabelframe.Label', background=card, foreground=sub, font=("Helvetica Neue", 11, 'bold'))
        style.configure('TLabel', background=bg, foreground=fg)
        style.configure('TCheckbutton', background=card)
        style.configure('TFrame', background=bg)
        style.configure('Accent.TButton', padding=6)
        style.map('Accent.TButton', background=[('active', acc)], foreground=[('active', 'white')])

    def _max_altitude(self) -> float:
        try:
            return float(self.node.get_parameter('max_altitude').get_parameter_value().double_value)
        except Exception:
            return 10.0

    # ---- Safety ----
    def on_estop_press(self):
        self.node.engage_estop()

    def on_estop_reset(self):
        self.node.reset_estop()

    def on_quit(self):
        try:
            self.node.destroy_node(); rclpy.shutdown()
        finally:
            self.root.destroy()

    def _send_cmd(self, cmd: str):
        if self.mission_cmd_pub:
            try:
                self.mission_cmd_pub.publish(String(data=cmd))
            except Exception:
                pass

    # ---- Camera ----
    def poll_img(self):
        img = None
        try:
            while True:
                img = self.q_img.get_nowait()
        except queue.Empty:
            pass

        if img is not None:
            img_disp = img.copy()
            img_disp.thumbnail((self._cam_max_w, self._cam_max_h))
            photo = ImageTk.PhotoImage(image=img_disp)
            self._last_photo = photo
            self.cam_label.configure(image=photo, text="")
        self.root.after(50, self.poll_img)

    # ---- LiDAR + Clusters ----
    def poll_scan(self):
        item = None
        try:
            while True:
                item = self.q_scan.get_nowait()
        except queue.Empty:
            pass

        now = time.time()
        if now - self._last_draw_ts < (1.0 / self._max_fps):
            self.root.after(40, self.poll_scan)
            return

        if item is not None:
            pts = item.get('points_xy', [])
            clusters: List[List[Tuple[float, float]]] = []
            if self.show_clusters_var.get() and pts:
                clusters = self.euclidean_clusters(pts, eps=self.eps_var.get(), min_pts=self.minpts_var.get())
            self.redraw_scatter(
                pts if self.show_points_var.get() else [],
                clusters,
                src=item.get('src', ''),
                frame=item.get('frame', ''),
                stamp=item.get('stamp', ''),
                n=item.get('n_total', '')
            )
            self._update_header_fps(now)
        self.root.after(40, self.poll_scan)

    def poll_altitude(self):
        # Altitude
        z = self.node.altitude_m
        if z is None:
            self.lbl_alt.config(text=" m")
            self.alt_bar['value'] = 0
        else:
            self.lbl_alt.config(text=f"{z:.2f} m")
            self.alt_bar['value'] = max(0.0, min(self._max_altitude(), z))

        # XY, yaw
        xy = self.node.position_xy
        self.lbl_xy.config(text="" if xy is None else f"({xy[0]:.2f}, {xy[1]:.2f}) m")
        yaw = self.node.yaw_rad
        self.lbl_yaw.config(text="" if yaw is None else f"{math.degrees(yaw):.1f}�")

        # Reflect E-STOP state
        self.status_lbl.config(text="Status: E-STOP ENGAGED" if self.node.estop_active() else "Status: NORMAL")

        # Battery
        bp = self.node.battery_pct
        if bp is not None:
            self.batt_bar['value'] = int(round(bp * 100))
            self.lbl_batt.config(text=f"{bp * 100:.0f}%")
        else:
            self.batt_bar['value'] = 0
            self.lbl_batt.config(text="")

        # GPS
        fix = self.node.gps_fix
        if fix is not None and math.isfinite(fix.latitude) and math.isfinite(fix.longitude):
            lat = f"{fix.latitude:.5f}"; lon = f"{fix.longitude:.5f}"
            alt = f"{fix.altitude:.1f}" if math.isfinite(fix.altitude) else ""
            fq = { -1: "NF", 0: "NF", 1: "GPS", 2: "DGPS" }.get(getattr(fix.status, 'status', 0), "?")
            self.lbl_gps.config(text=f"{lat}, {lon}  ({alt} m, {fq})")
        else:
            self.lbl_gps.config(text="")

        # Flight mode
        self.lbl_mode.config(text=self.node.flight_mode or "")

        # IMU RPY
        if self.node.imu_rpy:
            r, p, y = self.node.imu_rpy
            self.lbl_rpy.config(text=f"{math.degrees(r):.0f} / {math.degrees(p):.0f} / {math.degrees(y):.0f}�")
        else:
            self.lbl_rpy.config(text="")

        # Waypoints & Trees summary
        n_wp = len(self.node.waypoints_xy)
        nexti = self.node.next_wp_idx if n_wp else -1
        eta = ""
        if self.node.position_xy and n_wp and 0 <= nexti < n_wp:
            rx, ry = self.node.position_xy; wx, wy = self.node.waypoints_xy[nexti]
            dist = math.hypot(wx - rx, wy - ry)
            # naive ETA at 3 m/s (tune as needed)
            eta = f"{dist / 3.0:.0f}s"
        self.lbl_wp.config(text=f"Waypoints: {n_wp}  |  Next: {nexti if nexti>=0 else ''}  |  ETA: {eta}")

        nt = len(self.node.tree_positions_xy)
        last_tree = self.node.tree_positions_xy[-1] if nt else None
        self.lbl_trees.config(text=f"Cut-tree detections: {nt}  |  Last: {last_tree if last_tree else ''}")

        self.root.after(200, self.poll_altitude)

    # ---- Utils: clustering & drawing ----
    @staticmethod
    def euclidean_clusters(pts_xy: List[Tuple[float, float]], eps: float = 0.3, min_pts: int = 4) -> List[List[Tuple[float, float]]]:
        if not pts_xy:
            return []
        pts = np.asarray(pts_xy, dtype=float)
        N = len(pts)
        visited = np.zeros(N, dtype=bool)
        clusters: List[List[Tuple[float, float]]] = []
        eps2 = eps * eps
        for i in range(N):
            if visited[i]:
                continue
            queue_idx = [i]
            visited[i] = True
            cluster_idx = [i]
            while queue_idx:
                j = queue_idx.pop()
                diff = pts - pts[j]
                dist2 = (diff[:, 0] ** 2 + diff[:, 1] ** 2)
                nbrs = np.where((dist2 <= eps2) & (~visited))[0]
                for k in nbrs.tolist():
                    visited[k] = True
                    queue_idx.append(k)
                    cluster_idx.append(k)
            if len(cluster_idx) >= min_pts:
                clusters.append([tuple(pts[k]) for k in cluster_idx])
        return clusters

    @staticmethod
    def _convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        P = sorted(set(points))
        if len(P) <= 2:
            return P
        def cross(o, a, b):
            return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
        lower = []
        for p in P:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        upper = []
        for p in reversed(P):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        return lower[:-1] + upper[:-1]

    def _nice_step(self, span: float) -> float:
        if span <= 0:
            return 1.0
        raw = span / 8.0
        mag = 10 ** math.floor(math.log10(raw))
        for m in (1, 2, 5, 10):
            step = m * mag
            if raw <= step:
                return step
        return 10 * mag

    def _update_header_fps(self, now: float):
        dt = now - self._last_draw_ts
        self._last_draw_ts = now
        if dt > 0:
            self.fps_lbl.config(text=f"{1.0/dt:5.1f} FPS")

    def _compute_bounds(self, pts_xy, clusters):
        if self._fixed_view:
            r = self._fixed_range_m
            if self.node.position_xy is not None:
                rx, ry = self.node.position_xy
            else:
                rx, ry = 0.0, 0.0
            return rx - r, rx + r, ry - r, ry + r
        # auto: use EMA to smooth
        all_pts = pts_xy[:]
        for c in clusters:
            all_pts.extend(c)
        if all_pts:
            arr = np.asarray(all_pts, dtype=float)
            xs, ys = arr[:, 0], arr[:, 1]
            x0, x1 = np.percentile(xs, [5, 95]); y0, y1 = np.percentile(ys, [5, 95])
            if x0 == x1: x0, x1 = x0 - 1.0, x1 + 1.0
            if y0 == y1: y0, y1 = y0 - 1.0, y1 + 1.0
        else:
            x0, x1, y0, y1 = -5, 5, -5, 5
        if self._bounds_ema is None:
            self._bounds_ema = [x0, x1, y0, y1]
        else:
            bx0, bx1, by0, by1 = self._bounds_ema
            a = self._bounds_alpha
            self._bounds_ema = [
                a * x0 + (1 - a) * bx0,
                a * x1 + (1 - a) * bx1,
                a * y0 + (1 - a) * by0,
                a * y1 + (1 - a) * by1,
            ]
        return self._bounds_ema

    def _draw_static_grid(self, x0, x1, y0, y1):
        self.canvas.delete("static")
        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        pad = 20
        grid_c = "#E5E5EA"  # light grey
        axes_c = "#C7C7CC"
        text_c = "#3A3A3C"

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2 * pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2 * pad) / (y1 - y0))
            return X, Y

        self.canvas.create_rectangle(0, 0, w, h, fill="#FFFFFF", outline="", tags="static")

        step = self._nice_step(max(x1 - x0, y1 - y0))
        x = math.floor(x0 / step) * step
        while x <= x1:
            X, _ = to_pix(x, 0)
            self.canvas.create_line(X, pad, X, h - pad, fill=grid_c, tags="static")
            x += step
        y = math.floor(y0 / step) * step
        while y <= y1:
            _, Y = to_pix(0, y)
            self.canvas.create_line(pad, Y, w - pad, Y, fill=grid_c, tags="static")
            y += step

        ox, oy = to_pix(0.0, 0.0)
        self.canvas.create_line(ox, h - pad, ox, pad, fill=axes_c, tags="static")
        self.canvas.create_line(pad, oy, w - pad, oy, fill=axes_c, tags="static")

        sx0, sy0 = to_pix(x0 + 0.8 * (x1 - x0) - 1.2, y0 + 0.1 * (y1 - y0))
        sx1, sy1 = to_pix(x0 + 0.8 * (x1 - x0) - 0.2, y0 + 0.1 * (y1 - y0))
        self.canvas.create_line(sx0, sy0, sx1, sy1, width=2, fill=text_c, tags="static")
        self.canvas.create_text((sx0 + sx1) / 2, sy0 - 10, text="1 m", fill=text_c, font=("Helvetica Neue", 10), tags="static")

        self._static_bounds = (x0, x1, y0, y1)

    def redraw_scatter(self, pts_xy: List[Tuple[float, float]], clusters: List[List[Tuple[float, float]]], *, src: str, frame: str, stamp: str, n: int):
        x0, x1, y0, y1 = self._compute_bounds(pts_xy, clusters)
        if self._static_bounds != (x0, x1, y0, y1):
            self._draw_static_grid(x0, x1, y0, y1)

        self.canvas.delete("dyn")

        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        pad = 20
        pt_c = "#007AFF"  # iOS blue
        pal = ["#FF3B30", "#34C759", "#007AFF", "#FFCC00", "#AF52DE", "#30B0C7", "#FF9F0A", "#5856D6"]
        text_c = "#3A3A3C"

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2 * pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2 * pad) / (y1 - y0))
            return X, Y

        # Raw points
        if pts_xy:
            r = 2
            for x, y in pts_xy:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline=pt_c, tags="dyn")

        # Clusters (hulls + centroids)
        for ci, cluster in enumerate(clusters):
            col = pal[ci % len(pal)]
            hull = self._convex_hull(cluster)
            if len(hull) >= 3:
                coords = []
                for x, y in hull:
                    X, Y = to_pix(x, y); coords.extend([X, Y])
                self.canvas.create_polygon(*coords, outline=col, fill='', width=2, tags="dyn")
            arr = np.asarray(cluster)
            cx, cy = float(arr[:, 0].mean()), float(arr[:, 1].mean())
            r2 = 2
            for x, y in cluster:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X - r2, Y - r2, X + r2, Y + r2, outline=col, tags="dyn")
            Cx, Cy = to_pix(cx, cy)
            self.canvas.create_line(Cx - 5, Cy, Cx + 5, Cy, fill=col, tags="dyn")
            self.canvas.create_line(Cx, Cy - 5, Cx, Cy + 5, fill=col, tags="dyn")

        # Breadcrumb trail
        if len(self.node.breadcrumb) >= 2:
            coords = []
            for x, y in self.node.breadcrumb:
                X, Y = to_pix(x, y); coords.extend([X, Y])
            self.canvas.create_line(*coords, fill="#C7C7CC", width=1, tags="dyn")

        # Waypoints
        if self.node.waypoints_xy:
            for i, (wx, wy) in enumerate(self.node.waypoints_xy):
                X, Y = to_pix(wx, wy)
                r = 3
                col = "#FF9F0A" if i == self.node.next_wp_idx else "#8E8E93"
                self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline=col, width=2, tags="dyn")
            # Line to next waypoint
            if self.node.position_xy and 0 <= self.node.next_wp_idx < len(self.node.waypoints_xy):
                rx, ry = self.node.position_xy
                wx, wy = self.node.waypoints_xy[self.node.next_wp_idx]
                X1, Y1 = to_pix(rx, ry); X2, Y2 = to_pix(wx, wy)
                self.canvas.create_line(X1, Y1, X2, Y2, dash=(3, 3), fill="#FF9F0A", width=2, tags="dyn")

        # Tree detections
        for (tx, ty) in self.node.tree_positions_xy:
            X, Y = to_pix(tx, ty)
            r = 4
            self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline="#34C759", width=2, tags="dyn")

        # Robot marker (triangle)
        if self.node.position_xy is not None and self.node.yaw_rad is not None:
            rx, ry = self.node.position_xy
            yaw = self.node.yaw_rad
            tri = np.array([[0.35, 0.0], [-0.20, 0.12], [-0.20, -0.12]])
            R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            ptsR = (tri @ R.T) + np.array([rx, ry])
            coords = []
            for px, py in ptsR:
                X, Y = to_pix(px, py); coords.extend([X, Y])
            self.canvas.create_polygon(*coords, outline="#34C759", fill='', width=2, tags="dyn")

        # Info text
        info = f"src {src}  |  frame {frame}  |  t {stamp}  |  pts {n}"
        self.canvas.create_text(pad + 6, pad + 12, text=info, anchor='w', fill=text_c, font=("Helvetica Neue", 10), tags="dyn")

        # E-STOP overlay
        if self.node.estop_active():
            self.canvas.create_text(w/2, h/2, text="E-STOP ACTIVE", fill="#FF3B30", font=("Helvetica Neue", 28, "bold"), tags="dyn")

    # ---- Main loop ----
    def run(self):
        self.root.mainloop()


# ------------------------------ main ------------------------------
def ros_spin(node: GuiNode):
    rclpy.spin(node)

def main():
    rclpy.init()
    q_scan, q_img = queue.Queue(), queue.Queue()
    node = GuiNode(q_scan, q_img)
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()
    App(node, q_scan, q_img).run()

if __name__ == "__main__":
    main()
