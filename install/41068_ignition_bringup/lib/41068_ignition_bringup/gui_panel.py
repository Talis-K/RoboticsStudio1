#!/usr/bin/env python3
"""
Mission Console  Tk GUI (modern aesthetic + position upgrade)

Keeps:
- Camera auto-discovery (raw/compressed)
- LiDAR points + optional Euclidean clustering (eps/min_pts)
- Scale bar, grid, axes
- E-STOP publish/reset
- Altitude/XY/Yaw telemetry

Adds:
- Modern dark theme (cards, app bar, consistent fonts)
- Smoothed odom pose + fading breadcrumb trail
- Follow / Auto-fit view modes
- Zoom -, Zoom +, Re-center controls (+/-/c shortcuts)
- Optional speed (from Odometry.twist)
"""

import threading, queue, math, numpy as np, tkinter as tk, io, time
from tkinter import ttk
from collections import deque
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan, CompressedImage, PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2 as pc2

from PIL import Image as PILImage
from PIL import ImageTk

RAW_IMAGE_TYPE  = 'sensor_msgs/msg/Image'
COMP_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
SUPPORTED_RAW   = {'rgb8','bgr8','mono8','rgba8','bgra8'}

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

        self.msg_queue = msg_queue
        self.img_queue = img_queue

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriptions
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.get_logger().info(f"[GUI] scan: {scan_topic}")

        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos)
            self.get_logger().info(f"[GUI] cloud: {cloud_topic}")

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.get_logger().info(f"[GUI] odom: {odom_topic}")

        # E-STOP
        etopic = self.get_parameter('estop_topic').get_parameter_value().string_value
        self.estop_pub = self.create_publisher(Bool, etopic, 10)
        self._estop = False

        # Camera discovery
        self._img_sub = None
        self._cam_type: Optional[str] = None
        self._requested_image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._discovery_timer = self.create_timer(1.0, self._ensure_camera_subscription)
        self._ensure_camera_subscription(initial=True)

        # Telemetry state
        self.altitude_m: Optional[float] = None
        self.position_xy: Optional[Tuple[float,float]] = None
        self.yaw_rad: Optional[float] = None
        self.speed_mps: Optional[float] = None

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
        pts = []
        ang = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and math.isfinite(r):
                pts.append((r * math.cos(ang), r * math.sin(ang)))
            ang += msg.angle_increment
        try:
            self.msg_queue.put({
                'src': 'scan',
                'stamp': f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}",
                'frame': msg.header.frame_id,
                'n_total': len(pts),
                'points_xy': pts
            }, block=False)
        except queue.Full:
            pass

    def on_cloud(self, msg: PointCloud2):
        if self._estop:
            return
        pts = []
        for p in pc2.read_points(msg, field_names=('x','y'), skip_nans=True):
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
            # Decode to RGB robustly across common encodings
            w, h = msg.width, msg.height
            enc = msg.encoding.lower()
            if enc in ('rgb8', 'rgba8'):
                mode = 'RGB' if enc == 'rgb8' else 'RGBA'
                img = PILImage.frombytes(mode, (w, h), bytes(msg.data))
                if mode == 'RGBA':
                    img = img.convert('RGB')
            elif enc in ('bgr8', 'bgra8'):
                mode = 'RGB' if enc == 'bgr8' else 'RGBA'
                img = PILImage.frombytes(mode, (w, h), bytes(msg.data))
                img = PILImage.fromarray(np.array(img)[:, :, ::-1])  # BGR->RGB
                if mode == 'RGBA':
                    img = img.convert('RGB')
            elif enc == 'mono8':
                img = PILImage.frombytes('L', (w, h), bytes(msg.data)).convert('RGB')
            else:
                self.get_logger().warn(f"raw image encoding not supported: {msg.encoding}")
                return

            try:
                self.img_queue.put(img, block=False)
            except queue.Full:
                pass
        except Exception as e:
            self.get_logger().error(f"[GUI] raw image decode failed: {e}")

    def on_image_compressed(self, msg: CompressedImage):
        if self._estop:
            return
        try:
            bio = io.BytesIO(bytes(msg.data))
            img = PILImage.open(bio).convert('RGB')
            try:
                self.img_queue.put(img, block=False)
            except queue.Full:
                pass
        except Exception as e:
            self.get_logger().error(f"[GUI] compressed image decode failed: {e}")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.altitude_m = float(p.z)
        self.position_xy = (float(p.x), float(p.y))
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        # speed (optional)
        try:
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            vz = float(msg.twist.twist.linear.z)
            self.speed_mps = math.sqrt(vx*vx + vy*vy + vz*vz)
        except Exception:
            self.speed_mps = None


# ------------------------------ GUI ------------------------------
class App:
    def __init__(self, node: GuiNode, q_scan: queue.Queue, q_img: queue.Queue):
        self.node = node
        self.q_scan = q_scan
        self.q_img = q_img

        self.root = tk.Tk()
        self.root.title("Mission Console")
        self.root.geometry("1320x820")
        self.root.minsize(1120, 720)
        self._apply_theme()

        # Header bar (app bar)
        header = ttk.Frame(self.root, padding=(16,12,16,8), style="Header.TFrame")
        header.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(header, text="Mission Console", style="Header.TTitle").pack(side=tk.LEFT)
        self.fps_lbl = ttk.Label(header, text="", style="Header.TMeta")
        self.fps_lbl.pack(side=tk.RIGHT)

        # Main content area (two columns + rail)
        body = ttk.Frame(self.root, padding=(16,8,16,16))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=3)
        body.columnconfigure(2, weight=0)
        body.rowconfigure(0, weight=1)

        # Camera card
        cam_card = ttk.LabelFrame(body, text="Camera", padding=10, style="Card.TLabelframe")
        cam_card.grid(row=0, column=0, sticky="nsew", padx=(0,10))
        self.cam_label = ttk.Label(cam_card, text="(waiting for image&)", anchor="center", style="CardBody.TLabel")
        self.cam_label.pack(fill="both", expand=True)

        # LiDAR + Position card
        lidar_card = ttk.LabelFrame(body, text="LiDAR (top-down) + Position", padding=10, style="Card.TLabelframe")
        lidar_card.grid(row=0, column=1, sticky="nsew")
        self.canvas = tk.Canvas(lidar_card, width=540, height=560, bg="#0e1116", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Controls row beneath canvas
        controls = ttk.Frame(lidar_card, padding=(8,4,8,0), style="Subtle.TFrame")
        controls.pack(fill="x")
        self.show_points_var   = tk.BooleanVar(value=True)
        self.show_clusters_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="Points", variable=self.show_points_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(controls, text="Clusters", variable=self.show_clusters_var).grid(row=0, column=1, sticky="w", padx=(10,0))
        ttk.Label(controls, text="eps (m):", style="Meta.TLabel").grid(row=0, column=2, sticky="e", padx=(16,4))
        self.eps_var = tk.DoubleVar(value=0.30)
        ttk.Spinbox(controls, from_=0.05, to=2.0, increment=0.05, width=6, textvariable=self.eps_var).grid(row=0, column=3, sticky="w")
        ttk.Label(controls, text="min pts:", style="Meta.TLabel").grid(row=0, column=4, sticky="e", padx=(12,4))
        self.minpts_var = tk.IntVar(value=4)
        ttk.Spinbox(controls, from_=1, to=40, increment=1, width=6, textvariable=self.minpts_var).grid(row=0, column=5, sticky="w")

        # View controls
        self.view_mode = tk.StringVar(value="follow")   # "follow" or "autofit"
        ttk.Radiobutton(controls, text="Follow", value="follow", variable=self.view_mode, command=self._redraw_now).grid(row=0, column=6, padx=(16,0))
        ttk.Radiobutton(controls, text="Auto-fit", value="autofit", variable=self.view_mode, command=self._redraw_now).grid(row=0, column=7, padx=(6,0))
        ttk.Button(controls, text="Zoom ", style="Accent.TButton", command=lambda: self._zoom(-1)).grid(row=0, column=8, padx=(16,4))
        ttk.Button(controls, text="Zoom +", style="Accent.TButton", command=lambda: self._zoom(+1)).grid(row=0, column=9, padx=(4,8))
        ttk.Button(controls, text="Re-center", command=self._recenter).grid(row=0, column=10)

        # Side panel (Safety + Telemetry)
        side = ttk.LabelFrame(body, text="Safety & Telemetry", padding=12, style="Card.TLabelframe")
        side.grid(row=0, column=2, sticky="ns", padx=(10,0))

        # E-STOP buttons
        self.estop_btn = tk.Button(side, text="EMERGENCY\nSTOP", font=("Segoe UI", 18, "bold"),
                                   width=12, height=3, bg="#d22d2d", fg="white", bd=0,
                                   activebackground="#b71f1f", activeforeground="white",
                                   cursor="hand2", command=self.on_estop_press)
        self.estop_btn.pack(fill="x", pady=(0,8))
        self.reset_btn = tk.Button(side, text="Reset", font=("Segoe UI", 12, "bold"),
                                   bg="#2e3440", fg="#e5e9f0", bd=0, cursor="hand2",
                                   activebackground="#4c566a", activeforeground="#eceff4",
                                   command=self.on_estop_reset)
        self.reset_btn.pack(fill="x")
        self.status_lbl = ttk.Label(side, text="Status: NORMAL", style="Meta.TLabel")
        self.status_lbl.pack(fill="x", pady=(10,14))

        # Telemetry readouts
        telem = ttk.Frame(side, style="Subtle.TFrame")
        telem.pack(fill="x")
        ttk.Label(telem, text="Altitude", style="Meta.TLabel").grid(row=0, column=0, sticky="w")
        self.alt_bar = ttk.Progressbar(telem, mode="determinate", length=140, maximum=self._max_altitude())
        self.alt_bar.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2,0))
        self.lbl_alt = ttk.Label(telem, text=" m"); self.lbl_alt.grid(row=1, column=2, sticky="w", padx=(8,0))

        ttk.Label(telem, text="Position", style="Meta.TLabel").grid(row=2, column=0, sticky="w", pady=(8,0))
        self.lbl_xy = ttk.Label(telem, text=""); self.lbl_xy.grid(row=2, column=1, columnspan=2, sticky="e", pady=(8,0))

        ttk.Label(telem, text="Yaw", style="Meta.TLabel").grid(row=3, column=0, sticky="w", pady=(6,0))
        self.lbl_yaw = ttk.Label(telem, text=""); self.lbl_yaw.grid(row=3, column=1, columnspan=2, sticky="e", pady=(6,0))

        ttk.Label(telem, text="Speed", style="Meta.TLabel").grid(row=4, column=0, sticky="w", pady=(6,0))
        self.lbl_spd = ttk.Label(telem, text=""); self.lbl_spd.grid(row=4, column=1, columnspan=2, sticky="e", pady=(6,0))

        # Footer
        footer = ttk.Frame(self.root, padding=(16,0,16,12), style="Footer.TFrame")
        footer.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Label(footer, text="Space = E-STOP, R = Reset, +/- = Zoom, C = Re-center", style="Footer.THint").pack(side=tk.LEFT)
        ttk.Button(footer, text="Quit", command=self.on_quit, style="Accent.TButton").pack(side=tk.RIGHT)

        # Internal state
        self._last_photo = None
        self._cam_max_w = 960
        self._cam_max_h = 600
        self._last_draw_ts = time.time()

        # Pose smoothing + trail + view
        self._trail = deque(maxlen=700)      # (x,y)
        self._smooth_xy: Optional[Tuple[float,float]] = None
        self._smooth_yaw: Optional[float] = None
        self._alpha = 0.25                   # LPF
        self._follow_span_m = 12.0           # half-span
        self._zoom_scale = 1.0               # affects follow span

        # Pollers
        self.root.after(50, self.poll_img)
        self.root.after(80, self.poll_scan)
        self.root.after(150, self.poll_altitude)

        # Keys
        self.root.bind("<space>", lambda e: self.on_estop_press())
        self.root.bind("<r>",      lambda e: self.on_estop_reset())
        self.root.bind("+",        lambda e: self._zoom(+1))
        self.root.bind("=",        lambda e: self._zoom(+1))
        self.root.bind("-",        lambda e: self._zoom(-1))
        self.root.bind("c",        lambda e: self._recenter())

    # ---- Theme ----
    def _apply_theme(self):
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception:
            pass
        # Palette
        BG   = '#0b0e14'
        SURF = '#11151c'
        CARD = '#0e1116'
        TEXT = '#e6edf3'
        SUB  = '#a6b3c2'
        ACC  = '#3b82f6'
        MUT  = '#5b677a'

        self.root.configure(bg=BG)
        style.configure('.', background=BG, foreground=TEXT, font=("Segoe UI", 10))
        # App bar
        style.configure('Header.TFrame', background=SURF)
        style.configure('Header.TTitle', background=SURF, foreground=TEXT, font=("Segoe UI", 16, 'bold'))
        style.configure('Header.TMeta',  background=SURF, foreground=SUB, font=("Segoe UI", 10))
        # Cards
        style.configure('Card.TLabelframe', background=CARD, foreground=TEXT, relief='flat')
        style.configure('Card.TLabelframe.Label', background=CARD, foreground=SUB, font=("Segoe UI", 10, 'bold'))
        style.configure('CardBody.TLabel', background=CARD, foreground=TEXT)
        style.configure('Subtle.TFrame', background=CARD)
        # Footer
        style.configure('Footer.TFrame', background=BG)
        style.configure('Footer.THint', background=BG, foreground=MUT)
        # Buttons
        style.configure('Accent.TButton', padding=6)
        style.map('Accent.TButton',
                  background=[('!disabled', ACC), ('active', '#60a5fa')],
                  foreground=[('!disabled', BG), ('active', BG)])
        # Meta text
        style.configure('Meta.TLabel', background=CARD, foreground=SUB, font=("Segoe UI", 9))

        # Store for canvas rendering
        self._theme = dict(grid='#1e2430', axes='#2e3440', text=TEXT, sub=SUB, card=CARD)

    def _max_altitude(self) -> float:
        try:
            return float(self.node.get_parameter('max_altitude').get_parameter_value().double_value)
        except Exception:
            return 10.0

    # ---- Safety ----
    def on_estop_press(self):
        self.node.engage_estop()
        self.status_lbl.config(text="Status: E-STOP ENGAGED")

    def on_estop_reset(self):
        self.node.reset_estop()
        self.status_lbl.config(text="Status: NORMAL")

    def on_quit(self):
        try:
            self.node.destroy_node(); rclpy.shutdown()
        finally:
            self.root.destroy()

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

    # ---- LiDAR + Position ----
    def poll_scan(self):
        item = None
        try:
            while True:
                item = self.q_scan.get_nowait()
        except queue.Empty:
            pass

        # Update pose smoothing + trail
        self._update_pose_cache()

        if item is not None:
            self._update_header_fps()
            pts = item.get('points_xy', [])
            clusters: List[List[Tuple[float,float]]] = []
            if self.show_clusters_var.get() and pts:
                clusters = self.euclidean_clusters(pts, eps=self.eps_var.get(), min_pts=self.minpts_var.get())
            self.redraw_scatter(
                pts if self.show_points_var.get() else [],
                clusters,
                src=item.get('src',''), frame=item.get('frame',''),
                stamp=item.get('stamp',''), n=item.get('n_total','')
            )
        else:
            self.redraw_scatter([], [], src='', frame='', stamp='', n='')
        self.root.after(80, self.poll_scan)

    def poll_altitude(self):
        z = self.node.altitude_m
        if z is None:
            self.lbl_alt.config(text=" m")
            self.alt_bar['value'] = 0
        else:
            self.lbl_alt.config(text=f"{z:.2f} m")
            self.alt_bar['value'] = max(0.0, min(self._max_altitude(), z))
        xy = self.node.position_xy
        self.lbl_xy.config(text="" if xy is None else f"({xy[0]:.2f}, {xy[1]:.2f}) m")
        yaw = self.node.yaw_rad
        self.lbl_yaw.config(text="" if yaw is None else f"{math.degrees(yaw):.1f}�")
        spd = self.node.speed_mps
        self.lbl_spd.config(text="" if spd is None else f"{spd:.2f} m/s")
        self.root.after(200, self.poll_altitude)

    # ---- Pose smoothing + trail ----
    def _update_pose_cache(self):
        xy = self.node.position_xy
        yaw = self.node.yaw_rad
        if xy is None or yaw is None:
            return
        if self._smooth_xy is None:
            self._smooth_xy = (xy[0], xy[1])
            self._smooth_yaw = yaw
        else:
            ax = self._alpha
            sx, sy = self._smooth_xy
            sx = ax*xy[0] + (1-ax)*sx
            sy = ax*xy[1] + (1-ax)*sy
            dy = (yaw - self._smooth_yaw + math.pi) % (2*math.pi) - math.pi
            self._smooth_yaw = (self._smooth_yaw + ax*dy) % (2*math.pi)
            self._smooth_xy = (sx, sy)
        self._trail.append(self._smooth_xy)

    # ---- Utils: clustering & drawing ----
    @staticmethod
    def euclidean_clusters(pts_xy: List[Tuple[float,float]], eps: float = 0.3, min_pts: int = 4) -> List[List[Tuple[float,float]]]:
        if not pts_xy:
            return []
        pts = np.asarray(pts_xy, dtype=float)
        N = len(pts)
        visited = np.zeros(N, dtype=bool)
        clusters: List[List[Tuple[float,float]]] = []
        eps2 = eps*eps
        for i in range(N):
            if visited[i]:
                continue
            queue_idx = [i]
            visited[i] = True
            cluster_idx = [i]
            while queue_idx:
                j = queue_idx.pop()
                diff = pts - pts[j]
                dist2 = (diff[:,0]**2 + diff[:,1]**2)
                nbrs = np.where((dist2 <= eps2) & (~visited))[0]
                for k in nbrs.tolist():
                    visited[k] = True
                    queue_idx.append(k)
                    cluster_idx.append(k)
            if len(cluster_idx) >= min_pts:
                clusters.append([tuple(pts[k]) for k in cluster_idx])
        return clusters

    @staticmethod
    def _convex_hull(points: List[Tuple[float,float]]) -> List[Tuple[float,float]]:
        """Monotone chain convex hull (O(n log n))."""
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
        """Choose a pleasing grid step for the given span (in meters)."""
        if span <= 0:
            return 1.0
        raw = span / 8.0
        mag = 10 ** math.floor(math.log10(raw))
        for m in (1, 2, 5, 10):
            step = m * mag
            if raw <= step:
                return step
        return 10 * mag

    def _update_header_fps(self):
        now = time.time()
        dt = now - self._last_draw_ts
        self._last_draw_ts = now
        if dt > 0:
            self.fps_lbl.config(text=f"{1.0/dt:5.1f} FPS")

    # --- View logic: follow vs auto-fit ---
    def _view_bounds(self, pts_xy: List[Tuple[float,float]]) -> Tuple[float,float,float,float]:
        # Follow: fixed window around smoothed pose (zoom affects span)
        if self.view_mode.get() == "follow" and self._smooth_xy is not None:
            span = self._follow_span_m / max(0.25, self._zoom_scale)
            cx, cy = self._smooth_xy
            return cx - span, cx + span, cy - span, cy + span

        # Auto-fit: include points + trail + robot
        all_pts = []
        all_pts.extend(pts_xy)
        all_pts.extend(list(self._trail))
        if self._smooth_xy is not None:
            all_pts.append(self._smooth_xy)
        if not all_pts:
            return -5, 5, -5, 5
        arr = np.asarray(all_pts, dtype=float)
        xs, ys = arr[:,0], arr[:,1]
        x0, x1 = np.percentile(xs, [5, 95]); y0, y1 = np.percentile(ys, [5, 95])
        if x0 == x1: x0, x1 = x0-1.0, x1+1.0
        if y0 == y1: y0, y1 = y0-1.0, y1+1.0
        pad = 0.08
        dx, dy = (x1-x0), (y1-y0)
        return x0 - pad*dx, x1 + pad*dx, y0 - pad*dy, y1 + pad*dy

    def _zoom(self, direction: int):
        self._zoom_scale = float(np.clip(self._zoom_scale * (1.15 if direction > 0 else 1/1.15), 0.25, 4.0))
        self._redraw_now()

    def _recenter(self):
        self.view_mode.set("follow")
        self._zoom_scale = 1.0
        self._redraw_now()

    def _redraw_now(self):
        self.redraw_scatter([], [], src='', frame='', stamp='', n='')

    def _lerp_color(self, c1: str, c2: str, t: float) -> str:
        t = float(np.clip(t, 0.0, 1.0))
        def hex_to_rgb(c): c = c.lstrip('#'); return tuple(int(c[i:i+2], 16) for i in (0,2,4))
        def rgb_to_hex(r,g,b): return f"#{r:02x}{g:02x}{b:02x}"
        r1,g1,b1 = hex_to_rgb(c1); r2,g2,b2 = hex_to_rgb(c2)
        r = int(r1 + (r2 - r1)*t); g = int(g1 + (g2 - g1)*t); b = int(b1 + (b2 - b1)*t)
        return rgb_to_hex(r,g,b)

    def redraw_scatter(self, pts_xy: List[Tuple[float, float]], clusters: List[List[Tuple[float,float]]], *, src: str, frame: str, stamp: str, n: int):
        theme = self._theme
        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        self.canvas.delete("all")
        pad = 20

        # Card background
        self.canvas.create_rectangle(0, 0, w, h, fill=theme['card'], outline="")

        # Bounds
        x0, x1, y0, y1 = self._view_bounds(pts_xy)

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2*pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2*pad) / (y1 - y0))
            return X, Y

        # Grid
        step = self._nice_step(max(x1-x0, y1-y0))
        x = math.floor(x0 / step) * step
        while x <= x1:
            X, _ = to_pix(x, 0)
            self.canvas.create_line(X, pad, X, h-pad, fill=theme['grid'])
            x += step
        y = math.floor(y0 / step) * step
        while y <= y1:
            _, Y = to_pix(0, y)
            self.canvas.create_line(pad, Y, w-pad, Y, fill=theme['grid'])
            y += step

        # Axes
        ox, oy = to_pix(0.0, 0.0)
        self.canvas.create_line(ox, h-pad, ox, pad, fill=theme['axes'])
        self.canvas.create_line(pad, oy, w-pad, oy, fill=theme['axes'])

        # Scale bar (1 m)
        sx0, sy0 = to_pix(x0 + 0.8*(x1-x0) - 1.2, y0 + 0.1*(y1-y0))
        sx1, sy1 = to_pix(x0 + 0.8*(x1-x0) - 0.2, y0 + 0.1*(y1-y0))
        self.canvas.create_line(sx0, sy0, sx1, sy1, width=3, fill=theme['sub'])
        self.canvas.create_text((sx0+sx1)/2, sy0-10, text="1 m", fill=theme['sub'], font=("Segoe UI", 9))

        # Raw points
        if pts_xy:
            r = 1.8
            for x, y in pts_xy:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X-r, Y-r, X+r, Y+r, outline="#89b4fa")

        # Clusters
        pal = ["#f38ba8", "#a6e3a1", "#89b4fa", "#f9e2af", "#b4befe", "#94e2d5", "#fab387", "#cba6f7"]
        for ci, cluster in enumerate(clusters):
            col = pal[ci % len(pal)]
            hull = self._convex_hull(cluster)
            if len(hull) >= 3:
                coords = []
                for x, y in hull:
                    X, Y = to_pix(x, y); coords.extend([X, Y])
                self.canvas.create_polygon(*coords, outline=col, fill='', width=2)
            arr = np.asarray(cluster)
            cx, cy = float(arr[:,0].mean()), float(arr[:,1].mean())
            r2 = 2.2
            for x, y in cluster:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X-r2, Y-r2, X+r2, Y+r2, outline=col)
            Cx, Cy = to_pix(cx, cy)
            self.canvas.create_line(Cx-5, Cy, Cx+5, Cy, fill=col)
            self.canvas.create_line(Cx, Cy-5, Cx, Cy+5, fill=col)

        # Trail (fade older segments)
        if len(self._trail) >= 2:
            pts = list(self._trail)
            L = len(pts) - 1
            for i in range(L):
                (xA, yA), (xB, yB) = pts[i], pts[i+1]
                XA, YA = to_pix(xA, yA)
                XB, YB = to_pix(xB, yB)
                t = i / L
                k = 0.25 + 0.75*(1 - t)**2
                col = self._lerp_color("#1F7AA8", "#7DD3FC", k)
                width = 1 + int(2 * (1 - t))
                self.canvas.create_line(XA, YA, XB, YB, fill=col, width=width)

        # Robot marker (triangle + label)
        if self._smooth_xy is not None and self._smooth_yaw is not None:
            rx, ry = self._smooth_xy
            yaw = self._smooth_yaw
            tri = np.array([[0.35, 0.0], [-0.20, 0.12], [-0.20, -0.12]])
            R = np.array([[math.cos(yaw), -math.sin(yaw)],
                          [math.sin(yaw),  math.cos(yaw)]])
            ptsR = (tri @ R.T) + np.array([rx, ry])
            coords = []
            for px, py in ptsR:
                X, Y = to_pix(px, py); coords.extend([X, Y])
            self.canvas.create_polygon(*coords, outline="#7dd3fc", fill='')

            txt = f"({rx:.2f}, {ry:.2f})  �={math.degrees(yaw):.1f}�"
            Xr, Yr = to_pix(rx, ry)
            self.canvas.create_text(Xr + 16, Yr - 14, text=txt, anchor='w', fill=theme['text'], font=("Segoe UI", 9))

        # Info text
        info = f"{src} | frame {frame} | t {stamp} | pts {n}"
        self.canvas.create_text(pad+6, pad+10, text=info, anchor='w', fill=theme['sub'], font=("Segoe UI", 9))

        # E-STOP overlay
        if self.node.estop_active():
            self.canvas.create_text(w/2, h/2, text="E-STOP ACTIVE", fill="#ff5e5e", font=("Segoe UI", 28, "bold"))

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
