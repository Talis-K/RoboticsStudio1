#!/usr/bin/env python3
"""
Tk GUI panel  v2 (aesthetic refresh)

Adds a modern dark theme, cleaner layout, nicer LiDAR visuals (grid, scale bar,
robot marker, convex hulls around clusters), and a small telemetry panel with an
altitude bar. Still lightweight (tkinter + Pillow + numpy) and ROS 2 friendly.

Features
- Live camera (autodiscovers raw/compressed)
- LiDAR points + optional Euclidean clustering (tune eps/minpts)
- Pretty topdown plot: grid, axes, adaptive scaling, 1 m scale bar
- Robot marker from /odometry (XY + yaw) and altitude (Z) readout
- Altitude progress bar with configurable max altitude
- ESTOP publish/clear

Params (set via launch or CLI)
  image_topic:  camera topic (auto if empty)
  scan_topic:   LaserScan topic (default /scan)
  cloud_topic:  optional PointCloud2 topic (drawn like scan)
  odom_topic:   nav_msgs/Odometry topic (default /odometry)
  estop_topic:  Bool topic for estop (default /e_stop)
  max_altitude: for the altitude progress bar (default 10.0 m)

Run
  colcon build --symlink-install && . install/setup.bash
  ros2 run <pkg> gui_panel.py --ros-args -p scan_topic:=/scan -p odom_topic:=/odometry
"""

import threading, queue, math, numpy as np, tkinter as tk, io, time
from tkinter import ttk
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

        # EStop
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

    # ---- ESTOP ----
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
            self.get_logger().info("[GUI] Estop reset")

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
            if msg.encoding not in SUPPORTED_RAW:
                self.get_logger().warn(f"raw image encoding not supported: {msg.encoding}")
                return
            w, h = msg.width, msg.height
            img = PILImage.frombytes('RGB' if 'rgb' in msg.encoding else 'L', (w, h), bytes(msg.data))
            if msg.encoding in ('bgr8','bgra8'):
                img = PILImage.fromarray(np.array(img)[:,:,::-1])
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


# ------------------------------ GUI ------------------------------
class App:
    def __init__(self, node: GuiNode, q_scan: queue.Queue, q_img: queue.Queue):
        self.node = node
        self.q_scan = q_scan
        self.q_img = q_img

        self.root = tk.Tk(); self.root.title("41068: Camera + LiDAR Panel"); self.root.geometry("1280x800")
        self.root.minsize(1100, 720)
        self._apply_theme()

        # Header bar
        header = ttk.Frame(self.root, padding=(16,12,16,8), style="Header.TFrame")
        header.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(header, text="Mission Console", style="Header.TLabel").pack(side=tk.LEFT)
        self.fps_lbl = ttk.Label(header, text="", style="HeaderSub.TLabel")
        self.fps_lbl.pack(side=tk.RIGHT)

        # Main content area (two columns)
        body = ttk.Frame(self.root, padding=(16,8,16,16))
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=3)
        body.columnconfigure(2, weight=0)
        body.rowconfigure(0, weight=1)

        # Camera card
        cam_card = ttk.LabelFrame(body, text="Camera", padding=10, style="Card.TLabelframe")
        cam_card.grid(row=0, column=0, sticky="nsew", padx=(0,8))
        self.cam_label = ttk.Label(cam_card, text="(waiting for image&)", anchor="center")
        self.cam_label.pack(fill="both", expand=True)

        # LiDAR card
        lidar_card = ttk.LabelFrame(body, text="LiDAR (topdown)", padding=10, style="Card.TLabelframe")
        lidar_card.grid(row=0, column=1, sticky="nsew")
        self.canvas = tk.Canvas(lidar_card, width=520, height=540, bg="#0e1116", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Controls row beneath canvas
        controls = ttk.Frame(lidar_card)
        controls.pack(fill="x", pady=(8,0))
        self.show_points_var   = tk.BooleanVar(value=True)
        self.show_clusters_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="Points", variable=self.show_points_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(controls, text="Clusters", variable=self.show_clusters_var).grid(row=0, column=1, sticky="w", padx=(12,0))
        ttk.Label(controls, text="eps (m):").grid(row=0, column=2, sticky="e", padx=(16,4))
        self.eps_var = tk.DoubleVar(value=0.30)
        ttk.Spinbox(controls, from_=0.05, to=2.0, increment=0.05, width=5, textvariable=self.eps_var).grid(row=0, column=3, sticky="w")
        ttk.Label(controls, text="min pts:").grid(row=0, column=4, sticky="e", padx=(12,4))
        self.minpts_var = tk.IntVar(value=4)
        ttk.Spinbox(controls, from_=1, to=40, increment=1, width=5, textvariable=self.minpts_var).grid(row=0, column=5, sticky="w")

        # Side panel (Safety + Telemetry)
        side = ttk.LabelFrame(body, text="Safety & Telemetry", padding=12, style="Card.TLabelframe")
        side.grid(row=0, column=2, sticky="ns", padx=(8,0))

        # ESTOP buttons
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
        self.status_lbl = ttk.Label(side, text="Status: NORMAL")
        self.status_lbl.pack(fill="x", pady=(10,14))

        # Telemetry readouts
        telem = ttk.Frame(side)
        telem.pack(fill="x")
        ttk.Label(telem, text="Altitude:").grid(row=0, column=0, sticky="e")
        self.alt_bar = ttk.Progressbar(telem, mode="determinate", length=120, maximum=self._max_altitude())
        self.alt_bar.grid(row=0, column=1, sticky="w", padx=(6,0))
        self.lbl_alt = ttk.Label(telem, text=" m"); self.lbl_alt.grid(row=0, column=2, sticky="w", padx=(6,0))

        ttk.Label(telem, text="Position:").grid(row=1, column=0, sticky="e", pady=(6,0))
        self.lbl_xy = ttk.Label(telem, text=""); self.lbl_xy.grid(row=1, column=1, columnspan=2, sticky="w", padx=(6,0), pady=(6,0))

        ttk.Label(telem, text="Yaw:").grid(row=2, column=0, sticky="e", pady=(6,0))
        self.lbl_yaw = ttk.Label(telem, text=""); self.lbl_yaw.grid(row=2, column=1, columnspan=2, sticky="w", padx=(6,0), pady=(6,0))

        # Footer
        footer = ttk.Frame(self.root, padding=(16,0,16,12))
        footer.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(footer, text="Quit", command=self.on_quit, style="Accent.TButton").pack(side=tk.RIGHT)
        ttk.Label(footer, text="Space = ESTOP, R = Reset", style="Footer.TLabel").pack(side=tk.LEFT)

        # Internal state
        self._last_photo = None
        self._cam_max_w = 900
        self._cam_max_h = 560
        self._last_draw_ts = time.time()

        # Pollers
        self.root.after(50, self.poll_img)
        self.root.after(80, self.poll_scan)
        self.root.after(150, self.poll_altitude)
        self.root.bind("<space>", lambda e: self.on_estop_press())
        self.root.bind("<r>",      lambda e: self.on_estop_reset())

    # ---- Theme ----
    def _apply_theme(self):
        style = ttk.Style()
        # Use a platformneutral theme
        try:
            style.theme_use('clam')
        except Exception:
            pass
        # Palette (Nordish)
        bg   = '#0b0e14'
        bg2  = '#11151c'
        card = '#0e1116'
        fg   = '#e6edf3'
        sub  = '#a6b3c2'
        acc  = '#3b82f6'  # Accent blue
        self.root.configure(bg=bg)
        style.configure('.', background=bg, foreground=fg, font=("Segoe UI", 10))
        style.configure('Header.TFrame', background=bg2)
        style.configure('Header.TLabel', background=bg2, foreground=fg, font=("Segoe UI", 16, 'bold'))
        style.configure('HeaderSub.TLabel', background=bg2, foreground=sub)
        style.configure('Footer.TLabel', background=bg, foreground=sub)
        style.configure('Card.TLabelframe', background=card, foreground=fg, relief='flat')
        style.configure('Card.TLabelframe.Label', background=card, foreground=sub, font=("Segoe UI", 10, 'bold'))
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
        self.status_lbl.config(text="Status: ESTOP ENGAGED")

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

    # ---- LiDAR + Clusters ----
    def poll_scan(self):
        item = None
        try:
            while True:
                item = self.q_scan.get_nowait()
        except queue.Empty:
            pass

        if item is not None:
            self._update_header_fps()
            pts = item.get('points_xy', [])
            clusters: List[List[Tuple[float,float]]] = []
            if self.show_clusters_var.get() and pts:
                clusters = self.euclidean_clusters(pts, eps=self.eps_var.get(), min_pts=self.minpts_var.get())
            self.redraw_scatter(pts if self.show_points_var.get() else [], clusters,
                                src=item.get('src',''), frame=item.get('frame',''), stamp=item.get('stamp',''), n=item.get('n_total',''))
        else:
            self.redraw_scatter([], [], src='', frame='', stamp='', n='')
        self.root.after(80, self.poll_scan)

    def poll_altitude(self):
        z = self.node.altitude_m
        if z is None:
            self.lbl_alt.config(text=" m")
            self.alt_bar['value'] = 0
        else:
            self.lbl_alt.config(text=f"{z:.2f} m")
            self.alt_bar['value'] = max(0.0, min(self._max_altitude(), z))
        xy = self.node.position_xy
        self.lbl_xy.config(text="oh" if xy is None else f"({xy[0]:.2f}, {xy[1]:.2f}) m")
        yaw = self.node.yaw_rad
        self.lbl_yaw.config(text="what" if yaw is None else f"{math.degrees(yaw):.1f}�")
        self.root.after(200, self.poll_altitude)

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
        # Round raw to 1, 2, 5 � 10^k
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

    def redraw_scatter(self, pts_xy: List[Tuple[float, float]], clusters: List[List[Tuple[float,float]]], *, src: str, frame: str, stamp: str, n: int):
        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        self.canvas.delete("all")
        pad = 20

        # Colors
        grid_c = "#1e2430"; axes_c = "#2e3440"; text_c = "#c7d0dd"; pt_c = "#89b4fa"
        pal = ["#f38ba8", "#a6e3a1", "#89b4fa", "#f9e2af", "#b4befe", "#94e2d5", "#fab387", "#cba6f7"]

        # Card background
        self.canvas.create_rectangle(0, 0, w, h, fill="#0e1116", outline="")

        # Decide bounds
        all_pts = pts_xy[:]
        for c in clusters:
            all_pts.extend(c)
        if all_pts and w >= 10 and h >= 10:
            arr = np.asarray(all_pts, dtype=float)
            xs, ys = arr[:,0], arr[:,1]
            x0, x1 = np.percentile(xs, [5, 95]); y0, y1 = np.percentile(ys, [5, 95])
            if x0 == x1: x0, x1 = x0-1.0, x1+1.0
            if y0 == y1: y0, y1 = y0-1.0, y1+1.0
        else:
            x0, x1, y0, y1 = -5, 5, -5, 5

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2*pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2*pad) / (y1 - y0))
            return X, Y

        # Grid (nice step)
        step = self._nice_step(max(x1-x0, y1-y0))
        # Vertical lines
        x = math.floor(x0 / step) * step
        while x <= x1:
            X, _ = to_pix(x, 0)
            self.canvas.create_line(X, pad, X, h-pad, fill=grid_c)
            x += step
        # Horizontal lines
        y = math.floor(y0 / step) * step
        while y <= y1:
            _, Y = to_pix(0, y)
            self.canvas.create_line(pad, Y, w-pad, Y, fill=grid_c)
            y += step

        # Axes
        ox, oy = to_pix(0.0, 0.0)
        self.canvas.create_line(ox, h-pad, ox, pad, fill=axes_c)
        self.canvas.create_line(pad, oy, w-pad, oy, fill=axes_c)

        # Scale bar (1 m)
        sx0, sy0 = to_pix(x0 + 0.8* (x1-x0) - 1.2, y0 + 0.1*(y1-y0))
        sx1, sy1 = to_pix(x0 + 0.8* (x1-x0) - 0.2, y0 + 0.1*(y1-y0))
        self.canvas.create_line(sx0, sy0, sx1, sy1, width=3, fill=text_c)
        self.canvas.create_text((sx0+sx1)/2, sy0-10, text="1 m", fill=text_c, font=("Segoe UI", 9))

        # Raw points
        if pts_xy:
            r = 1.8
            for x, y in pts_xy:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X-r, Y-r, X+r, Y+r, outline=pt_c)

        # Clusters
        for ci, cluster in enumerate(clusters):
            col = pal[ci % len(pal)]
            # Draw hull
            hull = self._convex_hull(cluster)
            if len(hull) >= 3:
                coords = []
                for x, y in hull:
                    X, Y = to_pix(x, y); coords.extend([X, Y])
                self.canvas.create_polygon(*coords, outline=col, fill='', width=2)
            # Points + centroid
            arr = np.asarray(cluster)
            cx, cy = float(arr[:,0].mean()), float(arr[:,1].mean())
            r2 = 2.2
            for x, y in cluster:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X-r2, Y-r2, X+r2, Y+r2, outline=col)
            Cx, Cy = to_pix(cx, cy)
            self.canvas.create_line(Cx-5, Cy, Cx+5, Cy, fill=col)
            self.canvas.create_line(Cx, Cy-5, Cx, Cy+5, fill=col)

        # Robot marker (triangle arrow at (x,y) with yaw)
        if self.node.position_xy is not None and self.node.yaw_rad is not None:
            rx, ry = self.node.position_xy
            yaw = self.node.yaw_rad
            # Triangle in robot frame
            tri = np.array([[0.35, 0.0], [-0.20, 0.12], [-0.20, -0.12]])
            R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            ptsR = (tri @ R.T) + np.array([rx, ry])
            coords = []
            for px, py in ptsR:
                X, Y = to_pix(px, py); coords.extend([X, Y])
            self.canvas.create_polygon(*coords, outline="#7dd3fc", fill='')

        # Info text (source/frame/stamp/points)
        info = f"src {src}  |  frame {frame}  |  t {stamp}  |  pts {n}"
        self.canvas.create_text(pad+6, pad+10, text=info, anchor='w', fill=text_c, font=("Segoe UI", 9))

        # ESTOP overlay
        if self.node.estop_active():
            self.canvas.create_text(w/2, h/2, text="ESTOP ACTIVE", fill="#ff5e5e", font=("Segoe UI", 28, "bold"))

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