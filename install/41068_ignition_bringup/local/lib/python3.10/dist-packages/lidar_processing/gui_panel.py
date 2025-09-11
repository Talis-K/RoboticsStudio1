#!/usr/bin/env python3
import threading, queue, math, numpy as np, tkinter as tk, io
from tkinter import ttk
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan, CompressedImage, PointCloud2
from std_msgs.msg import Bool

# PointCloud2 decoder (ROS 2 Humble)
from sensor_msgs_py import point_cloud2 as pc2

# GUI imaging (no OpenCV)
from PIL import Image as PILImage
from PIL import ImageTk

RAW_IMAGE_TYPE  = 'sensor_msgs/msg/Image'
COMP_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
SUPPORTED_RAW   = {'rgb8','bgr8','mono8','rgba8','bgra8'}


class GuiNode(Node):
    """
    Subscribes to LaserScan (/scan) and optional PointCloud2 (cloud_topic),
    and to a camera topic (raw or compressed). Auto-discovers camera if not set.
    Also publishes /e_stop Bool with latched state.
    """
    def __init__(self, msg_queue: queue.Queue, img_queue: queue.Queue):
        super().__init__('gui_panel_node')

        # ---- Parameters (override via launch or CLI) ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '')           # optional PointCloud2
        self.declare_parameter('image_topic', '')           # empty = auto-discover
        self.declare_parameter('estop_topic', '/e_stop')

        self.msg_queue = msg_queue
        self.img_queue = img_queue

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # LiDAR LaserScan
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos)
        self.get_logger().info(f"[GUI] Listening to scan: {scan_topic}")

        # Optional PointCloud2
        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos)
            self.get_logger().info(f"[GUI] Listening to cloud: {cloud_topic}")

        # E-Stop publisher
        etopic = self.get_parameter('estop_topic').get_parameter_value().string_value
        self.estop_pub = self.create_publisher(Bool, etopic, 10)
        self._estop = False

        # Camera discovery / subscribe
        self._img_sub = None
        self._cam_type: Optional[str] = None  # 'raw' or 'compressed'
        self._requested_image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self._discovery_timer = self.create_timer(1.0, self._ensure_camera_subscription)
        self._ensure_camera_subscription(initial=True)

    # ---------- E-Stop ----------
    def engage_estop(self):
        if not self._estop:
            self._estop = True
            try: self.estop_pub.publish(Bool(data=True))
            except Exception: pass
            self.get_logger().warn("[GUI] EMERGENCY STOP ENGAGED")

    def reset_estop(self):
        if self._estop:
            self._estop = False
            try: self.estop_pub.publish(Bool(data=False))
            except Exception: pass
            self.get_logger().info("[GUI] E-Stop reset")

    def estop_active(self) -> bool:
        return self._estop

    # ---------- Camera discovery ----------
    def _ensure_camera_subscription(self, initial: bool = False):
        if self._img_sub is not None:
            if self._discovery_timer:
                self._discovery_timer.cancel()
                self._discovery_timer = None
            return

        cam_topic, cam_type = self._resolve_camera_topic(self._requested_image_topic)
        if cam_topic is None:
            if initial:
                self.get_logger().warn("[GUI] No camera topic yet; will retry…")
            return

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        if cam_type == 'raw':
            self._img_sub = self.create_subscription(Image, cam_topic, self.on_image_raw, qos)
        else:
            self._img_sub = self.create_subscription(CompressedImage, cam_topic, self.on_image_compressed, qos)

        self._cam_type = cam_type
        self.get_logger().info(f"[GUI] Subscribed to camera ({cam_type}): {cam_topic}")

        if self._discovery_timer:
            self._discovery_timer.cancel()
            self._discovery_timer = None

    def _resolve_camera_topic(self, requested: str) -> Tuple[Optional[str], Optional[str]]:
        topics_and_types = self.get_topic_names_and_types()

        def type_of(name: str) -> Optional[str]:
            for n, types in topics_and_types:
                if n == name:
                    if RAW_IMAGE_TYPE in types:  return 'raw'
                    if COMP_IMAGE_TYPE in types: return 'compressed'
            return None

        if requested:
            t = type_of(requested)
            if t: return requested, t
            alt = requested if requested.endswith('/compressed') else requested + '/compressed'
            t2 = type_of(alt)
            if t2: return alt, t2
            return None, None

        # Prefer '/camera/image' if available
        for n, ts in topics_and_types:
            if RAW_IMAGE_TYPE in ts and n.endswith('/camera/image'):
                return n, 'raw'
        # Any Image then any CompressedImage
        for n, ts in topics_and_types:
            if RAW_IMAGE_TYPE in ts: return n, 'raw'
        for n, ts in topics_and_types:
            if COMP_IMAGE_TYPE in ts: return n, 'compressed'
        return None, None

    # ---------- Callbacks ----------
    def on_scan(self, msg: LaserScan):
        if self._estop: return
        pts = []
        ang = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
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
        if self._estop: return
        try:
            # Read XYZ; keep XY for 2D scatter; sample to keep UI snappy
            max_points = 8000
            count = 0
            pts_xy = []
            for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
                pts_xy.append((float(p[0]), float(p[1])))
                count += 1
                if count >= max_points:
                    break

            self.msg_queue.put({
                'src': 'cloud',
                'stamp': f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}",
                'frame': msg.header.frame_id,
                'n_total': count,
                'points_xy': pts_xy
            }, block=False)
        except Exception as e:
            self.get_logger().warn(f"[GUI] cloud decode failed: {e}")

    def on_image_raw(self, msg: Image):
        if self._estop: return
        try:
            enc = msg.encoding.lower()
            w, h, step = msg.width, msg.height, msg.step
            buf = memoryview(msg.data)

            if enc in ('rgb8','bgr8'):
                arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, step)[:, :3*w].reshape(h, w, 3)
                if enc == 'bgr8': arr = arr[:, :, ::-1]
                img = PILImage.fromarray(arr, 'RGB')
            elif enc in ('rgba8','bgra8'):
                arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, step)[:, :4*w].reshape(h, w, 4)
                if enc == 'bgra8': arr = arr[:, :, [2,1,0,3]]
                img = PILImage.fromarray(arr, 'RGBA').convert('RGB')
            elif enc == 'mono8':
                arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, step)[:, :w].reshape(h, w)
                img = PILImage.fromarray(arr, 'L').convert('RGB')
            else:
                self.get_logger().warn(f"[GUI] Unsupported encoding '{msg.encoding}', assuming RGB")
                arr = np.frombuffer(buf, dtype=np.uint8)[:h*w*3].reshape(h, w, 3)
                img = PILImage.fromarray(arr, 'RGB')

            try: self.img_queue.put(img, block=False)
            except queue.Full: pass

        except Exception as e:
            self.get_logger().error(f"[GUI] raw image decode failed: {e}")

    def on_image_compressed(self, msg: CompressedImage):
        if self._estop: return
        try:
            bio = io.BytesIO(bytes(msg.data))
            img = PILImage.open(bio).convert('RGB')
            try: self.img_queue.put(img, block=False)
            except queue.Full: pass
        except Exception as e:
            self.get_logger().error(f"[GUI] compressed image decode failed: {e}")


# --------------------------- Tk GUI -----------------------------------
class App:
    def __init__(self, node: GuiNode, q_scan: queue.Queue, q_img: queue.Queue):
        self.node = node; self.q_scan = q_scan; self.q_img = q_img
        self.root = tk.Tk(); self.root.title("41068: Camera + LiDAR Panel"); self.root.geometry("1180x720")

        main = ttk.Frame(self.root, padding=10); main.grid(sticky="nsew")
        for i in range(3): main.columnconfigure(i, weight=1 if i < 2 else 0)
        main.rowconfigure(1, weight=1)

        ttk.Label(main, text="Live Camera & LiDAR", font=("Segoe UI", 16, "bold")).grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0,10)
        )

        # Camera
        cam_frame = ttk.LabelFrame(main, text="Camera", padding=8); cam_frame.grid(row=1, column=0, sticky="nsew", padx=(0,8))
        self.cam_label = ttk.Label(cam_frame, text="(waiting for image…)"); self.cam_label.pack(fill="both", expand=True)

        # LiDAR
        lidar_frame = ttk.LabelFrame(main, text="LiDAR (top-down X-Y)", padding=8); lidar_frame.grid(row=1, column=1, sticky="nsew")
        self.canvas = tk.Canvas(lidar_frame, width=460, height=360, bg="#111111", highlightthickness=0); self.canvas.pack(fill="both", expand=True)

        info = ttk.Frame(lidar_frame); info.pack(fill="x", pady=(8,0))
        ttk.Label(info, text="Source:").grid(row=0, column=0, sticky="e"); self.val_src = ttk.Label(info, text="–"); self.val_src.grid(row=0, column=1, sticky="w", padx=(6,20))
        ttk.Label(info, text="Frame:").grid(row=0, column=2, sticky="e"); self.val_frame = ttk.Label(info, text="–"); self.val_frame.grid(row=0, column=3, sticky="w", padx=(6,20))
        ttk.Label(info, text="Stamp:").grid(row=0, column=4, sticky="e"); self.val_stamp = ttk.Label(info, text="–"); self.val_stamp.grid(row=0, column=5, sticky="w", padx=(6,20))
        ttk.Label(info, text="Points:").grid(row=0, column=6, sticky="e"); self.val_npts = ttk.Label(info, text="–"); self.val_npts.grid(row=0, column=7, sticky="w")

        # Safety
        estop_frame = ttk.LabelFrame(main, text="Safety", padding=12)
        estop_frame.grid(row=1, column=2, sticky="ns", padx=(8,0))
        self.estop_btn = tk.Button(estop_frame, text="EMERGENCY\nSTOP", font=("Segoe UI", 18, "bold"),
                                   width=10, height=4, bg="#cc0000", fg="white", activebackground="#990000",
                                   command=self.on_estop_press)
        self.estop_btn.pack(fill="x", pady=(0,8))
        self.reset_btn = tk.Button(estop_frame, text="Reset", font=("Segoe UI", 12, "bold"),
                                   bg="#dddddd", command=self.on_estop_reset)
        self.reset_btn.pack(fill="x")
        self.status_lbl = ttk.Label(estop_frame, text="Status: NORMAL"); self.status_lbl.pack(fill="x", pady=(8,0))

        # Footer
        btn_row = ttk.Frame(main); btn_row.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10,0))
        ttk.Button(btn_row, text="Quit", command=self.on_quit).pack()

        self._last_photo = None
        self._cam_max_w = 900
        self._cam_max_h = 520

        # Pollers
        self.root.after(50, self.poll_img)
        self.root.after(80, self.poll_scan)

        # Shortcuts
        self.root.bind("<space>", lambda e: self.on_estop_press())
        self.root.bind("<r>", lambda e: self.on_estop_reset())

    # Safety handlers
    def on_estop_press(self):
        self.node.engage_estop()
        self.status_lbl.config(text="Status: ESTOP ACTIVE", foreground="#cc0000")

    def on_estop_reset(self):
        self.node.reset_estop()
        self.status_lbl.config(text="Status: NORMAL", foreground="")

    def on_quit(self):
        try:
            self.node.destroy_node(); rclpy.shutdown()
        finally:
            self.root.destroy()

    # Camera updates
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

    # LiDAR updates (from scan or cloud)
    def poll_scan(self):
        item = None
        try:
            while True:
                item = self.q_scan.get_nowait()
        except queue.Empty:
            pass

        if item is not None:
            self.val_src.config(text=item.get('src', '–'))
            self.val_frame.config(text=item.get('frame', '–'))
            self.val_stamp.config(text=item.get('stamp', '–'))
            self.val_npts.config(text=str(item.get('n_total', '–')))
            self.redraw_scatter(item.get('points_xy', []))
        else:
            self.redraw_scatter([])
        self.root.after(80, self.poll_scan)

    def redraw_scatter(self, pts_xy: List[Tuple[float, float]]):
        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        self.canvas.delete("all")
        pad = 20
        # Frame box
        self.canvas.create_rectangle(pad, pad, w-pad, h-pad, outline="#333333")

        if pts_xy and w >= 10 and h >= 10:
            arr = np.asarray(pts_xy, dtype=float)
            xs, ys = arr[:,0], arr[:,1]
            # Scale to central 5–95% to keep it stable
            x0, x1 = np.percentile(xs, [5, 95]); y0, y1 = np.percentile(ys, [5, 95])
            if x0 == x1: x0, x1 = x0-1, x1+1
            if y0 == y1: y0, y1 = y0-1, y1+1

            def to_pix(x,y):
                X = pad + (x-x0) * (w-2*pad) / (x1-x0)
                Y = h - (pad + (y-y0) * (h-2*pad) / (y1-y0))
                return X, Y

            r = 1.5
            for x,y in pts_xy:
                X,Y = to_pix(x,y)
                self.canvas.create_oval(X-r, Y-r, X+r, Y+r, outline="#66ccff")

        if self.node.estop_active():
            self.canvas.create_text(
                w/2, h/2,
                text="ESTOP ACTIVE",
                fill="#ff4444",
                font=("Segoe UI", 28, "bold")
            )

    def run(self): self.root.mainloop()


def ros_spin(node: GuiNode): rclpy.spin(node)

def main():
    rclpy.init()
    q_scan, q_img = queue.Queue(), queue.Queue()
    node = GuiNode(q_scan, q_img)
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()
    App(node, q_scan, q_img).run()

if __name__ == "__main__":
    main()
