#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mission Console   Figma Layout (Tkinter + ROS2)
- Matches the Figma screenshot structure and visuals as closely as Tkinter allows.
- Fully wired to your existing backend (camera, lidar, odometry, telemetry, estop, waypoints).
"""

import io, math, time, queue, threading, datetime
from typing import Optional, Tuple, List

import numpy as np
import tkinter as tk
from tkinter import ttk
import os
from std_msgs.msg import Float32, String, Int32

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time


from PIL import Image as PILImage
from PIL import ImageTk

from sensor_msgs.msg import Image, LaserScan, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String, Float32MultiArray
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, FluidPressure, Temperature
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSDurabilityPolicy



RAW_IMAGE_TYPE  = 'sensor_msgs/msg/Image'
COMP_IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
SUPPORTED_RAW   = {'rgb8','bgr8','mono8','rgba8','bgra8'}

SEA_LEVEL_P0_PA = 101325.0

def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def _hypsometric_altitude(p_pa: float, T_k: float, p0_pa: float = SEA_LEVEL_P0_PA) -> float:
    if p_pa <= 0.0 or p0_pa <= 0.0:
        return float('nan')
    try:
        return 44330.0 * (1.0 - (p_pa / p0_pa) ** 0.1903)
    except Exception:
        return float('nan')

# ------------------------------ ROS NODE ------------------------------
class GuiNode(Node):
    def __init__(self, msg_queue: queue.Queue, img_queue: queue.Queue):
        super().__init__('gui_panel_node')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('estop_topic', '/e_stop')
        self.declare_parameter('max_altitude', 10.0)


        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('flight_mode_topic', '/flight_mode')

        

        # Altitude source selection
        self.declare_parameter('altitude_mode', 'auto')
        self.declare_parameter('altitude_topic', '')

        self.declare_parameter('tree_count_topic',   '/mission/tree_count')
        self.declare_parameter('people_count_topic', '/mission/people_count')
        

     
        self.declare_parameter('stump_count_topic', '/mission/stump_count')
        self.declare_parameter('detections_topic', '/trees/cut')

        #Audio Detetction
        # Audio / Chainsaw detector topics
        self.declare_parameter('chainsaw_status_topic',  '/audio/chainsaw/status')   # std_msgs/String
        self.declare_parameter('chainsaw_metrics_topic', '/audio/chainsaw/metrics')  # std_msgs/Float32MultiArray [class_id, conf, f0_hz, band_power]
        self.declare_parameter('battery_topic', '/battery')
        self.declare_parameter('baro_topic', '/baro')
        self.declare_parameter('temperature_topic', '/temperature')

        self.msg_queue = msg_queue
        self.img_queue = img_queue

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_transient = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )


        from std_msgs.msg import Int32

        self.tree_count: int   = 0
        self.people_count: int = 0
        self.stump_count: int   = 0

        tct = self.get_parameter('tree_count_topic').get_parameter_value().string_value or ''
        pct = self.get_parameter('people_count_topic').get_parameter_value().string_value or ''
        sct = self.get_parameter('stump_count_topic').get_parameter_value().string_value or ''
        self.declare_parameter('legal_cut_count_topic',   '/mission/cuts_legal')
        self.declare_parameter('illegal_cut_count_topic', '/mission/cuts_illegal')
        self.legal_cuts: int = 0
        self.illegal_cuts: int = 0
        lct = self.get_parameter('legal_cut_count_topic').get_parameter_value().string_value or ''
        ilct = self.get_parameter('illegal_cut_count_topic').get_parameter_value().string_value or ''


        if tct:
            self.create_subscription(Int32, tct, lambda m: setattr(self, 'tree_count', int(m.data)), qos_transient)
        if pct:
            self.create_subscription(Int32, pct, lambda m: setattr(self, 'people_count', int(m.data)), qos_transient)
        if sct:
            self.create_subscription(Int32, sct, lambda m: setattr(self, 'stump_count', int(m.data)), qos_transient)
        if lct:
            self.create_subscription(Int32, lct, lambda m: setattr(self, 'legal_cuts', int(m.data)), qos_transient)
        if ilct:
            self.create_subscription(Int32, ilct, lambda m: setattr(self, 'illegal_cuts', int(m.data)), qos_transient)

        # Subscriptions
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_best_effort)

        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        if cloud_topic:
            self.create_subscription(PointCloud2, cloud_topic, self.on_cloud, qos_best_effort)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

        alt_over = self.get_parameter('altitude_topic').get_parameter_value().string_value
        if alt_over:
            self.create_subscription(Odometry, alt_over, self.on_odom_alt_override, 10)

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
        self._altitude_odom: Optional[float] = None
        self._altitude_gps: Optional[float] = None
        self._altitude_baro: Optional[float] = None

        self.position_xy: Optional[Tuple[float, float]] = None
        self.yaw_rad: Optional[float] = None



                # --- IMU accel display options ---
        self.declare_parameter('imu_gravity_comp', True)     # subtract gravity? (world frame)
        self.declare_parameter('imu_show_world', True)       # display in world(odom) frame; else IMU body frame
        self.declare_parameter('g0', 9.80665)                # gravity constant (m/s^2)
        self.declare_parameter('accel_alpha', 0.3)           # EMA smoothing factor (0..1)

        self._accel_alpha = float(self.get_parameter('accel_alpha').value)
        self._g0 = float(self.get_parameter('g0').value)
        self._imu_gravity_comp = bool(self.get_parameter('imu_gravity_comp').value)
        self._imu_show_world    = bool(self.get_parameter('imu_show_world').value)

        # live accel state
        self.accel_body = None     # (ax, ay, az) in IMU/body frame (m/s^2)
        self.accel_world = None    # (Ax, Ay, Az) in world/odom frame (m/s^2)
        self.accel_mag = None      # |A| of whichever set we choose to show


        # Mission/health state
        self.battery_pct: Optional[float] = None
        self.flight_mode: Optional[str] = None
        self.gps_fix: Optional[NavSatFix] = None
        self.imu_rpy: Optional[Tuple[float, float, float]] = None
        self.wind_ms: Optional[float] = None
        self.wind_heading_deg: Optional[float] = None

        # Audio detection state
        self.audio_class: Optional[str] = None      # "chainsaw" / "ambient" / "other"
        self.audio_conf: Optional[float] = None     # 0..1
        self.audio_f0_hz: Optional[float] = None    # dominant Hz
        self.audio_band_power: Optional[float] = None


        # Barometer / Temperature
        self.baro_pressure_pa: Optional[float] = None
        self.temperature_c: Optional[float] = None
        self._p0_pa: float = SEA_LEVEL_P0_PA

        self.breadcrumb: List[Tuple[float, float]] = []
        self.breadcrumb_max = 200

        self.waypoints_xy: List[Tuple[float, float]] = []
        self.next_wp_idx: int = 0

        self.tree_positions_xy: List[Tuple[float, float]] = []

        # LiDAR prefilter
        self._scan_keep_every = 2
        self._scan_min_valid = 0.03

    
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Time sanity
        self._last_scan_stamp_ns: Optional[int] = None
        self._start_time = time.time()

        # Subscribe to extra topics
        self._subscribe_extras()




                # ---- AUDIO (embedded detector) ----
        self.declare_parameter('mic_audio_topic', '/microphone/audio')  # raw Float32MultiArray audio blocks
        self.declare_parameter('audio_fs',        16000)
        self.declare_parameter('audio_frame_ms',  500)
        self.declare_parameter('audio_hop_ms',    250)
        self.declare_parameter('chainsaw_low_hz', 90.0)
        self.declare_parameter('chainsaw_high_hz',400.0)
        self.declare_parameter('audio_conf_thresh', 0.55)
        self.declare_parameter('audio_decision_window', 5)

        
        # Waypoints
        self.declare_parameter('waypoints_path_topic',  '/mission/waypoints_path')
        self.declare_parameter('waypoints_array_topic', '/mission/waypoints')


        wpt_path = self.get_parameter('waypoints_path_topic').get_parameter_value().string_value or ''
        wpt_arr  = self.get_parameter('waypoints_array_topic').get_parameter_value().string_value or ''

        if wpt_path:
            self.create_subscription(Path, wpt_path, self.on_path, qos_transient)
        if wpt_arr:
            self.create_subscription(PoseArray, wpt_arr, self.on_posearray_waypoints, qos_transient)

        
        self.wp_idx = 0
        self.wp_total = 0

        self.sub_wp_idx   = self.create_subscription(Int32, '/mission/waypoint_index',
                                                    lambda m: self._on_wp_idx(m.data), qos_transient)
        self.sub_wp_total = self.create_subscription(Int32, '/mission/waypoint_total',
                                                    lambda m: self._on_wp_total(m.data), qos_transient)
        self.declare_parameter('mission_cmd_topic', '/mission/cmd')
        cmd_topic = self.get_parameter('mission_cmd_topic').get_parameter_value().string_value or '/mission/cmd'
        self.mission_cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.get_logger().info(f"[GUI] Command publisher on {cmd_topic}")
        self.mission_state = "IDLE"
        self.mission_progress = 0.0


        self._aud_topic = self.get_parameter('mic_audio_topic').get_parameter_value().string_value
        self._fs        = int(self.get_parameter('audio_fs').value)
        self._frame_len = int(self.get_parameter('audio_frame_ms').value) * self._fs // 1000
        self._hop_len   = int(self.get_parameter('audio_hop_ms').value)   * self._fs // 1000
        self._band_lo   = float(self.get_parameter('chainsaw_low_hz').value)
        self._band_hi   = float(self.get_parameter('chainsaw_high_hz').value)
        self._aud_conf_thresh = float(self.get_parameter('audio_conf_thresh').value)
        from collections import deque
        self._aud_votes = deque(maxlen=max(1, int(self.get_parameter('audio_decision_window').value)))

        import numpy as _np
        self._aud_buf = _np.zeros(0, dtype=_np.float32)

        # Subscribe to mic audio (Float32MultiArray blocks)
        from std_msgs.msg import Float32MultiArray
        qos_audio = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Float32MultiArray, self._aud_topic, self._on_audio_block, qos_audio)

        # Process timer (every hop)
        self._aud_timer = self.create_timer(self._hop_len / max(1, self._fs), self._audio_process)


 
    def _subscribe_extras(self):
        bt = self.get_parameter('battery_topic').get_parameter_value().string_value
        if bt:
            self.create_subscription(BatteryState, bt, self.on_battery, 10)

        gt = self.get_parameter('gps_topic').get_parameter_value().string_value
        if gt:
            self.create_subscription(NavSatFix, gt, self.on_gps, 10)

        it = self.get_parameter('imu_topic').get_parameter_value().string_value
        if it:
            self.create_subscription(Imu, it, self.on_imu, 10)

        fmt = self.get_parameter('flight_mode_topic').get_parameter_value().string_value
        if fmt:
            self.create_subscription(String, fmt, self.on_flight_mode, 10)

        # Baro & Temperature
        btpc = self.get_parameter('baro_topic').get_parameter_value().string_value
        if btpc:
            self.create_subscription(FluidPressure, btpc, self.on_baro, 10)

        tpc = self.get_parameter('temperature_topic').get_parameter_value().string_value
        if tpc:
            self.create_subscription(Temperature, tpc, self.on_temperature, 10)





        dt = self.get_parameter('detections_topic').get_parameter_value().string_value
        if dt:
            self.create_subscription(PoseArray, dt, self.on_tree_detections, 10)

        # Chainsaw detector subscriptions
        st = self.get_parameter('chainsaw_status_topic').get_parameter_value().string_value
        if st:
            self.create_subscription(String, st, self.on_chainsaw_status, 10)

        mt = self.get_parameter('chainsaw_metrics_topic').get_parameter_value().string_value
        if mt:
            self.create_subscription(Float32MultiArray, mt, self.on_chainsaw_metrics, 10)

        # initial GUI state = PAUSED until Start is pressed
        self.mission_state = "PAUSED"

        # get mission state from main (latched QoS so late joiners see last)
        self.create_subscription(
            String,
            '/mission/state',
            self._on_state,
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

    def on_stumps(self, msg: Float32MultiArray):
        """
        Accepts a single stump per message: [x, y, r, h_lb].
        If you publish batches, loop over chunks of 4 before appending.
        """
        try:
            data = list(msg.data)
            if len(data) >= 4:
                x, y, r, h = map(float, data[:4])
                self.stumps.append((x, y, r, h))
                # keep memory bounded
                if len(self.stumps) > 500:
                    self.stumps = self.stumps[-500:]
        except Exception:
            pass

    def _on_wp_idx(self, v):   self.wp_idx = int(v)
    def _on_wp_total(self, v): self.wp_total = int(v)

    # ---- E-STOP ----
    def engage_estop(self):
        if not self._estop:
            self._estop = True
            try:
                self.estop_pub.publish(Bool(data=True))
            except Exception:
                pass

    def reset_estop(self):
        if self._estop:
            self._estop = False
            try:
                self.estop_pub.publish(Bool(data=False))
            except Exception:
                pass

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
            return

        if cam_type == 'raw':
            self._img_sub = self.create_subscription(Image, cam_topic, self.on_image_raw, 10)
        else:
            self._img_sub = self.create_subscription(CompressedImage, cam_topic, self.on_image_compressed, 10)
        self._cam_type = cam_type

    @staticmethod
    def _quat_to_R(qx, qy, qz, qw):
        # Rotation matrix: body -> world
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        return [
            [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
            [  2*(xy+wz), 1-2*(xx+zz),     2*(yz-wx)],
            [  2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]
        ]

    @staticmethod
    def _mat_vec3(M, v):
        return (
            M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
            M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
            M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
        )

    @staticmethod
    def _ema(prev, new, alpha):
        if prev is None: return new
        return tuple(alpha*n + (1-alpha)*p for p, n in zip(prev, new))


    def on_chainsaw_status(self, msg: String):
        """
        Accepts lines like: 'class=chainsaw conf=1.00 f0=180.0Hz bandPwr=0.91'
        Robust to extra/missing fields.
        """
        s = msg.data.strip()
        # defaults
        cls, conf, f0, bp = None, None, None, None
        try:
            for tok in s.replace(',', ' ').split():
                if tok.startswith('class='):
                    cls = tok.split('=',1)[1]
                elif tok.startswith('conf='):
                    conf = float(tok.split('=',1)[1])
                elif tok.startswith('f0='):
                    v = tok.split('=',1)[1].lower().replace('hz','')
                    f0 = float(v)
                elif tok.startswith('bandPwr=') or tok.startswith('bandpwr='):
                    bp = float(tok.split('=',1)[1])
        except Exception:
            pass

        # apply if present
        if cls is not None: self.audio_class = cls
        if conf is not None: self.audio_conf = conf
        if f0 is not None: self.audio_f0_hz = f0
        if bp is not None: self.audio_band_power = bp

    def on_chainsaw_metrics(self, msg: Float32MultiArray):
        """
        Expects [class_id, conf, f0_hz, band_power]
        class_id: 0=ambient, 1=chainsaw, 2=other (tweak to your node)
        """
        try:
            data = list(msg.data)
            self.create_subscription(String,  '/audio/classification',
                         lambda m: setattr(self, 'audio_class', m.data.strip()), 10)

            self.create_subscription(Float32, '/audio/chainsaw_confidence',
                                    lambda m: setattr(self, 'audio_conf', float(m.data)), 10)

            self.create_subscription(Float32, '/audio/dominant_frequency',
                                    lambda m: setattr(self, 'audio_f0_hz', float(m.data)), 10)

            self.create_subscription(Float32, '/audio/psd_band_power',
                                    lambda m: setattr(self, 'audio_band_power', float(m.data)), 10)
            if len(data) >= 4:
                class_id = int(round(data[0]))
                self.audio_conf = float(data[1])
                self.audio_f0_hz = float(data[2])
                self.audio_band_power = float(data[3])
                self.audio_class = {0: "ambient", 1: "chainsaw", 2: "other"}.get(class_id, f"class_{class_id}")
        except Exception:
            pass

    def _on_state(self, msg: String):
        st = (msg.data or "").strip().upper()
        self.mission_state = st

        # Keep only *flags* here; never call GUI methods from the node.
        if st == "E-STOP":
            self._estop = True
        elif st in ("RUNNING", "PAUSED", "IDLE", "STOPPED", "RTL", "LAND"):
            # Clear local E-STOP flag when mission reports a non-estop state.
            # (If you want a physical latch, remove this line and require Reset.)
            self._estop = False



    def _on_progress(self, msg):
        self.mission_progress = msg.data
        try:
            self.progress_bar['value'] = int(self.mission_progress * 100)
        except:
            pass

    def _update_wp_label(self):
    # whichever label you use:
    # e.g., self.tree_count or self.waypoint_count_label — just set the textvariable or configure
        text = f"{self.wp_idx}/{self.wp_total}" if self.wp_total else "0/0"
        try:
            self.waypoint_label_var.set(text)   # if using a StringVar
        except Exception:
            self.waypoint_label.configure(text=text)

    def _update_waypoint_label(self):
        text = f"{self.wp_idx}/{self.wp_total}"
        try:
            self.waypoint_label_var.set(text)  # if using StringVar
        except:
            try:
                self.waypoint_label.configure(text=text)  # if direct widget configure
            except:
                pass

    def _resolve_camera_topic(self, requested: str):
        if requested:
            for n, ts in self.get_topic_names_and_types():
                if n == requested and (RAW_IMAGE_TYPE in ts or COMP_IMAGE_TYPE in ts):
                    return (requested, 'raw') if RAW_IMAGE_TYPE in ts else (requested, 'compressed')
            return None, None

        for n, ts in self.get_topic_names_and_types():
            if RAW_IMAGE_TYPE in ts:
                return n, 'raw'
        for n, ts in self.get_topic_names_and_types():
            if COMP_IMAGE_TYPE in ts:
                return n, 'compressed'
        return None, None

    def on_estop_pressed(self):
        self.estop_pub.publish(Bool(data=True))


    def _on_audio_block(self, msg):
        """Append incoming audio samples (Float32MultiArray) to the buffer."""
        try:
            import numpy as np
            arr = np.asarray(msg.data, dtype=np.float32).ravel()
            if arr.size:
                self._aud_buf = np.concatenate([self._aud_buf, arr])
        except Exception:
            pass

    def _audio_process(self):
        """Run a simple FFT-based detector over frames and update GUI fields."""
        try:
            import numpy as np
            if self._aud_buf.size < self._frame_len:
                return

            # Take one frame, keep overlap (hop)
            x = self._aud_buf[:self._frame_len]
            self._aud_buf = self._aud_buf[self._hop_len:]

            # Window + FFT
            win = np.hanning(len(x))
            xw  = x * win
            spec = np.fft.rfft(xw)
            mag  = np.abs(spec) + 1e-12
            freqs = np.fft.rfftfreq(len(x), d=1.0 / self._fs)

            # Focus chainsaw band
            mask = (freqs >= self._band_lo) & (freqs <= self._band_hi)
            if not np.any(mask):
                return
            band_mag   = mag[mask]
            band_freqs = freqs[mask]
            peak_idx   = int(np.argmax(band_mag))
            f0         = float(band_freqs[peak_idx])

            # Relative band power
            band_power  = float(np.sum(band_mag**2))
            total_power = float(np.sum(mag**2)) + 1e-12
            rel_band    = band_power / total_power

            # Harmonicity (quick & dirty)
            max_hz = 2000.0
            kmax   = int(max_hz // max(f0, 1.0))
            hvals  = []
            bw_hz  = max(5.0, f0 * 0.05)
            for k in range(1, max(2, kmax + 1)):
                tgt = k * f0
                if tgt > freqs[-1]:
                    break
                m = (freqs >= tgt - bw_hz) & (freqs <= tgt + bw_hz)
                if np.any(m):
                    hvals.append(np.max(mag[m]))
            harm = float(np.mean(hvals) / (np.mean(mag) + 1e-12)) if hvals else 0.0

            # Confidence & label
            conf  = float(0.6 * np.clip(rel_band * 2.0, 0.0, 1.0) + 0.4 * np.clip(harm, 0.0, 1.0))
            label = 'chainsaw' if conf >= self._aud_conf_thresh else 'ambient'

            # Smooth over a short window
            self._aud_votes.append((label, conf, f0, rel_band))
            labels = [d[0] for d in self._aud_votes]
            maj    = max(set(labels), key=labels.count)
            mean_c = float(np.mean([d[1] for d in self._aud_votes]))
            mean_f = float(np.mean([d[2] for d in self._aud_votes]))
            mean_p = float(np.mean([d[3] for d in self._aud_votes]))

            # Update the same fields your poll_telemetry() already reads
            self.audio_class       = maj
            self.audio_conf        = mean_c
            self.audio_f0_hz       = mean_f
            self.audio_band_power  = mean_p

        except Exception:
            # Keep GUI robust
            pass


    # ---- Callbacks ----
    def on_scan(self, msg: LaserScan):
        # if self._estop:
        #     return

        # Drop out-of-order timestamps
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_scan_stamp_ns is not None and stamp_ns < self._last_scan_stamp_ns:
            return
        self._last_scan_stamp_ns = stamp_ns

        # Build points
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

        # Transform to od# Transform to a world frame
        src_frame = (msg.header.frame_id or 'laser').lstrip('/')  # strip leading '/'
        target_candidates = ['odom', 'map', 'base_link']          # try these in order
        frame_used = src_frame
        world = False

        def _apply_tf(tfm, pts_local):
            tx = tfm.transform.translation.x
            ty = tfm.transform.translation.y
            q  = tfm.transform.rotation
            yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            cy, sy = math.cos(yaw), math.sin(yaw)
            out = []
            for x, y in pts_local:
                X = cy*x - sy*y + tx
                Y = sy*x + cy*y + ty
                out.append((X, Y))
            return out

        tf_ok = False
        for tgt in target_candidates:
            try:
                # Latest available TF (time=0) and a more generous timeout
                tfm = self.tf_buffer.lookup_transform(
                    tgt, src_frame, Time(), timeout=Duration(seconds=0.5)
                )
                pts = _apply_tf(tfm, pts)
                frame_used = tgt
                world = True
                tf_ok = True
                break
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

        # If no TF worked, we’ll keep points in sensor frame and let the drawer auto-fit

        try:
            self.msg_queue.put({
            'src': 'scan',
            'stamp': f"{msg.header.stamp.sec}.{str(msg.header.stamp.nanosec).zfill(9)}",
            'frame': frame_used,
            'n_total': len(pts),
            'points_xy': pts,
            'world': world,
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
                return
            w, h = msg.width, msg.height
            img = PILImage.frombytes('RGB' if 'rgb' in msg.encoding else 'L', (w, h), bytes(msg.data))
            if msg.encoding in ('bgr8', 'bgra8'):
                img = PILImage.fromarray(np.array(img)[:, :, ::-1])
            self.img_queue.put_nowait(img)
        except Exception:
            pass

    def on_image_compressed(self, msg: CompressedImage):
        if self._estop:
            return
        try:
            bio = io.BytesIO(bytes(msg.data))
            img = PILImage.open(bio).convert('RGB')
            self.img_queue.put_nowait(img)
        except Exception:
            pass

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._altitude_odom = float(p.z)
        self.position_xy = (float(p.x), float(p.y))
        q = msg.pose.pose.orientation
        self.yaw_rad = _yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.position_xy is not None:
            self.breadcrumb.append(self.position_xy)
            if len(self.breadcrumb) > self.breadcrumb_max:
                self.breadcrumb.pop(0)

    def on_odom_alt_override(self, msg: Odometry):
        try:
            self._altitude_odom = float(msg.pose.pose.position.z)
        except Exception:
            pass

    # ---- extra callbacks ----
    def on_battery(self, msg: BatteryState):
        if msg.percentage is not None and math.isfinite(msg.percentage):
            self.battery_pct = max(0.0, min(1.0, float(msg.percentage)))
        else:
            self.battery_pct = None

    def on_flight_mode(self, msg: String):
        self.flight_mode = msg.data.strip()

    def on_gps(self, msg: NavSatFix):
        self.gps_fix = msg
        if math.isfinite(getattr(msg, 'altitude', float('nan'))):
            self._altitude_gps = float(msg.altitude)

    def on_start_clicked(self):
    
     self.mission_cmd_pub.publish(String(data='start'))

    def on_stop_clicked(self):
     self.mission_cmd_pub.publish(String(data='stop'))

    def on_imu(self, msg: Imu):
        # --- orientation to RPY (unchanged) ---
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

        # --- raw linear acceleration in IMU/body frame ---
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)
        a_body = (ax, ay, az)

        # smooth body accel (EMA)
        self.accel_body = self._ema(self.accel_body, a_body, self._accel_alpha)

        # --- rotate to world and (optionally) remove gravity ---
        R_bw = self._quat_to_R(q.x, q.y, q.z, q.w)       # body -> world
        a_world = self._mat_vec3(R_bw, self.accel_body)

        if self._imu_gravity_comp:
            # subtract gravity in world Z (down = +g depending on convention; here we subtract +g on +Z)
            a_world = (a_world[0], a_world[1], a_world[2] - self._g0)

        # smooth world accel too (keeps both representations pleasant)
        self.accel_world = self._ema(self.accel_world, a_world, self._accel_alpha)

        # pick what to present in GUI
        Ax, Ay, Az = (self.accel_world if self._imu_show_world else self.accel_body)
        self.accel_mag = math.sqrt(Ax*Ax + Ay*Ay + Az*Az)


    def on_baro(self, msg: FluidPressure):
        if math.isfinite(msg.fluid_pressure) and msg.fluid_pressure > 0:
            self.baro_pressure_pa = float(msg.fluid_pressure)
            T_k = (self.temperature_c + 273.15) if (self.temperature_c is not None and math.isfinite(self.temperature_c)) else 288.15
            self._altitude_baro = _hypsometric_altitude(self.baro_pressure_pa, T_k, self._p0_pa)

    def on_temperature(self, msg: Temperature):
        if math.isfinite(msg.temperature):
            self.temperature_c = float(msg.temperature)
            if self.baro_pressure_pa is not None:
                T_k = self.temperature_c + 273.15
                self._altitude_baro = _hypsometric_altitude(self.baro_pressure_pa, T_k, self._p0_pa)

    def on_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses] if msg.poses else []
        self.waypoints_xy = pts
        if self.position_xy and pts:
            rx, ry = self.position_xy
            try:
                self.next_wp_idx = min(range(len(pts)), key=lambda i: (rx - pts[i][0])**2 + (ry - pts[i][1])**2)
            except ValueError:
                self.next_wp_idx = 0

    def on_posearray_waypoints(self, msg: PoseArray):
        pts = [(p.position.x, p.position.y) for p in msg.poses] if msg.poses else []
        self.waypoints_xy = pts
        if self.position_xy and pts:
            rx, ry = self.position_xy
            try:
                self.next_wp_idx = min(range(len(pts)), key=lambda i: (rx - pts[i][0])**2 + (ry - pts[i][1])**2)
            except ValueError:
                self.next_wp_idx = 0

    def on_tree_detections(self, msg: PoseArray):
        self.tree_positions_xy = [(p.position.x, p.position.y) for p in msg.poses]


class AppFigma:
    """
    Fixed (non-responsive) Figma layout with icons on chips/cards/buttons.
    Keeps all callbacks/queues/pollers from your original app; only visuals differ.
    """
    def __init__(self, node: GuiNode, q_scan: queue.Queue, q_img: queue.Queue):
        self.node, self.q_scan, self.q_img = node, q_scan, q_img
        self.icons = IconManager(ICONS_DIR)  

        # --- Window ---
        self.root = tk.Tk()
        self.root.title("Mission Console")
        self.root.geometry("1920x1080")
        #self.root.state("zoomed")
        self.root.resizable(True, True)


        # --- Theme ---
        self._apply_theme()

        # === TOP BAR ===========================================================
        topbar = ttk.Frame(self.root, padding=(16, 10, 16, 8), style="Topbar.TFrame")
        topbar.pack(side=tk.TOP, fill=tk.X)

        # left
        left_wrap = ttk.Frame(topbar, style="Topbar.TFrame"); left_wrap.pack(side=tk.LEFT)
        self.lbl_sys = ttk.Label(left_wrap, text="System Active", style="TopTitle.TLabel")
        self.lbl_sys.pack(side=tk.LEFT, padx=(0, 8))
        # profile badge with tiny robot
        robot_ic = self.icons.get("robot", 16)
        self.lbl_profile = ttk.Label(left_wrap, text="Drone Control", style="BadgeGrey.TLabel",
                                     image=robot_ic, compound="left")
        self.lbl_profile.image = robot_ic
        self.lbl_profile.pack(side=tk.LEFT)

        # right
        right = ttk.Frame(topbar, style="Topbar.TFrame"); right.pack(side=tk.RIGHT)
        wifi_ic = self.icons.get("wifi", 16) or None
        rc_ic   = self.icons.get("rc", 16) or None
        clock_ic= self.icons.get("time", 16) or None
        self.lbl_tel = ttk.Label(right, text="Telemetry: 100%", style="TopMeta.TLabel",
                                 image=wifi_ic, compound="left")
        self.lbl_tel.image = wifi_ic
        self.lbl_tel.pack(side=tk.LEFT, padx=(10,10))
        self.lbl_rc  = ttk.Label(right, text="RC: --", style="TopMeta.TLabel",
                                 image=rc_ic, compound="left")
        self.lbl_rc.image = rc_ic
        self.lbl_rc.pack(side=tk.LEFT, padx=(10,10))
        self.lbl_utc = ttk.Label(right, text="UTC --:--:--", style="TopMeta.TLabel",
                                 image=clock_ic, compound="left")
        self.lbl_utc.image = clock_ic
        self.lbl_utc.pack(side=tk.LEFT, padx=(10,0))

        # === METRICS ROW (6) ===================================================
        metrics = ttk.Frame(self.root, padding=(16, 0, 16, 8), style="Bg.TFrame")
        metrics.pack(side=tk.TOP, fill=tk.X)

        self.card_tree    = self._metric_card(metrics, "Tree Count",  "0", "Detected Trees", 0, icon=("tree",16))
        self.tree_people_var = tk.StringVar(value="People: 0")
        # Inline (to the right of the big number)
        self.tree_cuts_var = tk.StringVar(value="Legal: 0 | Illegal: 0")
        self._add_inline_right_of_value(self.card_tree, self.tree_cuts_var)


        self._add_footer_counter(self.card_tree, self.tree_people_var)
        self.card_audio   = self._metric_card(metrics, "Audio (Hz)",  "-- Hz", "—",           1, icon=("mic",16))
        self.card_speed   = self._metric_card(metrics, "Speed",      "-- m/s",  "-- km/h",           2, icon=("speed",16))
        self.card_home    = self._metric_card(metrics, "Home Dist",  "-- m",    "Within bounds",     3, icon=("home",16))
        self.card_time    = self._metric_card(metrics, "Flight Time","00:00",   "Elapsed",           4, icon=("time",16))
        self.card_waypts  = self._metric_card(metrics, "Waypoints",   "0 / 0", "Completed", 5, icon=("map-pin",16))



        self._hover_swap(self.lbl_tel, "TopMeta.TLabel", "TopMetaHover.TLabel")
        self._hover_swap(self.lbl_rc,  "TopMeta.TLabel", "TopMetaHover.TLabel")
        self._hover_swap(self.lbl_utc, "TopMeta.TLabel", "TopMetaHover.TLabel")
        self._hover_swap(self.lbl_sys, "TopTitle.TLabel", "TopTitleHover.TLabel")
        self._hover_swap(self.lbl_profile, "BadgeGrey.TLabel", "TopMetaHover.TLabel")

        # === BODY GRID  ===================================================
        body = ttk.Frame(self.root, padding=(16, 0, 16, 16), style="Bg.TFrame")
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3); body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1);    body.rowconfigure(1, weight=3)

        # (A) Camera (top-left)
        # --- Camera Feed Banner Card ---
        cam = ttk.Frame(body, style="Card.TFrame")
        cam.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        body.grid_columnconfigure(0, weight=1)
        body.grid_rowconfigure(0, weight=1)

        # Header banner
        cam_hdr = ttk.Frame(cam, style="BannerBlue.TFrame")
        cam_hdr.pack(fill="x")

        # Left: title + icon
        ttk.Label(cam_hdr, text="Camera Feed", style="BannerBlue.TLabel",
                image=self.icons.get("camera", 16), compound="left").pack(side="left", padx=10, pady=8)

        # Right: rounded red “LIVE” chip
        live_chip = tk.Label(
            cam_hdr,
            text="LIVE",
            bg="#FDECEC",     # soft red background
            fg="#E11900",     # bold red text
            font=("SF Pro Text", 9, "bold"),
            padx=10, pady=3,
            bd=0
        )
        live_chip.pack(side="right", padx=10, pady=8)
        live_chip.configure(relief="flat")
        live_chip.bind("<Enter>", lambda e: live_chip.config(bg="#FAD7D7"))
        live_chip.bind("<Leave>", lambda e: live_chip.config(bg="#FDECEC"))




        
        cam_hdr = ttk.Frame(cam, style="Card.TFrame"); cam_hdr.pack(fill="x")
        rec_ic = self.icons.get("record", 14)
        ttk.Label(cam_hdr, text=" REC", style="BadgeRed.TLabel", image=rec_ic, compound="left").pack(side=tk.LEFT, padx=(0,8))
        self.lbl_cam_info = ttk.Label(cam_hdr, text=" 1920x1080 30fps", style="BadgeGrey.TLabel",
                                      image=self.icons.get("camera",14), compound="left")
        self.lbl_cam_info.pack(side=tk.LEFT)
        ttk.Label(cam_hdr, text=" AUTO Mode", style="BadgeGrey.TLabel",
                  image=self.icons.get("robot",14), compound="left").pack(side=tk.RIGHT)
        self.cam_label = ttk.Label(cam, text="Camera Stream InActive",
                                   style="CardMutedCenter.TLabel", anchor="center")
        self.cam_label.pack(fill="both", expand=True, pady=8)

        # (B) Control panel (top-right)
        ctl = ttk.Frame(body, style="Card.TFrame")
        ctl.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        body.grid_columnconfigure(1, weight=2)

        # Banner header
        ctl_hdr = ttk.Frame(ctl, style="BannerTeal.TFrame")
        ctl_hdr.pack(fill="x")

        # Left: title + compass icon
        ttk.Label(ctl_hdr, text="Flight Controls", style="BannerTeal.TLabel",
                image=self.icons.get("compass", 16), compound="left").pack(side="left", padx=10, pady=8)

        # Right: flight mode chip (dynamic)
        #self._chip(ctl_hdr, "AUTO", bg="#E9FFF6", fg="#156F4B", hover_bg="#D9FFEF")
        self._mode_chip = self._chip(ctl_hdr, "Running", bg="#33992F", fg="#A4E7AF", hover_bg="#9DE68E")

        # Body (your existing content)
        ctl_body = ttk.Frame(ctl, style="Card.TFrame", padding=(6, 8))
        ctl_body.pack(fill="both", expand=True)
        fm_row = ttk.Frame(ctl, style="Card.TFrame"); fm_row.pack(fill="x", pady=(2,8))
        ttk.Label(fm_row, text="Mode", style="Muted.TLabel",
                  image=self.icons.get("compass",14), compound="left").pack(side=tk.LEFT)
        self.flight_mode_var = tk.StringVar(value="Auto")
        ttk.Combobox(fm_row, textvariable=self.flight_mode_var,
                     values=["Auto","Guided","Hold","Manual"], state="readonly",
                     width=12).pack(side=tk.RIGHT, ipadx=8)

        ttk.Label(ctl, text="Emergency Controls", style="Section.TLabel",
          image=self.icons.get("alert",16), compound="left").pack(anchor="w", pady=(6,6))

        em = ttk.Frame(ctl, style="Card.TFrame"); em.pack(fill="x", pady=(0,6))
        em.columnconfigure(0, weight=1, uniform="em")
        em.columnconfigure(1, weight=1, uniform="em")

        # LEFT column (E-STOP + centered banner)
        left = ttk.Frame(em, style="Card.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0,8))
        left.columnconfigure(0, weight=1)

        self.estop_btn = tk.Button(
            left, text="  E-STOP", font=("SF Pro Text", 14, "bold"),
            bg="#E11900", fg="white", bd=0, height=2, cursor="hand2",
            activebackground="#C41600", activeforeground="white",
            command=self.on_estop_press
        )
        estop_ic = self.icons.get("stop", 18) or self.icons.get("alert", 18)
        if estop_ic:
            self.estop_btn.config(image=estop_ic, compound="left")
            self.estop_btn.image = estop_ic
        self.estop_btn.grid(row=0, column=0, sticky="ew")

        self.estop_banner = tk.Label(
            left, text="EMERGENCY STOP ACTIVE",
            font=("SF Pro Text", 14, "bold"),
            bg="#C73B3B", fg="white", bd=0, pady=10, anchor="center", justify="center"
        )
        self.estop_banner.grid(row=1, column=0, sticky="ew", pady=(8,0))
        self.estop_banner.grid_remove()

        # RIGHT column (Reset button)
        right = ttk.Frame(em, style="Card.TFrame")
        right.grid(row=0, column=1, sticky="nsew", padx=(8,0))

        self.reset_btn = tk.Button(
            right, text="  Reset", font=("SF Pro Text", 12, "bold"),
            bg="#EEF0F5", fg="#111", bd=0, height=2, cursor="hand2",
            activebackground="#E2E6EF", activeforeground="#111",
            command=self.on_estop_reset
        )
        reset_ic = self.icons.get("reset", 16)
        if reset_ic:
            self.reset_btn.config(image=reset_ic, compound="left")
            self.reset_btn.image = reset_ic
        self.reset_btn.pack(fill="x")






        # Mission Controls
        ttk.Label(ctl, text="Mission Controls", style="Section.TLabel",
                  image=self.icons.get("gps",16), compound="left").pack(anchor="w", pady=(8,6))
        mc = ttk.Frame(ctl, style="Card.TFrame"); mc.pack(fill="x")
        # Make a 2×2 grid inside `mc`
        for c in (0, 1):
            mc.columnconfigure(c, weight=1, uniform="mc")
        for r in (0, 1):
            mc.rowconfigure(r, weight=1)

        btn_start  = self._pill(mc, " Start",  "#23A559", self._send_cmd, "start",  icon=("play",18))
        btn_stop   = self._pill(mc, " Stop",   "#EF5944", self._send_cmd, "stop",   icon=("stop",18))
        btn_pause  = self._pill(mc, " Pause",  "#F5A623", self._send_cmd, "pause",  icon=("pause",18))
        btn_resume = self._pill(mc, " Resume", "#4A7BD0", self._send_cmd, "resume", icon=("resume",18))
        for r in (0, 1, 2): mc.rowconfigure(r, weight=1)  # extend grid
        # btn_rtl  = self._pill(mc, " RTL",  "#A855F7", self._send_cmd, "rtl",  icon=("rtl",18))
        # btn_land = self._pill(mc, " Land", "#22D3EE", self._send_cmd, "land", icon=("land",18))

        btn_start .grid(row=0, column=0, sticky="nsew", padx=(0,6), pady=(0,6))
        btn_stop  .grid(row=0, column=1, sticky="nsew", padx=(6,0), pady=(0,6))
        btn_pause .grid(row=1, column=0, sticky="nsew", padx=(0,6), pady=(6,0))
        btn_resume.grid(row=1, column=1, sticky="nsew", padx=(6,0), pady=(6,0))
        # btn_rtl .grid(row=2, column=0, sticky="nsew", padx=(0,6), pady=(6,0))
        # btn_land.grid(row=2, column=1, sticky="nsew", padx=(6,0), pady=(6,0))



        self.lbl_status = ttk.Label(ctl, text="Status: Started", style="BadgeGrey.TLabel")
        self.lbl_status.pack(anchor="w", pady=(8,0))

        # (C) LiDAR map (bottom-left)
        # --- LiDAR Map Banner Card ---
        lidar = ttk.Frame(body, style="Card.TFrame")
        lidar.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        body.grid_rowconfigure(1, weight=3)   # you already had row weights; OK to repeat
        body.grid_columnconfigure(0, weight=3)

        lid_hdr = ttk.Frame(lidar, style="BannerCyan.TFrame")
        lid_hdr.pack(fill="x")

        # Left: title + icon
        ttk.Label(lid_hdr, text="LiDAR Map", style="BannerCyan.TLabel",
                image=self.icons.get("lidar",16), compound="left").pack(side="left", padx=10, pady=8)

        # Right: cyan “360° Scan Active” chip
        self._chip(lid_hdr, "360° Scan Active", bg="#E6FAFF", fg="#116B7A", hover_bg="#D2F4FF")

        # Body area (canvas stays the same, just parented to the banner-card body)
        lid_body = ttk.Frame(lidar, style="Card.TFrame", padding=0)
        lid_body.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(lid_body, height=300, bg="#FAFBFD", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, pady=(6, 0))


        # (D) Telemetry side panel (bottom-right)
    # --- Odometry Banner Card ---
        odo = ttk.Frame(body, style="Card.TFrame")
        odo.grid(row=1, column=1, sticky="nsew", padx=8, pady=8)
        body.grid_columnconfigure(1, weight=2)

        odo_hdr = ttk.Frame(odo, style="BannerPurple.TFrame")
        odo_hdr.pack(fill="x")

        # Left: title + icon
        ttk.Label(odo_hdr, text="Odometry Data", style="BannerPurple.TLabel",
                image=self.icons.get("robot",16), compound="left").pack(side="left", padx=10, pady=8)

        # Right: purple “LIVE” chip
        self._chip(odo_hdr, "LIVE", bg="#F2ECFF", fg="#5A3EAA", hover_bg="#E8DEFF")

        # Body where you put the odometry labels/IMU/altitude/etc.
        telem = ttk.Frame(odo, style="Card.TFrame")
        telem.pack(fill="both", expand=True)

        odo_wrap = ttk.Frame(telem, style="Card.TFrame"); odo_wrap.pack(fill="x", pady=(0,2))
        ttk.Label(odo_wrap, text=" Odometry Data", style="Section.TLabel",
                  image=self.icons.get("compass",16), compound="left").pack(anchor="w")
        self.lbl_odo = ttk.Label(telem, text="", style="Mono.TLabel", justify="left")
        self.lbl_odo.pack(anchor="nw", fill="x")
        ttk.Separator(telem, orient="horizontal").pack(fill="x", pady=6)

            # odo = ttk.Frame(body, style="Card.TFrame")
            # odo.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)

            # odo_hdr = ttk.Frame(odo, style="BannerPurple.TFrame")
            # odo_hdr.pack(fill="x")
            # ttk.Label(odo_hdr, text="Odometry Data", style="BannerPurple.TLabel",
            #         image=self.icons.get("robot",16), compound="left").pack(side="left", padx=10, pady=8)
            # ttk.Label(odo_hdr, text="LIVE", style="BannerPurple.TLabel").pack(side="right", padx=10, pady=8)

            # odo_body = ttk.Frame(odo, style="Card.TFrame", padding=10)
            # odo_body.pack(fill="both", expand=True)


        imu_wrap = ttk.Frame(telem, style="Card.TFrame"); imu_wrap.pack(fill="x", pady=(2,2))
        ttk.Label(imu_wrap, text=" IMU Data", style="Section.TLabel",
                  image=self.icons.get("compass",16), compound="left").pack(anchor="w")
        self.lbl_imu = ttk.Label(imu_wrap, text="Roll:Pitch:Yaw:Accel: 9.81 m/s", style="Muted.TLabel")
        self.lbl_imu.pack(anchor="w", pady=(3,0))

        alt_wrap = ttk.Frame(telem, style="Card.TFrame"); alt_wrap.pack(fill="x", pady=(8,2))
        ttk.Label(alt_wrap, text=" Altitude", style="Section.TLabel",
                  image=self.icons.get("altitude",16) or self.icons.get("time",16), compound="left").pack(anchor="w")
        self.alt_var = tk.DoubleVar(value=0.0)
        self.alt_pb = ttk.Progressbar(
            alt_wrap,
            variable=self.alt_var,
            maximum=max(1.0, float(self.node.get_parameter('max_altitude').value)),
            style="AltBar.Horizontal.TProgressbar",
            mode="determinate"
        )
        self.alt_pb.pack(fill="x", pady=(6,0))
        self.lbl_alt_text = ttk.Label(alt_wrap, text="Current:  m  (Ground: 0 m  Max: {} m)".format(self.alt_pb["maximum"]), style="Muted.TLabel")
        self.lbl_alt_text.pack(anchor="w", pady=(4,0))

        # bat_wrap = ttk.Frame(telem, style="Card.TFrame"); bat_wrap.pack(fill="x", pady=(8,2))
        # ttk.Label(bat_wrap, text=" Battery", style="Section.TLabel",
        #           image=self.icons.get("battery",16), compound="left").pack(anchor="w")
        # self.bat_pb = ttk.Progressbar(bat_wrap, maximum=100.0); self.bat_pb.pack(fill="x", pady=(6,0))
        # self.lbl_bat_text = ttk.Label(bat_wrap, text="Charge:  %   40.0V     ~  min", style="Muted.TLabel")
        # self.lbl_bat_text.pack(anchor="w", pady=(4,0))

        # p_wrap = ttk.Frame(telem, style="Card.TFrame"); p_wrap.pack(fill="x", pady=(8,2))
        # ttk.Label(p_wrap, text=" Power", style="Section.TLabel",
        #           image=self.icons.get("power",16) or self.icons.get("bolt",16), compound="left").pack(anchor="w")
        # self.lbl_power = ttk.Label(p_wrap, text="Current:   A    Power:   W    Consumed:   mAh", style="Muted.TLabel")
        # self.lbl_power.pack(anchor="w", pady=(4,0))

        # baro_wrap = ttk.Frame(telem, style="Card.TFrame"); baro_wrap.pack(fill="x", pady=(8,2))
        # ttk.Label(baro_wrap, text=" Barometer", style="Section.TLabel",
        #           image=self.icons.get("barometer",16) or self.icons.get("thermo",16), compound="left").pack(anchor="w")
        # self.lbl_baro = ttk.Label(baro_wrap, text="Pressure:   hPa    Temperature:   �C    Humidity:  %", style="Muted.TLabel")
        # self.lbl_baro.pack(anchor="w", pady=(4,0))

        # stat_wrap = ttk.Frame(telem, style="Card.TFrame"); stat_wrap.pack(fill="x", pady=(10, 0))
        # self.stat_card_trees = self._mini_stat(stat_wrap, "Trees Cut", "0", icon=("tree",18))
        # self.stat_card_wp    = self._mini_stat(stat_wrap, "Waypoints", "0/0", icon=("waypoint",18))

        # internals/pollers/bindings (unchanged)
        self._last_photo = None; self._cam_max_w = 1000; self._cam_max_h = 520
        self._last_draw_ts = 0.0; self._max_fps = 20.0
        self._static_bounds = None; self._fixed_view = True; self._fixed_range_m = 8.0
        self._bounds_ema = None; self._bounds_alpha = 0.2

        self.root.after(50, self.poll_img)
        self.root.after(80, self.poll_scan)
        self.root.after(200, self.poll_telemetry)

        self.root.bind("<space>", lambda e: self.on_estop_press())
        self.root.bind("<r>",      lambda e: self.on_estop_reset())

    # ---------- theme / helpers (same as your current version, with icon support) ----------
    def _apply_theme(self):
        style = ttk.Style()
        try: style.theme_use('clam')
        except Exception: pass
        bg = "#F5F7FB"; card = "#FFFFFF"; fg = "#0B1625"; muted = "#6B778C"
        self.root.configure(bg=bg)
        style.configure('.', background=bg, foreground=fg, font=("SF Pro Text", 11))
        style.configure('Bg.TFrame', background=bg)
        style.configure('Topbar.TFrame', background=card)
        style.configure('MutedSmall.TLabel',
    background=card, foreground=muted, font=("SF Pro Text", 9))
        style.configure('Card.TFrame', background=card)
        style.configure('Card.TLabelframe', background=card, relief='solid', borderwidth=1)
        style.configure('Card.TLabelframe.Label', background=card, foreground=muted, font=("SF Pro Text", 10, 'bold'))
        style.configure('TopTitle.TLabel', background=card, foreground=fg, font=("SF Pro Display", 16, 'bold'))
        style.configure('TopMeta.TLabel',  background=card, foreground=muted, font=("SF Pro Text", 10))
        style.configure('BadgeGrey.TLabel', background="#EEF1F6", foreground="#304050", padding=(10,4), font=("SF Pro Text", 10, "bold"))
        style.configure('BadgeRed.TLabel',  background="#FEE2E2", foreground="#9B1C1C", padding=(10,4), font=("SF Pro Text", 10, "bold"))
        style.configure('MetricValue.TLabel', background=card, foreground=fg,    font=("SF Pro Display", 18, "bold"))
        style.configure('MetricSub.TLabel',   background=card, foreground=muted, font=("SF Pro Text", 10))
        style.configure('Section.TLabel', background=card, foreground=fg,    font=("SF Pro Text", 12, "bold"))
        style.configure('Muted.TLabel',   background=card, foreground=muted, font=("SF Pro Text", 10))
        style.configure('CardMutedCenter.TLabel', background=card, foreground=muted, font=("SF Pro Text", 12))
        style.configure('Mono.TLabel', background=card, foreground=fg, font=("SF Mono", 11))
        style.configure("PillOutline.TButton", font=("SF Pro Text", 12, "bold"),
                        padding=(10,10), background=card, foreground="#111")
        style.map("PillOutline.TButton", background=[("active","#F3F4F7")])
        style.configure(
            "AltBar.Horizontal.TProgressbar",
            troughcolor="#EEF1F6",   # track colour
            background="#34C759",    # fill colour (green)
            bordercolor="#EEF1F6",
            lightcolor="#34C759",
            darkcolor="#34C759"
        )
        style.configure("AltBarWarn.Horizontal.TProgressbar", troughcolor="#FDECEC", background="#E11900")
        style.configure("AltBarMid.Horizontal.TProgressbar",  troughcolor="#FFF6E6", background="#F5A623")

        style.configure('TopMeta.TLabel',
            background="#FFFFFF", foreground="#6B778C", font=("SF Pro Text", 10), padding=(10,6))
        style.configure('TopMetaHover.TLabel',
            background="#F3F4F7", foreground="#0B1625", font=("SF Pro Text", 10), padding=(10,6))
       
        style.configure('TopTitleHover.TLabel',
            background="#FFFFFF", foreground="#0B1625", font=("SF Pro Display", 16, "bold"))
        style.configure("BannerBlue.TFrame",   background="#E9F2FF")
        style.configure("BannerBlue.TLabel",   background="#E9F2FF", foreground="#1B4B91", font=("SF Pro Text", 10, "bold"))
        style.configure("BannerPurple.TFrame", background="#F2ECFF")
        style.configure("BannerPurple.TLabel", background="#F2ECFF", foreground="#5A3EAA", font=("SF Pro Text", 10, "bold"))
        style.configure("BannerRed.TFrame",   background="#E9F2FF")
        style.configure("BannerRed.TLabel",   background="#EB7B67", foreground="#FF1E00", font=("SF Pro Text", 10, "bold"))

          
        style.configure("BannerBlue.TFrame",   background="#E9F2FF")
        style.configure("BannerBlue.TLabel",   background="#E9F2FF", foreground="#1B4B91", font=("SF Pro Text", 10, "bold"))

        style.configure("BannerCyan.TFrame",   background="#E6FAFF")
        style.configure("BannerCyan.TLabel",   background="#E6FAFF", foreground="#116B7A", font=("SF Pro Text", 10, "bold"))

        style.configure("BannerPurple.TFrame", background="#F2ECFF")
        style.configure("BannerPurple.TLabel", background="#F2ECFF", foreground="#5A3EAA", font=("SF Pro Text", 10, "bold"))
        style.configure("BannerTeal.TFrame",  background="#E9FFF6")
        style.configure("BannerTeal.TLabel",  background="#E9FFF6", foreground="#156F4B", font=("SF Pro Text", 10, "bold"))


    def _hover_swap(self, widget, normal: str, hover: str, cursor="hand2"):
        widget.bind("<Enter>", lambda e: (widget.configure(style=hover), widget.configure(cursor=cursor)))
        widget.bind("<Leave>", lambda e: (widget.configure(style=normal), widget.configure(cursor="")))



    def _card(self, parent, title, *, row, col, colspan=1, rowspan=1):
        wrap = ttk.Labelframe(parent, text=title, padding=12, style="Card.TLabelframe")
        wrap.grid(row=row, column=col, columnspan=colspan, rowspan=rowspan, sticky="nsew", padx=8, pady=8)
        parent.grid_columnconfigure(col, weight=1); parent.grid_rowconfigure(row, weight=1)
        frame = ttk.Frame(wrap, style="Card.TFrame"); frame.pack(fill="both", expand=True)
        return frame

    def _metric_card(self, parent, title, value, sub, idx, icon=None):
        card = ttk.Labelframe(parent, text=title, padding=(12,8, 6, 0), style="Card.TLabelframe")
        card.grid(row=0, column=idx, sticky="nsew", padx=8, pady=(4,8))
        parent.grid_columnconfigure(idx, weight=1)

        if icon:
            name, size = icon
            ic = self.icons.get(name, size)
            if ic:
                glyph = ttk.Label(card, image=ic, style="Card.TFrame")
                glyph.image = ic
                glyph.place(relx=1.0, x=-10, y=6, anchor="ne")

        # create once; store refs on the frame
        card.val_lbl = ttk.Label(card, text=value, style="MetricValue.TLabel")
        card.sub_lbl = ttk.Label(card, text=sub,   style="MetricSub.TLabel")
        card.val_lbl.pack(anchor="w", pady=(2,0))
        card.sub_lbl.pack(anchor="w", pady=(0, 6))
        return card
    def _add_footer_counter(self, card, var: tk.StringVar):
        lbl = ttk.Label(card, textvariable=var, style="MutedSmall.TLabel")
        lbl.place(relx=1.0, rely=1.0, anchor="se", x=-10, y=-8)  # was y=-8; either is fine now
        card.footer_people_lbl = lbl


    def _add_footer_left(self, card, var: tk.StringVar):
        """Small label in the bottom-left corner of a metric card."""
        lbl = ttk.Label(card, textvariable=var, style="MutedSmall.TLabel")
        lbl.place(relx=0.0, rely=1.0, anchor="sw", x=10, y=-8)  # bottom-left
        card.footer_left_lbl = lbl



    def _pill(self, parent, text, color, cb, payload, outline=False, icon=None):
        if outline:
            btn = ttk.Button(parent, text=text, style="PillOutline.TButton",
                             command=lambda: cb(payload))
            if icon:
                name, size = icon
                ic = self.icons.get(name, size)
                if ic:
                    # ttk.Button supports 'image' too
                    btn.config(image=ic, compound="left")
                    btn.image = ic
            return btn
        b = tk.Button(parent, text=text, font=("SF Pro Text", 12, "bold"),
                      bg=color, fg="white", bd=0, height=2, cursor="hand2",
                      activebackground=color, activeforeground="white",
                      command=lambda: cb(payload))
        if icon:
            name, size = icon
            ic = self.icons.get(name, size)
            if ic:
                b.config(image=ic, compound="left")
                b.image = ic
        return b
    
    def _safe_has(self, name: str) -> bool:
        return hasattr(self, name) and getattr(self, name) is not None

  


    def _mini_stat(self, parent, title, value, icon=None):
        tile = ttk.Frame(parent, style="Card.TFrame")
        tile.pack(side=tk.LEFT, fill="x", expand=True, padx=4)
        name_img = None
        if icon:
            name_img = self.icons.get(icon[0], icon[1])
        ttk.Label(tile, text=(" "+title if name_img else title), style="Muted.TLabel",
                  image=name_img, compound="left").pack(anchor="w")
        lbl = ttk.Label(tile, text=value, style="MetricValue.TLabel"); lbl.pack(anchor="w")
        if name_img: lbl.image = name_img
        return lbl

    # ---- rest of class (poll_img, poll_scan, poll_telemetry, clustering/drawing) stays the same ----


    # ------------------------------ Actions (unchanged) ------------------------------
    def on_estop_press(self):
        self.node.engage_estop()
        self._set_estop_ui(True)

    

    def on_estop_reset(self):
        self.node.reset_estop()
        self._set_estop_ui(False)

  

    def _send_cmd(self, cmd: str):
        try:
            self.node.mission_cmd_pub.publish(String(data=cmd))
            # optimistic local preview
            if   cmd == "start":  self.node.mission_state = "RUNNING"
            elif cmd == "pause":  self.node.mission_state = "PAUSED"
            elif cmd == "resume": self.node.mission_state = "RUNNING"
            elif cmd == "stop":   self.node.mission_state = "STOPPED"
            elif cmd == "rtl":    self.node.mission_state = "RTL"
            elif cmd == "land":   self.node.mission_state = "LAND"
            print(f"[GUI] Sent mission cmd: {cmd}")
        except Exception as e:
            print(f"[GUI] Failed to publish mission cmd: {e}")



    # ------------------------------ Pollers (kept, with extra label updates) ----
    def poll_img(self):
        img = None
        try:
            while True:
                img = self.q_img.get_nowait()
        except queue.Empty:
            pass

        if img is not None:
            self.lbl_cam_info.config(text=f"{img.width}x{img.height}")
            img_disp = img.copy()
            img_disp.thumbnail((self._cam_max_w, self._cam_max_h))
            photo = ImageTk.PhotoImage(image=img_disp)
            self._last_photo = photo
            self.cam_label.configure(image=photo, text="")
        self.root.after(50, self.poll_img)

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
            if pts:
                clusters = self.euclidean_clusters(pts, eps=0.30, min_pts=4)
            self.redraw_scatter(
                pts,
                clusters,
                src=item.get('src',''),
                frame=item.get('frame',''),
                stamp=item.get('stamp',''),
                n=item.get('n_total',''),
                world=item.get('world', True)  # NEW
            )

        self.root.after(40, self.poll_scan)

    def _add_inline_right_of_value(self, card, var: tk.StringVar):
        """Attach a small label just to the right of the big value number."""
        lbl = ttk.Label(card, textvariable=var, style="MutedSmall.TLabel")
        # Place relative to the big value label so it hugs its right edge
        lbl.place(in_=card.val_lbl, relx=1.0, rely=0.55, x=12, anchor="w")
        card.inline_right_lbl = lbl


    def poll_telemetry(self):
        try:
            # Top bar
            bp = self.node.battery_pct
            self.lbl_tel.config(text=f"Telemetry: {bp*100:.0f}%" if bp is not None else "Telemetry: 100%")
            self.lbl_rc.config(text="RC: Strong")
            self.lbl_utc.config(text=datetime.datetime.utcnow().strftime("UTC %H:%M:%S"))

            # --- Metric cards you actually have: tree, audio, speed, home, time, waypts ---

            # Tree count (from PoseArray of detections)
            # --- Trees (prefer count topic, else fallback to positions) ---
            trees_via_topic = getattr(self.node, 'tree_count', None)
            if isinstance(trees_via_topic, (int, float)) and trees_via_topic >= 0:
                self._metric_set(self.card_tree, str(int(trees_via_topic)), "Detected Trees")
            else:
                trees = len(self.node.tree_positions_xy) if getattr(self.node, 'tree_positions_xy', None) else 0
                self._metric_set(self.card_tree, str(trees), "Detected Trees")

            # --- People (prefer count topic, else fallback to any stored positions list if you have one) ---
            people = getattr(self.node, 'people_count', None)
            if people is None:
                people = len(getattr(self.node, 'people_positions_xy', []))  # safe if you don't have it
            people = int(people)

            # --- Stumps (prefer count topic, else fallback to local stumps list) ---
            stumps_via_topic = getattr(self.node, 'stump_count', None)  # create this topic later if you like
            if isinstance(stumps_via_topic, (int, float)) and stumps_via_topic >= 0:
                stumps = int(stumps_via_topic)
            else:
                stumps = len(getattr(self.node, 'stumps', []))

            # Optional: average stump height label if you’re publishing [x,y,r,h] and keeping self.node.stumps
            avg_h = None
            try:
                if stumps and getattr(self.node, 'stumps', None):
                    avg_h = sum(h for (_, _, _, h) in self.node.stumps) / len(self.node.stumps)
            except Exception:
                avg_h = None

            # --- Footer text on the Tree card ---
            footer = f"People: {people}| Stumps: {stumps}"
            if avg_h is not None:
                footer += f"  (avg h≈{avg_h:.2f} m)"
            self.tree_people_var.set(footer)
            legal  = getattr(self.node, 'legal_cuts', 0)
            illegal = getattr(self.node, 'illegal_cuts', 0)
            self.tree_cuts_var.set(f"Legal: {int(legal)} | Illegal: {int(stumps)}")




            # Audio / chainsaw detector
            if (self.node.audio_f0_hz is not None) or (self.node.audio_class is not None):
                f0 = f"{self.node.audio_f0_hz:.0f} Hz" if self.node.audio_f0_hz is not None else "-- Hz"
                cls = (self.node.audio_class or "—")
                sub = f"{cls} ({self.node.audio_conf:.2f})" if (
                    self.node.audio_conf is not None and math.isfinite(self.node.audio_conf)
                ) else cls
                self._metric_set(self.card_audio, f0, sub)
            else:
                self._metric_set(self.card_audio, "-- Hz", "no signal")

            # Speed (breadcrumb-based estimate)
            spd_ms = self._estimate_speed_ms()
            if spd_ms is not None:
                self._metric_set(self.card_speed, f"{spd_ms:.1f} m/s", f"{spd_ms*3.6:.1f} km/h")
            else:
                self._metric_set(self.card_speed, "-- m/s", "-- km/h")

            # Home distance (distance from origin)
            if self.node.position_xy:
                rx, ry = self.node.position_xy
                dist = math.hypot(rx, ry)
                self._metric_set(self.card_home, f"{dist:.0f} m", "Within bounds" if dist < 500 else "Far")
            else:
                self._metric_set(self.card_home, "-- m", "")

            # Flight time
            ft = int(time.time() - getattr(self.node, "_start_time", time.time()))
            self._metric_set(self.card_time, f"{ft//60:02d}:{ft%60:02d}", "Elapsed")

            # Waypoints (from Path/PoseArray + nearest index)
            # Waypoints via mission topics if present, else fall back to local estimate
            if getattr(self.node, "wp_total", 0) > 0:
                self._metric_set(self.card_waypts, f"{self.node.wp_idx}/{self.node.wp_total}", "Completed")
            else:
                wp_total = len(self.node.waypoints_xy) if self.node.waypoints_xy else 0
                wp_done  = min(self.node.next_wp_idx, wp_total)
                self._metric_set(self.card_waypts, f"{wp_done}/{wp_total}", "Completed")



            # --- Odometry / IMU / Altitude panels ---

            # Odometry
            xy = self.node.position_xy or (float('nan'), float('nan'))
            yaw = self.node.yaw_rad
            z = self._select_altitude()
            self.lbl_odo.config(text=(
                f"Position X: {xy[0]:6.2f}  m\n"
                f"Position Y: {xy[1]:6.2f}  m\n"
                f"Position Z: {(z if (z is not None and math.isfinite(z)) else float('nan')):6.2f}  m\n"
                f"Heading:    {(math.degrees(yaw) if yaw is not None else float('nan')):6.1f}"
            ))

                        # --- IMU panel readout ---
            if self.node.imu_rpy:
                r, p, y = self.node.imu_rpy
                # acceleration text
                if (self.node.accel_world is not None) or (self.node.accel_body is not None):
                    Ax, Ay, Az = (self.node.accel_world if self.node._imu_show_world else self.node.accel_body)
                    Amag = self.node.accel_mag if (self.node.accel_mag is not None) else float('nan')
                    frame_tag = "world" if self.node._imu_show_world else "body"
                    gtag = " (gravity-comp)" if self.node._imu_gravity_comp and self.node._imu_show_world else ""
                    self.lbl_imu.config(
                        text=f"Roll: {math.degrees(r):.1f}°   Pitch: {math.degrees(p):.1f}°   Yaw: {math.degrees(y):.1f}°\n"
                            f"a[{frame_tag}]{gtag}:  Ax={Ax:.2f}  Ay={Ay:.2f}  Az={Az:.2f}  |a|={Amag:.2f} m/s²"
                    )
                else:
                    self.lbl_imu.config(
                        text=f"Roll: {math.degrees(r):.1f}°   Pitch: {math.degrees(p):.1f}°   Yaw: {math.degrees(y):.1f}°\n"
                            f"a: — m/s²"
                    )
            else:
                self.lbl_imu.config(text="Roll:    Pitch:    Yaw:    \na: — m/s²")


            # Altitude progress bar
            max_alt = max(1.0, float(self.node.get_parameter('max_altitude').value))
            cur_alt = z if (z is not None and math.isfinite(z)) else 0.0
            self.alt_pb["maximum"] = max_alt
            self.alt_var.set(max(0.0, min(max_alt, cur_alt)))
            self.lbl_alt_text.config(text=f"Current: {cur_alt:.1f} m   Ground: 0 m   Max: {max_alt:.0f} m")
            if cur_alt >= 0.8 * max_alt:
                self.alt_pb.configure(style="AltBarWarn.Horizontal.TProgressbar")
            elif cur_alt >= 0.5 * max_alt:
                self.alt_pb.configure(style="AltBarMid.Horizontal.TProgressbar")
            else:
                self.alt_pb.configure(style="AltBar.Horizontal.TProgressbar")

            # Status line (use your counters if you added them; otherwise simple)
                        # Status line driven by /mission/state (fallbacks for safety)
            state = (self.mission_state or "").upper()
            if not state:
                state = "PAUSED"  # default visual until we hear from main
            self.lbl_status.config(text=f"Status: {state}")


                        # Update chip style by state
            if state == "RUNNING":
                self._mode_chip.config(text="RUNNING", bg="#E9FFF6", fg="#156F4B")
            elif state == "PAUSED":
                self._mode_chip.config(text="PAUSED", bg="#FFF6E6", fg="#9A6B00")
            elif state == "E-STOP":
                self._mode_chip.config(text="E-STOP", bg="#FDECEC", fg="#E11900")
            else:
                self._mode_chip.config(text=state or "PAUSED", bg="#EEF1F6", fg="#304050")


        except Exception as e:
            # keep the loop alive; throttle spam
            if not hasattr(self, "_pt_last_err") or (time.time() - getattr(self, "_pt_last_err", 0) > 2.0):
                print("[poll_telemetry] error:", repr(e))
                self._pt_last_err = time.time()
        finally:
            self.root.after(300, self.poll_telemetry)



    def _set_estop_ui(self, active: bool):
        if active:
            try: self.estop_banner.grid()  # show centered
            except Exception: pass
            self.estop_btn.config(bg="#E07A7A", activebackground="#D86D6D")
        else:
            try: self.estop_banner.grid_remove()
            except Exception: pass
            self.estop_btn.config(bg="#E11900", activebackground="#C41600")

    def _chip(self, parent, text, *, bg="#EEF1F6", fg="#304050", hover_bg=None, side="right", padx=10):
        chip = tk.Label(parent, text=text, bg=bg, fg=fg,
                        font=("SF Pro Text", 9, "bold"), padx=10, pady=3, bd=0, relief="flat")
        chip.pack(side=side, padx=padx, pady=8)
        chip.configure(cursor="hand2")
        if hover_bg:
            chip.bind("<Enter>", lambda e: chip.config(bg=hover_bg))
            chip.bind("<Leave>", lambda e: chip.config(bg=bg))
        return chip

    # ------------------------------ Helpers (unchanged) -------------------------
    def _estimate_speed_ms(self):
        if len(self.node.breadcrumb) < 5:
            return None
        d = 0.0
        pts = self.node.breadcrumb[-5:]
        for i in range(1, len(pts)):
            d += math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1])
        return d / (len(pts)-1) / 0.2

    def _metric_set(self, card, value, subtext):
        try:
            card.val_lbl.config(text=value)
            card.sub_lbl.config(text=subtext)
        except Exception:
            # fallback if an older card was created differently
            for w in card.winfo_children():
                w.destroy()
            ttk.Label(card, text=value, style="MetricValue.TLabel").pack(anchor="w")
            ttk.Label(card, text=subtext, style="MetricSub.TLabel").pack(anchor="w")


    def _select_altitude(self) -> Optional[float]:
        for v in (self.node._altitude_odom, self.node._altitude_gps, self.node._altitude_baro):
            if v is not None and math.isfinite(v):
                return v
        return None

    # ------- drawing & clustering (unchanged from your file) -------
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

    def redraw_scatter(self, pts_xy: List[Tuple[float, float]], clusters: List[List[Tuple[float, float]]], *, src: str, frame: str, stamp: str, n: int, world=True):
        if world and self._fixed_view and (self.node.position_xy is not None):
            x0, x1, y0, y1 = self._compute_bounds([], [])  # will use fixed range around robot
        else:
            x0, x1, y0, y1 = self._compute_bounds(pts_xy, clusters)
                
        
        x0, x1, y0, y1 = self._compute_bounds(pts_xy, clusters)
        if self._static_bounds != (x0, x1, y0, y1):
            self._draw_static_grid(x0, x1, y0, y1)

        self.canvas.delete("dyn")

        w = self.canvas.winfo_width(); h = self.canvas.winfo_height()
        pad = 20
        pt_c = "#2F80ED"
        pal = ["#D64545", "#27AE60", "#2F80ED", "#E2B93B", "#9B59B6", "#30B0C7", "#FF9F0A", "#5856D6"]
        text_c = "#3A3A3C"

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2 * pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2 * pad) / (y1 - y0))
            return X, Y

        if pts_xy:
            r = 2
            for x, y in pts_xy:
                X, Y = to_pix(x, y)
                self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline=pt_c, tags="dyn")

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

        if len(self.node.breadcrumb) >= 2:
            coords = []
            for x, y in self.node.breadcrumb:
                X, Y = to_pix(x, y); coords.extend([X, Y])
            self.canvas.create_line(*coords, fill="#C7C7CC", width=1, tags="dyn")

        if self.node.waypoints_xy:
            for i, (wx, wy) in enumerate(self.node.waypoints_xy):
                X, Y = to_pix(wx, wy)
                r = 3
                col = "#FF9F0A" if i == self.node.next_wp_idx else "#8E8E93"
                self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline=col, width=2, tags="dyn")
            if self.node.position_xy and 0 <= self.node.next_wp_idx < len(self.node.waypoints_xy):
                rx, ry = self.node.position_xy
                wx, wy = self.node.waypoints_xy[self.node.next_wp_idx]
                X1, Y1 = to_pix(rx, ry); X2, Y2 = to_pix(wx, wy)
                self.canvas.create_line(X1, Y1, X2, Y2, dash=(3, 3), fill="#FF9F0A", width=2, tags="dyn")

        for (tx, ty) in self.node.tree_positions_xy:
            X, Y = to_pix(tx, ty)
            r = 4
            self.canvas.create_oval(X - r, Y - r, X + r, Y + r, outline="#34C759", width=2, tags="dyn")

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

        info = f"src {src}  |  frame {frame}  |  t {stamp}  |  pts {n}"
        self.canvas.create_text(pad + 6, pad + 12, text=info, anchor='w', fill=text_c, font=("SF Pro Text", 10), tags="dyn")

        if self.node.estop_active():
            self.canvas.create_text(w/2, h/2, text="E-STOP ACTIVE", fill="#E11900", font=("SF Pro Display", 28, "bold"), tags="dyn")

    def _compute_bounds(self, pts_xy, clusters):
        if self._fixed_view:
            r = self._fixed_range_m
            rx, ry = self.node.position_xy or (0.0, 0.0)
            return rx - r, rx + r, ry - r, ry + r
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
        grid_c = "#E5E5EA"
        axes_c = "#C7C7CC"
        text_c = "#3A3A3C"

        def to_pix(x, y):
            X = pad + (x - x0) * (w - 2 * pad) / (x1 - x0)
            Y = h - (pad + (y - y0) * (h - 2 * pad) / (y1 - y0))
            return X, Y

        self.canvas.create_rectangle(0, 0, w, h, fill="#FAFBFD", outline="", tags="static")

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
        self.canvas.create_text((sx0 + sx1) / 2, sy0 - 10, text="1 m", fill=text_c, font=("SF Pro Text", 10), tags="static")

        self._static_bounds = (x0, x1, y0, y1)

    # ------------------------------ Main loop ------------------------------
    def run(self):
        self.root.mainloop()

# ========================= ICONS =========================
ICONS_DIR = "./icons"  # change if your icons live elsewhere

class IconManager:
    """
    Tiny helper that loads/Resizes PNG icons and keeps PhotoImage refs alive.
    Falls back to None (text-only) if a file is missing.
    """
    def __init__(self, base_dir: str = ICONS_DIR):
        self.base = base_dir
        self.cache = {}

    def get(self, name: str, size: int) -> Optional[ImageTk.PhotoImage]:
        # key by (name, size) so we can reuse objects
        key = (name, size)
        if key in self.cache:
            return self.cache[key]
        try:
            path = os.path.join(self.base, name)
            if not os.path.isfile(path):
                # Try with .png automatically if the user passed bare names
                if not name.lower().endswith(".png"):
                    path = os.path.join(self.base, f"{name}.png")
            img = PILImage.open(path).convert("RGBA")
            img = img.resize((size, size), PILImage.LANCZOS)
            ph = ImageTk.PhotoImage(img)
            self.cache[key] = ph
            return ph
        except Exception:
            return None
# ========================================================

class ScrollableFrame(ttk.Frame):
    """A vertical scrollable area that you can pack/place/grid widgets into via .body."""
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.canvas = tk.Canvas(self, highlightthickness=0, bd=0)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)

        self.vsb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        # inner frame that actually holds widgets
        self.body = ttk.Frame(self.canvas)
        self._window = self.canvas.create_window((0, 0), window=self.body, anchor="nw")

        # keep scrollregion sized to content
        self.body.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # mouse wheel (cross-platform)
        self._bind_mousewheel(self.canvas)

    def _on_canvas_configure(self, event):
        # keep inner frame width equal to canvas width
        self.canvas.itemconfigure(self._window, width=event.width)

    def _bind_mousewheel(self, widget):
        # Windows/macOS
        widget.bind_all("<MouseWheel>", self._on_mousewheel, add="+")
        # Linux/X11
        widget.bind_all("<Button-4>", lambda e: self.canvas.yview_scroll(-1, "units"), add="+")
        widget.bind_all("<Button-5>", lambda e: self.canvas.yview_scroll( 1, "units"), add="+")

    def _on_mousewheel(self, event):
        # On macOS event.delta is small; on Win it's multiples of 120
        delta = -1 if event.delta > 0 else 1
        self.canvas.yview_scroll(delta, "units")


# ------------------------------ main ------------------------------
def ros_spin(node: GuiNode):
    rclpy.spin(node)

def main():
    rclpy.init()
    q_scan, q_img = queue.Queue(), queue.Queue()
    node = GuiNode(q_scan, q_img)
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()
    AppFigma(node, q_scan, q_img).run()

if __name__ == "__main__":
    main()