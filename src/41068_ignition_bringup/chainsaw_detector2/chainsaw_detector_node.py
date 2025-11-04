#!/usr/bin/env python3
"""
Chainsaw vs Ambient frequency detector (integrated into 41068_ignition_bringup).
Publishes GUI-friendly topics:
  /audio/classification       (std_msgs/String)  "chainsaw" | "ambient"
  /audio/chainsaw_confidence  (std_msgs/Float32) 0..1
  /audio/dominant_frequency   (std_msgs/Float32) Hz
  /audio/psd_band_power       (std_msgs/Float32) 0..1
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
# try:
#     from audio_common_msgs.msg import AudioData
# except Exception:
AudioData = None

# --- drop-in replacement for your ChainsawDetector class ---

import time
from collections import deque

WINDOW_EPS = 1e-12

class ChainsawDetector(Node):
    def __init__(self):
        super().__init__('chainsaw_detector')

        # ---------- Params ----------
        self.declare_parameter('audio_topic', '/audio')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 500)
        self.declare_parameter('hop_ms', 250)
        self.declare_parameter('chainsaw_low_hz', 90.0)
        self.declare_parameter('chainsaw_high_hz', 400.0)
        self.declare_parameter('confidence_thresh', 0.55)
        self.declare_parameter('pcm_width_bits', 16)
        self.declare_parameter('channels', 1)

        # NEW: smoothing + logging
        self.declare_parameter('decision_window', 5)   # number of frames to vote over
        self.declare_parameter('log_period_sec', 1.0)  # throttle console prints

        self.audio_topic = self.get_parameter('audio_topic').get_parameter_value().string_value
        self.fs = int(self.get_parameter('sample_rate').value)
        self.frame_len = int(self.get_parameter('frame_ms').value) * self.fs // 1000
        self.hop_len = int(self.get_parameter('hop_ms').value) * self.fs // 1000
        self.band_lo = float(self.get_parameter('chainsaw_low_hz').value)
        self.band_hi = float(self.get_parameter('chainsaw_high_hz').value)
        self.conf_thresh = float(self.get_parameter('confidence_thresh').value)
        self.pcm_bits = int(self.get_parameter('pcm_width_bits').value)
        self.channels = int(self.get_parameter('channels').value)

        self.decision_window = int(self.get_parameter('decision_window').value)
        self.log_period = float(self.get_parameter('log_period_sec').value)

        # Publishers
        self.pub_class = self.create_publisher(String, '/audio/classification', 10)
        self.pub_conf  = self.create_publisher(Float32, '/audio/chainsaw_confidence', 10)
        self.pub_f0    = self.create_publisher(Float32, '/audio/dominant_frequency', 10)
        self.pub_pwr   = self.create_publisher(Float32, '/audio/psd_band_power', 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriber: AudioData (preferred) or Float32MultiArray fallback
        if AudioData is None:
            from std_msgs.msg import Float32MultiArray
            self._sub = self.create_subscription(Float32MultiArray, self.audio_topic, self._on_audio_float_array, qos)
            self.get_logger().warn('audio_common_msgs not found; listening for Float32MultiArray samples.')
        else:
            self._sub = self.create_subscription(AudioData, self.audio_topic, self._on_audio_data, qos)

        # Buffers
        self.buffer = np.zeros(0, dtype=np.float32)
        self.last_log_t = 0.0
        self.decisions = deque(maxlen=max(1, self.decision_window))  # holds tuples (label, confidence, f0, rel_band_power)

        self.get_logger().info(f"[chainsaw_detector] Listening on {self.audio_topic} (fs={self.fs} Hz, frame={self.frame_len} samples)")

    # ---- Audio callbacks ----
    def _on_audio_float_array(self, msg):
        arr = np.asarray(msg.data, dtype=np.float32).ravel()
        if arr.size:
            self._append_and_process(arr)

    def _on_audio_data(self, msg):
        samples = self._decode_audio(msg.data, self.pcm_bits, self.channels)
        if samples.size:
            self._append_and_process(samples)

    def _decode_audio(self, data_bytes: bytes, bits_per_sample: int, channels: int):
        if bits_per_sample == 16:
            x = np.frombuffer(data_bytes, dtype='<i2')
            if channels > 1:
                x = x.reshape(-1, channels).mean(axis=1)
            return (x.astype(np.float32) / 32768.0)
        elif bits_per_sample == 32:
            x = np.frombuffer(data_bytes, dtype='<f4')
            if channels > 1:
                x = x.reshape(-1, channels).mean(axis=1)
            return x.astype(np.float32)
        else:
            self.get_logger().warn(f'Unsupported PCM width: {bits_per_sample} bits')
            return np.zeros(0, dtype=np.float32)

    def _append_and_process(self, new_samples: np.ndarray):
        self.buffer = np.concatenate([self.buffer, new_samples])
        while self.buffer.size >= self.frame_len:
            frame = self.buffer[:self.frame_len]
            self.buffer = self.buffer[self.hop_len:]
            self._analyze_frame(frame)

    # ---- Core analysis + smoothing + throttled logging ----
    def _analyze_frame(self, x: np.ndarray):
        # FFT
        win = np.hanning(len(x))
        xw = x * win
        spec = np.fft.rfft(xw)
        mag = np.abs(spec) + WINDOW_EPS
        freqs = np.fft.rfftfreq(len(x), d=1.0/self.fs)

        # band
        band_mask = (freqs >= self.band_lo) & (freqs <= self.band_hi)
        if not np.any(band_mask):
            return

        band_mag = mag[band_mask]
        band_freqs = freqs[band_mask]
        peak_idx = int(np.argmax(band_mag))
        f0 = float(band_freqs[peak_idx])
        band_power = float(np.sum(band_mag**2))
        total_power = float(np.sum(mag**2)) + WINDOW_EPS
        rel_band_power = band_power / total_power

        # harmonicity
        max_hz = 2000.0
        kmax = int(max_hz // max(f0, 1.0))
        hvals = []
        bw_hz = max(5.0, f0 * 0.05)
        for k in range(1, max(2, kmax + 1)):
            tgt = k * f0
            if tgt > freqs[-1]:
                break
            mask = (freqs >= tgt - bw_hz) & (freqs <= tgt + bw_hz)
            if np.any(mask):
                hvals.append(np.max(mag[mask]))
        harm_score = float(np.mean(hvals) / (np.mean(mag) + WINDOW_EPS)) if hvals else 0.0

        confidence = float(0.6 * np.clip(rel_band_power * 2.0, 0.0, 1.0) +
                           0.4 * np.clip(harm_score, 0.0, 1.0))
        label = 'chainsaw' if confidence >= self.conf_thresh else 'ambient'

        # ----- NEW: push raw decision to the smoothing queue -----
        self.decisions.append((label, confidence, f0, rel_band_power))

        # Majority vote label over last N frames
        labels = [d[0] for d in self.decisions]
        majority_label = max(set(labels), key=labels.count)

        # Mean confidence/f0/power over the window
        mean_conf = float(np.mean([d[1] for d in self.decisions]))
        mean_f0   = float(np.mean([d[2] for d in self.decisions]))
        mean_pwr  = float(np.mean([d[3] for d in self.decisions]))

        # Publish the smoothed values
        self.pub_class.publish(String(data=majority_label))
        self.pub_conf.publish(Float32(data=mean_conf))
        self.pub_f0.publish(Float32(data=mean_f0))
        self.pub_pwr.publish(Float32(data=mean_pwr))

        # Throttled logging (at most once per log_period seconds)
        now = time.time()
        if now - self.last_log_t >= self.log_period:
            self.get_logger().info(
                f"class={majority_label} conf={mean_conf:.2f} f0={mean_f0:.1f}Hz bandPwr={mean_pwr:.2f}"
            )
            self.last_log_t = now

def main(args=None):
    rclpy.init(args=args)
    node = ChainsawDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



