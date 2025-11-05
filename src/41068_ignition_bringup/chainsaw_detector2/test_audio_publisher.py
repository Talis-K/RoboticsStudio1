#!/usr/bin/env python3
import time, math, random, argparse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RandomAudioPublisher(Node):
    """
    Publishes Float32MultiArray blocks on /microphone/audio (or param topic),
    switching between 'chainsaw', 'ambient', 'speechy', 'bursty', 'silence'
    at a configurable interval. Designed to drive your GUI's embedded detector.
    """

    def __init__(self):
        super().__init__('random_audio_publisher')

        # ---------- Parameters ----------
        self.declare_parameter('topic', '/microphone/audio')
        self.declare_parameter('fs', 16000)                 # sample rate
        self.declare_parameter('block', 2048)               # samples per publish (timer period = block/fs)
        self.declare_parameter('switch_period_s', 5.0)      # how often to pick a new mode
        self.declare_parameter('snr_db', 15.0)              # SNR used when mode == chainsaw
        self.declare_parameter('chainsaw_f0_min', 140.0)    # Hz
        self.declare_parameter('chainsaw_f0_max', 220.0)    # Hz
        self.declare_parameter('seed', 0)                   # RNG seed (0=system time)
        # Mode probabilities (must sum to ~1.0)
        self.declare_parameter('p_chainsaw', 0.45)
        self.declare_parameter('p_ambient',  0.35)
        self.declare_parameter('p_speechy',  0.15)
        self.declare_parameter('p_bursty',   0.04)
        self.declare_parameter('p_silence',  0.01)

        # ---------- Read params ----------
        self.topic  = self.get_parameter('topic').get_parameter_value().string_value
        self.fs     = int(self.get_parameter('fs').value)
        self.block  = int(self.get_parameter('block').value)
        self.period = float(self.get_parameter('switch_period_s').value)
        self.snr_db = float(self.get_parameter('snr_db').value)
        self.f0min  = float(self.get_parameter('chainsaw_f0_min').value)
        self.f0max  = float(self.get_parameter('chainsaw_f0_max').value)
        seed        = int(self.get_parameter('seed').value)
        if seed != 0:
            random.seed(seed); np.random.seed(seed)

        probs = [
            float(self.get_parameter('p_chainsaw').value),
            float(self.get_parameter('p_ambient' ).value),
            float(self.get_parameter('p_speechy' ).value),
            float(self.get_parameter('p_bursty'  ).value),
            float(self.get_parameter('p_silence' ).value),
        ]
        s = sum(probs)
        self.probs = [p/s for p in probs] if s > 0 else [0.45,0.35,0.15,0.04,0.01]
        self.modes = ['chainsaw','ambient','speechy','bursty','silence']

        # ---------- ROS ----------
        self.pub = self.create_publisher(Float32MultiArray, self.topic, 10)

        # switching
        self.t0 = 0  # sample counter
        self.next_switch = time.time()
        self.cur_mode, self.cur_f0 = self._pick_mode()

        # publish timer
        self.timer = self.create_timer(self.block / float(self.fs), self._tick)

        self.get_logger().info(
            f"[random_audio_publisher] topic={self.topic} fs={self.fs} block={self.block} "
            f"switch={self.period}s init_mode={self.cur_mode} f0={self.cur_f0:.1f}Hz"
        )

    # ---------- DSP helpers ----------
    @staticmethod
    def _pink_noise(n):
        # quick 1/f-ish noise
        white = np.random.randn(n).astype(np.float32)
        b = np.array([0.049922, -0.095993, 0.050613, -0.004408], dtype=np.float32)
        a = np.array([1.000000, -2.494956, 2.017265, -0.522189], dtype=np.float32)
        y = np.zeros_like(white)
        for i in range(3, n):
            y[i] = b[0]*white[i] + b[1]*white[i-1] + b[2]*white[i-2] + b[3]*white[i-3] \
                 - a[1]*y[i-1]    - a[2]*y[i-2]    - a[3]*y[i-3]
        m = max(1e-6, np.max(np.abs(y)))
        return (y / m).astype(np.float32)

    def _make_block(self, mode, n, f0, snr_db):
        t = (self.t0 + np.arange(n)) / float(self.fs)
        if mode == 'chainsaw':
            s = (0.6*np.sin(2*np.pi*f0*t) +
                 0.3*np.sin(2*np.pi*2*f0*t) +
                 0.2*np.sin(2*np.pi*3*f0*t)).astype(np.float32)
            x = s + 0.05*np.random.randn(n).astype(np.float32)
            # SNR scaling (optional)
            if snr_db is not None:
                p_sig = float(np.mean(s**2)) + 1e-9
                p_noise = p_sig / (10**(snr_db/10))
                noise = np.random.randn(n).astype(np.float32)
                noise = noise / max(1e-9, np.std(noise)) * math.sqrt(p_noise)
                x = s + noise
        elif mode == 'ambient':
            x = 0.03*self._pink_noise(n)
        elif mode == 'speechy':
            mod = (0.5*(1.0+np.sin(2*np.pi*2.5*t))).astype(np.float32)
            x = (mod * np.random.randn(n).astype(np.float32) * 0.05)
        elif mode == 'bursty':
            x = np.random.randn(n).astype(np.float32) * 0.02
            if int(self.t0 / self.fs) % 2 == 0:
                idx = np.random.randint(0, max(1, n//4))
                L = max(8, n//16)
                x[idx:idx+L] += 0.8*np.hanning(L).astype(np.float32)
        else:  # silence
            x = np.zeros(n, dtype=np.float32)

        m = max(1.0, np.max(np.abs(x)))
        return (x / m).astype(np.float32)

    def _pick_mode(self):
        mode = random.choices(self.modes, weights=self.probs, k=1)[0]
        f0   = random.uniform(self.f0min, self.f0max) if mode == 'chainsaw' else 0.0
        return mode, f0

    # ---------- Timer ----------
    def _tick(self):
        # maybe switch mode
        now = time.time()
        if now >= self.next_switch:
            self.cur_mode, self.cur_f0 = self._pick_mode()
            self.next_switch = now + self.period
            self.get_logger().info(f"[audio] -> mode={self.cur_mode} f0={self.cur_f0:.1f}Hz")

        # publish one block
        block = self._make_block(self.cur_mode, self.block, self.cur_f0, self.snr_db)
        self.t0 += self.block
        self.pub.publish(Float32MultiArray(data=block.tolist()))

def main():
    rclpy.init()
    node = RandomAudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
