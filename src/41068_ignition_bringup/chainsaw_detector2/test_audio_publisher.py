#!/usr/bin/env python3
import argparse, numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

FS   = 16000          # sample rate
HOPS = 2048           # samples per publish

def pink_noise(n):
    # approximate pink (1/f) noise
    white = np.random.randn(n).astype(np.float32)
    b = np.array([0.049922, -0.095993, 0.050613, -0.004408], dtype=np.float32)
    a = np.array([1.000000, -2.494956, 2.017265, -0.522189], dtype=np.float32)
    y = np.zeros_like(white)
    for i in range(3, n):
        y[i] = b[0]*white[i] + b[1]*white[i-1] + b[2]*white[i-2] + b[3]*white[i-3] \
             - a[1]*y[i-1]    - a[2]*y[i-2]    - a[3]*y[i-3]
    return (y / max(1e-6, np.max(np.abs(y)))).astype(np.float32)

def make_block(mode, t0, n, fs, f0, snr_db):
    t = (t0 + np.arange(n)) / fs
    if mode == 'chainsaw':
        s = (
            0.6*np.sin(2*np.pi*f0*t) +
            0.3*np.sin(2*np.pi*2*f0*t) +
            0.2*np.sin(2*np.pi*3*f0*t)
        ).astype(np.float32)
        noise = 0.05*np.random.randn(n).astype(np.float32)
        x = s + noise
    elif mode == 'ambient':
        x = 0.03*pink_noise(n)  # low level room-ish noise
    elif mode == 'speechy':
        # amplitude-modulated noise ≈ “speech-like”
        mod = (0.5*(1.0+np.sin(2*np.pi*2.5*t))).astype(np.float32)
        x = (mod * np.random.randn(n).astype(np.float32) * 0.05)
    elif mode == 'bursty':
        x = np.random.randn(n).astype(np.float32) * 0.02
        if int(t0) % 2 == 0:          # add bursts every ~1s
            idx = np.random.randint(0, n//4)
            x[idx:idx+n//16] += 0.8*np.hanning(n//16).astype(np.float32)
    elif mode == 'silence':
        x = np.zeros(n, dtype=np.float32)
    else:
        x = 0.03*np.random.randn(n).astype(np.float32)

    # scale to target SNR if requested
    if snr_db is not None and mode == 'chainsaw':
        p_sig = np.mean((x**2))
        if p_sig > 1e-9:
            p_noise = p_sig / (10**(snr_db/10))
            noise   = np.random.randn(n).astype(np.float32)
            noise   = noise / max(1e-9, np.std(noise)) * np.sqrt(p_noise)
            x = x + noise

    # clamp to [-1,1]
    m = max(1.0, np.max(np.abs(x)))
    return (x / m).astype(np.float32)

class TestAudio(Node):
    def __init__(self, topic, mode, f0, snr_db):
        super().__init__('test_audio_publisher')
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)
        self.mode = mode
        self.f0 = float(f0)
        self.snr_db = snr_db
        self.t0 = 0
        self.timer = self.create_timer(HOPS/FS, self.tick)
        self.get_logger().info(f"Publishing mode='{mode}' on {topic} @ {FS}Hz; f0={self.f0}Hz")

    def tick(self):
        block = make_block(self.mode, self.t0, HOPS, FS, self.f0, self.snr_db)
        self.t0 += HOPS
        self.pub.publish(Float32MultiArray(data=block.tolist()))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/microphone/audio')
    ap.add_argument('--mode',  default='chainsaw',
                    choices=['chainsaw','ambient','speechy','bursty','silence'])
    ap.add_argument('--f0', type=float, default=180.0, help='chainsaw base Hz')
    ap.add_argument('--snr', type=float, default=None, help='chainsaw SNR dB (optional)')
    args = ap.parse_args()

    rclpy.init()
    node = TestAudio(args.topic, args.mode, args.f0, args.snr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
