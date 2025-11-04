#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

FS = 16000           # sampling rate
HOPS = 2048          # number of samples per publish
FREQ = 180.0         # frequency of the fake "chainsaw" tone

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/microphone/audio', 10)
        self.timer = self.create_timer(HOPS / FS, self.publish_audio)
        self.t = 0
        self.get_logger().info(f"Publishing fake audio on /microphone/audio at {FS} Hz")

    def publish_audio(self):
        n = np.arange(HOPS, dtype=np.float32)
        # Generate a noisy chainsaw-like tone
        s = (
            0.6 * np.sin(2 * np.pi * FREQ * (self.t + n) / FS)
            + 0.3 * np.sin(2 * np.pi * 2 * FREQ * (self.t + n) / FS)
            + 0.1 * np.random.randn(HOPS).astype(np.float32) * 0.05
        )
        msg = Float32MultiArray(data=s.tolist())
        self.pub.publish(msg)
        self.t += HOPS

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
