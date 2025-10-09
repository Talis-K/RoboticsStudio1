from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Drone is ready to be controlled")

    # ---- Movement functions ----
    def move(self, speed, duration): #Negative is reverse, positive is forward
        self.get_logger().info(f"Moving {duration}s at {speed} m/s")
        twist = Twist()
        twist.linear.x = speed
        self._publish_for_duration(twist, duration)

    def height(self, speed, duration): #Negative is down, positive is up
        self.get_logger().info(f"Changing height {duration}s at {speed} m/s")
        twist = Twist()
        twist.linear.z = speed
        self._publish_for_duration(twist, duration)

    def turn(self, angular_speed, duration): #Negative is turn right, positive is turn left
        self.get_logger().info(f"Turning {duration}s at {angular_speed} rad/s")
        twist = Twist()
        twist.angular.z = angular_speed
        self._publish_for_duration(twist, duration)
        

    def stop(self):
        self.get_logger().warn("Drone has stopped")
        twist = Twist()  # all zeros
        self.cmd_pub.publish(twist)
        

    # ---- Internal helper ----
    def _publish_for_duration(self, twist, duration):
        """Publish a command at 10Hz for 'duration' seconds."""
        end_time = self.get_clock().now().seconds_nanoseconds()[0] + duration
        while self.get_clock().now().seconds_nanoseconds()[0] < end_time:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.stop()
