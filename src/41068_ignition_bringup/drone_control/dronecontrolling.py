from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import rclpy

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("[Drone Control] Drone is ready to be controlled")

        self.timer = None
        self.motion_queue = []
        self.end_time = None

        # --- Altitude oscillation parameters ---
        self.oscillate_thread = None
        self.oscillate_active = threading.Event()
        self.osc_dir = 1.0  # +1 up, -1 down
        self.drone_min_height = 0.4
        self.drone_max_height = 0.6
        self.vert_speed = 0.1  # m/s

    def start_altitude_oscillation(self):
        """Start background thread for vertical oscillation."""
        if self.oscillate_thread and self.oscillate_thread.is_alive():
            self.get_logger().warn("Altitude oscillation already running.")
            return

        self.oscillate_active.set()
        self.oscillate_thread = threading.Thread(target=self._altitude_loop, daemon=True)
        self.oscillate_thread.start()
        self.get_logger().info("[Drone Control] Altitude oscillation started.")

    def stop_altitude_oscillation(self):
        """Stop the oscillation loop."""
        self.oscillate_active.clear()
        self.get_logger().info("[Drone Control] Altitude oscillation stopped.")
        # Send stop command for safety
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)

    def _altitude_loop(self):
        """Runs continuously, flipping climb/descent at bounds."""
        from drone_control.odometry_listener import OdometryListener
        rate = 0.1  # 10 Hz
        while self.oscillate_active.is_set() and rclpy.ok():

            pose = OdometryListener.update()
            if pose is None:
                time.sleep(rate)
                continue

            z = pose[2]
            twist = Twist()

            # Flip direction if bounds exceeded
            if z >= self.drone_max_height:
                self.osc_dir = -1.0
            elif z <= self.drone_min_height:
                self.osc_dir = +1.0

            twist.linear.z = self.vert_speed * self.osc_dir
            self.cmd_pub.publish(twist)

            time.sleep(rate)

    # ---- Movement functions ----
    def move_x(self, speed, duration): #Negative is reverse, positive is forward
        twist = Twist() #Creates an empty Twist movement command to be adapted for the desired movement
        twist.linear.x = speed #Creates command to move the Drone in the linear x direction at the desired speed
        type = "moving Forward/Backwards" #Type of movement for Terminal Printing
        units = "m/s" #Movement units for Terminal Printing
        self.motion_queue.append((twist, duration, speed, type, units)) #Adds this to list of queued movement commands

    def move_y(self, speed, duration): #
        twist = Twist() 
        twist.linear.y = speed #Creates command to move the Drone in the linear y direction at the desired speed
        type = "moving Left/Right"
        units = "m/s"
        self.motion_queue.append((twist, duration, speed, type, units))


    def height(self, speed, duration): #Negative is down, positive is up
        twist = Twist()
        twist.linear.z = speed #Creates command to move the Drone in the linear z direction at the desired speed
        type = "moving Up/Down"
        units = "m/s"
        self.motion_queue.append((twist, duration, speed, type, units))

    def turn(self, angular_speed, duration): #Negative is turn right, positive is turn left
        twist = Twist()
        twist.angular.z = angular_speed
        type = "turning"
        units = "rad/s"
        self.motion_queue.append((twist, duration, angular_speed, type, units))
        

    def stop(self):
        self.get_logger().warn("[Drone Control] Drone has stopped")
        twist = Twist()  # Empty reference to Twst class setting all linear and angular vectors equal to 0
        self.cmd_pub.publish(twist)

    #--------Movement Management and Publishing Functions-------------    
    def start(self):
        """Start up, check que is generated and begin movements"""

        if not self.motion_queue:
            self.get_logger().warn("[Drone Control] No motions queued.")
            return
        
        self.start_motions()

    def start_motions(self):
        """Runs the motions in the queue."""

        if not self.motion_queue:
            self.get_logger().info("[Drone Control] All motions complete.")
            #self.stop() #Stopping drone once movement is complete
            return

        twist, duration, speed, type, units = self.motion_queue.pop(0)
        self.get_logger().info(f"[Drone Control] Drone is {type} at {speed}{units} for {duration}s")

        #Duration Set Up
        now = self.get_clock().now().seconds_nanoseconds()[0]
        self.end_time = now + duration

        # Cancel existing timer, to ensure accurate movement of drone for the current step/movement command
        if self.timer:
            self.timer.cancel()

        # Updating the timer to a new 10 Hz publishing timer with the corresponding twist command
        self.timer = self.create_timer(0.1, lambda: self._timer_callback(twist))

    def _timer_callback(self, twist):
        """Called at 10 Hz while a motion is active."""
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now < self.end_time:
            self.cmd_pub.publish(twist)
        else:
            #self.stop()
            self.timer.cancel()
            self.timer = None
            self.start_motions()   # continue to next command
