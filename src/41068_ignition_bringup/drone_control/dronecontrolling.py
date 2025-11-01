from rclpy.node import Node
from geometry_msgs.msg import Twist

class DroneController(Node): #Inherits from rclpy.node.Node, which provides the full ROS 2 communication interface: publishers, subscribers, timers, parameters, etc
    def __init__(self):
        super().__init__('drone_controller') 
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # creates a ROS publisher that sends linear and angluar velocity commands to the /cmd_vel topic in the ROS environment
        self.get_logger().info("[Drone Control] Drone is ready to be controlled")

        self.timer = None           # active publishing timer
        self.motion_queue = []      # list of (Twist, duration)
        self.end_time = None        # end time for current motion

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
