#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Before launching this file run with:
    python3 src/41068_ignition_bringup/main.py 

In a seperate terminal run:
    git rm -rf build/ install/ log/
    colcon build
    source ~/RoboticsStudio1/install/setup.bash
    ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees

Ensure all lines are run within the /RoboticsStudio1 directory in your bash terminal
"""

import rclpy            # Used for ROS 2 Communication
import threading        # Used for Odometry operation
import time             # Used for timers and delays                                
import numpy as np      # Used for Hypotenous, Sin, Cos and Arctan^2 and Sign calculations

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # For managing all nodes

from drone_control.dronecontrolling  import DroneController       # Drone Control Access
from path_planning.snake_path        import Goals                 # Initial Waypoint List Access
from drone_control.odometry_listener import OdometryListener      # Live Odometry Feed Access
from lidar_processing.filtered_lidar import FilteredLidar         # For Collision Avoidance
from lidar_processing.tree_detector  import TreeDetection         # Used for Tree Detection

from std_msgs.msg import Int32, Bool, String, Float32
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, HistoryPolicy

#------ Mission control flags -------
#Module-level so helpers can see them
estop_flag   = threading.Event()
pause_flag   = threading.Event()
started_flag = threading.Event()
stop_flag    = threading.Event()
#------------------------------------

#------ Durable QoS for GUI state/waypoints ---------
transient_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,)
#----------------------------------------------------

class Mission(Node):
    @staticmethod
    def wait_motion_finish(controller: DroneController, timeout: float = 30.0):
        """////////////////// CHECKS, SAFETY AND COMPLETION ////////////////////"""
        start = time.time() #Records current time to track elapsed duration.
        while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
        #---------------------- GUI Pause engaged check -----------------------
            if pause_flag.is_set():
                time.sleep(0.05)
                continue
        #----------------------------------------------------------------------
        #------------------- Emergency Stop engaged check ---------------------
            if estop_flag.is_set():
                try:
                    controller.stop()
                except Exception:
                    pass
                return
        #----------------------------------------------------------------------
        #---------------------Completion and Safety Check----------------------
            if not controller.motion_queue and controller.timer is None: #Motion is complete when: The motion queue is done and there's no active timer
                return
            if time.time() - start > timeout: #Safety timeout: stops waiting after 30 seconds. Prevents infinite hang if drone gets stuck.
                return
        #----------------------------------------------------------------------
            time.sleep(0.01) # Small delay for computer processing

    @staticmethod
    def rotate(controller: DroneController, pose, dx: float, dy: float, tolerance: float = 0.03):
        """Rotate to face the (dx, dy) direction relative to current pose."""
        while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
        #---------------------- GUI Pause engaged check -----------------------
            if pause_flag.is_set():
                time.sleep(0.05)
                continue
        #----------------------------------------------------------------------
        #------------------- Emergency Stop engaged check ---------------------
            if estop_flag.is_set():
                return
        #----------------------------------------------------------------------
            pose = OdometryListener.update() #Updates the live pose of the drone.
        #--------------------- Small Wait if No Pose- -------------------------
            if pose is None:
                time.sleep(0.05)
                continue
        #----------------------------------------------------------------------
        #------------------------ Desired yaw error ---------------------------
            desired_yaw = np.arctan2(dy, dx)
            yaw_error = np.arctan2(np.sin(desired_yaw - pose[5]), np.cos(desired_yaw - pose[5]))
        #----------------------------------------------------------------------
            if abs(yaw_error) <= tolerance: #When the yaw is approximately facing the correct direction exit function.
                return
        #-------------- Calculate the proportional angular speed ---------------
            angular_speed = max(0.01, min(0.5, abs(yaw_error))) #Angular speed can be no smaller then 0.01 rad/s and no larger than 0.5 rad/s and will otherwise be the absolute yaw error
            duration = abs(yaw_error) / angular_speed #The duration needed to reach the desired yaw and the angular speed is calculated 
        #-----------------------------------------------------------------------
        #--------------------- Command and queue motion ------------------------
            controller.turn(np.sign(yaw_error) * angular_speed, duration) #Queuing turn
            controller.start() #Checking motion is queued and executing turn
            Mission.wait_motion_finish(controller) #Safety check and queue empty check
        #-----------------------------------------------------------------------

    @staticmethod
    def move_to(controller: DroneController, target, tolerance: float = 0.2, speed: float = 1.0):
        """Translate toward 'target' in XY, aligning yaw first; obey pause/E-STOP."""
        while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
        #---------------------- GUI Pause engaged check -----------------------
            if pause_flag.is_set():
                time.sleep(0.05)
                continue
        #----------------------------------------------------------------------
        #------------------- Emergency Stop engaged check ---------------------
            if estop_flag.is_set():
                try:
                    controller.stop()
                except Exception:
                    pass
                return
        #----------------------------------------------------------------------
            pose = OdometryListener.update() #Updates the live pose of the drone.
        #--------------------- Small Wait if No Pose- -------------------------
            if pose is None:
                time.sleep(0.05)
                continue
        #----------------------------------------------------------------------
        #-------------------- Error in x and y planes -------------------------
            dx = target[0] - pose[0]
            dy = target[1] - pose[1]
        #----------------------------------------------------------------------
        #------------------ Ensure Correct Yaw Orientation --------------------
            Mission.rotate(controller, pose, dx, dy)
        #----------------------------------------------------------------------
            distance = np.hypot(dx, dy) #XY Plane distance to target
            if distance <= tolerance: #If within tolerance exit function
                return

            if distance < 1.5: #Slow down near target to minimise overshooting
                speed = distance/6 #Making speed proportional to distance when close to target

            duration = distance / speed/2 #Calculating duration so it

            controller.move_x(speed, duration)  #Queue forward motion
            controller.start() #Checking motion is queued and executing turn
            Mission.wait_motion_finish(controller) #Safety check and queue empty check

def main():
    """
    ////// NODES, THREADING, WAYPOINT & MOVEMENT PROCESSING & SET UP //////
    """
    #------------------ Initialise ROS communication ----------------------
    rclpy.init()

    #----------------------------- Nodes ----------------------------------
    odom = OdometryListener()
    OdometryListener._instance = odom

    controller = DroneController()

    filter_node  = FilteredLidar()

    checker_node = TreeDetection()

    #--------------------------- Publishers -------------------------------
    #Durable so GUI gets last values
    pub_wp_total = controller.create_publisher(Int32,  '/mission/waypoint_total', transient_qos)
    pub_wp_idx   = controller.create_publisher(Int32,  '/mission/waypoint_index', transient_qos)
    pub_wp_array = controller.create_publisher(PoseArray, '/mission/waypoints', transient_qos)
    pub_wp_path  = controller.create_publisher(Path, '/mission/waypoints_path', transient_qos)

    for ev in (estop_flag, pause_flag, started_flag, stop_flag):
        ev.clear()

    pub_state = controller.create_publisher(String,  '/mission/state', transient_qos)
    pub_prog  = controller.create_publisher(Float32, '/mission/progress', 10)

    #----------------------- E-STOP subscriber ----------------------------
    def _on_estop(msg: Bool):
        if bool(msg.data):
            estop_flag.set()
            pause_flag.clear()
            pub_state.publish(String(data='E-STOP'))
            try:
                controller.stop()
            except Exception:
                pass
        else:
            estop_flag.clear()
            pub_state.publish(String(data='IDLE'))
    controller.create_subscription(Bool, '/e_stop', _on_estop, 10)
    #----------------------------------------------------------------------

    #------------------ Mission command subscriber ------------------------
    def _on_cmd(msg: String):
        cmd = (msg.data or '').strip().lower()
        if cmd == 'start':
            stop_flag.clear()
            pause_flag.clear()
            started_flag.set()
            pub_state.publish(String(data='RUNNING'))
        elif cmd == 'stop':
            stop_flag.set()
            pause_flag.clear()
            pub_state.publish(String(data='STOPPED'))
            try:
                controller.stop()
            except Exception:
                pass
        elif cmd == 'pause':
            pause_flag.set()
            pub_state.publish(String(data='PAUSED'))
            try:
                controller.stop()
            except Exception:
                pass
        elif cmd == 'resume':
            pause_flag.clear()
            started_flag.set()
            pub_state.publish(String(data='RUNNING'))
        elif cmd == 'rtl':
            pub_state.publish(String(data='RTL'))
        elif cmd == 'land':
            pub_state.publish(String(data='LAND'))
    controller.create_subscription(String, '/mission/cmd', _on_cmd, 10)
    #----------------------------------------------------------------------
    #----------------------- Initial GUI state ----------------------------
    pub_state.publish(String(data='IDLE'))

    #---------------------- Executor for all nodes ------------------------
    executor = MultiThreadedExecutor(num_threads=4) #Runs callbacks in a pool of threads
    #----------------------------------------------------------------------
    # ------------------------- List of Nodes------------------------------
    executor.add_node(odom)
    executor.add_node(controller)
    executor.add_node(filter_node)
    # executor.add_node(checker_node)
    #----------------------------------------------------------------------
    #------------- Spin executor in background thread (safe) --------------
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # ---------------------------------------------------------------------
    # ------------------------ X Y Waypoints ------------------------------
    waypoints = [(p[0], p[1]) for p in Goals().position()]
    # ---------------------------------------------------------------------
    #------------- Publish waypoints as PoseArray + Path ------------------
    wp_array = PoseArray()
    wp_array.header.frame_id = 'odom'
    wp_array.header.stamp = controller.get_clock().now().to_msg()

    wp_path = Path()
    wp_path.header.frame_id = 'odom'
    wp_path.header.stamp = controller.get_clock().now().to_msg()

    for (x, y) in waypoints:
        ps = Pose()
        ps.position.x = float(x)
        ps.position.y = float(y)
        ps.position.z = 0.0
        wp_array.poses.append(ps)

        ps_stamped = PoseStamped()
        ps_stamped.header.frame_id = 'odom'
        ps_stamped.header.stamp = controller.get_clock().now().to_msg()
        ps_stamped.pose = ps
        wp_path.poses.append(ps_stamped)

    pub_wp_array.publish(wp_array)
    pub_wp_path.publish(wp_path)
    pub_wp_total.publish(Int32(data=len(waypoints)))
    #----------------------------------------------------------------------
    # --------------------- Wait for Initial Pose -------------------------
    while OdometryListener.update() is None and rclpy.ok():
        time.sleep(0.1)
    #----------------------------------------------------------------------
    #------------------------- Move to waypoints --------------------------
    try:
        if not started_flag.is_set(): # Check if already started
            pub_state.publish(String(data='IDLE')) # Set state to IDLE
        while rclpy.ok() and (not started_flag.is_set()) and (not stop_flag.is_set()): #Checking safety and start conditions
            time.sleep(0.05)

        total = len(waypoints)
        for i, wp in enumerate(waypoints):
            # Hard stop ends mission
            if stop_flag.is_set():
                break

            # Safety hold: if paused or E-STOP, wait here without advancing i
            while rclpy.ok() and (pause_flag.is_set() or estop_flag.is_set()):
                # ensure motors are stopped while held
                try:
                    controller.stop()
                except Exception:
                    pass
                time.sleep(0.05)

            # After hold is released, require an explicit 'start'/'resume' before continuing
            while rclpy.ok() and (not started_flag.is_set()) and (not stop_flag.is_set()):
                time.sleep(0.05)
            if stop_flag.is_set():
                break


            if total > 0:
                pub_prog.publish(Float32(data=float(i) / float(total)))

            pub_wp_idx.publish(Int32(data=i + 1))
            print(f"[MAIN] Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")
            Mission.move_to(controller, wp)

        if total > 0:
            pub_prog.publish(Float32(data=1.0))
        pub_state.publish(String(data='IDLE' if not estop_flag.is_set() else 'E-STOP'))
    #----------------------------------------------------------------------
    #--------------------- Post simulation processing ---------------------
    finally:
        if rclpy.ok():
            try:
                controller.stop()
            except Exception:
                pass
            controller.destroy_node()
            odom.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Drone has finished survey.")
    #----------------------------------------------------------------------

if __name__ == '__main__':
    main()
