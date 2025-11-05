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

from rclpy.executors                 import MultiThreadedExecutor # For managing all nodes
from drone_control.dronecontrolling  import DroneController       # Drone Control Access
from path_planning.snake_path        import Goals                 # Initial Waypoint List Access
from drone_control.odometry_listener import OdometryListener      # Live Odometry Feed Access
from lidar_processing.all_lidar_processing import LidarDetection         # For Collision Avoidance


def wait_motion_finish(controller: DroneController, timeout=30.0):
    """
    /////////////////// CHECKS, SAFETY AND COMPLETION ////////////////////
    """
    start = time.time() #Records current time to track elapsed duration.
    while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.

    #---------------------Completion and Safety Check----------------------
        if not controller.motion_queue and controller.timer is None: #Motion is complete when: The motion queue is done and there's no active timer
            return
        if time.time() - start > timeout: #Safety timeout: stops waiting after 30 seconds. Prevents infinite hang if drone gets stuck.
            return
    #----------------------------------------------------------------------
        time.sleep(0.01) # Small delay for computer processing

def rotate(controller: DroneController, pose, dx, dy, tolerance=0.03):
    """
    //////////////// ENSURES CORRECT YAW FOR WAYPOINT GOAL ////////////////
    """

    while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
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
        wait_motion_finish(controller) #Safety check and queue empty check
    #-----------------------------------------------------------------------


def move_to(controller: DroneController, target, tolerance=0.2, speed = 1.0):
    """
    ///////////////// TRANSLATION & ROTATION TO WAYPOINT ///////////////////
    """

    while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
        
        pose = OdometryListener.update() #Updates the live pose of the drone.
        
    #-------------------- Error in x and y planes -------------------------
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
    #----------------------------------------------------------------------
    #------------------ Ensure Correct Yaw Orientation --------------------
        rotate(controller, pose, dx, dy)
    #----------------------------------------------------------------------
        
        distance = np.hypot(dx, dy) #XY Plane distance to target
        if distance <= tolerance: #If within tolerance exit function
            return
        
        if distance < 1.5: #Slow down near target to minimise overshooting
            speed = distance/6 #Making speed proportional to distance when close to target

        duration = distance / speed/2 #Calculating duration so it

        controller.move_x(speed, duration)  #Queue forward motion
        controller.start() #Checking motion is queued and executing turn
        wait_motion_finish(controller) #Safety check and queue empty check
        

def main():
    """
    ////// NODES, THREADING, WAYPOINT & MOVEMENT PROCESSING & SET UP //////
    """
    # ----------------- Initialise ROS communication ----------------------
    rclpy.init() 
    # ---------------------------------------------------------------------
    # ---------------------------- Nodes ----------------------------------
    odom = OdometryListener()
    OdometryListener._instance = odom

    controller = DroneController()

    lidar = LidarDetection()

    #---------------------- Executor for all nodes ------------------------
    executor = MultiThreadedExecutor(num_threads=4) #Runs callbacks in a pool of threads
    #----------------------------------------------------------------------
    # ------------------------- List of Nodes------------------------------
    executor.add_node(odom)
    executor.add_node(controller)
    executor.add_node(lidar)
    #----------------------------------------------------------------------
    #------------- Spin executor in background thread (safe) --------------
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    # ---------------------------------------------------------------------
    # ------------------------ X Y Waypoints ------------------------------
    waypoints = [(p[0], p[1]) for p in Goals().position()]
    # ---------------------------------------------------------------------
    # --------------------- Wait for Initial Pose -------------------------
    while OdometryListener.update() is None:
        time.sleep(0.1)
    #----------------------------------------------------------------------
    #------------------------- Move to waypoints --------------------------
    try:
        for i, wp in enumerate(waypoints):
            print(f"[MAIN] Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")
            move_to(controller, wp)

        print("\n--- Detetced Trees ---")
        for i, tree in enumerate(lidar.Trees):
            print(f"Tree {i}:")
            print(f"  Centroid: {tree['centroid']}")
            print(f"  Radius: {tree['radius']}")
            print(f"  Num points: {len(tree['points'])}")
            print(f"  Scan index: {tree['scan_index']}")

        print("\n--- Detected People ---")
        for i, person in enumerate(lidar.people):
            print(f"Person {i}:")
            print(f"  Centroid: {person['centroid']}")
            print(f"  Radius {person['radius']}")
            print(f"  Num points: {len(person['points'])}")
            print(f"  Scan index: {person['scan_index']}")
        
        print("\n--- object avoidance ---")
        for i, geometry in enumerate(lidar.geometries):
            print(f"Geometry {i}:")
            print(f"  Centroid: {geometry[0]:.2f}, {geometry[1]:.2f}")
            print(f"  Radius {geometry[2]}")
    #----------------------------------------------------------------------
    #--------------------- Post simulation processing ---------------------
    finally:
        if rclpy.ok():
            controller.stop()
            controller.destroy_node()
            odom.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Drone has finished survey.")
    #----------------------------------------------------------------------


if __name__ == '__main__':
    main()