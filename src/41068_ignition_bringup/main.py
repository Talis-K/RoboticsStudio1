"""
Before launching this file with:
    python3 src/41068_ignition_bringup/main.py 

In a seperate terminal run:
    git rm -rf build/ install/ log/
    colcon build
    source ~/RoboticsStudio1/install/setup.bash
    ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees

Ensure all lines are run within the /RoboticsStudio1 directory in your bash terminal
"""

import rclpy                                                 # Used for ROS 2 Communication
import threading                                             # Used for Odometry operation
import time                                                  # Used for timers and delays
import numpy as np                                           # Used for Hypotenous and Sign calculations

from drone_control.dronecontrolling import DroneController   # Drone Control Access
from path_planning.snake_path import Goals                   # Initial Waypoint List Access
from drone_control.odometry_listener import OdometryListener # Live Odometry Feed Access


def xy_distance(pose, goal):
    #////////////////////// XY VECTOR CALCULATIONS ////////////////////////
    return np.hypot(pose[0] - goal[0], pose[1] - goal[1])
    #//////////////////////////////////////////////////////////////////////

def wait_motion_finish(controller: DroneController, timeout=30.0):
    #/////////////////// CHECKS, SAFETY AND COMPLETION ////////////////////
    start = time.time() #Records current time to track elapsed duration.
    while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.
    #---------------------------- Node Calls ------------------------------
        rclpy.spin_once(controller, timeout_sec=0)                 #Processes callback for Drone Movement
        rclpy.spin_once(OdometryListener._instance, timeout_sec=0) #Processes callback for Live Odometry
    #----------------------------------------------------------------------
    #---------------------Completion and Safety Check----------------------
        if not controller.motion_queue and controller.timer is None: #Motion is complete when: The motion queue is done and there's no active timer
            return
        if time.time() - start > timeout: #Safety timeout: stops waiting after 30 seconds. Prevents infinite hang if drone gets stuck.
            return
    #----------------------------------------------------------------------
        time.sleep(0.01) # Small delay for computer processing
    #//////////////////////////////////////////////////////////////////////

def move_to(controller: DroneController, target, speed=1.0, tolerance=0.45):
    #////////////////////////// DRONE MOVEMENT ////////////////////////////
    tx, ty = target
    while rclpy.ok(): #Keep trying until goal reached or ROS2 shuts down
    #------------------------ Getting Live Odometry -----------------------
        pose = OdometryListener.update()
        if pose is None:
            time.sleep(0.05) # Small delay for computer processing
            continue
    #----------------------------------------------------------------------
    #------------------------ Live XY Vector to Goal ----------------------
        dist = xy_distance(pose, target)
    #----------------------------------------------------------------------
    #--------------Display information for live terminal updates-----------
        controller.get_logger().info(f" Live Pose: ({pose[0]:.2f}, {pose[1]:.2f}) | Goal Pose: ({tx:.2f}, {ty:.2f})")
    #----------------------------------------------------------------------
    #-----------------Approximately at Goal, stop the drone----------------
        if dist <= tolerance:
            controller.stop()
            return
    #----------------------------------------------------------------------
    #------------------------ X Axis movement first -----------------------
        if abs(pose[0] - tx) >= 0.12: # If Error in X Axis is >= 12 cm
            duration = min(0.5, abs(pose[0] - tx) / speed) # Move at speed for 0.5 seconds or less
            controller.move_x(np.sign(tx - pose[0]) * speed, duration) # Queue a command to move the drone as such
            controller.start() #Start motion
            wait_motion_finish(controller) # Wait for it to finish
            continue # Check Y
    #----------------------------------------------------------------------
    #------------------------- Then Y Axis movement -----------------------
        if abs(pose[1] - ty) >= 0.12: # If Error in Y Axis is >= 12 cm
            duration = min(0.5, abs(pose[1] - ty) / speed) # Move at speed for 0.5 seconds or less
            controller.move_y(np.sign(ty - pose[1]) * speed, duration) # Queue a command to move the drone as such
            controller.start() #Start motion
            wait_motion_finish(controller) # Wait for it to finish
            continue # Loop back around
    #----------------------------------------------------------------------
        time.sleep(0.05) # Small delay for computer processing
    #//////////////////////////////////////////////////////////////////////

def main():
    #/////////////////////////--- SET UP ---///////////////////////////////
    # ----------------- Initialise ROS communication ----------------------
    rclpy.init()
    #----------------------------------------------------------------------
    # -------------------------- Odometry ---------------------------------
    odom = OdometryListener()
    OdometryListener._instance = odom
    threading.Thread(target=rclpy.spin, args=(odom,), daemon=True).start()
    # ---------------------------------------------------------------------
    # ------------------------ X Y Waypoints ------------------------------
    waypoints = [(p[0], p[1]) for p in Goals().position()]
    # ---------------------------------------------------------------------
    # --------------------- Wait for Initial Pose -------------------------
    while OdometryListener.update() is None:
        time.sleep(0.1)
    #----------------------------------------------------------------------
    # ---------------------- Start Drone Control --------------------------
    controller = DroneController()
    #----------------------------------------------------------------------
    #//////////////////////////////////////////////////////////////////////

    #///////////////////////// MOVE TO WAYPOINTS //////////////////////////
    try:
        for i, wp in enumerate(waypoints):
            print(f"Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")
            move_to(controller, wp)
    #//////////////////////////////////////////////////////////////////////

    #/////////////////////// POST GOAL ACHIEVEMENT ////////////////////////
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Drone has finished survey.")
    #//////////////////////////////////////////////////////////////////////


if __name__ == '__main__':
    main()