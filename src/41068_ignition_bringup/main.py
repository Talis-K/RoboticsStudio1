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

import rclpy                                                 # Used for ROS 2 Communication
import threading                                             # Used for Odometry operation
import time
import math                                                  # Used for timers and delays
import numpy as np                                           # Used for Hypotenous and Sign calculations

from drone_control.dronecontrolling import DroneController   # Drone Control Access
from path_planning.snake_path import Goals                   # Initial Waypoint List Access
from drone_control.odometry_listener import OdometryListener # Live Odometry Feed Access


def wait_motion_finish(controller: DroneController, timeout=30.0):
    #/////////////////// CHECKS, SAFETY AND COMPLETION ////////////////////
    start = time.time() #Records current time to track elapsed duration.
    while rclpy.ok(): #Loops only while ROS is running, prevents hanging on shutdown.

    #---------------------Completion and Safety Check----------------------
        if not controller.motion_queue and controller.timer is None: #Motion is complete when: The motion queue is done and there's no active timer
            return
        if time.time() - start > timeout: #Safety timeout: stops waiting after 30 seconds. Prevents infinite hang if drone gets stuck.
            return
    #----------------------------------------------------------------------
        time.sleep(0.01) # Small delay for computer processing
    #//////////////////////////////////////////////////////////////////////

def rotate(controller: DroneController, target, tolerance=0.03):
    """
    Rotate the drone to face the waypoint before approaching.
    """

    while rclpy.ok():
        pose = OdometryListener.update()
        if pose is None:
            time.sleep(0.05)
            continue

        print(f"Current pose: [{pose[0]},{pose[1]}], target pose: [{target[0]},{target[1]}]")

        dx = target[0] - pose[0]
        dy = target[1] - pose[1]

        # Desired yaw direction
        desired_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(desired_yaw - pose[5]), math.cos(desired_yaw - pose[5]))

        if abs(yaw_error) <= tolerance:
            print("[rotate] Orientation aligned.")
            return

        # Proportional angular speed
        angular_speed = max(0.01, min(0.5, abs(yaw_error)))
        duration = abs(yaw_error) / angular_speed

        # Command motion
        controller.turn(math.copysign(angular_speed, yaw_error), duration)
        controller.start()
        wait_motion_finish(controller)


def approach(controller: DroneController, target, tolerance=0.2, speed = 1.0):
    """
    Move the drone toward the target (x, y) coordinate until within tolerance.
    """

    while rclpy.ok():
        pose = OdometryListener.update()
        if pose is None:
            time.sleep(0.05)
            continue

        print(f"Current pose: [{pose[0]},{pose[1]}], target pose: [{target[0]},{target[1]}]")

        dx = target[0] - pose[0]
        dy = target[1] - pose[1]

        # Desired yaw direction
        desired_yaw = math.atan2(dy, dx)
        yaw_error = math.atan2(math.sin(desired_yaw - pose[5]), math.cos(desired_yaw - pose[5]))

        if abs(yaw_error) > 0.03:
            rotate(controller, target)


        distance = math.hypot(dx, dy)
        print(distance)

        if distance <= tolerance:
            print("[approach] Target reached.")
            return True

        # Slow down near goal (min speed 0.1, max 1.0)

        if distance < 1.5:
            speed = distance/8   
        duration = distance / speed /2

        controller.move_x(speed, duration)
        controller.start()
        wait_motion_finish(controller)
        

def main():
    #/////////////////////////--- SET UP ---///////////////////////////////
    # ----------------- Initialise ROS communication ----------------------
    rclpy.init()
    #----------------------------------------------------------------------
    # -------------------------- Nodes ---------------------------------
    odom = OdometryListener()
    OdometryListener._instance = odom
    controller = DroneController()


    # One executor handles both nodes
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(odom)
    executor.add_node(controller)

    # Spin executor in background thread (safe)
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
    # ---------------------- Start Drone Control --------------------------
    #----------------------------------------------------------------------
    #//////////////////////////////////////////////////////////////////////

    #///////////////////////// MOVE TO WAYPOINTS //////////////////////////
    try:
        for i, wp in enumerate(waypoints):
            print(f"Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")
            rotate(controller, wp)
            approach(controller, wp)
    #//////////////////////////////////////////////////////////////////////

    #/////////////////////// POST GOAL ACHIEVEMENT ////////////////////////
    finally:
        if rclpy.ok():
            controller.stop()
            controller.destroy_node()
            odom.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Drone has finished survey.")
    #//////////////////////////////////////////////////////////////////////


if __name__ == '__main__':
    main()