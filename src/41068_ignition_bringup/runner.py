import rclpy
import threading
import time
import numpy as np

from drone_control.dronecontrolling import DroneController
from path_planning.snake_path import Goals
from drone_control.odometry_listener import OdometryListener

def initialise():
    rclpy.init()

    # Spin odom node in background
    odom_node = OdometryListener()
    executor_thread = threading.Thread(target=rclpy.spin, args=(odom_node,), daemon=True)
    executor_thread.start()

    # Wait until odometry data arrives
    while OdometryListener.update() is None:
        time.sleep(0.1)

    current_pose = OdometryListener.update()
    print("Initial pose:", current_pose)

    # Initial waypoints
    snake_goals = Goals().position()
    # print("Goals:", snake_goals)

    # Setup drone control
    drone_cntrl = DroneController()
    return drone_cntrl

def forwards_movement(controller, speed=0.5):
    controller.move(speed, 2)


if __name__ == '__main__':
    drone_cntrl = initialise()

    # Move drone
    forwards_movement(drone_cntrl)

    # Get updated odometry
    current_pose = OdometryListener.update()
    print("Current pose:", current_pose)

    # Shutdown ROS after everything is done
    rclpy.shutdown()
