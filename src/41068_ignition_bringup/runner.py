#!/usr/bin/env python3
# runner.py  –  X/Y‑only, robust, no crash on shutdown

import rclpy
import threading
import time
import numpy as np

from drone_control.dronecontrolling import DroneController
from path_planning.snake_path import Goals
from drone_control.odometry_listener import OdometryListener


# ----------------------------------------------------------------------
# X‑Y Euclidean distance only
# ----------------------------------------------------------------------
def xy_distance(pose, goal):
    return np.hypot(pose[0] - goal[0], pose[1] - goal[1])


# ----------------------------------------------------------------------
# Spin until motion finishes – safe during shutdown
# ----------------------------------------------------------------------
def wait_motion_finish(controller: DroneController, timeout: float = 30.0):
    start = time.time()
    while rclpy.ok():  # ← Critical: respects shutdown
        try:
            rclpy.spin_once(controller, timeout_sec=0.0)
            rclpy.spin_once(OdometryListener._instance, timeout_sec=0.0)
        except rclpy.executors.ExternalShutdownException:
            return  # ROS is shutting down

        if not controller.motion_queue and controller.timer is None:
            return

        if time.time() - start > timeout:
            controller.get_logger().warn("wait_motion_finish() timed out")
            return
        time.sleep(0.01)


# ----------------------------------------------------------------------
# Move to 2D waypoint – one axis at a time
# ----------------------------------------------------------------------
def move_to(controller: DroneController,
            target,
            speed: float = 1.0,
            tolerance: float = 0.25):
    tx, ty = target

    while rclpy.ok():  # ← Also respect shutdown here
        pose = OdometryListener.update()
        if pose is None:
            time.sleep(0.05)
            continue

        dist = xy_distance(pose, target)
        controller.get_logger().info(
            f"| Pose ({pose[0]:.2f},{pose[1]:.2f},{pose[2]:.2f}) | "
            f"Goal ({tx:.2f},{ty:.2f}) | XY-dist {dist:.3f}"
        )

        if dist <= tolerance:
            controller.stop()
            return

        moved = False

        if abs(pose[0] - tx) >= 0.12 and not moved:
            dur = min(0.5, abs(pose[0] - tx) / speed)
            controller.move_x(speed=np.sign(tx - pose[0]) * speed, duration=dur)
            moved = True

        if abs(pose[1] - ty) >= 0.12 and not moved:
            dur = min(0.5, abs(pose[1] - ty) / speed)
            controller.move_y(speed=np.sign(ty - pose[1]) * speed, duration=dur)
            moved = True

        if moved:
            controller.start()
            wait_motion_finish(controller)
        else:
            time.sleep(0.05)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    rclpy.init()

    # --- Odometry in background ---
    odom_node = OdometryListener()
    OdometryListener._instance = odom_node
    odom_thread = threading.Thread(target=rclpy.spin, args=(odom_node,), daemon=True)
    odom_thread.start()

    while OdometryListener.update() is None:
        time.sleep(0.1)

    # --- Controller ---
    controller = DroneController()

    # --- Waypoints (X/Y only) ---
    snake_goals = Goals().position()
    waypoints = [(float(p[0]), float(p[1])) for p in snake_goals]
    print(f"Following {len(waypoints)} X/Y waypoints: {waypoints}")

    # --- Navigate ---
    try:
        for i, wp in enumerate(waypoints):
            print(f"\n=== Waypoint {i+1}/{len(waypoints)} : ({wp[0]:.2f}, {wp[1]:.2f}) ===")
            move_to(controller, target=wp, speed=0.7, tolerance=0.25)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        try:
            controller.stop()
        except:
            pass
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("Mission finished: ROS2 shut down cleanly.")


if __name__ == '__main__':
    main()