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
from lidar_processing.all_lidar_processing import LidarDetection  # LiDAR clustering + counts

from std_msgs.msg import Int32, Bool, String, Float32, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import threading
import time

from typing import List, Tuple, Optional
from math import atan2, cos, sin, acos, hypot, isfinite
from threading import Lock


#------ Mission control flags -------
#Module-level so helpers can see them
estop_flag   = threading.Event()
pause_flag   = threading.Event()
started_flag = threading.Event()
stop_flag    = threading.Event()
hold_flag    = threading.Event()      # when set, all motion loops hard-stop & wait
#------------------------------------

#------ Durable QoS for GUI state/waypoints ---------
transient_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,)
#----------------------------------------------------


# --------- Obstacle state (thread-safe) ----------
_obstacles: List[Tuple[float, float, float, float]] = []  # (cx,cy,r,timestamp)
_obst_lock = Lock()

# Tunables for avoidance
DRONE_RADIUS = 0.25     # m (conservative)
SAFETY_MARGIN = 0.25    # m (inflate obstacles by this)
LOOKAHEAD = 9.0         # m (only consider obstacles within this distance from current pose)
LINE_CLEAR_EXTRA = 0.10 # m (require this extra clearance beyond inflated radius)
MAX_SUBGOALS = 3       # avoid infinite detours


def _now_s() -> float:
    return time.time()

def _forward_cone_blocked(pose, obstacles, cone_deg=50.0, stop_dist=3.2) -> bool:
    """
    True if an inflated obstacle center is inside a yaw-centered cone within stop_dist.
    """
    px, py, yaw = float(pose[0]), float(pose[1]), float(pose[5])
    half = float(np.deg2rad(cone_deg) / 2.0)
    for (cx, cy, R) in obstacles:
        dx, dy = cx - px, cy - py
        d = hypot(dx, dy)
        if d <= max(stop_dist, R):  # close enough to worry
            ang = atan2(dy, dx)
            off = abs(np.arctan2(np.sin(ang - yaw), np.cos(ang - yaw)))
            if off <= half:
                return True
    return False

def _min_obst_distance(p, obstacles):
    """Minimum clearance (center distance minus inflated R)."""
    px, py = p
    best = 1e9
    for (cx, cy, R) in obstacles:
        d = hypot(cx - px, cy - py) - R
        if d < best:
            best = d
    return best

def _escape_manoeuvre(controller, pose, *, side='left', lat=0.7, back=0.35, v=0.45):
    """
    Quick 'unstick': lateral sidestep then a tiny reverse to open space.
    Requires move_y() to be implemented (you have it).
    """
    sign = +1.0 if side == 'left' else -1.0
    # 1) lateral nudge
    t_lat = max(0.15, lat / max(v, 1e-3))
    controller.move_y(sign * v, t_lat)
    controller.start()
    Mission.wait_motion_finish(controller)
    # 2) short backtrack
    if back > 0.0:
        t_back = back / max(v, 1e-3)
        controller.move_x(-v, t_back)
        controller.start()
        Mission.wait_motion_finish(controller)

def _snapshot_obstacles() -> List[Tuple[float, float, float]]:
    """Return [(cx,cy,R_inflated), ...] fresh obstacles."""
    with _obst_lock:
        # Keep very recent obstacles (last ~2.0s) to avoid stale ghosts
        fresh = [(cx, cy, r) for (cx, cy, r, ts) in _obstacles if _now_s() - ts <= 2.0]
    # Inflate by drone size + margin
    return [(cx, cy, r + DRONE_RADIUS + SAFETY_MARGIN) for (cx, cy, r) in fresh]

def _dist(a: Tuple[float,float], b: Tuple[float,float]) -> float:
    return hypot(a[0]-b[0], a[1]-b[1])

def _dist_point_to_segment(c: Tuple[float,float],
                           a: Tuple[float,float],
                           b: Tuple[float,float]) -> Tuple[float, float, Tuple[float,float]]:
    """Return (distance, t, closest_point) from point C to segment AB, with t in [0,1]."""
    ax, ay = a; bx, by = b; cx, cy = c
    abx, aby = bx-ax, by-ay
    ab2 = abx*abx + aby*aby
    if ab2 == 0.0:
        return hypot(cx-ax, cy-ay), 0.0, a
    t = ((cx-ax)*abx + (cy-ay)*aby) / ab2
    t = max(0.0, min(1.0, t))
    px, py = ax + t*abx, ay + t*aby
    return hypot(cx-px, cy-py), t, (px, py)

def _choose_tangent(current: Tuple[float,float],
                    goal: Tuple[float,float],
                    center: Tuple[float,float],
                    R: float) -> Tuple[float,float]:
    """
    Compute a good tangent point from current->circle(center,R).
    Chooses left/right tangent based on smaller heading change toward the goal.
    """
    px, py = current
    cx, cy = center
    dx, dy = px - cx, py - cy
    d = hypot(dx, dy)
    if d <= R + 1e-3:
        # We're inside/too close; push straight away from center by R+margin
        ang = atan2(py - cy, px - cx)
        return (cx + (R + 0.40)*cos(ang), cy + (R + 0.40)*sin(ang))

    # Angle from center to current
    base = atan2(dy, dx)
    # Tangent offset
    alpha = acos(max(-1.0, min(1.0, R / d)))

    # Two candidate tangent points
    t1 = (cx + R * cos(base + alpha), cy + R * sin(base + alpha))
    t2 = (cx + R * cos(base - alpha), cy + R * sin(base - alpha))

    # Prefer the one that better aligns with heading to the final goal
    def heading_cost(tp):
        gx, gy = goal
        return abs(atan2(gy - tp[1], gx - tp[0]) - atan2(gy - py, gx - px))

    return t1 if heading_cost(t1) < heading_cost(t2) else t2

def _first_blocking_obstacle(current: Tuple[float,float],
                             goal: Tuple[float,float],
                             obstacles: List[Tuple[float,float,float]]) -> Optional[Tuple[float,float,float]]:
    """Return the first obstacle whose inflated circle blocks line segment current->goal, else None."""
    best = None
    best_t = 1e9
    for (cx, cy, R) in obstacles:
        # Ignore far obstacles (beyond lookahead from current)
        if _dist(current, (cx, cy)) > LOOKAHEAD:
            continue
        dist_to_line, t, _ = _dist_point_to_segment((cx, cy), current, goal)
        if dist_to_line <= (R + LINE_CLEAR_EXTRA) and t < best_t:
            best = (cx, cy, R)
            best_t = t
    return best


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

        #---------------------- Sound HOLD engaged check ----------------------
            if hold_flag.is_set():   # silently hold here while motors are stopped
                try:
                    controller.stop()
                except Exception:
                    pass
                time.sleep(0.05)
                continue
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

        #---------------------- Sound HOLD engaged check ----------------------
            if hold_flag.is_set():   # fully stopped during HOLD
                try:
                    controller.stop()
                except Exception:
                    pass
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

        subgoals_done = 0
        last_brake_ts = 0.0
        brake_cooldown = 0.35      # don’t re-brake instantly
        consec_brakes  = 0         # count repeated brakes at the same spot
        MAX_BRAKES_BEFORE_ESCAPE = 3
        last_progress_check = time.time()
        last_progress_dist  = 1e9  # distance to goal at last check

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

        #-------------------- Sound HOLD engaged check ----------------------
            if hold_flag.is_set():   # fully stopped during HOLD
                try:
                    controller.stop()
                except Exception:
                    pass
                time.sleep(0.05)
                continue
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
            cur = (float(pose[0]), float(pose[1]))
            goal = (float(target[0]), float(target[1]))
            distance = float(np.hypot(goal[0]-cur[0], goal[1]-cur[1]))
            if distance <= max(1e-3, float(tolerance)):
                return

            obs_list = _snapshot_obstacles()
            blocker  = _first_blocking_obstacle(cur, goal, obs_list)



                        # --- if blocked and we still have budget, visit a tangent sub-goal first
            if blocker and subgoals_done < MAX_SUBGOALS:
                try:
                    cx, cy, R = blocker
                    sub = _choose_tangent(cur, goal, (float(cx), float(cy)), float(max(0.0, R)))

                    # Inner loop to reach the sub-goal safely
                    while rclpy.ok():
                        # Respect pause/hold/estop
                        if pause_flag.is_set() or hold_flag.is_set():
                            try: controller.stop()
                            except Exception: pass
                            time.sleep(0.05); continue
                        if estop_flag.is_set():
                            try: controller.stop()
                            except Exception: pass
                            return

                        pose2 = OdometryListener.update()
                        if pose2 is None:
                            time.sleep(0.05); continue

                        cur2 = (float(pose2[0]), float(pose2[1]))
                        dsub = float(np.hypot(sub[0]-cur2[0], sub[1]-cur2[1]))
                        if dsub <= (tolerance + 0.10):
                            break

                        # Align then small straight nudge toward sub-goal
                        Mission.rotate(controller, pose2, sub[0]-cur2[0], sub[1]-cur2[1])
                        v = min(speed, max(0.15, dsub/6.0))      # gentle near subgoal
                        duration = max(0.05, (dsub / max(v,1e-3)) / 2.0)
                        # Emergency forward cone brake
                        if _forward_cone_blocked(pose if 'pose2' not in locals() else pose2, obs_list, cone_deg=50.0, stop_dist=1.2):
                            try: controller.stop()
                            except Exception: pass
                            time.sleep(0.1)
                            continue  # re-evaluate instead of committing to motion

                        controller.move_x(v, duration)
                        controller.start()
                        Mission.wait_motion_finish(controller)

                    subgoals_done += 1
                    # After subgoal, loop continues to pursue the original goal
                    continue
                except Exception:
                    # If detour math fails, fall through and try straight motion
                    pass


            # try:
            #     
            # except Exception:
            #     blocker = None  # if anything goes wrong, just fly straight

            

            v = float(speed)
            if distance < 1.5:
                v = max(0.15, distance/6.0)              # slow near goal
            duration = max(0.05, (distance / max(v,1e-3)) / 2.0)  # clamp
            # Emergency forward cone brake
            if _forward_cone_blocked(pose if 'pose2' not in locals() else pose2, obs_list, cone_deg=50.0, stop_dist=1.2):
                try: controller.stop()
                except Exception: pass
                time.sleep(0.1)
                continue  # re-evaluate instead of committing to motion


            controller.move_x(v, duration)
            controller.start()
            Mission.wait_motion_finish(controller)

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

    lidar_node = LidarDetection()

    #--------------------------- Publishers -------------------------------
    #Durable so GUI gets last values
    pub_wp_total = controller.create_publisher(Int32,  '/mission/waypoint_total', transient_qos)
    pub_wp_idx   = controller.create_publisher(Int32,  '/mission/waypoint_index', transient_qos)
    pub_wp_array = controller.create_publisher(PoseArray, '/mission/waypoints', transient_qos)
    pub_wp_path  = controller.create_publisher(Path, '/mission/waypoints_path', transient_qos)

    pub_tree_count   = controller.create_publisher(Int32, '/mission/tree_count', transient_qos)
    pub_people_count = controller.create_publisher(Int32, '/mission/people_count', transient_qos)
    pub_stump_count = controller.create_publisher(Int32, '/mission/stump_count', transient_qos)

    # geometry can be live (no need to be transient)
    pub_avoid_geom   = controller.create_publisher(Float32MultiArray, '/mission/avoidance_object', 10)


    for ev in (estop_flag, pause_flag, started_flag, stop_flag):
        ev.clear()

    pub_state = controller.create_publisher(String,  '/mission/state', transient_qos)
    pub_prog  = controller.create_publisher(Float32, '/mission/progress', 10)


        # --- Sound-triggered full-stop (no GUI pause) ---
    _hold_lock = threading.Lock()      # prevent overlapping holds
    _hold_timer = None                 # threading.Timer for auto-resume
    _last_hold_ts = 0.0                # debounce timestamp (epoch)
    _hold_cooldown = 3.0               # ignore re-triggers within 3 s
    _auto_hold_secs = 5.0              # stop duration on sound


    

    def _trigger_timed_hold(seconds: float, reason: str = "sound"):
        """ Immediately stop motors, set an internal hold for `seconds`, then auto-resume."""
        nonlocal _hold_timer, _last_hold_ts
        with _hold_lock:
            now = time.time()
            if now - _last_hold_ts < _hold_cooldown:
                return  # debounce multiple detections
            _last_hold_ts = now

            # Engage hold
            hold_flag.set()                                # (loops will respect this)
            try:
                controller.stop()                          # publish zero Twist right away
            except Exception:
                pass

            # Reset/arm the one-shot resume timer
            if _hold_timer is not None:
                try:
                    _hold_timer.cancel()
                except Exception:
                    pass
            _hold_timer = threading.Timer(seconds, _clear_hold)
            _hold_timer.daemon = True
            _hold_timer.start()

    def _clear_hold():
        """ Clear hold and let loops proceed (if not E-STOP)."""
        nonlocal _hold_timer
        with _hold_lock:
            _hold_timer = None
            if estop_flag.is_set():
                # If an E-STOP happened during hold, remain stopped.
                return
            hold_flag.clear()          # loops will continue on next iteration


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

        # ---------------- Audio / Frequency detection subscribers --------------
    # String classifier (e.g., "class=chainsaw conf=0.87 ...")
    def _on_chainsaw_status(msg: String):
        s = (msg.data or '').lower()
        if 'chainsaw' in s:
            _trigger_timed_hold(_auto_hold_secs, reason='sound')

    controller.create_subscription(String, '/audio/chainsaw/status', _on_chainsaw_status, 10)

    # Simple boolean trigger (True => detected)
    def _on_chainsaw_bool(msg: Bool):
        if bool(msg.data):
            _trigger_timed_hold(_auto_hold_secs, reason='sound')

    controller.create_subscription(Bool, '/chainsaw_detected', _on_chainsaw_bool, 10)
    # ----------------------------------------------------------------------


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

    def _on_tree_count(msg: Int32):
        pub_tree_count.publish(Int32(data=msg.data))
    controller.create_subscription(Int32, '/tree_count', _on_tree_count, 10)

    def _on_people_count(msg: Int32):
        pub_people_count.publish(Int32(data=msg.data))
    controller.create_subscription(Int32, '/people_count', _on_people_count, 10)

    def _on_stump_count(msg: Int32):
         pub_stump_count.publish(Int32(data=msg.data))
    controller.create_subscription(Int32, '/stump_count', _on_stump_count, 10)



    def _on_obj_geom(msg: Float32MultiArray):
        # Each message is [cx, cy, r]; forward to GUI/live avoidance topic
        pub_avoid_geom.publish(msg)

        # keep a fresh, de-bounced obstacle list
        data = list(msg.data or [])
        if len(data) >= 3:
            cx, cy, r = float(data[0]), float(data[1]), float(data[2])
            if all(map(isfinite, (cx, cy, r))):
                with _obst_lock:
                    _obstacles.append((cx, cy, r, _now_s()))
                    if len(_obstacles) > 256:
                        del _obstacles[:128]

    controller.create_subscription(Float32MultiArray, '/obj_geometry', _on_obj_geom, 10)

    



    #----------------------------------------------------------------------
    #----------------------- Initial GUI state ----------------------------
    pub_state.publish(String(data='IDLE'))

    #---------------------- Executor for all nodes ------------------------
    executor = MultiThreadedExecutor(num_threads=4) #Runs callbacks in a pool of threads
    #----------------------------------------------------------------------
    # ------------------------- List of Nodes------------------------------
    executor.add_node(odom)
    executor.add_node(controller)
    executor.add_node(lidar_node)
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
            while rclpy.ok() and (pause_flag.is_set() or estop_flag.is_set() or hold_flag.is_set()):
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
            try:
                lidar_node.destroy_node()
            except Exception:
                pass
            controller.destroy_node()
            odom.destroy_node()
        rclpy.shutdown()
        print("[MAIN] Drone has finished survey.")
    #----------------------------------------------------------------------

if __name__ == '__main__':
    main()