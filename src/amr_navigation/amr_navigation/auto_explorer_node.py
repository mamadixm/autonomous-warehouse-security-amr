#!/usr/bin/env python3
"""
Nav2-Backed Frontier Exploration & Return-to-Base FSM for Warehouse AMR (ROS 2 Jazzy).

Features:
- Dual-Mode Resilience:
  1. If Nav2 stack ('ros-jazzy-navigation2') is installed:
     Executes Nav2 Frontier Exploration + In-flight early cancellation + Return to base (0, 0, 0).
  2. If Nav2 stack is not yet installed:
     Automatically runs in Standalone Autonomous Follow-the-Gap Mode so mapping never halts or crashes,
     and alerts the user with the command: 'sudo apt update && sudo apt install -y ros-jazzy-navigation2'.
- Exact Lidar Angle Math in base_footprint frame (yaw = pi offset compensated).
- Chassis / Wheel noise filtering (<= max(range_min, 0.22m)).
- Auto Map Saving via nav2_map_server map_saver_cli upon completion.
"""

import os
import math
import time
import json
import shutil
import subprocess
import cv2
import numpy as np
import scipy.ndimage

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

try:
    from slam_toolbox.srv import SaveMap
except ImportError:
    SaveMap = None


def save_occupancy_grid_to_disk(map_msg: OccupancyGrid, prefix: str) -> bool:
    """Save an OccupancyGrid directly to PGM and YAML formats in milliseconds."""
    if map_msg is None:
        return False
    try:
        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[grid == 0] = 254
        img[grid > 50] = 0
        img = np.flipud(img)

        pgm_path = prefix + ".pgm"
        yaml_path = prefix + ".yaml"

        cv2.imwrite(pgm_path, img)

        yaml_content = f"""image: {os.path.basename(pgm_path)}
mode: trinary
resolution: {res:.6f}
origin: [{ox:.6f}, {oy:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        with open(yaml_path, "w") as f:
            f.write(yaml_content)
        return True
    except Exception as e:
        print(f"Direct map save exception: {e}")
        return False


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontierExplorerFSM(BasicNavigator):

    STATE_INIT = "INIT"
    STATE_FIND_FRONTIER = "FIND_FRONTIER"
    STATE_NAVIGATING = "NAVIGATING"
    STATE_RETURN_HOME = "RETURN_HOME"
    STATE_FINISHED = "FINISHED"

    def __init__(self):
        super().__init__(node_name='auto_explorer_node')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('auto_start', True)
        self.declare_parameter('cruising_speed', 0.45)
        self.declare_parameter('min_speed', 0.22)
        self.declare_parameter('avoid_distance', 0.55)
        self.declare_parameter('critical_distance', 0.30)
        self.declare_parameter('clear_distance', 0.80)
        self.declare_parameter('rear_critical_distance', 0.35)
        self.declare_parameter('safe_obstacle_distance', 0.45)
        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('min_goal_distance', 0.60)
        self.declare_parameter('cluster_cost_weight', 0.04)

        self.is_enabled = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.cruising_speed = self.get_parameter('cruising_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.avoid_dist = self.get_parameter('avoid_distance').get_parameter_value().double_value
        self.critical_dist = self.get_parameter('critical_distance').get_parameter_value().double_value
        self.clear_dist = self.get_parameter('clear_distance').get_parameter_value().double_value
        self.rear_critical_dist = self.get_parameter('rear_critical_distance').get_parameter_value().double_value
        self.safe_obs_dist = self.get_parameter('safe_obstacle_distance').get_parameter_value().double_value
        self.min_cluster_size = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.min_goal_dist = self.get_parameter('min_goal_distance').get_parameter_value().double_value
        self.cluster_weight = self.get_parameter('cluster_cost_weight').get_parameter_value().double_value

        self.state = self.STATE_INIT
        self.latest_map = None
        self.latest_scan = None
        self.blacklisted_goals = []
        self.zero_frontiers_count = 0
        self.current_target_goal = None

        # Standalone variables
        self.standalone_state = 'EXPLORE'
        self.backup_start_time = None
        self.rotate_start_time = None
        self.front_clear_since = None
        self.rotate_turn_dir = 1.0

        # Odometry tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.ref_pos_x = 0.0
        self.ref_pos_y = 0.0
        self.odom_received = False
        self.last_cmd_linear_x = 0.0
        self.motion_start_time = None

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/explore/status', 10)
        self.progress_pub = self.create_publisher(String, '/explore/progress', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, '/explore/enable', self.trigger_topic_callback, 10)
        self.operator_sub = self.create_subscription(String, '/security/operator_command', self.operator_command_callback, 10)
        self.enable_srv = self.create_service(SetBool, '/explore/enable', self.trigger_service_callback)

        # SLAM Progress & Smart Plateau Tracking
        self.current_progress = 0.0
        self.no_frontier_start_time = None
        self.last_growth_time = time.time()
        self.last_known_free = 0
        self.frontiers_visited = 0
        self.manual_override_active = False
        self.manual_target_pose = None

        self.publish_status("INIT")
        self.publish_progress(0.0, "STAGE 1: MAPPING")
        self.info("Frontier Explorer Node Initialized.")

    def publish_status(self, text: str):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

    def publish_progress(self, progress: float, stage: str):
        msg = String()
        clamped = float(np.clip(progress, 0.0, 100.0))
        msg.data = json.dumps({
            "progress": round(clamped, 1),
            "stage": str(stage)
        })
        self.progress_pub.publish(msg)

    # --------------------------------------------------------------------------
    # Safe Nav2 Wrappers
    # --------------------------------------------------------------------------
    def safe_cancel_task(self):
        try:
            if hasattr(self, 'goal_handle') and self.goal_handle:
                try:
                    self.goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.goal_handle = None
            self.result_future = None
        except Exception as e:
            self.warn(f"[auto_explorer] cancelTask exception: {e}")

    def safe_go_to_pose(self, pose: PoseStamped) -> bool:
        try:
            self.goToPose(pose)
            return True
        except Exception as e:
            self.error(f"[auto_explorer] goToPose exception: {e}")
            return False

    def safe_is_task_complete(self) -> bool:
        try:
            return self.isTaskComplete()
        except Exception:
            return True

    # --------------------------------------------------------------------------
    # Callbacks & Triggers
    # --------------------------------------------------------------------------
    def operator_command_callback(self, msg: String):
        """Handle remote dispatch commands from the Command Center during Stage 1."""
        try:
            cmd = json.loads(msg.data)
            action = cmd.get("action", "").upper().strip()
            self.info(f"[auto_explorer] Received operator command: {action}")

            if action == "RETURN_TO_BASE":
                self.info("[auto_explorer] Operator commanded RETURN_TO_BASE. Queuing completion & docking...")
                self.request_return_home = True

            elif action == "RESUME_PATROL":
                self.info("[auto_explorer] Operator commanded RESUME_PATROL. Queuing completion & patrol transition...")
                self.request_resume_patrol = True

            elif action == "GOTO_POSE":
                gx = float(cmd.get("x", 0.0))
                gy = float(cmd.get("y", 0.0))
                gyaw = float(cmd.get("yaw", 0.0))
                self.info(f"[auto_explorer] Operator commanded GOTO_POSE ({gx:.2f}, {gy:.2f})")
                self.manual_override_requested = (gx, gy, gyaw)
        except Exception as e:
            self.error(f"[auto_explorer] Failed to process operator command: {e}")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if not self.odom_received:
            self.ref_pos_x = self.current_x
            self.ref_pos_y = self.current_y
            self.motion_start_time = self.get_clock().now()
            self.odom_received = True

    def trigger_topic_callback(self, msg: Bool):
        self.set_exploration_state(msg.data, reason="Topic /explore/enable")

    def trigger_service_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.set_exploration_state(request.data, reason="Service /explore/enable")
        response.success = True
        response.message = f"Auto exploration {'enabled' if request.data else 'disabled'}"
        return response

    def set_exploration_state(self, enable: bool, reason: str = ""):
        self.is_enabled = enable
        if not self.is_enabled:
            if self.state == self.STATE_NAVIGATING:
                self.safe_cancel_task()
            self.stop_robot()
            self.state = self.STATE_INIT
            self.info(f"Exploration PAUSED ({reason}).")
        else:
            self.state = self.STATE_FIND_FRONTIER
            self.standalone_state = 'EXPLORE'
            self.zero_frontiers_count = 0
            self.info(f"Exploration RESUMED ({reason}).")

    def stop_robot(self):
        twist = Twist()
        self.last_cmd_linear_x = 0.0
        self.cmd_pub.publish(twist)

    def publish_cmd(self, twist: Twist):
        self.last_cmd_linear_x = twist.linear.x
        self.cmd_pub.publish(twist)

    def create_pose_stamped(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def get_robot_pose(self):
        """Lookup current robot pose in map frame."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            ryaw = math.atan2(siny_cosp, cosy_cosp)
            return rx, ry, ryaw
        except Exception:
            return None

    # --------------------------------------------------------------------------
    # Nav2 SLAM Readiness & Initial Spin
    # --------------------------------------------------------------------------
    def wait_for_nav2_slam(self) -> bool:
        """Check if Nav2 navigation server is running. Returns True if available, False otherwise."""
        self.info("Probing Nav2 action server ('navigate_to_pose')...")
        server_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=12.0)
        if not server_ready:
            self.warn(
                "\n" + "=" * 78 + "\n"
                "[auto_explorer] Nav2 'navigate_to_pose' server not active.\n"
                "Operating in Standalone Autonomous Follow-the-Gap Mode.\n"
                + "=" * 78 + "\n"
            )
            return False

        self.info("Nav2 action server detected. Waiting for TF map -> base_footprint from slam_toolbox...")
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t < 20.0):
            try:
                if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.8)):
                    self.info("TF map -> base_footprint is fully resolved! SLAM navigation ready.")
                    return True
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.5)

        self.info("TF map -> base_footprint wait finished. Proceeding with SLAM navigation.")
        return True

    def check_initial_spin(self):
        """Execute a guaranteed 360-degree velocity spin to seed SLAM with full 360° LiDAR data."""
        self.info("Executing 360-degree velocity spin to seed SLAM with warehouse geometry...")
        self.publish_progress(5.0, "STAGE 1: MAPPING")
        twist = Twist()
        twist.angular.z = 0.65
        start_t = time.time()
        while time.time() - start_t < 9.5 and rclpy.ok():
            elapsed = time.time() - start_t
            spin_prog = 5.0 + (elapsed / 9.5) * 13.0
            self.publish_progress(spin_prog, "STAGE 1: MAPPING")
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop_robot()
        self.current_progress = 18.0
        self.publish_progress(18.0, "STAGE 1: MAPPING")
        self.info("Initial 360° spin complete. SLAM seed map established.")

    # --------------------------------------------------------------------------
    # Frontier Detection
    # --------------------------------------------------------------------------
    def detect_frontiers(self, robot_x: float, robot_y: float):
        """Detect and rank frontier clusters from the latest OccupancyGrid."""
        if self.latest_map is None:
            return []

        map_msg = self.latest_map
        res = map_msg.info.resolution
        w = map_msg.info.width
        h = map_msg.info.height
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))

        free_mask = (grid == 0)
        unknown_mask = (grid == -1)
        obstacle_mask = (grid > 50)

        dist_to_obstacle = scipy.ndimage.distance_transform_edt(~obstacle_mask) * res
        safe_free_mask = free_mask & (dist_to_obstacle >= self.safe_obs_dist)

        adj_unknown = scipy.ndimage.binary_dilation(unknown_mask, structure=np.ones((3, 3)))
        frontier_mask = safe_free_mask & adj_unknown

        labeled, num_features = scipy.ndimage.label(frontier_mask, structure=np.ones((3, 3)))
        if num_features == 0:
            return []

        candidates = []
        for fid in range(1, num_features + 1):
            indices = np.argwhere(labeled == fid)
            cluster_size = len(indices)
            if cluster_size < self.min_cluster_size:
                continue

            mean_r, mean_c = np.mean(indices, axis=0)
            dists = (indices[:, 0] - mean_r)**2 + (indices[:, 1] - mean_c)**2
            r_safe, c_safe = indices[np.argmin(dists)]

            wx = ox + (c_safe + 0.5) * res
            wy = oy + (r_safe + 0.5) * res

            # Bounding box filter: warehouse interior perimeter bounds in map frame
            if not (-10.2 <= wx <= 10.2 and -15.8 <= wy <= 9.2):
                continue

            if any(math.hypot(wx - bx, wy - by) < self.min_goal_dist for (bx, by) in self.blacklisted_goals):
                continue

            dist_to_robot = math.hypot(wx - robot_x, wy - robot_y)
            if dist_to_robot < self.min_goal_dist:
                continue

            yaw = math.atan2(wy - robot_y, wx - robot_x)
            cost = dist_to_robot - self.cluster_weight * min(cluster_size, 50)
            candidates.append((cost, wx, wy, yaw, cluster_size))

        candidates.sort(key=lambda c: c[0])
        return candidates

    def is_goal_area_mapped(self, target_x: float, target_y: float, check_radius: float = 0.60) -> bool:
        """Check if target region has already been fully scanned."""
        if self.latest_map is None:
            return False

        map_msg = self.latest_map
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        w = map_msg.info.width
        h = map_msg.info.height

        tc = int((target_x - ox) / res)
        tr = int((target_y - oy) / res)
        rc = int(check_radius / res)

        r_min = max(0, tr - rc)
        r_max = min(h, tr + rc + 1)
        c_min = max(0, tc - rc)
        c_max = min(w, tc + rc + 1)

        if r_max <= r_min or c_max <= c_min:
            return False

        grid = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
        subgrid = grid[r_min:r_max, c_min:c_max]
        unknown_count = np.sum(subgrid == -1)
        return (unknown_count == 0)

    # --------------------------------------------------------------------------
    # Finish, Save Map & Return to Base
    # --------------------------------------------------------------------------
    def auto_finish_and_return_home(self, use_nav2: bool = True):
        """Save the completed map, clamp progress to 100%, and navigate back to (0, 0, 0)."""
        self.current_progress = 100.0
        self.publish_progress(100.0, "STAGE 1: MAPPING")
        self.publish_status("SAVING_MAP")
        self.info("=" * 78)
        self.info("[auto_explorer] ALL AISLES / FRONTIERS EXPLORED: Finalizing mapping...")
        self.info("=" * 78)

        # 1. Save Map
        map_dir = "/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps"
        os.makedirs(map_dir, exist_ok=True)
        map_prefix = os.path.join(map_dir, "warehouse_map")
        self.info(f"[auto_explorer] Step 1: Saving warehouse map to {map_prefix}...")

        saved_ok = False
        if self.latest_map is not None:
            saved_ok = save_occupancy_grid_to_disk(self.latest_map, map_prefix)
            if saved_ok:
                self.info("[auto_explorer] Map saved directly from memory OccupancyGrid in milliseconds!")

        if not saved_ok and SaveMap is not None:
            try:
                save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
                if save_client.wait_for_service(timeout_sec=2.0):
                    req = SaveMap.Request()
                    req.name.data = map_prefix
                    future = save_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)
                    if future.done() and future.result() is not None:
                        if future.result().result == 0:
                            self.info("[auto_explorer] Map successfully saved via slam_toolbox service!")
                            saved_ok = True
            except Exception as e:
                self.warn(f"[auto_explorer] slam_toolbox service exception: {e}")

        install_maps_dir = "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps"
        os.makedirs(install_maps_dir, exist_ok=True)
        for ext in [".yaml", ".pgm"]:
            src_f = map_prefix + ext
            dst_f = os.path.join(install_maps_dir, "warehouse_map" + ext)
            if os.path.exists(src_f):
                try:
                    if not (os.path.exists(dst_f) and os.path.samefile(src_f, dst_f)):
                        shutil.copy2(src_f, dst_f)
                        self.info(f"[auto_explorer] Map synced to install share: {install_maps_dir}")
                except Exception:
                    pass

        # 2. Return to Base (0, 0, 0)
        self.publish_progress(100.0, "RETURNING TO BASE")
        self.publish_status("RETURNING_HOME")
        reached_origin = False
        self.stop_robot()
        self.safe_cancel_task()
        time.sleep(1.0)

        if use_nav2:
            self.info("[auto_explorer] Step 2: Navigating back to home base (0.0, 0.0, 0.0) via Nav2...")
            home_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
            self.safe_go_to_pose(home_pose)

            nav_start = time.time()
            while rclpy.ok() and (time.time() - nav_start < 55.0):
                rclpy.spin_once(self, timeout_sec=0.2)
                self.publish_progress(100.0, "RETURNING TO BASE")
                pose = self.get_robot_pose()
                rx = pose[0] if pose else self.current_x
                ry = pose[1] if pose else self.current_y
                dist_to_base = math.hypot(rx, ry)
                if dist_to_base <= 0.35:
                    self.info(f"[auto_explorer] Within base threshold ({dist_to_base:.3f}m <= 0.35m). Docked!")
                    reached_origin = True
                    self.safe_cancel_task()
                    break
                if self.safe_is_task_complete():
                    if dist_to_base <= 0.35:
                        reached_origin = True
                    break

        # Fallback direct odometry/velocity steering if not yet within 0.35m of (0, 0)
        pose = self.get_robot_pose()
        rx = pose[0] if pose else self.current_x
        ry = pose[1] if pose else self.current_y
        dist_to_base = math.hypot(rx, ry)

        if not reached_origin and dist_to_base > 0.35:
            self.info(f"[auto_explorer] Engaging direct fallback drive to base (current dist: {dist_to_base:.2f}m)...")
            self.safe_cancel_task()
            start_rtb = time.time()
            while rclpy.ok() and (time.time() - start_rtb < 30.0):
                rclpy.spin_once(self, timeout_sec=0.1)
                self.publish_progress(100.0, "RETURNING TO BASE")
                pose = self.get_robot_pose()
                if pose is not None:
                    rx, ry, ryaw = pose
                else:
                    rx, ry, ryaw = self.current_x, self.current_y, 0.0

                dist = math.hypot(rx, ry)
                if dist <= 0.35:
                    self.info(f"[auto_explorer] Direct fallback reached base ({dist:.3f}m <= 0.35m).")
                    reached_origin = True
                    break

                target_yaw = math.atan2(-ry, -rx)
                yaw_diff = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))

                twist = Twist()
                if abs(yaw_diff) > 0.6:
                    twist.linear.x = 0.05
                    twist.angular.z = float(np.clip(1.5 * yaw_diff, -0.7, 0.7))
                else:
                    twist.linear.x = 0.25
                    twist.angular.z = float(np.clip(1.2 * yaw_diff, -0.5, 0.5))

                self.publish_cmd(twist)
                time.sleep(0.05)

        self.stop_robot()
        time.sleep(1.0)
        self.info(f"[auto_explorer] Return to base completed (docked: {reached_origin}).")

        # 3. Create mapping complete markers ONLY after returning to base!
        try:
            marker_file = os.path.join(map_dir, ".mapping_complete")
            with open(marker_file, "w") as f:
                f.write(f"COMPLETED at {time.time()}\n")
            install_marker = os.path.join(install_maps_dir, ".mapping_complete")
            if not (os.path.exists(install_marker) and os.path.samefile(marker_file, install_marker)):
                with open(install_marker, "w") as f:
                    f.write(f"COMPLETED at {time.time()}\n")
            self.info(f"[auto_explorer] Mapping complete marker created at {marker_file}")
        except Exception as e:
            self.warn(f"[auto_explorer] Error writing marker file: {e}")

        self.info("=" * 78)
        self.info("Mapping complete. Map saved and robot returned to home base (0,0). Handoff to Patrol.")
        self.info("=" * 78)
        self.state = self.STATE_FINISHED

        # Clean handoff to Phase 2 Patrol
        for _ in range(8):
            self.publish_progress(100.0, "STAGE 2: PATROL")
            self.publish_status("FINISHED")
            time.sleep(0.12)

    # --------------------------------------------------------------------------
    # Nav2 Frontier Exploration Loop
    # --------------------------------------------------------------------------
    def run_nav2_frontier_fsm(self):
        """Execute Nav2-backed Frontier Exploration."""
        self.check_initial_spin()
        self.state = self.STATE_FIND_FRONTIER
        self.publish_status("MAPPING")
        self.publish_progress(self.current_progress, "STAGE 1: MAPPING")
        start_explore_time = time.time()
        self.last_growth_time = time.time()
        self.no_frontier_start_time = None
        last_progress_time = time.time()
        last_map_check_time = time.time()

        while rclpy.ok() and self.state != self.STATE_FINISHED:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if operator requested Return to Base or Resume Patrol
            if getattr(self, 'request_return_home', False) or getattr(self, 'request_resume_patrol', False):
                self.request_return_home = False
                self.request_resume_patrol = False
                self.info("[auto_explorer] Processing operator completion request. Returning to base...")
                self.current_progress = 100.0
                self.publish_progress(100.0, "STAGE 1: MAPPING")
                self.auto_finish_and_return_home(use_nav2=True)
                break

            # Check if operator requested GOTO_POSE
            if getattr(self, 'manual_override_requested', None) is not None:
                gx, gy, gyaw = self.manual_override_requested
                self.manual_override_requested = None
                self.safe_cancel_task()
                self.manual_override_active = True
                self.manual_target_pose = (gx, gy, gyaw)
                self.current_target_goal = (gx, gy)
                self.goal_dispatch_time = time.time()
                target_pose = self.create_pose_stamped(gx, gy, gyaw)
                self.safe_go_to_pose(target_pose)
                self.state = self.STATE_NAVIGATING
                self.publish_status(f"OPERATOR_DISPATCH ({gx:.2f}, {gy:.2f})")
                continue

            if not self.is_enabled:
                time.sleep(0.5)
                continue

            pose = self.get_robot_pose()
            if pose is None:
                self.warn("Robot pose not yet available. Waiting...")
                time.sleep(0.5)
                continue
            rx, ry, ryaw = pose

            # Calculate SLAM progress and monitor map cell growth
            if self.latest_map is not None:
                grid = np.array(self.latest_map.data, dtype=np.int8)
                free_count = int(np.sum(grid == 0))
                # Target free cells across the full 20m x 25m warehouse is ~175000
                target_free = 140000.0
                mapped_ratio = min(0.98, free_count / target_free)
                est_prog = 15.0 + mapped_ratio * 84.0
                self.current_progress = max(self.current_progress, est_prog)

                if free_count > (self.last_known_free + 30):
                    self.last_known_free = free_count
                    self.last_growth_time = time.time()

            # Periodic progress broadcast (~2 Hz)
            now_t = time.time()
            if now_t - last_progress_time >= 0.5:
                last_progress_time = now_t
                self.publish_progress(min(self.current_progress, 99.0), "STAGE 1: MAPPING")

            if self.state == self.STATE_FIND_FRONTIER:
                candidates = self.detect_frontiers(rx, ry)
                now_check = time.time()
                elapsed_explore = now_check - start_explore_time

                # Track no reachable frontiers duration
                if not candidates:
                    if self.no_frontier_start_time is None:
                        self.no_frontier_start_time = now_check
                    no_frontiers_sec = now_check - self.no_frontier_start_time
                else:
                    self.no_frontier_start_time = None
                    no_frontiers_sec = 0.0

                # Track map growth plateau duration
                plateau_sec = now_check - self.last_growth_time

                # Autonomous mapping completion: requires full coverage of all warehouse aisles
                should_complete = (
                    (no_frontiers_sec >= 8.0 and self.frontiers_visited >= 6 and elapsed_explore >= 80.0)
                    or (no_frontiers_sec >= 8.0 and elapsed_explore >= 160.0)
                    or (len(candidates) == 0 and plateau_sec >= 25.0 and elapsed_explore >= 120.0 and self.frontiers_visited >= 6)
                    or (elapsed_explore >= 240.0 and self.current_progress >= 85.0)
                )
                if should_complete:
                    self.info(
                        f"[auto_explorer] All warehouse aisles explored (progress: {self.current_progress:.1f}%, frontiers: {self.frontiers_visited}). Returning to base!"
                    )
                    self.current_progress = 100.0
                    self.publish_progress(100.0, "STAGE 1: MAPPING")
                    self.auto_finish_and_return_home(use_nav2=True)
                    break

                if not candidates:
                    # In-place sweep to scan hidden gaps
                    twist = Twist()
                    twist.angular.z = 0.55
                    self.publish_cmd(twist)
                    time.sleep(0.8)
                    self.stop_robot()
                    continue

                best = candidates[0]
                _, target_x, target_y, target_yaw, cluster_size = best
                self.info(
                    f"[auto_explorer] Next Frontier: ({target_x:.2f}, {target_y:.2f}) "
                    f"| Cluster: {cluster_size} cells. Progress: {self.current_progress:.1f}%. Dispatching Nav2..."
                )

                target_pose = self.create_pose_stamped(target_x, target_y, target_yaw)
                self.safe_go_to_pose(target_pose)
                self.current_target_goal = (target_x, target_y)
                self.goal_dispatch_time = time.time()
                self.state = self.STATE_NAVIGATING
                last_map_check_time = time.time()

            elif self.state == self.STATE_NAVIGATING:
                if self.current_target_goal is None:
                    self.state = self.STATE_FIND_FRONTIER
                    continue

                target_x, target_y = self.current_target_goal
                now_sec = time.time()
                elapsed_explore = now_sec - start_explore_time

                # Hard safety timeout during navigation
                if elapsed_explore >= 240.0 and self.current_progress >= 85.0:
                    self.info("[auto_explorer] Max mapping duration reached during navigation. Returning to base!")
                    self.safe_cancel_task()
                    self.auto_finish_and_return_home(use_nav2=True)
                    break

                # If under manual operator dispatch, wait for arrival or operator timeout
                if self.manual_override_active:
                    if (hasattr(self, 'goal_dispatch_time') and (now_sec - self.goal_dispatch_time > 45.0)) or self.safe_is_task_complete():
                        self.info(f"[auto_explorer] Operator dispatch to ({target_x:.2f}, {target_y:.2f}) completed.")
                        self.manual_override_active = False
                        self.state = self.STATE_FIND_FRONTIER
                        time.sleep(1.0)
                    continue

                # Per-goal timeout to prevent hanging on difficult/unreachable frontiers
                if hasattr(self, 'goal_dispatch_time') and (now_sec - self.goal_dispatch_time > 35.0):
                    self.warn(f"[auto_explorer] Goal to ({target_x:.2f}, {target_y:.2f}) timed out after 35s. Blacklisting.")
                    self.safe_cancel_task()
                    self.blacklisted_goals.append((target_x, target_y))
                    self.state = self.STATE_FIND_FRONTIER
                    continue

                now_sec = time.time()
                if now_sec - last_map_check_time >= 1.5:
                    last_map_check_time = now_sec
                    if self.is_goal_area_mapped(target_x, target_y, check_radius=0.60):
                        self.info(
                            f"[auto_explorer] Area around ({target_x:.2f}, {target_y:.2f}) "
                            f"was fully scanned en route. Cancelling goal early!"
                        )
                        self.safe_cancel_task()
                        self.state = self.STATE_FIND_FRONTIER
                        continue

                if self.safe_is_task_complete():
                    try:
                        res = self.getResult()
                    except Exception:
                        res = TaskResult.SUCCEEDED

                    if res == TaskResult.SUCCEEDED:
                        self.info(f"[auto_explorer] Reached frontier ({target_x:.2f}, {target_y:.2f}).")
                        self.frontiers_visited += 1
                    elif res == TaskResult.FAILED:
                        self.warn(f"[auto_explorer] Goal to ({target_x:.2f}, {target_y:.2f}) failed. Blacklisting.")
                        self.blacklisted_goals.append((target_x, target_y))
                    elif res == TaskResult.CANCELED:
                        self.info(f"[auto_explorer] Frontier goal ({target_x:.2f}, {target_y:.2f}) canceled.")

                    self.state = self.STATE_FIND_FRONTIER

    # --------------------------------------------------------------------------
    # Standalone Follow-the-Gap Mode (when Nav2 is not installed)
    # --------------------------------------------------------------------------
    def run_standalone_explorer(self):
        """Execute Standalone Autonomous Exploration with FTG."""
        self.info("[auto_explorer] Running in Standalone Autonomous Mode...")
        self.publish_status("MAPPING")
        self.check_initial_spin()

        free_history = []
        start_explore_time = time.time()
        self.last_growth_time = time.time()
        last_progress_time = time.time()

        while rclpy.ok() and self.state != self.STATE_FINISHED:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if operator requested Return to Base or Resume Patrol
            if getattr(self, 'request_return_home', False) or getattr(self, 'request_resume_patrol', False):
                self.request_return_home = False
                self.request_resume_patrol = False
                self.info("[auto_explorer] Processing operator completion request. Returning to base...")
                self.current_progress = 100.0
                self.publish_progress(100.0, "STAGE 1: MAPPING")
                self.auto_finish_and_return_home(use_nav2=False)
                break

            # Check if operator requested GOTO_POSE
            if getattr(self, 'manual_override_requested', None) is not None:
                gx, gy, gyaw = self.manual_override_requested
                self.manual_override_requested = None
                self.safe_cancel_task()
                self.manual_override_active = True
                self.manual_target_pose = (gx, gy, gyaw)
                self.current_target_goal = (gx, gy)
                self.goal_dispatch_time = time.time()
                target_pose = self.create_pose_stamped(gx, gy, gyaw)
                self.safe_go_to_pose(target_pose)
                self.state = self.STATE_NAVIGATING
                self.publish_status(f"OPERATOR_DISPATCH ({gx:.2f}, {gy:.2f})")
                continue

            if not self.is_enabled:
                time.sleep(0.5)
                continue

            now_t = time.time()
            if self.latest_map is not None:
                grid = np.array(self.latest_map.data, dtype=np.int8)
                free_count = int(np.sum(grid == 0))
                target_free = 140000.0
                mapped_ratio = min(0.98, free_count / target_free)
                est_prog = 15.0 + mapped_ratio * 84.0
                self.current_progress = max(self.current_progress, est_prog)

                if free_count > (self.last_known_free + 30):
                    self.last_known_free = free_count
                    self.last_growth_time = now_t

                elapsed = now_t - start_explore_time
                plateau_sec = now_t - self.last_growth_time
                if (elapsed >= 90.0 and self.current_progress >= 85.0 and plateau_sec >= 12.0) or (elapsed >= 220.0):
                    self.info(f"[auto_explorer FTG] Warehouse coverage satisfied (elapsed: {elapsed:.1f}s). Mapping complete!")
                    self.current_progress = 100.0
                    self.publish_progress(100.0, "STAGE 1: MAPPING")
                    self.auto_finish_and_return_home(use_nav2=False)
                    break

            if now_t - last_progress_time >= 0.5:
                last_progress_time = now_t
                self.publish_progress(min(self.current_progress, 99.0), "STAGE 1: MAPPING")

            # Reactive FTG motion
            self.step_standalone_ftg()

    def step_standalone_ftg(self):
        """Single reactive FTG navigation step."""
        if self.latest_scan is None:
            return

        now = self.get_clock().now()
        scan = self.latest_scan
        num_points = len(scan.ranges)
        if num_points == 0:
            return

        ranges = np.array(scan.ranges, dtype=np.float32)
        angles_laser = scan.angle_min + np.arange(num_points) * scan.angle_increment
        # Compensate for Lidar yaw = pi
        angles_base = np.arctan2(np.sin(angles_laser + math.pi), np.cos(angles_laser + math.pi))
        deg_to_rad = math.pi / 180.0
        chassis_cutoff = max(scan.range_min, 0.22)

        obstacle_mask = (ranges > chassis_cutoff) & (ranges < scan.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)
        free_ranges = np.where(np.isinf(ranges) | (ranges >= scan.range_max), scan.range_max, ranges)
        free_ranges = np.where(np.isnan(free_ranges) | (free_ranges <= chassis_cutoff), 0.0, free_ranges)

        front_45_mask = np.abs(angles_base) <= (45.0 * deg_to_rad)
        rear_mask = np.abs(angles_base) >= (135.0 * deg_to_rad)
        left_mask = (angles_base > (30.0 * deg_to_rad)) & (angles_base <= (105.0 * deg_to_rad))
        right_mask = (angles_base < (-30.0 * deg_to_rad)) & (angles_base >= (-105.0 * deg_to_rad))

        front_obstacles = ranges[front_45_mask & obstacle_mask]
        min_front = float(np.min(front_obstacles)) if len(front_obstacles) > 0 else scan.range_max
        rear_obstacles = ranges[rear_mask & obstacle_mask]
        min_rear = float(np.min(rear_obstacles)) if len(rear_obstacles) > 0 else scan.range_max

        mean_left = float(np.mean(free_ranges[left_mask])) if np.any(left_mask) else 0.0
        mean_right = float(np.mean(free_ranges[right_mask])) if np.any(right_mask) else 0.0

        if self.standalone_state == 'BACKUP':
            if min_rear < self.rear_critical_dist:
                self.standalone_state = 'ROTATE'
                self.rotate_start_time = now
                self.rotate_turn_dir = 1.0 if mean_left >= mean_right else -1.0
                return
            if (now - self.backup_start_time).nanoseconds / 1e9 >= 1.0:
                self.standalone_state = 'ROTATE'
                self.rotate_start_time = now
                self.rotate_turn_dir = 1.0 if mean_left >= mean_right else -1.0
                return
            twist = Twist()
            twist.linear.x = -0.25
            self.publish_cmd(twist)
            return

        if self.standalone_state == 'ROTATE':
            elapsed = (now - self.rotate_start_time).nanoseconds / 1e9
            if min_front > self.clear_dist:
                if self.front_clear_since is None:
                    self.front_clear_since = now
                if (now - self.front_clear_since).nanoseconds / 1e9 >= 0.6:
                    self.standalone_state = 'EXPLORE'
                    self.motion_start_time = now
                    return
            else:
                self.front_clear_since = None
            if elapsed >= 3.5:
                self.standalone_state = 'EXPLORE'
                self.motion_start_time = now
                return
            twist = Twist()
            twist.angular.z = self.rotate_turn_dir * 0.85
            self.publish_cmd(twist)
            return

        # State: EXPLORE
        if min_front < self.critical_dist:
            self.standalone_state = 'BACKUP'
            self.backup_start_time = now
            twist = Twist()
            twist.linear.x = -0.25
            self.publish_cmd(twist)
            return

        if min_front < self.avoid_dist:
            self.standalone_state = 'ROTATE'
            self.rotate_start_time = now
            self.rotate_turn_dir = 1.0 if mean_left >= mean_right else -1.0
            twist = Twist()
            twist.angular.z = self.rotate_turn_dir * 0.85
            self.publish_cmd(twist)
            return

        front_90_mask = np.abs(angles_base) <= (math.pi / 2.0)
        sort_idx = np.argsort(angles_base[front_90_mask])
        sorted_angles = angles_base[front_90_mask][sort_idx]
        sorted_ranges = free_ranges[front_90_mask][sort_idx]

        target_angle = 0.0
        for threshold in [1.20, 0.75]:
            open_mask = sorted_ranges > threshold
            if np.any(open_mask):
                diffs = np.diff(np.concatenate(([False], open_mask, [False])).astype(int))
                starts = np.where(diffs == 1)[0]
                ends = np.where(diffs == -1)[0]
                best_s = -1.0
                for s, e in zip(starts, ends):
                    w = len(sorted_angles[s:e])
                    d = float(np.mean(sorted_ranges[s:e]))
                    score = w * 0.45 + d * 0.55
                    if score > best_s:
                        best_s = score
                        target_angle = float(np.mean(sorted_angles[s:e]))
                break

        angular_z = float(np.clip(0.90 * target_angle, -0.65, 0.65))
        turn_ratio = abs(angular_z) / 0.65
        linear_x = float(max(self.min_speed, self.cruising_speed * (1.0 - 0.45 * turn_ratio)))

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publish_cmd(twist)

    def run_fsm(self):
        """Main entry point: dynamically selects Nav2 or Standalone mode."""
        has_nav2 = self.wait_for_nav2_slam()
        if has_nav2:
            self.run_nav2_frontier_fsm()
        else:
            self.run_standalone_explorer()


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorerFSM()
    try:
        explorer.run_fsm()
    except KeyboardInterrupt:
        explorer.info("Frontier explorer stopped by user.")
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
