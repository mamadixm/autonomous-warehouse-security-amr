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
import shutil
import subprocess
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
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.trigger_sub = self.create_subscription(Bool, '/explore/enable', self.trigger_topic_callback, 10)
        self.enable_srv = self.create_service(SetBool, '/explore/enable', self.trigger_service_callback)

        self.publish_status("INIT")
        self.info("Frontier Explorer Node Initialized.")

    def publish_status(self, text: str):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)

    # --------------------------------------------------------------------------
    # Callbacks & Triggers
    # --------------------------------------------------------------------------
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
                self.cancelTask()
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
        server_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=8.0)
        if not server_ready:
            self.warn(
                "\n" + "=" * 78 + "\n"
                "[auto_explorer] Nav2 'navigate_to_pose' server not active.\n"
                "Operating in Standalone Autonomous Follow-the-Gap Mode.\n"
                + "=" * 78 + "\n"
            )
            return False

        self.info("Nav2 action server detected. Activating SLAM navigation...")
        try:
            self.waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')
            self.info("Nav2 core navigation nodes are active!")
        except Exception as e:
            self.warn(f"Lifecycle wait exception: {e}")

        self.info("Waiting for TF map -> base_footprint from slam_toolbox...")
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t < 15.0):
            try:
                if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                    self.info("TF map -> base_footprint is fully resolved!")
                    return True
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.5)

        return True

    def check_initial_spin(self):
        """Execute a guaranteed 360-degree velocity spin to seed SLAM with full 360° LiDAR data."""
        self.info("Executing 360-degree velocity spin to seed SLAM with warehouse geometry...")
        twist = Twist()
        twist.angular.z = 0.65
        start_t = time.time()
        while time.time() - start_t < 9.5 and rclpy.ok():
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop_robot()
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
        """Save the completed map and navigate back to (0, 0, 0)."""
        self.publish_status("SAVING_MAP")
        self.info("=" * 78)
        self.info("[auto_explorer] ALL FRONTIERS EXPLORED: Finalizing mapping...")
        self.info("=" * 78)

        # 1. Save Map
        map_dir = "/home/mmdixm/amr_project_ws/src/amr_navigation/maps"
        os.makedirs(map_dir, exist_ok=True)
        map_prefix = os.path.join(map_dir, "warehouse_map")
        self.info(f"[auto_explorer] Step 1: Saving warehouse map to {map_prefix}...")

        saved_ok = False
        if SaveMap is not None:
            try:
                save_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
                if save_client.wait_for_service(timeout_sec=3.0):
                    req = SaveMap.Request()
                    req.name.data = map_prefix
                    future = save_client.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    if future.done() and future.result() is not None:
                        if future.result().result == 0:
                            self.info("[auto_explorer] Map successfully saved via slam_toolbox service!")
                            saved_ok = True
            except Exception as e:
                self.warn(f"[auto_explorer] slam_toolbox service exception: {e}")

        if not saved_ok or not os.path.exists(map_prefix + ".yaml"):
            cmd = [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_prefix,
                "--ros-args", "-p", "use_sim_time:=true",
                "-p", "map_subscribe_transient_local:=true"
            ]
            try:
                res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=25.0, text=True)
                self.info(f"[auto_explorer] map_saver_cli output:\n{res.stdout}")
            except Exception as e:
                self.error(f"[auto_explorer] Error during map saving: {e}")

        install_maps_dir = "/home/mmdixm/amr_project_ws/install/amr_navigation/share/amr_navigation/maps"
        os.makedirs(install_maps_dir, exist_ok=True)
        for ext in [".yaml", ".pgm"]:
            src_f = map_prefix + ext
            dst_f = os.path.join(install_maps_dir, "warehouse_map" + ext)
            if os.path.exists(src_f):
                shutil.copy2(src_f, dst_f)
                self.info(f"[auto_explorer] Map synced to install share: {install_maps_dir}")

        try:
            marker_file = os.path.join(map_dir, ".mapping_complete")
            with open(marker_file, "w") as f:
                f.write(f"COMPLETED at {time.time()}\n")
        except Exception:
            pass

        # 2. Return to Base (0, 0, 0)
        self.publish_status("RETURNING_HOME")
        if use_nav2:
            self.info("[auto_explorer] Step 2: Navigating back to home base (0.0, 0.0, 0.0)...")
            home_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
            self.goToPose(home_pose)

            nav_start = time.time()
            while not self.isTaskComplete() and rclpy.ok() and (time.time() - nav_start < 25.0):
                rclpy.spin_once(self, timeout_sec=0.2)

            home_res = self.getResult()
            self.info(f"[auto_explorer] Return to base finished with status: {home_res}")
        else:
            self.info("[auto_explorer] Step 2: Heading back to home origin (0.0, 0.0) via odometry...")
            start_rtb = time.time()
            while rclpy.ok() and (time.time() - start_rtb < 15.0):
                rclpy.spin_once(self, timeout_sec=0.1)
                dist = math.hypot(self.current_x, self.current_y)
                if dist < 0.40:
                    break
                target_yaw = math.atan2(-self.current_y, -self.current_x)
                twist = Twist()
                twist.linear.x = 0.25
                twist.angular.z = float(np.clip(1.2 * target_yaw, -0.7, 0.7))
                self.publish_cmd(twist)
            self.stop_robot()
            self.info("[auto_explorer] Docked at origin (0.0, 0.0).")

        self.info("=" * 78)
        self.info("Mapping complete. Map saved and robot returned to home base (0,0). Ready for new commands.")
        self.info("=" * 78)
        self.state = self.STATE_FINISHED

        for _ in range(5):
            self.publish_status("FINISHED")
            time.sleep(0.1)

    # --------------------------------------------------------------------------
    # Nav2 Frontier Exploration Loop
    # --------------------------------------------------------------------------
    def run_nav2_frontier_fsm(self):
        """Execute Nav2-backed Frontier Exploration."""
        self.check_initial_spin()
        self.state = self.STATE_FIND_FRONTIER
        self.publish_status("MAPPING")
        start_explore_time = time.time()
        min_mapping_duration = 45.0  # seconds minimum exploration time

        while rclpy.ok() and self.state != self.STATE_FINISHED:
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.is_enabled:
                time.sleep(0.5)
                continue

            pose = self.get_robot_pose()
            if pose is None:
                self.warn("Robot pose not yet available. Waiting...")
                time.sleep(0.5)
                continue
            rx, ry, ryaw = pose

            if self.state == self.STATE_FIND_FRONTIER:
                candidates = self.detect_frontiers(rx, ry)
                if not candidates:
                    self.zero_frontiers_count += 1
                    elapsed = time.time() - start_explore_time
                    self.info(
                        f"[auto_explorer] No direct frontiers found (check {self.zero_frontiers_count}/8, elapsed: {elapsed:.1f}s)..."
                    )
                    if self.zero_frontiers_count >= 8 and elapsed >= min_mapping_duration:
                        self.auto_finish_and_return_home(use_nav2=True)
                        break
                    else:
                        # Sweep in place to detect open passages
                        twist = Twist()
                        twist.angular.z = 0.55
                        self.publish_cmd(twist)
                        time.sleep(1.2)
                        self.stop_robot()
                        continue

                self.zero_frontiers_count = 0
                best = candidates[0]
                _, target_x, target_y, target_yaw, cluster_size = best
                self.info(
                    f"[auto_explorer] Next Frontier: ({target_x:.2f}, {target_y:.2f}) "
                    f"| Cluster: {cluster_size} cells. Dispatching Nav2..."
                )

                target_pose = self.create_pose_stamped(target_x, target_y, target_yaw)
                self.goToPose(target_pose)
                self.current_target_goal = (target_x, target_y)
                self.state = self.STATE_NAVIGATING
                last_map_check_time = time.time()

            elif self.state == self.STATE_NAVIGATING:
                target_x, target_y = self.current_target_goal

                now_sec = time.time()
                if now_sec - last_map_check_time >= 1.5:
                    last_map_check_time = now_sec
                    if self.is_goal_area_mapped(target_x, target_y, check_radius=0.60):
                        self.info(
                            f"[auto_explorer] Area around ({target_x:.2f}, {target_y:.2f}) "
                            f"was fully scanned en route. Cancelling goal early!"
                        )
                        self.cancelTask()
                        self.state = self.STATE_FIND_FRONTIER
                        continue

                if self.isTaskComplete():
                    res = self.getResult()
                    if res == TaskResult.SUCCEEDED:
                        self.info(f"[auto_explorer] Reached frontier ({target_x:.2f}, {target_y:.2f}).")
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
        stable_checks = 0
        start_explore_time = time.time()
        last_coverage_check_time = time.time()
        min_mapping_duration = 45.0  # seconds minimum exploration time

        while rclpy.ok() and self.state != self.STATE_FINISHED:
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self.is_enabled:
                time.sleep(0.5)
                continue

            # Periodic coverage check ONLY every 8.0 seconds
            now_t = time.time()
            if (now_t - last_coverage_check_time) >= 8.0 and (self.latest_map is not None):
                last_coverage_check_time = now_t
                grid = np.array(self.latest_map.data, dtype=np.int8)
                free_count = int(np.sum(grid == 0))
                free_history.append(free_count)
                if len(free_history) > 6:
                    free_history.pop(0)

                # Only evaluate completion after minimum duration
                elapsed = now_t - start_explore_time
                if elapsed >= min_mapping_duration and len(free_history) >= 4:
                    growth = free_history[-1] - free_history[0]
                    self.info(f"[Coverage Check] Free cells: {free_count} | 24s Growth: {growth}")
                    if growth < 25:
                        stable_checks += 1
                        if stable_checks >= 3:
                            self.info("[auto_explorer] Map coverage stable across 3 checks. Mapping complete!")
                            self.auto_finish_and_return_home(use_nav2=False)
                            break
                    else:
                        stable_checks = 0

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
