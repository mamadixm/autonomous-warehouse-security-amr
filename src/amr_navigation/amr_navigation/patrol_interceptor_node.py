#!/usr/bin/env python3
"""
Warehouse Security Guard FSM (patrol_interceptor_node.py)
Phase 2 Upgrade:
  - 360° Dual-Camera Security AI (Front & Rear monitoring)
  - Time-Based Security Policy & Virtual RFID Whitelist
  - Autonomous 180° Spin Interception on Rear Threats
  - SQLite Incident & Forensic Snapshot Logging
  - Real-Time JSON Telemetry (/security/telemetry) & Alert Stream (/security/alerts)
  - Remote Command Center Dispatch Interface (/security/operator_command)
"""

import os
import math
import time
import json
from datetime import datetime
from typing import Optional, List, Tuple
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from amr_navigation.security_vision import SecurityVisionAnalyzer, VisionDetectionResult
from amr_navigation.incident_logger import IncidentLogger


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SecurityGuardFSM(BasicNavigator):
    STATE_INIT = "INIT"
    STATE_PATROL = "PATROL"
    STATE_REAR_TURN = "REAR_TURN"
    STATE_ALERT = "ALERT"
    STATE_RECOVERY = "RECOVERY"
    STATE_RETURN_TO_BASE = "RETURN_TO_BASE"
    STATE_OPERATOR_OVERRIDE = "OPERATOR_OVERRIDE"

    def __init__(self):
        super().__init__(node_name='patrol_interceptor_node')

        # Declare parameters safely
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('snapshot_dir', '/home/mmdixm/amr_project_ws/data/snapshots')
        self.declare_parameter('db_path', '/home/mmdixm/amr_project_ws/data/security_incidents.db')
        self.declare_parameter('alert_hold_duration_sec', 4.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('min_detection_area', 400)
        self.declare_parameter('min_intruder_area', 400)  # alias for backward-compatibility
        self.declare_parameter('turn_kp', 0.003)

        # Policy & Schedule parameters
        self.declare_parameter('authorized_shift_start_hour', 8)
        self.declare_parameter('authorized_shift_end_hour', 18)
        self.declare_parameter('force_after_hours_mode', True)
        self.declare_parameter('virtual_rfid_topic', '/security/authorized_beacon')
        self.declare_parameter('rfid_timeout_sec', 5.0)

        # HSV and ArUco parameter arrays
        self.declare_parameter('authorized_vest_hsv_lower', [35, 70, 70])
        self.declare_parameter('authorized_vest_hsv_upper', [85, 255, 255])
        self.declare_parameter('unauthorized_red_lower1', [0, 100, 50])
        self.declare_parameter('unauthorized_red_upper1', [10, 255, 255])
        self.declare_parameter('unauthorized_red_lower2', [170, 100, 50])
        self.declare_parameter('unauthorized_red_upper2', [180, 255, 255])
        self.declare_parameter('authorized_aruco_ids', [0, 1, 2, 42, 100])

        # Waypoints [x1, y1, yaw1, x2, y2, yaw2, ...]
        self.declare_parameter(
            'waypoints',
            [2.0, 1.5, 0.0, 2.0, -1.5, -1.57, -1.5, -1.5, 3.14, -1.5, 1.5, 1.57]
        )

        # Retrieve parameters
        self.snapshot_dir = str(self.get_parameter('snapshot_dir').value)
        self.db_path = str(self.get_parameter('db_path').value)
        self.alert_hold_duration = float(self.get_parameter('alert_hold_duration_sec').value)
        self.min_detection_area = int(
            self.get_parameter('min_detection_area').value
            if self.has_parameter('min_detection_area')
            else self.get_parameter('min_intruder_area').value
        )
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.home_pose = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value)
        )

        self.shift_start = int(self.get_parameter('authorized_shift_start_hour').value)
        self.shift_end = int(self.get_parameter('authorized_shift_end_hour').value)
        self.force_after_hours = bool(self.get_parameter('force_after_hours_mode').value)
        self.rfid_timeout_sec = float(self.get_parameter('rfid_timeout_sec').value)

        vest_lower = list(self.get_parameter('authorized_vest_hsv_lower').value)
        vest_upper = list(self.get_parameter('authorized_vest_hsv_upper').value)
        red_lower1 = list(self.get_parameter('unauthorized_red_lower1').value)
        red_upper1 = list(self.get_parameter('unauthorized_red_upper1').value)
        red_lower2 = list(self.get_parameter('unauthorized_red_lower2').value)
        red_upper2 = list(self.get_parameter('unauthorized_red_upper2').value)
        aruco_ids = list(self.get_parameter('authorized_aruco_ids').value)

        # Initialize Subsystems
        self.logger_db = IncidentLogger(db_path=self.db_path, snapshot_dir=self.snapshot_dir)
        self.vision_analyzer = SecurityVisionAnalyzer(
            authorized_vest_lower=tuple(vest_lower),
            authorized_vest_upper=tuple(vest_upper),
            unauthorized_red_lower1=tuple(red_lower1),
            unauthorized_red_upper1=tuple(red_upper1),
            unauthorized_red_lower2=tuple(red_lower2),
            unauthorized_red_upper2=tuple(red_upper2),
            authorized_aruco_ids=aruco_ids,
            min_detection_area=self.min_detection_area
        )

        # Parse waypoints
        raw_wps = self.get_parameter('waypoints').value
        self.waypoints: List[Tuple[float, float, float]] = []
        if raw_wps and len(raw_wps) >= 3:
            for i in range(0, len(raw_wps) - 2, 3):
                self.waypoints.append((
                    float(raw_wps[i]),
                    float(raw_wps[i + 1]),
                    float(raw_wps[i + 2])
                ))
        else:
            self.waypoints = [
                (2.0, 1.5, 0.0),
                (2.0, -1.5, -1.57),
                (-1.5, -1.5, 3.14),
                (-1.5, 1.5, 1.57),
            ]

        self.get_logger().info(f"Loaded {len(self.waypoints)} patrol waypoints: {self.waypoints}")
        self.current_waypoint_idx = 0

        # State Variables
        self.state = self.STATE_INIT
        self.return_to_base_requested = False
        self.override_pose: Optional[Tuple[float, float, float]] = None

        # Robot Pose & Simulated Battery
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.current_yaw = 0.0
        self.battery_pct = 100.0

        # Sensor health & Frame Buffers
        self.bridge = CvBridge()
        self.scan_received = False
        self.latest_scan_time = None
        self.latest_front_cv: Optional[np.ndarray] = None
        self.latest_rear_cv: Optional[np.ndarray] = None

        # Detection Tracking
        self.front_result: Optional[VisionDetectionResult] = None
        self.rear_result: Optional[VisionDetectionResult] = None
        self.last_incident_id: Optional[int] = None
        self.last_incident_time = 0.0

        # RFID Whitelist State
        self.last_rfid_beacon_time = 0.0
        self.last_rfid_data: Optional[str] = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Dual Camera Feeds
        self.cam_front_sub = self.create_subscription(
            Image, '/camera_front/image_raw', self.front_camera_callback, sensor_qos
        )
        self.cam_rear_sub = self.create_subscription(
            Image, '/camera_rear/image_raw', self.rear_camera_callback, sensor_qos
        )
        self.cam_legacy_sub = self.create_subscription(
            Image, '/image_raw', self.front_camera_callback, sensor_qos
        )

        # Virtual RFID Beacon & Operator Trigger Subscriptions
        self.rfid_sub = self.create_subscription(
            String, '/security/authorized_beacon', self.rfid_callback, 10
        )
        self.rtb_sub = self.create_subscription(
            Bool, '/security/return_to_base', self.rtb_callback, 10
        )
        self.operator_sub = self.create_subscription(
            String, '/security/operator_command', self.operator_command_callback, 10
        )

        # Command Center Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/security/status', 10)
        self.telemetry_pub = self.create_publisher(String, '/security/telemetry', 10)
        self.alerts_pub = self.create_publisher(String, '/security/alerts', 10)

        # Telemetry & Battery Timer (2 Hz -> every 0.5s)
        self.telemetry_timer = self.create_timer(0.5, self.publish_telemetry)

        self.get_logger().info('Warehouse Security Guard FSM (Phase 2) initialized successfully.')

    def is_after_hours(self) -> bool:
        if self.force_after_hours:
            return True
        current_hour = datetime.now().hour
        return current_hour < self.shift_start or current_hour >= self.shift_end

    def is_rfid_active(self) -> bool:
        return (time.time() - self.last_rfid_beacon_time) < self.rfid_timeout_sec

    def odom_callback(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg: LaserScan):
        self.scan_received = True
        self.latest_scan_time = self.get_clock().now()

    def front_camera_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_front_cv = cv_img
            self.front_result = self.vision_analyzer.analyze_frame(
                cv_img,
                camera_source="front",
                is_after_hours=self.is_after_hours(),
                rfid_whitelisted=self.is_rfid_active()
            )
        except CvBridgeError as e:
            self.get_logger().error(f'Front CvBridge error: {e}')

    def rear_camera_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_rear_cv = cv_img
            self.rear_result = self.vision_analyzer.analyze_frame(
                cv_img,
                camera_source="rear",
                is_after_hours=self.is_after_hours(),
                rfid_whitelisted=self.is_rfid_active()
            )
        except CvBridgeError as e:
            self.get_logger().error(f'Rear CvBridge error: {e}')

    def rfid_callback(self, msg: String):
        self.last_rfid_beacon_time = time.time()
        self.last_rfid_data = msg.data
        self.get_logger().info(f"[RFID BEACON] Authorized officer ping received: {msg.data}")

    def rtb_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Return-to-Base trigger received via topic!')
            self.return_to_base_requested = True

    def operator_command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            action = cmd.get("action", "").upper().strip()
            self.get_logger().info(f"[OPERATOR COMMAND] Received action: {action}")

            if action == "CONFIRM_INTRUDER":
                if self.last_incident_id is not None:
                    self.logger_db.update_operator_status(self.last_incident_id, "CONFIRMED_INTRUDER")
                    self.publish_status(f"Operator CONFIRMED intruder incident #{self.last_incident_id}!")
            elif action == "MARK_AUTHORIZED":
                if self.last_incident_id is not None:
                    self.logger_db.update_operator_status(self.last_incident_id, "AUTHORIZED")
                    self.last_rfid_beacon_time = time.time() + 30.0  # grant 30s whitelist
                    self.publish_status(f"Operator marked incident #{self.last_incident_id} as AUTHORIZED staff.")
            elif action == "RESUME_PATROL":
                self.state = self.STATE_PATROL
                self.publish_status("Operator commanded RESUME_PATROL.")
            elif action == "RETURN_TO_BASE":
                self.return_to_base_requested = True
            elif action == "GOTO_POSE":
                gx = float(cmd.get("x", 0.0))
                gy = float(cmd.get("y", 0.0))
                gyaw = float(cmd.get("yaw", 0.0))
                self.override_pose = (gx, gy, gyaw)
                self.state = self.STATE_OPERATOR_OVERRIDE
                self.publish_status(f"Operator commanded GOTO_POSE ({gx:.2f}, {gy:.2f}, {gyaw:.2f})")
        except Exception as e:
            self.get_logger().error(f"Failed to parse operator command: {e}")

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(yaw))
        return pose

    def publish_status(self, text: str):
        msg = String()
        msg.data = f"[{self.state}] {text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def publish_telemetry(self):
        # Update simulated battery consumption
        if self.state in [self.STATE_PATROL, self.STATE_ALERT, self.STATE_REAR_TURN]:
            self.battery_pct = max(5.0, self.battery_pct - 0.005)
        elif self.state == self.STATE_RETURN_TO_BASE and math.hypot(self.pos_x - self.home_pose[0], self.pos_y - self.home_pose[1]) < 0.2:
            self.battery_pct = min(100.0, self.battery_pct + 0.1)

        front_det = {
            "classification": self.front_result.classification if self.front_result else None,
            "confidence": round(self.front_result.confidence, 2) if self.front_result else 0.0
        }
        rear_det = {
            "classification": self.rear_result.classification if self.rear_result else None,
            "confidence": round(self.rear_result.confidence, 2) if self.rear_result else 0.0
        }

        telemetry_payload = {
            "timestamp": datetime.now().isoformat(),
            "state": self.state,
            "security_mode": "AFTER_HOURS" if self.is_after_hours() else "DAY_SHIFT",
            "pose": {
                "x": round(self.pos_x, 3),
                "y": round(self.pos_y, 3),
                "yaw": round(self.current_yaw, 3)
            },
            "battery_pct": round(self.battery_pct, 1),
            "current_waypoint": self.current_waypoint_idx + 1,
            "total_waypoints": len(self.waypoints),
            "active_camera_detections": {
                "front": front_det,
                "rear": rear_det
            },
            "rfid_whitelisted": self.is_rfid_active()
        }

        msg = String()
        msg.data = json.dumps(telemetry_payload)
        self.telemetry_pub.publish(msg)

    def log_and_broadcast_alert(self, result: VisionDetectionResult):
        # Prevent rapid duplicate logging within 2.5 seconds
        now = time.time()
        if (now - self.last_incident_time) < 2.5:
            return

        self.last_incident_time = now
        incident_id = self.logger_db.log_incident(
            camera_source=result.camera_source,
            classification=result.classification or "UNAUTHORIZED_INTRUDER",
            pos_x=self.pos_x,
            pos_y=self.pos_y,
            confidence=result.confidence,
            annotated_image=result.annotated_frame,
            operator_status="PENDING"
        )
        self.last_incident_id = incident_id

        # Retrieve saved record
        rec = self.logger_db.get_incident_by_id(incident_id) or {}
        alert_event = {
            "incident_id": incident_id,
            "timestamp": rec.get("timestamp", datetime.now().isoformat()),
            "camera_source": result.camera_source,
            "classification": result.classification,
            "confidence": round(result.confidence, 2),
            "pose": {"x": round(self.pos_x, 3), "y": round(self.pos_y, 3)},
            "snapshot_path": rec.get("snapshot_path", ""),
            "operator_status": "PENDING",
            "details": result.details
        }

        alert_msg = String()
        alert_msg.data = json.dumps(alert_event)
        self.alerts_pub.publish(alert_msg)
        self.publish_status(f"SECURITY INCIDENT #{incident_id} LOGGED [{result.camera_source.upper()}]: {result.classification}")

    def execute_init(self) -> bool:
        """INIT: Verify sensors, set initial pose, wait for Nav2."""
        self.state = self.STATE_INIT
        self.publish_status("Verifying dual-camera and LiDAR feeds...")

        start_wait = time.time()
        while rclpy.ok() and (not self.scan_received or self.latest_front_cv is None):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - start_wait > 10.0:
                self.get_logger().warn("Sensor timeout: proceeding with active feeds.")
                break

        self.publish_status(
            f"Sensors verified: Scan={self.scan_received}, FrontCam={self.latest_front_cv is not None}, RearCam={self.latest_rear_cv is not None}"
        )

        # Set initial pose to (0, 0, 0)
        init_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        for _ in range(3):
            self.setInitialPose(init_pose)
            time.sleep(0.3)

        self.publish_status("Waiting for Nav2 to become active...")
        try:
            self.waitUntilNav2Active(localizer='amcl')
        except Exception as e:
            self.get_logger().warn(f"AMCL wait returned: {e}. Transitioning to PATROL.")
        self.publish_status("Nav2 active. Starting autonomous security patrol.")
        return True

    def execute_rear_spin(self):
        """Executes a 180° in-place spin so front camera & LiDAR lock onto rear threat."""
        self.state = self.STATE_REAR_TURN
        self.cancelTask()
        self.publish_status("REAR THREAT DETECTED! Initiating 180° in-place spin to confront subject.")

        # Emergency brake
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        # Attempt Nav2 Spin behavior
        spin_success = False
        try:
            if hasattr(self, 'spin'):
                self.spin(spin_dist=math.pi, time_allowance=6)
                spin_start = time.time()
                while not self.isTaskComplete() and (time.time() - spin_start < 6.0) and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                spin_success = self.getResult() == TaskResult.SUCCEEDED
        except Exception as e:
            self.get_logger().warn(f"Nav2 spin action unavailable, using fallback: {e}")

        # Fallback velocity rotation if Nav2 spin was not used or failed
        if not spin_success:
            self.get_logger().info("Executing velocity-based 180° turn fallback.")
            turn_cmd = Twist()
            turn_cmd.angular.z = 1.0  # rad/s
            turn_duration = 3.14  # seconds for ~180°
            start_t = time.time()
            while (time.time() - start_t) < turn_duration and rclpy.ok():
                self.cmd_vel_pub.publish(turn_cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            self.cmd_vel_pub.publish(stop_twist)

        self.publish_status("180° spin completed. Locking front sensors on target.")
        self.state = self.STATE_ALERT

    def handle_alert(self):
        """ALERT: Confront target with front camera, align bearing, log incident, hold standoff."""
        self.state = self.STATE_ALERT
        self.cancelTask()

        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        # Log and broadcast alert
        target_res = self.front_result if self.front_result and self.front_result.classification else self.rear_result
        if target_res:
            self.log_and_broadcast_alert(target_res)

        # Orient toward target using front camera bearing error
        if self.front_result and abs(self.front_result.pixel_err_x) > 25:
            turn_twist = Twist()
            turn_twist.angular.z = float(np.clip(self.turn_kp * self.front_result.pixel_err_x, -0.6, 0.6))
            self.cmd_vel_pub.publish(turn_twist)
            time.sleep(0.4)
            self.cmd_vel_pub.publish(stop_twist)

        self.publish_status(
            f"SECURITY ALERT STANDOFF! Intercepting threat. Holding position for {self.alert_hold_duration}s."
        )

        hold_start = time.time()
        while (time.time() - hold_start) < self.alert_hold_duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.cmd_vel_pub.publish(stop_twist)
            if self.return_to_base_requested or self.state == self.STATE_OPERATOR_OVERRIDE:
                break

        if self.state not in [self.STATE_OPERATOR_OVERRIDE, self.STATE_RETURN_TO_BASE]:
            self.publish_status("Incident recorded. Resuming security perimeter patrol.")
            self.state = self.STATE_PATROL

    def handle_recovery(self):
        """RECOVERY: Clear costmaps, short backup, advance waypoint."""
        self.state = self.STATE_RECOVERY
        self.publish_status("Initiating Recovery: Clearing costmaps & backing up.")

        try:
            self.clearAllCostmaps()
        except Exception:
            pass

        # Velocity-based physical backup away from obstacles
        backup_twist = Twist()
        backup_twist.linear.x = -0.18
        start_b = time.time()
        while time.time() - start_b < 1.5 and rclpy.ok():
            self.cmd_vel_pub.publish(backup_twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.cmd_vel_pub.publish(Twist())

        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        self.state = self.STATE_PATROL

    def handle_return_to_base(self):
        """RETURN_TO_BASE: Return to home pose (0, 0, 0)."""
        self.state = self.STATE_RETURN_TO_BASE
        self.cancelTask()
        self.publish_status(f"Returning to Home Base Station: x={self.home_pose[0]}, y={self.home_pose[1]}")

        home_stamped = self.create_pose_stamped(
            self.home_pose[0], self.home_pose[1], self.home_pose[2]
        )
        self.goToPose(home_stamped)

        while not self.isTaskComplete() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.publish_status("Robot safely docked at Home Base Station.")
        else:
            self.get_logger().warn("Failed to reach Home Base Station cleanly.")

        self.pause_and_scan(4.0)
        self.state = self.STATE_PATROL

    def handle_operator_override(self):
        """OPERATOR_OVERRIDE: Navigate to operator-commanded pose."""
        if not self.override_pose:
            self.state = self.STATE_PATROL
            return

        gx, gy, gyaw = self.override_pose
        self.cancelTask()
        self.publish_status(f"Executing Operator Dispatch to ({gx:.2f}, {gy:.2f})")
        target_stamped = self.create_pose_stamped(gx, gy, gyaw)
        self.goToPose(target_stamped)

        while not self.isTaskComplete() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.return_to_base_requested:
                break

        self.override_pose = None
        self.publish_status("Operator dispatch goal reached. Pausing for inspection.")
        self.pause_and_scan(3.0)
        self.state = self.STATE_PATROL

    def pause_and_scan(self, duration_sec: float):
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        start = time.time()
        while time.time() - start < duration_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_fsm(self):
        """Main FSM execution loop."""
        if not self.execute_init():
            return

        self.state = self.STATE_PATROL
        nav_started = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check RTB
            if self.return_to_base_requested:
                self.return_to_base_requested = False
                self.handle_return_to_base()
                nav_started = False
                continue

            # Check Operator Override
            if self.state == self.STATE_OPERATOR_OVERRIDE:
                self.handle_operator_override()
                nav_started = False
                continue

            # State: PATROL
            if self.state == self.STATE_PATROL:
                # 1. Check Rear Camera first: if intruder behind robot, execute 180° spin
                if (
                    self.rear_result
                    and self.rear_result.classification == "UNAUTHORIZED_INTRUDER"
                    and not self.is_rfid_active()
                ):
                    self.execute_rear_spin()
                    self.handle_alert()
                    nav_started = False
                    continue

                # 2. Check Front Camera: if intruder in front, alert and confront
                if (
                    self.front_result
                    and self.front_result.classification == "UNAUTHORIZED_INTRUDER"
                    and not self.is_rfid_active()
                ):
                    self.handle_alert()
                    nav_started = False
                    continue

                # 3. If authorized security personnel detected, log friendly greeting
                if (
                    self.front_result
                    and self.front_result.classification == "AUTHORIZED_SECURITY"
                ):
                    now = time.time()
                    if (now - self.last_incident_time) > 10.0:
                        self.publish_status(f"Authorized security staff verified: {self.front_result.details}")
                        self.last_incident_time = now

                # Start navigation to current waypoint if not active
                if not nav_started:
                    wp = self.waypoints[self.current_waypoint_idx]
                    goal_pose = self.create_pose_stamped(wp[0], wp[1], wp[2])
                    self.publish_status(
                        f"Navigating to Waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)}: x={wp[0]}, y={wp[1]}"
                    )
                    self.goToPose(goal_pose)
                    nav_started = True
                    wp_dispatch_time = time.time()

                # Check waypoint arrival or timeout
                wp_elapsed = time.time() - wp_dispatch_time if 'wp_dispatch_time' in locals() else 0.0
                if self.isTaskComplete() or wp_elapsed > 35.0:
                    result = self.getResult() if self.isTaskComplete() else TaskResult.FAILED
                    nav_started = False
                    if result == TaskResult.SUCCEEDED:
                        self.publish_status(f"Arrived at Waypoint {self.current_waypoint_idx+1}.")
                        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
                        self.pause_and_scan(2.0)
                    elif result in [TaskResult.FAILED, TaskResult.CANCELED]:
                        self.get_logger().warn(f"Navigation task failed at waypoint {self.current_waypoint_idx+1}.")
                        self.handle_recovery()

            elif self.state == self.STATE_RECOVERY:
                self.handle_recovery()
                nav_started = False

        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    fsm = SecurityGuardFSM()
    try:
        fsm.run_fsm()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
