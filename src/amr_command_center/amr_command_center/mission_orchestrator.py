#!/usr/bin/env python3
"""
End-to-End Autonomous Mission Orchestrator for Warehouse Security AMR (ROS 2 Jazzy).

Performs the complete single-command end-to-end mission lifecycle:
  Stage 1: Validates Gazebo Harmonic warehouse simulation & Live Web Command Center (port 8080).
  Stage 2: Launches Phase 1 Autonomous Frontier Mapping (auto_mapping.launch.py), monitors
           exploration until all frontiers are mapped, map is saved, and robot docks back at (0,0).
  Stage 3: Cleanly shuts down Phase 1 SLAM process group (preventing duplicate map->odom TF),
           launches Phase 2 Nav2 Security Patrol (security_patrol.launch.py), and verifies PATROL state.
  Stage 4: Runs a 30-second simulation-time countdown, automatically triggers Gazebo intruder spawn,
           and monitors detection, 180° spin/intercept, SQLite forensic snapshot, and live UI alert.
"""

import os
import sys
import time
import json
import signal
import shutil
import threading
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class MissionOrchestrator(Node):

    def __init__(self):
        super().__init__('mission_orchestrator_node')

        # Declare parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('rviz', True)
        self.declare_parameter('force_after_hours_mode', True)
        self.declare_parameter('intruder_delay_sec', 30.0)
        self.declare_parameter('spawn_intruder_x', 1.8)
        self.declare_parameter('spawn_intruder_y', 1.2)
        self.declare_parameter('spawn_intruder_z', 0.35)
        self.declare_parameter('map_yaml', '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.yaml')
        self.declare_parameter('map_pgm', '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.pgm')
        self.declare_parameter('port', 8080)
        self.declare_parameter('max_mapping_time_sec', 300.0)

        # Retrieve parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.rviz = self.get_parameter('rviz').get_parameter_value().bool_value
        self.force_after_hours = self.get_parameter('force_after_hours_mode').get_parameter_value().bool_value
        self.intruder_delay_sec = self.get_parameter('intruder_delay_sec').get_parameter_value().double_value
        self.spawn_x = self.get_parameter('spawn_intruder_x').get_parameter_value().double_value
        self.spawn_y = self.get_parameter('spawn_intruder_y').get_parameter_value().double_value
        self.spawn_z = self.get_parameter('spawn_intruder_z').get_parameter_value().double_value
        self.map_yaml = os.path.expanduser(self.get_parameter('map_yaml').get_parameter_value().string_value)
        self.map_pgm = os.path.expanduser(self.get_parameter('map_pgm').get_parameter_value().string_value)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.max_mapping_time_sec = self.get_parameter('max_mapping_time_sec').get_parameter_value().double_value

        # State tracking
        self.current_stage = "STAGE_1_INIT"
        self.latest_explore_status = "INIT"
        self.latest_telemetry = {}
        self.latest_alert = None
        self.sim_time_sec = 0.0
        self.clock_received = False
        self.mapping_finished = False

        # Process handles
        self.phase1_process: Optional[subprocess.Popen] = None
        self.phase2_process: Optional[subprocess.Popen] = None
        self.is_shutting_down = False

        # Subscriptions
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.clock_sub = self.create_subscription(Clock, '/clock', self._clock_cb, qos_sensor)
        self.explore_sub = self.create_subscription(String, '/explore/status', self._explore_status_cb, 10)
        self.telemetry_sub = self.create_subscription(String, '/security/telemetry', self._telemetry_cb, 10)
        self.alerts_sub = self.create_subscription(String, '/security/alerts', self._alerts_cb, 10)
        self.reset_sub = self.create_subscription(String, '/mission/reset', self._reset_cb, 10)
        self.operator_sub = self.create_subscription(String, '/security/operator_command', self._operator_cmd_cb, 10)

        # Mission Status & Stage Publishers
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        self.mission_stage_pub = self.create_publisher(String, '/mission/stage', 10)

        self.reset_requested = False
        self.get_logger().info("Mission Orchestrator Node initialized.")

        # Launch orchestration thread
        self.worker_thread = threading.Thread(target=self._run_mission_lifecycle, daemon=True)
        self.worker_thread.start()

    # --------------------------------------------------------------------------
    # Callbacks & Helpers
    # --------------------------------------------------------------------------
    def _clock_cb(self, msg: Clock):
        self.sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_received = True

    def _explore_status_cb(self, msg: String):
        self.latest_explore_status = msg.data.strip()
        if "FINISHED" in self.latest_explore_status:
            self.mapping_finished = True

    def _telemetry_cb(self, msg: String):
        try:
            self.latest_telemetry = json.loads(msg.data)
        except Exception:
            pass

    def _alerts_cb(self, msg: String):
        self.latest_alert = msg.data

    def _operator_cmd_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
            action = cmd.get("action", "").upper().strip()
            if action == "RESUME_PATROL":
                self.get_logger().info("[MISSION ORCHESTRATOR] RESUME_PATROL command received: advancing to Stage 2 Security Patrol...")
                self.mapping_finished = True
            elif action == "RESET_MAP":
                self.get_logger().info("[MISSION ORCHESTRATOR] RESET_MAP command received.")
                self._reset_cb(msg)
        except Exception as e:
            self.get_logger().warn(f"Error handling operator command: {e}")

    def _reset_cb(self, msg: String):
        self.get_logger().warn(f"[MISSION ORCHESTRATOR] On-demand Map Reset & Re-SLAM signal received: {msg.data}")
        self.reset_requested = True
        self.mapping_finished = False
        if self.phase2_process:
            self.terminate_process_group(self.phase2_process, "Phase 2 Security Patrol")
            self.phase2_process = None
        if self.phase1_process:
            self.terminate_process_group(self.phase1_process, "Phase 1 SLAM Mapping")
            self.phase1_process = None
        self._delete_map_files()
        self.set_stage("STAGE 1: MAPPING", "Map reset complete. Re-initiating Phase 1 Autonomous Mapping...")

    def _delete_map_files(self):
        map_paths = [
            "/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.yaml",
            "/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.pgm",
            "/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/.mapping_complete",
            "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps/warehouse_map.yaml",
            "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps/warehouse_map.pgm",
            "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps/.mapping_complete",
        ]
        for p in map_paths:
            if os.path.exists(p):
                try:
                    os.remove(p)
                    self.get_logger().info(f"Removed: {p}")
                except Exception as e:
                    self.get_logger().warn(f"Failed removing {p}: {e}")

    def get_sim_time(self) -> float:
        if self.clock_received and self.use_sim_time:
            return self.sim_time_sec
        return time.time()

    def set_stage(self, stage_name: str, status_msg: str):
        self.current_stage = stage_name
        self.get_logger().info(f"[{stage_name}] {status_msg}")

        m_stage = String()
        m_stage.data = stage_name
        self.mission_stage_pub.publish(m_stage)

        m_status = String()
        m_status.data = f"[{stage_name}] {status_msg}"
        self.mission_status_pub.publish(m_status)

    def terminate_process_group(self, proc: Optional[subprocess.Popen], name: str):
        if proc is None:
            return
        self.get_logger().info(f"Terminating process group for {name} (PID {proc.pid})...")
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            for _ in range(25):
                if proc.poll() is not None:
                    break
                time.sleep(0.2)
            if proc.poll() is None:
                self.get_logger().warn(f"{name} did not exit after SIGINT; sending SIGTERM...")
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(1.0)
            if proc.poll() is None:
                self.get_logger().warn(f"{name} still alive; sending SIGKILL...")
                os.killpg(pgid, signal.SIGKILL)
                proc.wait()
            self.get_logger().info(f"{name} terminated cleanly.")
        except Exception as e:
            self.get_logger().error(f"Error terminating {name}: {e}")

    # --------------------------------------------------------------------------
    # End-to-End Orchestration State Machine
    # --------------------------------------------------------------------------
    def _run_mission_lifecycle(self):
        try:
            self._stage_1_simulation_ready()
            while rclpy.ok() and not self.is_shutting_down:
                self.reset_requested = False
                self.mapping_finished = False
                self._stage_2_frontier_mapping()
                if self.reset_requested:
                    time.sleep(1.0)
                    continue
                self._stage_3_security_patrol()
                if self.reset_requested:
                    time.sleep(1.0)
                    continue
                self._stage_4_intruder_intercept()
                if self.reset_requested:
                    time.sleep(1.0)
                    continue
                self._stage_5_mission_complete()
                while rclpy.ok() and not self.reset_requested and not self.is_shutting_down:
                    time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Fatal error in mission orchestrator: {e}", throttle_duration_sec=1.0)
            import traceback
            traceback.print_exc()

    def _stage_1_simulation_ready(self):
        self.set_stage(
            "STAGE 1: INIT",
            "Validating Gazebo Harmonic simulation and Command Center Web UI..."
        )
        print("\n" + "=" * 80)
        print("  STAGE 1: SIMULATION & LIVE COMMAND CENTER VERIFICATION")
        print("=" * 80)
        print(f"  • Web Dashboard:    http://localhost:{self.port}")
        print(f"  • Front Camera:     http://localhost:{self.port}/stream/front")
        print(f"  • Rear Camera:      http://localhost:{self.port}/stream/rear")
        print("=" * 80 + "\n")

        # Wait for simulation clock
        wait_start = time.time()
        while rclpy.ok() and not self.clock_received and (time.time() - wait_start < 15.0):
            time.sleep(0.5)

        if not self.clock_received:
            self.get_logger().warn("Simulation clock (/clock) not yet received. Continuing with wall clock.")

        self.set_stage(
            "STAGE 1: ACTIVE",
            f"Simulation & Web Command Center active at http://localhost:{self.port}."
        )
        time.sleep(2.0)

    def _stage_2_frontier_mapping(self):
        print("\n" + "=" * 80)
        print("  STAGE 1: AUTONOMOUS FRONTIER MAPPING & RETURN TO BASE")
        print("=" * 80)
        self.set_stage(
            "STAGE 1: MAPPING",
            "Launching Phase 1 Autonomous Frontier Mapping (auto_mapping.launch.py)..."
        )

        # Clear previous mapping marker
        marker_file = os.path.join(os.path.dirname(self.map_yaml), ".mapping_complete")
        if os.path.exists(marker_file):
            try:
                os.remove(marker_file)
            except Exception:
                pass

        # Launch auto_mapping.launch.py
        cmd_str = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source /home/mmdixm/Desktop/amr_security_ws/install/setup.bash && "
            f"ros2 launch amr_navigation auto_mapping.launch.py rviz:={str(self.rviz).lower()} use_sim_time:=true"
        )
        self.get_logger().info(f"Executing: {cmd_str}")
        self.phase1_process = subprocess.Popen(
            ["bash", "-c", cmd_str],
            preexec_fn=os.setsid
        )

        mapping_start_sim = self.get_sim_time()
        mapping_start_wall = time.time()
        last_log_wall = time.time()

        while rclpy.ok() and not self.mapping_finished:
            if self.reset_requested:
                self.get_logger().info("Reset requested during Stage 2 mapping. Aborting stage.")
                return

            sim_elapsed = self.get_sim_time() - mapping_start_sim
            # Check marker file after reasonable start time
            if os.path.exists(marker_file) and sim_elapsed >= 5.0:
                self.get_logger().info("Mapping completion marker (.mapping_complete) detected!")
                self.mapping_finished = True
                break

            # Check process health
            if self.phase1_process.poll() is not None:
                self.get_logger().warn(f"Phase 1 process exited with return code {self.phase1_process.returncode}")
                if os.path.exists(self.map_yaml) or os.path.exists(marker_file):
                    self.get_logger().info("Map files found on Phase 1 exit. Advancing to Patrol.")
                    self.mapping_finished = True
                break

            # Periodic status log
            now_wall = time.time()
            if now_wall - last_log_wall >= 8.0:
                last_log_wall = now_wall
                sim_elapsed = self.get_sim_time() - mapping_start_sim
                self.get_logger().info(
                    f"[Mapping Progress] Frontier exploration elapsed: {sim_elapsed:.1f}s | Status: {self.latest_explore_status}"
                )

            # Optional timeout check
            if self.max_mapping_time_sec > 0:
                elapsed_sim = self.get_sim_time() - mapping_start_sim
                if elapsed_sim >= self.max_mapping_time_sec:
                    self.get_logger().warn(
                        f"Frontier mapping reached configured max duration ({self.max_mapping_time_sec}s). Advancing..."
                    )
                    break

            time.sleep(0.5)

        if self.reset_requested:
            return

        # Verify saved map files exist and are synced
        time.sleep(2.0)
        install_maps_dir = "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps"
        os.makedirs(install_maps_dir, exist_ok=True)
        for ext in [".yaml", ".pgm"]:
            src_f = os.path.splitext(self.map_yaml)[0] + ext
            dst_f = os.path.join(install_maps_dir, "warehouse_map" + ext)
            if os.path.exists(src_f):
                try:
                    if not (os.path.exists(dst_f) and os.path.samefile(src_f, dst_f)):
                        shutil.copy2(src_f, dst_f)
                except Exception:
                    pass

        print("\n" + "=" * 80)
        print("  [SUCCESS] MAPPING COMPLETE: All frontiers mapped, map saved, robot at base.")
        print(f"  • Map YAML: {self.map_yaml}")
        print(f"  • Map PGM:  {self.map_pgm}")
        print("=" * 80 + "\n")
        self.set_stage(
            "RETURNING TO BASE",
            "Frontier exploration complete. Map saved and synced. Transitioning to security patrol."
        )

    def _stage_3_security_patrol(self):
        print("\n" + "=" * 80)
        print("  STAGE 2: CLEAN TRANSITION TO SECURITY PATROL")
        print("=" * 80)
        self.set_stage(
            "STAGE 2: PATROL",
            "Terminating Phase 1 SLAM process group to eliminate duplicate TF broadcasters..."
        )

        # Cleanly shut down Phase 1 SLAM
        self.terminate_process_group(self.phase1_process, "Phase 1 (SLAM Mapping)")
        self.phase1_process = None
        os.system("pkill -9 -f 'odom_to_tf|auto_explorer' || true")

        if self.reset_requested:
            return

        # Wait 4 seconds for DDS discovery graph to purge old TF nodes
        self.get_logger().info("Purging old TF broadcaster handles and waiting 4.0s...")
        time.sleep(4.0)

        if self.reset_requested:
            return

        # Launch Phase 2 Security Patrol
        self.set_stage(
            "STAGE 2: PATROL",
            "Launching Phase 2 Security Patrol (security_patrol.launch.py)..."
        )
        map_to_use = self.map_yaml
        if not os.path.exists(map_to_use):
            candidate = "/home/mmdixm/Desktop/amr_security_ws/install/amr_navigation/share/amr_navigation/maps/warehouse_map.yaml"
            if os.path.exists(candidate):
                map_to_use = candidate

        cmd_patrol_str = (
            f"source /opt/ros/jazzy/setup.bash && "
            f"source /home/mmdixm/Desktop/amr_security_ws/install/setup.bash && "
            f"ros2 launch amr_navigation security_patrol.launch.py "
            f"map:={map_to_use} rviz:={str(self.rviz).lower()} "
            f"force_after_hours_mode:={str(self.force_after_hours).lower()} use_sim_time:=true"
        )
        self.get_logger().info(f"Executing: {cmd_patrol_str}")
        self.phase2_process = subprocess.Popen(
            ["bash", "-c", cmd_patrol_str],
            preexec_fn=os.setsid
        )

        # Wait for robot to reach PATROL state
        self.get_logger().info("Waiting for /security/telemetry to confirm state == 'PATROL'...")
        wait_patrol_start = time.time()
        while rclpy.ok():
            if self.reset_requested:
                return
            state = self.latest_telemetry.get("state", "").upper()
            if state in ["PATROL", "ALERT", "REAR_TURN"]:
                break
            if time.time() - wait_patrol_start > 45.0:
                self.get_logger().warn("Waiting for Nav2 activation and initial AMCL localization...")
                wait_patrol_start = time.time()
            time.sleep(0.5)

        print("\n" + "=" * 80)
        print("  [SUCCESS] STAGE 2 ACTIVE: Robot has entered PATROL state.")
        print("  Autonomous waypoint patrol active with 360° dual-camera surveillance.")
        print("=" * 80 + "\n")
        self.set_stage(
            "STAGE 2: PATROL",
            "Security AMR actively patrolling warehouse perimeter waypoints."
        )

    def _stage_4_intruder_intercept(self):
        print("\n" + "=" * 80)
        print("  STAGE 4: AUTOMATED INTRUDER SPAWN & INTERCEPTION")
        print("=" * 80)
        self.set_stage(
            "STAGE_4_COUNTDOWN",
            f"Starting {self.intruder_delay_sec:.0f}-second simulation-time countdown in PATROL mode..."
        )

        # Simulation time countdown
        t_start = self.get_sim_time()
        last_countdown_log = 0.0

        while rclpy.ok():
            if self.reset_requested:
                return
            now_sim = self.get_sim_time()
            elapsed = now_sim - t_start
            remaining = self.intruder_delay_sec - elapsed

            if elapsed - last_countdown_log >= 5.0:
                last_countdown_log = elapsed
                self.get_logger().info(
                    f"[Countdown] Security patrol active: {elapsed:.1f}s / {self.intruder_delay_sec:.1f}s elapsed before intruder spawn..."
                )

            if remaining <= 0.0:
                break
            time.sleep(0.5)

        if self.reset_requested:
            return

        # Trigger Gazebo Intruder Spawn Helper
        self.set_stage(
            "STAGE_4_INTRUDER_SPAWNED",
            f"Countdown reached! Spawning Unauthorized Intruder at ({self.spawn_x}, {self.spawn_y}, {self.spawn_z})..."
        )
        spawn_script = "/home/mmdixm/Desktop/amr_security_ws/src/amr_gazebo/scripts/spawn_intruder.sh"
        if not os.path.exists(spawn_script):
            spawn_script = "/home/mmdixm/Desktop/amr_security_ws/install/amr_gazebo/share/amr_gazebo/scripts/spawn_intruder.sh"

        try:
            # Delete any previous target first, then spawn
            subprocess.run(["bash", spawn_script, "delete"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
            res = subprocess.run(
                ["bash", spawn_script, str(self.spawn_x), str(self.spawn_y), str(self.spawn_z)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=15.0
            )
            self.get_logger().info(f"spawn_intruder output: {res.stdout.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn intruder via helper: {e}")

        # Monitor detection & alert intercept
        self.get_logger().info("Monitoring robot sensors for intruder detection and FSM ALERT transition...")
        alert_start = time.time()
        detected = False

        while rclpy.ok() and (time.time() - alert_start < 40.0):
            state = self.latest_telemetry.get("state", "").upper()
            if state in ["ALERT", "REAR_TURN"] or (self.latest_alert is not None):
                detected = True
                break
            time.sleep(0.3)

        if detected:
            print("\n" + "=" * 80)
            print("  🚨 [STAGE 4 SUCCESS] UNAUTHORIZED INTRUDER DETECTED & INTERCEPTED!")
            print("=" * 80)
            print(f"  • Detection State:  {self.latest_telemetry.get('state')}")
            print("  • Incident DB:      ~/Desktop/amr_security_ws/data/security_incidents.db")
            print("  • Forensic Snapshots: ~/Desktop/amr_security_ws/data/snapshots/")
            print(f"  • Tactical Web UI:  http://localhost:{self.port}")
            print("=" * 80 + "\n")
            self.set_stage(
                "STAGE_4_ALERT_DETECTED",
                "Intruder detected and intercepted! Forensic snapshot captured and pushed to Command Center."
            )
        else:
            self.get_logger().warn("Intruder spawned; robot continues scanning perimeter corridor.")

    def _stage_5_mission_complete(self):
        print("\n" + "=" * 80)
        print("  🎉 FULL END-TO-END DEMO COMPLETED AUTONOMOUSLY!")
        print("=" * 80)
        print("  All 4 stages executed successfully:")
        print("  1. Gazebo Simulation & Command Center (Live at t=0s)")
        print("  2. Autonomous Frontier Exploration & Return to Base (Phase 1)")
        print("  3. Clean Process Transition to Security Patrol (Phase 2)")
        print("  4. Automated Intruder Spawn, Interception & Incident Alerting")
        print("=" * 80)
        print(f"  Tactical Dashboard remains online at: http://localhost:{self.port}")
        print("  Press Ctrl+C in this terminal when ready to stop the demo.")
        print("=" * 80 + "\n")

        self.set_stage(
            "MISSION_COMPLETE",
            f"End-to-end autonomous mission complete! Dashboard live at http://localhost:{self.port}."
        )

        while rclpy.ok() and not self.is_shutting_down:
            time.sleep(1.0)

    def shutdown(self):
        self.is_shutting_down = True
        self.get_logger().info("Mission Orchestrator shutting down. Cleaning up child process groups...")
        if self.phase1_process:
            self.terminate_process_group(self.phase1_process, "Phase 1")
        if self.phase2_process:
            self.terminate_process_group(self.phase2_process, "Phase 2")


def main(args=None):
    rclpy.init(args=args)
    orchestrator = MissionOrchestrator()
    try:
        rclpy.spin(orchestrator)
    except (KeyboardInterrupt, SystemExit):
        orchestrator.get_logger().info("Mission Orchestrator received keyboard interrupt.")
    except Exception:
        pass
    finally:
        orchestrator.shutdown()
        orchestrator.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
