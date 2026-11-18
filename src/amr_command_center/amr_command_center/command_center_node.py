#!/usr/bin/env python3
"""
Command Center Bridge Node (command_center_node.py)
High-performance FastAPI, WebSocket, and Dual-MJPEG Video Streaming Server
for Autonomous Warehouse Security AMR Command Center.
"""

import os
import sys
import io
import time
import json
import sqlite3
import threading
import asyncio
from datetime import datetime
from typing import Optional, Dict, Any, Set, List
import yaml
from PIL import Image as PILImage
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image as RosImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String as RosString
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Body, Security, Depends, status
from fastapi.responses import StreamingResponse, Response, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import APIKeyHeader, HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, Field
import uvicorn


def _load_env_file():
    candidates = [
        os.path.expanduser('~/Desktop/amr_security_ws/.env'),
        os.path.join(os.getcwd(), '.env'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '.env'),
    ]
    for env_path in candidates:
        if os.path.exists(env_path):
            try:
                with open(env_path, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#') and '=' in line:
                            k, v = line.split('=', 1)
                            k, v = k.strip(), v.strip().strip('"').strip("'")
                            if k and k not in os.environ:
                                os.environ[k] = v
                break
            except Exception:
                pass


_load_env_file()
OPERATOR_TOKEN = os.environ.get("AMR_OPERATOR_TOKEN", "amr-sec-2026")

api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)
bearer_scheme = HTTPBearer(auto_error=False)


def verify_operator_token(
    x_api_key: Optional[str] = Security(api_key_header),
    bearer: Optional[HTTPAuthorizationCredentials] = Security(bearer_scheme),
) -> str:
    token = x_api_key or (bearer.credentials if bearer else None)
    if not token or token != OPERATOR_TOKEN:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Unauthorized: Valid Operator Security Token required.",
            headers={"WWW-Authenticate": "Bearer"}
        )
    return token


class TeleopPayload(BaseModel):
    linear_x: float = Field(0.0, ge=-0.8, le=0.8)
    angular_z: float = Field(0.0, ge=-1.5, le=1.5)


class IncidentStatusUpdate(BaseModel):
    status: str


class CommandCenterBridge(Node):
    def __init__(self):
        super().__init__('amr_command_center_node')

        # Declare ROS 2 parameters
        self.declare_parameter('port', 8000)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('db_path', '/home/mmdixm/Desktop/amr_security_ws/data/security_incidents.db')
        self.declare_parameter('snapshot_dir', '/home/mmdixm/Desktop/amr_security_ws/data/snapshots')
        self.declare_parameter('map_yaml', '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.yaml')
        self.declare_parameter('map_pgm', '/home/mmdixm/Desktop/amr_security_ws/src/amr_navigation/maps/warehouse_map.pgm')

        self.port = int(self.get_parameter('port').value)
        self.host = str(self.get_parameter('host').value)
        self.db_path = os.path.expanduser(str(self.get_parameter('db_path').value))
        self.snapshot_dir = os.path.expanduser(str(self.get_parameter('snapshot_dir').value))
        self.map_yaml = os.path.expanduser(str(self.get_parameter('map_yaml').value))
        self.map_pgm = os.path.expanduser(str(self.get_parameter('map_pgm').value))

        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        os.makedirs(self.snapshot_dir, exist_ok=True)
        self._init_sqlite()

        # Thread synchronization & frame buffers
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.front_jpeg: bytes = self._make_placeholder("FRONT CAMERA - NO SIGNAL")
        self.rear_jpeg: bytes = self._make_placeholder("REAR CAMERA - NO SIGNAL")
        self.last_front_time = 0.0
        self.last_rear_time = 0.0

        # Latest telemetry and alerts cache
        self.latest_telemetry: Optional[Dict[str, Any]] = None
        self.latest_alert: Optional[Dict[str, Any]] = None

        # Live Map and SLAM Progress state
        self.latest_explore_progress: Optional[Dict[str, Any]] = {"progress": 0.0, "stage": "STAGE 1: MAPPING"}
        self.live_map_png: Optional[bytes] = None
        self.live_map_meta: Optional[Dict[str, Any]] = None
        self.last_map_time = 0.0
        self.last_map_reset_time = 0.0

        # Asyncio event loop reference (set on FastAPI startup)
        self.async_loop: Optional[asyncio.AbstractEventLoop] = None
        self.active_websockets: Set[WebSocket] = set()

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

        # Subscriptions
        self.sub_cam_front = self.create_subscription(
            RosImage, '/camera_front/image_raw', self.front_cam_callback, sensor_qos
        )
        self.sub_cam_rear = self.create_subscription(
            RosImage, '/camera_rear/image_raw', self.rear_cam_callback, sensor_qos
        )
        self.sub_telemetry = self.create_subscription(
            RosString, '/security/telemetry', self.telemetry_callback, 10
        )
        self.sub_alerts = self.create_subscription(
            RosString, '/security/alerts', self.alerts_callback, 10
        )
        self.sub_mission_stage = self.create_subscription(
            RosString, '/mission/stage', self.mission_stage_callback, 10
        )
        self.sub_mission_status = self.create_subscription(
            RosString, '/mission/status', self.mission_status_callback, 10
        )
        self.sub_explore_progress = self.create_subscription(
            RosString, '/explore/progress', self.explore_progress_callback, 10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos
        )

        # Publishers
        self.pub_operator_cmd = self.create_publisher(RosString, '/security/operator_command', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mission_reset = self.create_publisher(RosString, '/mission/reset', 10)

        self.get_logger().info(f"AMR Command Center Bridge initialized (Port: {self.port})")

    def _init_sqlite(self):
        try:
            with sqlite3.connect(self.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    CREATE TABLE IF NOT EXISTS incidents (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp TEXT NOT NULL,
                        camera_source TEXT NOT NULL,
                        classification TEXT NOT NULL,
                        pos_x REAL DEFAULT 0.0,
                        pos_y REAL DEFAULT 0.0,
                        confidence REAL DEFAULT 0.0,
                        snapshot_path TEXT,
                        operator_status TEXT DEFAULT 'PENDING'
                    )
                """)
                conn.commit()
        except Exception as e:
            self.get_logger().error(f"Error initializing SQLite DB: {e}")

    def _make_placeholder(self, text: str) -> bytes:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = (20, 24, 34)  # dark slate color
        # Draw grid
        for y in range(0, 480, 40):
            cv2.line(img, (0, y), (640, y), (30, 38, 50), 1)
        for x in range(0, 640, 40):
            cv2.line(img, (x, 0), (x, 480), (30, 38, 50), 1)
        cv2.rectangle(img, (120, 180), (520, 300), (45, 55, 72), -1)
        cv2.rectangle(img, (120, 180), (520, 300), (74, 85, 104), 2)
        cv2.putText(img, text, (140, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 210, 225), 2)
        cv2.putText(img, "WAITING FOR SIMULATION STREAM", (170, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (113, 128, 150), 1)
        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return jpeg.tobytes()

    def front_cam_callback(self, msg: RosImage):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with self.lock:
                self.front_jpeg = jpeg.tobytes()
                self.last_front_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Front cam frame encode error: {e}")

    def rear_cam_callback(self, msg: RosImage):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with self.lock:
                self.rear_jpeg = jpeg.tobytes()
                self.last_rear_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Rear cam frame encode error: {e}")

    def telemetry_callback(self, msg: RosString):
        try:
            data = json.loads(msg.data)
            self.latest_telemetry = data
            self._broadcast_ws({"event": "telemetry", "data": data})
        except Exception as e:
            self.get_logger().error(f"Failed to parse telemetry: {e}")

    def alerts_callback(self, msg: RosString):
        try:
            data = json.loads(msg.data)
            self.latest_alert = data
            self._broadcast_ws({"event": "alert", "data": data})
        except Exception as e:
            self.get_logger().error(f"Failed to parse alert: {e}")

    def mission_stage_callback(self, msg: RosString):
        self._broadcast_ws({"event": "mission_stage", "data": msg.data})

    def mission_status_callback(self, msg: RosString):
        self._broadcast_ws({"event": "mission_status", "data": msg.data})

    def explore_progress_callback(self, msg: RosString):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.latest_explore_progress = data
            self._broadcast_ws({"event": "explore_progress", "data": data})
        except Exception as e:
            self.get_logger().error(f"Failed to parse explore progress: {e}")

    def map_callback(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            if w <= 0 or h <= 0:
                return

            raw = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # Grayscale map: 0 (free) -> 254 (white), >50 (occupied) -> 0 (black), -1 (unknown) -> 205 (gray)
            img = np.full((h, w), 205, dtype=np.uint8)
            img[raw == 0] = 254
            img[raw > 50] = 0

            # Flip vertically since ROS grid row 0 is bottom (y_min)
            img = np.flipud(img)
            _, png = cv2.imencode('.png', img)

            with self.lock:
                self.live_map_png = png.tobytes()
                self.live_map_meta = {
                    "resolution": float(res),
                    "origin": [float(ox), float(oy), 0.0],
                    "width": int(w),
                    "height": int(h),
                    "occupied_thresh": 0.65,
                    "free_thresh": 0.196,
                    "image": "live_map"
                }
                self.last_map_time = time.time()

            self._broadcast_ws({"event": "map_update", "timestamp": self.last_map_time})
        except Exception as e:
            self.get_logger().error(f"Error converting OccupancyGrid /map to live PNG: {e}")

    def _broadcast_ws(self, payload: dict):
        if self.async_loop and self.async_loop.is_running() and self.active_websockets:
            msg_str = json.dumps(payload)
            asyncio.run_coroutine_threadsafe(
                self._broadcast_async(msg_str), self.async_loop
            )

    async def _broadcast_async(self, text: str):
        dead_sockets = set()
        for ws in list(self.active_websockets):
            try:
                await ws.send_text(text)
            except Exception:
                dead_sockets.add(ws)
        for ws in dead_sockets:
            self.active_websockets.discard(ws)

    def publish_operator_command(self, cmd_json_str: str):
        msg = RosString()
        msg.data = cmd_json_str
        self.pub_operator_cmd.publish(msg)
        self.get_logger().info(f"[COMMAND CENTER] Published operator command: {cmd_json_str}")

    def publish_teleop(self, linear_x: float, angular_z: float):
        try:
            lx = float(linear_x)
            az = float(angular_z)
            if not (np.isfinite(lx) and np.isfinite(az)):
                return
            twist = Twist()
            # Strict velocity clamping: linear_x in [-0.8, 0.8], angular_z in [-1.5, 1.5]
            twist.linear.x = float(np.clip(lx, -0.8, 0.8))
            twist.angular.z = float(np.clip(az, -1.5, 1.5))
            self.pub_cmd_vel.publish(twist)
        except Exception as e:
            self.get_logger().warn(f"Invalid teleop command: {e}")

    def get_jpeg(self, camera: str) -> bytes:
        with self.lock:
            return self.front_jpeg if camera == "front" else self.rear_jpeg


def create_app(node: CommandCenterBridge) -> FastAPI:
    app = FastAPI(
        title="AMR Security Command Center API",
        description="REST & Real-Time WebSocket API for Autonomous Warehouse Security AMR",
        version="1.0.0"
    )

    # Restrict CORS to localhost/same-origin to prevent Cross-Site WebSocket/Request Hijacking
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://localhost:8080",
            "http://127.0.0.1:8080",
            "http://localhost:8000",
            "http://127.0.0.1:8000",
        ],
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )

    @app.on_event("startup")
    async def startup_event():
        node.async_loop = asyncio.get_running_loop()
        node.get_logger().info("FastAPI Event Loop bound to ROS Bridge.")

    # 1. Dual MJPEG Video Streams
    async def mjpeg_generator(camera: str):
        while True:
            frame_bytes = node.get_jpeg(camera)
            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n'
                b'Content-Length: ' + str(len(frame_bytes)).encode() + b'\r\n\r\n' +
                frame_bytes + b'\r\n'
            )
            await asyncio.sleep(0.05)  # 20 FPS

    @app.get("/stream/front", summary="Live Front Camera MJPEG Stream")
    async def stream_front():
        return StreamingResponse(
            mjpeg_generator("front"),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )

    @app.get("/stream/rear", summary="Live Rear Camera MJPEG Stream")
    async def stream_rear():
        return StreamingResponse(
            mjpeg_generator("rear"),
            media_type="multipart/x-mixed-replace; boundary=frame"
        )

    # Internal helper for rate-limited map reset
    async def execute_map_reset() -> dict:
        now = time.time()
        elapsed = now - node.last_map_reset_time
        if elapsed < 15.0:
            remaining = int(15.0 - elapsed)
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Rate limit active: Map reset cooldown in effect ({remaining}s remaining)."
            )
        node.last_map_reset_time = now

        try:
            # 1. Delete saved map files from both src and install
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
                        node.get_logger().info(f"[MAP RESET] Deleted: {p}")
                    except Exception as e:
                        node.get_logger().warn(f"Failed deleting {p}: {e}")

            # 2. Clear cached map state and reset progress
            with node.lock:
                node.live_map_png = None
                node.live_map_meta = None
                node.latest_explore_progress = {"progress": 0.0, "stage": "STAGE 1: MAPPING"}

            # 3. Publish to /mission/reset to restart Phase 1 SLAM
            msg = RosString()
            msg.data = "RESET"
            node.pub_mission_reset.publish(msg)
            node.get_logger().info("[MAP RESET] Published reset to /mission/reset.")

            # 4. Broadcast reset events over WebSocket
            node._broadcast_ws({"event": "explore_progress", "data": {"progress": 0.0, "stage": "STAGE 1: MAPPING"}})
            node._broadcast_ws({"event": "mission_stage", "data": "STAGE 1: MAPPING"})
            node._broadcast_ws({"event": "map_reset", "data": {"status": "success"}})

            return {
                "success": True,
                "message": "Map reset initiated. Starting Phase 1 autonomous mapping from (0,0)."
            }
        except HTTPException:
            raise
        except Exception as e:
            node.get_logger().error(f"Error handling map reset: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to reset map: {e}")

    # 2. Authenticated Real-Time WebSocket (Policy Violation 1008 on unauthorized)
    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket, token: Optional[str] = None):
        # Authenticate via query param ?token=... or header X-API-Key
        auth_token = token or ws.query_params.get("token") or ws.headers.get("x-api-key")
        if not auth_token or auth_token != OPERATOR_TOKEN:
            node.get_logger().warn(
                f"[SECURITY] Unauthorized WebSocket connection attempt rejected from {ws.client.host if ws.client else 'unknown'}"
            )
            # Standard RFC 6455 close code 1008: Policy Violation
            await ws.close(code=1008, reason="Policy Violation: Unauthorized Operator Token")
            return

        await ws.accept()
        node.active_websockets.add(ws)

        # Send latest telemetry immediately if available
        if node.latest_telemetry:
            await ws.send_text(json.dumps({"event": "telemetry", "data": node.latest_telemetry}))
        if node.latest_explore_progress:
            await ws.send_text(json.dumps({"event": "explore_progress", "data": node.latest_explore_progress}))

        try:
            while True:
                data = await ws.receive_json()
                msg_type = data.get("type", "")
                if msg_type == "operator_command":
                    payload = data.get("payload", {})
                    if not isinstance(payload, dict):
                        continue
                    action = str(payload.get("action", "")).upper().strip()
                    if action == "RESET_MAP":
                        try:
                            await execute_map_reset()
                        except HTTPException as he:
                            await ws.send_text(json.dumps({"event": "error", "detail": he.detail}))
                    elif action == "GOTO_POSE":
                        try:
                            gx = float(payload.get("x", 0.0))
                            gy = float(payload.get("y", 0.0))
                            gyaw = float(payload.get("yaw", 0.0))
                            if np.isfinite(gx) and np.isfinite(gy) and np.isfinite(gyaw):
                                # Clamp navigation coordinates to warehouse bounds
                                payload["x"] = float(np.clip(gx, -50.0, 50.0))
                                payload["y"] = float(np.clip(gy, -50.0, 50.0))
                                payload["yaw"] = float(np.clip(gyaw, -6.29, 6.29))
                                node.publish_operator_command(json.dumps(payload))
                        except Exception as e:
                            node.get_logger().warn(f"Malformed GOTO_POSE coordinates: {e}")
                    else:
                        node.publish_operator_command(json.dumps(payload))
                elif msg_type == "teleop":
                    try:
                        lx = float(data.get("linear_x", 0.0))
                        az = float(data.get("angular_z", 0.0))
                        node.publish_teleop(lx, az)
                    except (ValueError, TypeError):
                        pass
                elif msg_type == "reset_map":
                    try:
                        await execute_map_reset()
                    except HTTPException as he:
                        await ws.send_text(json.dumps({"event": "error", "detail": he.detail}))
                elif msg_type == "ping":
                    await ws.send_text(json.dumps({"event": "pong", "time": time.time()}))
        except WebSocketDisconnect:
            pass
        except Exception as e:
            node.get_logger().warn(f"WebSocket client error: {e}")
        finally:
            node.active_websockets.discard(ws)

    # 3. REST API: Map Metadata & Floorplan Image
    @app.get("/api/map/metadata", summary="Warehouse Map Metadata")
    async def get_map_metadata():
        with node.lock:
            if node.live_map_meta:
                return node.live_map_meta

        if not os.path.exists(node.map_yaml):
            return {
                "resolution": 0.05,
                "origin": [-7.921, -6.627, 0.0],
                "width": 605,
                "height": 303,
                "image": "warehouse_map.pgm"
            }
        try:
            with open(node.map_yaml, 'r') as f:
                data = yaml.safe_load(f)
            pgm_path = node.map_pgm if os.path.exists(node.map_pgm) else os.path.join(os.path.dirname(node.map_yaml), data.get('image', ''))
            width, height = 605, 303
            if os.path.exists(pgm_path):
                with PILImage.open(pgm_path) as im:
                    width, height = im.size
            return {
                "resolution": float(data.get("resolution", 0.05)),
                "origin": [float(x) for x in data.get("origin", [-7.921, -6.627, 0.0])],
                "width": width,
                "height": height,
                "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
                "free_thresh": float(data.get("free_thresh", 0.196)),
                "image": str(data.get("image", "warehouse_map.pgm"))
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to read map metadata: {e}")

    @app.get("/api/map/image", summary="Warehouse Floorplan PNG Image")
    async def get_map_image():
        with node.lock:
            if node.live_map_png:
                return Response(content=node.live_map_png, media_type="image/png")

        pgm_path = node.map_pgm
        if not os.path.exists(pgm_path):
            pgm_path = os.path.join(os.path.dirname(node.map_yaml), "warehouse_map.pgm")
        if not os.path.exists(pgm_path):
            raise HTTPException(status_code=404, detail="Map PGM file not found.")

        try:
            with PILImage.open(pgm_path) as im:
                buf = io.BytesIO()
                im.save(buf, format="PNG")
                png_bytes = buf.getvalue()
            return Response(content=png_bytes, media_type="image/png")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to convert map PGM to PNG: {e}")

    @app.post("/api/map/reset", summary="Reset Map & Re-SLAM Autonomous Mapping")
    async def reset_map_endpoint(token: str = Depends(verify_operator_token)):
        return await execute_map_reset()

    # 4. REST API: Security Incident Logs & Operator Status Updates
    @app.get("/api/incidents", summary="Get Latest Forensic Security Incidents")
    async def get_incidents(limit: int = 50):
        try:
            safe_limit = max(1, min(int(limit), 200))
            with sqlite3.connect(node.db_path) as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT id, timestamp, camera_source, classification,
                           pos_x, pos_y, confidence, snapshot_path, operator_status
                    FROM incidents
                    ORDER BY id DESC
                    LIMIT ?
                """, (safe_limit,))
                rows = cursor.fetchall()
                incidents = []
                for r in rows:
                    item = dict(r)
                    if item.get("snapshot_path"):
                        # Format relative URL with sanitized basename to prevent path traversal
                        fname = os.path.basename(item["snapshot_path"])
                        item["snapshot_url"] = f"/snapshots/{fname}"
                    incidents.append(item)
                return {"incidents": incidents}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Database query error: {e}")

    @app.post("/api/incidents/{incident_id}/status", summary="Update Incident Operator Status")
    async def update_incident_status(
        incident_id: int,
        payload: IncidentStatusUpdate = Body(...),
        token: str = Depends(verify_operator_token)
    ):
        status_val = payload.status.upper().strip()
        if status_val not in ["CONFIRMED_INTRUDER", "FALSE_ALARM", "AUTHORIZED", "RESOLVED", "PENDING"]:
            raise HTTPException(status_code=400, detail="Invalid operator status.")

        try:
            safe_id = int(incident_id)
            with sqlite3.connect(node.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    UPDATE incidents
                    SET operator_status = ?
                    WHERE id = ?
                """, (status_val, safe_id))
                conn.commit()

            # Forward corresponding action to ROS 2 FSM
            action_name = status_val
            node.publish_operator_command(json.dumps({
                "action": action_name,
                "incident_id": safe_id
            }))

            return {
                "success": True,
                "incident_id": safe_id,
                "operator_status": status_val
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Database update error: {e}")

    @app.post("/api/operator/command", summary="Dispatch Operator Command to ROS 2 FSM")
    async def operator_command_endpoint(
        payload: dict = Body(...),
        token: str = Depends(verify_operator_token)
    ):
        try:
            action = str(payload.get("action", "")).upper().strip()
            if action == "RESET_MAP":
                return await execute_map_reset()
            elif action == "GOTO_POSE":
                try:
                    gx = float(payload.get("x", 0.0))
                    gy = float(payload.get("y", 0.0))
                    gyaw = float(payload.get("yaw", 0.0))
                    if np.isfinite(gx) and np.isfinite(gy) and np.isfinite(gyaw):
                        payload["x"] = float(np.clip(gx, -50.0, 50.0))
                        payload["y"] = float(np.clip(gy, -50.0, 50.0))
                        payload["yaw"] = float(np.clip(gyaw, -6.29, 6.29))
                except Exception:
                    pass

            cmd_str = json.dumps(payload)
            node.publish_operator_command(cmd_str)
            return {"success": True, "command": payload}
        except HTTPException:
            raise
        except Exception as e:
            node.get_logger().error(f"Error handling operator command API: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to dispatch operator command: {e}")

    @app.post("/api/auth/verify", summary="Verify Operator Token")
    async def verify_auth_endpoint(token: str = Depends(verify_operator_token)):
        return {
            "authenticated": True,
            "operator": "AUTHORIZED",
            "server_time": datetime.now().isoformat()
        }

    # Path-traversal safe snapshot download endpoint
    @app.get("/api/snapshots/{filename}", summary="Download Forensic Snapshot")
    async def get_snapshot_file(filename: str):
        safe_filename = os.path.basename(filename)
        target_path = os.path.realpath(os.path.join(node.snapshot_dir, safe_filename))
        real_snap_dir = os.path.realpath(node.snapshot_dir)
        if not target_path.startswith(real_snap_dir) or not os.path.exists(target_path):
            raise HTTPException(status_code=404, detail="Forensic snapshot not found.")
        return FileResponse(target_path, media_type="image/jpeg")

    @app.get("/api/health", summary="Command Center Health Check")
    async def health():
        return {
            "status": "online",
            "security_hardened": True,
            "server_time": datetime.now().isoformat(),
            "active_ws_clients": len(node.active_websockets),
            "front_camera_active": (time.time() - node.last_front_time) < 2.0,
            "rear_camera_active": (time.time() - node.last_rear_time) < 2.0
        }
        return {
            "status": "online",
            "server_time": datetime.now().isoformat(),
            "active_ws_clients": len(node.active_websockets),
            "front_camera_active": (time.time() - node.last_front_time) < 2.0,
            "rear_camera_active": (time.time() - node.last_rear_time) < 2.0
        }

    # Mount snapshots static folder
    if os.path.exists(node.snapshot_dir):
        app.mount("/snapshots", StaticFiles(directory=node.snapshot_dir), name="snapshots")

    # Determine web frontend static directory
    web_dir = None
    try:
        pkg_share = get_package_share_directory('amr_command_center')
        candidate = os.path.join(pkg_share, 'web')
        if os.path.exists(candidate):
            web_dir = candidate
    except Exception:
        pass

    if not web_dir:
        src_web = "/home/mmdixm/Desktop/amr_security_ws/src/amr_command_center/web"
        if os.path.exists(src_web):
            web_dir = src_web

    @app.get("/", summary="Command Center Dashboard")
    async def index_page():
        if web_dir:
            index_path = os.path.join(web_dir, "index.html")
            if os.path.exists(index_path):
                return FileResponse(index_path, media_type="text/html")
        return {"status": "online", "message": "AMR Command Center Backend Active"}

    if web_dir and os.path.exists(web_dir):
        app.mount("/static", StaticFiles(directory=web_dir), name="static")

    return app


def main(args=None):
    rclpy.init(args=args)
    bridge_node = CommandCenterBridge()

    # Spin ROS 2 in a background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(bridge_node,), daemon=True)
    ros_thread.start()

    app = create_app(bridge_node)
    config = uvicorn.Config(
        app,
        host=bridge_node.host,
        port=bridge_node.port,
        log_level="info",
        access_log=False
    )
    server = uvicorn.Server(config)

    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
