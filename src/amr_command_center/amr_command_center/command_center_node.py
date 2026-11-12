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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String as RosString
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Body
from fastapi.responses import StreamingResponse, Response, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn


class CommandCenterBridge(Node):
    def __init__(self):
        super().__init__('amr_command_center_node')

        # Declare ROS 2 parameters
        self.declare_parameter('port', 8000)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('db_path', '/home/mmdixm/amr_project_ws/data/security_incidents.db')
        self.declare_parameter('snapshot_dir', '/home/mmdixm/amr_project_ws/data/snapshots')
        self.declare_parameter('map_yaml', '/home/mmdixm/amr_project_ws/src/amr_navigation/maps/warehouse_map.yaml')
        self.declare_parameter('map_pgm', '/home/mmdixm/amr_project_ws/src/amr_navigation/maps/warehouse_map.pgm')

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

        # Asyncio event loop reference (set on FastAPI startup)
        self.async_loop: Optional[asyncio.AbstractEventLoop] = None
        self.active_websockets: Set[WebSocket] = set()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
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

        # Publishers
        self.pub_operator_cmd = self.create_publisher(RosString, '/security/operator_command', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

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
        twist = Twist()
        twist.linear.x = float(np.clip(linear_x, -0.6, 0.6))
        twist.angular.z = float(np.clip(angular_z, -1.5, 1.5))
        self.pub_cmd_vel.publish(twist)

    def get_jpeg(self, camera: str) -> bytes:
        with self.lock:
            return self.front_jpeg if camera == "front" else self.rear_jpeg


def create_app(node: CommandCenterBridge) -> FastAPI:
    app = FastAPI(
        title="AMR Security Command Center API",
        description="REST & Real-Time WebSocket API for Autonomous Warehouse Security AMR",
        version="1.0.0"
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
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

    # 2. Real-Time WebSocket
    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()
        node.active_websockets.add(ws)
        # Send latest telemetry immediately if available
        if node.latest_telemetry:
            await ws.send_text(json.dumps({"event": "telemetry", "data": node.latest_telemetry}))
        try:
            while True:
                data = await ws.receive_json()
                msg_type = data.get("type", "")
                if msg_type == "operator_command":
                    payload = data.get("payload", {})
                    node.publish_operator_command(json.dumps(payload))
                elif msg_type == "teleop":
                    lx = float(data.get("linear_x", 0.0))
                    az = float(data.get("angular_z", 0.0))
                    node.publish_teleop(lx, az)
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
            # Read image dimensions
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
        pgm_path = node.map_pgm
        if not os.path.exists(pgm_path):
            # Try finding alongside map_yaml
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

    # 4. REST API: Security Incident Logs & Operator Status Updates
    @app.get("/api/incidents", summary="Get Latest Forensic Security Incidents")
    async def get_incidents(limit: int = 50):
        try:
            with sqlite3.connect(node.db_path) as conn:
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT id, timestamp, camera_source, classification,
                           pos_x, pos_y, confidence, snapshot_path, operator_status
                    FROM incidents
                    ORDER BY id DESC
                    LIMIT ?
                """, (limit,))
                rows = cursor.fetchall()
                incidents = []
                for r in rows:
                    item = dict(r)
                    if item.get("snapshot_path"):
                        # Format relative URL for frontend display
                        fname = os.path.basename(item["snapshot_path"])
                        item["snapshot_url"] = f"/snapshots/{fname}"
                    incidents.append(item)
                return {"incidents": incidents}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Database query error: {e}")

    @app.post("/api/incidents/{incident_id}/status", summary="Update Incident Operator Status")
    async def update_incident_status(incident_id: int, payload: dict = Body(...)):
        status = payload.get("status", "").upper().strip()
        if status not in ["CONFIRMED_INTRUDER", "FALSE_ALARM", "AUTHORIZED", "RESOLVED", "PENDING"]:
            raise HTTPException(status_code=400, detail="Invalid operator status.")

        try:
            with sqlite3.connect(node.db_path) as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    UPDATE incidents
                    SET operator_status = ?
                    WHERE id = ?
                """, (status, incident_id))
                conn.commit()

            # Forward corresponding action to ROS 2 FSM
            action_name = status
            node.publish_operator_command(json.dumps({
                "action": action_name,
                "incident_id": incident_id
            }))

            return {
                "success": True,
                "incident_id": incident_id,
                "operator_status": status
            }
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Database update error: {e}")

    @app.get("/api/health", summary="Command Center Health Check")
    async def health():
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
        src_web = "/home/mmdixm/amr_project_ws/src/amr_command_center/web"
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
