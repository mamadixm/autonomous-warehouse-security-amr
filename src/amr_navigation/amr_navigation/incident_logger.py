#!/usr/bin/env python3
"""
SQLite Incident & Blackbox Logger (incident_logger.py)
Manages persistent forensic logging of security events and camera snapshots
for the Autonomous Warehouse Security AMR Command Center.
"""

import os
import sqlite3
from datetime import datetime
from typing import Optional, Dict, Any, List
import cv2
import numpy as np


class IncidentLogger:
    def __init__(
        self,
        db_path: str = "/home/mmdixm/Desktop/amr_security_ws/data/security_incidents.db",
        snapshot_dir: str = "/home/mmdixm/Desktop/amr_security_ws/data/snapshots"
    ):
        self.db_path = os.path.expanduser(db_path)
        self.snapshot_dir = os.path.expanduser(snapshot_dir)

        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        os.makedirs(self.snapshot_dir, exist_ok=True)

        self._init_db()

    def _get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=10.0)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self):
        with self._get_connection() as conn:
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

    def save_snapshot(
        self,
        cv_image: np.ndarray,
        camera_source: str,
        timestamp_str: Optional[str] = None
    ) -> Optional[str]:
        if cv_image is None or cv_image.size == 0:
            return None

        if timestamp_str is None:
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:19]

        clean_ts = timestamp_str.replace(":", "-").replace(".", "_")
        filename = f"snapshot_{camera_source}_{clean_ts}.jpg"
        full_path = os.path.join(self.snapshot_dir, filename)

        try:
            cv2.imwrite(full_path, cv_image)
            return full_path
        except Exception as e:
            print(f"[IncidentLogger] Failed to save snapshot: {e}")
            return None

    def log_incident(
        self,
        camera_source: str,
        classification: str,
        pos_x: float = 0.0,
        pos_y: float = 0.0,
        confidence: float = 1.0,
        annotated_image: Optional[np.ndarray] = None,
        operator_status: str = "PENDING"
    ) -> int:
        timestamp_iso = datetime.now().isoformat()
        snapshot_path = None

        if annotated_image is not None:
            snapshot_path = self.save_snapshot(
                cv_image=annotated_image,
                camera_source=camera_source,
                timestamp_str=timestamp_iso
            )

        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO incidents (
                    timestamp, camera_source, classification,
                    pos_x, pos_y, confidence, snapshot_path, operator_status
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                timestamp_iso,
                camera_source,
                classification,
                float(pos_x),
                float(pos_y),
                float(confidence),
                snapshot_path,
                operator_status
            ))
            conn.commit()
            return cursor.lastrowid

    def update_operator_status(self, incident_id: int, status: str) -> bool:
        valid_statuses = {"PENDING", "CONFIRMED_INTRUDER", "FALSE_ALARM", "RESOLVED", "AUTHORIZED"}
        status_clean = status.upper().strip()
        if status_clean not in valid_statuses:
            status_clean = "PENDING"

        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                UPDATE incidents
                SET operator_status = ?
                WHERE id = ?
            """, (status_clean, int(incident_id)))
            conn.commit()
            return cursor.rowcount > 0

    def get_recent_incidents(self, limit: int = 20) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                SELECT id, timestamp, camera_source, classification,
                       pos_x, pos_y, confidence, snapshot_path, operator_status
                FROM incidents
                ORDER BY id DESC
                LIMIT ?
            """, (limit,))
            rows = cursor.fetchall()
            return [dict(row) for row in rows]

    def get_incident_by_id(self, incident_id: int) -> Optional[Dict[str, Any]]:
        with self._get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
                SELECT id, timestamp, camera_source, classification,
                       pos_x, pos_y, confidence, snapshot_path, operator_status
                FROM incidents
                WHERE id = ?
            """, (incident_id,))
            row = cursor.fetchone()
            return dict(row) if row else None
