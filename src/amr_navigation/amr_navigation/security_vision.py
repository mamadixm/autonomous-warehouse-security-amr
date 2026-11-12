#!/usr/bin/env python3
"""
Dual-Camera Security AI & Anomaly Analyzer (security_vision.py)
Performs multi-camera image processing, ArUco badge verification,
vest HSV color classification, HOG/MOG2 motion detection, and
Authorized Security vs. Unauthorized Intruder classification.
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple, List, Dict, Any
import math
import numpy as np
import cv2


@dataclass
class VisionDetectionResult:
    camera_source: str              # "front" or "rear"
    classification: Optional[str]   # "AUTHORIZED_SECURITY", "UNAUTHORIZED_INTRUDER", or None
    confidence: float               # 0.0 to 1.0
    bearing_error_rad: float        # angular offset from camera center axis
    pixel_err_x: int                # pixel offset (img_center_x - cx)
    bbox: Optional[Tuple[int, int, int, int]]  # (x, y, w, h)
    annotated_frame: np.ndarray     # image with overlaid forensic HUD
    target_center: Tuple[int, int]  # (cx, cy)
    details: str                    # descriptive detection method notes


class SecurityVisionAnalyzer:
    def __init__(
        self,
        authorized_vest_lower: Tuple[int, int, int] = (35, 70, 70),
        authorized_vest_upper: Tuple[int, int, int] = (85, 255, 255),
        unauthorized_red_lower1: Tuple[int, int, int] = (0, 100, 50),
        unauthorized_red_upper1: Tuple[int, int, int] = (10, 255, 255),
        unauthorized_red_lower2: Tuple[int, int, int] = (170, 100, 50),
        unauthorized_red_upper2: Tuple[int, int, int] = (180, 255, 255),
        authorized_aruco_ids: Optional[List[int]] = None,
        min_detection_area: int = 400,
        camera_hfov_rad: float = 1.3962634  # 80 degrees
    ):
        self.vest_lower = np.array(authorized_vest_lower, dtype=np.uint8)
        self.vest_upper = np.array(authorized_vest_upper, dtype=np.uint8)

        self.red_lower1 = np.array(unauthorized_red_lower1, dtype=np.uint8)
        self.red_upper1 = np.array(unauthorized_red_upper1, dtype=np.uint8)
        self.red_lower2 = np.array(unauthorized_red_lower2, dtype=np.uint8)
        self.red_upper2 = np.array(unauthorized_red_upper2, dtype=np.uint8)

        self.authorized_aruco_ids = set(authorized_aruco_ids or [0, 1, 2, 42, 100])
        self.min_detection_area = min_detection_area
        # Frame counters for background learning warmup
        self.frame_count_front = 0
        self.frame_count_rear = 0
        self.camera_hfov = camera_hfov_rad

        # Separate background subtractors for front and rear feeds
        self.bg_subtractor_front = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=35, detectShadows=False
        )
        self.bg_subtractor_rear = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=35, detectShadows=False
        )

        # ArUco dictionary (DICT_4X4_50)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

    def _detect_aruco(self, gray: np.ndarray) -> Tuple[List[Any], Optional[np.ndarray]]:
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
            return corners, ids
        except Exception:
            return [], None

    def analyze_frame(
        self,
        frame: np.ndarray,
        camera_source: str = "front",
        is_after_hours: bool = True,
        rfid_whitelisted: bool = False
    ) -> VisionDetectionResult:
        if frame is None or frame.size == 0:
            return VisionDetectionResult(
                camera_source=camera_source,
                classification=None,
                confidence=0.0,
                bearing_error_rad=0.0,
                pixel_err_x=0,
                bbox=None,
                annotated_frame=np.zeros((100, 100, 3), dtype=np.uint8),
                target_center=(0, 0),
                details="Empty frame"
            )

        h, w = frame.shape[:2]
        center_x = w // 2
        center_y = h // 2
        annotated = frame.copy()

        # Step 1: ArUco Security Badge Detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect_aruco(gray)

        if ids is not None and len(ids) > 0:
            for idx, marker_id in enumerate(ids.flatten()):
                if int(marker_id) in self.authorized_aruco_ids:
                    c = corners[idx][0]
                    cx = int(c[:, 0].mean())
                    cy = int(c[:, 1].mean())
                    min_x, max_x = int(c[:, 0].min()), int(c[:, 0].max())
                    min_y, max_y = int(c[:, 1].min()), int(c[:, 1].max())
                    bbox = (min_x, min_y, max_x - min_x, max_y - min_y)

                    pixel_err_x = center_x - cx
                    bearing_rad = -((cx - center_x) / (center_x)) * (self.camera_hfov / 2.0)

                    # Annotate badge
                    cv2.polylines(annotated, [c.astype(np.int32)], True, (0, 255, 0), 2)
                    cv2.putText(
                        annotated,
                        f"AUTH BADGE #{marker_id}",
                        (min_x, max(20, min_y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    self._draw_hud(annotated, camera_source, "AUTHORIZED_SECURITY", 0.98, is_after_hours)

                    return VisionDetectionResult(
                        camera_source=camera_source,
                        classification="AUTHORIZED_SECURITY",
                        confidence=0.98,
                        bearing_error_rad=bearing_rad,
                        pixel_err_x=pixel_err_x,
                        bbox=bbox,
                        annotated_frame=annotated,
                        target_center=(cx, cy),
                        details=f"Valid ArUco badge #{marker_id} detected"
                    )

        # Step 2: HSV Color Analysis
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2a: Authorized Vest (High-Vis Green / Cyan / Blue)
        vest_mask = cv2.inRange(hsv, self.vest_lower, self.vest_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        vest_mask = cv2.morphologyEx(vest_mask, cv2.MORPH_OPEN, kernel)
        vest_contours, _ = cv2.findContours(vest_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_vest_c = None
        max_vest_area = 0.0
        for c in vest_contours:
            area = cv2.contourArea(c)
            if area > self.min_detection_area and area > max_vest_area:
                max_vest_area = area
                best_vest_c = c

        # 2b: Unauthorized Intruder Hazard Red Signature
        mask_r1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask_r2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_red_c = None
        max_red_area = 0.0
        for c in red_contours:
            area = cv2.contourArea(c)
            if area > self.min_detection_area and area > max_red_area:
                max_red_area = area
                best_red_c = c

        # Whitelist by active RFID beacon
        if rfid_whitelisted:
            bbox = (center_x - 40, center_y - 40, 80, 80)
            cx, cy = center_x, center_y
            pixel_err_x = 0
            bearing_rad = 0.0
            conf = 0.90
            details = "Simulated RFID beacon whitelist active"
            cv2.rectangle(annotated, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"AUTHORIZED STAFF ({conf*100:.0f}%)",
                (bbox[0], max(25, bbox[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            self._draw_hud(annotated, camera_source, "AUTHORIZED_SECURITY", conf, is_after_hours)
            return VisionDetectionResult(
                camera_source=camera_source,
                classification="AUTHORIZED_SECURITY",
                confidence=conf,
                bearing_error_rad=bearing_rad,
                pixel_err_x=pixel_err_x,
                bbox=bbox,
                annotated_frame=annotated,
                target_center=(cx, cy),
                details=details
            )

        # Disambiguate: if red intruder signature is dominant over vest reflections
        if best_red_c is not None and max_red_area >= max_vest_area:
            x, y, bw, bh = cv2.boundingRect(best_red_c)
            cx = x + bw // 2
            cy = y + bh // 2
            bbox = (x, y, bw, bh)
            pixel_err_x = center_x - cx
            bearing_rad = -((cx - center_x) / (center_x)) * (self.camera_hfov / 2.0)
            conf = min(0.99, 0.85 + (max_red_area / (w * h)) * 1.5)
            classification = "UNAUTHORIZED_INTRUDER"
            details = f"Hazard red anomaly detected (area={int(max_red_area)})"

            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.drawMarker(annotated, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(
                annotated,
                f"INTRUDER ALERT ({conf*100:.0f}%)",
                (x, max(25, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )
            self._draw_hud(annotated, camera_source, classification, conf, is_after_hours)

            return VisionDetectionResult(
                camera_source=camera_source,
                classification=classification,
                confidence=conf,
                bearing_error_rad=bearing_rad,
                pixel_err_x=pixel_err_x,
                bbox=bbox,
                annotated_frame=annotated,
                target_center=(cx, cy),
                details=details
            )

        # If authorized vest is dominant
        if best_vest_c is not None and max_vest_area > self.min_detection_area:
            x, y, bw, bh = cv2.boundingRect(best_vest_c)
            cx = x + bw // 2
            cy = y + bh // 2
            bbox = (x, y, bw, bh)
            pixel_err_x = center_x - cx
            bearing_rad = -((cx - center_x) / (center_x)) * (self.camera_hfov / 2.0)
            conf = min(0.95, 0.60 + (max_vest_area / (w * h)) * 2.0)
            details = f"Authorized security vest detected (area={int(max_vest_area)})"

            cv2.rectangle(annotated, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"AUTHORIZED STAFF ({conf*100:.0f}%)",
                (bbox[0], max(25, bbox[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            self._draw_hud(annotated, camera_source, "AUTHORIZED_SECURITY", conf, is_after_hours)

            return VisionDetectionResult(
                camera_source=camera_source,
                classification="AUTHORIZED_SECURITY",
                confidence=conf,
                bearing_error_rad=bearing_rad,
                pixel_err_x=pixel_err_x,
                bbox=bbox,
                annotated_frame=annotated,
                target_center=(cx, cy),
                details=details
            )

        # Step 3: Motion / Foreground fallback
        if camera_source == "front":
            self.frame_count_front += 1
            warmup = self.frame_count_front <= 5
            subtractor = self.bg_subtractor_front
        else:
            self.frame_count_rear += 1
            warmup = self.frame_count_rear <= 5
            subtractor = self.bg_subtractor_rear

        fg_mask = subtractor.apply(frame)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)

        best_motion_c = None
        max_motion_area = 0.0

        # Only evaluate motion after warmup frames and ignore full-frame transitions
        if not warmup:
            motion_contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in motion_contours:
                area = cv2.contourArea(c)
                # Discard entire-screen flashes or background initializations
                if area > (self.min_detection_area * 1.5) and area < (0.50 * w * h) and area > max_motion_area:
                    max_motion_area = area
                    best_motion_c = c

        # Determine if an unauthorized threat is present
        target_c = best_red_c if best_red_c is not None else best_motion_c
        threat_area = max_red_area if best_red_c is not None else max_motion_area

        if target_c is not None and threat_area > self.min_detection_area:
            x, y, bw, bh = cv2.boundingRect(target_c)
            cx = x + bw // 2
            cy = y + bh // 2
            bbox = (x, y, bw, bh)
            pixel_err_x = center_x - cx
            bearing_rad = -((cx - center_x) / (center_x)) * (self.camera_hfov / 2.0)

            # Confidence calculation based on area and color vs motion
            base_conf = 0.85 if best_red_c is not None else 0.70
            conf = min(0.99, base_conf + (threat_area / (w * h)) * 1.5)

            # Enforce shift policy: after-hours non-badged = UNAUTHORIZED_INTRUDER
            classification = "UNAUTHORIZED_INTRUDER" if is_after_hours else "UNAUTHORIZED_INTRUDER"
            details = (
                f"Hazard red anomaly detected (area={int(threat_area)})"
                if best_red_c is not None
                else f"Unbadged motion signature in restricted zone (area={int(threat_area)})"
            )

            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            cv2.drawMarker(annotated, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
            cv2.putText(
                annotated,
                f"INTRUDER ALERT ({conf*100:.0f}%)",
                (x, max(25, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )
            self._draw_hud(annotated, camera_source, classification, conf, is_after_hours)

            return VisionDetectionResult(
                camera_source=camera_source,
                classification=classification,
                confidence=conf,
                bearing_error_rad=bearing_rad,
                pixel_err_x=pixel_err_x,
                bbox=bbox,
                annotated_frame=annotated,
                target_center=(cx, cy),
                details=details
            )

        # Normal scene - No Threat
        self._draw_hud(annotated, camera_source, "CLEAR", 0.0, is_after_hours)
        return VisionDetectionResult(
            camera_source=camera_source,
            classification=None,
            confidence=0.0,
            bearing_error_rad=0.0,
            pixel_err_x=0,
            bbox=None,
            annotated_frame=annotated,
            target_center=(center_x, center_y),
            details="All clear, no security anomalies"
        )

    def _draw_hud(
        self,
        img: np.ndarray,
        camera_source: str,
        status: str,
        confidence: float,
        is_after_hours: bool
    ):
        h, w = img.shape[:2]
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        mode_str = "AFTER-HOURS RESTRICTED" if is_after_hours else "DAY SHIFT"

        # Top banner
        header_text = f"CAM: {camera_source.upper()} | {mode_str} | {now_str}"
        cv2.putText(img, header_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)

        # Status indicator
        color = (0, 255, 0) if status in ["CLEAR", "AUTHORIZED_SECURITY"] else (0, 0, 255)
        status_text = f"STATUS: {status}" + (f" ({confidence*100:.0f}%)" if confidence > 0 else "")
        cv2.putText(img, status_text, (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
