"""
ArUco marker generation and detection for room-level localization.

Uses DICT_4X4_50 — compact markers reliable at low camera resolutions.
Compatible with OpenCV 4.0+ (handles API differences between versions).
"""

import cv2
import os
import numpy as np

ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
MARKER_SIZE_PX = 400
MARKER_SIZE_CM = 15.0

MARKERS_DIR = os.path.join(os.path.dirname(__file__), '..', 'markers')

try:
    _dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    HAS_ARUCO = True
except AttributeError:
    HAS_ARUCO = False


def generate_markers(room_markers, output_dir=None):
    """Generate printable ArUco marker PNGs.

    room_markers: list of (marker_id, room_label) tuples.
    Returns output directory path.
    """
    if not HAS_ARUCO:
        print("[ArUco] cv2.aruco unavailable — install opencv-contrib-python")
        return None

    output_dir = output_dir or MARKERS_DIR
    os.makedirs(output_dir, exist_ok=True)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

    for marker_id, label in room_markers:
        try:
            img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PX)
        except AttributeError:
            img = cv2.aruco.drawMarker(aruco_dict, marker_id, MARKER_SIZE_PX)

        bordered = np.ones((MARKER_SIZE_PX + 120, MARKER_SIZE_PX + 60), dtype=np.uint8) * 255
        bordered[30:30 + MARKER_SIZE_PX, 30:30 + MARKER_SIZE_PX] = img

        cv2.putText(bordered, f"ID:{marker_id} - {label}",
                     (30, MARKER_SIZE_PX + 70),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
        cv2.putText(bordered, "Mount at ~15cm height on wall",
                     (30, MARKER_SIZE_PX + 100),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, 100, 1)

        fname = f"marker_{marker_id}_{label.lower().replace(' ', '_')}.png"
        path = os.path.join(output_dir, fname)
        cv2.imwrite(path, bordered)
        print(f"[ArUco] Saved {path}")

    return output_dir


class ArucoDetector:
    """Stateless ArUco marker detector. Thread-safe (no mutable state)."""

    def __init__(self):
        if not HAS_ARUCO:
            self._enabled = False
            return
        self._enabled = True
        self._dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self._params = cv2.aruco.DetectorParameters()
        try:
            self._detector = cv2.aruco.ArucoDetector(self._dict, self._params)
            self._use_new_api = True
        except AttributeError:
            self._detector = None
            self._use_new_api = False

    def detect(self, frame):
        """Detect markers in a BGR frame.

        Returns list of (marker_id, corners_4x2, center_xy) tuples.
        """
        if not self._enabled:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame

        if self._use_new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dict, parameters=self._params)

        if ids is None:
            return []

        results = []
        for i, mid in enumerate(ids.flatten()):
            c = corners[i][0]
            center = c.mean(axis=0)
            results.append((int(mid), c, (float(center[0]), float(center[1]))))
        return results

    def estimate_distance_cm(self, corners, frame_width):
        """Rough distance estimate from apparent marker size."""
        if corners is None or len(corners) < 2:
            return None
        side_px = float(np.linalg.norm(corners[0] - corners[1]))
        if side_px < 1:
            return None
        focal_px = frame_width * 0.8
        return (MARKER_SIZE_CM * focal_px) / side_px
