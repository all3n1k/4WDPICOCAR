"""
LocalizationModule — fuses ArUco marker detection with RSSI zone estimation.

ArUco markers are the primary localization source. When a marker is detected,
the robot's position snaps to the room center on the apartment floor plan.
Between detections, odometry shifts the position. RSSI is a fallback when
no markers have been seen recently.

Usage:
    loc = LocalizationModule()
    loc.start()                       # starts RSSI polling thread
    loc.update_aruco(marker_id)       # called when camera sees a marker
    loc.update_position(delta, hdg)   # called on odometry ticks
    print(loc.get_zone())             # "Kitchen"
    print(loc.get_position())         # (135, 155)
"""

import math
import subprocess
import threading
import time
import re

from dimos_lite.floorplan import (
    APARTMENT_W, APARTMENT_H, MARKER_TO_ROOM, room_center,
)

# ── RSSI Zone Calibration ────────────────────────────────────────────────────
ZONE_MAP = {
    "Living Room":  (-55,   0),
    "Kitchen":      (-70, -56),
    "Bedroom":      (-90, -71),
}

AIRPORT_PATH = "/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport"
POLL_INTERVAL = 3.0
ARUCO_STALENESS = 30.0


def _get_rssi():
    try:
        result = subprocess.run(
            [AIRPORT_PATH, "-I"],
            capture_output=True, text=True, timeout=3,
        )
        match = re.search(r'agrCtlRSSI:\s*(-\d+)', result.stdout)
        if match:
            return int(match.group(1))
    except Exception:
        pass
    return None


def _rssi_to_zone(rssi):
    if rssi is None:
        return "Unknown"
    best_zone = "Unknown"
    best_delta = float('inf')
    for zone, (low, high) in ZONE_MAP.items():
        if low <= rssi <= high:
            return zone
        center = (low + high) / 2
        delta = abs(rssi - center)
        if delta < best_delta:
            best_delta = delta
            best_zone = zone
    return best_zone


class LocalizationModule:

    def __init__(self):
        self._lock = threading.Lock()

        # RSSI state
        self._zone = "Unknown"
        self._rssi = None

        # ArUco state
        self._aruco_room = None
        self._aruco_marker_id = None
        self._aruco_time = 0.0

        # Landmark positions (marker_id -> (x, y))
        self._marker_positions = {}

        # Floor plan position (apartment coordinates)
        self._robot_x = float(APARTMENT_W) / 2
        self._robot_y = float(APARTMENT_H) / 2

    def start(self):
        threading.Thread(target=self._poll_loop, daemon=True).start()
        print("[Localization] RSSI + ArUco tracking started")

    def _poll_loop(self):
        while True:
            rssi = _get_rssi()
            zone = _rssi_to_zone(rssi)
            with self._lock:
                self._rssi = rssi
                self._zone = zone
            time.sleep(POLL_INTERVAL)

    def set_marker_position(self, marker_id, x, y):
        with self._lock:
            # Only log and update if it actually changed
            mid = int(marker_id)
            new_pos = (float(x), float(y))
            if self._marker_positions.get(mid) != new_pos:
                self._marker_positions[mid] = new_pos
                print(f"[Localization] Marker {mid} mapped to ({x:.0f}, {y:.0f})")

    # ── ArUco updates ────────────────────────────────────────────────────

    def update_aruco(self, marker_id, distance_cm=0, bearing_deg=0, heading_deg=0):
        room_data = MARKER_TO_ROOM.get(marker_id)
        if not room_data:
            return

        with self._lock:
            self._aruco_room = room_data if isinstance(room_data, str) else room_data.get("label", "Unknown")
            self._aruco_marker_id = marker_id
            self._aruco_time = time.time()

            # Use specific marker position if user set one
            if marker_id in self._marker_positions:
                mx, my = self._marker_positions[marker_id]

                # Invert the forward-motion convention used in update_position:
                # from robot at heading H, a marker at camera-bearing B sits at
                # (x + D*sin(H+B), y - D*cos(H+B)). Solving for robot position:
                rad = math.radians(heading_deg + bearing_deg)
                self._robot_x = mx - distance_cm * math.sin(rad)
                self._robot_y = my + distance_cm * math.cos(rad)
            else:
                # Fallback to room center
                cx, cy = room_center(room_data)
                self._robot_x = cx
                self._robot_y = cy

    # ── Odometry updates ─────────────────────────────────────────────────

    def set_position(self, x, y):
        with self._lock:
            self._robot_x = float(x)
            self._robot_y = float(y)

    def update_position(self, delta_cm, heading_deg):
        with self._lock:
            rad = math.radians(heading_deg)
            self._robot_x += delta_cm * math.sin(rad)
            self._robot_y -= delta_cm * math.cos(rad)
            self._robot_x = max(0.0, min(float(APARTMENT_W), self._robot_x))
            self._robot_y = max(0.0, min(float(APARTMENT_H), self._robot_y))

    # ── Queries ──────────────────────────────────────────────────────────

    def get_zone(self):
        with self._lock:
            if self._aruco_room and (time.time() - self._aruco_time) < ARUCO_STALENESS:
                return self._aruco_room
            return self._zone

    def get_position(self):
        with self._lock:
            return (self._robot_x, self._robot_y)

    def get_aruco_marker_id(self):
        with self._lock:
            if (time.time() - self._aruco_time) < ARUCO_STALENESS:
                return self._aruco_marker_id
            return None

    def get_rssi(self):
        with self._lock:
            return self._rssi


def calibrate_zones(duration=30):
    print("\n===== RSSI Zone Calibration =====")
    print(f"Sampling RSSI every 1s for {duration} seconds.\n")
    samples = []
    for i in range(duration):
        rssi = _get_rssi()
        if rssi is not None:
            samples.append(rssi)
            print(f"  [{i+1:02d}s] RSSI: {rssi} dBm  (zone: {_rssi_to_zone(rssi)})")
        else:
            print(f"  [{i+1:02d}s] RSSI: [could not read]")
        time.sleep(1)
    if samples:
        print(f"\n  Min: {min(samples)} dBm  Max: {max(samples)} dBm  "
              f"Avg: {sum(samples)/len(samples):.1f} dBm")
        print("Update ZONE_MAP in dimos_lite/localization.py with these values.")


if __name__ == "__main__":
    calibrate_zones()
