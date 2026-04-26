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
import json
import os

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
POLL_INTERVAL = 1.0   # Faster polling for Cartographer (1Hz)
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
        self._robot_hdg = 0.0 # Initial heading

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

    def set_position(self, x, y):
        with self._lock:
            self._robot_x = float(x)
            self._robot_y = float(y)
            
    def set_heading(self, hdg):
        with self._lock:
            self._robot_hdg = float(hdg % 360)
            if hasattr(self, "_last_yaw"):
                delattr(self, "_last_yaw") # Force new baseline

    def set_marker_position(self, marker_id, x, y):
        with self._lock:
            # Only log and update if it actually changed
            mid = int(marker_id)
            new_pos = (float(x), float(y))
            if self._marker_positions.get(mid) != new_pos:
                self._marker_positions[mid] = new_pos
                print(f"[Localization] Marker {mid} mapped to ({x:.0f}, {y:.0f})")
                self._save_constellation()

    def _save_constellation(self):
        try:
            with open("constellation_map.json", "w") as f:
                json.dump(self._marker_positions, f, indent=2)
        except Exception as e:
            print(f"[Localization] Save error: {e}")

    def load_constellation(self):
        try:
            if os.path.exists("constellation_map.json"):
                with open("constellation_map.json", "r") as f:
                    data = json.load(f)
                    with self._lock:
                        # Convert keys back to ints
                        self._marker_positions = {int(k): v for k, v in data.items()}
                print(f"[Localization] Loaded {len(self._marker_positions)} constellation markers")
        except Exception as e:
            print(f"[Localization] Load error: {e}")

    # ── ArUco updates ────────────────────────────────────────────────────

    def update_aruco(self, marker_id, distance_cm=0, bearing_deg=0, heading_deg=0, discovery_mode=False):
        # Filter for valid IDs (0-9 Room, 10-30 Constellation)
        if not (0 <= marker_id <= 30):
            return

        # Room markers are 0-9, Floor constellation is 10-30
        room_data = MARKER_TO_ROOM.get(marker_id)
        
        with self._lock:
            # --- DISTANCE TRUST FILTER ---
            if distance_cm > 150:
                if discovery_mode and marker_id >= 10 and marker_id not in self._marker_positions:
                    rad = math.radians(heading_deg + bearing_deg)
                    mx = self._robot_x + distance_cm * math.sin(rad)
                    my = self._robot_y - distance_cm * math.cos(rad)
                    if 0 <= mx <= APARTMENT_W and 0 <= my <= APARTMENT_H:
                        self._marker_positions[marker_id] = (mx, my)
                        print(f"[Localization] Remote Discovery: Marker {marker_id}")
                        self._save_constellation()
                return

            # Inside Trust Range (0-150cm)
            self._aruco_room = room_data.get("label", "Unknown") if isinstance(room_data, dict) else (room_data or "Unknown")
            self._aruco_marker_id = marker_id
            self._aruco_time = time.time()

            # --- SMOOTH WEIGHTED FUSION ---
            alpha = 0.2 
            
            # Use specific marker position if it exists in our constellation
            if marker_id in self._marker_positions or (room_data and "marker_pos" in room_data):
                if marker_id in self._marker_positions:
                    mx, my = self._marker_positions[marker_id]
                else:
                    mx, my = room_data["marker_pos"]
                
                rad = math.radians(heading_deg + bearing_deg)
                est_x = mx - distance_cm * math.sin(rad)
                est_y = my + distance_cm * math.cos(rad)
                
                # Smoothly drift toward the estimate
                self._robot_x = (1 - alpha) * self._robot_x + alpha * est_x
                self._robot_y = (1 - alpha) * self._robot_y + alpha * est_y
                
                # Heading Correction (Wall markers only)
                if room_data and "marker_pos" in room_data:
                    # Fix heading based on marker bearing
                    est_hdg = (360 - bearing_deg) % 360
                    self._robot_hdg = (1 - alpha) * self._robot_hdg + alpha * est_hdg
            
            elif discovery_mode and marker_id >= 10:
                # Discover new marker (Close range)
                mx = self._robot_x + distance_cm * math.sin(math.radians(heading_deg + bearing_deg))
                my = self._robot_y - distance_cm * math.cos(math.radians(heading_deg + bearing_deg))
                self._marker_positions[marker_id] = (mx, my)
                self._save_constellation()

    def update_imu(self, yaw_deg):
        """Update the internal heading using IMU data (incremental)."""
        with self._lock:
            # We treat the first IMU reading as the baseline
            if not hasattr(self, "_last_yaw"):
                self._last_yaw = yaw_deg
                return
            
            delta_yaw = yaw_deg - self._last_yaw
            # Handle 360-degree wraparound for smooth delta
            if delta_yaw > 180: delta_yaw -= 360
            if delta_yaw < -180: delta_yaw += 360
            
            # SIGN FIX: Right turn (positive delta) should INCREASE heading (0 -> 90)
            self._robot_hdg = (self._robot_hdg + delta_yaw) % 360 
            self._last_yaw = yaw_deg

    def update_position(self, delta_cm):
        """Update position based on mileage delta and CURRENT fused heading."""
        with self._lock:
            rad = math.radians(self._robot_hdg)
            self._robot_x += delta_cm * math.sin(rad)
            self._robot_y -= delta_cm * math.cos(rad)
            self._robot_x = max(0.0, min(float(APARTMENT_W), self._robot_x))
            self._robot_y = max(0.0, min(float(APARTMENT_H), self._robot_y))
            
    def get_heading(self):
        with self._lock:
            return self._robot_hdg

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
