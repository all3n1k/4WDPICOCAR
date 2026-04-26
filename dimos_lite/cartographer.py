"""dimos_lite.cartographer — High-precision SLAM pre-pass.

Run via `python3 main_os.py --carto`. Dedicates all sensors to building the
constellation map (`constellation_map.json`) and obstacle memory
(`semantic_map.json`), then exits so main_os can load them on the next boot.
No LLM, no speech, just raw spatial discovery.
"""

import time
import math
import json
import os
from dimos_lite.core import Module, StreamIn, StreamOut
from dimos_lite.floorplan import APARTMENT_W, APARTMENT_H, MARKER_TO_ROOM
from dimos_lite.dashboard import start_dashboard, update_dashboard

RESET_DB = False  # Preserve constellation_map.json + room_fingerprints.json across runs.
SAFETY_BUFFER_CM = 45.0 # Increased for robot length (24cm) + turn radius

class DeepScanCartographer(Module):
    def __init__(self, localization, vision):
        super().__init__("Cartographer")
        self.loc = localization
        self._vision = vision
        self._running = False
        self._calibrated = False
        self._last_marker_id = None
        self._show_labels = True
        self._discovery_mode = True
        
        if RESET_DB:
            print("[Cartographer] RESET: Wiping old map data...")
            for f in ["constellation_map.json", "room_fingerprints.json"]:
                if os.path.exists(f):
                    try: os.remove(f)
                    except: pass
        else:
            self.loc.load_constellation()

        from dimos_lite.aruco import ArucoDetector
        from dimos_lite.agent import SemanticMap
        self._aruco = ArucoDetector()
        self.smap = SemanticMap()
        
        # Initial State: Actual Center of Living Room, facing Dead North
        self.loc.set_position(306.0, 541.0)
        self.loc.set_heading(0.0)
        self.smap.x = 30 
        self.smap.y = 54 
        self.smap.heading = 0.0
        self.heading = 0.0
        self.tick = 0 
        
        # Sensor data
        self._forward_dist = 999.0
        self._mileage = -1.0 
        self._last_mile = 0.0
        self._sonar_angle = 0.0
        self._imu_offset = 0.0
        
        # Streams
        self.cmd_vel = StreamOut("cmd_vel")
        self.color_image = StreamIn("color_image")
        self.color_image.subscribe(self._on_image)
        self.ultrasonic_distance = StreamIn("ultrasonic_distance")
        self.ultrasonic_distance.subscribe(self._on_distance)
        self.odometry = StreamIn("odometry")
        self.odometry.subscribe(self._on_odometry)
        self.battery_voltage = StreamIn("battery_voltage")
        self.battery_voltage.subscribe(self._on_battery)
        self.imu_data = StreamIn("imu_data")
        self.imu_data.subscribe(self._on_imu)
        self.tof_distance = StreamIn("tof_distance")
        self.tof_distance.subscribe(self._on_tof)
        
        self._tof_dist = 999.0
        
        # Visualizer Setup
        import cv2
        cv2.namedWindow("dimOS Deep-Scan Visualizer")
        cv2.setMouseCallback("dimOS Deep-Scan Visualizer", self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        import cv2
        if event == cv2.EVENT_LBUTTONDOWN:
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                # SHIFT-CLICK: Pin nearest marker
                best_id = None
                min_dist = 50
                with self.loc._lock:
                    for mid, pos in self.loc._marker_positions.items():
                        d = math.sqrt((pos[0]-x)**2 + (pos[1]-y)**2)
                        if d < min_dist:
                            min_dist = d
                            best_id = mid
                if best_id is not None:
                    print(f"[Cartographer] Manual Override: Pinning Marker {best_id} to ({x}, {y})")
                    self.loc.set_marker_position(best_id, x, y)
            else:
                # CLICK: Teleport robot
                print(f"[Cartographer] Manual Teleport: Moving robot to ({x}, {y})")
                self.loc.set_position(x, y)
                self.smap.x = int(x / 10)
                self.smap.y = int(y / 10)

    def _on_imu(self, data):
        # Firmware publishes [heading, gyro_z, ax, ay, az] — index, not key.
        if not self._calibrated or not isinstance(data, list) or len(data) < 1:
            return
        self.loc.update_imu(float(data[0]))
        self.heading = self.loc.get_heading()
        self.smap.heading = self.heading

    def _on_image(self, frame):
        if not self._aruco:
            update_dashboard(frame=frame)
            return
        import cv2
        annotated = frame.copy()
        hits = self._aruco.detect(frame)
        for marker_id, corners, center in hits:
            self._last_marker_id = marker_id
            dist_cm = self._aruco.estimate_distance_cm(corners, frame.shape[1])
            bearing = (center[0] - frame.shape[1]/2) / (frame.shape[1]/2) * 30

            # 1. Update localization with discovery mode
            self.loc.update_aruco(
                marker_id, dist_cm, bearing,
                heading_deg=self.heading,
                discovery_mode=self._discovery_mode
            )

            # 2. RSSI Fingerprinting (if within 100cm)
            if dist_cm < 100:
                rssi = self.loc.get_rssi()
                if rssi is not None:
                    self._save_fingerprint(marker_id, rssi)

            # 3. Overlay for dashboard video feed
            pts = corners.reshape((-1, 1, 2)).astype(int)
            cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)
            cv2.putText(annotated, f"ID:{marker_id} {dist_cm:.0f}cm",
                        (int(center[0]), int(center[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        update_dashboard(frame=annotated)

    def _save_fingerprint(self, marker_id, rssi):
        # Fingerprint: maps a known landmark to a signal strength
        fingerprint = {
            "marker_id": marker_id,
            "rssi": rssi,
            "coords": self.loc.get_position(),
            "heading": self.heading,
            "time": time.time()
        }
        try:
            path = "room_fingerprints.json"
            data = []
            if os.path.exists(path):
                with open(path, "r") as f:
                    data = json.load(f)
            data.append(fingerprint)
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"[Cartographer] Fingerprinted Marker {marker_id}: {rssi} dBm")
        except Exception as e:
            print(f"[Cartographer] Fingerprint error: {e}")

    def _on_distance(self, data):
        if isinstance(data, list) and len(data) == 2:
            angle, dist = float(data[0]), float(data[1])
            self._sonar_angle = angle
            self._forward_dist = dist
            if self._running:
                # Map obstacles from sonar
                self.smap.add_obstacle(angle, dist)

    def _on_tof(self, dist):
        if dist is not None:
            self._tof_dist = float(dist)
            # Map laser obstacles (0 degrees offset from heading)
            if self._running:
                self.smap.add_obstacle(0, self._tof_dist)

    def _on_odometry(self, mileage):
        self._mileage = float(mileage)
        if self._last_mile == 0:
            self._last_mile = self._mileage
        delta = self._mileage - self._last_mile
        if delta > 0:
            self.loc.update_position(delta)
            self.smap.update_position(delta)
            self._last_mile = self._mileage

    def _on_battery(self, v):
        # Silenced: Pico 4WD PCB doesn't have battery ADC wired
        pass

    def _push_to_dashboard(self):
        """Publish cartographer pose + constellation + obstacles to the web dashboard."""
        rx, ry = self.loc.get_position()
        with self.loc._lock:
            markers = {int(mid): [float(p[0]), float(p[1])]
                       for mid, p in self.loc._marker_positions.items()}
        update_dashboard(
            tick=self.tick,
            robot_x=rx,
            robot_y=ry,
            heading=self.heading,
            forward_dist=self._forward_dist,
            mileage=self._mileage,
            action="mapping" if self._running else "calibrating",
            observation=f"Calib: {'OK' if self._calibrated else 'PENDING'} | Markers: {len(markers)}",
            reasoning=f"Last marker: {self._last_marker_id}  Discovery: {'ON' if self._discovery_mode else 'LOCK'}",
            marker_positions=markers,
            obstacles=self.smap.get_obstacles(),
            aruco_marker_id=self._last_marker_id,
            has_imu=hasattr(self.loc, "_last_yaw"),
        )

    def _visualize(self):
        """Draw a high-res SLAM map similar to the provided reference."""
        import cv2
        import numpy as np
        
        # Create canvas (700x700)
        canvas = np.ones((700, 700, 3), dtype=np.uint8) * 255 # White background
        
        # 1. Draw North Arrow (Compass Rose)
        cv2.arrowedLine(canvas, (50, 100), (50, 50), (0, 0, 255), 2)
        cv2.putText(canvas, "N", (42, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 2. Draw Floorplan (Grey)
        from dimos_lite.floorplan import ROOMS
        for room in ROOMS:
            x, y, w, h = room["bounds"]
            cv2.rectangle(canvas, (int(x), int(y)), (int(x+w), int(y+h)), (230, 230, 230), 1)
            cv2.putText(canvas, room["label"], (int(x+5), int(y+20)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 3. Draw Discovered Markers (Green Stars)
        if self._show_labels:
            with self.loc._lock:
                for mid, pos in self.loc._marker_positions.items():
                    mx, my = int(pos[0]), int(pos[1])
                    color = (0, 180, 0)
                    # Draw marker + ID + Distance
                    cv2.circle(canvas, (int(mx), int(my)), 8, color, -1)
                    cv2.putText(canvas, f"ID:{mid}", (int(mx)+10, int(my)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                    # Show measured distance if it's the one we are currently seeing
                    if mid == self._last_marker_id:
                        cv2.putText(canvas, "ACTIVE", (int(mx)+10, int(my)+12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

        # 4. Draw Obstacles (Magenta Point Cloud)
        with self.smap._lock:
            for r in range(self.smap.size):
                for c in range(self.smap.size):
                    if self.smap.grid[r][c] == 'W':
                        # Convert grid cell back to coordinates (1 cell = 10cm)
                        cv2.circle(canvas, (c*10 + 5, r*10 + 5), 2, (255, 0, 255), -1)

        # 4. Draw Robot (Blue Triangle) + Sonar Beam
        rx, ry = self.loc.get_position()
        rad = math.radians(self.heading)
        p1 = (int(rx + 15 * math.sin(rad)), int(ry - 15 * math.cos(rad)))
        p2 = (int(rx + 10 * math.sin(rad + 2.5)), int(ry - 10 * math.cos(rad + 2.5)))
        p3 = (int(rx + 10 * math.sin(rad - 2.5)), int(ry - 10 * math.cos(rad - 2.5)))
        cv2.polylines(canvas, [np.array([p1, p2, p3])], True, (255, 0, 0), 2)
        
        # Draw Sonar Beam
        s_rad = math.radians(self.heading + self._sonar_angle)
        sx = int(rx + self._forward_dist * math.sin(s_rad))
        sy = int(ry - self._forward_dist * math.cos(s_rad))
        cv2.line(canvas, (int(rx), int(ry)), (sx, sy), (0, 0, 255), 1) # Red beam

        # 5. Draw Telemetry Text
        cv2.putText(canvas, f"Hdg: {self.heading:.1f} (Raw: {self.loc._last_yaw if hasattr(self.loc, '_last_yaw') else 0:.1f})", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(canvas, f"Pos: ({rx:.0f}, {ry:.0f})", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(canvas, f"Miles: {self._mileage:.1f} | Calib: {'OK' if self._calibrated else 'PENDING'}", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
        
        cv2.imshow("dimOS Deep-Scan Visualizer", canvas)
        cv2.waitKey(1)

    def run(self):
        print("\n" + "="*60)
        print("  dimOS-lite DEEP-SCAN CARTOGRAPHER  ")
        print("="*60)

        start_dashboard()
        print("[Cartographer] Live mini-map: http://localhost:8080")

        # --- PRE-FLIGHT SANITY CHECK ---
        print("[Check] Performing sensor sanity check...")
        checks_passed = False
        for i in range(15):
            cam_ok = self._vision.frame_count > 0
            pico_ok = self._mileage >= 0 
            sonar_ok = 0 < self._forward_dist < 400
            if cam_ok and pico_ok and sonar_ok:
                print(f"[Check] PASS: Camera OK | Pico OK | Sonar: {self._forward_dist:.1f}cm")
                checks_passed = True
                break
            time.sleep(1.0)
        
        if not checks_passed:
            print("[Check] FAIL: Sensors not responding.")
            return

        # --- CALIBRATION PHASE ---
        # HARD RESET state to Living Room Center
        print(f"[DEBUG] HEADING RESET: Forcing 0.0 (Previous: {self.heading})")
        self.loc.set_position(306.0, 541.0)
        self.loc.set_heading(0.0)
        self.heading = 0.0
        self.smap.x, self.smap.y = 30, 54
        print(f"[DEBUG] BRAIN HEADING: {self.loc.get_heading()}")
        
        print("\n" + "="*40)
        print("  CALIBRATION PHASE  ")
        print("  Arrows: Rotate | F: Flip 180 | Click: Move  ")
        print("  Press 'S' to START MAPPING  ")
        print("="*40)
        
        import cv2
        current_hdg = self.heading
        while not self._calibrated:
            self._visualize()
            self._push_to_dashboard()
            key = cv2.waitKey(10) & 0xFF
            if key == ord('s'):
                print(f"\n[Calibration] LOCKED-IN: Robot is facing {current_hdg:.1f}°")
                print("[Calibration] IMU baseline synchronized. Beginning discovery...")
                self._calibrated = True
            elif key == ord('n'): # Snap to North
                current_hdg = 0.0
                self.loc.set_heading(current_hdg)
            elif key == ord('f'): # Flip 180
                current_hdg = (current_hdg + 180) % 360
                self.loc.set_heading(current_hdg)
            elif key == 2: # Left Arrow
                current_hdg = (current_hdg - 5) % 360
                self.loc.set_heading(current_hdg)
            elif key == 3: # Right Arrow
                current_hdg = (current_hdg + 5) % 360
                self.loc.set_heading(current_hdg)
            elif key == ord('l'): # Toggle Discovery Lock
                self._discovery_mode = not self._discovery_mode
                print(f"[Cartographer] Discovery Mode: {'ON' if self._discovery_mode else 'OFF (Locked)'}")
            elif key == 27: # ESC
                return
            self.heading = self.loc.get_heading()
            time.sleep(0.01)

        print(f"Final Calibration: Hdg {self.heading:.1f}°")
        print("[Cartographer] Sampling RSSI baseline...")
        time.sleep(1) 
        
        self._running = True
        self.tick = 0
        self.cmd_vel.publish({"radar": "config", "step": 15, "poll": 2})
        time.sleep(0.5)
        self.cmd_vel.publish({"radar": "sweep"})
        
        last_turn_tick = 0
        current_speed = 30

        print("\n" + "*"*40)
        print("  CAT ENTERTAINMENT MODE (CARTOGRAPHER)  ")
        print("  Status: EXPLORING  ")
        print("  Press Ctrl+C to stop.  ")
        print("*"*40 + "\n")

        try:
            while self._running:
                self.tick += 1
                
                # FUSE Sonar and TOF for avoidance
                # We use the minimum of both to be safe.
                obs_dist = min(self._forward_dist, self._tof_dist)
                
                if obs_dist > 32:
                    # Clear path: Move forward
                    # Speed variation every 5 seconds to keep it dynamic
                    if self.tick % 50 == 0:
                        current_speed = random_speed()
                    self.cmd_vel.publish({"K": "forward", "A": current_speed})
                    
                    # Periodic random turn to explore new areas
                    if self.tick - last_turn_tick > 250:
                        import random
                        if random.random() > 0.96:
                            turn_dir = random.choice(["left", "right"])
                            print(f"[Wanderer] Picking new direction: {turn_dir}")
                            self.cmd_vel.publish({"K": turn_dir, "A": 35})
                            time.sleep(random.uniform(0.4, 0.9))
                            last_turn_tick = self.tick
                else:
                    # Obstacle detected! 
                    print(f"[Wanderer] Obstacle at {obs_dist:.0f}cm (S:{self._forward_dist:.0f} T:{self._tof_dist:.0f})")
                    self.cmd_vel.publish({"K": "stop", "A": 0})
                    
                    # Back up slightly to give space for turn
                    if obs_dist < 18:
                        self.cmd_vel.publish({"K": "backward", "A": 25})
                        time.sleep(0.5)
                        self.cmd_vel.publish({"K": "stop", "A": 0})
                    
                    # Pivot until clear
                    import random
                    turn_dir = random.choice(["left", "right"])
                    turn_time = random.uniform(0.7, 1.4) 
                    
                    print(f"[Wanderer] Turning {turn_dir}...")
                    self.cmd_vel.publish({"K": turn_dir, "A": 35})
                    time.sleep(turn_time)
                    self.cmd_vel.publish({"K": "stop", "A": 0})
                    
                    last_turn_tick = self.tick
                
                self._visualize()
                self._push_to_dashboard()
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[Wanderer] Stopping motors...")
            self.cmd_vel.publish({"K": "stop", "A": 0})
            time.sleep(0.2)
        finally:
            self._running = False
            self.cmd_vel.publish({"K": "stop", "A": 0})
            self.cmd_vel.publish({"radar": "stop"})
            self.smap.save_to_disk()
            print("[Wanderer] Session ended.")

def random_speed():
    import random
    return random.randint(28, 42)

