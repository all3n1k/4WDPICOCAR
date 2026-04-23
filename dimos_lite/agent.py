import json
import os
import requests
import time
import base64
import cv2
import math
import threading
from collections import deque
from dimos_lite.core import Module, StreamIn, StreamOut
from dimos_lite.dashboard import start_dashboard, update_dashboard, get_manual_command, get_position_override, get_marker_updates
from dimos_lite.aruco import ArucoDetector, HAS_ARUCO
from dimos_lite.floorplan import MARKER_TO_ROOM

# --- Global Constants ---
COLLISION_THRESHOLD = 45
COLLISION_EMERGENCY = 20
COLLISION_WIDE_ANGLE = 50
COLLISION_WIDE_DIST = 25
MOVEMENT_SPEED = 50
TURN_SPEED = 65
MANUAL_SPEED = 75


class SemanticMap:
    def __init__(self, size=21, cell_cm=10):
        self.size = size
        self.cell_cm = cell_cm
        self.grid = [[' ' for _ in range(size)] for _ in range(size)]
        self.x = size // 2
        self.y = size // 2
        self.heading = 0.0
        self._lock = threading.Lock()

    def load_prior(self, prior_grid):
        with self._lock:
            for r, row in enumerate(prior_grid):
                for c, cell in enumerate(row):
                    if r < self.size and c < self.size:
                        self.grid[r][c] = cell

    def add_obstacle(self, angle, distance):
        if distance <= 0 or distance > 180:
            return
        with self._lock:
            abs_angle = (self.heading + angle) % 360
            rad = math.radians(abs_angle)
            dx = int((distance * math.sin(rad)) / self.cell_cm)
            dy = int((distance * math.cos(rad)) / self.cell_cm)
            tx, ty = self.x + dx, self.y - dy
            if 0 <= tx < self.size and 0 <= ty < self.size:
                self.grid[ty][tx] = 'W'

    def update_position(self, delta_cm):
        with self._lock:
            rad = math.radians(self.heading)
            dx = int((delta_cm * math.sin(rad)) / self.cell_cm)
            dy = int((delta_cm * math.cos(rad)) / self.cell_cm)
            if dx or dy:
                self.x = max(0, min(self.size - 1, self.x + dx))
                self.y = max(0, min(self.size - 1, self.y - dy))

    def turn(self, degrees):
        with self._lock:
            self.heading = (self.heading + degrees) % 360

    def set_heading(self, heading):
        with self._lock:
            self.heading = heading % 360

    def render(self):
        with self._lock:
            rows = []
            for r in range(self.size):
                row = ""
                for c in range(self.size):
                    if r == self.y and c == self.x:
                        row += "[R]"
                    else:
                        row += f"[{self.grid[r][c]}]"
                rows.append(row)
            return "\n".join(rows)

    def update_from_action(self, action):
        """Advance robot position on the grid based on last executed action."""
        if action == "forward":
            self.update_position(self.cell_cm)
        elif action == "backward":
            self.update_position(-self.cell_cm)
        elif action == "left":
            self.turn(-35)
        elif action == "right":
            self.turn(35)

    def get_neighbors(self):
        with self._lock:
            def cell(dy, dx):
                r, c = self.y + dy, self.x + dx
                if 0 <= r < self.size and 0 <= c < self.size:
                    return self.grid[r][c]
                return 'W'
            return {"N": cell(-1, 0), "S": cell(1, 0), "E": cell(0, 1), "W": cell(0, -1)}

    def get_obstacles(self):
        with self._lock:
            obstacles = []
            for r in range(self.size):
                for c in range(self.size):
                    if self.grid[r][c] == 'W':
                        # Convert grid cell to floorplan pixels
                        ox = 350 + (c - 10) * 10
                        oy = 350 + (r - 10) * 10
                        obstacles.append((ox, oy))
            return obstacles


TOOLS = [
    {
        "name": "move",
        "description": "Move forward or backward by a specific distance in cm.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["forward", "backward"]},
                "distance_cm": {"type": "integer", "description": "10-150cm"}
            },
            "required": ["direction", "distance_cm"]
        }
    },
    {
        "name": "turn",
        "description": "Turn left or right by a specific number of degrees.",
        "parameters": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["left", "right"]},
                "degrees": {"type": "integer", "description": "15-180 degrees"}
            },
            "required": ["direction", "degrees"]
        }
    },
    {
        "name": "look",
        "description": "Stop sweeping and point the camera/sonar at a specific angle (-90 to 90). 0 is center.",
        "parameters": {
            "type": "object",
            "properties": {
                "angle": {"type": "integer", "description": "Degrees from center"}
            },
            "required": ["angle"]
        }
    },
    {
        "name": "scan",
        "description": "Perform a full 180-degree sweep and return a summary of all obstacles.",
        "parameters": {"type": "object", "properties": {}}
    },
    {
        "name": "speak",
        "description": "Communicate a thought or status to the user via the dashboard.",
        "parameters": {
            "type": "object",
            "properties": {
                "text": {"type": "string", "description": "Text message"}
            },
            "required": ["text"]
        }
    },
    {
        "name": "set_mood",
        "description": "Change the underglow color to express an emotion or status.",
        "parameters": {
            "type": "object",
            "properties": {
                "color": {"type": "string", "enum": ["red", "green", "blue", "yellow", "purple", "cyan", "white", "off"]}
            },
            "required": ["color"]
        }
    }
]


class AgentCoreModule(Module):

    def __init__(self, ollama_url="http://localhost:11434/api/generate",
                 model="gemma4:e4b", prior_map=None, localization=None):
        super().__init__("AgentCore")
        self.ollama_url = ollama_url
        self.model = model
        self._localization = localization
        self._aruco = ArucoDetector() if HAS_ARUCO else None

        self.semantic_map = SemanticMap()
        if prior_map:
            self.semantic_map.load_prior(prior_map)

        # --- Sensor state (written by stream callbacks) ---
        self._sensor_lock = threading.Lock()
        self._frame_b64 = None
        self._frame_raw = None
        self._forward_dist = 999.0
        self._last_dist_time = 0.0
        self._radar_sweep = []
        self._grayscale = [0, 0, 0]
        self._mileage = 0.0
        self._prev_mileage = 0.0
        self._aruco_detections = []
        self._imu_heading = None
        self._imu_gyro_z = 0.0
        self._imu_accel = [0.0, 0.0, 0.0]
        self._has_imu = False
        self._battery_v = 0.0
        self._current_leds = {"bottom": [0,0,0], "rear": [0,0,0]}

        # --- Brain state (written by think thread, read by control loop) ---
        self._brain_lock = threading.Lock()
        self._brain = {
            "action": "stop", "observation": "", "reasoning": "",
            "urgency": 1, "latency": 0.0, "latency_avg": 0.0,
        }
        self._plan_ready = threading.Event()
        self._speech_lock = threading.Lock()

        # --- Control state ---
        self.tick = 0
        self._running = False
        self._emergency_stop = False
        self._action_history = deque(maxlen=12)
        self._memory = deque(maxlen=5) # Short-term visual memory
        self._last_tool_result = ""
        self._latencies = deque(maxlen=20)

        # --- Streams ---
        self.cmd_vel = StreamOut("cmd_vel")
        self.color_image = StreamIn("color_image")
        self.color_image.subscribe(self._on_image)
        self.ultrasonic_distance = StreamIn("ultrasonic_distance")
        self.ultrasonic_distance.subscribe(self._on_distance)
        self.odometry = StreamIn("odometry")
        self.odometry.subscribe(self._on_odometry)
        self.grayscale_line = StreamIn("grayscale_line")
        self.grayscale_line.subscribe(self._on_grayscale)
        self.imu_data = StreamIn("imu_data")
        self.imu_data.subscribe(self._on_imu)
        self.battery_voltage = StreamIn("battery_voltage")
        self.battery_voltage.subscribe(self._on_battery)

    # ── Stream Callbacks (run in worker threads) ─────────────

    def _on_image(self, frame):
        # 1. Detection
        aruco_hits = []
        if self._aruco:
            aruco_hits = self._aruco.detect(frame)
            
            # Draw HUD for dashboard
            annotated = frame.copy()
            for marker_id, corners, center in aruco_hits:
                # Draw box
                pts = corners.reshape((-1, 1, 2)).astype(int)
                cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)
                # Draw ID and Label
                dist_cm = self._aruco.estimate_distance_cm(corners, frame.shape[1])
                bearing = (center[0] - frame.shape[1]/2) / (frame.shape[1]/2) * 30 # Approx 60 deg FOV
                label = f"ID:{marker_id} {dist_cm:.0f}cm"
                cv2.putText(annotated, label, (int(center[0]), int(center[1])-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Snap localization if we know this room
                if self._localization and marker_id in MARKER_TO_ROOM:
                    self._localization.update_aruco(
                        marker_id, dist_cm, bearing,
                        heading_deg=self.semantic_map.heading,
                    )
        else:
            annotated = frame

        # 2. Encode for LLM (high quality 896x896)
        small = cv2.resize(frame, (896, 896))
        _, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 85])
        b64 = base64.b64encode(buf).decode('utf-8')

        with self._sensor_lock:
            self._frame_raw = annotated # Push annotated frame to dashboard
            self._frame_b64 = b64
            self._aruco_detections = aruco_hits

    def _on_distance(self, data):
        if not isinstance(data, list) or len(data) != 2:
            return
        angle, dist = data[0], float(data[1])
        with self._sensor_lock:
            if -35 <= angle <= 35:
                # If it's a very center reading (within 10 deg), it's the most authoritative
                if abs(angle) < 10:
                    self._forward_dist = dist
                else:
                    # Otherwise, take the minimum of the cone
                    self._forward_dist = min(dist, self._forward_dist)
                self._last_dist_time = time.time()
            found = False
            for i, entry in enumerate(self._radar_sweep):
                if entry[0] == angle:
                    self._radar_sweep[i] = [angle, dist]
                    found = True
                    break
            if not found:
                self._radar_sweep.append([angle, dist])
            if len(self._radar_sweep) > 40:
                self._radar_sweep = self._radar_sweep[-34:]
        if self._current_action() == "stop" and 0 < dist < 180:
            self.semantic_map.add_obstacle(angle, dist)

    def _on_odometry(self, mileage):
        with self._sensor_lock:
            if self._prev_mileage == 0:
                self._prev_mileage = mileage
            delta = mileage - self._prev_mileage
            if delta > 0:
                self.semantic_map.update_position(delta)
                if self._localization:
                    self._localization.update_position(delta, self.semantic_map.heading)
                self._prev_mileage = mileage
            self._mileage = mileage

    def _on_grayscale(self, data):
        if isinstance(data, list) and len(data) == 3:
            with self._sensor_lock:
                self._grayscale = data

    def _on_imu(self, data):
        if not isinstance(data, list) or len(data) < 2:
            return
        heading = float(data[0])
        gyro_z = float(data[1])
        accel = data[2:5] if len(data) >= 5 else [0.0, 0.0, 0.0]
        with self._sensor_lock:
            self._imu_heading = heading
            self._imu_gyro_z = gyro_z
            self._imu_accel = accel
            self._has_imu = True
        self.semantic_map.set_heading(heading)

    def _on_battery(self, voltage):
        with self._sensor_lock:
            self._battery_v = float(voltage)

    def _current_action(self):
        with self._brain_lock:
            return self._brain["action"]

    def _check_sweep_danger(self):
        """Check full radar sweep for dangerously close obstacles.
        Returns (min_forward_dist, any_close_in_wide_cone)."""
        with self._sensor_lock:
            sweep = list(self._radar_sweep)
            fwd = self._forward_dist
        min_fwd = fwd
        wide_danger = False
        for entry in sweep:
            if len(entry) < 2:
                continue
            angle, dist = entry[0], float(entry[1])
            if dist <= 0 or dist > 300:
                continue
            if -35 <= angle <= 35 and dist < min_fwd:
                min_fwd = dist
            if -COLLISION_WIDE_ANGLE <= angle <= COLLISION_WIDE_ANGLE and dist < COLLISION_WIDE_DIST:
                wide_danger = True
        return min_fwd, wide_danger

    # ── Think Thread ─────────────────────────────────────────

    def _think_loop(self):
        print(f"[{self.name}] Think thread started")
        min_interval = 0.5
        while self._running:
            t0 = time.time()

            with self._sensor_lock:
                frame_b64 = self._frame_b64
                forward_dist = self._forward_dist

            if not frame_b64:
                time.sleep(0.5)
                continue

            # Visual feedback: Thinking...
            self._set_leds(bottom=[20, 20, 60]) # Soft blue thinking
            prompt = self._build_prompt()
            result = self._call_llm(prompt, frame_b64)

            if result:
                thought = result.get("thought", "")
                tool_calls = result.get("plan", [])
                if not tool_calls and "call" in result: # Backward compatibility
                    tool_calls = [result["call"]]

                with self._brain_lock:
                    self._brain["reasoning"] = thought
                    self._brain["observation"] = result.get("observation", "Mapping environment...")
                    self._brain["latency"] = result.get("latency", 0)
                    self._brain["latency_avg"] = result.get("latency_avg", 0)

                if thought:
                    self._memory.append(thought)
                    if tool_calls and tool_calls[0].get("name") != "speak":
                        self._speak_async(thought)

                if tool_calls:
                    update_dashboard(log_msg=f"[PLAN] {thought[:60]}...")
                    for call in tool_calls:
                        t_name = call.get("name")
                        t_args = call.get("arguments", {})
                        if not t_name: continue
                        
                        update_dashboard(log_msg=f"[TOOL] Executing {t_name}")
                        res_str = self._execute_tool(t_name, t_args)
                        self._last_tool_result = res_str
                        update_dashboard(log_msg=f"[RESULT] {res_str}")
                        
                        # Sequential speech for results
                        if t_name != "speak" and ("moved" in res_str.lower() or "turned" in res_str.lower()):
                            self._speak_async(res_str)
                else:
                    self._last_tool_result = "No actions planned."
                
                self._plan_ready.set()

            elapsed = time.time() - t0
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)

    def _execute_tool(self, name, args):
        print(f"[{self.name}] Tool: {name}({args})")
        if name == "move":
            dist = args.get("distance_cm", 40)
            direction = args.get("direction", "forward")
            self._set_leds(bottom=[0, 60, 0]) # Green for move
            duration = dist / 100.0
            self._execute(direction, MOVEMENT_SPEED, duration)
            self._set_leds(bottom=[0, 0, 0])
            self.semantic_map.update_from_action(direction)
            return f"Moved {direction} {dist}cm."

        elif name == "turn":
            deg = args.get("degrees", 35)
            direction = args.get("direction", "left")
            self._set_leds(bottom=[50, 40, 0]) # Yellow for turn
            duration = (deg / 90.0) * 0.8
            self._execute(direction, TURN_SPEED, duration)
            self._set_leds(bottom=[0, 0, 0])
            self.semantic_map.turn(deg if direction == "right" else -deg)
            return f"Turned {direction} {deg} degrees."

        elif name == "look":
            angle = args.get("angle", 0)
            self.cmd_vel.publish(("L", angle))
            time.sleep(0.3)
            return f"Looking at {angle} degrees. Sonar: {self._forward_dist:.0f}cm"

        elif name == "scan":
            self.cmd_vel.publish(("L", None)) # Trigger sweep resume
            time.sleep(2.0) # Wait for full sweep to populate self._radar_sweep
            
            with self._sensor_lock:
                sweep = list(self._radar_sweep)
            
            left_min = 999
            right_min = 999
            center_min = 999
            
            for ang, dist in sweep:
                if -15 <= ang <= 15: center_min = min(center_min, dist)
                elif ang < -15: left_min = min(left_min, dist)
                elif ang > 15: right_min = min(right_min, dist)
            
            summary = (f"Left: {'CLEAR' if left_min > 50 else f'{left_min:.0f}cm'}, "
                       f"Center: {'CLEAR' if center_min > 50 else f'{center_min:.0f}cm'}, "
                       f"Right: {'CLEAR' if right_min > 50 else f'{right_min:.0f}cm'}")
            
            return f"Scan complete. {summary}"

        elif name == "speak":
            txt = args.get("text", "...")
            print(f"[{self.name}] SPEAK: {txt}")
            update_dashboard(log_msg=f"[VOICE] {txt}")
            self._speak_async(txt)
            return "Message delivered."

        elif name == "set_mood":
            c = args.get("color", "blue")
            mapping = {
                "red": [60, 0, 0], "green": [0, 60, 0], "blue": [0, 0, 60],
                "yellow": [50, 40, 0], "purple": [40, 0, 50], "cyan": [0, 50, 50],
                "white": [40, 40, 40], "off": [0, 0, 0]
            }
            color = mapping.get(c, [0, 0, 60])
            self._set_leds(bottom=color)
            return f"Mood set to {c}."

        return f"Unknown tool: {name}"

    def _set_leds(self, bottom=None, rear=None):
        payload = {}
        if bottom is not None and bottom != self._current_leds["bottom"]:
            payload["L_BOT"] = bottom
            self._current_leds["bottom"] = bottom
        if rear is not None and rear != self._current_leds["rear"]:
            payload["L_REAR"] = rear
            self._current_leds["rear"] = rear
        
        if payload:
            self.cmd_vel.publish(payload)

    def _speak_async(self, text):
        """Sequential speech using macOS 'say'."""
        import subprocess
        def speak_thread():
            with self._speech_lock:
                # Strip special chars for 'say'
                clean_text = text.replace('"', '').replace("'", "").replace("{", "").replace("}", "")
                subprocess.run(["say", "-v", "Samantha", "-r", "180", clean_text])
        threading.Thread(target=speak_thread, daemon=True).start()

    def _build_prompt(self):
        with self._sensor_lock:
            dist = self._forward_dist
            gs = self._grayscale
            batt = self._battery_v
            aruco = [h[0] for h in self._aruco_detections]
        
        neighbors = self.semantic_map.get_neighbors()
        zone = self._localization.get_zone() if self._localization else "Unknown"
        rx, ry = self.semantic_map.x, self.semantic_map.y
        mem = "\n- ".join(self._memory) if self._memory else "none"
        
        return f"""YOU ARE DIMOS: A highly intelligent 4-wheel robot car (4 inches tall).
HARDWARE: Raspberry Pi Pico, Ultrasonic on Servo, Grayscale (L/C/R), Photo-interruptors (Mileage).
YOUR MISSION: Explore the apartment, map the environment, and interact with the human owner.

CURRENT STATE:
- Position: ({rx}, {ry}) in room: {zone}
- Heading: {self.semantic_map.heading:.0f} degrees
- Forward Dist: {dist:.0f}cm
- Grayscale (Floor): {gs}
- Vision (ArUco IDs seen): {aruco if aruco else 'None'}
- Neighbors: N={neighbors['N']}, S={neighbors['S']}, E={neighbors['E']}, W={neighbors['W']}
- Battery: {batt:.1f}V
- Memory:
{mem}

LAST RESULT: {self._last_tool_result}

RULES:
1. You are self-aware. Use your sensors to navigate safely.
2. If Center is blocked (<40cm), you MUST call 'scan' to see Left/Right.
3. Be curious. Describe what you see and what you plan to do next.
4. You can plan MULTIPLE steps in one JSON output.

TOOLS:
{json.dumps(TOOLS, indent=2)}

OUTPUT ONLY JSON:
{{
  "thought": "Your internal high-level reasoning and self-awareness",
  "plan": [
    {{"name": "tool_name", "arguments": {{...}}}},
    {{"name": "tool_name", "arguments": {{...}}}}
  ]
}}"""

    def _call_llm(self, prompt, frame_b64):
        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [frame_b64],
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0.1,
                "num_predict": 512,
                "repeat_penalty": 1.5,
                "presence_penalty": 1.5,
                "repeat_last_n": 64
            }
        }
        try:
            start_t = time.time()
            resp = requests.post(self.ollama_url, json=payload, timeout=90)
            latency = time.time() - start_t
            if resp.status_code != 200:
                print(f"[{self.name}] LLM Error: {resp.status_code}")
                return None

            raw = resp.json().get("response", "{}").strip()
            
            # Better self-repair for truncation
            if not raw.endswith("}"):
                if 'plan' in raw and ']' not in raw: raw += ']}'
                if not raw.endswith('}'): raw += '}'
            
            # Robust JSON extraction
            try:
                import re
                m = re.search(r'(\{.*\})', raw, re.DOTALL)
                if m:
                    result = json.loads(m.group(1))
                else:
                    print(f"[{self.name}] No JSON found in response: {raw[:100]}")
                    return None
            except Exception as e:
                print(f"[{self.name}] JSON parse failed: {e} | Raw: {raw[:100]}")
                return None

            result["latency"] = round(latency, 3)
            return result

        except requests.exceptions.Timeout:
            print(f"[{self.name}] LLM timeout")
            return None
        except Exception as e:
            print(f"[{self.name}] LLM error: {e}")
            return None

    # ── Reflex Thread (Safety) ───────────────────────────────

    def _reflex_loop(self):
        print(f"[{self.name}] Reflex thread started (25Hz)")
        while self._running:
            action = self._current_action()
            min_fwd, wide_danger = self._check_sweep_danger()
            should_stop = False
            reason = ""

            if action == "forward":
                if min_fwd < COLLISION_EMERGENCY:
                    should_stop = True
                    reason = f"EMERGENCY {min_fwd:.0f}cm"
                elif min_fwd < COLLISION_THRESHOLD:
                    should_stop = True
                    reason = f"TOO CLOSE {min_fwd:.0f}cm"
                elif wide_danger:
                    should_stop = True
                    reason = f"WIDE CONE obstacle <{COLLISION_WIDE_DIST}cm"

            if action in ("left", "right") and wide_danger:
                with self._sensor_lock:
                    dist = self._forward_dist
                if dist < COLLISION_THRESHOLD:
                    should_stop = True
                    reason = f"TURN EMERGENCY {dist:.0f}cm"

            if should_stop:
                if not self._emergency_stop:
                    print(f"[{self.name}] REFLEX STOP: {reason}")
                    self.cmd_vel.publish(("stop", 0))
                    self._set_leds(rear=[100, 0, 0], bottom=[60, 0, 0]) # Bright red danger
                    self._emergency_stop = True
            else:
                if self._emergency_stop:
                    self._set_leds(rear=[0, 0, 0], bottom=[0, 0, 0]) # Clear danger
                self._emergency_stop = False
            time.sleep(0.04)

    # ── Control Loop (Main Thread) ───────────────────────────

    def start(self):
        print(f"[{self.name}] Online. Model: {self.model}")
        start_dashboard()
        self._running = True
        
        # Initial LED flash to confirm connection
        self._set_leds(bottom=[40, 40, 40], rear=[40, 40, 40])
        time.sleep(0.5)
        self._set_leds(bottom=[0, 0, 0], rear=[0, 0, 0])

        threading.Thread(target=self._think_loop, daemon=True).start()
        threading.Thread(target=self._reflex_loop, daemon=True).start()

        while self._running:
            self.tick += 1

            # Faster loop for better manual response
            self._plan_ready.wait(timeout=0.05)
            self._plan_ready.clear()

            # Manual override — direct passthrough, high priority
            manual = get_manual_command()
            if manual:
                speed = TURN_SPEED if manual in ("left", "right") else MANUAL_SPEED
                self.cmd_vel.publish((manual, speed if manual != "stop" else 0))
                self._set_leds(bottom=[0, 40, 40]) # Cyan for manual
                # Position/heading deltas flow in from _on_odometry (encoders)
                # and _on_imu — don't add dead-reckoning here or we double-count.
                # Turn estimate is a fallback only when the IMU is absent.
                if not self._has_imu:
                    if manual == "left": self.semantic_map.turn(-5)
                    if manual == "right": self.semantic_map.turn(5)
                with self._brain_lock:
                    self._brain["action"] = manual
                    self._brain["reasoning"] = "MANUAL CONTROL"
                self._sync_dashboard()
                continue

            # Check for marker updates
            m_updates = get_marker_updates()
            if m_updates and self._localization:
                for mid, mpos in m_updates.items():
                    self._localization.set_marker_position(mid, mpos[0], mpos[1])

            # Check for position override
            pos = get_position_override()
            if pos:
                if self._localization:
                    self._localization.set_position(pos[0], pos[1])
                # Position override is in apartment pixels — the 21x21 semantic
                # grid is a local window around the robot, so recenter it and
                # drop any stale obstacles tagged to the prior pose.
                with self.semantic_map._lock:
                    self.semantic_map.x = self.semantic_map.size // 2
                    self.semantic_map.y = self.semantic_map.size // 2
                print(f"[{self.name}] Position reset to ({pos[0]:.0f}, {pos[1]:.0f})")

            self._sync_dashboard()

    def _execute(self, action, speed, duration):
        if self._emergency_stop and action == "forward":
            self.cmd_vel.publish(("stop", 0))
            return
        
        with self._brain_lock:
            self._brain["action"] = action
        
        start_t = time.time()
        while (time.time() - start_t) < duration and self._running:
            # Check for manual interruption
            if get_manual_command():
                print(f"[{self.name}] AI action {action} interrupted")
                break
            
            # Check for emergency stop
            if action == "forward" and self._emergency_stop:
                print(f"[{self.name}] AI action {action} aborted: EMERGENCY")
                break
            
            # Send movement command
            self.cmd_vel.publish((action, speed))

            # Translational motion is integrated from encoder ticks in
            # _on_odometry; only dead-reckon turns, and only when the IMU
            # isn't providing absolute heading.
            if not self._has_imu:
                if action == "left": self.semantic_map.turn(-5)
                if action == "right": self.semantic_map.turn(5)

            time.sleep(0.05)

        self.cmd_vel.publish(("stop", 0))
        with self._brain_lock:
            self._brain["action"] = "stop"

    def _sync_dashboard(self):
        with self._sensor_lock:
            dist = self._forward_dist
            sweep = list(self._radar_sweep)
            gs = list(self._grayscale)
            mileage = self._mileage
            frame = self._frame_raw
            aruco_hits = list(self._aruco_detections)
            imu_heading = self._imu_heading
            imu_gyro_z = self._imu_gyro_z
            imu_accel = list(self._imu_accel)
            has_imu = self._has_imu
            battery_v = self._battery_v
        with self._brain_lock:
            brain = dict(self._brain)

        zone = self._localization.get_zone() if self._localization else "Unknown"
        if self._localization:
            robot_x, robot_y = self._localization.get_position()
            aruco_mid = self._localization.get_aruco_marker_id()
        else:
            # Center at 350,350 + relative movement (10cm per grid cell)
            robot_x = 350 + (self.semantic_map.x - 10) * 10
            robot_y = 350 + (self.semantic_map.y - 10) * 10
            aruco_mid = None

        aruco_room = None
        if aruco_hits:
            mid = aruco_hits[0][0]
            r = MARKER_TO_ROOM.get(mid)
            if r:
                aruco_room = r["label"]

        heading = imu_heading if has_imu and imu_heading is not None else self.semantic_map.heading

        update_dashboard(
            frame=frame,
            tick=self.tick,
            heading=heading,
            zone=zone,
            forward_dist=dist,
            speed=0,
            mileage=mileage,
            robot_x=robot_x,
            robot_y=robot_y,
            aruco_room=aruco_room or "",
            aruco_marker_id=aruco_mid,
            has_imu=has_imu,
            imu_gyro_z=imu_gyro_z,
            imu_accel=imu_accel,
            radar_sweep=sweep,
            grayscale=gs,
            obstacles=self.semantic_map.get_obstacles(),
            emergency_stop=self._emergency_stop,
            action=brain.get("action", "stop"),
            observation=brain.get("observation", ""),
            reasoning=brain.get("reasoning", ""),
            latency=brain.get("latency", 0),
            latency_avg=brain.get("latency_avg", 0),
            battery_v=battery_v,
        )
