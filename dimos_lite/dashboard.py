import http.server
import socketserver
import threading
import json
import os
import time
import cv2

PORT = 8080

class DashboardState:
    def __init__(self):
        self.latest_frame = None
        self.manual_cmds = []
        self.map_data = ""
        self.logs = []
        self.lock = threading.Lock()
        self.vision_module = None
        # Direct WS reference for servo passthrough — bypasses the agent's
        # manual-cmd queue so HTTP→Pico latency is one method call + WS send,
        # not HTTP→queue→agent-loop-wait→queue-drain→cmd_vel→WS.
        self.pico_hw = None
        self.position_override = None
        self.telemetry = {
            "tick": 0,
            "heading": 0,
            "zone": "Unknown",
            "forward_dist": 999,
            "tof_distance": None,
            "speed": 0,
            "mileage": 0,
            "action": "stop",
            "observation": "",
            "reasoning": "",
            "latency": 0,
            "latency_avg": 0,
            "radar_sweep": [],
            "grayscale": [0, 0, 0],
            "robot_x": 350,
            "robot_y": 350,
            "aruco_room": "",
            "aruco_marker_id": None,
            "has_imu": False,
            "imu_gyro_z": 0,
            "imu_accel": [0, 0, 0],
            "emergency_stop": False,
            "uptime": 0,
            "marker_positions": {},
            "obstacles": [],
        }
        self.marker_positions = {}
        # Marker survey (M0): in-progress pins keyed by marker ID.
        # Wall markers (0-4): {x, y, facing_deg, type: "wall"}
        # Floor markers (10+): {x, y, type: "floor"}
        self.survey_pins = {}
        self._load_survey_from_disk()
        self.start_time = time.time()

    def _load_survey_from_disk(self):
        """Restore the survey state from room_markers.json + constellation_map.json so a
        previous session's pins are visible on the dashboard without re-pinning."""
        try:
            if os.path.exists("room_markers.json"):
                with open("room_markers.json") as f:
                    for mid_str, entry in json.load(f).items():
                        self.survey_pins[int(mid_str)] = {
                            "x": float(entry["x"]),
                            "y": float(entry["y"]),
                            "facing_deg": float(entry.get("facing_deg", 0)),
                            "type": "wall",
                        }
        except Exception as e:
            print(f"[Dashboard] room_markers.json load error: {e}")
        try:
            if os.path.exists("constellation_map.json"):
                with open("constellation_map.json") as f:
                    for mid_str, pos in json.load(f).items():
                        mid = int(mid_str)
                        if mid not in self.survey_pins:
                            self.survey_pins[mid] = {
                                "x": float(pos[0]),
                                "y": float(pos[1]),
                                "type": "floor",
                            }
        except Exception as e:
            print(f"[Dashboard] constellation_map.json load error: {e}")
        if self.survey_pins:
            print(f"[Dashboard] Loaded {len(self.survey_pins)} surveyed markers")

state = DashboardState()


class DashboardHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/video_feed':
            self._serve_mjpeg()
        elif self.path == '/api/state':
            self._serve_state()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/api/control':
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            cmd = body.get('command')
            with state.lock:
                state.manual_cmds.append(cmd)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        elif self.path == '/api/set_marker':
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            mid = int(body.get('id', 0))
            x, y = float(body.get('x', 350)), float(body.get('y', 350))
            with state.lock:
                state.marker_positions[mid] = (x, y)
                state.telemetry["marker_positions"] = state.marker_positions
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True}).encode())
        elif self.path == '/api/set_position':
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            x, y = float(body.get('x', 350)), float(body.get('y', 350))
            with state.lock:
                state.position_override = (x, y)
                state.telemetry["robot_x"] = x
                state.telemetry["robot_y"] = y
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "x": x, "y": y}).encode())
        elif self.path == '/api/pin_marker':
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            mid = int(body.get('id'))
            mtype = body.get('type', 'floor')
            entry = {
                "x": float(body.get('x', 0)),
                "y": float(body.get('y', 0)),
                "type": mtype,
            }
            if mtype == "wall":
                entry["facing_deg"] = float(body.get('facing_deg', 0))
            with state.lock:
                state.survey_pins[mid] = entry
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "count": len(state.survey_pins)}).encode())
        elif self.path == '/api/unpin_marker':
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            mid = int(body.get('id'))
            with state.lock:
                state.survey_pins.pop(mid, None)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "count": len(state.survey_pins)}).encode())
        elif self.path == '/api/save_survey':
            with state.lock:
                pins = dict(state.survey_pins)
            room_markers, constellation = {}, {}
            for mid, entry in pins.items():
                if entry.get("type") == "wall":
                    room_markers[str(mid)] = {
                        "x": entry["x"], "y": entry["y"],
                        "facing_deg": entry.get("facing_deg", 0),
                    }
                else:
                    constellation[str(mid)] = [entry["x"], entry["y"]]
            ok = True
            try:
                with open("room_markers.json", "w") as f:
                    json.dump(room_markers, f, indent=2)
                with open("constellation_map.json", "w") as f:
                    json.dump(constellation, f, indent=2)
                print(f"[Dashboard] Survey saved: {len(room_markers)} wall + {len(constellation)} floor markers")
            except Exception as e:
                print(f"[Dashboard] Survey save error: {e}")
                ok = False
            self.send_response(200 if ok else 500)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"ok": ok, "wall": len(room_markers), "floor": len(constellation)}).encode())
        elif self.path == '/api/servo':
            # Direct passthrough to the Pico. Servo commands (pan/tilt head)
            # don't need to flow through the agent — they're hardware-level
            # control, not behavioural intent. Going direct cuts the path
            # from "HTTP → manual_cmds queue → agent loop wake → cmd_vel
            # publish → WS" down to "HTTP → method call → WS", eliminating
            # the agent-loop wait (was up to 50ms, now 10ms — but irrelevant
            # because we no longer use that path) and any queue buildup.
            length = int(self.headers['Content-Length'])
            body = json.loads(self.rfile.read(length).decode())
            pin = int(body.get('pin', 21))
            angle = int(body.get('angle', 90))
            hw = state.pico_hw
            if hw is not None:
                hw._on_cmd_vel((f"S{pin}", angle))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        elif self.path == '/api/led':
            length = int(self.headers['Content-Length'])
            data = json.loads(self.rfile.read(length).decode())
            brightness = data.get('brightness', 0)
            print(f"[Dashboard] LED toggle request: brightness={brightness}")
            
            # If state hasn't been linked yet, try to find it
            target = state.vision_module
            if not target:
                from dimos_lite.core import Module
                # Module.registry stores all active modules by name
                target = Module.registry.get("VisionV4")
                
            if target:
                print(f"[Dashboard] Found target: {target.name}. Dispatching {brightness} to hardware...")
                target._set_led(brightness)
            else:
                print("[Dashboard] Error: Vision module not found in registry")
            
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')

    def _serve_state(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        with state.lock:
            data = {
                **state.telemetry,
                "logs": state.logs[-50:],
                "map": state.map_data,
                "uptime": int(time.time() - state.start_time),
                "survey_pins": {str(k): v for k, v in state.survey_pins.items()},
            }
        self.wfile.write(json.dumps(data).encode())

    def _serve_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with state.lock:
                    if state.latest_frame is None:
                        time.sleep(0.1)
                        continue
                    _, jpeg = cv2.imencode('.jpg', state.latest_frame)
                    frame_bytes = jpeg.tobytes()
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame_bytes)}\r\n\r\n'.encode())
                self.wfile.write(frame_bytes)
                self.wfile.write(b'\r\n')
                time.sleep(0.033)  # ~30 FPS
        except Exception:
            pass

    def _serve_html(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(DASHBOARD_HTML.encode())

    def log_message(self, format, *args):
        return


DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Agentic Pet OS</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
  --bg:#0a0e17;--surface:#111827;--border:#1e293b;--border-hi:#334155;
  --text:#e2e8f0;--text-dim:#64748b;--accent:#22d3ee;--accent2:#a78bfa;
  --danger:#ef4444;--warn:#f59e0b;--ok:#22c55e;
}
body{background:var(--bg);color:var(--text);font-family:'Inter','SF Pro Display',-apple-system,sans-serif;height:100vh;overflow:hidden;display:flex;flex-direction:column}
.topbar{display:flex;align-items:center;justify-content:space-between;padding:8px 16px;background:var(--surface);border-bottom:1px solid var(--border);min-height:44px}
.topbar-left{display:flex;align-items:center;gap:12px}
.logo{font-size:14px;font-weight:700;letter-spacing:1px;color:var(--accent)}
.status-dot{width:8px;height:8px;border-radius:50%;background:var(--ok);animation:pulse 2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.topbar-stats{display:flex;gap:16px;font-size:12px;color:var(--text-dim)}
.topbar-stats span{display:flex;align-items:center;gap:4px}
.topbar-stats .val{color:var(--text);font-weight:600;font-variant-numeric:tabular-nums}
.main{display:flex;flex:1;overflow:hidden}
.col-left{flex:0 0 55%;display:flex;flex-direction:column;min-width:280px;overflow:hidden}
.col-right{flex:1;display:flex;flex-direction:column;min-width:240px;overflow:hidden}
.panel{background:var(--surface);border:1px solid var(--border);margin:6px;border-radius:8px;overflow:hidden;display:flex;flex-direction:column}
.panel-head{display:flex;align-items:center;justify-content:space-between;padding:8px 12px;border-bottom:1px solid var(--border);font-size:11px;text-transform:uppercase;letter-spacing:.8px;color:var(--text-dim);font-weight:600}
.panel-body{flex:1;overflow:hidden;position:relative}
/* Video panel takes the full column width; height comes from aspect-ratio
   so it tracks whatever the camera is actually streaming. JS overrides the
   aspect-ratio once the first frame loads (see naturalWidth handler in
   poll()), defaulting to 4/3 for VGA. Capped at 60vh so the brain/radar/map
   panels below always get a usable share of the column. */
.video-panel{flex:0 0 auto;width:100%;aspect-ratio:4/3;max-height:60vh;min-height:0}
.video-panel .panel-body{background:#000;display:flex;justify-content:center;align-items:center;padding:0;overflow:hidden}
.video-panel img{width:100%;height:100%;object-fit:contain;background:#000;image-rendering:pixelated}
.brain-panel{flex:1 1 420px;min-height:420px}
.brain-panel .panel-body{overflow-y:auto}
.radar-panel{flex:0 0 280px;min-height:140px}
.radar-panel .panel-body{padding:0;overflow:hidden;position:relative;display:flex;align-items:center;justify-content:center}
.map-panel{flex:1;min-height:120px}
.map-panel .panel-body{padding:0;overflow:hidden}
.log-panel{flex:0 0 180px;min-height:80px}
.log-panel .panel-body{overflow-y:auto;padding:4px 8px;font-family:'JetBrains Mono','Fira Code',monospace;font-size:11px;line-height:1.6}
.log-entry{padding:2px 0;border-bottom:1px solid rgba(255,255,255,.03)}
.log-entry .act{font-weight:700}
.log-entry .act-forward{color:var(--ok)}
.log-entry .act-backward{color:var(--warn)}
.log-entry .act-left,.log-entry .act-right{color:var(--accent2)}
.log-entry .act-stop{color:var(--danger)}
canvas#radar{display:block}
canvas#floorplan{display:block;width:100%;height:100%}
.resize-h{flex:0 0 5px;background:var(--border);cursor:row-resize;position:relative;z-index:5;transition:background .15s}
.resize-h:hover,.resize-h.dragging{background:rgba(34,211,238,0.5)}
.resize-v{flex:0 0 5px;background:var(--border);cursor:col-resize;position:relative;z-index:5;transition:background .15s}
.resize-v:hover,.resize-v.dragging{background:rgba(34,211,238,0.5)}
body.resizing-h{cursor:row-resize !important;user-select:none}
body.resizing-v{cursor:col-resize !important;user-select:none}
.brain-grid{display:grid;grid-template-columns:1fr 1fr;gap:1px;background:var(--border)}
.brain-cell{background:var(--surface);padding:8px 10px;display:flex;flex-direction:column}
.brain-cell .label{font-size:10px;text-transform:uppercase;letter-spacing:.5px;color:var(--text-dim);margin-bottom:4px}
.brain-cell .value{font-size:13px;font-weight:600;line-height:1.3;word-break:break-word}
.sensor-bar{height:4px;border-radius:2px;background:var(--border);margin-top:4px;overflow:hidden}
.sensor-bar-fill{height:100%;border-radius:2px;transition:width .3s}
.controls-row{display:flex;gap:6px;padding:6px;background:var(--surface);border-top:1px solid var(--border)}
.ctrl-btn{flex:1;padding:10px;border:1px solid var(--border);border-radius:6px;background:transparent;color:var(--text);font-size:14px;font-weight:700;cursor:pointer;transition:.15s;text-align:center}
.ctrl-btn:hover{border-color:var(--accent);color:var(--accent);background:rgba(34,211,238,.05)}
.ctrl-btn:active{background:rgba(34,211,238,.15)}
.ctrl-btn.danger{border-color:var(--danger)}
.ctrl-btn.danger:hover{color:var(--danger);border-color:var(--danger);background:rgba(239,68,68,.1)}
.alert-bar{display:none;padding:6px 12px;font-size:12px;font-weight:700;text-align:center;animation:flash .5s infinite}
.alert-bar.active{display:block}
.alert-bar.emergency{background:var(--danger);color:#fff}
@keyframes flash{0%,100%{opacity:1}50%{opacity:.7}}
.aruco-tag{display:inline-block;padding:1px 6px;border-radius:3px;background:rgba(245,158,11,0.15);color:#f59e0b;font-size:10px;font-weight:700;margin-left:6px}
.survey-toggle{cursor:pointer;background:rgba(34,211,238,0.1);border:1px solid var(--border);color:var(--text-dim);padding:1px 8px;border-radius:3px;font-size:10px;font-weight:700;letter-spacing:.5px;text-transform:uppercase}
.survey-toggle.active{background:var(--accent);color:#000;border-color:var(--accent)}
.survey-overlay{position:absolute;left:6px;right:6px;bottom:6px;background:rgba(8,12,20,0.96);border:1px solid var(--accent);border-radius:6px;display:none;z-index:10;cursor:pointer}
.survey-overlay.active{display:block}
.survey-overlay-bar{padding:5px 10px;display:flex;align-items:center;justify-content:space-between;font-size:10px;color:var(--text-dim);font-family:monospace;letter-spacing:.3px}
.survey-overlay-bar .chev{color:var(--accent);font-weight:700;font-family:sans-serif;font-size:11px}
.survey-overlay.expanded{cursor:default}
.survey-overlay-body{display:none;padding:6px 8px 8px;border-top:1px solid var(--border)}
.survey-overlay.expanded .survey-overlay-body{display:block}
.survey-overlay .grp{display:flex;flex-wrap:wrap;gap:3px;margin-bottom:4px;align-items:center}
.survey-overlay .grp-label{font-size:9px;color:var(--text-dim);text-transform:uppercase;letter-spacing:.5px;margin-right:4px}
.mid-btn{width:22px;height:22px;border-radius:3px;border:1px solid var(--border);background:transparent;color:var(--text-dim);font-size:10px;font-weight:700;cursor:pointer;font-family:monospace}
.mid-btn:hover{border-color:var(--accent);color:var(--accent)}
.mid-btn.wall{border-color:#f59e0b}
.mid-btn.pinned{background:rgba(34,197,94,0.2);color:#22c55e;border-color:#22c55e}
.mid-btn.active{background:var(--accent);color:#000;border-color:var(--accent);box-shadow:0 0 6px rgba(34,211,238,0.6)}
.survey-actions{display:flex;gap:4px;margin-top:6px;align-items:center}
.survey-actions .info{flex:1;font-size:10px;color:var(--text-dim);font-family:monospace}
.survey-btn{padding:3px 8px;font-size:10px;font-weight:700;border-radius:3px;border:1px solid var(--border);background:transparent;color:var(--text);cursor:pointer;text-transform:uppercase;letter-spacing:.5px}
.survey-btn.primary{border-color:var(--ok);color:var(--ok)}
.survey-btn.primary:hover{background:rgba(34,197,94,0.15)}
.survey-btn.danger{border-color:var(--danger);color:var(--danger)}
.survey-btn.danger:hover{background:rgba(239,68,68,0.15)}
.facing-popup{position:fixed;background:#0a0e17;border:1px solid var(--accent);border-radius:6px;padding:6px;display:none;z-index:50;box-shadow:0 4px 12px rgba(0,0,0,0.5)}
.facing-popup.active{display:block}
.facing-popup .face-grid{display:grid;grid-template-columns:repeat(3,28px);grid-template-rows:repeat(3,28px);gap:2px}
.facing-popup .face-btn{border:1px solid var(--border);background:transparent;color:var(--text-dim);font-size:10px;font-weight:700;cursor:pointer;border-radius:3px;font-family:monospace}
.facing-popup .face-btn:hover{border-color:var(--accent);color:var(--accent)}
.facing-popup .face-btn.center{cursor:default;color:#f59e0b;border-color:#f59e0b;font-size:14px}
.facing-popup .face-label{font-size:9px;color:var(--text-dim);text-align:center;margin-bottom:4px;text-transform:uppercase;letter-spacing:.5px}
</style>
</head>
<body>
<div class="topbar">
  <div class="topbar-left">
    <div class="status-dot" id="statusDot"></div>
    <div class="logo">AGENTIC PET OS</div>
    <button id="ledBtn" onclick="toggleLED()" style="background:#f1c40f; color:#000; border:none; padding:4px 8px; border-radius:4px; font-weight:bold; cursor:pointer; font-size:10px; display:flex; align-items:center; gap:4px;">
      💡 LED
    </button>
  </div>
  <div class="topbar-stats">
    <span>TICK <span class="val" id="statTick">0</span></span>
    <span>UPTIME <span class="val" id="statUptime">0s</span></span>
    <span>LATENCY <span class="val" id="statLatency">---</span></span>
    <span>ZONE <span class="val" id="statZone">---</span></span>
    <span id="arucoTag" style="display:none" class="aruco-tag"></span>
  </div>
</div>
<div class="alert-bar" id="alertBar"></div>
<div class="main">
  <div class="col-left">
    <div class="panel video-panel">
      <div class="panel-head"><span>LIVE VISION</span><span id="camRes">---</span></div>
      <div class="panel-body"><img id="video" src="/video_feed" alt="Camera"></div>
    </div>
    <div class="resize-h" id="resizeVideoBrain" title="Drag to resize"></div>
    <div class="panel brain-panel">
      <div class="panel-head"><span>AGENTIC INTELLIGENCE</span><span id="brainAction" style="background:var(--accent);color:#000;padding:0 6px;border-radius:3px">IDLE</span></div>
      <div class="panel-body">
        <div class="brain-grid">
          <div class="brain-cell" style="grid-column: span 2; background: rgba(34, 211, 238, 0.03); border-bottom: 1px solid var(--border)">
            <div class="label">Thinking Process</div>
            <div class="value" id="brainReason" style="font-family:'JetBrains Mono',monospace; font-size:11px; color:var(--accent); min-height:44px">Analyzing environment...</div>
          </div>
          <div class="brain-cell">
            <div class="label">Observation</div>
            <div class="value" id="brainObs" style="font-size:12px">Waiting for vision...</div>
          </div>
          <div class="brain-cell">
            <div class="label">Sonar (HC-SR04)</div>
            <div class="value" id="brainDist">---</div>
            <div class="sensor-bar"><div class="sensor-bar-fill" id="distBar" style="width:0;background:var(--ok)"></div></div>
          </div>
          <div class="brain-cell">
            <div class="label">Laser (VL53L0X)</div>
            <div class="value" id="brainTof">---</div>
            <div class="sensor-bar"><div class="sensor-bar-fill" id="tofBar" style="width:0;background:var(--accent2)"></div></div>
          </div>
          <div class="brain-cell">
            <div class="label">Heading <span id="imuBadge" style="display:none;font-size:8px;padding:1px 4px;border-radius:2px;background:rgba(34,211,238,0.2);color:#22d3ee;margin-left:4px">IMU</span></div>
            <div class="value" id="brainHeading">0 deg</div>
          </div>
          <div class="brain-cell">
            <div class="label">Speed / Mileage</div>
            <div class="value" id="brainSpeed">0 cm/s | 0 cm</div>
          </div>
          <div class="brain-cell" style="grid-column: span 2">
            <div class="label">IMU Accel (g)</div>
            <div class="value" id="brainAccel" style="font-family:monospace;font-size:11px">x: --- &nbsp;&nbsp; y: --- &nbsp;&nbsp; z: ---</div>
          </div>
          <div class="brain-cell" style="grid-column: span 2">
            <div class="label">Grayscale (Floor Sensors)</div>
            <div class="value" id="brainGS" style="font-family:monospace;font-size:10px">---</div>
            <div style="display:flex;gap:2px;margin-top:4px">
              <div style="flex:1;height:3px;background:var(--border)"><div id="gs0" style="height:100%;background:var(--accent);width:0"></div></div>
              <div style="flex:1;height:3px;background:var(--border)"><div id="gs1" style="height:100%;background:var(--accent);width:0"></div></div>
              <div style="flex:1;height:3px;background:var(--border)"><div id="gs2" style="height:100%;background:var(--accent);width:0"></div></div>
            </div>
          </div>
        </div>
      </div>
    </div>
    <div class="panel" style="flex: 0 0 52px; margin: 6px">
      <div class="controls-row">
        <button class="ctrl-btn" data-cmd="forward">W Fwd</button>
        <button class="ctrl-btn" data-cmd="left">A Left</button>
        <button class="ctrl-btn" data-cmd="backward">S Back</button>
        <button class="ctrl-btn" data-cmd="right">D Right</button>
        <button class="ctrl-btn danger" data-cmd="stop">&#9251; Stop</button>
      </div>
    </div>
  </div>
  <div class="resize-v" id="resizeColumns" title="Drag to resize columns"></div>
  <div class="col-right">
    <div class="panel radar-panel">
      <div class="panel-head"><span>RADAR SWEEP</span><span id="radarCount">0 pts</span></div>
      <div class="panel-body" id="radarBody"><canvas id="radar"></canvas></div>
    </div>
    <div class="resize-h" id="resizeRadarMap" title="Drag to resize"></div>
    <div class="panel map-panel">
      <div class="panel-head">
        <span>FLOOR PLAN</span>
        <span style="display:flex;align-items:center;gap:8px">
          <span id="mapPos">---</span>
          <button class="survey-toggle" id="surveyToggle">📍 Survey</button>
        </span>
      </div>
      <div class="panel-body" id="floorplanBody" style="position:relative">
        <canvas id="floorplan"></canvas>
        <div class="survey-overlay" id="surveyOverlay">
          <div class="survey-overlay-bar" id="surveyBar">
            <span id="surveyBarStatus">Survey ready — click to expand</span>
            <span class="chev" id="surveyChev">▲</span>
          </div>
          <div class="survey-overlay-body">
            <div class="grp">
              <span class="grp-label">Wall</span>
              <button class="mid-btn wall" data-mid="0" data-type="wall">0</button>
              <button class="mid-btn wall" data-mid="1" data-type="wall">1</button>
              <button class="mid-btn wall" data-mid="2" data-type="wall">2</button>
              <button class="mid-btn wall" data-mid="3" data-type="wall">3</button>
              <button class="mid-btn wall" data-mid="4" data-type="wall">4</button>
            </div>
            <div class="grp" id="floorBtnGrp">
              <span class="grp-label">Floor</span>
            </div>
            <div class="survey-actions">
              <span class="info" id="surveyInfo">Pick a marker, then click on the floor plan</span>
              <button class="survey-btn primary" id="surveySaveBtn">Save</button>
              <button class="survey-btn danger" id="surveyClearBtn">Clear</button>
            </div>
          </div>
        </div>
        <div class="facing-popup" id="facingPopup">
          <div class="face-label">Marker faces…</div>
          <div class="face-grid">
            <button class="face-btn" data-deg="315">NW</button>
            <button class="face-btn" data-deg="0">N</button>
            <button class="face-btn" data-deg="45">NE</button>
            <button class="face-btn" data-deg="270">W</button>
            <button class="face-btn center" disabled>◆</button>
            <button class="face-btn" data-deg="90">E</button>
            <button class="face-btn" data-deg="225">SW</button>
            <button class="face-btn" data-deg="180">S</button>
            <button class="face-btn" data-deg="135">SE</button>
          </div>
        </div>
      </div>
    </div>
    <div class="resize-h" id="resizeMapLog" title="Drag to resize"></div>
    <div class="panel log-panel">
      <div class="panel-head"><span>ACTION LOG</span><span id="logCount">0</span></div>
      <div class="panel-body" id="logBody"></div>
    </div>
  </div>
</div>
<script>
function send(cmd){
  fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:cmd})});
}
function toggleLED(){
  const btn = document.getElementById('ledBtn');
  const active = btn.style.boxShadow !== '';
  const val = active ? 0 : 50;
  btn.style.boxShadow = active ? '' : '0 0 15px #f1c40f';
  btn.style.background = active ? '#f1c40f' : '#fff';
  fetch('/api/led',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({brightness:val})});
}

/* ── Pan-Tilt Servo State ─────────────────────────── */
/* ── Pan/Tilt servo control ───────────────────────────────────────────────
 * Architecture: the JS side maintains a *target* angle for each servo (pan,
 * tilt) and a separate sender that flushes the latest target whenever the
 * previous fetch completes. This means:
 *   - holding an arrow key advances the target every ARROW_INTERVAL_MS,
 *   - only ever ONE in-flight fetch per pin (no localhost queue buildup),
 *   - on release, no in-flight tail to drain — the last sent target IS the
 *     final target, the Pico interpolates to it at 400°/s and stops.
 * The Pico is the speed governor (interp at 400°/s); JS just declares intent.
 */
let pan = 90, tilt = 90;          // last value confirmed sent to the Pico
let panTarget = 90, tiltTarget = 90;
let panInFlight = false, tiltInFlight = false;
// JS intent rate must stay below the Pico's real interp rate, otherwise the
// target creeps ahead of pan_current and the gap drains on release as visible
// "slow continued motion." Pico interp is ~300°/s real (3°/tick at ~10ms loop
// period including sensor reads). 5°/40ms = 125°/s here gives the Pico ~2.4×
// headroom — at release, max in-flight lag is one 5° bump = ~17ms drain.
const SERVO_STEP_PER_TICK = 5;
const ARROW_INTERVAL_MS = 40;

function flushServo(pin){
  const isPan = pin === 20;
  if((isPan ? panInFlight : tiltInFlight)) return;
  const want = isPan ? panTarget : tiltTarget;
  const last = isPan ? pan : tilt;
  if(want === last) return;
  if(isPan){ pan = want; panInFlight = true; } else { tilt = want; tiltInFlight = true; }
  fetch('/api/servo', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({pin, angle:want}), keepalive:true})
    .finally(()=>{
      if(isPan){
        panInFlight = false;
        if(panTarget !== pan) flushServo(20);  // newer target arrived during flight
      } else {
        tiltInFlight = false;
        if(tiltTarget !== tilt) flushServo(21);
      }
    });
}

/* Continuous-while-held for arrow keys (mirrors WASD's startManual). */
const arrowMap = {ArrowLeft:[SERVO_STEP_PER_TICK,0], ArrowRight:[-SERVO_STEP_PER_TICK,0], ArrowUp:[0,-SERVO_STEP_PER_TICK], ArrowDown:[0,SERVO_STEP_PER_TICK]};
const arrowsHeld = new Set();
let arrowTx = null;
// Clamp matches Pico firmware ±75°. The mapping is JS angle = Pico angle + 90,
// so JS range 15..165 → Pico ±75°. Margins prevent SG90 end-stop dither.
const SERVO_MIN = 15, SERVO_MAX = 165;
function bumpTarget(dp, dt){
  if(dp){ panTarget = Math.max(SERVO_MIN, Math.min(SERVO_MAX, panTarget + dp)); flushServo(20); }
  if(dt){ tiltTarget = Math.max(SERVO_MIN, Math.min(SERVO_MAX, tiltTarget + dt)); flushServo(21); }
}
function startArrows(){
  if(arrowTx) return;
  arrowTx = setInterval(()=>{
    if(!arrowsHeld.size){clearInterval(arrowTx); arrowTx=null; return;}
    let dp=0, dt=0;
    arrowsHeld.forEach(k=>{ const [p,t]=arrowMap[k]; dp+=p; dt+=t; });
    bumpTarget(dp, dt);
  }, ARROW_INTERVAL_MS);
}

const keyMap={w:'forward',a:'left',s:'backward',d:'right',' ':'stop'};
const pressed=new Set();
let manualTx=null;
// Shift = boost. When held alongside any movement key, we suffix the command
// with "_boost" and the agent dispatches at BOOST_SPEED instead of MANUAL_SPEED.
// Shift-state changes mid-hold are picked up automatically because startManual
// re-sends the active direction every 80ms.
let boostHeld = false;

function activeCmd(){
  if(!pressed.size) return null;
  const last=[...pressed].pop();
  const base = keyMap[last];
  if(!base || base==='stop') return null;
  return boostHeld ? base + '_boost' : base;
}

function startManual(){
  if(manualTx)return;
  manualTx=setInterval(()=>{
    if(!pressed.size){clearInterval(manualTx);manualTx=null;return;}
    const cmd = activeCmd();
    if(cmd) send(cmd);
  },80);
}

document.addEventListener('keydown',e=>{
  if(e.key === 'Shift'){ boostHeld = true; return; }   // Shift = boost flag
  if(e.repeat)return;
  const k=e.key.toLowerCase();
  const cmd=keyMap[k];
  if(cmd){
    e.preventDefault();
    pressed.add(k);
    if(cmd==='stop'){send('stop');return;}
    const out = activeCmd();
    if(out) send(out);
    startManual();
    return;
  }
  if(arrowMap[e.key]){
    e.preventDefault();
    if(e.repeat) return;       // OS auto-repeat would double-bump on each frame
    arrowsHeld.add(e.key);
    const [p,t] = arrowMap[e.key];
    bumpTarget(p, t);          // immediate first step
    startArrows();             // then repeat while held
  }
});

document.addEventListener('keyup',e=>{
  if(e.key === 'Shift'){ boostHeld = false; return; }  // releasing Shift = drop boost
  const k=e.key.toLowerCase();
  if(keyMap[k]){
    pressed.delete(k);
    if(!pressed.size){
      if(manualTx){clearInterval(manualTx);manualTx=null;}
      send('stop');
    }
    return;
  }
  if(arrowMap[e.key]){
    arrowsHeld.delete(e.key);
    if(!arrowsHeld.size && arrowTx){clearInterval(arrowTx); arrowTx=null;}
  }
});

// If the browser tab loses focus mid-hold, the keyup never fires and the
// robot would keep driving. Reset everything and force-stop.
window.addEventListener('blur', ()=>{
  if(pressed.size){
    pressed.clear();
    if(manualTx){clearInterval(manualTx); manualTx=null;}
    send('stop');
  }
  arrowsHeld.clear();
  if(arrowTx){clearInterval(arrowTx); arrowTx=null;}
  boostHeld = false;
});

/* Button hold-to-move */
document.querySelectorAll('.ctrl-btn').forEach(btn=>{
  const cmd=btn.dataset.cmd;
  if(!cmd)return;
  btn.addEventListener('mousedown',()=>{send(cmd);btn._hold=setInterval(()=>send(cmd),80);});
  btn.addEventListener('mouseup',()=>{clearInterval(btn._hold);send('stop');});
  btn.addEventListener('mouseleave',()=>{clearInterval(btn._hold);send('stop');});
});

/* ── Radar (Upgraded with Persistence) ───────────────── */
const radarCanvas=document.getElementById('radar');
const rctx=radarCanvas.getContext('2d');
let radarHistory=[];

function drawRadar(sweep){
  const w=radarCanvas.width,h=radarCanvas.height;
  const cx=w/2,cy=h-10,maxDist=150,sc=(h-20)/maxDist;
  rctx.clearRect(0,0,w,h);
  
  // Rings
  rctx.strokeStyle='rgba(34,211,238,0.1)';rctx.lineWidth=1;
  [30,60,90,120,150].forEach(d=>{
    const r=d*sc;rctx.beginPath();rctx.arc(cx,cy,r,Math.PI,2*Math.PI);rctx.stroke();
    rctx.fillStyle='rgba(100,116,139,0.5)';rctx.font='9px sans-serif';
    rctx.fillText(d+'cm',cx+r*0.7+2,cy-r*0.7);
  });

  if(!sweep||!sweep.length)return;
  
  // Add current sweep to history
  sweep.forEach(([angle,dist])=>{
    if(dist>0 && dist<maxDist) {
        radarHistory.push({a:angle, d:dist, t:Date.now()});
    }
  });
  // Keep only last 2 seconds of radar data for "persistence"
  const now=Date.now();
  radarHistory=radarHistory.filter(p=>(now-p.t)<2000);

  // Draw points with fading
  radarHistory.forEach(p=>{
    const age=(now-p.t)/2000;
    const rad=(p.a-90)*Math.PI/180, r=p.d*sc;
    const x=cx+Math.cos(rad)*r, y=cy+Math.sin(rad)*r;
    rctx.beginPath();rctx.arc(x,y,p.d<30?4:2.5,0,Math.PI*2);
    rctx.fillStyle=p.d<30?`rgba(239,68,68,${0.9-age})`:`rgba(34,211,238,${0.8-age})`;
    rctx.fill();
  });

  // Robot marker
  rctx.beginPath();rctx.moveTo(cx,cy-8);rctx.lineTo(cx-5,cy+2);rctx.lineTo(cx+5,cy+2);
  rctx.closePath();rctx.fillStyle='#22c55e';rctx.fill();
}

/* ── Floor Plan ──────────────────────────────────────── */
const ROOMS=[
  {id:'kitchen',    label:'Kitchen',     x:10, y:10, w:250,h:290,mid:1,dims:"9'6\"\u00d710'"},
  {id:'bathroom',   label:'Bath',        x:270,y:10, w:110,h:190,mid:3,dims:''},
  {id:'bedroom',    label:'Bedroom',     x:390,y:10, w:300,h:350,mid:2,dims:"12'1\"\u00d713'10\""},
  {id:'hallway',    label:'Hall',        x:270,y:210,w:110,h:90, mid:null,dims:''},
  {id:'living_room',label:'Living Room', x:10, y:310,w:680,h:320,mid:0,dims:"13'5\"\u00d715'2\""},
  {id:'entry',      label:'Entry',       x:10, y:640,w:120,h:50, mid:4,dims:''},
];
const APT_W=700,APT_H=700;
const fpCanvas=document.getElementById('floorplan');
const fpCtx=fpCanvas.getContext('2d');
const fpBody=document.getElementById('floorplanBody');
function fitFloorplanCanvas(){
  // Match the canvas drawing buffer to its rendered CSS size so the floor plan
  // never overflows or letterboxes. Keep a 1:1 buffer-to-display ratio for crisp lines.
  const w = Math.max(40, fpBody.clientWidth);
  const h = Math.max(40, fpBody.clientHeight);
  if(fpCanvas.width !== w) fpCanvas.width = w;
  if(fpCanvas.height !== h) fpCanvas.height = h;
}
fitFloorplanCanvas();
window.addEventListener('resize', fitFloorplanCanvas);
if(window.ResizeObserver) new ResizeObserver(fitFloorplanCanvas).observe(fpBody);

const radarBody=document.getElementById('radarBody');
function fitRadarCanvas(){
  // Radar is square; inscribe it in the smaller dimension of the panel-body.
  const w = Math.max(40, radarBody.clientWidth - 8);
  const h = Math.max(40, radarBody.clientHeight - 8);
  const size = Math.min(w, h);
  if(radarCanvas.width !== size) radarCanvas.width = size;
  if(radarCanvas.height !== size) radarCanvas.height = size;
}
fitRadarCanvas();
window.addEventListener('resize', fitRadarCanvas);
if(window.ResizeObserver) new ResizeObserver(fitRadarCanvas).observe(radarBody);

/* ── Resize gutters (drag handles between panels) ────────── */
function setupResizer(handle, target, axis, sign){
  // axis: 'h' for vertical drag (row-resize, height change); 'v' for horizontal drag (col-resize, width change).
  // sign: +1 if dragging in +coord direction grows the target; -1 if -coord direction grows it.
  if(!handle || !target) return;
  let startSize = 0, startCoord = 0;
  function onMove(ev){
    const cur = axis === 'h' ? ev.clientY : ev.clientX;
    const delta = (cur - startCoord) * sign;
    const newSize = Math.max(60, startSize + delta);
    target.style.flex = `0 0 ${newSize}px`;
  }
  function onUp(){
    document.removeEventListener('mousemove', onMove);
    document.removeEventListener('mouseup', onUp);
    handle.classList.remove('dragging');
    document.body.classList.remove(axis === 'h' ? 'resizing-h' : 'resizing-v');
  }
  handle.addEventListener('mousedown', e=>{
    e.preventDefault();
    startCoord = axis === 'h' ? e.clientY : e.clientX;
    startSize = axis === 'h' ? target.offsetHeight : target.offsetWidth;
    handle.classList.add('dragging');
    document.body.classList.add(axis === 'h' ? 'resizing-h' : 'resizing-v');
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
  });
}

const colLeftEl = document.querySelector('.col-left');
const brainPanelEl = document.querySelector('.brain-panel');
const radarPanelEl = document.querySelector('.radar-panel');
const logPanelEl = document.querySelector('.log-panel');
// Handle between video and brain: drag UP grows brain (sign=-1).
setupResizer(document.getElementById('resizeVideoBrain'), brainPanelEl, 'h', -1);
// Handle between radar and map: drag DOWN grows radar (sign=+1).
setupResizer(document.getElementById('resizeRadarMap'), radarPanelEl, 'h', +1);
// Handle between map and log: drag UP grows log (sign=-1).
setupResizer(document.getElementById('resizeMapLog'), logPanelEl, 'h', -1);
// Handle between columns: drag RIGHT grows col-left (sign=+1).
setupResizer(document.getElementById('resizeColumns'), colLeftEl, 'v', +1);
let trailPts=[];
let smoothX=350,smoothY=350,smoothHdg=0;
let placePulse=0;

/* ── Marker Survey (M0) ─────────────────────────────── */
const FLOOR_IDS = Array.from({length:22}, (_,i)=>10+i);  // 10..31
const WALL_LABELS = {0:'Living Rm', 1:'Kitchen', 2:'Bedroom', 3:'Bath', 4:'Entry'};
let surveyMode = false;
let activeMid = null;
let activeType = null;
let surveyPins = {};  // mirrors server state
let pendingFacing = null;  // {mid, x, y} awaiting facing choice
const surveyOverlay = document.getElementById('surveyOverlay');
const surveyToggle = document.getElementById('surveyToggle');
const surveyInfo = document.getElementById('surveyInfo');
const facingPopup = document.getElementById('facingPopup');

// Build the 22 floor-marker buttons.
const floorGrp = document.getElementById('floorBtnGrp');
FLOOR_IDS.forEach(mid=>{
  const b = document.createElement('button');
  b.className = 'mid-btn';
  b.dataset.mid = mid; b.dataset.type = 'floor';
  b.textContent = mid;
  floorGrp.appendChild(b);
});

surveyToggle.onclick = ()=>{
  surveyMode = !surveyMode;
  surveyToggle.classList.toggle('active', surveyMode);
  surveyOverlay.classList.toggle('active', surveyMode);
  if(!surveyMode){
    activeMid = null; pendingFacing = null;
    facingPopup.classList.remove('active');
    surveyOverlay.classList.remove('expanded');
  }
  refreshSurveyButtons();
};

const surveyBar = document.getElementById('surveyBar');
const surveyChev = document.getElementById('surveyChev');
surveyBar.onclick = (e)=>{
  // Expanding from collapsed; collapse via the chev when expanded.
  if(!surveyOverlay.classList.contains('expanded')){
    surveyOverlay.classList.add('expanded');
    surveyChev.textContent = '▼';
  } else if(e.target === surveyChev){
    surveyOverlay.classList.remove('expanded');
    surveyChev.textContent = '▲';
  }
};

function refreshSurveyButtons(){
  document.querySelectorAll('.mid-btn').forEach(b=>{
    const mid = parseInt(b.dataset.mid);
    b.classList.toggle('pinned', surveyPins[mid] !== undefined);
    b.classList.toggle('active', mid === activeMid);
  });
  const wallDone = [0,1,2,3,4].filter(m=>surveyPins[m]).length;
  const floorDone = FLOOR_IDS.filter(m=>surveyPins[m]).length;
  const summary = `Survey: ${wallDone}/5 wall · ${floorDone}/22 floor`;
  document.getElementById('surveyBarStatus').textContent =
    activeMid !== null
      ? `${summary} · active: ${activeMid}${activeType==='wall'?' ('+(WALL_LABELS[activeMid]||'Wall')+')':''}`
      : summary;
  if(activeMid !== null){
    const lbl = activeType === 'wall' ? `${activeMid} (${WALL_LABELS[activeMid]||'Wall'})` : `${activeMid} (Floor)`;
    const pinned = surveyPins[activeMid];
    surveyInfo.textContent = pinned
      ? `${lbl} — pinned at (${Math.round(pinned.x)}, ${Math.round(pinned.y)}). Click to re-pin.`
      : `${lbl} — click on the floor plan to pin.`;
  } else if(surveyMode){
    surveyInfo.textContent = `Pinned: ${wallDone}/5 wall, ${floorDone}/22 floor`;
  }
}

document.querySelectorAll('.mid-btn').forEach(b=>{
  b.addEventListener('click', ()=>{
    activeMid = parseInt(b.dataset.mid);
    activeType = b.dataset.type;
    pendingFacing = null;
    facingPopup.classList.remove('active');
    refreshSurveyButtons();
  });
});

document.getElementById('surveySaveBtn').onclick = async ()=>{
  const r = await fetch('/api/save_survey', {method:'POST', headers:{'Content-Type':'application/json'}, body:'{}'});
  const j = await r.json();
  surveyInfo.textContent = j.ok ? `Saved ${j.wall} wall + ${j.floor} floor markers to disk. Restart agent to apply.` : 'Save failed — check server log.';
};
document.getElementById('surveyClearBtn').onclick = async ()=>{
  if(!confirm('Clear all surveyed markers (in-memory; disk untouched until you Save)?')) return;
  for(const mid of Object.keys(surveyPins)){
    await fetch('/api/unpin_marker', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({id:parseInt(mid)})});
  }
  surveyPins = {}; activeMid = null; refreshSurveyButtons();
};

document.querySelectorAll('.facing-popup .face-btn').forEach(b=>{
  if(!b.dataset.deg) return;
  b.addEventListener('click', async ()=>{
    if(!pendingFacing) return;
    const facing_deg = parseInt(b.dataset.deg);
    const {mid, x, y} = pendingFacing;
    await fetch('/api/pin_marker', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({id:mid, x, y, type:'wall', facing_deg})});
    surveyPins[mid] = {x, y, type:'wall', facing_deg};
    pendingFacing = null;
    facingPopup.classList.remove('active');
    activeMid = null;
    refreshSurveyButtons();
  });
});

/* Click-to-place / Survey-pin: click on map either pins active survey marker or sets robot position */
fpCanvas.style.cursor='crosshair';
fpCanvas.onclick=async e=>{
  const rect=fpCanvas.getBoundingClientRect();
  const cx=e.clientX-rect.left, cy=e.clientY-rect.top;
  const cw=fpCanvas.width,ch=fpCanvas.height;
  const s=Math.min(cw/APT_W,ch/APT_H)*0.92;
  const ox=(cw-APT_W*s)/2,oy=(ch-APT_H*s)/2;
  const ax=(cx-ox)/s, ay=(cy-oy)/s;
  if(ax<0||ax>APT_W||ay<0||ay>APT_H)return;

  if(surveyMode && activeMid !== null){
    if(activeType === 'wall'){
      // Two-step: position then facing.
      pendingFacing = {mid:activeMid, x:ax, y:ay};
      // Position the popup near the click, clamped to viewport.
      const popupRect = {w:120, h:130};
      let px = e.clientX + 12, py = e.clientY + 12;
      if(px + popupRect.w > window.innerWidth) px = e.clientX - popupRect.w - 12;
      if(py + popupRect.h > window.innerHeight) py = e.clientY - popupRect.h - 12;
      facingPopup.style.left = px + 'px';
      facingPopup.style.top = py + 'px';
      facingPopup.classList.add('active');
      surveyInfo.textContent = `Marker ${activeMid} at (${Math.round(ax)}, ${Math.round(ay)}) — pick facing direction.`;
    } else {
      await fetch('/api/pin_marker', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({id:activeMid, x:ax, y:ay, type:'floor'})});
      surveyPins[activeMid] = {x:ax, y:ay, type:'floor'};
      // Auto-advance to next unpinned floor marker.
      const next = FLOOR_IDS.find(m => !surveyPins[m] && m > activeMid) || FLOOR_IDS.find(m => !surveyPins[m]);
      activeMid = next ?? null;
      activeType = next != null ? 'floor' : null;
      refreshSurveyButtons();
    }
    return;
  }

  // Default: set robot position.
  smoothX=ax;smoothY=ay;
  placePulse=Date.now();
  trailPts=[];
  const room=ROOMS.find(r=>ax>=r.x&&ax<=r.x+r.w&&ay>=r.y&&ay<=r.y+r.h);
  document.getElementById('mapPos').textContent='\u{1f4cd} '+(room?room.label:'Placed')+' ('+Math.round(ax)+','+Math.round(ay)+')';
  fetch('/api/set_position',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:ax,y:ay})});
};

  function drawFloorplan(d){
    const cw=fpCanvas.width,ch=fpCanvas.height;
    const s=Math.min(cw/APT_W,ch/APT_H)*0.92;
    const ox=(cw-APT_W*s)/2,oy=(ch-APT_H*s)/2;
    fpCtx.clearRect(0,0,cw,ch);
    fpCtx.fillStyle='#080c14';fpCtx.fillRect(0,0,cw,ch);

    const curRoom=(d.aruco_room||d.zone||'').toLowerCase();

    ROOMS.forEach(r=>{
      const rx=ox+r.x*s,ry=oy+r.y*s,rw=r.w*s,rh=r.h*s;
      const active=curRoom&&(curRoom===r.label.toLowerCase()||curRoom.includes(r.id.replace('_',' ')));

      fpCtx.fillStyle=active?'rgba(34,211,238,0.1)':'rgba(30,41,59,0.5)';
      fpCtx.fillRect(rx,ry,rw,rh);
      fpCtx.strokeStyle=active?'#22d3ee':'#334155';
      fpCtx.lineWidth=active?2:1;
      fpCtx.strokeRect(rx,ry,rw,rh);

      fpCtx.textAlign='center';fpCtx.textBaseline='middle';
      fpCtx.fillStyle=active?'#e2e8f0':'#64748b';
      fpCtx.font=(active?'bold ':'')+'12px sans-serif';
      fpCtx.fillText(r.label,rx+rw/2,ry+rh/2-(r.dims?9:0));

      if(r.dims){
        fpCtx.fillStyle='#475569';fpCtx.font='9px sans-serif';
        fpCtx.fillText(r.dims,rx+rw/2,ry+rh/2+9);
      }
    });

    // Draw surveyed markers (M0). Wall markers get a facing arrow.
    if(d.survey_pins){
      Object.entries(d.survey_pins).forEach(([midStr, pin])=>{
        const mid = parseInt(midStr);
        const mx = ox + pin.x * s, my = oy + pin.y * s;
        const isWall = pin.type === 'wall';
        const isActive = (mid === activeMid && surveyMode);
        fpCtx.beginPath();
        fpCtx.arc(mx, my, isActive ? 6 : 4, 0, Math.PI*2);
        fpCtx.fillStyle = isWall ? '#f59e0b' : '#22d3ee';
        fpCtx.fill();
        if(isActive){ fpCtx.strokeStyle='#fff'; fpCtx.lineWidth=1.5; fpCtx.stroke(); }
        // Facing arrow for wall markers
        if(isWall && pin.facing_deg !== undefined){
          const rad = pin.facing_deg * Math.PI / 180;
          fpCtx.beginPath();
          fpCtx.moveTo(mx, my);
          fpCtx.lineTo(mx + Math.sin(rad) * 12, my - Math.cos(rad) * 12);
          fpCtx.strokeStyle = '#f59e0b'; fpCtx.lineWidth = 2; fpCtx.stroke();
        }
        // ID label
        fpCtx.fillStyle = '#e2e8f0';
        fpCtx.font = 'bold 9px monospace';
        fpCtx.textAlign = 'center'; fpCtx.textBaseline = 'middle';
        fpCtx.fillText(mid, mx, my - 9);
      });
    }

    // Draw discovered obstacles (SLAM)
    if(d.obstacles){
      fpCtx.fillStyle='rgba(148,163,184,0.4)';
      d.obstacles.forEach(p=>{
        fpCtx.beginPath();
        fpCtx.arc(ox+p[0]*s, oy+p[1]*s, 2, 0, Math.PI*2);
        fpCtx.fill();
      });
    }

    // Outer apartment wall
    fpCtx.strokeStyle='#475569';fpCtx.lineWidth=2;
    fpCtx.strokeRect(ox+5*s,oy+5*s,(APT_W-10)*s,(APT_H-10)*s);

    // Smooth interpolation toward server position
    const tx=d.robot_x??350,ty=d.robot_y??350,th=d.heading??0;
    smoothX+=(tx-smoothX)*0.25;
    smoothY+=(ty-smoothY)*0.25;
    let dh=th-smoothHdg;
    if(dh>180)dh-=360; if(dh<-180)dh+=360;
    smoothHdg=(smoothHdg+dh*0.3+360)%360;

    // Trail
    trailPts.push({x:smoothX,y:smoothY,t:Date.now()});
    if(trailPts.length>120)trailPts=trailPts.slice(-80);
    const now=Date.now();
    trailPts.forEach(p=>{
      const age=(now-p.t)/1000;
      if(age>60)return;
      const alpha=Math.max(0.03,0.35-age*0.006);
      fpCtx.beginPath();
      fpCtx.arc(ox+p.x*s,oy+p.y*s,1.5,0,Math.PI*2);
      fpCtx.fillStyle='rgba(34,197,94,'+alpha+')';
      fpCtx.fill();
    });

    const px=ox+smoothX*s,py=oy+smoothY*s;
    const hdg=smoothHdg*Math.PI/180;

    // Heading line
    fpCtx.beginPath();fpCtx.moveTo(px,py);
    fpCtx.lineTo(px+Math.sin(hdg)*18,py-Math.cos(hdg)*18);
    fpCtx.strokeStyle='rgba(34,211,238,0.7)';fpCtx.lineWidth=2;fpCtx.stroke();

    // Robot dot
    fpCtx.beginPath();fpCtx.arc(px,py,5,0,Math.PI*2);
    fpCtx.fillStyle='#22c55e';fpCtx.fill();
    fpCtx.strokeStyle='#fff';fpCtx.lineWidth=1.5;fpCtx.stroke();

    // Pulse ring (animated)
    const pulse=6+Math.sin(now/300)*3;
    fpCtx.beginPath();fpCtx.arc(px,py,pulse,0,Math.PI*2);
    fpCtx.strokeStyle='rgba(34,197,94,0.3)';fpCtx.lineWidth=1;fpCtx.stroke();

    // Place-marker flash
    if(placePulse&&(now-placePulse)<1500){
      const pAge=(now-placePulse)/1500;
      const pR=8+pAge*30;
      fpCtx.beginPath();fpCtx.arc(px,py,pR,0,Math.PI*2);
      fpCtx.strokeStyle='rgba(245,158,11,'+(0.8-pAge*0.8)+')';
      fpCtx.lineWidth=2;fpCtx.stroke();
    }

    // Compass rose (top-right corner)
    const ccx=cw-20,ccy=25;
    fpCtx.save();fpCtx.translate(ccx,ccy);fpCtx.rotate(-hdg);
    fpCtx.beginPath();fpCtx.moveTo(0,-12);fpCtx.lineTo(-4,6);fpCtx.lineTo(4,6);fpCtx.closePath();
    fpCtx.fillStyle='rgba(239,68,68,0.7)';fpCtx.fill();
    fpCtx.beginPath();fpCtx.moveTo(0,12);fpCtx.lineTo(-4,-6);fpCtx.lineTo(4,-6);fpCtx.closePath();
    fpCtx.fillStyle='rgba(100,116,139,0.4)';fpCtx.fill();
    fpCtx.restore();
    fpCtx.fillStyle='#64748b';fpCtx.font='8px sans-serif';fpCtx.textAlign='center';
    fpCtx.fillText('N',ccx,ccy-16);
  }

/* ── Utilities ───────────────────────────────────────── */
function formatUptime(s){
  const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
  if(h>0)return h+'h '+m+'m';
  if(m>0)return m+'m '+sec+'s';
  return sec+'s';
}

/* ── Poll Loop ───────────────────────────────────────── */
let lastTick=0;
async function poll(){
  try{
    const r=await fetch('/api/state');
    const d=await r.json();
    document.getElementById('statTick').textContent=d.tick;
    document.getElementById('statUptime').textContent=formatUptime(d.uptime);
    document.getElementById('statLatency').textContent=d.latency?d.latency.toFixed(2)+'s':'---';
    document.getElementById('statZone').textContent=d.zone||'---';

    // ArUco tag indicator
    const aTag=document.getElementById('arucoTag');
    if(d.aruco_room){aTag.style.display='inline-block';aTag.textContent='\u25c6 '+d.aruco_room;}
    else{aTag.style.display='none';}

    // Alert bar
    const ab=document.getElementById('alertBar');
    if(d.emergency_stop){ab.className='alert-bar active emergency';ab.textContent='EMERGENCY STOP \u2014 OBSTACLE < 30cm';}
    else{ab.className='alert-bar';}

    // Brain state
    const actEl=document.getElementById('brainAction');
    actEl.textContent=d.action?d.action.toUpperCase():'IDLE';
    actEl.style.color=({forward:'var(--ok)',backward:'var(--warn)',left:'var(--accent2)',right:'var(--accent2)',stop:'var(--danger)'})[d.action]||'var(--text-dim)';
    document.getElementById('brainObs').textContent=d.observation||'---';
    document.getElementById('brainReason').textContent=d.reasoning||'---';
    const dist=d.forward_dist<900?d.forward_dist.toFixed(1)+' cm':'UNKNOWN';
    document.getElementById('brainDist').textContent=dist;
    const distPct=d.forward_dist<900?Math.min(100,d.forward_dist/1.5):100;
    const distBar=document.getElementById('distBar');
    distBar.style.width=distPct+'%';
    distBar.style.background=d.forward_dist<30?'var(--danger)':d.forward_dist<60?'var(--warn)':'var(--ok)';
    // Laser tile (VL53L0X). tof_distance is null when out-of-range.
    const tofVal=(d.tof_distance==null)?'---':d.tof_distance.toFixed(1)+' cm';
    document.getElementById('brainTof').textContent=tofVal;
    const tofBar=document.getElementById('tofBar');
    if(d.tof_distance==null){tofBar.style.width='0';}
    else{
      tofBar.style.width=Math.min(100,d.tof_distance/2)+'%';
      tofBar.style.background=d.tof_distance<30?'var(--danger)':d.tof_distance<60?'var(--warn)':'var(--accent2)';
    }
    document.getElementById('brainHeading').textContent=Math.round(d.heading)+' deg'+(d.has_imu?' (gz:'+d.imu_gyro_z.toFixed(1)+')':'');
    document.getElementById('imuBadge').style.display=d.has_imu?'inline':'none';
    document.getElementById('brainSpeed').textContent=(d.speed||0).toFixed(1)+' cm/s | '+(d.mileage||0).toFixed(1)+' cm';
    // IMU accel tile — three axes in g
    const ac=d.imu_accel||[0,0,0];
    document.getElementById('brainAccel').textContent='x: '+(ac[0]||0).toFixed(2)+'  y: '+(ac[1]||0).toFixed(2)+'  z: '+(ac[2]||0).toFixed(2);
    const gs=d.grayscale||[0,0,0];
    document.getElementById('brainGS').textContent=gs.map(v=>v.toFixed?v.toFixed(0):v).join(' / ');
    ['gs0','gs1','gs2'].forEach((id,i)=>{
      const pct=Math.min(100,(gs[i]/65535)*100);
      document.getElementById(id).style.width=pct+'%';
    });

    // Map position header
    const posStr=d.aruco_room?'\u25c6 '+d.aruco_room:'('+Math.round(d.robot_x)+','+Math.round(d.robot_y)+')';
    document.getElementById('mapPos').textContent=posStr+' hdg '+Math.round(d.heading)+' deg';

    // Hydrate surveyed-marker state from the server on first poll only — after that
    // the JS is authoritative (avoids overwriting in-flight pins on the next poll tick).
    if(d.survey_pins && !window._surveyHydrated){
      window._surveyHydrated = true;
      for(const [k, v] of Object.entries(d.survey_pins)) surveyPins[parseInt(k)] = v;
      refreshSurveyButtons();
    }

    try {
      drawRadar(d.radar_sweep);
      drawFloorplan(d);
    } catch(e) { console.error("Draw error:", e); }

    // Action log
    const logBody=document.getElementById('logBody');
    if(d.tick!==lastTick || (d.logs && d.logs.length !== parseInt(document.getElementById('logCount').textContent))){
      lastTick=d.tick;
      const logs=d.logs||[];
      logBody.innerHTML=logs.map(l=>{
        const actMatch=l.match(/\[(\w+)\]/);
        const act=actMatch?actMatch[1].toLowerCase():'';
        return '<div class="log-entry"><span class="act act-'+act+'">['+act.toUpperCase()+']</span> '+l.replace(/\[\w+\]\s*/,'')+'</div>';
      }).join('');
      logBody.scrollTop=logBody.scrollHeight;
      document.getElementById('logCount').textContent=logs.length;
    }
    // Camera resolution + dynamic aspect-ratio so the panel matches the
    // stream and we don't get black bars eating the brain panel's space.
    const vid=document.getElementById('video');
    if(vid.naturalWidth){
      document.getElementById('camRes').textContent=vid.naturalWidth+'x'+vid.naturalHeight;
      const aspect=vid.naturalWidth/vid.naturalHeight;
      const panel=document.querySelector('.video-panel');
      if(panel && Math.abs(parseFloat(panel.style.aspectRatio||'1.333')-aspect)>0.01){
        panel.style.aspectRatio=aspect.toFixed(4);
      }
    }

    document.getElementById('statusDot').style.background='var(--ok)';
  }catch(e){
    document.getElementById('statusDot').style.background='var(--danger)';
  }
  setTimeout(poll,300);
}
poll();
</script>
</body>
</html>"""

def start_dashboard():
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    try:
        server = socketserver.ThreadingTCPServer(("", PORT), DashboardHandler)
    except OSError:
        # Port already in use from a previous session — kill it and retry
        import socket, os, signal
        try:
            s = socket.socket()
            s.connect(('127.0.0.1', PORT))
            s.close()
        except Exception:
            pass
        import subprocess
        pids = subprocess.run(['lsof', '-ti', f':{PORT}'], capture_output=True, text=True).stdout.strip().split()
        for pid in pids:
            try: os.kill(int(pid), signal.SIGKILL)
            except Exception: pass
        import time as _t; _t.sleep(0.5)
        server = socketserver.ThreadingTCPServer(("", PORT), DashboardHandler)
    server.daemon_threads = True
    print(f"[Dashboard] http://localhost:{PORT}")
    threading.Thread(target=server.serve_forever, daemon=True).start()


def set_vision_module(vision):
    with state.lock:
        state.vision_module = vision


def set_pico_hw(hw):
    """Register the PicoHardwareModule so /api/servo can dispatch to it
    directly, bypassing the agent's manual-cmd queue."""
    with state.lock:
        state.pico_hw = hw


def update_dashboard(frame=None, map_str=None, log_msg=None, **kwargs):
    with state.lock:
        if frame is not None:
            state.latest_frame = frame
        if map_str is not None:
            state.map_data = map_str
        if log_msg is not None:
            state.logs.append(log_msg)
            if len(state.logs) > 200:
                state.logs = state.logs[-100:]
        for k, v in kwargs.items():
            if k in state.telemetry:
                state.telemetry[k] = v


def get_manual_command():
    with state.lock:
        if state.manual_cmds:
            return state.manual_cmds.pop(0)
        return None


def get_marker_updates():
    with state.lock:
        return dict(state.marker_positions)


def get_position_override():
    with state.lock:
        pos = state.position_override
        state.position_override = None
        return pos
