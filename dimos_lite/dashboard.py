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
        self.map_data = ""
        self.logs = []
        self.lock = threading.Lock()
        self.manual_cmd = None
        self.position_override = None
        self.telemetry = {
            "tick": 0,
            "heading": 0,
            "zone": "Unknown",
            "forward_dist": 999,
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
            "battery_v": 0.0,
            "marker_positions": {},
            "obstacles": [],
        }
        self.marker_positions = {}
        self.start_time = time.time()

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
                state.manual_cmd = cmd
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
.col-left{flex:0 0 55%;display:flex;flex-direction:column;border-right:1px solid var(--border)}
.col-right{flex:1;display:flex;flex-direction:column;overflow:hidden}
.panel{background:var(--surface);border:1px solid var(--border);margin:6px;border-radius:8px;overflow:hidden;display:flex;flex-direction:column}
.panel-head{display:flex;align-items:center;justify-content:space-between;padding:8px 12px;border-bottom:1px solid var(--border);font-size:11px;text-transform:uppercase;letter-spacing:.8px;color:var(--text-dim);font-weight:600}
.panel-body{flex:1;overflow:hidden;position:relative}
.video-panel{flex:1}
.video-panel .panel-body{background:#000}
.video-panel img{width:100%;height:100%;object-fit:contain;display:block}
.brain-panel{flex:0 0 200px}
.radar-panel{flex:0 0 280px}
.radar-panel .panel-body{display:flex;align-items:center;justify-content:center;padding:8px}
.map-panel{flex:1}
.map-panel .panel-body{display:flex;align-items:center;justify-content:center;padding:4px;overflow:hidden}
.log-panel{flex:0 0 180px}
.log-panel .panel-body{overflow-y:auto;padding:4px 8px;font-family:'JetBrains Mono','Fira Code',monospace;font-size:11px;line-height:1.6}
.log-entry{padding:2px 0;border-bottom:1px solid rgba(255,255,255,.03)}
.log-entry .act{font-weight:700}
.log-entry .act-forward{color:var(--ok)}
.log-entry .act-backward{color:var(--warn)}
.log-entry .act-left,.log-entry .act-right{color:var(--accent2)}
.log-entry .act-stop{color:var(--danger)}
canvas#radar{display:block}
canvas#floorplan{display:block}
.brain-grid{display:grid;grid-template-columns:1fr 1fr;gap:1px;height:100%;background:var(--border)}
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
</style>
</head>
<body>
<div class="topbar">
  <div class="topbar-left">
    <div class="status-dot" id="statusDot"></div>
    <div class="logo">AGENTIC PET OS</div>
  </div>
  <div class="topbar-stats">
    <span>TICK <span class="val" id="statTick">0</span></span>
    <span>UPTIME <span class="val" id="statUptime">0s</span></span>
    <span>LATENCY <span class="val" id="statLatency">---</span></span>
    <span>ZONE <span class="val" id="statZone">---</span></span>
    <span id="arucoTag" style="display:none" class="aruco-tag"></span>
    <span id="battSpan" title="Battery Voltage">🔋 <span class="val" id="statBatt">---</span></span>
  </div>
</div>
<div class="alert-bar" id="alertBar"></div>
<div class="main">
  <div class="col-left">
    <div class="panel video-panel">
      <div class="panel-head"><span>LIVE VISION</span><span id="camRes">---</span></div>
      <div class="panel-body"><img id="video" src="/video_feed" alt="Camera"></div>
    </div>
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
            <div class="label">Forward Distance</div>
            <div class="value" id="brainDist">---</div>
            <div class="sensor-bar"><div class="sensor-bar-fill" id="distBar" style="width:0;background:var(--ok)"></div></div>
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
    <div class="controls-row">
      <button class="ctrl-btn" data-cmd="forward">W Forward</button>
      <button class="ctrl-btn" data-cmd="left">A Left</button>
      <button class="ctrl-btn danger" data-cmd="stop">STOP</button>
      <button class="ctrl-btn" data-cmd="right">D Right</button>
      <button class="ctrl-btn" data-cmd="backward">S Back</button>
    </div>
  </div>
  <div class="col-right">
    <div class="panel radar-panel">
      <div class="panel-head"><span>RADAR SWEEP</span><span id="radarCount">0 pts</span></div>
      <div class="panel-body"><canvas id="radar" width="260" height="260"></canvas></div>
    </div>
    <div class="panel map-panel">
      <div class="panel-head"><span>FLOOR PLAN</span><span id="mapPos">---</span></div>
      <div class="panel-body"><canvas id="floorplan" width="380" height="380"></canvas></div>
    </div>
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

/* ── FPV-style real-time control ─────────────────────── */
const keyMap={w:'forward',a:'left',s:'backward',d:'right',' ':'stop',ArrowUp:'forward',ArrowLeft:'left',ArrowDown:'backward',ArrowRight:'right'};
const pressed=new Set();
let manualTx=null;

function startManual(){
  if(manualTx)return;
  manualTx=setInterval(()=>{
    if(!pressed.size){clearInterval(manualTx);manualTx=null;return;}
    const last=[...pressed].pop();
    const cmd=keyMap[last];
    if(cmd&&cmd!=='stop')send(cmd);
  },80);
}
document.addEventListener('keydown',e=>{
  const cmd=keyMap[e.key];
  if(!cmd||e.repeat)return;
  e.preventDefault();
  pressed.add(e.key);
  if(cmd==='stop'){send('stop');return;}
  send(cmd);
  startManual();
});
document.addEventListener('keyup',e=>{
  pressed.delete(e.key);
  if(!pressed.size){send('stop');if(manualTx){clearInterval(manualTx);manualTx=null;}}
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
let trailPts=[];
let smoothX=350,smoothY=350,smoothHdg=0;
let placePulse=0;

/* Click-to-place: click on map to set robot position */
  // Marker Placement (Shift+Click)
  fpCanvas.style.cursor='crosshair';
  fpCanvas.onclick=e=>{
    const rect=fpCanvas.getBoundingClientRect();
    const cx=e.clientX-rect.left, cy=e.clientY-rect.top;
    const cw=fpCanvas.width,ch=fpCanvas.height;
    const s=Math.min(cw/APT_W,ch/APT_H)*0.92;
    const ox=(cw-APT_W*s)/2,oy=(ch-APT_H*s)/2;
    const ax=(cx-ox)/s, ay=(cy-oy)/s;
    if(ax<0||ax>APT_W||ay<0||ay>APT_H)return;

    if(e.shiftKey){
      const mid = prompt("Enter Marker ID for this spot (0-4):", "0");
      if(mid!==null){
        fetch('/api/set_marker',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({id:parseInt(mid),x:ax,y:ay})});
      }
      return;
    }

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

    // Draw user-placed markers
    if(d.marker_positions){
      Object.entries(d.marker_positions).forEach(([mid,pos])=>{
        const mx=ox+pos[0]*s, my=oy+pos[1]*s;
        fpCtx.fillStyle='#f59e0b';fpCtx.font='10px sans-serif';
        fpCtx.fillText('\u25c6',mx,my);
        fpCtx.font='7px sans-serif';fpCtx.fillText('ID:'+mid,mx,my+8);
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

    // Battery
    const bv=d.battery_v||0;
    const battEl=document.getElementById('statBatt');
    const battSpan=document.getElementById('battSpan');
    if(bv>1.0){
      const pct=Math.min(100,Math.max(0,((bv-6.0)/(8.4-6.0))*100));
      const col=bv>7.0?'var(--ok)':bv>6.5?'var(--warn)':'var(--danger)';
      battEl.textContent=bv.toFixed(2)+'V ('+Math.round(pct)+'%)';
      battEl.style.color=col;
      battSpan.title='Battery voltage';
    } else {
      battEl.textContent='N/A';
      battEl.style.color='var(--text-dim)';
      battSpan.title='No battery ADC wired — add voltage divider to free GPIO';
    }

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
    document.getElementById('brainHeading').textContent=Math.round(d.heading)+' deg'+(d.has_imu?' (gz:'+d.imu_gyro_z.toFixed(1)+')':'');
    document.getElementById('imuBadge').style.display=d.has_imu?'inline':'none';
    document.getElementById('brainSpeed').textContent=(d.speed||0).toFixed(1)+' cm/s | '+(d.mileage||0).toFixed(1)+' cm';
    const gs=d.grayscale||[0,0,0];
    document.getElementById('brainGS').textContent=gs.map(v=>v.toFixed?v.toFixed(0):v).join(' / ');
    ['gs0','gs1','gs2'].forEach((id,i)=>{
      const pct=Math.min(100,(gs[i]/65535)*100);
      document.getElementById(id).style.width=pct+'%';
    });

    // Map position header
    const posStr=d.aruco_room?'\u25c6 '+d.aruco_room:'('+Math.round(d.robot_x)+','+Math.round(d.robot_y)+')';
    document.getElementById('mapPos').textContent=posStr+' hdg '+Math.round(d.heading)+' deg';

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
    // Camera resolution
    const vid=document.getElementById('video');
    if(vid.naturalWidth)document.getElementById('camRes').textContent=vid.naturalWidth+'x'+vid.naturalHeight;

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
        cmd = state.manual_cmd
        state.manual_cmd = None
        return cmd


def get_marker_updates():
    with state.lock:
        return dict(state.marker_positions)


def get_position_override():
    with state.lock:
        pos = state.position_override
        state.position_override = None
        return pos
