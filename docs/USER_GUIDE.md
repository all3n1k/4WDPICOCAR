# Pico-4WD Agentic Pet OS — User Guide

<!-- GUIDE VERSION: v5.5 | Matches firmware v5.5 | Last updated: 2026-04-23 -->
<!-- Changelog:
  v5.5 - High-Definition Vision: ESP32-CAM bumped to SVGA (800x600)
       - Vision HUD: AR-style bounding boxes and ID/Distance labels on stream
       - Pose Estimation SLAM: Exact distance/bearing snapping from ArUco markers
       - Live SLAM Mapping: Discovered obstacles now appear as dots on dashboard
       - PWM LED Overhaul: Support for analog LEDs on Pins 19, 20, 21, 22
       - Turn Signals: Automatic blinking headlamps (Fast=Left, Slow=Right)
       - LLM Stability: 90s timeout and optimized token predict (512) for 31B model
  v4.1 - Added MPU6050 IMU (verified, live heading correction)
       - Upgraded LLM to gemma4:31b
       - Vision resolution bumped to 896x896 for chair-leg detection
       - ArUco marker localization live (fuses with RSSI)
       - New full-featured dashboard (radar canvas, floor plan, IMU badge)
       - Safety reflex thread split from main control loop (think thread)
       - Pico firmware auto-stop on command timeout (CMD_TIMEOUT_MS=2000)
  v4.0 - Multi-stream architecture (6 streams)
       - Mapper module (--map mode)
       - Dashboard added
  v3.5 - Super-smooth radar sweep (5° steps, 10ms tick)
  v3.x - Original LLM navigation
-->

## System Overview

The Pico-4WD Agentic Pet OS is an autonomous robot platform built on three layers:

```
┌──────────────────────────────────────────────────────────────┐
│  Mac Brain (Python 3.14)                                     │
│  ┌───────────┐  ┌─────────────────────┐  ┌───────────────┐  │
│  │ Dashboard  │  │ AgentCore (Ollama)  │  │ Localization  │  │
│  │ :8080     │  │ gemma4:31b (31B)    │  │ RSSI + ArUco  │  │
│  └───────────┘  └─────────────────────┘  └───────────────┘  │
│       ▲                   │                      ▲           │
│       │  Think thread     │ cmd_vel         zone + pos       │
│       │  Reflex thread    ▼                      │           │
│  ┌──────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │ Vision   │  │ PicoHW      │  │ Mapper (--map mode)     │ │
│  │ (OpenCV) │  │ (WebSocket) │  │ perimeter + lawnmower   │ │
│  └────┬─────┘  └──────┬──────┘  └─────────────────────────┘ │
└───────┼───────────────┼──────────────────────────────────────┘
        │ MJPEG         │ WebSocket JSON
        │ :81           │ :8765
   ┌────┴────┐   ┌───────┴──────────────────────────────┐
   │ESP32-CAM│   │ Pico RP2040 (firmware v4.1)           │
   │ WiFi    │   │ ┌────────┐ ┌─────────────────────┐    │
   │ OV2640  │   │ │ESP8266 │ │ 4x Motors + Servo   │    │
   │         │   │ │(UART1) │ │ HC-SR04 Sonar       │    │
   │         │   │ │GP4/GP5 │ │ 3x Grayscale        │    │
   │         │   │ └────────┘ │ 24x WS2812 LEDs     │    │
   └─────────┘   │            │ MPU6050 IMU (I2C1)  │    │
                 │            │   SDA=GP2, SCL=GP3  │    │
                 │            └─────────────────────┘    │
                 └───────────────────────────────────────┘
```

## Network Addresses

| Device | Address | Protocol |
|--------|---------|----------|
| Pico-4WD WebSocket | `ws://192.168.1.217:8765` | JSON over WebSocket |
| ESP32-CAM Stream | `http://192.168.1.216:81/stream` | MJPEG |
| ESP32-CAM Control | `http://192.168.1.216/control?var=X&val=Y` | HTTP GET |
| Dashboard | `http://localhost:8080` | HTTP + MJPEG |
| Ollama LLM | `http://digitalstorm:11434/api/generate` | HTTP JSON |

## Hardware Inventory

### Pico-4WD Robot Car

| Component | Pins | Notes |
|-----------|------|-------|
| Left Front Motor | GP17, GP16 | dir=1 |
| Right Front Motor | GP15, GP14 | dir=1 |
| Left Rear Motor | GP13, GP12 | dir=-1 |
| Right Rear Motor | GP11, GP10 | dir=1 |
| Servo (radar) | GP18 | Sweep: -75° to +90° (asymmetric) |
| Ultrasonic Trig | GP6 | HC-SR04 |
| Ultrasonic Echo | GP7 | HC-SR04 |
| Left Speed Encoder | GP8 | Photo interruptor |
| Right Speed Encoder | GP9 | Photo interruptor |
| Grayscale Left | GP26 (ADC0) | Cliff/line detect |
| Grayscale Center | GP27 (ADC1) | Cliff/line detect |
| Grayscale Right | GP28 (ADC2) | Cliff/line detect |
| WS2812 LEDs (24x) | GP19 | 8 rear + 8 bottom-L + 8 bottom-R |
| ESP8266 (WiFi) | GP4 TX, GP5 RX | UART1, 115200 baud |
| **MPU6050 IMU** | **GP2 SDA, GP3 SCL** | **I2C1, addr 0x68 — verified working** |
| Onboard LED | GP25 | Status indicator |

**Free GPIO**: GP0, GP1, GP20, GP21, GP22

### MPU6050 IMU (Active)

The MPU6050 is connected on I2C bus 1 (GP2=SDA, GP3=SCL, 400kHz) and is automatically detected at boot. It provides:
- **Fused heading** (degrees, continuous) — replaces dead-reckoning for turns
- **Gyroscope Z-axis** (deg/s) — shown on dashboard as `gz`
- **Accelerometer XYZ** (g) — available for tilt/shock detection

The Pico firmware calls `imu.calibrate()` at boot (hold still for ~3 seconds). If the IMU is absent, the firmware degrades gracefully and the Mac brain falls back to time-estimated heading.

### ESP32-CAM

| Spec | Value |
|------|-------|
| Board | AI-THINKER ESP32-CAM |
| Camera | OV2640 |
| WiFi | Stationite (STA mode) |
| IP | 192.168.1.216 (may require power cycle if unresponsive) |
| Flash LED | GPIO 4 (double-blink on WiFi connect) |
| Power | 5V from robot servo header (SIG+GND) |

> **Note:** The ESP32-CAM can become unresponsive if battery voltage drops below ~4.8V. If the stream hangs, check battery charge and power-cycle the robot.

---

## Running the System

### Quick Start

```bash
cd /Users/neo/Downloads/pico_4wd_car-main

# 1. Power on Pico-4WD battery switch (MPU6050 calibrates for ~3s on boot)
# 2. ESP32-CAM joins network automatically

# 3. Start the brain (normal navigation mode)
python3 main_os.py

# 4. Open the control dashboard
open http://localhost:8080
```

### Mapping Mode

Runs an LLM-free autonomous mapping sweep before normal navigation:

```bash
python3 main_os.py --map
```

Phases:
1. **Perimeter Trace** — left-hand wall follow to find room boundaries
2. **Interior Sweep** — boustrophedon (lawnmower) pattern to fill in the map
3. **RSSI Heatmap** — records WiFi signal strength at each grid position
4. **Export** — writes `apartment_map.py` (loaded automatically on next run)

### Floor Plan Import (LiDAR PNG)

Convert an iPhone LiDAR scan or any floor plan image directly to the robot's grid:

```bash
python3 dimos_lite/importer.py floorplan.png
```

- Produces a 21×21 `W`/` ` grid in `apartment_map.py`
- Applies a safety perimeter and clears a 3×3 center start zone
- Re-run whenever the floor plan changes

### RSSI Zone Calibration

Sample signal strength in each room for localization tuning:

```bash
python3 -m dimos_lite.localization
```

Update `ZONE_MAP` in `dimos_lite/localization.py` with the measured ranges.

### Generate ArUco Markers

Print and mount 15cm markers in each room for precise localization snapping:

```bash
python3 generate_markers.py
# Outputs PNG files to markers/
```

Mount markers at ~15cm height on a wall visible from the robot's camera.

---

## WebSocket Protocol

The Pico sends/receives JSON at `ws://192.168.1.217:8765`.

### Commands (Mac → Pico)

```json
{"K": "forward",  "A": 30}
{"K": "backward", "A": 30}
{"K": "left",     "A": 30}
{"K": "right",    "A": 30}
{"K": "stop",     "A": 0}
```

- `K` — direction: `forward`, `backward`, `left`, `right`, `stop`
- `A` — power: 0–100 (PWM duty %). Autonomous navigation uses **30**.
- Commands must arrive within `CMD_TIMEOUT_MS` (2000ms) or the Pico auto-stops.

### Telemetry (Pico → Mac)

```json
{
  "B": 12.5,
  "C": 3.45,
  "D": [[-75, 45.2], [-70, 43.1], ..., [85, 120.0], [90, 88.3]],
  "H": [45000, 52000, 48000],
  "I": [142.3, -0.8, 0.12, -0.05, 0.98]
}
```

| Key | Type | Description |
|-----|------|-------------|
| `B` | float | Current speed (cm/s) |
| `C` | float | Cumulative mileage (cm) |
| `D` | list | Radar sweep: `[[angle, distance_cm], ...]` |
| `H` | list | Grayscale ADC values: `[left, center, right]` (0–65535) |
| `I` | list | IMU telemetry: `[heading_deg, gyro_z, accel_x, accel_y, accel_z]` |

### Radar Sweep Details

The servo sweeps asymmetrically from **-75°** to **+90°** in **5° steps** at a **10ms tick rate**.
Sonar reads every 2nd step (effective 10° resolution, ~165 readings per full sweep).
The sweep list is reset at the start of each new pass (angle wraps back to -75°).

---

## Pico Firmware (examples/app_control.py — v4.1)

### Key Constants

```python
SERVO_OFFSET  = 0       # Physical mount offset (degrees)
MIN_ANGLE     = -75     # Right-most sweep limit
MAX_ANGLE     = 90      # Left-most sweep limit
ANGLE_STEP    = 5       # Degrees per tick
TICK_MS       = 10      # Main loop period (ms) → ~100Hz
READ_EVERY_N  = 2       # Sonar reads every N ticks
CMD_TIMEOUT_MS = 2000   # Auto-stop if no command received (ms)
```

### IMU Integration

On boot, the firmware attempts to initialize the MPU6050 at I2C address `0x68` (or `0x69`):

```python
# Automatic at boot — no user action needed
imu = MPU6050(I2C(1, sda=Pin(2), scl=Pin(3), freq=400000))
imu.calibrate()   # Hold robot still for ~3 seconds
```

IMU telemetry is sent every tick as key `I` in the WebSocket JSON.

### Flashing Firmware

```bash
# Deploy firmware to Pico (Pico must be USB-connected)
mpremote fs cp examples/app_control.py :main.py && mpremote reset

# Verify IMU detected
mpremote run examples/app_control.py
# Should print: "MPU6050 at 0x68 — calibrating (hold still)..."
```

---

## Mac-Side Python Modules

### core.py — Stream Architecture

All modules communicate via typed named streams:

```python
from dimos_lite.core import Module, StreamIn, StreamOut, autoconnect

class MyModule(Module):
    def __init__(self):
        super().__init__("MyModule")
        self.output = StreamOut("my_data")
        self.input  = StreamIn("other_data")
        self.input.subscribe(self._on_data)

    def _on_data(self, data):
        self.output.publish(processed)

# Wire all matching stream names automatically
autoconnect(vision, hardware, brain)
```

Active streams (6 total, auto-wired by `autoconnect`):

| Stream | Direction | Data Type |
|--------|-----------|-----------|
| `color_image` | Vision → AgentCore | OpenCV BGR frame |
| `ultrasonic_distance` | PicoHW → AgentCore | `[angle, dist_cm]` |
| `odometry` | PicoHW → AgentCore | `mileage_cm` float |
| `grayscale_line` | PicoHW → AgentCore | `[left, center, right]` |
| `imu_data` | PicoHW → AgentCore | `[heading, gyro_z, ax, ay, az]` |
| `cmd_vel` | AgentCore → PicoHW | `(direction, power)` tuple |

### agent.py — LLM Brain (AgentCoreModule)

The core decision loop runs two background threads plus the main control loop:

**Think Thread** (async): 
1. Encodes camera frame at **896×896** (optimal for `gemma4:31b`)
2. Builds a structured prompt with sensor data, map state, zone, and action history
3. Calls Ollama at `http://digitalstorm:11434` with a **60s timeout**
4. Validates and posts the resulting action plan

**Reflex Thread** (20Hz safety):
- Polls `_forward_dist` continuously
- If robot is moving forward and dist < 30cm → immediate emergency stop
- Operates independently of LLM latency

**Control Loop** (main thread):
1. Waits for a new plan from the think thread (100ms timeout)
2. Checks for manual override from the dashboard
3. Re-validates collision guard with fresh sensor data
4. Scales step duration by proximity (`closer → shorter step`)
5. Executes movement and syncs dashboard

```python
brain = AgentCoreModule(
    ollama_url="http://digitalstorm:11434/api/generate",
    model="gemma4:31b",
    prior_map=APARTMENT_PRIOR,   # from apartment_map.py
    localization=loc,
)
brain.start()  # blocks
```

**Safety stack (highest → lowest priority):**

| Layer | Trigger | Action |
|-------|---------|--------|
| Reflex Thread (20Hz) | dist < 30cm while moving forward | Immediate motor stop |
| Control Loop Guard | dist < 30cm at plan execution | Override action → stop |
| Proximity Scale | dist < 100cm | Reduce forward step duration |
| Oscillation Detector | 8 consecutive turns/backward | Escape maneuver |

**LLM prompt includes:**
- Forward distance and stale-sensor flag
- Adjacent grid cells (N/S/E/W wall/open status)
- Zone label (ArUco → RSSI → "Unknown")
- Heading from IMU (or dead-reckoning fallback)
- Last 8 actions
- Warning: *"BEWARE: Identify thin objects like chair legs or cables visually. They are invisible to radar."*

### pico.py — Hardware Bridge

Translates the stream architecture to the Pico's JSON WebSocket protocol. Auto-reconnects on disconnect (2s retry).

```python
hw = PicoHardwareModule(ws_url="ws://192.168.1.217:8765")
hw.start()  # background thread

# Publish via stream
hw.cmd_vel → accepts (direction, power) tuples

# Subscribed streams (published to AgentCore)
hw.ultrasonic_distance  # [angle, dist_cm]
hw.grayscale_line       # [left, center, right]
hw.odometry             # mileage_cm float
hw.imu_data             # [heading, gyro_z, ax, ay, az]
```

### vision.py — Camera Feed

Connects to ESP32-CAM MJPEG stream; publishes OpenCV frames ~10 FPS. Auto-reconnects on stream drop.

### localization.py — Fused Localization

Combines two sources of room-level location:

1. **ArUco Markers** (primary): When camera sees a DICT_4X4_50 marker, snaps robot position to that room's center. Stays valid for 30 seconds.
2. **RSSI zones** (fallback): Polls macOS `airport` utility every 3s and maps signal strength to room name.

```python
loc = LocalizationModule()
loc.start()

loc.get_zone()      # "Living Room", "Kitchen", "Bedroom", etc.
loc.get_position()  # (x, y) in apartment coordinates
loc.get_rssi()      # -52 (dBm)
```

Zone map (calibrate for your apartment):
```python
# dimos_lite/localization.py
ZONE_MAP = {
    "Living Room":  (-55,   0),   # strong signal near router
    "Kitchen":      (-70, -56),
    "Bedroom":      (-90, -71),   # weak signal, far from router
}
```

### aruco.py — Marker Detection

Uses OpenCV's DICT_4X4_50 dictionary. Detects marker IDs and maps them to rooms via `floorplan.py`:

| Marker ID | Room |
|-----------|------|
| 0 | Living Room |
| 1 | Kitchen |
| 2 | Bedroom |
| 3 | Bathroom |
| 4 | Entry |

Print markers with `python3 generate_markers.py`. Mount at ~15cm height on a wall.

### floorplan.py — Apartment Layout

Defines the programmatic floor plan in abstract coordinates (700×700 units ≈ apartment scale):

```
Rooms: Kitchen (9'6" × 10'), Bedroom (12'1" × 13'10"),
       Living Room (13'5" × 15'2"), Bath, Hall, Entry
```

Used by the dashboard floor plan renderer and localization module.

### dashboard.py — Web Control Deck

Serves a full-featured dark-mode dashboard on port 8080 with no external dependencies.

**Panels:**
- **Live Vision** — MJPEG stream from ESP32-CAM
- **Brain State** — current action, observation, reasoning, distance bar, heading (with IMU badge), speed/mileage, grayscale bars
- **Radar Sweep** — animated canvas polar plot (red dots = danger zone <30cm)
- **Floor Plan** — apartment layout with robot position, heading arrow, 30s movement trail, active-room highlight
- **Action Log** — color-coded history (FORWARD=green, STOP=red, TURN=purple)

**Manual Override Controls:**
- Keyboard: `W/A/S/D` or Arrow keys for movement, `Space` = stop
- On-screen buttons: Forward / Left / STOP / Right / Backward
- Manual commands execute at **50% power** for 0.5s (vs autonomous 30%)

**API endpoints:**
- `GET /` — Dashboard HTML
- `GET /video_feed` — MJPEG stream
- `GET /api/state` — JSON telemetry snapshot
- `POST /api/control` — `{"command": "forward"}` manual override

### mapper.py — Autonomous Mapping Mode

LLM-free exploration routine (`--map` flag):
- **Phase 1**: Left-hand wall-following perimeter trace
- **Phase 2**: Boustrophedon (lawnmower) interior sweep
- **Phase 3**: RSSI heatmap at each visited position
- **Phase 4**: Export to `apartment_map.py`

### importer.py — LiDAR Floor Plan Converter

Converts any floor plan PNG/JPG to the robot's 21×21 occupancy grid:

```bash
python3 dimos_lite/importer.py floorplan.png
```

Processing pipeline:
1. Load image → grayscale
2. Threshold at 200 (pixels <200 = wall)
3. Resize to 21×21 with `INTER_AREA` (preserves thin walls)
4. Strict wall threshold: <150 = `W`, else ` `
5. Force 1-cell `W` perimeter around entire grid
6. Clear 3×3 center area for robot spawn point
7. Write to `apartment_map.py`

### Semantic Map (21×21 Grid)

| Property | Value |
|----------|-------|
| Grid size | 21 × 21 cells |
| Cell size | 10cm |
| Total coverage | 2.1m × 2.1m |
| Cell values | `' '` open, `'W'` obstacle, `'R'` robot (render only) |
| Heading source | IMU (primary) or time-based dead reckoning (fallback) |
| Obstacle writes | **Only when robot is stopped** (prevents motion blur) |
| Thread safety | All grid methods are mutex-locked |

---

## ESP32-CAM Control

### Camera Settings (via HTTP)

```bash
# Resolution (10=UXGA 1600x1200, 8=XGA, 5=VGA, 4=CIF)
curl "http://192.168.1.216/control?var=framesize&val=5"

# Quality (10-63, lower = better quality)
curl "http://192.168.1.216/control?var=quality&val=12"

# Brightness (-2 to +2)
curl "http://192.168.1.216/control?var=brightness&val=1"

# Flash LED (0-255)
curl "http://192.168.1.216/control?var=led_intensity&val=100"

# Flip / Mirror
curl "http://192.168.1.216/control?var=vflip&val=1"
curl "http://192.168.1.216/control?var=hmirror&val=1"
```

### Capture Endpoints

```bash
# Single JPEG snapshot
curl "http://192.168.1.216/capture" -o photo.jpg

# MJPEG stream (used by VisionModule)
# http://192.168.1.216:81/stream
```

---

## Known Limitations

1. **Ultrasonic misses thin objects** — beam is ~15° wide; chair legs and cables are invisible to radar. The 31B model at 896×896 is instructed to compensate visually.
2. **Map is small** — 21×21 at 10cm = 2.1m × 2.1m coverage. Large apartment requires position tracking across re-localizations.
3. **ESP32-CAM drops** — unstable at `192.168.1.216`; power cycle fixes it. LED flash on stream connect is normal.
4. **LLM latency** — `gemma4:31b` on a 3090 runs ~3–6s per inference. Reflex thread handles safety during thinking gaps.
5. **IMU heading drift** — MPU6050 gyro drifts slowly (~1–3°/min). ArUco markers correct accumulated drift when seen.

## Pending Hardware Upgrades

| Part | Purpose | Interface |
|------|---------|-----------| 
| 2× VL53L0X | Front L/R ToF laser (detects thin obstacles) | I2C + 2 GPIO (XSHUT) |
| 1× QMC5883L | Magnetometer (absolute compass) | I2C |

---

## File Map

```
pico_4wd_car-main/
├── main_os.py                  # Entry point (normal + --map modes)
├── apartment_map.py            # Floor plan grid (generated or manual)
├── floorplan.png               # Source image for map import
├── generate_markers.py         # Generates printable ArUco PNGs → markers/
├── markers/                    # Printed ArUco marker images
├── dimos_lite/
│   ├── core.py                 # Stream architecture (StreamIn/Out, autoconnect)
│   ├── agent.py                # LLM brain, reflex safety, semantic map
│   ├── vision.py               # ESP32-CAM MJPEG consumer
│   ├── pico.py                 # WebSocket hardware bridge (auto-reconnect)
│   ├── dashboard.py            # Web UI on :8080 (radar, floor plan, controls)
│   ├── localization.py         # RSSI + ArUco fused localization
│   ├── aruco.py                # ArUco marker generation and detection
│   ├── floorplan.py            # Apartment room definitions + marker mapping
│   ├── mapper.py               # Autonomous mapping mode (--map)
│   └── importer.py             # LiDAR PNG → apartment_map.py converter
├── examples/
│   └── app_control.py          # Pico firmware v4.1 (deploy as main.py)
├── libs/                       # MicroPython libraries (on Pico)
│   ├── pico_4wd.py             # Hardware driver (motors, LEDs, sensors)
│   ├── pico_rdp.py             # Motor, Servo, Ultrasonic, WS2812, Speed classes
│   ├── mpu6050.py              # MPU6050 IMU driver
│   └── ws.py                   # ESP8266 UART WebSocket server
├── debug/
│   ├── latest_frame.jpg        # Last camera frame (auto-refresh in Preview)
│   └── latency.json            # Rolling LLM inference latency stats
└── docs/
    └── USER_GUIDE.md           # This file (v4.1)
```
