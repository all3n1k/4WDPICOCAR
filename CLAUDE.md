# CLAUDE.md — Agent OS Project Status

**Read this first on cold start.** This is the living snapshot. For deep technical detail see `docs/USER_GUIDE.md`. For change history see `CHANGELOG.md`.

Keep this file current — update it in the same commit as any behavioural change.

---

## TL;DR

A four-wheel robot car (Raspberry Pi Pico + ESP8266 + ESP32-CAM) driven by a Python brain on a Mac, with an LLM (gemma4:31b on `digitalstorm:11434`) doing the reasoning. Localisation fuses ArUco markers + WiFi RSSI. The dashboard (port 8080) shows live video, radar, floor plan, and SLAM obstacles.

**Current focus: making it smart.** Hardware, comms, control, stream latency are all working. The remaining agenda is the reasoning layer — better prompts, better action selection, better memory, smarter exploration.

## What works

- Pico motor control, servo radar sweep, HC-SR04 sonar, 3× grayscale, wheel encoders (odometry)
- MPU6050 IMU on I²C1 (GP2/GP3) — absolute heading replaces dead-reckoning
- WebSocket bridge Pico↔Mac at `ws://192.168.1.217:8765`, auto-reconnect, 2s command-timeout auto-stop on the Pico side
- ESP32-CAM MJPEG stream at `http://192.168.1.216:81/stream`, low-latency grab/decode thread, SVGA (800×600)
- ArUco DICT_4X4_50 room snapping (30s validity window, fused with RSSI fallback)
- Dashboard: live video, radar polar plot, 700×700 floor plan with robot pose, trail, SLAM obstacle dots, action log, manual controls (WASD / buttons)
- LLM brain: think thread, reflex safety thread, control loop, oscillation detector, tool-calling JSON protocol (`move`/`turn`/`look`/`scan`/`speak`/`set_mood`)
- Mapping mode (`python3 main_os.py --map`): perimeter trace → boustrophedon sweep → RSSI heatmap → export
- LiDAR floor-plan import (`python3 dimos_lite/importer.py floorplan.png`)
- Chassis LED strip on GP19 (24× WS2812, 8 rear + 8 bottom-L + 8 bottom-R). `L_BOT` / `L_REAR` set steady colours; firmware auto-blinks the bottom-left segment amber while turning left (fast ~3 Hz) and the bottom-right segment while turning right (slow ~1.5 Hz), restoring the cached `L_BOT` colour when the turn ends.

## What doesn't work / known issues

- **No battery ADC.** `read_battery_v()` is stubbed to return `0.0` — the dashboard shows `N/A`. Adding a real voltage divider on a free GPIO is a pending hardware upgrade.
- **gemma4:31b latency is 3–6 s on a 3090.** Reflex thread covers the gap; no known regression.
- **21×21 semantic grid is small** (2.1 m × 2.1 m). Apartment-scale coverage depends on ArUco re-localisation resets. Position override from the dashboard recenters the grid — that's intentional.

## How to run

```bash
cd /Users/neo/Downloads/pico_4wd_car-main

# Normal mode
python3 main_os.py
open http://localhost:8080

# Mapping mode (LLM-free exploration, generates apartment_map.py)
python3 main_os.py --map

# Deploy Pico firmware (USB-connected)
mpremote fs cp examples/app_control.py :main.py && mpremote reset
```

## Architecture at a glance

Three layers, each isolated by a single network protocol:

| Layer | Hardware | Protocol to next layer |
|-------|----------|------------------------|
| Brain | Mac / Python 3.14 | WebSocket JSON + MJPEG |
| Comms | ESP8266 on Pico (UART1) + ESP32-CAM (WiFi) | UART AT commands / HTTP |
| Actuators | RP2040 Pico, motors/servo/sensors/IMU | GPIO / I²C |

Inside the Python brain, modules communicate through typed named streams (`dimos_lite/core.py`):

```
VisionModule   ──color_image──▶ AgentCoreModule
PicoHardware   ──ultrasonic/odometry/grayscale/imu/battery──▶ AgentCoreModule
AgentCoreModule ──cmd_vel──▶ PicoHardwareModule
LocalizationModule (independent, polls airport utility + gets ArUco updates)
```

`autoconnect(*modules)` wires all matching `StreamOut` → `StreamIn` pairs by name.

## File map

```
pico_4wd_car-main/
├── CLAUDE.md                  # This file — living status snapshot
├── CHANGELOG.md               # What changed, when, why
├── README.md                  # User-facing overview
├── main_os.py                 # Entry point (normal + --map)
├── apartment_map.py           # 21×21 prior map (generated)
├── floorplan.png              # Source image for map import
├── generate_markers.py        # Prints ArUco PNGs → markers/
├── markers/                   # Printable ArUco marker images
├── dimos_lite/                # Mac-side Python modules
│   ├── core.py                #   Stream architecture
│   ├── agent.py               #   LLM brain + reflex + semantic map
│   ├── vision.py              #   ESP32-CAM consumer
│   ├── pico.py                #   WS hardware bridge
│   ├── dashboard.py           #   :8080 web UI
│   ├── localization.py        #   RSSI + ArUco fusion
│   ├── aruco.py               #   Marker detect/generate
│   ├── floorplan.py           #   Apartment room definitions
│   ├── mapper.py              #   Autonomous mapping mode
│   └── importer.py            #   LiDAR PNG → apartment_map.py
├── examples/
│   └── app_control.py         # Pico firmware (flash as main.py)
├── libs/                      # MicroPython libs (on Pico)
│   ├── pico_4wd.py            #   Car driver (motors/LEDs/sensors)
│   ├── pico_rdp.py            #   Motor/Servo/Sonar/WS2812/Speed
│   ├── mpu6050.py             #   IMU driver
│   └── ws.py                  #   ESP8266 UART WS server
├── debug/                     # Runtime artifacts (gitignored)
└── docs/
    └── USER_GUIDE.md          # Deep technical reference
```

## Design conventions / gotchas

- **Coordinate system.** Two of them. (1) Apartment frame, 0–700 px, origin top-left, +y down, used by `dimos_lite/floorplan.py` and the dashboard floor plan. (2) Semantic grid, 21×21 cells × 10 cm, robot at centre, used by `SemanticMap`. Never mutate semantic_map cells from apartment pixels.
- **Heading convention.** 0° = north, +y = south. `update_position(delta, heading)` does `x += delta·sin(H); y -= delta·cos(H)`. `update_aruco` inverts this (see `localization.py`).
- **Translation only from odometry.** The agent loop does *not* dead-reckon translation — encoder deltas flow through `_on_odometry`. Turns are dead-reckoned only when the IMU is absent.
- **Reflex thread overrides everything.** 25 Hz poll, stops on (forward && dist<20cm) emergency, forward/turn with dist<45cm, or wide-cone <25cm. Manual dashboard input does *not* relax these; space key is the user's kill switch.
- **Pico auto-stop.** If no `{K,A}` packet arrives for 2000 ms, motors cut. Manual dashboard press repeats every 80 ms.
- **Obstacle writes only when stopped.** `_on_distance` only calls `add_obstacle` when `_current_action() == "stop"`, to avoid motion-blur smearing the map.
- **Command payloads can be either a 2-tuple `(direction, speed)` or a dict.** Dicts pass through verbatim (used for LED/servo control, keys `L`, `L_BOT`, `L_REAR`).

## Pico GPIO map (cross-check before wiring changes)

| GP | Use |
|----|-----|
| 10–17 | 4× motor direction/PWM |
| 18 | Servo (radar) |
| 2/3 | I²C1 for MPU6050 |
| 4/5 | UART1 to ESP8266 |
| 6/7 | HC-SR04 trig/echo |
| 8/9 | Wheel encoders |
| 19 | WS2812 chassis LEDs (24× — 8 rear + 8 bottom-L + 8 bottom-R), PIO state machine 0 |
| 25 | Onboard LED |
| 26/27/28 | Grayscale L/C/R (ADC0–2) |
| 29 | ADC3 — reads VSYS, not the battery |
| 0, 1, 20, 21, 22 | Free |

## Network

| Device | Address | Notes |
|--------|---------|-------|
| Pico WebSocket | `ws://192.168.1.217:8765` | Static; power-cycle if unresponsive |
| ESP32-CAM stream | `http://192.168.1.216:81/stream` | Flaky under <4.8 V |
| ESP32-CAM control | `http://192.168.1.216/control?var=X&val=Y` | Resolution/exposure tweaks |
| Dashboard | `http://localhost:8080` | Served by `dashboard.py` |
| Ollama | `http://digitalstorm:11434/api/generate` | 90 s timeout, 512 tokens |

## Pointers for the LLM brain agenda

Where the reasoning lives right now:

- `dimos_lite/agent.py:_build_prompt` — the system prompt. Currently a single string with sensor snapshot + neighbours + memory + tool list. Main knobs to tune.
- `dimos_lite/agent.py:TOOLS` — the action vocabulary.
- `dimos_lite/agent.py:_execute_tool` — what each tool actually does on the hardware.
- `dimos_lite/agent.py:_think_loop` — the 0.5 s minimum-interval plan loop; calls Ollama, parses JSON, dispatches tools.
- `_memory` (deque, maxlen=5) — short-term thought buffer fed back into the prompt.
- `_action_history` (deque, maxlen=12) — oscillation detector input (not currently read).

Obvious smart-upgrade directions: richer spatial memory (not just recent thoughts), landmark-based goals, better "why did I stop / what next" reasoning after reflex stops, selective `scan` usage, ESP32-CAM exposure/quality tuning mid-run.

---

_Last snapshot: 2026-04-23 (v5.5.2). Bump the date on substantive edits._
