# CLAUDE.md — Agent OS Project Status

**Read this first on cold start.** This is the living snapshot. For deep technical detail see `docs/USER_GUIDE.md`. For change history see `CHANGELOG.md`.

Keep this file current — update it in the same commit as any behavioural change.

---

## TL;DR

A four-wheel robot car (Raspberry Pi Pico + ESP8266 + ESP32-CAM) driven by a Python brain on a Mac, with an agentic 31B multimodal LLM (`gemma4:31b` on `digitalstorm:11434`) doing the reasoning and authorised to rewrite its own source via `patch_self` / `reboot`. Vision runs YOLOv8-Nano on Apple MPS. Localisation fuses ArUco markers (DICT_4X4_50) + WiFi RSSI. The dashboard (port 8080) shows live video with detection boxes, radar, floor plan with constellation + obstacle cloud, action log, and manual controls.

**Current focus: localisation + spatial memory.** Mapping pipeline was end-to-end broken as of v5.5.2; repaired in v5.6.0. Cartographer pre-pass now feeds the dashboard mini-map live, persists the constellation + obstacle grid to disk, and normal mode loads both on boot. Remaining agenda: polish the cartographer calibration UX into the dashboard, tune self-modification loops, improve LLM spatial reasoning.

## What works

- Pico motor control, servo radar sweep, HC-04 sonar, 3× grayscale, wheel encoders (odometry)
- **Rear LEDs**: [VERIFIED WORKING] - Confirmed functional during diagnostic tests; previously suspected wiring issue. `main_os.py` software likely suppresses them.
- **MPU6050 IMU**: [OPERATIONAL] - Wired to I²C0 (GP0/GP1) at 0x68. Driven via SoftI²C on the firmware (HW I²C peripheral on MicroPython 1.28.0 EIOs on every clock-stretched register read; SoftI²C handles stretching cleanly, costs ~1ms/tick). Calibrates on boot.
- **VL53L0X TOF Laser**: [OPERATIONAL] - Wired to I²C1 (GP2/GP3) at 0x29. Driven via SoftI²C with the canonical Pololu-port driver (full ST init: SPAD calibration, Vhv/phase calibration, signal-rate limit). Continuous-mode ranging — `read()` polls latest cached value in ~8 ms. Reads ~5 cm at a 5.08 cm (2 in) ground-truth target, within VL53L0X uncalibrated spec.
- Pan/tilt head servos on GP20 (pan) / GP21 (tilt). Direct passthrough path: dashboard `/api/servo` HTTP handler calls `state.pico_hw._on_cmd_vel(("S20", angle))` synchronously — the agent's manual-cmd queue is **bypassed entirely** for servo commands. JS keeps a `panTarget`/`tiltTarget` and a one-in-flight-per-pin sender (`flushServo` in `dashboard.py`), so even rapid arrow-key spam never queues more than one outstanding fetch. **Critical headroom rule**: the Pico's real interp rate must exceed JS intent rate, otherwise the target creeps ahead of `pan_current` and the gap visibly drains on key-release as "slow continued motion." Current numbers: Pico interp = 5°/tick ≈ 500°/s typical / ~250°/s on sensor-read ticks. JS intent = 5°/40ms = 125°/s. That's 2-4× headroom; max in-flight gap at release drains in <10ms, imperceptible. SG90 mechanical max is ~600°/s so 500°/s stays in spec. Deadband 2° (`SERVO_DEADBAND` in `app_control.py`) — JS sends clean 5° increments so every command passes the deadband. `set_angle` is only re-issued when the interpolated angle actually changes — writing the same PWM 200×/s caused observable micro-twitch on the SG90s.
- **Shift = boost** (dashboard manual control). Holding Shift while pressing W/A/S/D suffixes the command with `_boost`; the agent strips the suffix and dispatches at `BOOST_SPEED=100` (motor 100% duty) instead of `MANUAL_SPEED=50` (motor 60% duty). Note the Pico's motor mapping is `duty% = 20 + 0.8 × input` (`libs/pico_rdp.py:Motor.power.setter`) — the 20% floor overcomes static friction. Cruise is intentionally below max so boost has real headroom (60% → 100% = 40-point duty jump). `TURN_SPEED=60` (motor 68%) — turns get slightly more torque than rolling cruise. Shift state is tracked separately so it can be toggled mid-press: pressing W → adding Shift → releasing Shift seamlessly transitions because `startManual` re-sends the active direction every 80 ms. Bottom LED tint switches to magenta when boosting (cyan = normal manual). Browser-tab-blur handler clears all held keys + sends `stop` so the robot doesn't run away if focus is lost mid-press.
- WebSocket bridge Pico↔Mac at `ws://192.168.1.217:8765`, auto-reconnect, 2 s command-timeout auto-stop on the Pico side
- ESP32-CAM MJPEG stream at `http://192.168.1.210/stream` (port 80) — manual M-JPEG reader prevents WiFi-drop crashes.
- ESP32-CAM LED Control: Binary On/Off confirmed working. Implemented 'Silence & Strike' mechanism (0.25s stream cut) to bypass ESP32-CAM hardware single-threading limits.
- Vision Orientation: Set to Flip -1 (Right-side up). Corrected 'label' KeyError that was crashing the agent's think loop.
- YOLOv8-Nano object detection on MPS, 80 COCO classes, bounding boxes published on the `detections` stream and overlaid on the dashboard video feed
- ArUco DICT_4X4_50: room markers 0–4 (`dimos_lite/floorplan.py:MARKER_TO_ROOM`), floor/constellation markers 10–30 (discovered during `--carto`)
- Cartographer pre-pass (`python3 main_os.py --carto`): manual-drive + ArUco discovery + sonar obstacle logging; saves `constellation_map.json` + `semantic_map.json`; normal mode reads both on boot
- Dashboard: live video with YOLO + ArUco HUD, radar polar plot, 700×700 floor plan with robot pose / trail / discovered markers / obstacle cloud, action log, WASD + arrow-key pan/tilt, click-to-place, shift-click to pin a marker
- LLM brain: think thread, reflex safety thread, control loop, tool-calling JSON (`move`/`turn`/`look`/`scan`/`speak`/`set_mood`/`tune_parameters`/`patch_self`/`reboot`)
- Autonomous mapping mode (`python3 main_os.py --map`): perimeter trace → boustrophedon sweep → RSSI heatmap → export to `apartment_map.py`
- LiDAR floor-plan import (`python3 dimos_lite/importer.py floorplan.png`)
- Chassis LED strip on GP19 (8 rear + 8 bottom-L + 8 bottom-R). Note: Rear LEDs have a physical wiring issue and are currently non-functional; bottom LEDs work fine (auto-blinks amber during turns, fast ~3 Hz left, slow ~1.5 Hz right).

## What doesn't work / known issues

- **ESP32-CAM brightness control regressed.** A different ESP32-CAM was swapped in and the new firmware variant only exposes binary on/off LED control — extensive `?var=led_intensity` URL probing did not unlock brightness. Either reflash the original CameraWebServer firmware or live with binary LED. Deferred until everything else is done.
- **Object Identification Errors.** YOLOv8-Nano occasionally 'smears' or falsely identifies objects as being close when the path is clear.
- **Perception Gap.** The LLM currently does not know how to autonomously use the pan/tilt servos to 'look around' before moving.

## How to run

```bash
cd /Users/neo/Downloads/pico_4wd_car-main
source venv/bin/activate   # Python 3.14 + torch + ultralytics

# SLAM pre-pass — manually drive around, discover markers, log obstacles
python3 main_os.py --carto
# → opens the dashboard mini-map (http://localhost:8080) and a cv2 calibration window.
#   On exit, writes constellation_map.json + semantic_map.json.

# Normal mode — LLM brain, loads the persisted constellation + obstacle grid
python3 main_os.py
open http://localhost:8080

# Autonomous mapping mode (LLM-free exploration, generates apartment_map.py)
python3 main_os.py --map

# Deploy Pico firmware AND its libs (USB-connected).
# IMPORTANT: copy the libs every time alongside main.py — `app_control.py` depends
# on symbols defined in `libs/pico_4wd.py` (e.g. `servo20`, `servo21`), and a stale
# lib silently breaks the loop body via AttributeError → all servos go dead, sonar
# stops reporting, sweep stops advancing. The 2026-04-24 incident ate hours of
# debugging because only `main.py` was being re-flashed.
mpremote fs cp libs/pico_4wd.py :pico_4wd.py \
  && mpremote fs cp libs/pico_rdp.py :pico_rdp.py \
  && mpremote fs cp libs/mpu6050.py :mpu6050.py \
  && mpremote fs cp libs/ws.py :ws.py \
  && mpremote fs cp examples/app_control.py :main.py \
  && mpremote reset
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
├── CLAUDE.md                         # This file — living status snapshot
├── CHANGELOG.md                      # What changed, when, why
├── README.md                         # User-facing overview
├── main_os.py                        # Entry point (normal / --carto / --map)
├── apartment_map.py                  # 71×71 prior map (generated)
├── floorplan.png                     # Source image for map import
├── generate_markers.py               # Prints room ArUco PNGs → markers/
├── generate_constellation.py         # Prints floor/constellation ArUco PNGs
├── generate_constellation_sheets.py  # Layouts markers for printing on sheets
├── markers/                          # Printable ArUco marker images
├── constellation_map.json            # Discovered marker positions (runtime)
├── semantic_map.json                 # Persisted obstacle grid (runtime)
├── yolov8n.pt                        # YOLOv8-Nano weights (auto-downloaded)
├── dimos_lite/                       # Mac-side Python modules
│   ├── core.py                       #   Stream architecture
│   ├── agent.py                      #   LLM brain + reflex + semantic map
│   ├── vision.py                     #   ESP32-CAM consumer + YOLO
│   ├── pico.py                       #   WS hardware bridge
│   ├── dashboard.py                  #   :8080 web UI
│   ├── localization.py               #   RSSI + ArUco fusion + constellation persistence
│   ├── aruco.py                      #   Marker detect/generate
│   ├── floorplan.py                  #   Apartment room definitions + room markers
│   ├── cartographer.py               #   --carto SLAM pre-pass
│   ├── mapper.py                     #   --map autonomous mapping mode
│   └── importer.py                   #   LiDAR PNG → apartment_map.py
├── examples/
│   └── app_control.py                # Pico firmware (flash as main.py)
├── libs/                             # MicroPython libs (on Pico)
│   ├── pico_4wd.py                   #   Car driver (motors/LEDs/sensors)
│   ├── pico_rdp.py                   #   Motor/Servo/Sonar/WS2812/Speed
│   ├── mpu6050.py                    #   IMU driver (publishes [heading, gz, ax, ay, az])
│   └── ws.py                         #   ESP8266 UART WS server
├── venv/                             # Python 3.14 + torch + ultralytics (gitignored)
├── debug/                            # Runtime artifacts (gitignored)
└── docs/
    └── USER_GUIDE.md                 # Deep technical reference
```

## Design conventions / gotchas

- **Coordinate system.** Two of them. (1) Apartment frame, 0–700 px, origin top-left, +y down, used by `dimos_lite/floorplan.py` and the dashboard floor plan. (2) Semantic grid, 71×71 cells × 10 cm, robot defaulting to the centre cell, used by `SemanticMap`. Never mutate semantic_map cells from apartment pixels.
- **Heading convention.** 0° = north, +y = south. `update_position(delta, heading)` does `x += delta·sin(H); y -= delta·cos(H)`. `update_aruco` inverts this (see `localization.py`).
- **Translation only from odometry.** The agent loop does *not* dead-reckon translation — encoder deltas flow through `_on_odometry`. Turns are dead-reckoned only when the IMU is absent.
- **IMU packet format.** `libs/mpu6050.py:get_telemetry` returns a **list**: `[heading, gyro_z, ax, ay, az]`. Consumers must index by position (`data[0]`), never by key. Cartographer v5.5.2 got this wrong and was silently running with a stuck heading — see `CHANGELOG.md` v5.6.0.
- **Reflex thread overrides everything.** 25 Hz poll, stops on (forward && dist<20cm) emergency, forward/turn with dist<45cm, or wide-cone <25cm. Manual dashboard input does *not* relax these; space key is the user's kill switch.
- **Pico auto-stop.** If no `{K,A}` packet arrives for 2000 ms, motors cut. Manual dashboard press repeats every 80 ms.
- **Obstacle writes only when stopped.** `_on_distance` only calls `add_obstacle` when `_current_action() == "stop"`, to avoid motion-blur smearing the map.
- **Command payloads can be either a 2-tuple `(direction, speed)` or a dict.** Dicts pass through verbatim. Keys include `L`, `L_BOT`, `L_REAR` (LEDs), `S20` / `S21` (pan/tilt servos), `radar` (sweep config), and `pan` / `tilt` (shortcuts that get translated to `S20` / `S21` in `pico.py`).
- **Self-modifying agent.** `patch_self`, `tune_parameters`, and `reboot` are intentional tools in the agent's vocabulary. The 31B model is expected to rewrite its own `agent.py` / `floorplan.py` and `os.execv` itself to apply changes. This is a design choice, not a safety oversight.
- **Constellation vs. room markers.** Marker IDs 0–9 are wall-mounted room anchors (see `floorplan.py:MARKER_TO_ROOM`, each with a fixed `marker_pos`). Marker IDs 10–30 are the "constellation" — floor/mobile markers the cartographer discovers and persists to `constellation_map.json`. The distance-trust rule (`localization.py:update_aruco`) rejects sightings >150 cm for pose fusion but still uses them for constellation discovery.

## Pico GPIO map (cross-check before wiring changes)

| GP | Use |
|----|-----|
| 0/1 | I²C0 SDA/SCL → MPU6050 IMU (SoftI²C in firmware) |
| 2/3 | I²C1 SDA/SCL → VL53L0X TOF laser (SoftI²C in firmware) |
| 4/5 | UART1 to ESP8266 |
| 6/7 | HC-SR04 trig/echo |
| 8/9 | Wheel encoders |
| 10–17 | 4× motor direction/PWM |
| 18 | Servo (radar) |
| 19 | WS2812 chassis LEDs (24× — 8 rear + 8 bottom-L + 8 bottom-R), PIO state machine 0 |
| 20 | Pan servo (head, S20) |
| 21 | Tilt servo (head, S21) |
| 25 | Onboard LED |
| 26/27/28 | Grayscale L/C/R (ADC0–2) |
| 29 | ADC3 — reads VSYS, not the battery |
| 22 | Free |

## Network

| Device | Address | Notes |
|--------|---------|-------|
| Pico WebSocket | `ws://192.168.1.217:8765` | DHCP via `Pico4WD` hostname (`app_control.py:42`); IP only sticky because of DHCP lease, not actually static. Reserve in router for stability. |
| ESP32-CAM stream | `http://192.168.1.210/stream` | Port 80 (not the standard CameraWebServer 81 — this firmware variant uses one port). DHCP — IP changed from `.216` to `.210` on 2026-04-24. Hostname `ESP32-96B0FC`. Reserve in router. |
| ESP32-CAM control | `http://192.168.1.210/control?var=X&val=Y` | Resolution/exposure tweaks |
| Dashboard | `http://localhost:8080` | Served by `dashboard.py` |
| Ollama | `http://digitalstorm:11434/api/generate` | 90 s timeout, 512 tokens |

## Robot persona — the intended behaviour

Authoritative intent from the human owner (paraphrased from their own words — if you edit, edit carefully, this is the design spec for the reasoning layer):

> "I am a small 4-wheel robot with sensors and tool-calling. I know I'm in the centre of the living room, facing north. I have a programmatic map showing rooms and known obstacles. I can move forward / backward / left / right and look around. My human can take manual control at any time, and when they do I should not interfere — I wait 30 seconds of command silence before resuming autonomy. My general objective is to **explore**. If I see a cat I switch to playful behaviours to entertain it; when the cat leaves I go back to exploring. I summarise what I see to my human periodically. I can combine tool calls to do complex tasks. I can suggest source-code patches to fix my own issues."

Key commitments this implies (and which the current code does **not** yet honour — see the representation gap below):
- **Identity-first framing.** The LLM should be thinking in first person, not reading a sensor dashboard.
- **Spatial continuity.** "I saw a cat near the kitchen door 20 s ago" must survive across ticks.
- **Explicit task state.** `explore` vs `play_with_cat` vs `report` should persist across ticks, not get re-derived every tick.
- **Cooperative autonomy.** After human input, back off for 30 s, then narrate the resume.
- **Narrative reporting.** Periodic summaries of what was seen, to the human.

## Where the reasoning lives right now

- `dimos_lite/agent.py:_build_prompt` (line 669) — the system prompt. Sensor snapshot + 4 grid-cell neighbours + 5-line thought deque + current YOLO/ArUco hits + 4 rules + tool schema.
- `dimos_lite/agent.py:TOOLS` (line 122) — 8-tool vocabulary: `move`, `turn`, `look`, `scan`, `speak`, `set_mood`, `tune_parameters`, `patch_self`, `reboot`.
- `dimos_lite/agent.py:_execute_tool` (~line 500) — dispatch table for each tool.
- `dimos_lite/agent.py:_think_loop` (~line 820) — 0.5 s minimum-interval plan loop; calls Ollama, parses JSON, dispatches tools.
- `_memory` (deque, maxlen=5) — short-term thought buffer, strings only (no timestamps, no locations).
- `_last_detections` (overwritten each frame, `agent.py:404`) — YOLO hits, lost on the next tick.
- `_aruco_detections` (overwritten each frame, `agent.py:335`) — marker hits, lost on the next tick.
- `_localization._marker_positions` (in-memory dict, loaded from `constellation_map.json`) — known but **not surfaced to the LLM**.
- `semantic_map.grid` (71×71 × 10 cm, loaded from `semantic_map.json`) — known but **only 4 neighbour cells surfaced to the LLM**.

## The representation gap — why the robot feels dumb

The system tracks a rich world. The LLM sees a thin slice with no continuity. Concretely:

1. **Prompt is a dashboard, not a narrative.** Opens with "YOU ARE DIMOS" and one sentence of mission, then pivots to raw sensor numbers. No operational rules (human-cooldown, cat-play, summarise-periodically) are in the prompt — they live in the human's head.
2. **YOLO detections are ephemeral.** `_last_detections` gets overwritten every frame. No "I saw a cat at (420, 180) 12 s ago."
3. **30-second manual cooldown is not implemented.** Manual commands preempt the plan, but the agent resumes the next tick with no awareness that the human was just driving.
4. **Cat-mode is not wired.** COCO class 15 shows up in the prompt as `Objects: cat (78%)` and then disappears; no behavioural branch, no mood lock-in, no decay.
5. **Priors are loaded but invisible.** `constellation_map.json` and `semantic_map.json` are read into RAM, used for reflex avoidance and pose correction, but the prompt only shows the 4 immediate grid neighbours (N/S/E/W) and the marker IDs in the **current** frame. The LLM never sees "marker 17 is 1.8 m SE, that's the living-room south wall."
6. **No episodic memory.** `_memory` is a 5-line chat log of thought strings. No timestamps, no locations, no events.
7. **No persistent task state.** Every tick is an independent decision. Nothing to anchor multi-tick intent.

## Active workstream — mapping & navigation (v5.8.0)

**This is the priority.** The intelligence-layer roadmap below (Stages B–F) is parked; the robot needs to know where it is in the apartment before more reasoning gets layered on. Goal: animal-equivalent spatial competence — the robot can drive around for ten minutes, never wedge its position more than ~30 cm from physical truth, and produce an obstacle map that visually matches the apartment.

**Why mapping is broken today** (verified against `localization.py:149-204`, `agent.py:42-118`, `agent.py:938-972`):
1. **Two parallel pose estimators that don't talk.** `Localization._robot_x/y` (apartment pixels, fused with ArUco at α=0.2) and `SemanticMap.x/y` (grid cells, integrated odometry only — never corrected by ArUco). Dashboard dot reads from Localization, obstacles read from SemanticMap → they drift apart over time.
2. **Obstacle frame is anchored at apartment centre, not world origin.** `SemanticMap.get_obstacles()` (`agent.py:117-118`) renders cell `(c, r)` at apartment pixel `(350 + (c-35)·10, 350 + (r-35)·10)`. Grid origin = "wherever the robot booted, drawn at apartment centre." If the robot started in the kitchen, every sonar hit ends up around the living room. This is the "obstacle cloud spilling into hall and bedroom" symptom.
3. **Obstacle insertion uses SemanticMap's local pose** (`add_obstacle`, `agent.py:42-52`), not Localization's fused pose — so even if Localization knew the truth, the obstacle math wouldn't see it.
4. **Discovery from a wrong initial pose.** Robot boots at apartment centre by assumption. First ArUco sighting in discovery mode pins the marker at `robot_pos + d·(sin/cos)`, so markers get saved to `constellation_map.json` at wrong apartment coords. Every later sighting then pulls toward those bad pinned positions — visible as "robot teleports when seeing markers."
5. **Marker positions are not surveyed.** Boot log shows `Loaded 2 constellation markers`. The user has 27 ArUco markers physically placed in the apartment (room + floor). Without ground-truth marker positions, every fusion correction is pulling toward fiction.
6. **Sensor reality.** YouTube SLAM demos use 360° LiDAR (~1800 samples/s). This robot has one HC-SR04 ultrasonic on a sweeping servo — sparse beam, ~30° cone, slow returns. The right architecture for this stack is **fiducial-anchored sparse mapping**: ArUco pins the world frame, encoders + IMU dead-reckon between fixings, ultrasonic adds occasional point obstacles to a probabilistic grid. That's what we're building.

### Stages

| Stage | Item | Files / entry points | LOC | Depends on |
|-------|------|----------------------|-----|------------|
| M0 | **Marker survey tool.** Dashboard panel listing marker IDs 0–30. User selects an ID, clicks on the floor-plan canvas to pin its physical position, then sets the marker's facing direction (wall markers only — for the heading-correction term in `update_aruco`). Save button writes positions to `constellation_map.json` (existing format) and overrides `MARKER_TO_ROOM[mid]["marker_pos"]` via a new `room_markers.json` (don't auto-edit `floorplan.py`). On save, also push live into `Localization._marker_positions` so the running agent picks them up without restart. Existing shift-click stub (`dashboard.py:449-455`) is replaced — it currently doesn't persist anywhere. | `dimos_lite/dashboard.py` (new survey panel + persistence endpoint), `dimos_lite/localization.py` (live-reload hook), new `room_markers.json` loader in `dimos_lite/floorplan.py` | ~200 | — |
| M1 | **Unify pose state.** Make `SemanticMap` stop tracking its own `(x, y, heading)` as independent state — read from `Localization` on every access. Or alternatively, have `Localization.update_aruco` push the corrected delta into `SemanticMap` whenever it fuses. Removes the dashboard-dot-vs-obstacle-cloud drift. | `dimos_lite/agent.py` (SemanticMap class, `_on_odometry`, `_on_imu`, `add_obstacle`, dashboard publish at line 938) | ~80 | M0 |
| M2 | **World-anchored obstacle grid + free-space clearing.** Store obstacles in apartment-pixel coords (not robot-relative grid cells). `add_obstacle(angle, distance)` computes `(robot_apartment_x + d·sin(heading+angle), robot_apartment_y - d·cos(heading+angle))` and writes to a 700×700 (or 70×70 with 10cm cells) world-frame grid. Also clear the cone between robot and the hit (free-space update) so phantom obstacles fade as the robot moves through a region. Make occupancy probabilistic: each cell holds a hit count or log-odds; render as 'W' only above a threshold. | `dimos_lite/agent.py` (SemanticMap rewrite — coord system + occupancy model + `get_obstacles` simplification), `cartographer.py` consumers | ~150 | M1 |
| M3 | **Fusion tuning + per-marker reliability.** Tighten ArUco trust filter (reject `distance_cm < 30` where bearing noise blows up, reject high bearing-from-camera-axis where pose error compounds). Drop α to ~0.1 for normal fusion. Track per-marker reliability (variance of recent fixes); markers with consistent fixes get higher α, jittery ones get downweighted. Replace the heading-correction line in `update_aruco` (`localization.py:194-197`) with a version that uses surveyed marker orientation, not the (currently broken) `est_hdg = (360 - bearing_deg) % 360` shortcut. | `dimos_lite/localization.py:149-204` | ~80 | M0, M1 |
| M4 | **Recovery / kidnapped-robot handling.** When no ArUco visible for >10 s, agent should `scan` actively to search for markers. When a high-confidence marker is sighted after a long gap (>30 s) AND its est_pose differs from current pose by >50 cm, allow a one-time hard snap (instead of α-fusion) to recover from accumulated odometry drift. Log every snap to the action log so the user can see when relocalization fired. | `dimos_lite/agent.py` (think loop + scan hook), `dimos_lite/localization.py` (snap mode) | ~80 | M1, M3 |

**Total scope: ~590 LOC across five stages.** No firmware changes. No new hardware. The user's offer to physically survey 27 markers is the M0 input that unblocks everything else.

### Acceptance tests

- **M0:** User pins all 27 markers in the dashboard, hits save, restarts the agent → boot log shows `Loaded 27 constellation markers` (not 2). `constellation_map.json` and `room_markers.json` contain all 27 positions. Driving the robot near a marker shows it picks up the right marker ID and zone label without restart.
- **M1:** Set robot position via dashboard click → obstacle cloud moves with the dot, not stuck at apartment centre. After an ArUco sighting that snaps the dot, the obstacle cloud snaps with it.
- **M2:** Drive a circuit around a chair → obstacles cluster around the chair's actual apartment position, not around (350, 350). Drive *through* a region previously marked obstacle (e.g. picked up a phantom hit from sonar noise) → that cell clears within a few passes. `semantic_map.json` payload format changes; old files are rejected on load (don't silently corrupt).
- **M3:** Watching the robot dot during a marker sighting → smooth drift to the correct position over 1–2 seconds, not a yank. Heading correction visible in IMU vs Localization heading delta — they converge slowly.
- **M4:** Manually carry the robot to a different room, set it down → within 30 s of seeing any marker, dot snaps to roughly correct position, action log shows `[Localization] RECOVERY SNAP: marker N, pose was X cm off`.

### Execution order

**M0 → M1 → M2 → M3 → M4.** M0 is the ground-truth bottleneck; without surveyed markers, M1–M4 can't be evaluated. M1 and M2 are the structural fixes — most of the visible improvement comes from these. M3 and M4 are the polish that gets us from "works most of the time" to "animal-equivalent."

### If resuming from a cold context

Check the boot log of `python3 main_os.py`. If `[Localization] Loaded N constellation markers` shows N < 5, **M0 hasn't been completed by the user yet** — surface the survey panel and wait for them to pin markers before doing anything else. If `SemanticMap` in `dimos_lite/agent.py` still has its own `self.x`, `self.y`, `self.heading` attributes that get incremented in `update_position` / `turn`, M1 hasn't shipped. If `get_obstacles()` still does `350 + (c - cx) * self.cell_cm`, M2 hasn't shipped.

Stage-progress breadcrumbs will live in `CHANGELOG.md` under `[5.8.0-preX]` entries.

---

## Deferred — intelligence-layer roadmap (v5.7.x)

Picked up *after* the mapping work above lands. Stage A is shipped; B–F are parked because layering richer reasoning on top of a broken pose estimator is wasted work — the LLM can't reason about "the cat is in the kitchen" if the kitchen isn't where it thinks it is.

Six items as originally scoped:

| Stage | Item | Files / entry points | LOC | Depends on |
|-------|------|----------------------|-----|------------|
| A (#1) | **Rewrite `_build_prompt` as first-person narrative.** Open with a persona + operational commitments (explore, 30 s human cooldown, cat-play rule, summarise-periodically, self-patch allowed). Reorganise the sensor block as "I perceive…". Keep the tool schema + JSON output spec. | `dimos_lite/agent.py:669-714` | ~100 | — |
| B (#3) | **30-second manual-command cooldown.** Add `self._last_manual_ts`, update it in the manual-command path, have `_think_loop` early-exit when `now - last_manual_ts < 30`. Dashboard telemetry: push `"action": "standing_by"` + countdown. Prompt: when just exited cooldown, inject "Human was driving X s ago — resuming autonomy." | `dimos_lite/agent.py` (`_think_loop` ~820, manual path ~837, `_build_prompt`) + `dimos_lite/dashboard.py` (telemetry key) | ~25 | — |
| C (#4) | **Describe surroundings in the prompt.** New `_describe_surroundings()` helper that emits: nearest known marker (distance + bearing + room-label from `floorplan.ROOMS`), obstacle density per compass direction (count of `'W'` cells in a 30 cm cone), nearest known room boundary, seconds-since-last-marker-correction. Replace the thin `Neighbors: N=… S=… E=… W=…` line. | new helper in `dimos_lite/agent.py`; consumer in `_build_prompt` | ~70 | A |
| D (#2) | **Spatial object memory.** New `SpatialMemory` class: `{label: [(x, y, heading, room, ts, conf), …]}`. Write on YOLO detections where confidence > 0.6 and the label held for ≥ 2 consecutive frames. Compute apartment position from `_localization.get_position()` + YOLO bbox bearing. Dedupe: drop if within 30 cm of an existing entry of the same label. Decay after 300 s. Cap per-label at 20 entries. Persist to `spatial_memory.json`. Surface the top N most-recent into the prompt as "Recent sightings: cat in Living Room 12 s ago (420, 180); chair in Kitchen 2 min ago (120, 80)." | new class in `dimos_lite/agent.py`; wired into `_on_detections` ~404, `_build_prompt`; main_os loads on boot | ~150 | A |
| E (#5) | **Cat-play mode.** Watch `SpatialMemory` for cat sightings. If cat label hit with conf > 0.7 on ≥ 3 of the last 5 frames → `_cat_present = True`; clear if no sighting for 60 s. When true, inject into prompt: "🐱 CAT IN VIEW — playful mode. Short moves (≤ 20 cm), 180° pivots, gentle speeds, don't approach fast." | `_on_detections`, `_build_prompt` | ~40 | A, D |
| F (#6) | **Task state machine.** Add `self._current_task ∈ {explore, play_with_cat, report, idle}`. New `set_task` tool. Auto-transition: cat present → `play_with_cat`; >5 min since last speak → `report`; default → `explore`; 30 s manual cooldown → `idle`. Inject current task into prompt ("Current task: explore — systematically visit unseen rooms"). Expose in dashboard telemetry. | `dimos_lite/agent.py` (TOOLS list, _execute_tool, think loop, prompt) + dashboard telemetry key | ~70 | A, E |

**Total scope: ~450 LOC across six stages.** None of this requires firmware changes. None of it requires new hardware.

### Acceptance tests

Per stage, what "done" looks like — check these before moving to the next stage:

- **A:** LLM's `thought` field references identity and current task, not just reaction to distances. Prompt now includes operational rules as prose, not a numbered rule list. Verify by running one tick and reading stdout.
- **B:** After pressing WASD, agent does not fire the think loop for 30 s; dashboard action cell shows "standing by" with countdown; first tick after cooldown mentions the resume in the prompt.
- **C:** Prompt mentions rooms and bearings (e.g. "Nearest marker: ID 17, 1.8 m SE — Living Room south wall"), not just grid cells. Grep the prompt output for `Kitchen|Bedroom|Living Room|Bath|Hall|Entry`.
- **D:** Drive past a chair → restart → chair still appears in prompt as a recent sighting with `spatial_memory.json` containing the entry. Dedupe working: pass the same chair twice from different angles → only one entry per ~30 cm cluster.
- **E:** Cat visible → action log shows short moves and pivots rather than forward exploration. Cat gone 60 s → mode clears, back to exploration.
- **F:** Task persists across ticks (grep prompt for `Current task:`). Dashboard telemetry shows current task. Transitions logged to action log.

### Intelligence-layer execution order (deferred)

Original order: **A → B → C → D → E → F**. A is shipped (`v5.7.0-preA`). B–F resume only after the mapping workstream above has landed M0–M2 at minimum (a unified, world-anchored pose is a precondition for the spatial-context prompt and spatial object memory in stages C and D).

---

_Last snapshot: 2026-04-25 (v5.8.0-preM0b — hardware sweep + firmware regression repairs: MPU6050 brought back online via SoftI²C on its actual bus (GP0/GP1), VL53L0X swapped from a broken 51-line stub to the full Pololu-port driver in continuous mode, dashboard tiles restored (sonar / **TOF laser** / heading w/ IMU badge / speed / mileage / grayscale / IMU accel) and camera panel CSS fixed to match stream aspect ratio. Mapping workstream M1 still pending; user has not yet completed the marker survey. See `CHANGELOG.md`.). Bump the date on substantive edits._
