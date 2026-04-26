# Changelog

All notable changes to the Agent OS project are recorded here. Newest at the top. Keep entries dated (YYYY-MM-DD) and grouped by what shipped together. `CLAUDE.md` carries the current snapshot; this file carries the history.

Format: loosely [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versioning aligned with `docs/USER_GUIDE.md`.

---

## [5.8.0-preM0b] — 2026-04-25

Hardware-sweep + firmware regression repairs. A previous editor's VL53L0X integration silently broke the MPU6050 (wrong I²C bus in firmware) and shipped a 51-line VL53L0X "stub" that returned `None` 100% of the time. Dashboard tiles for grayscale / odometry / IMU were missing or stale, and the camera panel had `flex:1; height:100%` so the video letterboxed inside a giant box that stole space from the brain panel. After a full live-Pico diagnostic this commit fixes both the firmware and the dashboard.

### Sensor / firmware

- **MPU6050 moved to its actual bus.** `examples/app_control.py` was opening MPU on `I2C(1, sda=Pin(2), scl=Pin(3))` (I²C1 / GP2/GP3 — where the VLX lives). The MPU is physically wired to **I²C0 / GP0/GP1** per `libs/mpu6050.py`'s docstring, so the boot scan never saw 0x68 and `HAS_IMU` stayed `False` forever. Fixed.
- **Both sensors switched to SoftI²C.** Hardware-I²C peripheral on this Pico's MicroPython 1.28.0 build EIOs every clock-stretched register read on both buses (verified across 50/100/200/400 kHz, with internal pull-ups, with bus-clear sequences, with each device tested in isolation). SoftI²C handles stretching cleanly. Cost is ~1 ms/tick — sonar's HC-SR04 echo-wait already blocks for 10–25 ms per read every 4 ticks, so SoftI²C is well below the loop's existing budget.
- **`libs/vl53l0x.py` replaced.** The previous 51-line stub had an incomplete init sequence and polled the wrong register for "data ready"; `tof.read()` returned `None` on every call. Replaced with the canonical Pololu-port MicroPython driver (uceeatz/VL53L0X, 648 LOC) — full ST init: SPAD calibration, Vhv calibration, phase calibration, signal-rate limit, sequence-step config, timing-budget tuning. Added `read_mm()` (raw, raises on timeout) + `read()` (cm float or `None`, never raises) so the firmware's call site at `app_control.py:228` keeps working unchanged. **Continuous mode** is started after init (`tof.start(period=0)`) — `read()` now polls the latest cached result in ~8 ms instead of triggering+waiting 37 ms per single-shot.
- **Verified live** against a physical 5.08 cm (2 in) ground-truth target: reads cluster 4.9–5.5 cm, well within the chip's ±5% spec for an uncalibrated unit.

### Dashboard (`dimos_lite/dashboard.py`, `dimos_lite/pico.py`, `dimos_lite/agent.py`)

- **Camera panel sized to match the stream.** `.video-panel` was `flex:1` — the panel grew to fill the entire left column, the image inside used `object-fit: contain`, so VGA frames letterboxed into ~50% black bars. Switched to `flex: 0 0 auto; aspect-ratio: 4/3; max-height: 55vh; min-height: 160px` and added a JS hook that updates `panel.style.aspectRatio` from `vid.naturalWidth/naturalHeight` once the first frame loads — auto-adapts to whatever resolution the ESP32-CAM is currently configured for. Brain panel now `flex: 1 1 240px` so it claims the reclaimed space.
- **TOF telemetry plumbed end-to-end.** Pico firmware already publishes `T` in `ws.send_dict`. Added `tof_distance` `StreamOut` to `PicoHardwareModule` + a `'T'` packet handler. Added `_tof_distance` state + `_on_tof` subscriber + `tof_distance` key in the 10 Hz dashboard telemetry push from `AgentCoreModule._dashboard_loop`. Added a `tof_distance: None` slot to `DashboardState.telemetry`.
- **New tiles in the AGENTIC INTELLIGENCE panel.**
  - **Sonar (HC-SR04)** — was labelled `Forward Distance` before; now distinguished from the laser.
  - **Laser (VL53L0X)** — new tile, cm reading + saturation bar (red <30 cm, amber <60 cm).
  - **Heading** — already existed; the IMU badge / `(gz: …)` annotation only fires when `has_imu` is `True`, so this lights up automatically now that MPU is back online.
  - **Speed / Mileage** — already existed; updates in real-time again because the `B`/`C` packets are getting through.
  - **IMU Accel (g)** — new tile, 3-axis accelerometer in g.
  - **Grayscale** — already existed; previously stuck because the IMU regression cascaded into a stale telemetry push, now updates as expected.

### Documentation

- `CLAUDE.md`: GPIO map corrected (MPU on GP0/GP1 / I²C0, VLX on GP2/GP3 / I²C1 — previous map said both were on I²C1). Sensor status section reflects current reality. "What doesn't work" section dropped the now-resolved dashboard regressions, kept the ESP32-CAM brightness regression as deferred.

### Deferred / known limitations

- **ESP32-CAM brightness** — the swapped board's firmware exposes only binary on/off via `?led=on/off`. URL probing for `led_intensity`/`led_brightness`/`led_value` did not unlock continuous control. Either reflash original CameraWebServer or live with binary. User explicitly deferred this to the very last step.
- **Marker survey not yet completed by user.** M1 (unified pose state) remains unevaluable until the survey is done. Independent of this commit.

### Acceptance test

1. Power-cycle Pico → boot log shows `MPU6050 at 0x68 on GP0/GP1 — calibrating...` then `MPU6050 ready` and `VL53L0X TOF Laser ready on GP2/GP3 (continuous mode)`.
2. Place a ~5 cm object in front of the laser → `tof.read()` returns ~5 cm.
3. `python3 main_os.py` and open `http://localhost:8080` → camera panel matches the camera's aspect ratio (no thick black bars), brain panel grew accordingly. `Sonar`, `Laser`, `Heading` (with IMU badge), `Speed/Mileage`, `IMU Accel`, and `Grayscale` tiles all update at ~10 Hz.

---

## [5.8.0-preM0] — 2026-04-24

Stage M0 of the mapping/navigation roadmap (`CLAUDE.md` → "Active workstream — mapping & navigation"): in-dashboard ArUco marker survey tool. The user can now visually pin all 27 physical markers (5 wall + 22 floor) onto the floor-plan canvas and persist their positions to disk.

### Added

- **Survey toggle in the floor-plan panel header (`dimos_lite/dashboard.py`).** A `📍 Survey` button in the header switches the floor-plan canvas between two click modes: default click sets the robot's manual position (existing behaviour), survey click pins the active marker. The toggle reveals an overlay panel with a 27-button palette: 5 wall buttons (IDs 0–4, orange-bordered) and 22 floor buttons (IDs 10–31). Clicking a button selects it; clicking on the canvas pins it. For wall markers a small floating popup with eight cardinal direction buttons (N/NE/E/SE/S/SW/W/NW) appears at the cursor for facing direction. Floor markers auto-advance to the next unpinned ID after each pin. `Save` persists to disk; `Clear` wipes in-memory pins (disk untouched until next save).
- **Three new dashboard endpoints:** `POST /api/pin_marker` (single pin write to in-memory state), `POST /api/unpin_marker` (single pin remove), `POST /api/save_survey` (writes the in-memory state to disk). Survey state is also exposed in the `/api/state` payload as `survey_pins` so the UI can hydrate on page load.
- **`DashboardState._load_survey_from_disk()`.** On dashboard boot, restores the survey state from `room_markers.json` (wall) and `constellation_map.json` (floor) so previously-surveyed markers are visible without re-pinning.
- **`room_markers.json` overlay loader in `dimos_lite/floorplan.py`.** Module-level `_apply_room_marker_survey()` reads the surveyed wall-marker positions and overrides `MARKER_TO_ROOM[mid]["marker_pos"]` + adds a `marker_facing_deg` field. The facing field isn't yet consumed by `update_aruco` — that's M3 — but it's stored for the heading-correction rewrite.
- **Floor-plan canvas now renders `survey_pins`.** Wall markers as orange dots with facing arrows; floor markers as cyan dots; both labelled with their IDs. The active marker (during survey) glows white-edged.

### Changed

- **Removed the shift-click marker-stub in the canvas click handler.** It used a JS `prompt()` for marker IDs 0–4 only, didn't persist to disk, and didn't push into `Localization._marker_positions`. The new survey UI is the canonical path; the orphaned `/api/set_marker` endpoint is left in place but unused (will be cleaned up in a later sweep).

### Deferred / known limitations

- **Restart required to apply the survey.** The agent reads `room_markers.json` (via `floorplan.py` import) and `constellation_map.json` (via `Localization.load_constellation()`) only at boot. Workflow: pin all 27, hit `Save`, restart `python3 main_os.py`. Live push into the running agent is small additional work but not part of M0.
- **Wall-marker facing is captured but not used by fusion yet.** `update_aruco`'s heading-correction term still uses the (incorrect) `est_hdg = (360 - bearing_deg) % 360` shortcut. M3 replaces it with `est_hdg = (marker_facing_deg - bearing_deg + 180) % 360`, which is what the surveyed facings actually unlock.
- **`room_markers.json` is not gitignored.** Per-deployment data, so the user may want to add it; left untouched to avoid changing things outside scope.

### Acceptance test

User opens `http://localhost:8080`, clicks `📍 Survey`, pins all 27 markers (5 wall with facings + 22 floor), hits `Save`. Server log shows `[Dashboard] Survey saved: 5 wall + 22 floor markers`. Restart `python3 main_os.py` → boot log shows `[floorplan] Applied surveyed positions for 5 wall marker(s)` and `[Localization] Loaded 22 constellation markers`. After this, M1 (unify pose) becomes evaluable.

---

## [5.8.0-pre] — 2026-04-23

Roadmap pivot, no code change. After running v5.7.0-preA on real hardware, the user observed the floor-plan map is fundamentally broken: robot dot teleports on ArUco sightings, obstacle cloud renders centred on the apartment middle (spilling into hall/bedroom regardless of where the robot actually is), and the position never matches physical truth. Diagnosis (recorded in CLAUDE.md → "Why mapping is broken today") identified four structural issues — two parallel pose estimators that don't share state, an obstacle frame anchored at apartment centre rather than world origin, obstacle insertion using the un-corrected pose, and discovery from a wrong initial pose pinning markers at fictional coords. The intelligence-layer stages B–F are parked; mapping/navigation (M0–M4) is now the active workstream. Goal: animal-equivalent spatial competence (pose drift ≤ ~30 cm over a 10-min session). Stage A remains shipped — the first-person prompt is independent of pose correctness and pays off once the map underneath it is real. M0 (marker survey tool) is the immediate unblocking step; the user has 27 ArUco markers physically placed and is ready to pin them.

---

## [5.7.0-preA] — 2026-04-23

Stage A of the intelligence-layer roadmap (`CLAUDE.md` → "Intelligence-layer roadmap"): rewrite the LLM prompt as a first-person narrative.

### Changed

- **`AgentCoreModule._build_prompt` (`dimos_lite/agent.py:670`).** Replaced the `YOU ARE DIMOS` sensor-dashboard prompt with a first-person opening that names the persona, hardware, and operational commitments (explore, ~30s manual-control cooldown, cat-play pause, periodic spoken summaries, self-patch + reboot allowed, defer to the reflex layer). Sensor block reorganised as "What I perceive right now" prose. Numbered `RULES:` list dropped — the rules now live as prose inside the persona section. Tool schema and JSON output spec are unchanged. The cooldown / cat-play / summarise rules are encoded in the prompt now even though their enforcement code lands in stages B–F; this lets the LLM start reasoning in those terms early.
- **`_last_detections` is now read inside `_sensor_lock`** while building the prompt, removing a small race against the YOLO callback that overwrites it.

### Notes

- Stage A breadcrumb (per `CLAUDE.md` → "If resuming from a cold context"): `_build_prompt` no longer opens with "YOU ARE DIMOS" — it opens with "I am Dimos". That is the signal that Stage A has shipped; Stage B (30s manual cooldown) is next.
- No firmware, dashboard, hardware, or tool changes. Acceptance test: run one think tick and confirm the model's `thought` field references identity / what it is doing, not just raw distances.

---

## [5.6.0] — 2026-04-23

Cleanup sweep + mapping-pipeline repair. The robot can now run `--carto` as a pre-pass (manual-drive + ArUco discovery + sonar obstacle logging), persist the result to disk, and have normal mode load it on the next boot. The cartographer's live pose + constellation + obstacle cloud render into the main dashboard floor-plan canvas.

### Added

- **`--carto` mode in `main_os.py`.** Runs the cartographer without the LLM brain so mapping isn't steered or slowed by reasoning. Shares the same `VisionModule` / `PicoHardwareModule` / `LocalizationModule` instances that normal mode uses.
- **`SemanticMap.save_to_disk()` / `load_from_disk()`** (`dimos_lite/agent.py`). Persists grid + pose to `semantic_map.json`. Cartographer saves in its `finally:` block; `main_os.py` calls `brain.semantic_map.load_from_disk()` right after constructing the agent so normal mode starts with prior obstacle memory. Shape-mismatched files are rejected rather than crashing.
- **Constellation auto-load in normal mode.** `main_os.py` now calls `loc.load_constellation()` before `loc.start()`, so discovered ArUco markers from a cartographer pass are immediately available for pose correction. Previously, markers lived in `constellation_map.json` but were never read back.
- **Cartographer → dashboard mini-map.** The mapping run now pushes pose, heading, marker positions, obstacle cloud, forward distance, mileage, and annotated camera frames into the live `/api/state` stream at `http://localhost:8080`. The existing floor-plan canvas already had the scaffolding (`marker_positions`, `obstacles`, `robot_x/y`) — cartographer just wasn't feeding it. Calibration UI still uses the cv2 window for arrow-key / shift-click input; moving that into the dashboard is deferred.
- **`.gitignore`**: added `semantic_map.json` alongside the existing runtime artefacts.

### Fixed

- **CRITICAL: Cartographer's IMU handler never fired.** `_on_imu` tested `'y' in data` and read `data['y']`, but the firmware publishes the IMU frame as a list (`[heading, gyro_z, ax, ay, az]` — see `libs/mpu6050.py:get_telemetry`). The `in` check was a substring-in-list test against float values, so the conditional was always False and the cartographer's heading never updated. ArUco-based pose correction was therefore being computed against a stuck heading, making the whole map garbage. Now indexes by position with an explicit list check. `dimos_lite/agent.py` had always indexed correctly; only cartographer was affected.
- **Cartographer wiped the constellation on every run.** `RESET_DB = True` at module load deleted `constellation_map.json` and `room_fingerprints.json` in `__init__`. Flipped to `False`; when the flag is off, cartographer loads the existing constellation instead of deleting it.
- **`SemanticMap.get_obstacles()` mis-projected obstacles for any grid size other than 21.** Coordinates were hard-coded as `350 + (c - 10) * 10`, which assumed the robot started at grid cell (10, 10). The class actually defaults to `size=71` and centres the robot at `(size // 2, size // 2)` = (35, 35), so every obstacle was rendered ~250 px north-west of its true apartment position on the dashboard floor plan. Now derives the centre from `self.size`.
- **Dashboard HTML had a stranded `<button>S Back</button>`** between the brain panel and the servo panel, with no enclosing control row — the rest of the WASD buttons had been deleted in a prior edit, leaving the DOM malformed. Restored a clean `.controls-row` with forward / left / backward / right / stop buttons. The CSS class was already defined but had no element using it.
- **Dashboard double-registered its `keydown` / `keyup` handlers.** Two separate `document.addEventListener` blocks both mapped WASD, with different `e.key` vs `e.key.toLowerCase()` casings and different interval cadences (100 ms vs 80 ms). Typing 'w' ran both handlers, double-sending `forward` and leaking the first `setInterval`. Merged into one handler that covers WASD, space, and the pan/tilt arrow keys.

### Changed

- **`cartographer.py` moved from repo root into `dimos_lite/cartographer.py`** as a proper package module. Removed its `if __name__ == "__main__":` block and the module-level `vision` global it used to reach for inside `run()`; vision is now injected via `DeepScanCartographer.__init__(loc, vision)`. The only entry point is `python3 main_os.py --carto`.

### Removed

- **Repo-root test and one-shot scripts (17):** `auto_finder.py`, `check_esp.py`, `check_esp2.py`, `inject_servo.py`, `listen_serial.py`, `pet_brain.py`, `pet_brain_vlm.py`, `read_pico_boot.py`, `scan_uart.py`, `setup_vision.py`, `sweep_uart.py`, `test_4_5.py`, `test_8_9.py`, `test_motors.py`, `test_repl.py`, `test_servo.py`, `test_uart.py`, `test_uart_89.py`. Pre-dimos prototypes and UART-probe scripts, not referenced by any live module.
- **`controller.html`** (19 KB) — an orphan standalone mobile UI. The live dashboard is served inline from `dimos_lite/dashboard.py:DASHBOARD_HTML`; this file was unreferenced.
- **`dimos_os_v4_context_dump.md`** — one-time context paste left in the tree.
- **`tests/` directory** (10 files) — MicroPython device-side smoke tests last touched in 2023.
- **`models/` directory** — MobileNetSSD caffemodel + prototxt. Superseded by YOLOv8 via `ultralytics` (auto-downloads `yolov8n.pt`).
- **Most of `examples/`** — kept only `app_control.py` (the live Pico firmware). Removed 12 pre-dimos demos (`bull_fight.py`, `donot_push_me.py`, `esp_at_test.py`, `esp_test.py`, `follow_hand.py`, `force_wifi.py`, `line_track.py`, `move_forward.py`, `obstacle_avoid.py`, `test_uart_all.py`, `uart_listen.py`, `uart_scan.py`).

### Known issues / deferred

- Cartographer still opens its own `cv2.namedWindow` for calibration (arrow keys + shift-click). The dashboard mini-map now mirrors the same information in the browser, so the cv2 window is redundant except for calibration input. Moving calibration controls into the dashboard is pending.
- `SemanticMap` grid is 71×71 with 10 cm cells (~7.1 m × 7.1 m), not the 21×21 documented in pre-5.6 `CLAUDE.md`.
- `patch_self`, `reboot`, and `tune_parameters` remain in the agent's tool list. These are intentional — the 31B agent is being given self-modification as a first-class capability, not a safety oversight.

---

## [5.5.2] — 2026-04-23

Chassis-LED recovery.

### Fixed

- **WS2812 strip on GP19 was disabled and replaced with non-existent hardware.** A prior edit had commented out `np = WS2812(Pin(19, Pin.OUT), 24)`, gutted every `set_light_*` body, and added `PWM(Pin(19/20/21/22))` initialisers for analog LEDs that aren't wired in this build (GP20/21/22 are the free-GPIO pins in `docs/USER_GUIDE.md`). It also imported `PWM` nowhere — so `libs/pico_4wd.py` raised `NameError` at import time, which on the Pico probably cascaded into the whole stack failing at boot. Restored the PIO-driven WS2812 driver and the real function bodies (`write_light_color_at`, `light_excute`, `set_light_all_color`, `set_light_bottom_color`, `set_light_bottom_left_color`, `set_light_bottom_right_color`, `set_light_rear_color`, `set_light_off`). Deleted the bogus PWM block.
- **`set_light_off` called a function that had been commented out** (`set_light_all_color`) — now calls the restored version.
- **Blocking `set_turn_signal(direction)`** used `time.sleep` inside what should be a 200 Hz control loop and was only reached via the `car.move()` fallback in firmware. Removed the blocking implementation and the calls from `move()`.

### Added

- **Non-blocking turn signals in firmware.** `examples/app_control.py` now tracks a `turn_signal` state derived from the incoming `K` command (`left` / `right` with nonzero power) and blinks the corresponding bottom-strip segment amber from the main tick loop: ~3 Hz on the left side when turning left, ~1.5 Hz on the right side when turning right. Matches the `USER_GUIDE.md` v5.5 "Fast=Left, Slow=Right" spec. The cached `L_BOT` colour from the Mac is used as the "off-phase" background so the underglow isn't lost while a signal is active, and it's repainted to both halves when the turn ends. Signals also cancel on the `CMD_TIMEOUT_MS` auto-stop.

---

## [5.5.1] — 2026-04-23

Bug-fix pass against v5.5. No feature additions.

### Fixed

- **Pico firmware ignored every incoming command.** `on_receive` was defined at module scope in `examples/app_control.py` but never wired into the `WS_Server` instance, whose default handler is a no-op. Added `ws.on_receive = on_receive` so the bridge actually dispatches packets.
- **SLAM obstacles never reached the dashboard.** `update_dashboard(obstacles=...)` was filtered out by `if k in state.telemetry` in `dimos_lite/dashboard.py`. Added `"obstacles": []` to the telemetry dict so the floor-plan canvas actually draws the SLAM dots promised by the v5.5 release notes.
- **Reflex safety thresholds were dead code during motion.** `dimos_lite/agent.py` gated its 45 cm and wide-cone guards on `is_manual = ... and not self._pending_plan`, but `_pending_plan` was only ever initialised to `None`, so `is_manual` was always truthy and only the 20 cm emergency stop ever fired. Removed the gate; the full safety stack now applies while moving.
- **Dashboard "click to place" crashed the semantic grid.** `AgentCoreModule` wrote apartment pixels (0–700) divided by 10 directly into `semantic_map.x/y`, producing floats up to 70 on a 21-cell grid. `get_neighbors()` then returned `'W'` for every direction (LLM saw itself walled in) and any subsequent `add_obstacle` would have TypeError'd on a float index. Now recenters the local grid to (10, 10) and leaves world-frame tracking to the `LocalizationModule`.
- **Grid position double-counted during motion.** `_execute` and the manual-override path were incrementing `semantic_map.update_position(±1)` every 50 ms while `_on_odometry` was *also* applying the real encoder delta. The robot dot on the floor plan moved at ≈2× reality. Translation is now odometry-only; turns are dead-reckoned only when the IMU is absent.
- **Localization ArUco snap had a sign error and ignored robot heading.** `update_aruco` placed the robot using `mx + D·sin(B), my + D·cos(B)`, which only agrees with the forward-motion convention (`x += D·sin(H); y -= D·cos(H)`) when the robot faces exactly north. Added a `heading_deg` parameter and corrected the inverse placement to `mx − D·sin(H+B), my + D·cos(H+B)`. Agent passes its current heading when snapping.
- **Dead `_sync_dashboard` thread.** `AgentCoreModule.start` was spawning a thread whose target was a one-shot method, so it exited immediately. Removed; the main control loop already calls it each tick.
- **Redundant `lsof` call** in `dashboard.start_dashboard` whose stdout was discarded.
- **Duplicate constant definitions** (`MOVEMENT_SPEED`, `TURN_SPEED`, `COLLISION_*`) in `dimos_lite/agent.py` — removed the second copy.

### Docs

- Added `CLAUDE.md` (living status) and `CHANGELOG.md` (this file).
- Expanded `.gitignore` to cover pycache, macOS cruft, debug artifacts, and the 1 MB MicroPython binary.

---

## [5.5] — before 2026-04-23

Baseline for this changelog. Feature set from `docs/USER_GUIDE.md` header:

### Added
- High-Definition Vision — ESP32-CAM bumped to SVGA (800×600).
- Vision HUD — AR-style bounding boxes and ID/Distance labels on stream.
- Pose Estimation SLAM — exact distance/bearing snapping from ArUco markers.
- Live SLAM Mapping — discovered obstacles appear as dots on dashboard _(never actually reached the UI until 5.5.1; see Fixed above)_.
- PWM LED Overhaul — support for analog LEDs on Pins 19, 20, 21, 22 _(chassis strip on GP19 not currently lighting; see `CLAUDE.md` known issues)_.
- Turn Signals — automatic blinking headlamps (Fast=Left, Slow=Right).
- LLM Stability — 90 s timeout and optimised token predict (512) for 31B model.

## [4.1]

### Added
- MPU6050 IMU (verified, live heading correction).
- LLM upgraded to gemma4:31b.
- Vision bumped to 896×896 for chair-leg detection.
- ArUco marker localization live (fuses with RSSI).
- Full-featured dashboard (radar canvas, floor plan, IMU badge).
- Safety reflex thread split from main control loop (think thread).
- Pico firmware auto-stop on command timeout (`CMD_TIMEOUT_MS=2000`).

## [4.0]

### Added
- Multi-stream architecture (6 streams).
- Mapper module (`--map` mode).
- Dashboard.

## [3.5]

### Added
- Super-smooth radar sweep (5° steps, 10 ms tick).

## [3.x]

### Added
- Original LLM navigation.
