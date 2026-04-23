# Changelog

All notable changes to the Agent OS project are recorded here. Newest at the top. Keep entries dated (YYYY-MM-DD) and grouped by what shipped together. `CLAUDE.md` carries the current snapshot; this file carries the history.

Format: loosely [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), versioning aligned with `docs/USER_GUIDE.md`.

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
