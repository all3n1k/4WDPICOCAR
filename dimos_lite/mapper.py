"""
dimos_lite/mapper.py — Pre-operation Mapping Mode

Runs a fully autonomous, LLM-free mapping sweep before handing control to the
AgentCoreModule. Inspired by robot vacuum setup routines.

Three phases:
    1. Perimeter Trace   — wall-follow to map all room boundaries
    2. Interior Sweep    — boustrophedon (lawnmower) grid coverage
    3. RSSI Heatmap      — records Wi-Fi signal strength at each waypoint

Usage:
    python3 main_os.py --map
"""

import json
import math
import os
import time
import subprocess
import re
from collections import deque

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────
WALL_FOLLOW_TARGET_CM  = 20    # desired distance from left wall while tracing
WALL_FOLLOW_TOLERANCE  = 8     # ± cm before corrective turn
FORWARD_COLLISION_CM   = 35    # stop/turn if front < this during mapping (conservative)
SWEEP_ROW_SPACING_CM   = 20    # distance between boustrophedon sweep rows (~2 cells)
PERIMETER_TIMEOUT_S    = 300   # max seconds for perimeter phase
SWEEP_TIMEOUT_S        = 600   # max seconds for sweep phase
AIRPORT                = "/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport"
MAP_OUT                = os.path.join(os.path.dirname(__file__), '..', 'apartment_map.py')
RSSI_LOG               = os.path.join(os.path.dirname(__file__), '..', 'debug', 'rssi_heatmap.json')


def _get_rssi() -> int | None:
    try:
        r = subprocess.run([AIRPORT, "-I"], capture_output=True, text=True, timeout=2)
        m = re.search(r'agrCtlRSSI:\s*(-\d+)', r.stdout)
        return int(m.group(1)) if m else None
    except Exception:
        return None


# ──────────────────────────────────────────────────────────────────────────────
# MappingSession
# ──────────────────────────────────────────────────────────────────────────────
class MappingSession:
    """
    Orchestrates the full mapping sweep. Requires live references to:
      - cmd_fn:      callable(action: str) — sends 'forward'/'left'/'right'/'stop'/'backward'
      - distance_fn: callable() -> float   — returns current forward ultrasonic distance (cm)
      - sweep_fn:    callable() -> list    — returns full radar sweep [(angle, dist), ...]
      - mileage_fn:  callable() -> float   — returns cumulative wheel mileage (cm)
      - semantic_map: SemanticMap instance (shared with AgentCoreModule)
    """

    def __init__(self, cmd_fn, distance_fn, sweep_fn, mileage_fn, semantic_map):
        self.cmd        = cmd_fn
        self.distance   = distance_fn
        self.sweep      = sweep_fn
        self.mileage    = mileage_fn
        self.smap       = semantic_map
        self.rssi_log   = []         # [(grid_x, grid_y, rssi)]
        self._last_mile = 0.0

    # ── Primitive moves ────────────────────────────────────────────────────────

    def _step(self, action: str, duration: float = 0.3):
        """Execute a slow, methodical movement step with mid-step collision polling."""
        # Use 25% power for mapping (methodical and slow)
        self.cmd(action, speed=25)
        
        elapsed = 0.0
        poll_interval = 0.05
        while elapsed < duration:
            time.sleep(poll_interval)
            elapsed += poll_interval
            if action == 'forward' and self.distance() < FORWARD_COLLISION_CM:
                print(f"[Mapper] ⚠ Mid-step collision guard ({self.distance():.1f}cm) — stopping early")
                break
        
        self.cmd('stop')
        time.sleep(0.5)  # Settle time after every single movement burst

    def _do_radar_sweep(self):
        """Trigger a full radar sweep and update the semantic map. Very slow/methodical."""
        time.sleep(0.5)  # Let vibrations settle before reading
        readings = self.sweep()
        fwd = self.distance()
        for angle, dist in readings:
            self.smap.add_obstacle(angle, float(dist))
        time.sleep(0.2)  # Short pause after processing
        return fwd

    def _record_rssi(self):
        rssi = _get_rssi()
        if rssi is not None:
            self.rssi_log.append({
                "x": self.smap.x,
                "y": self.smap.y,
                "rssi": rssi
            })

    def _print_map(self, phase: str):
        print(f"\n[Mapper] Phase: {phase}")
        print(self.smap.render())
        print()

    # ── Phase 1: Perimeter Wall Follow ────────────────────────────────────────

    def phase_perimeter(self):
        """
        Simple left-hand wall-following algorithm.
        Robot keeps the left wall within WALL_FOLLOW_TARGET_CM ± TOLERANCE.
        Stops when it has returned close to the starting grid cell.
        """
        print("\n[Mapper] ═══ PHASE 1: PERIMETER TRACE ═══")
        print("[Mapper] Searching for a wall to follow...")

        start_x, start_y = self.smap.x, self.smap.y
        t0 = time.time()

        # Step 1: Drive forward until we find a wall to begin with
        for _ in range(20):
            fwd = self._do_radar_sweep()
            if fwd < FORWARD_COLLISION_CM:
                break
            self._step('forward', 0.4)
            self._update_mileage()
        self._step('right', 0.5)  # Turn to put wall on left

        returned = False
        steps_taken = 0

        while time.time() - t0 < PERIMETER_TIMEOUT_S:
            fwd = self._do_radar_sweep()
            self._record_rssi()
            self._print_map(f"Perimeter ({steps_taken} steps)")

            if fwd < FORWARD_COLLISION_CM:
                # Wall ahead — turn right
                print(f"[Mapper] Wall ahead ({fwd:.1f}cm) — turning right")
                self._step('right', 0.5)
                self.smap.heading = (self.smap.heading + 90) % 360
            else:
                # Check left side distance (angle = -90)
                left_dist = self._left_distance()

                if left_dist > WALL_FOLLOW_TARGET_CM + WALL_FOLLOW_TOLERANCE:
                    # Drifted away from left wall — steer left
                    self._step('left', 0.15)
                    self.smap.heading = (self.smap.heading - 15) % 360
                elif left_dist < WALL_FOLLOW_TARGET_CM - WALL_FOLLOW_TOLERANCE:
                    # Too close to left wall — steer right
                    self._step('right', 0.15)
                    self.smap.heading = (self.smap.heading + 15) % 360
                else:
                    # On track — go forward
                    self._step('forward', 0.4)
                    self._update_mileage()

            steps_taken += 1

            # Check if we've returned near the start
            dist_to_start = math.sqrt(
                (self.smap.x - start_x) ** 2 + (self.smap.y - start_y) ** 2
            )
            if steps_taken > 20 and dist_to_start < 2:
                print("[Mapper] ✓ Returned to start — perimeter complete!")
                returned = True
                break

        if not returned:
            print("[Mapper] ⚠ Perimeter timeout — proceeding with partial map")

        self._step('stop', 0.0)

    def _left_distance(self) -> float:
        """Get the distance reading closest to -90 degrees (left side)."""
        readings = self.sweep()
        best = 999.0
        best_angle_delta = 999
        for angle, dist in readings:
            delta = abs(angle - (-90))
            if delta < best_angle_delta:
                best_angle_delta = delta
                best = float(dist)
        return best

    # ── Phase 2: Interior Boustrophedon Sweep ────────────────────────────────

    def phase_sweep(self):
        """
        Systematic lawnmower pattern covering the interior.
        Sweeps left→right, shifts down, right→left, repeat.
        """
        print("\n[Mapper] ═══ PHASE 2: INTERIOR SWEEP ═══")
        t0 = time.time()
        going_right = True
        row_count = 0
        steps_taken = 0

        # Navigate to top-left corner of the map's known free space
        self._navigate_to_sweep_start()

        while time.time() - t0 < SWEEP_TIMEOUT_S:
            fwd = self._do_radar_sweep()
            self._record_rssi()

            if steps_taken % 5 == 0:
                self._print_map(f"Sweep row {row_count} ({'→' if going_right else '←'})")

            if fwd > FORWARD_COLLISION_CM:
                self._step('forward', 0.4)
                self._update_mileage()
                steps_taken += 1
            else:
                # End of row — shift down and reverse direction
                print(f"[Mapper] End of sweep row {row_count} — reversing")
                self._step('right' if going_right else 'left', 0.5)
                self.smap.heading = (self.smap.heading + (90 if going_right else -90)) % 360
                # Drive one row width
                for _ in range(2):
                    if self.distance() > FORWARD_COLLISION_CM:
                        self._step('forward', 0.4)
                        self._update_mileage()
                # Turn to face new sweep direction
                self._step('right' if going_right else 'left', 0.5)
                self.smap.heading = (self.smap.heading + (90 if going_right else -90)) % 360
                going_right = not going_right
                row_count += 1

                # Stop if map is mostly covered
                explored = self._explored_fraction()
                print(f"[Mapper] Map coverage: {explored:.0%}")
                if explored > 0.70:
                    print("[Mapper] ✓ 70% coverage reached — sweep complete!")
                    break

        self._step('stop', 0.0)

    def _navigate_to_sweep_start(self):
        """Best-effort navigation to top-left free cell to start sweep."""
        # Simple: just back up to top of room
        print("[Mapper] Navigating to sweep start...")
        for _ in range(10):
            if self.distance() < FORWARD_COLLISION_CM:
                break
            self._step('forward', 0.4)
            self._update_mileage()

    def _explored_fraction(self) -> float:
        total = self.smap.size * self.smap.size
        empty = sum(
            1 for r in range(self.smap.size)
            for c in range(self.smap.size)
            if self.smap.grid[r][c] == ' '
        )
        return 1.0 - (empty / total)

    # ── Phase 3: RSSI Heatmap Summary ────────────────────────────────────────

    def phase_rssi_summary(self):
        print("\n[Mapper] ═══ PHASE 3: RSSI HEATMAP SUMMARY ═══")
        if not self.rssi_log:
            print("[Mapper] No RSSI data collected (airport not available or no readings).")
            return

        sorted_by_rssi = sorted(self.rssi_log, key=lambda r: r['rssi'], reverse=True)
        strong = sorted_by_rssi[:3]
        weak   = sorted_by_rssi[-3:]

        print("[Mapper] Strongest signal zones (closest to router):")
        for r in strong:
            print(f"  Grid ({r['x']}, {r['y']}) → {r['rssi']} dBm")

        print("[Mapper] Weakest signal zones (farthest from router):")
        for r in weak:
            print(f"  Grid ({r['x']}, {r['y']}) → {r['rssi']} dBm")

        # Save raw heatmap
        os.makedirs(os.path.dirname(RSSI_LOG), exist_ok=True)
        with open(RSSI_LOG, 'w') as f:
            json.dump(self.rssi_log, f, indent=2)
        print(f"[Mapper] Raw RSSI heatmap saved → {RSSI_LOG}")

        # Auto-generate ZONE_MAP entries for localization.py
        print("\n[Mapper] Suggested ZONE_MAP entries for dimos_lite/localization.py:")
        print("  # Paste these into ZONE_MAP and rename zones to match your rooms:")
        if len(sorted_by_rssi) >= 2:
            step = max(1, len(sorted_by_rssi) // 3)
            zones = [
                ("Zone A (near router)", sorted_by_rssi[:step]),
                ("Zone B (mid-range)",   sorted_by_rssi[step:2*step]),
                ("Zone C (far)",         sorted_by_rssi[2*step:]),
            ]
            for name, readings in zones:
                if readings:
                    vals = [r['rssi'] for r in readings]
                    print(f"  \"{name}\": ({min(vals)}, {max(vals)}),")

    # ── Phase 4: Export Map ───────────────────────────────────────────────────

    def export_map(self):
        print(f"\n[Mapper] ═══ EXPORTING MAP → {MAP_OUT} ═══")
        lines = [
            '"""',
            'apartment_map.py — Auto-generated by MappingSession.',
            f'Generated: {time.strftime("%Y-%m-%d %H:%M:%S")}',
            'Each cell is 10cm. W=wall, space=open.',
            '"""',
            '',
            'def make_row(s): return list(s)',
            '',
            'APARTMENT_PRIOR = [',
        ]
        for r in range(self.smap.size):
            row_chars = ""
            for c in range(self.smap.size):
                cell = self.smap.grid[r][c]
                row_chars += 'W' if cell == 'W' else ' '
            lines.append(f'    make_row("{row_chars}"),  # row {r}')
        lines += [
            ']',
            '',
            f'assert len(APARTMENT_PRIOR) == {self.smap.size}',
            f'assert all(len(r) == {self.smap.size} for r in APARTMENT_PRIOR)',
        ]
        with open(MAP_OUT, 'w') as f:
            f.write('\n'.join(lines))
        print(f"[Mapper] ✓ Map saved! Load it next boot via: from apartment_map import APARTMENT_PRIOR")

        # Optional PNG visualization
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import numpy as np
            grid = np.array([
                [1 if self.smap.grid[r][c] == 'W' else 0
                 for c in range(self.smap.size)]
                for r in range(self.smap.size)
            ], dtype=float)
            grid[self.smap.y][self.smap.x] = 0.5  # robot position
            fig, ax = plt.subplots(figsize=(8, 8))
            ax.imshow(grid, cmap='Blues', vmin=0, vmax=1)
            ax.set_title('dimOS-lite — Apartment Map')
            png_out = MAP_OUT.replace('.py', '.png')
            plt.savefig(png_out, bbox_inches='tight', dpi=120)
            plt.close()
            print(f"[Mapper] ✓ Map PNG saved → {png_out}")
        except ImportError:
            print("[Mapper] (matplotlib not installed — skipping PNG export)")

    # ── Odometry helper ───────────────────────────────────────────────────────

    def _update_mileage(self):
        current = self.mileage()
        if self._last_mile == 0:
            self._last_mile = current
        else:
            delta = current - self._last_mile
            if delta > 0:
                self.smap.update_position(delta)
                self._last_mile = current

    # ── Full run ──────────────────────────────────────────────────────────────

    def run(self):
        print("\n" + "="*60)
        print("  dimOS-lite MAPPING MODE  (like a robot vacuum setup)")
        print("="*60)
        print("Phases: Perimeter → Interior Sweep → RSSI Heatmap → Export")
        print("Press Ctrl+C at any time to abort and save partial map.\n")
        try:
            self.phase_perimeter()
            self.phase_sweep()
            self.phase_rssi_summary()
            self.export_map()
        except KeyboardInterrupt:
            print("\n[Mapper] Interrupted — saving partial map...")
            self.export_map()
        finally:
            self.cmd('stop')
        print("\n[Mapper] ✓ Mapping complete. Run without --map to start autonomous navigation.\n")
