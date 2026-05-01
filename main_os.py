"""
main_os.py — dimOS-lite entry point

Normal mode:        python3 main_os.py
Autonomous map:     python3 main_os.py --map
Cartographer pass:  python3 main_os.py --carto

Runtime state policy: every boot wipes semantic_map.json (obstacle cloud)
and constellation_map.json (auto-discovered marker positions). The robot
may have been physically picked up and moved while powered down, so prior
sensor data is presumed stale. User-surveyed wall markers (room_markers.json)
and the static apartment prior (apartment_map.py) are always preserved.

Pass --continue to keep last session's runtime state instead — useful if
the robot stayed put and you want to resume mapping where you left off.
"""

import sys
import os
import time
import threading

# Force local module priority
sys.path.insert(0, os.path.dirname(__file__))

from dimos_lite.core import autoconnect
from dimos_lite.vision import VisionModule
from dimos_lite.pico import PicoHardwareModule
from dimos_lite.agent import AgentCoreModule
from dimos_lite.localization import LocalizationModule
import dimos_lite
print(f"[OS] Modules loading from: {dimos_lite.__file__}")

MAP_FILE = os.path.join(os.path.dirname(__file__), 'apartment_map.py')
MAPPING_MODE = '--map' in sys.argv
CARTO_MODE = '--carto' in sys.argv
CONSTELLATION_MODE = '--constellation' in sys.argv
# Default: discard runtime sensor state on every boot so a robot that was
# moved while powered down doesn't smear its old obstacle cloud onto the new
# location. Pass --continue to keep the prior session's data instead.
CONTINUE_MODE = '--continue' in sys.argv
# The autonomous LLM brain (think loop) is OPT-IN. Without --auto, the dashboard,
# manual control, sensor streams, and reflex safety thread all run normally —
# but the LLM never speaks, plans, or issues tool calls. This makes it safe to
# bring up the robot for testing/calibration without the model immediately
# trying to drive around.
AUTO_MODE = '--auto' in sys.argv

# Files that hold runtime sensor data (cleared on boot unless --continue).
# room_markers.json and apartment_map.py are configuration, not sensor data,
# and are always preserved.
RUNTIME_STATE_FILES = ('semantic_map.json', 'constellation_map.json')


def reset_runtime_state():
    """Delete persisted obstacle map + auto-discovered marker positions so the
    robot wakes up with no preconceptions about where it is or what's around it.
    Called by default; skipped under --continue."""
    cleared = []
    for name in RUNTIME_STATE_FILES:
        path = os.path.join(os.path.dirname(__file__), name)
        if os.path.exists(path):
            try:
                os.remove(path)
                cleared.append(name)
            except OSError as e:
                print(f"[OS] Could not remove {name}: {e}")
    if cleared:
        print(f"[OS] Fresh runtime state — cleared: {', '.join(cleared)}")
    else:
        print("[OS] Fresh runtime state — nothing to clear")

def load_prior_map():
    if not os.path.exists(MAP_FILE):
        print("[OS] No floor plan found — robot will build from scratch.")
        return None
    import importlib.util
    spec = importlib.util.spec_from_file_location("apartment_map", MAP_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    print(f"[OS] Loaded floor plan from {MAP_FILE}")
    return mod.APARTMENT_PRIOR

def run_cartographer(loc, vision, hardware):
    """SLAM pre-pass: discover ArUco markers + sonar obstacles, persist, exit."""
    from dimos_lite.cartographer import DeepScanCartographer
    carto = DeepScanCartographer(loc, vision)
    autoconnect(vision, hardware, carto)
    threading.Thread(target=vision.start, daemon=True).start()
    threading.Thread(target=hardware.start, daemon=True).start()
    print("[OS] Waiting for hardware connections...")
    time.sleep(5)
    carto.run()


def main():
    print("=" * 48)
    print("  Agentic Pet OS (dimOS-lite) v4.0")
    if MAPPING_MODE:
        print("  *** MAPPING MODE ***")
    if CARTO_MODE:
        print("  *** CARTOGRAPHER MODE ***")
    if CONSTELLATION_MODE:
        print("  *** CONSTELLATION DISCOVERY MODE ***")
    if CONTINUE_MODE:
        print("  *** --continue: preserving prior session state ***")
    print("=" * 48 + "\n")

    if not CONTINUE_MODE:
        reset_runtime_state()

    loc = LocalizationModule()
    if CONTINUE_MODE:
        loc.load_constellation()
    loc.start()

    vision = VisionModule(stream_url="http://192.168.1.210/stream")
    hardware = PicoHardwareModule(ws_url="ws://192.168.1.217:8765")

    if CARTO_MODE:
        # Cartographer runs without the LLM brain so mapping isn't slowed
        # or steered by reasoning. It saves to disk; normal mode reads it.
        run_cartographer(loc, vision, hardware)
        return

    prior_map = load_prior_map()
    brain = AgentCoreModule(
        ollama_url="http://digitalstorm:11434/api/generate",
        model="gemma4:26b",
        prior_map=prior_map,
        localization=loc,
        discovery_mode=CONSTELLATION_MODE,
        auto=AUTO_MODE,
    )
    if AUTO_MODE:
        print("[OS] --auto: LLM brain ENABLED — robot will think and drive autonomously")
    else:
        print("[OS] LLM brain DISABLED (no --auto flag). Manual control + sensors + reflex only.")
        print("[OS] Pass --auto to activate the autonomous LLM.")
    if CONTINUE_MODE:
        brain.semantic_map.load_from_disk()

    autoconnect(vision, hardware, brain)

    threading.Thread(target=vision.start, daemon=True).start()
    threading.Thread(target=hardware.start, daemon=True).start()

    print("[OS] Waiting for hardware connections...")
    time.sleep(3)

    if MAPPING_MODE:
        from dimos_lite.mapper import MappingSession

        def cmd_fn(action, **kwargs):
            hardware._on_cmd_vel((action, kwargs.get("speed", 25)))

        mapper = MappingSession(
            cmd_fn=cmd_fn,
            distance_fn=lambda: brain._forward_dist,
            sweep_fn=lambda: [],
            mileage_fn=lambda: brain._mileage,
            semantic_map=brain.semantic_map,
        )
        mapper.run()
    else:
        from dimos_lite.dashboard import set_vision_module, set_pico_hw
        set_vision_module(vision)
        set_pico_hw(hardware)
        print("[OS] Dashboard: http://localhost:8080")
        brain.start()


if __name__ == "__main__":
    main()
