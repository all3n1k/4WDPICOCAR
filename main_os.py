"""
main_os.py — dimOS-lite entry point

Normal mode:        python3 main_os.py
Autonomous map:     python3 main_os.py --map
Cartographer pass:  python3 main_os.py --carto
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
    print("=" * 48 + "\n")

    loc = LocalizationModule()
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
        model="gemma4:31b",
        prior_map=prior_map,
        localization=loc,
        discovery_mode=CONSTELLATION_MODE,
    )
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
