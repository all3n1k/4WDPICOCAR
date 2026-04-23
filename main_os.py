"""
main_os.py — dimOS-lite entry point

Normal mode:  python3 main_os.py
Mapping mode: python3 main_os.py --map
"""

import sys
import os
import time
import threading

from dimos_lite.core import autoconnect
from dimos_lite.vision import VisionModule
from dimos_lite.pico import PicoHardwareModule
from dimos_lite.agent import AgentCoreModule
from dimos_lite.localization import LocalizationModule

MAP_FILE = os.path.join(os.path.dirname(__file__), 'apartment_map.py')
MAPPING_MODE = '--map' in sys.argv


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


def main():
    print("=" * 48)
    print("  Agentic Pet OS (dimOS-lite) v4.0")
    if MAPPING_MODE:
        print("  *** MAPPING MODE ***")
    print("=" * 48 + "\n")

    loc = LocalizationModule()
    loc.start()

    vision = VisionModule(stream_url="http://192.168.1.216:81/stream")
    hardware = PicoHardwareModule(ws_url="ws://192.168.1.217:8765")

    prior_map = load_prior_map()
    brain = AgentCoreModule(
        ollama_url="http://digitalstorm:11434/api/generate",
        model="gemma4:31b",
        prior_map=prior_map,
        localization=loc,
    )

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
        print("[OS] Dashboard: http://localhost:8080")
        brain.start()


if __name__ == "__main__":
    main()
