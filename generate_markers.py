#!/usr/bin/env python3
"""Generate printable ArUco markers for each room in the apartment.

Usage:  python3 generate_markers.py
Output: markers/ directory with one PNG per room.
Print each at 15cm × 15cm and mount on a wall at robot camera height (~15cm).
"""

from dimos_lite.floorplan import ROOMS
from dimos_lite.aruco import generate_markers

room_markers = [
    (r["marker_id"], r["label"])
    for r in ROOMS
    if r["marker_id"] is not None
]

output = generate_markers(room_markers)
if output:
    print(f"\nGenerated {len(room_markers)} markers in {output}/")
    print("Print each at 15cm × 15cm on white paper.")
    print("Mount on a wall in each room at ~15cm height (robot eye level).")
