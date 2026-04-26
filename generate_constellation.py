#!/usr/bin/env python3
"""Generate a constellation of small floor markers for high-precision SLAM.

Usage:  python3 generate_constellation.py
Output: markers/floor/ directory with 20 unique PNGs.
Print these at 5cm x 5cm and scatter them across your floor!
"""

import os
from dimos_lite.aruco import generate_markers

# IDs 10-30 for the floor constellation
floor_markers = [
    (i, f"Floor Marker {i}")
    for i in range(10, 31)
]

output_dir = "markers/floor"
os.makedirs(output_dir, exist_ok=True)

output = generate_markers(floor_markers, output_dir=output_dir)

if output:
    print(f"\n[Constellation] Generated {len(floor_markers)} markers in {output}/")
    print("Instruction:")
    print("1. Print these markers (size: ~5cm x 5cm).")
    print("2. Scatter them across the floors of your Living Room, Kitchen, etc.")
    print("3. They don't need to be perfectly aligned — Dimos will map them himself!")
