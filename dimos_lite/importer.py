"""
dimos_lite/importer.py — Converts a 2D floor plan image to the robot's grid.

This script reads a floor plan image (PNG/JPG), identifies walls (dark pixels),
and downsamples it into the 21x21 'W' and ' ' grid used by apartment_map.py.
"""

import cv2
import numpy as np
import os
import sys

# Constants
GRID_SIZE = 21
MAP_FILE = os.path.join(os.path.dirname(__file__), '..', 'apartment_map.py')

def import_floorplan(image_path):
    if not os.path.exists(image_path):
        print(f"[Importer] ✗ File not found: {image_path}")
        return

    # 1. Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"[Importer] ✗ Could not read image: {image_path}")
        return

    # 2. Pre-process
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Invert if the image has a dark background with light walls
    # (Most LiDAR apps export black walls on white background)
    # We want black = Wall (0), white = Open (255)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    
    # 3. Downsample to GRID_SIZE x GRID_SIZE
    # We use INTER_AREA for better downsampling of thin lines
    resized = cv2.resize(thresh, (GRID_SIZE, GRID_SIZE), interpolation=cv2.INTER_AREA)
    
    # 4. Convert to ASCII
    # Stricter threshold: Only very dark pixels are walls (0-150)
    # This prevents 'gray' antialiased pixels from becoming 10cm blocks
    grid = []
    for r in range(GRID_SIZE):
        row = []
        for c in range(GRID_SIZE):
            if resized[r, c] < 150: 
                row.append('W')
            else:
                row.append(' ')
        grid.append(row)

    # 5. Add a 1-cell perimeter of 'W' for safety
    for c in range(GRID_SIZE):
        grid[0][c] = 'W'
        grid[GRID_SIZE-1][c] = 'W'
    for r in range(GRID_SIZE):
        grid[r][0] = 'W'
        grid[r][GRID_SIZE-1] = 'W'

    # 6. Ensure the robot start area is open (3x3 clearing)
    mid = GRID_SIZE // 2
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            grid[mid + dr][mid + dc] = ' '

    # 7. Write to apartment_map.py
    with open(MAP_FILE, 'w') as f:
        f.write('"""\n')
        f.write(f'apartment_map.py — Imported from {os.path.basename(image_path)}\n')
        f.write('Generated via dimos_lite/importer.py\n')
        f.write('"""\n\n')
        f.write('def make_row(s): return list(s)\n\n')
        f.write('APARTMENT_PRIOR = [\n')
        for row in grid:
            line = "".join(row)
            f.write(f'    make_row("{line}"),\n')
        f.write(']\n\n')
        f.write(f'assert len(APARTMENT_PRIOR) == {GRID_SIZE}\n')
        f.write(f'assert all(len(r) == {GRID_SIZE} for r in APARTMENT_PRIOR)\n')

    print(f"[Importer] ✓ Successfully converted {image_path} to {MAP_FILE}")
    print(f"[Importer] Grid: {GRID_SIZE}x{GRID_SIZE}")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "floorplan.png"
    import_floorplan(path)
