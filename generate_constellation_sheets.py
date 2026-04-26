#!/usr/bin/env python3
"""Generate printable constellation sheets (10 markers per page).

Usage:  python3 generate_constellation_sheets.py
Output: markers/sheets/sheet_1.png and sheet_2.png
Print these at A4/Letter size, then cut out the 5cm x 5cm markers.
"""

import cv2
import os
import numpy as np

MARKER_DIR = "markers/floor"
SHEET_DIR = "markers/sheets"
os.makedirs(SHEET_DIR, exist_ok=True)

# 1. Collect all generated markers
marker_files = sorted([f for f in os.listdir(MARKER_DIR) if f.endswith(".png")])
if not marker_files:
    print(f"[Error] No markers found in {MARKER_DIR}. Run generate_constellation.py first.")
    exit(1)

def create_sheet(files, sheet_num):
    images = []
    for f in files:
        img = cv2.imread(os.path.join(MARKER_DIR, f))
        # Add a light grey border for easier cutting
        img = cv2.copyMakeBorder(img, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[240, 240, 240])
        images.append(img)

    # Pad with empty white images if we have fewer than 10
    while len(images) < 10:
        images.append(np.ones_like(images[0]) * 255)

    # Arrange into 4 columns x 5 rows (fits all 20 on one sheet!)
    rows = []
    for i in range(0, 20, 4):
        # Handle index out of bounds just in case
        chunk = []
        for j in range(4):
            if i + j < len(images):
                chunk.append(images[i+j])
            else:
                chunk.append(np.ones_like(images[0]) * 255)
        
        row = np.hstack(chunk)
        rows.append(row)
    
    sheet = np.vstack(rows)
    
    out_path = os.path.join(SHEET_DIR, "constellation_master_sheet.png")
    cv2.imwrite(out_path, sheet)
    print(f"[Sheet] Saved {out_path}")

# Generate one master sheet for all 20 markers
create_sheet(marker_files[:20], 1)

print("\nSuccess!")
print(f"Print the PNGs in {SHEET_DIR}/ at 'Fill Page' (A4 or Letter).")
print("Each marker will be approximately 5cm x 5cm.")
