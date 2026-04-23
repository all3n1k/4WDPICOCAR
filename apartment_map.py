"""
apartment_map.py — Imported from floorplan.png
Generated via dimos_lite/importer.py
"""

def make_row(s): return list(s)

APARTMENT_PRIOR = [
    make_row("WWWWWWWWWWWWWWWWWWWWW"),
    make_row("W  WW  W  WWW   WW  W"),
    make_row("W  W   W            W"),
    make_row("W  W     W W        W"),
    make_row("W                   W"),
    make_row("W  W                W"),
    make_row("W      W            W"),
    make_row("W      W            W"),
    make_row("W  W   W            W"),
    make_row("W  W                W"),
    make_row("W  W                W"),
    make_row("W                W  W"),
    make_row("W                   W"),
    make_row("W                   W"),
    make_row("W  WW               W"),
    make_row("W                   W"),
    make_row("W    W              W"),
    make_row("W    W              W"),
    make_row("W    W              W"),
    make_row("W    W  WW       W  W"),
    make_row("WWWWWWWWWWWWWWWWWWWWW"),
]

assert len(APARTMENT_PRIOR) == 21
assert all(len(r) == 21 for r in APARTMENT_PRIOR)
