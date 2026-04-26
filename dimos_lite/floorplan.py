"""
Programmatic apartment floor plan derived from floorplan.png dimensions.

Coordinate system: values in abstract units (proportional to cm).
Origin at top-left corner of apartment. X increases right, Y increases down.
"""

APARTMENT_W = 700
APARTMENT_H = 700

ROOMS = [
    {
        "id": "kitchen",
        "label": "Kitchen",
        "bounds": [10, 10, 250, 290],
        "marker_id": 1,
        "marker_pos": [250, 310], # Kitchen marker is by the door now
        "dims": "9'6\" \u00d7 10'",
    },
    {
        "id": "bathroom",
        "label": "Bath",
        "bounds": [270, 10, 110, 190],
        "marker_id": 3,
        "marker_pos": [325, 10], # North wall center of bath
        "dims": "",
    },
    {
        "id": "bedroom",
        "label": "Bedroom",
        "bounds": [390, 10, 300, 350],
        "marker_id": 2,
        "marker_pos": [540, 10], # North wall center of bedroom
        "dims": "12'1\" \u00d7 13'10\"",
    },
    {
        "id": "hallway",
        "label": "Hall",
        "bounds": [270, 210, 110, 90],
        "marker_id": None,
        "dims": "",
    },
    {
        "id": "living_room",
        "label": "Living Room",
        "bounds": [10, 310, 680, 320],
        "marker_id": 0,
        "marker_pos": [350, 630],
        "dims": "13'5\" \u00d7 15'2\"",
    },
    {
        "id": "entry",
        "label": "Entry",
        "bounds": [10, 640, 120, 50],
        "marker_id": 4,
        "marker_pos": [70, 690], # South wall center of entry
        "dims": "",
    },
]

MARKER_TO_ROOM = {r["marker_id"]: r for r in ROOMS if r["marker_id"] is not None}


def _apply_room_marker_survey(path="room_markers.json"):
    """Overlay surveyed wall-marker positions and facing directions from disk.

    The dashboard's marker survey (M0) writes physical (x, y, facing_deg) for
    markers 0-4 to room_markers.json. This loader applies those to MARKER_TO_ROOM
    so update_aruco fuses against real positions, and marker_facing_deg is
    available for the heading-correction rewrite in M3.
    """
    import json, os
    if not os.path.exists(path):
        return
    try:
        with open(path) as f:
            survey = json.load(f)
    except Exception as e:
        print(f"[floorplan] room_markers.json load error: {e}")
        return
    applied = 0
    for mid_str, entry in survey.items():
        mid = int(mid_str)
        room = MARKER_TO_ROOM.get(mid)
        if room is None:
            continue
        room["marker_pos"] = [float(entry["x"]), float(entry["y"])]
        room["marker_facing_deg"] = float(entry.get("facing_deg", 0))
        applied += 1
    if applied:
        print(f"[floorplan] Applied surveyed positions for {applied} wall marker(s)")


_apply_room_marker_survey()


def room_center(room):
    x, y, w, h = room["bounds"]
    return (x + w / 2, y + h / 2)


def room_by_marker_id(marker_id):
    return MARKER_TO_ROOM.get(marker_id)


def room_at_point(px, py):
    for room in ROOMS:
        x, y, w, h = room["bounds"]
        if x <= px <= x + w and y <= py <= y + h:
            return room
    return None
