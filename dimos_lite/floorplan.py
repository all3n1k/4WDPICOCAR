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
        "dims": "9'6\" \u00d7 10'",
    },
    {
        "id": "bathroom",
        "label": "Bath",
        "bounds": [270, 10, 110, 190],
        "marker_id": 3,
        "dims": "",
    },
    {
        "id": "bedroom",
        "label": "Bedroom",
        "bounds": [390, 10, 300, 350],
        "marker_id": 2,
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
        "dims": "13'5\" \u00d7 15'2\"",
    },
    {
        "id": "entry",
        "label": "Entry",
        "bounds": [10, 640, 120, 50],
        "marker_id": 4,
        "dims": "",
    },
]

MARKER_TO_ROOM = {r["marker_id"]: r for r in ROOMS if r["marker_id"] is not None}


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
