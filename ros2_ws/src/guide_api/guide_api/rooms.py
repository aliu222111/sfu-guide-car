"""Rooms straight from an SFU GIS file, for a building whose maps are not built yet (no ROS here).

``python -m sfu_map.fetch ASB`` writes ``data/sfu_gis/sfu_rooms_ASB.geojson``.
Until ``sfu-map build`` has turned that into floor grids and a route graph,
this gives the fake car and the frame fit each room's floor and centre in the
map frame.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


def to_map(frame: dict, lon: float, lat: float) -> tuple[float, float]:
    """Longitude/latitude -> the map frame of route_graph.json (ENU metres, equirectangular about its origin)."""
    r = frame.get("earth_radius_m", 6378137.0)
    return (math.radians(lon - frame["origin_lon"]) * r * math.cos(math.radians(frame["origin_lat"])),
            math.radians(lat - frame["origin_lat"]) * r)


def _centroid(points: list[tuple[float, float]]) -> tuple[float, float]:
    a = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        c = x0 * y1 - x1 * y0
        a += c; cx += (x0 + x1) * c; cy += (y0 + y1) * c
    if abs(a) < 1e-9:
        return points[0]
    return cx / (3 * a), cy / (3 * a)


def gis_rooms(geojson: Path | str, frame: dict) -> dict[str, dict]:
    """``{"ASB 9896": {"floor": "ASB-09", "x", "y", "room", "type", "name", "floor_name", "searchable"}}``"""
    doc = json.loads(Path(geojson).read_text())
    out: dict[str, dict] = {}
    for f in doc.get("features", []):
        p, g = f.get("properties") or {}, f.get("geometry")
        room, building = str(p.get("rm_id") or "").strip(), str(p.get("bl_abbr") or "").strip().upper()
        if not g or not room or not building:
            continue
        rings = g["coordinates"] if g["type"] == "Polygon" else [r for poly in g["coordinates"] for r in poly]
        ring = max(rings, key=len)
        x, y = _centroid([to_map(frame, pt[0], pt[1]) for pt in ring])
        out[f"{building} {room}"] = {
            "building": building, "room": room, "floor": f"{building}-{p.get('fl_id', '')}", "floor_name": p.get("fl_name", ""),
            "type": p.get("rm_type", ""), "name": p.get("rm_name", "") or "", "searchable": p.get("searchable") == "Y",
            "room_x": round(x, 2), "room_y": round(y, 2),
        }
    return out
