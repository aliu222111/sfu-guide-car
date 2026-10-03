"""Draft connector entries for a person to verify on site.

The GIS records each elevator shaft on one floor only. A floor is proposed as
a stop when the shaft's footprint is mostly uncovered by rooms on that floor
(it is the same shaft) and a corridor runs next to it (there is a landing).
"""

from __future__ import annotations

from shapely.ops import unary_union

from .gisdata import Floor

MAX_COVER = 0.3
MAX_CORRIDOR_GAP = 1.0


def elevator_candidates(floors: dict[str, Floor]) -> list[dict]:
    out = []
    seen = set()
    corridors = {fid: unary_union([r.geom for r in fl.rooms if r.rm_cat == "CIRCULATION"]) for fid, fl in floors.items()}
    for fl in floors.values():
        for shaft in fl.rooms:
            if shaft.rm_type != "Elevator Shaft":
                continue
            c = shaft.geom.centroid
            key = (fl.building, round(c.x), round(c.y))
            if key in seen:  # the same shaft drawn on two floors (WMC)
                continue
            seen.add(key)
            stops = []
            for fid, other in floors.items():
                if other.building != fl.building:
                    continue
                covered = sum(
                    shaft.geom.intersection(r.geom).area
                    for r in other.rooms
                    if r is not shaft and r.rm_type != "Elevator Shaft"
                )
                if covered / shaft.geom.area > MAX_COVER:
                    continue
                corr = corridors[fid]
                if corr.is_empty or shaft.geom.distance(corr) > MAX_CORRIDOR_GAP:
                    continue
                stops.append((other.ref_lvl, fid))
            if len(stops) < 2:
                continue
            out.append(
                {
                    "id": f"{fl.building}-{shaft.rm_id}",
                    "shaft": f"{fl.id}/{shaft.rm_id}",
                    "floors": [fid for _, fid in sorted(stops)],
                    "verified": False,
                }
            )
    return sorted(out, key=lambda e: e["id"])
