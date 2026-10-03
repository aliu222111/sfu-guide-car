"""Load the committed SFU GIS room polygons and group them by building floor."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from shapely.geometry import shape
from shapely.geometry.base import BaseGeometry
from shapely.ops import transform, unary_union

from .geo import shapely_lonlat_to_xy

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROOMS = REPO_ROOT / "data" / "sfu_gis" / "sfu_rooms_AQ_WMC_MBC.geojson"
DEFAULT_CAMPUS = REPO_ROOT / "data" / "sfu_gis" / "campus-geometry.json"

# Room categories the car may drive through.
FREE_CATEGORIES = {"CIRCULATION"}
# Public rooms the car may also cross: office suites and clubs are often
# reached through a reception, lounge or cafeteria that the GIS does not
# classify as circulation. Doors into them are bridged like any other doorway
# (the car stops and asks the student to hold the door).
PASSABLE_TYPES = {
    "Lobby", "Reception", "Public Lounge", "Public Study Area", "Student Lounge/Lunch Room",
    "Gallery, Display Room", "Dining", "Cafeteria", "Vending Machine Area",
}
# Of those, room types that are not floor space.
NOT_FREE_TYPES = {"Stairs", "Elevator Shaft"}
# Rooms that get no door node.
NO_DOOR_CATEGORIES = {"CIRCULATION", "PENETRATIONS"}

# Snap tolerance when merging adjacent corridor polygons, metres. GIS corridor
# polygons that share an edge are off by a few millimetres; walls are 15 cm or
# more, so 5 cm closes the seams without bridging any wall.
SEAM_TOLERANCE = 0.05


@dataclass
class Room:
    building: str
    floor_id: str
    rm_id: str
    rm_type: str
    rm_cat: str
    rm_name: str
    searchable: bool
    geom: BaseGeometry  # map frame, metres

    @property
    def is_free(self) -> bool:
        if self.rm_type in NOT_FREE_TYPES:
            return False
        return self.rm_cat in FREE_CATEGORIES or self.rm_type in PASSABLE_TYPES

    @property
    def gets_door(self) -> bool:
        return bool(self.rm_id) and self.rm_cat not in NO_DOOR_CATEGORIES


@dataclass
class Floor:
    building: str
    fl_id: str
    name: str
    ref_lvl: int  # building-relative level order from the GIS
    rooms: list[Room] = field(default_factory=list)
    _free: BaseGeometry | None = None

    @property
    def id(self) -> str:
        return floor_id(self.building, self.fl_id)

    @property
    def free_space(self) -> BaseGeometry:
        """Union of drivable polygons with GIS seams closed."""
        if self._free is None:
            polys = [r.geom for r in self.rooms if r.is_free]
            u = unary_union(polys)
            self._free = u.buffer(SEAM_TOLERANCE, join_style="mitre").buffer(
                -SEAM_TOLERANCE, join_style="mitre"
            )
        return self._free

    @property
    def footprint(self) -> BaseGeometry:
        return unary_union([r.geom for r in self.rooms])


def floor_id(building: str, fl_id: str) -> str:
    return f"{building}-{fl_id}"


def load_floors(path: Path | str = DEFAULT_ROOMS) -> dict[str, Floor]:
    """Return floors keyed by floor id (for example ``AQ-03``), in level order."""
    with open(path) as f:
        data = json.load(f)
    floors: dict[str, Floor] = {}
    for feat in data["features"]:
        p = feat["properties"]
        fid = floor_id(p["bl_abbr"], p["fl_id"])
        fl = floors.get(fid)
        if fl is None:
            fl = floors[fid] = Floor(p["bl_abbr"], p["fl_id"], p["fl_name"], int(p["ref_lvl"]))
        geom = transform(shapely_lonlat_to_xy, shape(feat["geometry"]))
        if not geom.is_valid:
            geom = geom.buffer(0)
        fl.rooms.append(
            Room(
                building=p["bl_abbr"],
                floor_id=fid,
                rm_id=(p["rm_id"] or "").strip(),
                rm_type=p["rm_type"] or "",
                rm_cat=p["rm_cat"] or "",
                rm_name=(p["rm_name"] or "").strip(),
                searchable=p.get("searchable") == "Y",
                geom=geom,
            )
        )
    return dict(sorted(floors.items(), key=lambda kv: (kv[1].building, kv[1].ref_lvl)))


def load_building_footprints(path: Path | str = DEFAULT_CAMPUS) -> dict[str, BaseGeometry]:
    """Building footprints from the student-project snapshot, in the map frame.

    The snapshot stores y as screen-down (south positive); it is flipped here.
    """
    from shapely.geometry import Polygon

    with open(path) as f:
        data = json.load(f)
    out = {}
    for b in data["buildings"]:
        polys = []
        for rings in b["polygons"]:
            ext = [(x, -y) for x, y in rings[0]]
            holes = [[(x, -y) for x, y in r] for r in rings[1:]]
            poly = Polygon(ext, holes)
            polys.append(poly if poly.is_valid else poly.buffer(0))
        out[b["id"]] = unary_union(polys)
    return out
