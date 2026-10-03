"""Hand-edited connectors: what the GIS does not encode (spec section 5).

See ``sfu_map/connectors.yaml`` for the format. Every entry carries
``verified``; it stays false until checked on site (spec section 12).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .geo import lonlat_to_xy

DEFAULT_CONNECTORS = Path(__file__).resolve().parents[2] / "connectors.yaml"

_POINT_KEYS = ("xy", "latlon", "shaft")


@dataclass
class Connectors:
    elevators: list[dict] = field(default_factory=list)
    entrances: list[dict] = field(default_factory=list)
    links: list[dict] = field(default_factory=list)
    outdoor_links: list[dict] = field(default_factory=list)
    doors: list[dict] = field(default_factory=list)

    def door_overrides(self) -> dict[str, dict]:
        out = {}
        for d in self.doors:
            if "xy" in d:
                x, y = d["xy"]
            else:
                lat, lon = d["latlon"]
                x, y = (float(v) for v in lonlat_to_xy(lon, lat))
            out[d["room"]] = {"x": x, "y": y}
        return out


def _check_point(entry: dict, where: str):
    if not any(k in entry for k in _POINT_KEYS):
        raise ValueError(f"{where}: needs one of {', '.join(_POINT_KEYS)}")


def load_connectors(path: Path | str | None = DEFAULT_CONNECTORS) -> Connectors:
    if path is None or not Path(path).exists():
        return Connectors()
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    c = Connectors(
        elevators=raw.get("elevators") or [],
        entrances=raw.get("entrances") or [],
        links=raw.get("links") or [],
        outdoor_links=raw.get("outdoor_links") or [],
        doors=raw.get("doors") or [],
    )
    ids = set()
    for kind in ("elevators", "entrances", "links", "outdoor_links"):
        for e in getattr(c, kind):
            if "id" not in e:
                raise ValueError(f"{kind}: entry without id: {e}")
            if (kind, e["id"]) in ids:
                raise ValueError(f"{kind}: duplicate id {e['id']}")
            ids.add((kind, e["id"]))
    for e in c.elevators:
        _check_point(e, f"elevator {e['id']}")
        if len(e.get("floors", [])) < 2:
            raise ValueError(f"elevator {e['id']}: needs at least two floors")
    for e in c.entrances:
        _check_point(e, f"entrance {e['id']}")
        if "floor" not in e:
            raise ValueError(f"entrance {e['id']}: needs floor")
    for e in c.links:
        for side in ("a", "b"):
            if side not in e or "floor" not in e[side]:
                raise ValueError(f"link {e['id']}: needs {side}.floor")
            _check_point(e[side], f"link {e['id']} end {side}")
    for e in c.outdoor_links:
        for k in ("from", "to"):
            if k not in e:
                raise ValueError(f"outdoor link {e['id']}: needs {k}")
    for d in c.doors:
        if "room" not in d or not ("xy" in d or "latlon" in d):
            raise ValueError(f"door override needs room and xy or latlon: {d}")
    return c
