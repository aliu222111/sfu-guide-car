"""Data the phone app needs, prepared from the built maps (no ROS here)."""

from __future__ import annotations

import json
from pathlib import Path


class MapData:
    def __init__(self, maps_dir: Path | str):
        maps_dir = Path(maps_dir)
        graph = json.loads((maps_dir / "route_graph.json").read_text())
        self.destinations = json.loads((maps_dir / "destinations.json").read_text())
        self.floors = graph.get("floors", {})
        nodes = {n["id"]: n for n in graph["nodes"]}
        self.floor_edges: dict[str, list] = {}
        for e in graph["edges"]:
            a, b = nodes[e["a"]], nodes[e["b"]]
            if a["floor"] and a["floor"] == b["floor"] and e["type"] in ("corridor", "door"):
                self.floor_edges.setdefault(a["floor"], []).append([a["x"], a["y"], b["x"], b["y"]])

    def destination_list(self) -> list[dict]:
        out = []
        for key, d in self.destinations.items():
            out.append(
                {
                    "key": key,
                    "type": d.get("type", ""),
                    "name": d.get("name", ""),
                    "floor": d.get("floor_name", ""),
                    "searchable": d.get("searchable", False),
                }
            )
        # Classrooms and searchable rooms first: what students look for.
        out.sort(key=lambda d: (d["type"] not in ("Classroom", "Lecture Theatre", "Seminar"), not d["searchable"], d["key"]))
        return out

    def floor(self, floor_id: str) -> dict | None:
        if floor_id not in self.floor_edges:
            return None
        edges = self.floor_edges[floor_id]
        xs = [v for e in edges for v in (e[0], e[2])]
        ys = [v for e in edges for v in (e[1], e[3])]
        meta = self.floors.get(floor_id, {})
        return {
            "id": floor_id,
            "name": f"{meta.get('building', '')} {meta.get('name', '')}".strip(),
            "bounds": [min(xs), min(ys), max(xs), max(ys)],
            "edges": edges,
        }
