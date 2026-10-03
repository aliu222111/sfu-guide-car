"""Route queries over the built graph (spec section 8, steps 1 to 3).

Standard library only, so the ROS mission node can import this module without
the map pipeline's build dependencies.
"""

from __future__ import annotations

import heapq
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path


# Every door leg stops the car and asks the student to hold the door, so a route
# should not go out and back in, or through extra doorways, to save a few metres.
DOOR_PENALTY = 15.0  # metres-equivalent added per door edge
# A passage through a room the GIS calls an office or lab is a last resort.
THROUGH_ROOM_PENALTY = 60.0


class RouteError(Exception):
    pass


@dataclass
class Leg:
    """One stretch the mission node handles as a unit."""

    kind: str  # "drive", "elevator", "door", "stairs"
    floor: str | None  # None outdoors
    nodes: list[str]
    length: float


class RouteGraph:
    def __init__(self, graph: dict, destinations: dict | None = None):
        self.floors = graph.get("floors", {})
        self.nodes: dict[str, dict] = {n["id"]: n for n in graph["nodes"]}
        self.adj: dict[str, list[tuple[str, dict]]] = {nid: [] for nid in self.nodes}
        for e in graph["edges"]:
            self.adj[e["a"]].append((e["b"], e))
            self.adj[e["b"]].append((e["a"], e))
        self.destinations = destinations or {}

    @classmethod
    def load(cls, graph_path: Path | str, destinations_path: Path | str | None = None) -> "RouteGraph":
        graph_path = Path(graph_path)
        with open(graph_path) as f:
            graph = json.load(f)
        if destinations_path is None:
            cand = graph_path.with_name("destinations.json")
            destinations_path = cand if cand.exists() else None
        dest = None
        if destinations_path is not None:
            with open(destinations_path) as f:
                dest = json.load(f)
        return cls(graph, dest)

    # Resolving places -------------------------------------------------------

    def resolve(self, query: str) -> str:
        """Node id for a node id, a room such as ``AQ 3150`` or ``aq3150``, or ``entrance:<id>``."""
        q = query.strip()
        if q in self.nodes:
            return q
        if f"entrance:{q}" in self.nodes:
            return f"entrance:{q}"
        key = normalise_room(q)
        if key in self.destinations:
            return self.destinations[key]["node"]
        raise RouteError(f"unknown place {query!r}")

    def search(self, text: str, limit: int = 10) -> list[str]:
        """Destination keys matching free text, best first."""
        t = text.strip().upper().replace("-", " ")
        compact = t.replace(" ", "")
        hits = []
        for key, d in self.destinations.items():
            k = key.replace(" ", "")
            if k == compact:
                score = 0
            elif k.startswith(compact):
                score = 1
            elif compact in k or t in (d.get("name") or "").upper():
                score = 2
            else:
                continue
            hits.append((score, not d.get("searchable", False), key))
        return [k for *_, k in sorted(hits)[:limit]]

    def nearest_node(self, floor: str | None, x: float, y: float, types: set[str] | None = None) -> str:
        best, best_d = None, math.inf
        for nid, n in self.nodes.items():
            if n["floor"] != floor or (types and n["type"] not in types):
                continue
            d = math.hypot(n["x"] - x, n["y"] - y)
            if d < best_d:
                best, best_d = nid, d
        if best is None:
            raise RouteError(f"no node on floor {floor}")
        return best

    # Shortest path ------------------------------------------------------------

    def edge_allowed(self, e: dict, car: bool, verified_only: bool) -> bool:
        if car and not e["car_traversable"]:
            return False
        if verified_only and not e["verified"]:
            return False
        return True

    def shortest_path(self, start: str, goal: str, car: bool = True, verified_only: bool = False) -> tuple[list[str], float]:
        """Dijkstra on edge length. ``car`` excludes stairs and other non-traversable edges."""
        for n in (start, goal):
            if n not in self.nodes:
                raise RouteError(f"unknown node {n!r}")
        dist = {start: 0.0}
        prev: dict[str, str] = {}
        heap = [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                break
            if d > dist.get(u, math.inf):
                continue
            for v, e in self.adj[u]:
                if not self.edge_allowed(e, car, verified_only):
                    continue
                nd = d + e["length"] + (DOOR_PENALTY if e["type"] == "door" else 0.0) + (THROUGH_ROOM_PENALTY if e.get("through_room") else 0.0)
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))
        if goal not in dist:
            raise RouteError(f"no route from {start} to {goal}" + (" for the car" if car else ""))
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1], dist[goal]

    def edge(self, a: str, b: str) -> dict:
        for v, e in self.adj[a]:
            if v == b:
                return e
        raise RouteError(f"no edge {a} - {b}")

    def legs(self, path: list[str]) -> list[Leg]:
        """Split a path into drive legs on one floor or outdoors, and the transitions between them."""
        legs: list[Leg] = []
        for a, b in zip(path, path[1:]):
            e = self.edge(a, b)
            floor = self.nodes[b]["floor"]
            if e["type"] in ("elevator", "stairs", "door"):
                kind = e["type"]
            else:
                kind = "drive"
                if self.nodes[a]["floor"] != floor:
                    floor = None  # mixed: entrance to outdoor
            last = legs[-1] if legs else None
            if last and last.kind == kind == "drive" and last.floor == floor:
                last.nodes.append(b)
                last.length += e["length"]
            else:
                legs.append(Leg(kind, floor, [a, b], e["length"]))
        return legs


BUILDINGS = ("WMC", "MBC", "AQ")  # longest first
_ROOM_RE = re.compile(r"^\s*([A-Za-z]+?)[\s-]*([0-9A-Za-z][0-9A-Za-z.\-]*)\s*$")


def normalise_room(text: str) -> str:
    """``aq3150``, ``AQ-3150`` and ``AQ 3150`` all become ``AQ 3150``.

    The building is matched against the known codes first, so room ids that
    start with letters or contain a hyphen survive: ``wmcs1300`` -> ``WMC S1300``,
    ``AQ-5003-9`` -> ``AQ 5003-9``.
    """
    t = text.strip().upper()
    for b in BUILDINGS:
        if t.startswith(b):
            rest = t[len(b):].lstrip(" -")
            if rest:
                return f"{b} {rest}"
    m = _ROOM_RE.match(t)
    if not m:
        return t
    building, room = m.groups()
    return f"{building} {room}"


def describe(rg: RouteGraph, path: list[str], length: float) -> str:
    lines = [f"{len(path)} nodes, {length:.0f} m equivalent"]
    for leg in rg.legs(path):
        a, b = rg.nodes[leg.nodes[0]], rg.nodes[leg.nodes[-1]]
        where = leg.floor or "outdoors"
        if leg.kind == "drive":
            lines.append(f"  drive {leg.length:6.1f} m on {where}: {leg.nodes[0]} -> {leg.nodes[-1]} ({len(leg.nodes)} nodes)")
        elif leg.kind == "elevator":
            lines.append(f"  elevator {a['floor']} -> {b['floor']} ({a.get('elevator', '')}), ask the student to press the button")
        elif leg.kind == "door":
            lines.append(f"  door on {where} at {leg.nodes[0]}, ask the student to hold it")
        else:
            lines.append(f"  {leg.kind} {a['floor']} -> {b['floor']}")
    lines.append("nodes:")
    for nid in path:
        n = rg.nodes[nid]
        lines.append(f"  {nid:32s} {n['type']:18s} {str(n['floor']):8s} {n['x']:9.2f} {n['y']:9.2f}")
    return "\n".join(lines)
