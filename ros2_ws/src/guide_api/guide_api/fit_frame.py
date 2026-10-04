"""Fit the app's floor-plan frame to the map frame from rooms present in both.

    python -m guide_api.fit_frame ASB          # from ros2_ws/src/guide_api

Reads ``guide_api/config/<building>_keyplan_rooms.json`` (room labels as the app draws
them: ``{"n", "lv", "x", "y"}``) and ``data/sfu_gis/sfu_rooms_<building>.geojson``
(``python -m sfu_map.fetch <building>``), matches rooms by number and level,
fits an affine transform, and writes it into ``guide_api/config/app_frames.json``.
Labels are not exactly at room centres, so the worst matches are dropped twice.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics

from .dev import CONFIG, repo_root
from .frames import fit
from .rooms import gis_rooms


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("building")
    ap.add_argument("--keep", type=float, default=0.85, help="share of matches kept at each refit")
    args = ap.parse_args(argv)
    b = args.building.upper()
    root = repo_root()
    frame = json.loads((root / "maps" / "route_graph.json").read_text())["frame"]
    gis = gis_rooms(root / "data" / "sfu_gis" / f"sfu_rooms_{b}.geojson", frame)
    app = json.loads((CONFIG / f"{b.lower()}_keyplan_rooms.json").read_text())["rooms"]

    pairs = []
    for r in app:
        g = gis.get(f"{b} {r['n']}")
        if g and g["floor"].split("-", 1)[1].lstrip("0") == str(r["lv"]):
            pairs.append(((r["x"], r["y"]), (g["room_x"], g["room_y"])))
    if len(pairs) < 3:
        print(f"only {len(pairs)} rooms matched: nothing fitted")
        return 1

    def errors(t, ps):
        return [math.hypot(t.apply(*a)[0] - m[0], t.apply(*a)[1] - m[1]) for a, m in ps]

    t = fit(pairs)
    for _ in range(2):
        e = errors(t, pairs)
        cut = sorted(e)[max(2, int(len(e) * args.keep) - 1)]
        pairs = [p for p, err in zip(pairs, e) if err <= cut]
        t = fit(pairs)
    e = errors(t, pairs)
    print(f"{b}: {len(pairs)} rooms, median error {statistics.median(e):.2f} m, worst {max(e):.2f} m, "
          f"{math.hypot(t.a, t.d):.4f} m per plan unit")

    out = CONFIG / "app_frames.json"
    doc = json.loads(out.read_text()) if out.is_file() else {"buildings": {}}
    doc["buildings"][b] = {"keyplan_to_map": [round(v, 6) for v in (t.a, t.b, t.c, t.d, t.e, t.f)],
                           "rooms_used": len(pairs), "median_error_m": round(statistics.median(e), 3)}
    out.write_text(json.dumps(doc, indent=2) + "\n")
    print(f"-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
