"""Download a building's room polygons from SFU's public ArcGIS server (spec section 5).

    python -m sfu_map.fetch ASB            # writes data/sfu_gis/sfu_rooms_ASB.geojson
    python -m sfu_map.fetch ASB --floor 9000

Standard library only, so it runs on any laptop with Python (no shapely). The
Room Finder layer has every room on campus with the same fields as the AQ, WMC
and MBC file; ``sfu-map build`` loads every ``sfu_rooms_*.geojson`` in data/sfu_gis.
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.parse
import urllib.request
from pathlib import Path

LAYER = "https://viewsfu.its.sfu.ca/fsgis/rest/services/RoomFinder/RoomFinder2024_RoomDisplay/MapServer/0/query"
OUT_DIR = Path(__file__).resolve().parents[3] / "data" / "sfu_gis"
PAGE = 1000


def fetch(building: str, floor: str = "", timeout: float = 60.0) -> dict:
    where = f"bl_abbr='{building.upper()}'" + (f" AND fl_name LIKE '{floor}%'" if floor else "")
    features, offset = [], 0
    while True:
        query = urllib.parse.urlencode({
            "where": where, "outFields": "*", "outSR": "4326", "f": "geojson",
            "orderByFields": "OBJECTID", "resultOffset": offset, "resultRecordCount": PAGE,
        })
        with urllib.request.urlopen(f"{LAYER}?{query}", timeout=timeout) as r:
            page = json.load(r)
        if "error" in page:
            raise RuntimeError(page["error"])
        got = page.get("features", [])
        features += got
        print(f"  {len(features)} rooms", file=sys.stderr)
        if len(got) < PAGE and not page.get("exceededTransferLimit"):
            break
        offset += len(got)
    return {"type": "FeatureCollection", "features": features}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("building", help="SFU building code, e.g. ASB")
    ap.add_argument("--floor", default="", help='only this level, e.g. "9000"')
    args = ap.parse_args(argv)
    doc = fetch(args.building, args.floor)
    if not doc["features"]:
        print(f"No rooms for {args.building}: check the building code (SFU Room Finder uses ASB, TASC1, ...)", file=sys.stderr)
        return 1
    levels = sorted({f["properties"]["fl_name"] for f in doc["features"]})
    out = OUT_DIR / f"sfu_rooms_{args.building.upper()}.geojson"
    out.write_text(json.dumps(doc))
    print(f"{len(doc['features'])} rooms on {', '.join(levels)} -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
