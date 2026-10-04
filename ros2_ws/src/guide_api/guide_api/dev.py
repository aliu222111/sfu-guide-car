"""Run the rider API against the fake car: no ROS, no car, any OS.

    python -m guide_api.dev                      # http://localhost:8090  (from ros2_ws/src/guide_api)
    python -m guide_api.dev --speedup 6 --door

Rooms come from ``maps/destinations.json`` plus every ``data/sfu_gis/sfu_rooms_*.geojson``
(so ASB works before its maps are built).
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from .fake_car import FakeCar
from .frames import AppFrames
from .chat import Concierge
from .http_api import serve
from .rooms import gis_rooms
from .service import ScoutService

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = Path(__file__).resolve().parent / "config"


def repo_root() -> Path:
    env = os.environ.get("GUIDE_CAR_ROOT")
    return Path(env) if env else PACKAGE.parents[2]


def load_rooms(root: Path) -> dict[str, dict]:
    rooms: dict[str, dict] = {}
    graph = root / "maps" / "route_graph.json"
    if graph.is_file():
        frame = json.loads(graph.read_text())["frame"]
        for f in sorted((root / "data" / "sfu_gis").glob("sfu_rooms_*.geojson")):
            rooms.update(gis_rooms(f, frame))
    dest = root / "maps" / "destinations.json"
    if dest.is_file():
        rooms.update(json.loads(dest.read_text()))  # the built maps win: their doors are real
    return rooms


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--speedup", type=float, default=4.0, help="how much faster than real time the fake car moves")
    ap.add_argument("--door", action="store_true", help="the fake car stops once for a door")
    ap.add_argument("--start", default="ASB 970.6", help="room the fake car starts at")
    ap.add_argument("--token", default=os.environ.get("SCOUT_TOKEN", ""))
    args = ap.parse_args(argv)

    root = repo_root()
    rooms = load_rooms(root)

    def locate(key):
        r = rooms.get(key)
        return (r["floor"], r["room_x"], r["room_y"]) if r and "room_x" in r else None

    car = FakeCar(locate, start=locate(args.start) or ("ASB-09", 0.0, 0.0), door=args.door)
    service = ScoutService(car, rooms, AppFrames.load(CONFIG / "app_frames.json"), car_id="fake-car", car_name="Scout (fake)")
    car.attach(service)
    chat = Concierge.from_env(rooms)
    serve(service, args.port, token=args.token, chat=chat)
    print("Chat with ElevenLabs: " + ("on" if chat else "off (set GEMINI_API_KEY and ELEVENLABS_API_KEY to turn it on)"))
    print(f"rider API with a fake car on http://localhost:{args.port}/  ({len(rooms)} rooms; Ctrl-C to stop)")
    try:
        last, since_tick = time.monotonic(), 0.0
        while True:
            time.sleep(0.2)
            now = time.monotonic()
            car.step((now - last) * args.speedup)
            since_tick += now - last
            last = now
            if since_tick >= 1.0:
                since_tick = 0.0
                service.tick()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
