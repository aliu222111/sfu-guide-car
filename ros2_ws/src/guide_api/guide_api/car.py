"""Run the rider API with the straight-driving car (ESP32 + L298N), no ROS.

    python -m guide_api.car --port COM5                      # Windows; /dev/ttyUSB0 on Linux
    python -m guide_api.car --port COM5 --jog 3              # bench test: forward for 3 s, then stop
    python -m guide_api.car --port COM5 --speed 0.62         # after calibrating (see README)
    python -m guide_api.car                                  # no board: the logic only

Then open http://<this laptop>:8090 on a phone on the same WiFi.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

from .frames import AppFrames
from .chat import Concierge
from .http_api import serve
from .planner import Planner
from .service import ScoutService
from .straight_car import NullTransport, SerialTransport, StraightCar, drive_line

CONFIG = Path(__file__).resolve().parent / "config"
HZ = 20.0


def jog(transport, seconds: float, throttle: float) -> None:
    """Drive straight for ``seconds`` at ``throttle``, then stop. Wheels off the floor the first time."""
    print(f"driving at throttle {throttle} for {seconds} s ...")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        transport.write(drive_line(throttle))
        time.sleep(1.0 / HZ)
    for _ in range(5):
        transport.write(drive_line(0.0))
        time.sleep(1.0 / HZ)
    print("stopped. Measure how far it went: speed = metres / seconds.")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="", help="the ESP32's serial port (COM5, /dev/ttyUSB0); leave out to run without a board")
    ap.add_argument("--http", type=int, default=8090, help="port for the rider API")
    ap.add_argument("--start", default="ASB 970.7", help="the room the car is standing at when this starts")
    ap.add_argument("--heading", type=float, default=None, help="which way it faces, degrees (0 east, 90 north); leave out and it asks to be pointed")
    ap.add_argument("--throttle", type=float, default=1.0, help="throttle while driving, 0..1 (1 is the firmware's MAX_DUTY, 120 of 255)")
    ap.add_argument("--speed", type=float, default=0.5, help="metres per second the car does at that throttle (calibrate with --jog)")
    ap.add_argument("--jog", type=float, default=0.0, metavar="SECONDS", help="only drive straight this long, then exit")
    ap.add_argument("--token", default=os.environ.get("SCOUT_TOKEN", ""))
    args = ap.parse_args(argv)

    if args.port:
        transport = SerialTransport(args.port)
        time.sleep(2.0)  # opening the port resets an ESP32: let it boot
        print(f"board on {args.port}: {transport.read().decode('ascii', errors='replace').strip() or '(no greeting yet)'}")
    else:
        transport = NullTransport()
        print("no --port: running without a motor board")
    if args.jog > 0:
        jog(transport, args.jog, args.throttle)
        return 0

    planner = Planner(CONFIG / "asb_plan.json", AppFrames.load(CONFIG / "app_frames.json").to_map["ASB"])
    start = planner.locate(args.start.upper())
    if start is None:
        print(f"{args.start} is not a room on the plan")
        return 1
    car = StraightCar(planner, transport, start=start, heading=None if args.heading is None else math.radians(args.heading),
                      throttle=args.throttle, speed=args.speed)
    service = ScoutService(car, planner.destinations(), AppFrames.load(CONFIG / "app_frames.json"),
                           speed=args.speed, car_id="scout-01", car_name="Scout")
    car.attach(service)
    chat = Concierge.from_env(planner.destinations())
    serve(service, args.http, token=args.token, chat=chat)
    print("Chat with ElevenLabs: " + ("on" if chat else "off (set GEMINI_API_KEY and ELEVENLABS_API_KEY to turn it on)"))
    print(f"car at {args.start}; rider API on http://localhost:{args.http}/  (Ctrl-C to stop)")
    try:
        last, since_tick = time.monotonic(), 0.0
        while True:
            time.sleep(1.0 / HZ)
            now = time.monotonic()
            car.step(now - last)
            since_tick += now - last
            last = now
            if since_tick >= 1.0:
                since_tick = 0.0
                service.tick()
    except KeyboardInterrupt:
        for _ in range(3):
            transport.write(drive_line(0.0))
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
