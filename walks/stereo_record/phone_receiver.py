"""Save Sensor Logger's live HTTP push on the walking laptop, beside stereo_record.py.

    py walks\\stereo_record\\phone_receiver.py walks\\phone\\walk1      # then start stereo_record.py

In Sensor Logger: Settings, Data Streaming, HTTP Push, URL http://<laptop IPv4>:8765/data
(ipconfig; on the iPhone hotspot it is 172.20.10.x), push on, then Start Recording.
Writes Gyroscope.csv, Accelerometer.csv, Gravity.csv, ... in Sensor Logger's own export
layout (time in ns, x, y, z), so clean_walk.py reads the folder the same way, and
clock_offset.txt: laptop clock minus phone clock, measured from when each push arrives,
which clean_walk.py uses instead of guessing. Ctrl+C to stop. Standard library only.
"""

from __future__ import annotations

import csv
import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock

FIELDS = {"location": ["latitude", "longitude", "altitude", "horizontalAccuracy", "verticalAccuracy", "speed", "bearing"],
          "orientation": ["qx", "qy", "qz", "qw", "roll", "pitch", "yaw"]}


class Store:
    def __init__(self, folder: Path):
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        self.files, self.lock = {}, Lock()
        self.offsets, self.samples = [], 0

    def writer(self, name: str, values: dict):
        if name not in self.files:
            cols = FIELDS.get(name.lower(), [k for k in ("x", "y", "z") if k in values] or sorted(values))
            f = open(self.folder / f"{name[0].upper() + name[1:]}.csv", "w", newline="")
            w = csv.writer(f)
            w.writerow(["time", *cols])
            self.files[name] = (f, w, cols)
        return self.files[name]

    def add(self, doc, received: float):
        newest = 0
        with self.lock:
            for item in doc.get("payload", []) if isinstance(doc, dict) else []:
                try:
                    name, t, v = str(item["name"]), int(item["time"]), item["values"]
                except (KeyError, TypeError, ValueError):
                    continue
                if not isinstance(v, dict):
                    continue
                f, w, cols = self.writer(name, v)
                w.writerow([t, *(v.get(c, "") for c in cols)])
                newest = max(newest, t)
                self.samples += 1
            for f, _, _ in self.files.values():
                f.flush()  # on disk after every push: a crash or a closed window loses nothing
            if newest:
                # The push left the phone after its newest sample: received - newest is the offset
                # plus the network delay. The smallest of these is the best estimate.
                self.offsets.append(received - newest / 1e9)
                (self.folder / "clock_offset.txt").write_text(f"{min(self.offsets):.4f}\n")

    def close(self):
        for f, _, _ in self.files.values():
            f.close()


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    folder = Path(argv[0] if argv else time.strftime("walks/phone/%Y%m%d%H%M%S"))
    port = int(argv[1]) if len(argv) > 1 else 8765
    store = Store(folder)

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            received = time.time()
            try:
                body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
                store.add(json.loads(body), received)
                self.send_response(200)
            except ValueError:
                self.send_response(400)
            self.end_headers()

        def log_message(self, *a):
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"saving to {folder}; Sensor Logger HTTP Push to http://<this laptop's IPv4>:{port}/data. Ctrl+C to stop")
    last = 0
    try:
        server.timeout = 1.0
        while True:
            server.handle_request()
            if store.samples != last and time.time() % 5 < 1:
                off = f", clock offset {min(store.offsets):+.3f} s" if store.offsets else ""
                print(f"  {store.samples} samples{off}", flush=True)
                last = store.samples
    except KeyboardInterrupt:
        pass
    store.close()
    print(f"{store.samples} samples saved in {folder}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
