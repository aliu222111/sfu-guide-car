"""Find people and campus objects in a recorded walk with the car's detector.

    py walks\\stereo_record\\yolo_walk.py aq3000_walk1 [--every 10] [--model school|coco]

Needs: pip install ultralytics (it brings torch; the first run downloads the model, about
30 MB, and its small text encoder from GitHub). Reads <base>_left.avi and <base>_times.csv,
looks at every --every'th frame (10 at 30 fps = 3 per second) and writes:
  <base>_yolo.csv   frame, unix_time, label, conf, box, x/y metres ahead/left (the car's nominal
                    camera: 28 cm up, 5 deg up, 70 deg lens; rough until the rig is calibrated)
  <base>_yolo.jpg   a contact sheet of the 12 frames with the most different things in them
                    (red: people, orange: obstacles, blue: parts of the building)
and prints how many of each thing it saw.
"""

from __future__ import annotations

import csv
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "guide_perception"))
from guide_perception.ground import mount_pose, nominal_K  # noqa: E402
from guide_perception.people import FLOOR_OBJECTS, SCHOOL_CLASSES, Person, carried, kind_of, locate  # noqa: E402


def load(kind: str):
    models = REPO / "models"
    models.mkdir(exist_ok=True)
    if kind == "coco":
        from ultralytics import YOLO
        return YOLO(str(models / "yolo11n.pt"))
    onnx = models / "yoloe-school.onnx"
    if onnx.exists():
        from ultralytics import YOLO
        return YOLO(str(onnx))
    from ultralytics import YOLOE
    m = YOLOE(str(models / "yoloe-11s-seg.pt"))
    m.set_classes(SCHOOL_CLASSES, m.get_text_pe(SCHOOL_CLASSES))
    return m


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__); return 1
    base = argv[0]
    every = int(next((argv[i + 1] for i, a in enumerate(argv) if a == "--every"), 10))
    kind = next((argv[i + 1] for i, a in enumerate(argv) if a == "--model"), "school")
    model = load(kind)
    times = np.loadtxt(f"{base}_times.csv", delimiter=",", skiprows=1)[:, 1]
    cap = cv2.VideoCapture(f"{base}_left.avi")
    K = nominal_K(640, 360, 70.0)
    R, t = mount_pose(0.50, 0.05, 0.28, -0.09, 0.0)
    counts, frames_with, rows, best = Counter(), Counter(), [], []
    n, t0 = 0, time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if n % every == 0:
            res = model.predict(frame, conf=0.3, imgsz=640, verbose=False)[0]
            found = []
            for xyxy, c, k in zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist(), res.boxes.cls.int().tolist()):
                found.append(Person(None, tuple(xyxy), float(c), label=res.names[k]))
            locate(found, K, R, t, frame.shape[0])
            people = [p for p in found if p.label == "person"]
            for p in found:
                if kind_of(p.label) == "obstacle" and carried(p, people):
                    p.label = f"{p.label} (with a person)"  # worn, held or sat in: not a floor obstacle
                counts[p.label] += 1
                rows.append([n, f"{times[min(n, len(times) - 1)]:.3f}", p.label, f"{p.conf:.2f}", *(f"{v:.0f}" for v in p.box),
                             f"{p.x:.2f}" if np.isfinite(p.x) else "", f"{p.y:.2f}" if np.isfinite(p.x) else ""])
            for lbl in {p.label for p in found}:
                frames_with[lbl] += 1
            kinds = len({p.label for p in found})
            if kinds:
                g = frame.copy()
                for p in found:
                    x1, y1, x2, y2 = map(int, p.box)
                    col = {"person": (0, 0, 255), "obstacle": (0, 170, 255), "building": (230, 120, 0)}.get(kind_of(p.label), (200, 200, 0))
                    cv2.rectangle(g, (x1, y1), (x2, y2), col, 2)
                    d = f" {p.distance:.1f}m" if np.isfinite(p.x) else ""
                    cv2.putText(g, f"{p.label}{d}", (x1 + 2, max(12, y1 - 4)), 0, 0.45, col, 1)
                cv2.putText(g, f"frame {n}  {(times[min(n, len(times) - 1)] - times[0]) / 60:.1f} min", (6, 352), 0, 0.45, (255, 255, 255), 1)
                best.append((kinds, sum(p.conf for p in found), n, cv2.resize(g, (320, 180))))
                best = sorted(best, key=lambda b: (-b[0], -b[1]))[:12]
        n += 1
        if n % 1500 == 0:
            print(f"  {n} frames, {time.time() - t0:.0f} s", flush=True)
    with open(f"{base}_yolo.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "unix_time", "label", "conf", "x1", "y1", "x2", "y2", "x_ahead_m", "y_left_m"])
        w.writerows(rows)
    if best:
        tiles = [b[3] for b in sorted(best, key=lambda b: b[2])]
        tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 3)
        sheet = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
        cv2.imwrite(f"{base}_yolo.jpg", sheet)
    looked = (n + every - 1) // every
    print(f"{base}: {n} frames, looked at {looked} ({time.time() - t0:.0f} s)")
    for lbl, c in counts.most_common():
        note = ("  (not mapped: the person stops the car)" if "(with a person)" in lbl else
                {"obstacle": "  (obstacle: goes in the costmap)", "building": "  (building landmark)"}.get(kind_of(lbl), ""))
        print(f"  {lbl:24s} {c:5d} detections, in {100 * frames_with[lbl] / max(looked, 1):4.1f}% of frames{note}")
    print(f"wrote {base}_yolo.csv and {base}_yolo.jpg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
