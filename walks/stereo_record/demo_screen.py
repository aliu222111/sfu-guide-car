"""Live "how our software works" screen for the judges: one 1920x1080 window, four big panels.

    py walks\\stereo_record\\demo_screen.py --left 3 --right 2 --rear 1
    py walks\\stereo_record\\demo_screen.py --left-file walks\\stereo_record\\test1_left.avi --right-file walks\\stereo_record\\test1_right.avi

Needs only: pip install opencv-python numpy ultralytics   (no ROS)

  FRONT LEFT   YOLO boxes (label, confidence, distance from stereo) + tracked feature points with trails
  FRONT RIGHT  YOLO boxes + the stereo depth (disparity) picture in the corner
  REAR         YOLO boxes ("no rear camera" when there is none)
  POINT MAP    top-down view: stereo depth points accumulated with the visual-odometry pose,
               the path driven, the car and its field of view, detected things where they are

How it works: the pair is rectified with the calibration in ros2_ws/src/guide_bringup/scripts/calib
(left.yaml / right.yaml), StereoSGBM gives disparity at half resolution, depth = fx * baseline / disparity.
Visual odometry tracks ~300 corners on the left picture (Lucas-Kanade), gives each a 3D position
from the previous frame's disparity and solves the camera motion with PnP + RANSAC. YOLO runs on a
background thread taking the three cameras in turn, so the screen stays smooth on a laptop CPU.

Keys: q / Esc quit, f fullscreen, space pause, r reset map, + / - zoom the map.
Replays loop (the map restarts each loop). For testing without a screen:
    --headless --record out.mp4 --frames 300 --snap shot   (writes shot_0060.jpg, ...)
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CALIB = REPO / "ros2_ws" / "src" / "guide_bringup" / "scripts" / "calib"
W, H = 640, 360                      # camera pictures
SCREEN_W, SCREEN_H = 1920, 1080      # the canvas
HEADER_H = 64
PANEL_W, PANEL_H = SCREEN_W // 2, (SCREEN_H - HEADER_H) // 2   # 960 x 508


def rgb(r, g, b):
    return (b, g, r)


BG = rgb(14, 16, 20)
PANEL_BG = rgb(22, 25, 31)
LINE = rgb(48, 54, 64)
TEXT = rgb(236, 239, 244)
MUTED = rgb(150, 158, 170)
ACCENT = rgb(204, 6, 51)             # SFU red
OK_C = rgb(64, 214, 120)
BAD_C = rgb(255, 82, 82)
FEAT_C = rgb(80, 235, 120)
FEAT_OUT = rgb(120, 130, 140)
TRAJ_C = rgb(0, 200, 255)
SCAN_C = rgb(90, 220, 255)

# person plus things that are in the car's way on campus
DEFAULT_CLASSES = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "traffic light", "fire hydrant",
                   "stop sign", "bench", "dog", "backpack", "umbrella", "handbag", "suitcase", "chair", "couch",
                   "potted plant", "dining table", "tv", "laptop", "skateboard", "bottle", "cup"]
CLASS_COLORS = {
    "person": rgb(255, 72, 96), "bicycle": rgb(255, 170, 0), "car": rgb(255, 120, 40), "motorcycle": rgb(255, 120, 40),
    "bus": rgb(255, 120, 40), "truck": rgb(255, 120, 40), "bench": rgb(80, 190, 255), "chair": rgb(120, 150, 255),
    "couch": rgb(120, 150, 255), "dining table": rgb(255, 214, 70), "potted plant": rgb(170, 230, 80),
    "backpack": rgb(200, 120, 255), "handbag": rgb(200, 120, 255), "suitcase": rgb(200, 120, 255),
    "umbrella": rgb(200, 120, 255), "dog": rgb(255, 150, 200), "laptop": rgb(100, 230, 220), "tv": rgb(100, 230, 220),
}


def class_color(name):
    if name in CLASS_COLORS:
        return CLASS_COLORS[name]
    h = sum(ord(c) * 31 ** i for i, c in enumerate(name)) % 180
    hsv = np.uint8([[[h, 170, 255]]])
    return tuple(int(v) for v in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


# ---------------------------------------------------------------- text (TrueType through Pillow when there is one)
class Text:
    """Anti-aliased text: each string is rendered once with Pillow (ultralytics brings it) and blended in.
    Falls back to OpenCV's Hershey font."""

    REG = ["segoeui.ttf", "C:/Windows/Fonts/segoeui.ttf", "arial.ttf", "C:/Windows/Fonts/arial.ttf",
           "DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
           "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", "/System/Library/Fonts/Helvetica.ttc"]
    BOLD = ["segoeuib.ttf", "C:/Windows/Fonts/segoeuib.ttf", "arialbd.ttf", "C:/Windows/Fonts/arialbd.ttf",
            "DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"]

    def __init__(self):
        self.cache = OrderedDict()
        self.fonts = {}
        try:
            from PIL import Image, ImageDraw, ImageFont
            self.Image, self.ImageDraw, self.ImageFont = Image, ImageDraw, ImageFont
            self.pil = self._font(20, False) is not None
        except Exception:
            self.pil = False

    def _font(self, size, bold):
        key = (size, bold)
        if key not in self.fonts:
            f = None
            for name in (self.BOLD if bold else []) + self.REG:
                try:
                    f = self.ImageFont.truetype(name, size); break
                except Exception:
                    continue
            self.fonts[key] = f
        return self.fonts[key]

    def _patch(self, text, size, bold):
        key = (text, size, bold)
        p = self.cache.get(key)
        if p is not None:
            self.cache.move_to_end(key); return p
        f = self._font(size, bold)
        asc, desc = f.getmetrics()
        w = max(1, int(math.ceil(f.getlength(text)))) + 2
        h = asc + desc
        im = self.Image.new("L", (w, h), 0)
        self.ImageDraw.Draw(im).text((1, 0), text, font=f, fill=255)
        p = np.asarray(im, np.float32)[..., None] / 255.0
        self.cache[key] = p
        if len(self.cache) > 3000:
            self.cache.popitem(last=False)
        return p

    def size(self, text, size, bold=False):
        if self.pil:
            p = self._patch(text, size, bold); return p.shape[1], p.shape[0]
        s = size / 30.0
        (w, h), b = cv2.getTextSize(self._ascii(text), cv2.FONT_HERSHEY_DUPLEX, s, 2 if bold else 1)
        return w, h + b

    @staticmethod
    def _ascii(t):
        return t.replace("\u2014", "-").replace("\u00b7", "|").replace("\u00b0", " deg").encode("ascii", "replace").decode()

    def put(self, img, text, x, y, size, color=TEXT, bold=False, align="left"):
        """Draws with (x, y) the top-left corner (align='right': top-right, 'center': top-centre). Returns width."""
        w, h = self.size(text, size, bold)
        x = int(x - w) if align == "right" else int(x - w // 2) if align == "center" else int(x)
        y = int(y)
        if not self.pil:
            s = size / 30.0
            cv2.putText(img, self._ascii(text), (x, y + int(size * 0.8)), cv2.FONT_HERSHEY_DUPLEX, s, color,
                        2 if bold else 1, cv2.LINE_AA)
            return w
        p = self._patch(text, size, bold)
        x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + w, img.shape[1]), min(y + h, img.shape[0])
        if x1 <= x0 or y1 <= y0:
            return w
        a = p[y0 - y:y1 - y, x0 - x:x1 - x]
        roi = img[y0:y1, x0:x1].astype(np.float32)
        img[y0:y1, x0:x1] = (roi * (1 - a) + np.float32(color) * a).astype(np.uint8)
        return w


def shade(img, x0, y0, x1, y1, color=(0, 0, 0), alpha=0.6):
    x0, y0 = max(int(x0), 0), max(int(y0), 0)
    x1, y1 = min(int(x1), img.shape[1]), min(int(y1), img.shape[0])
    if x1 > x0 and y1 > y0:
        roi = img[y0:y1, x0:x1]
        if color == (0, 0, 0):
            img[y0:y1, x0:x1] = cv2.convertScaleAbs(roi, alpha=1 - alpha)
        else:
            img[y0:y1, x0:x1] = cv2.addWeighted(roi, 1 - alpha, np.full_like(roi, color), alpha, 0)


def chip(img, T, text, x, y, size=20, fg=TEXT, bg=(0, 0, 0), alpha=0.62, bold=True, pad=8, dot=None):
    """A label on a translucent rounded-ish strip. Returns its width."""
    w, h = T.size(text, size, bold)
    extra = (size // 2 + 8) if dot is not None else 0
    shade(img, x, y, x + w + 2 * pad + extra, y + h + pad, bg, alpha)
    if dot is not None:
        cv2.circle(img, (int(x + pad + size // 4), int(y + (h + pad) / 2)), max(size // 4, 4), dot, -1, cv2.LINE_AA)
    T.put(img, text, x + pad + extra, y + pad // 2, size, fg, bold)
    return w + 2 * pad + extra


# ---------------------------------------------------------------- calibration
def read_camera_yaml(path):
    """K, D, R, P from a ROS camera_info YAML (rows/cols/data blocks)."""
    t = Path(path).read_text()

    def block(key, shape):
        m = re.search(key + r"\s*:.*?data\s*:\s*\[([^\]]*)\]", t, re.S)
        return np.array([float(v) for v in m.group(1).replace("\n", " ").split(",")], np.float64).reshape(shape)
    return block("camera_matrix", (3, 3)), block("distortion_coefficients", (1, -1)), \
        block("rectification_matrix", (3, 3)), block("projection_matrix", (3, 4))


def nominal_K(hfov_deg=70.0, w=W, h=H):
    f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)


class Calibration:
    def __init__(self, left_yaml, right_yaml):
        self.ok = False
        try:
            self.Kl, self.Dl, self.Rl, self.Pl = read_camera_yaml(left_yaml)
            self.Kr, self.Dr, self.Rr, self.Pr = read_camera_yaml(right_yaml)
            self.maps_l = cv2.initUndistortRectifyMap(self.Kl, self.Dl, self.Rl, self.Pl[:3, :3], (W, H), cv2.CV_32FC1)
            self.maps_r = cv2.initUndistortRectifyMap(self.Kr, self.Dr, self.Rr, self.Pr[:3, :3], (W, H), cv2.CV_32FC1)
            self.K_rect = self.Pl[:3, :3].copy()
            self.baseline = -self.Pr[0, 3] / self.Pr[0, 0]
            self.ok = self.baseline > 0.01
        except Exception as e:
            print(f"calibration not loaded ({e}): nominal 70 deg lenses, 0.126 m baseline, no rectification")
        if not self.ok:
            self.Kl = nominal_K(); self.baseline = 0.126

    def rectify(self, l, r):
        return (cv2.remap(l, *self.maps_l, cv2.INTER_LINEAR), cv2.remap(r, *self.maps_r, cv2.INTER_LINEAR))


def match_stats(a, b, orb, bf):
    """Median horizontal shift (left minus right) and median |vertical| error of ORB matches."""
    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY) if a.ndim == 3 else a
    gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY) if b.ndim == 3 else b
    ka, da = orb.detectAndCompute(ga, None)
    kb, db = orb.detectAndCompute(gb, None)
    if da is None or db is None or len(ka) < 20 or len(kb) < 20:
        return None
    m = bf.match(da, db)
    if len(m) < 20:
        return None
    pa = np.float32([ka[x.queryIdx].pt for x in m]); pb = np.float32([kb[x.trainIdx].pt for x in m])
    d = pa - pb
    keep = np.abs(d[:, 1]) < 25
    if keep.sum() < 15:
        return None
    return float(np.median(d[keep, 0])), float(np.median(np.abs(d[keep, 1])))


def choose_geometry(pairs, calib, force_swap, force_rect):
    """Picks camera order (median disparity must be positive) and whether the calibration fits this rig
    (rectified rows line up better than raw ones). Returns (swap, rectify, note)."""
    orb = cv2.ORB_create(1500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    raw = [s for s in (match_stats(a, b, orb, bf) for a, b in pairs) if s]
    notes = []
    swap = False
    if force_swap is not None:
        swap = force_swap
    elif raw:
        dx = float(np.median([s[0] for s in raw]))
        swap = dx < 0
        if swap:
            notes.append(f"median disparity {dx:+.1f} px is negative: left/right swapped automatically")
    rect = calib.ok
    if force_rect is not None:
        rect = force_rect and calib.ok
    elif calib.ok and raw:
        rs = []
        for a, b in pairs:
            if swap:
                a, b = b, a
            ra, rb = calib.rectify(a, b)
            s = match_stats(ra, rb, orb, bf)
            if s:
                rs.append(s)
        dy_raw = float(np.median([s[1] for s in raw]))
        dy_rect = float(np.median([s[1] for s in rs])) if rs else 99.0
        if dy_rect > dy_raw + 1.5:
            rect = False
            notes.append(f"calibration does not fit this footage (rows off {dy_rect:.1f} px rectified vs "
                         f"{dy_raw:.1f} px raw): using unrectified pictures, depth is approximate")
        else:
            notes.append(f"rectified with calibration (row error {dy_rect:.1f} px, raw {dy_raw:.1f} px)")
    return swap, rect, notes


# ---------------------------------------------------------------- sources
def open_cam(i, fps=30):
    c = cv2.VideoCapture(i, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(i)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, W); c.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    c.set(cv2.CAP_PROP_FPS, fps)
    return c


def fit(img):
    if img is None:
        return None
    return img if img.shape[1] == W and img.shape[0] == H else cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


class StereoSource:
    """Live pair (grab both, then decode: taken a few ms apart) or two recorded files (looping)."""

    def __init__(self, args):
        self.live = args.left_file is None
        self.swap = False
        if self.live:
            self.caps = [open_cam(args.left), open_cam(args.right)]
            self.name = f"LIVE  cams {args.left}/{args.right}"
            self.fps = 30.0
        else:
            self.caps = [cv2.VideoCapture(args.left_file), cv2.VideoCapture(args.right_file)]
            self.name = "REPLAY  " + Path(args.left_file).stem.replace("_left", "")
            self.fps = self.caps[0].get(cv2.CAP_PROP_FPS) or 15.0
            self.count = int(self.caps[0].get(cv2.CAP_PROP_FRAME_COUNT))
        if not all(c.isOpened() for c in self.caps):
            sys.exit("could not open the stereo " + ("cameras" if self.live else "files") +
                     " (run walks\\stereo_record\\stereo_test.py with no arguments to list cameras)")

    def peek(self, n=8):
        """A few pairs to decide camera order and whether the calibration fits."""
        out = []
        if self.live:
            for _ in range(n):
                ok, l, r = self.read()
                if ok:
                    out.append((l, r))
                time.sleep(0.05)
        else:
            for k in range(n):
                pos = int((k + 0.5) * self.count / n)
                for c in self.caps:
                    c.set(cv2.CAP_PROP_POS_FRAMES, pos)
                ok, l, r = self.read()
                if ok:
                    out.append((l, r))
            self.rewind()
        return out

    def rewind(self):
        for c in self.caps:
            c.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def read(self):
        a, b = self.caps
        g1 = a.grab(); g2 = b.grab()
        ok1, l = a.retrieve() if g1 else (False, None)
        ok2, r = b.retrieve() if g2 else (False, None)
        if not (ok1 and ok2):
            return False, None, None
        l, r = fit(l), fit(r)
        return (True, r, l) if self.swap else (True, l, r)

    def release(self):
        for c in self.caps:
            c.release()


class RearSource:
    """Rear camera: live on its own thread (latest picture, never blocks the pair), or a file read with the pair."""

    def __init__(self, args):
        self.cap = None; self.live = False; self.frame = None; self.t = 0.0; self.label = ""
        self.lock = threading.Lock(); self.stop = False
        if args.rear_file:
            self.cap = cv2.VideoCapture(args.rear_file)
            self.label = Path(args.rear_file).name
        elif args.rear is not None and args.left_file is None:
            self.cap = open_cam(args.rear); self.live = True
            self.label = f"cam {args.rear}"
            threading.Thread(target=self._run, daemon=True).start()
        if self.cap is not None and not self.cap.isOpened():
            print("rear camera could not be opened: carrying on without it")
            self.cap = None

    def _run(self):
        while not self.stop and self.cap is not None:
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.t = fit(f), time.time()
            else:
                time.sleep(0.05)

    def read(self):
        if self.cap is None:
            return None
        if self.live:
            with self.lock:
                return self.frame if time.time() - self.t < 1.5 else None
        ok, f = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, f = self.cap.read()
        return fit(f) if ok else None

    def rewind(self):
        if self.cap is not None and not self.live:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)


# ---------------------------------------------------------------- YOLO on a background thread
class Detector(threading.Thread):
    CAMS = ("left", "right", "rear")

    def __init__(self, model_path, all_classes, conf, threads=0):
        super().__init__(daemon=True)
        self.threads = threads or max(1, (os.cpu_count() or 4) - 2)   # leave cores for depth + drawing
        self.ok = False; self.status = "loading detector..."; self.model = None; self.stopping = False
        self.conf = conf; self.classes = None; self.names = {}
        self.lock = threading.Lock()
        self.frames = {}     # cam -> (seq, picture)
        self.results = {}    # cam -> dict(seq, xyxy, conf, cls, t, ms)
        self.rate = {c: deque(maxlen=10) for c in self.CAMS}
        self.label = Path(model_path).stem
        if self.label.lower().startswith("yolo"):
            self.label = "YOLO" + self.label[4:]
        self.all_classes = all_classes
        self.model_path = Path(model_path)
        downloadable = self.model_path.parent == Path(".") and self.model_path.suffix == ".pt"  # yolo11s.pt
        if not self.model_path.exists() and not downloadable:
            self.label = "no detector"
            self.status = f"no detector: {self.model_path} not found (--model)"; return
        try:
            from ultralytics import YOLO
        except Exception:
            self.label = "no detector"
            self.status = "no detector: pip install ultralytics"; return
        try:
            self.model = YOLO(str(self.model_path), task="detect")
        except Exception as e:
            self.label = "no detector"
            self.status = f"no detector: {e}"[:90]; return
        self.start()

    def run(self):
        try:
            self.model.predict(np.zeros((H, W, 3), np.uint8), verbose=False)   # load + warm up
            self.names = dict(self.model.names)
            self._limit_threads()
            if not self.all_classes:
                self.classes = [i for i, n in self.names.items() if n in DEFAULT_CLASSES] or None
            self.ok = True; self.status = ""
        except Exception as e:
            self.status = f"no detector: {e}"[:90]; return
        done = {c: -1 for c in self.CAMS}
        turn = 0
        while not self.stopping:
            pick = None
            with self.lock:
                for k in range(3):
                    cam = self.CAMS[(turn + k) % 3]
                    f = self.frames.get(cam)
                    if f is not None and f[0] != done[cam]:
                        pick = (cam, f[0], f[1]); turn = (turn + k + 1) % 3; break
            if pick is None:
                time.sleep(0.004); continue
            cam, seq, img = pick
            t0 = time.time()
            try:
                r = self.model.predict(img, conf=self.conf, classes=self.classes, verbose=False, imgsz=640)[0]
                b = r.boxes
                res = dict(seq=seq, xyxy=b.xyxy.cpu().numpy(), conf=b.conf.cpu().numpy(),
                           cls=b.cls.cpu().numpy().astype(int), t=time.time(), ms=(time.time() - t0) * 1000)
            except Exception as e:
                self.status = f"detector error: {e}"[:90]; time.sleep(0.5); continue
            done[cam] = seq
            with self.lock:
                self.results[cam] = res
                self.rate[cam].append(time.time())

    def _limit_threads(self):
        """ONNX Runtime takes every core by default, which starves the stereo + drawing loop: rebuild its
        session with fewer threads (best effort: ultralytics internals differ between versions)."""
        try:
            import onnxruntime as ort
            b = self.model.predictor.model
            holder = getattr(b, "backend", None) or b
            sess = getattr(holder, "session", None)
            if not isinstance(sess, ort.InferenceSession):
                return
            so = ort.SessionOptions()
            so.intra_op_num_threads = self.threads; so.inter_op_num_threads = 1
            holder.session = ort.InferenceSession(str(self.model_path), so, providers=sess.get_providers())
            self.model.predict(np.zeros((H, W, 3), np.uint8), verbose=False)
        except Exception as e:
            print("note: could not limit detector threads:", e)

    def stop(self):
        self.stopping = True
        if self.is_alive():
            self.join(timeout=3.0)

    def submit(self, cam, seq, img):
        if self.model is None:
            return
        with self.lock:
            self.frames[cam] = (seq, img)

    def drop(self, cam):
        with self.lock:
            self.frames.pop(cam, None); self.results.pop(cam, None)

    def get(self, cam):
        with self.lock:
            r = self.results.get(cam)
            ts = list(self.rate[cam])
        if r is None or time.time() - r["t"] > 2.0:
            return None, 0.0
        hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 and ts[-1] > ts[0] else 0.0
        return r, hz


# ---------------------------------------------------------------- stereo depth + visual odometry + map
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], np.float64)


class OccupancyGrid:
    """Bounded top-down map: hit counts in 10 cm cells over 160 x 160 m around the start."""

    def __init__(self, res=0.10, n=1600):
        self.res, self.n = res, n
        self.counts = np.zeros((n, n), np.float32)

    def reset(self):
        self.counts[:] = 0

    def add(self, x, z):
        ix = np.floor(x / self.res).astype(np.int64) + self.n // 2
        iz = np.floor(z / self.res).astype(np.int64) + self.n // 2
        ok = (ix >= 0) & (ix < self.n) & (iz >= 0) & (iz < self.n)
        flat = np.unique(iz[ok] * self.n + ix[ok])           # each cell counts once per frame
        self.counts.ravel()[flat] += 1.0
        np.minimum(self.counts, 60.0, out=self.counts)

    def cells(self):
        return int(np.count_nonzero(self.counts >= 2))


class StereoVO:
    def __init__(self, calib, rectify, args):
        self.calib, self.rectify = calib, rectify
        self.K = calib.K_rect if rectify else calib.Kl
        self.fx, self.fy, self.cx, self.cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        self.B = calib.baseline
        self.fxB = self.fx * self.B
        self.pitch = math.radians(args.pitch)
        self.cam_h = args.cam_height
        self.max_range = args.max_range
        self.ndisp = 64                                    # at half resolution: nearest ~0.8 m
        bs = 5
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=self.ndisp, blockSize=bs,
                                          P1=8 * bs * bs, P2=32 * bs * bs, disp12MaxDiff=1, uniquenessRatio=10,
                                          speckleWindowSize=60, speckleRange=2, preFilterCap=31,
                                          mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        hs, ws = H // 2, W // 2
        vv, uu = np.mgrid[0:hs:2, 0:ws:2]
        self.samp_v, self.samp_u = vv.ravel(), uu.ravel()
        self.grid = OccupancyGrid()
        self.reset()

    def reset(self):
        self.T = np.eye(4); self.T[:3, :3] = rot_x(self.pitch)   # world = level frame at the start
        self.traj = deque([(0.0, 0.0)], maxlen=20000)
        self.dist = 0.0
        self.prev_gray = None; self.prev_disp = None
        self.pts = np.zeros((0, 2), np.float32); self.tracks = []; self.inlier = np.zeros(0, bool)
        self.status = "starting"; self.n_inliers = 0; self.lost_frames = 0
        self.scan = np.zeros((0, 2), np.float32)
        self.grid.reset()

    # disparity is kept at half resolution, in full-resolution pixels
    def disp_at(self, disp, pts):
        u = np.clip((pts[:, 0] / 2).astype(int), 0, disp.shape[1] - 1)
        v = np.clip((pts[:, 1] / 2).astype(int), 0, disp.shape[0] - 1)
        return disp[v, u]

    def to_cam(self, u, v, d):
        z = self.fxB / d
        return np.stack([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z], 1)

    def heading(self):
        f = self.T[:3, 2]
        return math.atan2(f[0], f[2])

    def process(self, left, right):
        if self.rectify:
            left, right = self.calib.rectify(left, right)
        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY); gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        hl = cv2.resize(gl, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
        hr = cv2.resize(gr, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
        d16 = self.sgbm.compute(hl, hr)
        disp = d16.astype(np.float32) * (2.0 / 16.0)       # full-resolution pixels
        disp[d16 <= 8] = 0.0                                # < 0.5 px at half res: no match / too far
        self.disp = disp
        self._odometry(gl)
        self._map()
        self.prev_gray, self.prev_disp = gl, disp
        return left, right

    def _odometry(self, gray):
        if self.prev_gray is not None and len(self.pts) >= 8:
            p0 = self.pts.reshape(-1, 1, 2)
            lk = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None, **lk)
            pb, st2, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev_gray, p1, None, **lk)
            good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((pb - p0).reshape(-1, 2), axis=1) < 1.0)
            p1 = p1.reshape(-1, 2)
            good &= (p1[:, 0] >= 0) & (p1[:, 0] < W) & (p1[:, 1] >= 0) & (p1[:, 1] < H)
            d = self.disp_at(self.prev_disp, self.pts)
            use = good & (d > 1.0) & (self.fxB / np.maximum(d, 1e-3) < 25.0)
            moved = False
            inl_mask = np.zeros(len(self.pts), bool)
            if use.sum() >= 12:
                obj = self.to_cam(self.pts[use, 0], self.pts[use, 1], d[use])
                ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, p1[use].astype(np.float64), self.K, None,
                                                         iterationsCount=100, reprojectionError=2.0,
                                                         confidence=0.99)
                if ok and inl is not None and len(inl) >= 12:
                    idx = np.flatnonzero(use)[inl.ravel()]
                    inl_mask[idx] = True
                    R, _ = cv2.Rodrigues(rvec)
                    t = tvec.ravel()
                    step = float(np.linalg.norm(t))
                    ang = float(np.linalg.norm(rvec))
                    if step < 0.5 and ang < math.radians(20):
                        self.n_inliers = len(inl)
                        if step > 0.004 or ang > math.radians(0.2):   # sit still when nothing moves
                            Tcp = np.eye(4); Tcp[:3, :3] = R; Tcp[:3, 3] = t
                            self.T = self.T @ np.linalg.inv(Tcp)
                            self.dist += step
                            p = self.T[:3, 3]
                            if math.hypot(p[0] - self.traj[-1][0], p[2] - self.traj[-1][1]) > 0.02:
                                self.traj.append((p[0], p[2]))
                        moved = True
            if moved:
                self.status = "tracking"; self.lost_frames = 0
            else:
                self.lost_frames += 1
                self.n_inliers = 0
                if self.lost_frames >= 2:
                    self.status = "lost"
            self.pts = p1[good].astype(np.float32)
            self.inlier = inl_mask[good]
            self.tracks = [tr for tr, g in zip(self.tracks, good) if g]
            for tr, p in zip(self.tracks, self.pts):
                tr.append((float(p[0]), float(p[1])))
        elif self.prev_gray is not None:
            self.status = "lost"
        # top up the corners
        want = 300
        if len(self.pts) < want * 0.75:
            mask = np.full(gray.shape, 255, np.uint8)
            mask[gray < 8] = 0                       # black border left by rectification
            for p in self.pts:
                cv2.circle(mask, (int(p[0]), int(p[1])), 10, 0, -1)
            new = cv2.goodFeaturesToTrack(gray, maxCorners=want - len(self.pts), qualityLevel=0.01,
                                          minDistance=10, blockSize=7, mask=mask)
            if new is not None:
                new = new.reshape(-1, 2).astype(np.float32)
                self.pts = np.vstack([self.pts, new])
                self.inlier = np.concatenate([self.inlier, np.zeros(len(new), bool)])
                self.tracks += [deque([(float(p[0]), float(p[1]))], maxlen=5) for p in new]

    def _map(self):
        d = self.disp[self.samp_v, self.samp_u]
        ok = d > 1.0
        col = self.samp_u[ok]
        u = col * 2.0 + 0.5; v = self.samp_v[ok] * 2.0 + 0.5; d = d[ok]
        pc = self.to_cam(u, v, d)
        pw = pc @ self.T[:3, :3].T + self.T[:3, 3]
        height = self.cam_h - pw[:, 1]                     # world y points down
        keep = (pc[:, 2] > 0.4) & (pc[:, 2] < self.max_range) & (height > 0.12) & (height < 2.0)
        pw, col, z = pw[keep], col[keep], pc[keep, 2]
        if len(pw):
            # a "virtual laser scan": per picture column, the nearest thing at obstacle height (its 3rd nearest
            # sample, so one stray match does not count) - the map then shows outlines, not filled cones
            order = np.lexsort((z, col))
            col_s = col[order]
            first = np.flatnonzero(np.r_[True, col_s[1:] != col_s[:-1]])
            cnt = np.diff(np.r_[first, len(col_s)])
            pick = order[np.minimum(first + 2, first + cnt - 1)][cnt >= 3]
            pw = pw[pick]
        self.grid.add(pw[:, 0], pw[:, 2])
        self.scan = pw[:, [0, 2]].astype(np.float32)

    def locate_box(self, box):
        """World (x, z) and range of a detection on the left picture, from the disparity inside its lower middle."""
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        u0, u1 = int((x1 + 0.25 * w) / 2), int((x2 - 0.25 * w) / 2) + 1
        v0, v1 = int((y1 + 0.35 * h) / 2), int((y2 - 0.05 * h) / 2) + 1
        patch = self.disp[max(v0, 0):max(v1, 1), max(u0, 0):max(u1, 1)]
        vals = patch[patch > 1.0]
        if vals.size < 8:
            return None
        dd = float(np.median(vals))
        z = self.fxB / dd
        if z > 30:
            return None
        pc = self.to_cam(np.array([(x1 + x2) / 2]), np.array([(y1 + y2) / 2]), np.array([dd]))[0]
        pw = self.T[:3, :3] @ pc + self.T[:3, 3]
        return float(pw[0]), float(pw[2]), float(np.hypot(pc[0], pc[2]))


# ---------------------------------------------------------------- drawing
class Screen:
    def __init__(self, T, det, vo, args, source_name, notes):
        self.T, self.det, self.vo, self.args = T, det, vo, args
        self.source_name, self.notes = source_name, notes
        self.scale = args.map_scale                       # pixels per metre
        self.objects = {"left": [], "rear": []}
        self.Kr = nominal_K(70.0)
        ramp = np.arange(255, -1, -1, dtype=np.uint8).reshape(-1, 1)      # 0 -> red (near), 255 -> blue (far)
        self.lut = cv2.applyColorMap(ramp, getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)).reshape(256, 3)
        self.bg_canvas = np.empty((SCREEN_H, SCREEN_W, 3), np.uint8); self.bg_canvas[:] = BG
        self.bg_rear = np.empty((PANEL_H, PANEL_W, 3), np.uint8); self.bg_rear[:] = PANEL_BG
        # No rear camera: the point map takes the whole bottom row instead of an empty panel.
        self.has_rear = args.rear is not None or args.rear_file is not None
        self.map_w = PANEL_W if self.has_rear else SCREEN_W
        self.bg_map = np.empty((PANEL_H, self.map_w, 3), np.uint8); self.bg_map[:] = rgb(12, 18, 26)

    # camera panel: 640x360 picture filling 960 wide, cropped top and bottom to the panel height
    def cam_panel(self, img):
        s = PANEL_W / W
        big = cv2.resize(img, (PANEL_W, int(H * s)), interpolation=cv2.INTER_LINEAR)
        off = (big.shape[0] - PANEL_H) // 2
        return big[off:off + PANEL_H].copy(), s, off

    def draw_boxes(self, p, res, s, off, ranges=None):
        if res is None:
            return 0
        items = []
        for k, (b, c, k_cls) in enumerate(zip(res["xyxy"], res["conf"], res["cls"])):
            name = self.det.names.get(int(k_cls), str(k_cls))
            col = class_color(name)
            x1, y1, x2, y2 = b[0] * s, b[1] * s - off, b[2] * s, b[3] * s - off
            cv2.rectangle(p, (int(x1), int(y1)), (int(x2), int(y2)), col, 3, cv2.LINE_AA)
            txt = f"{name} {c * 100:.0f}%"
            if ranges is not None and ranges[k] is not None:
                txt += f"  {ranges[k]:.1f} m"
            items.append((c, txt, col, x1, y1))
        placed = []
        for c, txt, col, x1, y1 in sorted(items, key=lambda it: -it[0]):     # most confident gets its spot
            tw, th = self.T.size(txt, 19, True)
            lx = min(max(x1 - 1, 0), PANEL_W - tw - 14)
            ty = min(max(y1 - th - 6, 52), PANEL_H - 2 * th - 40)   # clear of the title and the stats line
            for _ in range(8):                                       # not on top of another label
                if not any(lx < qx + qw and qx < lx + tw + 13 and ty < qy + th + 8 and qy < ty + th + 8
                           for qx, qy, qw in placed):
                    break
                ty = ty - (th + 9) if ty - (th + 9) >= 52 else ty + (th + 9) * 2
            placed.append((lx, ty, tw + 13))
            cv2.rectangle(p, (int(lx), int(ty)), (int(lx + tw + 13), int(ty + th + 6)), col, -1)
            self.T.put(p, txt, lx + 6, ty + 3, 19, rgb(12, 14, 18), True)
        return len(items)

    def panel_title(self, p, title, sub=None):
        x = 14
        w = chip(p, self.T, title, x, 12, 22, bg=(0, 0, 0), alpha=0.66)
        if sub:
            chip(p, self.T, sub, x + w + 8, 12, 22, fg=MUTED, bold=False, alpha=0.5)

    def panel_stats(self, p, text):
        w, h = self.T.size(text, 18)
        chip(p, self.T, text, 14, PANEL_H - h - 22, 18, fg=TEXT, bold=False, alpha=0.6)

    def det_line(self, cam, n):
        if self.det.model is None:
            return "detector off"
        if not self.det.ok:
            return self.det.status or "loading detector..."
        r, hz = self.det.get(cam)
        if r is None:
            return "waiting for detector"
        return f"{n} object{'s' if n != 1 else ''}   \u00b7   {self.det.label} {r['ms']:.0f} ms, {hz:.1f} Hz on this camera"

    def left_panel(self, img):
        p, s, off = self.cam_panel(img)
        vo = self.vo
        for tr, inl in zip(vo.tracks, vo.inlier):
            col = FEAT_C if inl else FEAT_OUT
            if len(tr) > 1:
                pts = np.int32([[x * s, y * s - off] for x, y in tr])
                cv2.polylines(p, [pts], False, col, 2, cv2.LINE_AA)
            x, y = tr[-1]
            cv2.circle(p, (int(x * s), int(y * s - off)), 4 if inl else 3, col, -1, cv2.LINE_AA)
        r, _ = self.det.get("left") if self.det.ok else (None, 0)
        ranges = None
        if r is not None:
            ranges = []
            for b in r["xyxy"]:
                loc = vo.locate_box(b)
                ranges.append(loc[2] if loc else None)
        n = self.draw_boxes(p, r, s, off, ranges)
        self.panel_title(p, f"FRONT LEFT  \u00b7  {self.det.label}", "+ visual-odometry features")
        self.panel_stats(p, self.det_line("left", n) +
                         f"   \u00b7   {len(vo.pts)} features, {int(vo.inlier.sum())} agree on motion")
        return p, n

    def right_panel(self, img):
        p, s, off = self.cam_panel(img)
        r, _ = self.det.get("right") if self.det.ok else (None, 0)
        # stereo depth inset (first, so the detection labels stay on top of it)
        d = self.vo.disp
        z = self.vo.fxB / np.maximum(d, 1e-3)              # colour by distance: 0.8 m red .. 10 m blue
        u8 = np.clip(255.0 * np.log(z / 0.8) / math.log(10.0 / 0.8), 0, 255).astype(np.uint8)
        u8[d <= 0] = 255
        iw, ih = 352, 198
        cm = cv2.resize(self.lut[u8], (iw, ih), interpolation=cv2.INTER_NEAREST)
        cm[cv2.resize((d <= 0).astype(np.uint8), (iw, ih), interpolation=cv2.INTER_NEAREST) > 0] = (24, 24, 24)
        x0, y0 = PANEL_W - iw - 14, 14 + 32
        cv2.rectangle(p, (x0 - 3, y0 - 32), (x0 + iw + 2, y0 + ih + 2), (10, 10, 10), -1)
        p[y0:y0 + ih, x0:x0 + iw] = cm
        self.T.put(p, "DEPTH \u00b7 SGBM", x0 + 6, y0 - 27, 16, TEXT, True)
        bw, bx = 80, x0 + iw - 132                                   # colour key: 1 m .. 10 m
        bar = cv2.resize(self.lut[np.arange(256, dtype=np.uint8)].reshape(1, 256, 3), (bw, 10))
        p[y0 - 20:y0 - 10, bx:bx + bw] = bar
        self.T.put(p, "1 m", bx - 6, y0 - 27, 15, MUTED, False, "right")
        self.T.put(p, "10 m", bx + bw + 6, y0 - 27, 15, MUTED)
        n = self.draw_boxes(p, r, s, off)
        self.panel_title(p, f"FRONT RIGHT  \u00b7  {self.det.label}")
        self.panel_stats(p, self.det_line("right", n))
        return p, n

    def rear_panel(self, img):
        if img is None:
            p = self.bg_rear.copy()
            cx, cy = PANEL_W // 2, PANEL_H // 2
            cv2.circle(p, (cx, cy - 40), 34, LINE, 3, cv2.LINE_AA)
            cv2.line(p, (cx - 24, cy - 64), (cx + 24, cy - 16), LINE, 3, cv2.LINE_AA)
            self.T.put(p, "No rear camera", cx, cy + 10, 34, MUTED, True, "center")
            self.T.put(p, "start with --rear <camera number> (or --rear-file for a replay)", cx, cy + 58, 20,
                       MUTED, False, "center")
            self.panel_title(p, f"REAR  \u00b7  {self.det.label}")
            self.objects["rear"] = []
            return p, 0
        p, s, off = self.cam_panel(img)
        r, _ = self.det.get("rear") if self.det.ok else (None, 0)
        n = self.draw_boxes(p, r, s, off)
        self.rear_objects(r)
        self.panel_title(p, f"REAR  \u00b7  {self.det.label}", "watching behind the car")
        self.panel_stats(p, self.det_line("rear", n))
        return p, n

    def front_objects(self):
        r, _ = self.det.get("left") if self.det.ok else (None, 0)
        objs = []
        if r is not None:
            for b, c, k in zip(r["xyxy"], r["conf"], r["cls"]):
                loc = self.vo.locate_box(b)
                if loc:
                    objs.append((loc[0], loc[1], self.det.names.get(int(k), "?"), loc[2], True))
        self.objects["left"] = objs

    def rear_objects(self, r):
        """Rear camera has no depth: a person's distance from their height in the picture (1.7 m tall)."""
        objs = []
        if r is not None:
            K = self.Kr
            hd = self.vo.heading(); p = self.vo.T[:3, 3]
            for b, c, k in zip(r["xyxy"], r["conf"], r["cls"]):
                name = self.det.names.get(int(k), "?")
                hpx = b[3] - b[1]
                if name != "person" or hpx < 30 or b[1] < 3:
                    continue
                z = K[1, 1] * 1.7 / hpx
                x = ((b[0] + b[2]) / 2 - K[0, 2]) * z / K[0, 0]
                lx, lz = -x, -z - 0.4                      # facing backwards, 0.4 m behind the front cameras
                wx = p[0] + lx * math.cos(hd) + lz * math.sin(hd)
                wz = p[2] - lx * math.sin(hd) + lz * math.cos(hd)
                objs.append((wx, wz, name, z, False))
        self.objects["rear"] = objs

    # top-down map
    def map_panel(self):
        MW = self.map_w                                    # half the screen, or all of it with no rear camera
        vo, s = self.vo, self.scale
        p = self.bg_map.copy()
        car = vo.T[:3, 3]
        hd = vo.heading()
        cxp, cyp = MW * 0.5, PANEL_H * 0.5
        look = 2.5                                         # keep what is ahead of the car in view
        cx, cz = car[0] + look * math.sin(hd), car[2] + look * math.cos(hd)

        def px(x, z):
            return cxp + (x - cx) * s, cyp - (z - cz) * s

        # grid
        x_lo, x_hi = cx - cxp / s, cx + (MW - cxp) / s
        z_lo, z_hi = cz - (PANEL_H - cyp) / s, cz + cyp / s
        grid_labels = []
        for m in range(int(math.floor(x_lo)), int(math.ceil(x_hi)) + 1):
            u = int(round(px(m, 0)[0]))
            major = m % 5 == 0
            cv2.line(p, (u, 0), (u, PANEL_H), rgb(40, 52, 66) if major else rgb(24, 32, 42), 1)
            if major and 20 < u < MW - 60:
                grid_labels.append((f"{m} m", u + 4, PANEL_H - 62, "left"))
        for m in range(int(math.floor(z_lo)), int(math.ceil(z_hi)) + 1):
            v = int(round(px(0, m)[1]))
            major = m % 5 == 0
            cv2.line(p, (0, v), (MW, v), rgb(40, 52, 66) if major else rgb(24, 32, 42), 1)
            if major and 90 < v < PANEL_H - 90:
                grid_labels.append((f"{m} m", MW - 10, v + 3, "right"))
        # accumulated points (warp the grid straight into panel pixels)
        g = vo.grid
        k = s * g.res
        M = np.float32([[k, 0, cxp - (g.n // 2) * k - cx * s], [0, -k, cyp + (g.n // 2) * k + cz * s]])
        cnt = cv2.warpAffine(g.counts, M, (MW, PANEL_H), flags=cv2.INTER_NEAREST, borderValue=0)
        occ = cnt >= 2
        if occ.any():
            t = np.clip((np.log1p(cnt[occ]) - np.log1p(2.0)) / (np.log1p(25.0) - np.log1p(2.0)), 0.0, 1.0)[:, None]
            t = 0.12 + 0.88 * t ** 1.5                     # seen once or twice: faint; seen again and again: bright
            lo, hi = np.float32(rgb(40, 50, 90)), np.float32(rgb(255, 214, 140))
            p[occ] = (lo * (1 - t) + hi * t).astype(np.uint8)
        for txt, gx, gy, al in grid_labels:
            tw, th = self.T.size(txt, 15)
            lx = gx - tw if al == "right" else gx
            shade(p, lx - 3, gy - 1, lx + tw + 3, gy + th + 1, rgb(12, 18, 26), 0.8)
            self.T.put(p, txt, lx, gy, 15, rgb(120, 135, 155))
        # field of view
        hf = math.atan(W / 2 / vo.fx)
        rng = min(vo.max_range, 8.0)
        c0 = px(car[0], car[2])
        wedge = [c0] + [px(car[0] + rng * math.sin(hd + a), car[2] + rng * math.cos(hd + a))
                        for a in np.linspace(-hf, hf, 16)]
        ov = p.copy()
        cv2.fillPoly(ov, [np.int32(wedge)], rgb(0, 140, 200), cv2.LINE_AA)
        p[:] = cv2.addWeighted(ov, 0.12, p, 0.88, 0)
        cv2.polylines(p, [np.int32(wedge)], True, rgb(0, 140, 200), 1, cv2.LINE_AA)
        # this frame's depth points
        if len(vo.scan):
            u = (cxp + (vo.scan[:, 0] - cx) * s).astype(int)
            v = (cyp - (vo.scan[:, 1] - cz) * s).astype(int)
            ok = (u >= 1) & (u < MW - 1) & (v >= 1) & (v < PANEL_H - 1)
            u, v = u[ok], v[ok]
            for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
                p[v + dv, u + du] = SCAN_C
        # path driven
        if len(vo.traj) > 1:
            tr = np.array(vo.traj, np.float32)
            pts = np.stack([cxp + (tr[:, 0] - cx) * s, cyp - (tr[:, 1] - cz) * s], 1)
            cv2.polylines(p, [np.int32(pts)], False, TRAJ_C, 3, cv2.LINE_AA)
        sx, sy = px(0, 0)
        cv2.circle(p, (int(sx), int(sy)), 7, TRAJ_C, 2, cv2.LINE_AA)
        # detected things (labels pushed apart so they never sit on top of each other)
        placed = []
        marks = [(px(x, z), name, r, front) for x, z, name, r, front in self.objects["left"] + self.objects["rear"]]
        for (u, v), name, r, front in sorted(marks, key=lambda m: m[0][1]):
            if not (0 <= u < MW and 0 <= v < PANEL_H):
                continue
            col = class_color(name)
            if front:
                cv2.circle(p, (int(u), int(v)), 11, col, -1, cv2.LINE_AA)
                cv2.circle(p, (int(u), int(v)), 11, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                cv2.circle(p, (int(u), int(v)), 11, col, 3, cv2.LINE_AA)
            label = f"{name} {r:.1f} m" + ("" if front else " (rear, est.)")
            tw, th = self.T.size(label, 17, True)
            lx = u + 19 if u + 27 + tw < MW else u - 23 - tw
            ly = v - th / 2
            for _ in range(6):
                if not any(abs(ly - py) < th + 6 and lx < qx + qw and qx < lx + tw for qx, py, qw in placed):
                    break
                ly += th + 6
            placed.append((lx, ly, tw))
            if abs(ly - (v - th / 2)) > 1:
                cv2.line(p, (int(u), int(v)), (int(lx - 4), int(ly + th / 2)), col, 1, cv2.LINE_AA)
            shade(p, lx - 4, ly - 3, lx + tw + 4, ly + th + 3, (0, 0, 0), 0.6)
            self.T.put(p, label, lx, ly, 17, col, True)
        # the car
        def rot(lx, lz):
            return px(car[0] + lx * math.cos(hd) + lz * math.sin(hd), car[2] - lx * math.sin(hd) + lz * math.cos(hd))
        body = np.int32([rot(-0.22, -0.35), rot(0.22, -0.35), rot(0.22, 0.2), rot(0, 0.42), rot(-0.22, 0.2)])
        cv2.fillPoly(p, [body], (255, 255, 255), cv2.LINE_AA)
        cv2.polylines(p, [body], True, ACCENT, 2, cv2.LINE_AA)
        tip = rot(0, 1.3)
        cv2.arrowedLine(p, (int(c0[0]), int(c0[1])), (int(tip[0]), int(tip[1])), ACCENT, 3, cv2.LINE_AA, tipLength=0.3)
        # scale bar
        bx, by = MW - 30 - int(2 * s), 80
        cv2.line(p, (bx, by), (bx + int(2 * s), by), TEXT, 2)
        cv2.line(p, (bx, by - 5), (bx, by + 5), TEXT, 2); cv2.line(p, (bx + int(2 * s), by - 5), (bx + int(2 * s), by + 5), TEXT, 2)
        self.T.put(p, "2 m", bx + int(s), by - 26, 16, TEXT, False, "center")
        # legend + stats
        items = [(SCAN_C, "depth now"), (rgb(255, 214, 140), "map"), (TRAJ_C, "path"),
                 (rgb(255, 72, 96), "person")]
        x = 14; y = PANEL_H - 34
        shade(p, 0, y - 10, MW, PANEL_H, (0, 0, 0), 0.55)
        for col, name in items:
            cv2.circle(p, (x + 7, y + 11), 7, col, -1, cv2.LINE_AA)
            x += 20 + self.T.put(p, name, x + 20, y, 17, MUTED) + 22
        keys = "q quit \u00b7 f fullscreen \u00b7 space pause \u00b7 r reset \u00b7 +/- zoom"
        self.T.put(p, keys, MW - 14, y + 1, 16, rgb(120, 130, 145), False, "right")
        st_col = OK_C if vo.status == "tracking" else BAD_C if vo.status == "lost" else MUTED
        self.panel_title(p, "POINT MAP  \u00b7  stereo VO", "top-down, start heading up")
        chip(p, self.T, f"VO {vo.status}  \u00b7  {vo.n_inliers} inliers  \u00b7  {g.cells():,} map cells (10 cm)",
             14, 52, 18, fg=st_col, bold=False, alpha=0.55)
        return p

    def header(self, canvas, fps, ms, ndet, paused):
        T, vo = self.T, self.vo
        canvas[:HEADER_H] = rgb(10, 12, 16)
        cv2.rectangle(canvas, (0, 0), (8, HEADER_H), ACCENT, -1)
        x = 28
        x += T.put(canvas, "SFU Guide Car", x, 12, 32, TEXT, True) + 14
        T.put(canvas, "\u2014  live perception", x, 12, 32, MUTED, False)
        st_col = OK_C if vo.status == "tracking" else BAD_C if vo.status == "lost" else MUTED
        metrics = [("SOURCE", self.source_name, TEXT), ("FPS", f"{fps:4.1f}", TEXT),
                   ("PIPELINE", f"{ms:3.0f} ms", TEXT), ("DETECTIONS", str(ndet), TEXT),
                   ("VISUAL ODOMETRY", ("\u25cf " if T.pil else "") + vo.status, st_col),
                   ("TRAVELLED", f"{vo.dist:5.1f} m", TEXT)]
        if paused:
            metrics.insert(0, ("", "PAUSED", rgb(255, 200, 60)))
        x = SCREEN_W - 24
        for cap, val, col in reversed(metrics):
            wv, _ = T.size(val, 26, True)
            wc, _ = T.size(cap, 13, True)
            w = max(wv, wc)
            T.put(canvas, cap, x - w, 7, 13, MUTED, True)
            T.put(canvas, val, x - w, 24, 26, col, True)
            x -= w + 40
        cv2.line(canvas, (0, HEADER_H - 1), (SCREEN_W, HEADER_H - 1), LINE, 1)

    def compose(self, left, right, rear, fps, ms, paused):
        canvas = self.bg_canvas.copy()
        self.front_objects()
        pl, n1 = self.left_panel(left)
        pr, n2 = self.right_panel(right)
        pb, n3 = self.rear_panel(rear) if self.has_rear else (None, 0)
        pm = self.map_panel()
        y0 = HEADER_H
        canvas[y0:y0 + PANEL_H, :PANEL_W] = pl
        canvas[y0:y0 + PANEL_H, PANEL_W:] = pr
        if self.has_rear:
            canvas[y0 + PANEL_H:y0 + 2 * PANEL_H, :PANEL_W] = pb
            canvas[y0 + PANEL_H:y0 + 2 * PANEL_H, PANEL_W:] = pm
            cv2.line(canvas, (PANEL_W, y0), (PANEL_W, SCREEN_H), BG, 4)
        else:
            canvas[y0 + PANEL_H:y0 + 2 * PANEL_H, :] = pm
            cv2.line(canvas, (PANEL_W, y0), (PANEL_W, y0 + PANEL_H), BG, 4)
        cv2.line(canvas, (0, y0 + PANEL_H), (SCREEN_W, y0 + PANEL_H), BG, 4)
        if not self.det.ok and self.det.status:
            w, h = self.T.size(self.det.status, 20, True)
            chip(canvas, self.T, self.det.status, (PANEL_W - w) // 2 - 10, SCREEN_H - h - 40, 20,
                 fg=rgb(255, 200, 60), alpha=0.75)
        self.header(canvas, fps, ms, n1 + n2 + n3, paused)
        return canvas


# ---------------------------------------------------------------- main
def parse():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left", type=int, default=3, help="left camera index (default 3)")
    ap.add_argument("--right", type=int, default=2, help="right camera index (default 2)")
    ap.add_argument("--rear", type=int, default=None, help="rear camera index (e.g. 1)")
    ap.add_argument("--left-file"); ap.add_argument("--right-file"); ap.add_argument("--rear-file")
    ap.add_argument("--model", default=str(REPO / "models" / "yolo11n.onnx"),
                    help="detector: models/yolo11n.onnx (fast), yolo11s.pt or yolo11m.pt (more accurate, slower; "
                         "downloaded on first use), models/yoloe-school.onnx (school objects)")
    ap.add_argument("--obstacles-only", action="store_true",
                    help="label only people and obstacles (default: every class the model knows)")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--det-threads", type=int, default=0, help="CPU threads for YOLO (default: all but 2)")
    ap.add_argument("--calib-dir", default=str(CALIB))
    ap.add_argument("--swap", dest="swap", action="store_true", default=None, help="force swap left/right")
    ap.add_argument("--no-swap", dest="swap", action="store_false", help="never swap left/right")
    ap.add_argument("--rectify", dest="rect", action="store_true", default=None, help="always use the calibration")
    ap.add_argument("--no-rectify", dest="rect", action="store_false", help="never use the calibration")
    ap.add_argument("--pitch", type=float, default=5.0, help="cameras tilted up, degrees (levels the map)")
    ap.add_argument("--cam-height", type=float, default=0.28, help="camera height above the ground, m")
    ap.add_argument("--max-range", type=float, default=8.0, help="depth points further than this are not mapped")
    ap.add_argument("--map-scale", type=float, default=38.0, help="map pixels per metre (+/- keys)")
    ap.add_argument("--no-pace", action="store_true", help="replays: run as fast as possible, not at their fps")
    ap.add_argument("--headless", action="store_true", help="no window (for --record / --snap)")
    ap.add_argument("--record", help="write the screen to this video (mp4v; re-encode for sharing)")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames")
    ap.add_argument("--snap", help="save a screenshot every --snap-every frames as <SNAP>_0060.jpg")
    ap.add_argument("--snap-every", type=int, default=60)
    a = ap.parse_args()
    if (a.left_file is None) != (a.right_file is None):
        ap.error("--left-file and --right-file go together")
    return a


def main():
    args = parse()
    src = StereoSource(args)
    rear = RearSource(args)
    calib = Calibration(Path(args.calib_dir) / "left.yaml", Path(args.calib_dir) / "right.yaml")
    swap, rect, notes = choose_geometry(src.peek(), calib, args.swap, args.rect)
    src.swap = swap
    for n in notes:
        print("note:", n)
    vo = StereoVO(calib, rect, args)
    det = Detector(args.model, not args.obstacles_only, args.conf, args.det_threads)
    if det.status and det.model is None:
        print(det.status)
    T = Text()
    if not T.pil:
        print("note: Pillow/TrueType font not found, using OpenCV's built-in font")
    screen = Screen(T, det, vo, args, src.name + ("  (L/R swapped)" if swap else ""), notes)

    win = "SFU Guide Car - live perception"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1600, 900)
    writer = None
    if args.record:
        fps_out = src.fps if not src.live else 15.0
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), fps_out, (SCREEN_W, SCREEN_H))

    seq, paused, fullscreen, fails = 0, False, False, 0
    fps, ms = 0.0, 0.0
    t_last = time.time()
    stats = {"n": 0, "proc": 0.0, "t0": time.time()}
    left = right = rr = None
    canvas = None
    while True:
        t0 = time.time()
        if not paused or left is None:
            ok, l, r = src.read()
            if not ok:
                if src.live:
                    fails += 1
                    if fails > 60:
                        print("the stereo cameras stopped sending pictures"); break
                    time.sleep(0.02); continue
                src.rewind(); rear.rewind(); vo.reset()          # replay: loop, fresh map
                ok, l, r = src.read()
                if not ok:
                    print("could not read the replay files"); break
            fails = 0
            seq += 1
            left, right = vo.process(l, r)
            rr = rear.read()
            det.submit("left", seq, left); det.submit("right", seq, right)
            if rr is not None:
                det.submit("rear", seq, rr)
            else:
                det.drop("rear")
        canvas = screen.compose(left, right, rr, fps, ms, paused)
        proc = time.time() - t0
        ms = 0.9 * ms + 0.1 * proc * 1000 if ms else proc * 1000
        stats["n"] += 1; stats["proc"] += proc
        if writer is not None:
            writer.write(canvas)
        if args.snap and seq % args.snap_every == 0 and not paused:
            cv2.imwrite(f"{args.snap}_{seq:04d}.jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not args.headless:
            cv2.imshow(win, canvas)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("f"):
                fullscreen = not fullscreen
                cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            elif k == ord(" "):
                paused = not paused
            elif k == ord("r"):
                vo.reset()
            elif k in (ord("+"), ord("=")):
                screen.scale = min(screen.scale * 1.25, 200)
            elif k in (ord("-"), ord("_")):
                screen.scale = max(screen.scale / 1.25, 5)
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
        if not src.live and not args.no_pace:            # play replays at the speed they were recorded
            wait = 1.0 / src.fps - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)
        elif paused:
            time.sleep(0.03)
        now = time.time()
        dt = now - t_last; t_last = now
        fps = 0.9 * fps + 0.1 / max(dt, 1e-3) if fps else 1 / max(dt, 1e-3)
        if args.frames and seq >= args.frames:
            break
    el = time.time() - stats["t0"]
    if stats["n"]:
        print(f"{stats['n']} frames in {el:.1f} s: {stats['n'] / el:.1f} fps on screen, "
              f"{1000 * stats['proc'] / stats['n']:.0f} ms per frame of work (depth + odometry + drawing), "
              f"travelled {vo.dist:.1f} m")
    det.stop()
    src.release()
    rear.stop = True
    if writer is not None:
        writer.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
