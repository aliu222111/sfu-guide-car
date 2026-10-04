"""Line a Sensor Logger recording up with a stereo_record.py video, by matching turns.

    py walks\\stereo_record\\sync_phone.py <video base> <phone folder>
    py walks\\stereo_record\\sync_phone.py aq3000_walk1 "$HOME\\Downloads\\2026-10-04_04-49-39"

<video base> is the start of the video names (aq3000_walk1 for aq3000_walk1_left.avi and
aq3000_walk1_times.csv). <phone folder> is Sensor Logger's export (Gyroscope.csv, Gravity.csv,
Accelerometer.csv, Location.csv), or a phone_receiver.py folder.

The laptop and phone clocks differ by up to seconds. The rig's turning shows twice: as the
picture sliding sideways between video frames, and as the phone's gyro about the vertical.
Sliding one against the other until they match gives the offset (laptop time = phone time +
offset). Writes:
  <video base>_phone.csv    one row per video frame: laptop time, gyro, accelerometer, gravity,
                            and the latest GPS fix (lat, lon, accuracy)
  <phone folder>/clock_offset.txt   the offset, which clean_walk.py then uses
  <video base>_sync.png     the two turn rates on top of each other: they should overlap
Needs opencv-python and numpy.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import cv2
import numpy as np

HFOV = np.radians(70.0)  # the webcams' horizontal field of view
SEARCH = 300.0  # seconds either way to look for the offset


def read_csv(path: Path, cols):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["time"]) / 1e9 for r in rows])
    return t, np.array([[float(r[c]) if r.get(c) not in (None, "") else np.nan for c in cols] for r in rows])


def video_turn_rate(base: str):
    """Yaw rate (rad/s, left positive) from the sideways slide between left-camera frames."""
    times = np.loadtxt(f"{base}_times.csv", delimiter=",", skiprows=1)[:, 1]
    cap = cv2.VideoCapture(f"{base}_left.avi")
    prev, dx = None, []
    win = cv2.createHanningWindow((160, 90), cv2.CV_32F)
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        g = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 90)).astype(np.float32)
        dx.append(0.0 if prev is None else cv2.phaseCorrelate(prev, g, win)[0][0])
        prev = g
        n += 1
        if n % 1500 == 0:
            print(f"  {n} frames read", flush=True)
    cap.release()
    m = min(len(dx), len(times))
    dx, times = np.array(dx[:m]) * 640 / 160, times[:m]
    dt = np.diff(times, prepend=times[0] - 1 / 15)
    # The scene sliding right means the camera turned left: yaw = +slide * radians per pixel / dt
    return times, dx * (HFOV / 640) / np.maximum(dt, 1e-3)


def phone_turn_rate(folder: Path):
    tg, gy = read_csv(folder / "Gyroscope.csv", ("x", "y", "z"))
    tv, gr = read_csv(folder / "Gravity.csv", ("x", "y", "z"))
    grav = np.c_[[np.interp(tg, tv, gr[:, i]) for i in range(3)]].T
    up = -grav / np.linalg.norm(grav, axis=1, keepdims=True)
    return tg, (gy * up).sum(1)


def _norm(x):
    x = np.nan_to_num(x)
    return (x - x.mean()) / (x.std() + 1e-9)


def best_offset(tv, wv, tp, wp):
    """Offset (s) maximising the correlation of the two turn rates (either sign), to 1 ms."""
    hz = 20.0
    v = _norm(np.interp(np.arange(tv[0], tv[-1], 1 / hz), tv, np.clip(wv, -3, 3)))
    p = _norm(np.interp(np.arange(tp[0], tp[-1], 1 / hz), tp, np.clip(wp, -3, 3)))
    n = 1 << int(np.ceil(np.log2(len(v) + len(p))))
    cc = np.fft.irfft(np.fft.rfft(v, n) * np.conj(np.fft.rfft(p, n)), n)
    lags = np.r_[np.arange(0, len(v)), np.arange(-len(p) + 1, 0)]  # v[i] ~ p[i - lag]
    cc = np.r_[cc[:len(v)], cc[n - len(p) + 1:]]
    offsets = (tv[0] - tp[0]) + lags / hz  # laptop time = phone time + offset
    guess = np.median(tv) - np.median(tp)
    ok = np.abs(offsets - guess) <= SEARCH
    best = None
    for sign in (1, -1):
        c = sign * cc
        i = np.flatnonzero(ok)[np.argmax(c[ok])]
        if best is None or c[i] > best[0]:
            best = (c[i], offsets[i], sign)
    _, lag0, sign = best

    def match(lag):  # normalised correlation over the overlap
        grid = np.arange(tv[0], tv[-1], 1 / hz)
        q = np.interp(grid - lag, tp, sign * wp, left=np.nan, right=np.nan)
        m = ~np.isnan(q)
        if m.mean() < 0.3:
            return -1.0
        a, b = np.interp(grid[m], tv, np.clip(wv, -3, 3)), np.clip(q[m], -3, 3)
        return float((_norm(a) * _norm(b)).mean())

    fine = max(np.arange(lag0 - 0.1, lag0 + 0.1, 0.002), key=match)
    return match(fine), float(fine), sign


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) < 2:
        print(__doc__); return 1
    base, folder = argv[0], Path(argv[1])
    print("reading the video...")
    tv, wv = video_turn_rate(base)
    tp, wp = phone_turn_rate(folder)
    print(f"video {tv[-1] - tv[0]:.0f} s from {tv[0]:.1f}; phone {tp[-1] - tp[0]:.0f} s from {tp[0]:.1f}")
    corr, offset, sign = best_offset(tv, wv, tp, wp)
    print(f"offset: laptop time = phone time {offset:+.3f} s   (match {corr:.2f}; "
          f"{'good' if corr > 0.6 else 'weak: check the plot' if corr > 0.3 else 'poor: did the phone and video record the same walk?'})")
    (folder / "clock_offset.txt").write_text(f"{offset:.4f}\n")
    # The phone turning on its own (slipping on the rig) shows as gyro turning the video does not
    # see. Report where the two disagree for a second or more.
    ph = np.interp(tv, tp + offset, sign * wp)
    k = max(1, int(round(1.0 / np.median(np.diff(tv)))))
    resid = np.convolve(ph - np.clip(wv, -3, 3), np.ones(k) / k, "same")
    turned = np.cumsum(resid * np.diff(tv, prepend=tv[0]))
    slips, i = [], 0
    while i < len(tv):
        if abs(resid[i]) > 0.35:
            j = i
            while j < len(tv) and abs(resid[j]) > 0.15:
                j += 1
            ang = np.degrees(turned[min(j, len(tv) - 1)] - turned[max(i - 1, 0)])
            if abs(ang) > 15:
                slips.append((tv[i] - tv[0], tv[min(j, len(tv) - 1)] - tv[0], ang))
            i = j
        else:
            i += 1
    for a, b, ang in slips:
        print(f"phone turned {ang:+.0f} deg on its own at {a // 60:.0f}:{a % 60:04.1f}-{b // 60:.0f}:{b % 60:04.1f} into the video"
              " (slipped on the rig?): phone heading after this differs from the rig's")
    if not slips:
        print("phone stayed put on the rig (no turning the video did not see)")
    # one row per frame
    ta, ac = read_csv(folder / "Accelerometer.csv", ("x", "y", "z"))
    tgv, gr = read_csv(folder / "Gravity.csv", ("x", "y", "z"))
    tg, gy = read_csv(folder / "Gyroscope.csv", ("x", "y", "z"))
    loc = folder / "Location.csv"
    tl, ll = read_csv(loc, ("latitude", "longitude", "horizontalAccuracy")) if loc.exists() else (np.array([]), np.zeros((0, 3)))
    frames = np.loadtxt(f"{base}_times.csv", delimiter=",", skiprows=1)
    with open(f"{base}_phone.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "unix_time", "gx", "gy", "gz", "ax", "ay", "az", "grx", "gry", "grz", "lat", "lon", "gps_acc"])
        for fr, t in frames:
            tph = t - offset
            row = [int(fr), f"{t:.3f}"]
            for ts, arr in ((tg, gy), (ta, ac), (tgv, gr)):
                row += [f"{np.interp(tph, ts, arr[:, i]):.5f}" for i in range(3)]
            j = np.searchsorted(tl, tph) - 1
            row += [f"{v:.7f}" for v in ll[j][:2]] + [f"{ll[j][2]:.1f}"] if j >= 0 and len(tl) else ["", "", ""]
            w.writerow(row)
    # plot: both turn rates over the whole walk
    W, H = 1600, 300
    img = np.full((H, W, 3), 255, np.uint8)
    T = lambda t: int((t - tv[0]) / (tv[-1] - tv[0]) * (W - 1))
    Y = lambda v: int(H / 2 - np.clip(v, -2, 2) * H / 4.4)
    k = max(1, len(tv) // W)
    for i in range(0, len(tv) - k, k):
        cv2.line(img, (T(tv[i]), Y(wv[i])), (T(tv[i + k]), Y(wv[i + k])), (220, 120, 0), 1)
    ph = np.interp(tv, tp + offset, sign * wp)
    for i in range(0, len(tv) - k, k):
        cv2.line(img, (T(tv[i]), Y(ph[i])), (T(tv[i + k]), Y(ph[i + k])), (0, 0, 220), 1)
    cv2.putText(img, f"blue: video turn rate   red: phone gyro shifted {offset:+.2f} s   match {corr:.2f}", (10, 20), 0, 0.5, (0, 0, 0), 1)
    cv2.imwrite(f"{base}_sync.png", img)
    print(f"wrote {base}_phone.csv, {base}_sync.png and {folder / 'clock_offset.txt'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
