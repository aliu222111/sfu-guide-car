"""Calibrate the stereo pair with a printed checkerboard; no ROS, so it runs on Windows too.

    py stereo_calibrate.py board                          # checkerboard.png: print at 100 %, check a square is 25 mm
    py stereo_calibrate.py capture 1 0 --out calib        # left and right camera index, as for stereo_record.py
    py stereo_calibrate.py solve calib                    # writes calib/left.yaml and calib/right.yaml

capture: hold the board (taped flat to something stiff) in front of the rig. A pair is
saved by itself whenever both cameras see the whole board, it is sharp, and it is in a
new place, size or tilt; aim for 30 to 40 (near, far, every corner of the picture,
tilted left, right, up and down). Press s to save one by hand, q to finish.

solve: per-camera then stereo calibration. It prints the reprojection error (good
under 0.5 px), the baseline (the rig's camera spacing: about 0.10 m) and the vertical
misalignment after rectification (good under 1 px), and writes ROS camera_info YAML:
copy left.yaml and right.yaml to ~/.ros/camera_info on the car laptop (cameras.launch.py
reads them from calibration_dir).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

COLS, ROWS = 8, 6  # inner corners, as the README's cameracalibrator --size 8x6
SQUARE = 0.025  # metres
SIZE = (640, 360)  # what stereo_record.py and cameras.yaml capture


def board(out: Path, dpi: int = 300):
    px = round(SQUARE / 0.0254 * dpi)
    sq_x, sq_y = COLS + 1, ROWS + 1
    margin = px // 2
    img = np.full((sq_y * px + 2 * margin, sq_x * px + 2 * margin), 255, np.uint8)
    for r in range(sq_y):
        for c in range(sq_x):
            if (r + c) % 2 == 0:
                img[margin + r * px: margin + (r + 1) * px, margin + c * px: margin + (c + 1) * px] = 0
    cv2.putText(img, f"{COLS}x{ROWS} inner corners, {SQUARE * 1000:.0f} mm squares: print at 100 % and measure",
                (margin, margin * 2 // 3), cv2.FONT_HERSHEY_SIMPLEX, dpi / 400, 0, max(1, dpi // 150))
    cv2.imwrite(str(out), img)
    mm = (img.shape[1] / dpi * 25.4, img.shape[0] / dpi * 25.4)
    print(f"{out}: {mm[0]:.0f} x {mm[1]:.0f} mm at {dpi} dpi (fits letter landscape). Print with scaling off.")


def find(gray):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, c = cv2.findChessboardCorners(gray, (COLS, ROWS), flags)
    if not ok:
        # The sector-based detector finds small, far or tilted boards the classic one misses;
        # its corners are already sub-pixel. Upscaled 2x, squares down to ~5 px still work.
        big = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        ok, c = cv2.findChessboardCornersSB(big, (COLS, ROWS), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        return (c / 2).astype(np.float32) if ok else None
    return cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))


def pose_key(c, shape):
    """Where the board is, how big and how tilted: a new pair must differ from every saved one."""
    p = c.reshape(-1, 2)
    h, w = shape[:2]
    x, y = p[:, 0].mean() / w, p[:, 1].mean() / h
    size = np.sqrt(cv2.contourArea(cv2.convexHull(p.astype(np.float32))) / (w * h))
    top = np.linalg.norm(p[COLS - 1] - p[0]); bottom = np.linalg.norm(p[-1] - p[-COLS])
    left = np.linalg.norm(p[-COLS] - p[0]); right = np.linalg.norm(p[-1] - p[COLS - 1])
    return np.array([x, y, size, (top - bottom) / (top + bottom), (left - right) / (left + right)])


def capture(li: int, ri: int, out: Path, want: int = 40):
    def cam(i):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW) if sys.platform == "win32" else cv2.VideoCapture(i)
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, SIZE[0]); c.set(cv2.CAP_PROP_FRAME_HEIGHT, SIZE[1])
        c.set(cv2.CAP_PROP_FPS, 30)  # as the car runs them
        return c

    out.mkdir(parents=True, exist_ok=True)
    cl, cr = cam(li), cam(ri)
    for name, i, c in (("left", li, cl), ("right", ri, cr)):
        if not c.isOpened():
            found = [j for j in range(6) if j not in (li, ri) and cam(j).isOpened()]
            print(f"the {name} camera ({i}) did not open: unplugged, or still open in another window "
                  f"(close stereo_test / stereo_record / the Camera app). Other cameras that open: {found}")
            return
    keys, n = [], len(list(out.glob("left_*.png")))
    print(f"saving to {out}; {n} pairs already there. s = save now, q = finish")
    misses = 0
    while True:
        g1 = cl.grab(); g2 = cr.grab()  # both first: the two pictures are taken together
        ok1, a = cl.retrieve() if g1 else (False, None)
        ok2, b = cr.retrieve() if g2 else (False, None)
        if not (ok1 and ok2):
            misses += 1  # a USB camera often drops the first frames, or one now and then
            if misses < 30:
                continue
            bad = [f"{nm} ({i})" for nm, i, ok in (("left", li, ok1), ("right", ri, ok2)) if not ok]
            print(f"no picture from the {' and '.join(bad)} camera for 30 frames: check the cable, "
                  f"or plug the two cameras into different USB ports (one hub may not carry both)")
            break
        misses = 0
        ga, gb = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
        ca, cb = find(ga), find(gb)
        sharp = min(cv2.Laplacian(ga, cv2.CV_64F).var(), cv2.Laplacian(gb, cv2.CV_64F).var())
        key = pose_key(ca, ga.shape) if ca is not None else None
        new = key is not None and all(np.abs(key - k).max() > 0.08 for k in keys)
        view = [x.copy() for x in (a, b)]
        for v, c in ((view[0], ca), (view[1], cb)):
            if c is not None:
                cv2.drawChessboardCorners(v, (COLS, ROWS), c, True)
        k = cv2.waitKey(1) & 0xFF
        auto = ca is not None and cb is not None and new and sharp > 60
        if auto or (k == ord("s") and ca is not None and cb is not None):
            cv2.imwrite(str(out / f"left_{n:03d}.png"), a); cv2.imwrite(str(out / f"right_{n:03d}.png"), b)
            keys.append(key); n += 1
            print(f"  pair {n}" + ("" if n < want else "  (enough: q to finish, or keep adding tilted views)"))
            for v in view:
                v[:] = 255 - v  # flash
        msg = f"{n} saved | " + ("both see it" if ca is not None and cb is not None else "board not in both")
        cv2.putText(view[0], msg, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("left | right  (s save, q quit)", cv2.hconcat(view))
        if k == ord("q"):
            break
    cl.release(); cr.release(); cv2.destroyAllWindows()
    print(f"{n} pairs in {out}. Next: py stereo_calibrate.py solve {out}")


def camera_info(name, size, K, D, R, P) -> str:
    def mat(m, rows, cols):
        return f"  rows: {rows}\n  cols: {cols}\n  data: [{', '.join(f'{v:.8g}' for v in np.asarray(m).ravel())}]\n"
    return (f"image_width: {size[0]}\nimage_height: {size[1]}\ncamera_name: {name}\n"
            f"camera_matrix:\n{mat(K, 3, 3)}distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n{mat(np.asarray(D).ravel()[:5], 1, 5)}"
            f"rectification_matrix:\n{mat(R, 3, 3)}projection_matrix:\n{mat(P, 3, 4)}")


def solve(folder: Path) -> int:
    obj = np.zeros((COLS * ROWS, 3), np.float32)
    obj[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE
    objs, pl, pr, size = [], [], [], None
    for lf in sorted(folder.glob("left_*.png")):
        rf = lf.with_name(lf.name.replace("left_", "right_"))
        a, b = cv2.imread(str(lf), 0), cv2.imread(str(rf), 0)
        if a is None or b is None:
            continue
        size = a.shape[::-1]
        ca, cb = find(a), find(b)
        if ca is None or cb is None:
            continue
        objs.append(obj); pl.append(ca); pr.append(cb)
    if len(objs) < 10:
        print(f"only {len(objs)} usable pairs in {folder}: capture at least 20"); return 1
    e1, K1, D1, _, _ = cv2.calibrateCamera(objs, pl, size, None, None)
    e2, K2, D2, _, _ = cv2.calibrateCamera(objs, pr, size, None, None)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    e, K1, D1, K2, D2, R, T, _, _ = cv2.stereoCalibrate(objs, pl, pr, K1, D1, K2, D2, size,
                                                        flags=cv2.CALIB_USE_INTRINSIC_GUESS, criteria=crit)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, R, T, alpha=0)
    # How well rectification lines rows up: the same corner should land on the same row.
    m1 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)
    m2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1)
    rows = []
    for ca, cb in zip(pl, pr):
        ua = cv2.undistortPoints(ca, K1, D1, R=R1, P=P1).reshape(-1, 2)
        ub = cv2.undistortPoints(cb, K2, D2, R=R2, P=P2).reshape(-1, 2)
        rows.append(np.abs(ua[:, 1] - ub[:, 1]))
    rows = np.concatenate(rows)
    baseline = float(np.linalg.norm(T))
    fov = np.degrees(2 * np.arctan(size[0] / 2 / K1[0, 0]))
    print(f"{len(objs)} pairs, {size[0]}x{size[1]}")
    print(f"  reprojection error: left {e1:.2f} px, right {e2:.2f} px, stereo {e:.2f} px   (good: under 0.5)")
    print(f"  baseline {baseline * 100:.1f} cm (the rig: about 10), horizontal field of view {fov:.0f} deg")
    print(f"  after rectification, rows differ by {np.median(rows):.2f} px median, {np.percentile(rows, 95):.2f} px 95th   (good: under 1)")
    if T[0] > 0:
        print("  WARNING: the right camera is to the LEFT of the left one: swap the camera indices and capture again")
    (folder / "left.yaml").write_text(camera_info("left", size, K1, D1, R1, P1))
    (folder / "right.yaml").write_text(camera_info("right", size, K2, D2, R2, P2))
    fa, fb = cv2.imread(str(sorted(folder.glob("left_*.png"))[0])), cv2.imread(str(sorted(folder.glob("right_*.png"))[0]))
    ra = cv2.remap(fa, *m1, cv2.INTER_LINEAR); rb = cv2.remap(fb, *m2, cv2.INTER_LINEAR)
    check = cv2.hconcat([ra, rb])
    for y in range(0, check.shape[0], 30):
        cv2.line(check, (0, y), (check.shape[1], y), (0, 255, 0), 1)
    cv2.imwrite(str(folder / "rectified_check.png"), check)
    print(f"wrote {folder / 'left.yaml'}, {folder / 'right.yaml'} and rectified_check.png (rows should line up)")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("board"); b.add_argument("--out", type=Path, default=Path("checkerboard.png"))
    c = sub.add_parser("capture"); c.add_argument("left", type=int); c.add_argument("right", type=int)
    c.add_argument("--out", type=Path, default=Path("calib"))
    s = sub.add_parser("solve"); s.add_argument("folder", type=Path)
    a = ap.parse_args(argv)
    if a.cmd == "board":
        board(a.out); return 0
    if a.cmd == "capture":
        capture(a.left, a.right, a.out); return 0
    return solve(a.folder)


if __name__ == "__main__":
    sys.exit(main())
