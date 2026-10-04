import sys, csv, cv2, numpy as np
from pathlib import Path

STEP = 15
FPS = 15
MIN_FRAMES = 150
MAX_DRIFT = 4.0
MAX_SPREAD = 12.0

def scan(left, right):
    cl, cr = cv2.VideoCapture(str(left)), cv2.VideoCapture(str(right))
    orb = cv2.ORB_create(1200)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    prev, motion, align, n = None, [], {}, 0
    while True:
        ok1, a = cl.read()
        ok2, b = cr.read()
        if not (ok1 and ok2):
            break
        ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(ga, (160, 90)).astype(np.int16)
        motion.append(0.0 if prev is None else float(np.abs(small - prev).mean()))
        prev = small
        if n % STEP == 0:
            gb = cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
            ka, da = orb.detectAndCompute(ga, None)
            kb, db = orb.detectAndCompute(gb, None)
            if da is not None and db is not None and len(da) > 8 and len(db) > 8:
                m = sorted(bf.match(da, db), key=lambda x: x.distance)[:150]
                if len(m) >= 12:
                    d = np.array([[ka[x.queryIdx].pt[0] - kb[x.trainIdx].pt[0],
                                   ka[x.queryIdx].pt[1] - kb[x.trainIdx].pt[1]] for x in m])
                    dx, dy = np.median(d, axis=0)
                    align[n] = (dx, dy, float(np.median(np.abs(d[:, 1] - dy))))
        n += 1
    cl.release(); cr.release()
    return np.array(motion), align, n

def stretches(motion, align):
    ks = sorted(align)
    dy = np.array([align[k][1] for k in ks])
    spread = np.array([align[k][2] for k in ks])
    smooth = np.convolve(motion, np.ones(15) / 15, mode="same")
    moving = np.array([smooth[min(k, len(smooth) - 1)] for k in ks]) > np.percentile(smooth, 20)
    good = moving & (np.abs(np.diff(dy, prepend=dy[0])) < 1.5) & (spread < MAX_SPREAD)
    out, i = [], 0
    while i < len(ks):
        if not good[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(ks) and good[j + 1] and abs(dy[j + 1] - dy[i]) < MAX_DRIFT:
            j += 1
        if ks[j] - ks[i] >= MIN_FRAMES:
            out.append((ks[i], ks[j], float(np.median(dy[i:j + 1]))))
        i = j + 1
    return out

def phone(folder, lag):
    rows = {}
    for name, keys in (("Gyroscope.csv", "w"), ("Accelerometer.csv", "a")):
        with open(Path(folder) / name) as f:
            for r in csv.DictReader(f):
                t = round(float(r["time"]) / 1e9 + lag, 3)
                rows.setdefault(t, {})[keys] = (float(r["x"]), float(r["y"]), float(r["z"]))
    out = [(t, v["w"], v["a"]) for t, v in rows.items() if "w" in v and "a" in v]
    return sorted(out)


def write_imu(samples, name, a, b):
    rows = [s for s in samples if a <= s[0] <= b]
    if not rows:
        return 0
    with open(str(name) + "_imu.csv", "w") as f:
        f.write("unix_time,wx,wy,wz,ax,ay,az\n")
        for t, w, ac in rows:
            f.write(f"{t:.3f},{w[0]:.6f},{w[1]:.6f},{w[2]:.6f},{ac[0]:.6f},{ac[1]:.6f},{ac[2]:.6f}\n")
    return len(rows)


def cut(left, right, times, segs, outdir, swap, samples=None):
    stamps = {int(r["frame"]): r["unix_time"] for r in csv.DictReader(open(times))}
    cl, cr = cv2.VideoCapture(str(left)), cv2.VideoCapture(str(right))
    w = int(cl.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cl.get(cv2.CAP_PROP_FRAME_HEIGHT))
    four = cv2.VideoWriter_fourcc(*"MJPG")
    outdir.mkdir(parents=True, exist_ok=True)
    made = []
    for n, (a, b, dy) in enumerate(segs):
        s = int(round(-dy if swap else dy))
        keep = h - abs(s)
        name = outdir / f"seg{n:02d}"
        wl = cv2.VideoWriter(str(name) + "_left.avi", four, FPS, (w, keep))
        wr = cv2.VideoWriter(str(name) + "_right.avi", four, FPS, (w, keep))
        ts = open(str(name) + "_times.csv", "w")
        ts.write("frame,unix_time\n")
        cl.set(cv2.CAP_PROP_POS_FRAMES, a); cr.set(cv2.CAP_PROP_POS_FRAMES, a)
        for i in range(a, b + 1):
            ok1, fa = cl.read(); ok2, fb = cr.read()
            if not (ok1 and ok2):
                break
            eye_l, eye_r = (fb, fa) if swap else (fa, fb)
            if s >= 0:
                wl.write(eye_l[s:]); wr.write(eye_r[:keep])
            else:
                wl.write(eye_l[:keep]); wr.write(eye_r[abs(s):])
            if i in stamps:
                ts.write(f"{i - a},{stamps[i]}\n")
        wl.release(); wr.release(); ts.close()
        nimu = write_imu(samples, name, stamps.get(a, 0) and float(stamps[a]), float(stamps.get(b, stamps[a]))) if samples else 0
        made.append((name.name, b - a + 1, (b - a + 1) / FPS, dy, keep, nimu))
    cl.release(); cr.release()
    return made

if __name__ == "__main__":
    base = Path(sys.argv[1])
    outdir = Path(sys.argv[2])
    swap = "--no-swap" not in sys.argv
    sensors = next((x for x in sys.argv[3:] if not x.startswith("-")), None)
    lag = float(next((x.split("=")[1] for x in sys.argv if x.startswith("--lag=")), 0.13))
    left, right = Path(str(base) + "_left.avi"), Path(str(base) + "_right.avi")
    motion, align, n = scan(left, right)
    segs = stretches(motion, align)
    samples = phone(sensors, lag) if sensors else None
    print(f"{n} frames, {len(segs)} usable stretches, swap={swap}, imu lag {lag:+.2f} s")
    for name, frames, secs, dy, keep, nimu in cut(left, right, str(base) + "_times.csv", segs, outdir, swap, samples):
        print(f"  {name}  {frames:5d} frames  {secs:6.1f} s  shift {dy:+6.1f} px  height {keep}  imu {nimu:5d}")
