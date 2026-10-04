# py stereo_record.py <left camera index> <right camera index> [name] [--fps 30]
# Left and right as seen from behind the rig, looking where it looks. In test1 they were swapped:
# a thing seen by both is further right in the left picture than in the right one, never the reverse.
import sys, time, cv2
L, R = int(sys.argv[1]), int(sys.argv[2])
args = [a for a in sys.argv[3:] if not a.startswith("--")]
name = args[0] if args else time.strftime("walk%Y%m%d%H%M%S")
FPS = int(next((sys.argv[i + 1] for i, a in enumerate(sys.argv) if a == "--fps"), 30))  # MJPEG 640x360: 30 on Logitech webcams
def cam(i):
    c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    c.set(cv2.CAP_PROP_FPS, FPS)
    return c
cl, cr = cam(L), cam(R)
four = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
wl = cv2.VideoWriter(name + "_left.avi", four, FPS, (640, 360))
wr = cv2.VideoWriter(name + "_right.avi", four, FPS, (640, 360))
ts = open(name + "_times.csv", "w"); ts.write("frame,unix_time\n")
n = 0
print(f"recording {name} at {FPS} fps asked - q in the window stops, d opens the cameras' own settings")
t_check, n_check = time.time(), 0
while True:
    # grab() both first, decode after: the two pictures are taken a few ms apart, not a whole frame
    g1 = cl.grab(); t = time.time(); g2 = cr.grab()
    ok1, a = cl.retrieve() if g1 else (False, None)
    ok2, b = cr.retrieve() if g2 else (False, None)
    if not (ok1 and ok2):
        print("a camera stopped at frame", n); break
    wl.write(a); wr.write(b); ts.write(f"{n},{t:.3f}\n"); n += 1
    if t - t_check >= 10:
        got = (n - n_check) / (t - t_check)
        warn = "" if got >= 0.85 * FPS else ("  <- below what was asked: dim light makes the cameras' auto exposure slow them down"
                                            " (more light, or d: turn off 'low light compensation'), or the USB hub/laptop is too slow")
        print(f"  {n} frames, {got:.1f} fps{warn}", flush=True)
        t_check, n_check = t, n
    if n % 5 == 0:
        cv2.imshow("recording (q = stop)", cv2.resize(cv2.hconcat([a, b]), (640, 180)))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"): break
        if k == ord("d"): cl.set(cv2.CAP_PROP_SETTINGS, 1); cr.set(cv2.CAP_PROP_SETTINGS, 1)
for x in (cl, cr, wl, wr): x.release()
ts.close(); cv2.destroyAllWindows()
print(n, "frames saved")