import sys, time, cv2

def find():
    found = []
    for i in range(8):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if c.isOpened() and c.read()[0]:
            found.append(i)
        c.release()
    return found

def open_cam(i, w, h):
    c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    c.set(cv2.CAP_PROP_FPS, 15)
    return c

if len(sys.argv) < 3:
    print("cameras found at:", find())
    print("run: py stereo_test.py <left> <right> [720]")
    sys.exit()
L, R = int(sys.argv[1]), int(sys.argv[2])
w, h = (1280, 720) if "720" in sys.argv[3:] else (640, 360)
cl, cr = open_cam(L, w, h), open_cam(R, w, h)
n, t0 = 0, time.time()
while True:
    ok1, a = cl.read(); ok2, b = cr.read()
    if not (ok1 and ok2):
        print("a camera stopped (power or bandwidth?)"); break
    if a.shape != b.shape:
        print("sizes differ: left", a.shape[1], "x", a.shape[0], " right", b.shape[1], "x", b.shape[0])
        b = cv2.resize(b, (a.shape[1], a.shape[0]))
    view = cv2.hconcat([a, b])
    for y in range(0, view.shape[0], 40):      # alignment lines
        cv2.line(view, (0, y), (view.shape[1], y), (0, 255, 0), 1)
    n += 1
    fps = n / (time.time() - t0)
    cv2.putText(view, f"{a.shape[1]}x{a.shape[0]}  {fps:.1f} fps  L={L} R={R}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("stereo (s = save pair, q = quit)", view)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"): break
    if k == ord("s"):
        cv2.imwrite(f"left{n}.png", a); cv2.imwrite(f"right_{n}.png", b); print("saved pair", n)
cl.release(); cr.release(); cv2.destroyAllWindows()