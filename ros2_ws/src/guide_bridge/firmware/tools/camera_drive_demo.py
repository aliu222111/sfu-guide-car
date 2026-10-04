"""Camera -> laptop -> ESP32 pins, live: a person in view steers the car's DAC pins.

    pip install pyserial opencv-python ultralytics
    py camera_drive_demo.py --camera 0                      # preview only, no ESP32
    py camera_drive_demo.py --camera 0 --port COM5          # drive the ESP32 (steering only)
    py camera_drive_demo.py --camera 0 --port COM5 --throttle 0.2   # and throttle: WHEELS OFF THE GROUND
    py camera_drive_demo.py --camera 0 --track bright       # no YOLO: follow the brightest spot (a phone torch)

A stand-in for the whole ROS stack in one window: YOLO finds the biggest person, the
steering follows them (person on the left -> steer left -> GPIO25 goes LOW, on the right
-> HIGH, with guide_motor.ino's default voltages), and with --throttle the car creeps
forward while they are far (GPIO26 goes HIGH) and stops when they fill the frame. It sends
the same "C <steering> <throttle>" lines at 20 Hz as the car's serial bridge, and draws
the voltage each pin should be at, read from guide_motor.ino, so a multimeter on GPIO25 /
GPIO26 can be checked against the screen. Keys: space pauses (centres both), q quits.
Nobody in view, paused, or quitting: both pins go back to centre.
"""

import argparse
import os
import re
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
INO = os.path.join(HERE, "..", "guide_motor", "guide_motor.ino")
MODEL = os.path.join(HERE, *[".."] * 5, "models", "yolo11n.onnx")


def ino_volts(path=INO):
    """The six measured voltages and VREF from the firmware, so the screen matches the board."""
    v = {"VREF": 3.3, "STEER_CENTER_V": 1.65, "STEER_LEFT_V": 0.35, "STEER_RIGHT_V": 2.95,
         "THROTTLE_CENTER_V": 1.65, "THROTTLE_FULL_V": 2.95, "THROTTLE_REVERSE_V": 0.35}
    try:
        text = open(path, encoding="utf-8").read()
        for k in v:
            m = re.search(rf"const float {k} = ([0-9.]+)", text)
            if m:
                v[k] = float(m.group(1))
    except OSError:
        pass
    return v


def span(value, center, positive, negative):
    """guide_motor.ino's span(): linear from the centre to each measured end."""
    value = max(-1.0, min(1.0, value))
    return center + abs(value) * ((positive if value >= 0 else negative) - center)


def pin_volts(steer, thr, v):
    s = span(steer, v["STEER_CENTER_V"], v["STEER_LEFT_V"], v["STEER_RIGHT_V"])
    t = span(thr, v["THROTTLE_CENTER_V"], v["THROTTLE_FULL_V"], v["THROTTLE_REVERSE_V"])
    q = lambda x: round(min(max(x, 0), v["VREF"]) / v["VREF"] * 255)  # the DAC's 8-bit level
    return (s, q(s)), (t, q(t))


class Yolo:
    def __init__(self, model):
        from ultralytics import YOLO

        self.model = YOLO(model, task="detect")

    def __call__(self, bgr):
        r = self.model(bgr, classes=[0], conf=0.4, imgsz=640, verbose=False)[0]
        boxes = [tuple(b) for b in r.boxes.xyxy.tolist()] if r.boxes is not None else []
        return max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]), default=None)


def brightest(bgr):
    g = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (31, 31), 0)
    _, mx, _, (x, y) = cv2.minMaxLoc(g)
    if mx < 230:
        return None
    return (x - 30, y - 30, x + 30, y + 30)


def control(box, w, h, gain, max_thr):
    """Steering toward the box (+1 left), throttle while it is small (far)."""
    if box is None:
        return 0.0, 0.0
    off = ((box[0] + box[2]) / 2 - w / 2) / (w / 2)  # -1 left edge .. +1 right edge
    steer = max(-1.0, min(1.0, -gain * off))
    near = (box[3] - box[1]) / h  # how much of the frame the target fills
    thr = max_thr * max(0.0, min(1.0, (0.7 - near) / 0.3))  # full when < 40 % tall, 0 at 70 %
    return steer, thr


def draw_pin(img, x, name, role, volts, level, center, vref, low_word, high_word):
    """A vertical gauge 0..VREF with the centre marked and the live voltage."""
    top, bot = 70, img.shape[0] - 90
    y = lambda vv: int(bot - (bot - top) * vv / vref)
    cv2.rectangle(img, (x, top), (x + 60, bot), (90, 90, 90), 2)
    col = (60, 200, 255) if volts > center + 0.05 else (255, 160, 60) if volts < center - 0.05 else (200, 200, 200)
    cv2.rectangle(img, (x + 3, y(volts)), (x + 57, bot - 2), col, -1)
    cv2.line(img, (x - 6, y(center)), (x + 66, y(center)), (255, 255, 255), 1)
    cv2.putText(img, f"{center:.2f}", (x + 68, y(center) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(img, f"{vref:.1f}", (x + 68, top + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
    cv2.putText(img, "0", (x + 68, bot + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
    cv2.putText(img, name, (x - 4, top - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, role, (x - 4, top - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
    word = high_word if volts > center + 0.05 else low_word if volts < center - 0.05 else "centre"
    cv2.putText(img, f"{volts:.2f} V", (x - 4, bot + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
    cv2.putText(img, f"DAC {level}/255  {word}", (x - 4, bot + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


def compose(frame, box, steer, thr, v, line, replies, status):
    h, w = frame.shape[:2]
    view = frame.copy()
    cv2.line(view, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    if box is not None:
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(view, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cx = (x1 + x2) // 2
        cv2.arrowedLine(view, (w // 2, h - 30), (cx, h - 30), (0, 255, 0), 3, tipLength=0.15)
    cv2.putText(view, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    panel = np.full((480, 400, 3), 30, np.uint8)
    (sv, sl), (tv, tl) = pin_volts(steer, thr, v)
    draw_pin(panel, 40, "GPIO25", "steering wiper", sv, sl, v["STEER_CENTER_V"], v["VREF"],
             "left" if v["STEER_LEFT_V"] < v["STEER_CENTER_V"] else "right",
             "right" if v["STEER_RIGHT_V"] > v["STEER_CENTER_V"] else "left")
    draw_pin(panel, 210, "GPIO26", "throttle wiper", tv, tl, v["THROTTLE_CENTER_V"], v["VREF"],
             "reverse", "forward")
    if h != panel.shape[0]:  # the camera image to the panel's height, aspect kept
        view = cv2.resize(view, (round(w * panel.shape[0] / h), panel.shape[0]))
    out = np.hstack([view, panel])
    bar = np.full((70, out.shape[1], 3), 15, np.uint8)
    cv2.putText(bar, f"serial -> ESP32:  {line}", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (120, 255, 120), 2)
    cv2.putText(bar, f"ESP32 -> laptop:  {replies[-1] if replies else '(nothing yet)'}", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
    return np.vstack([out, bar])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="0", help="camera index, or a video file")
    ap.add_argument("--port", help="the ESP32's serial port (COM5, /dev/ttyUSB0); none: preview only")
    ap.add_argument("--track", choices=["person", "bright"], default="person")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--gain", type=float, default=1.5, help="steering per unit of off-centre (1: full lock at the frame edge)")
    ap.add_argument("--throttle", type=float, default=0.0, help="max throttle 0..1 (default 0: steering only)")
    ap.add_argument("--record", help="also write the window to this video file")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames (0: run until q)")
    ap.add_argument("--headless", action="store_true", help="no window (with --record)")
    a = ap.parse_args(argv)

    v = ino_volts()
    detect = Yolo(a.model) if a.track == "person" else brightest
    src = int(a.camera) if a.camera.isdigit() else a.camera
    cap = cv2.VideoCapture(src, cv2.CAP_DSHOW) if os.name == "nt" and isinstance(src, int) else cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"camera {a.camera} did not open")
        return 1

    ser = None
    if a.port:
        import serial

        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = a.port, 115200, 0
        ser.dtr = ser.rts = False  # do not reset the ESP32 on open
        ser.open()
        time.sleep(0.5)
    buf, replies = b"", []

    def send(steer, thr):
        nonlocal buf
        line = f"C {steer:.3f} {thr:.3f}"
        if ser:
            ser.write((line + "\n").encode())
            buf += ser.read(256)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                if text:
                    replies.append(text + ("   <- voltages above 3.3 V in guide_motor.ino: not driving" if text.startswith("E") else ""))
                    print("ESP32:", replies[-1])
        return line

    writer, paused, n, last_send, line = None, False, 0, 0.0, "C 0.000 0.000"
    steer = thr = 0.0
    print(f"pins from {os.path.normpath(INO)}: steering {v['STEER_LEFT_V']}/{v['STEER_CENTER_V']}/{v['STEER_RIGHT_V']} V, "
          f"throttle {v['THROTTLE_REVERSE_V']}/{v['THROTTLE_CENTER_V']}/{v['THROTTLE_FULL_V']} V")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            box = None if paused else detect(frame)
            steer, thr = (0.0, 0.0) if paused else control(box, frame.shape[1], frame.shape[0], a.gain, a.throttle)
            # 20 Hz like the serial bridge, however fast the camera runs; YOLO on a CPU is
            # slower than that, so resend the latest command to keep the watchdog fed.
            now = time.time()
            if now - last_send >= 0.05:
                line, last_send = send(steer, thr), now
            status = "PAUSED (space)" if paused else ("tracking" if box is not None else "nobody in view: centred")
            status += "" if ser else "   [preview: no --port]"
            img = compose(frame, box, steer, thr, v, line, replies, status)
            if a.record:
                if writer is None:
                    writer = cv2.VideoWriter(a.record, cv2.VideoWriter_fourcc(*"mp4v"), 15, (img.shape[1], img.shape[0]))
                writer.write(img)
            n += 1
            if a.frames and n >= a.frames:
                break
            if not a.headless:
                cv2.imshow("camera -> ESP32 pins", img)
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord(" "):
                    paused = not paused
    except KeyboardInterrupt:
        pass
    finally:
        send(0.0, 0.0)  # centre now rather than waiting for the 300 ms watchdog
        if ser:
            ser.close()
        if writer:
            writer.release()
        cap.release()
        if not a.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
