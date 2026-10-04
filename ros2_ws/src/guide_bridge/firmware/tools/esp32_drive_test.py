"""Drive the ESP32 by hand, without ROS: checks USB serial -> ESP32 -> controller -> car.

    pip install pyserial
    py esp32_drive_test.py                       # list serial ports (the ESP32 is usually COM3..COM9)
    py esp32_drive_test.py COM5 steer            # sweep the steering left-right-left for 12 s, no throttle
    py esp32_drive_test.py COM5 hold 1.0 0       # hold full left (steering 1.0) for 20 s: measure GPIO25
    py esp32_drive_test.py COM5 hold -1.0 0      # full right
    py esp32_drive_test.py COM5 hold 0 0.2       # a little throttle: WHEELS OFF THE GROUND

Sends "C <steering> <throttle>" 20 times a second, as the car's serial bridge does (the
firmware centres everything 300 ms after the last line), and prints what the ESP32 sends
back. Steering +1 is left, -1 right; throttle +1 forward. With guide_motor.ino's default
voltages GPIO25 should read about 0.35 V (left), 1.65 V (centre), 2.95 V (right).
"""

import sys
import time


def ports():
    from serial.tools import list_ports

    for p in list_ports.comports():
        print(f"  {p.device:8s} {p.description}")


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        print("serial ports:")
        ports()
        return 1
    import serial

    port, mode = argv[0], (argv[1] if len(argv) > 1 else "steer")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0
    s.dtr = s.rts = False  # opening with DTR/RTS low does not reset the ESP32 (as the car's bridge does)
    s.open()
    time.sleep(0.5)
    print(f"opened {port}; Ctrl+C stops (the ESP32 centres everything 0.3 s later)")
    buf = b""

    def send(steer, thr):
        nonlocal buf
        s.write(f"C {max(-1, min(1, steer)):.3f} {max(-1, min(1, thr)):.3f}\n".encode())
        buf += s.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            if text:
                note = "  <- the firmware's measured voltages are above 3.3 V: it will not drive" if text.startswith("E") else ""
                print(f"  ESP32 says: {text}{note}")

    try:
        if mode == "steer":
            t0 = time.time()
            while time.time() - t0 < 12:
                t = time.time() - t0
                steer = [0, 1, 0, -1][int(t // 1.5) % 4] if t < 6 else ((t - 6) / 1.5) % 2 - 1  # steps, then a ramp
                send(steer, 0.0)
                print(f"\r  steering {steer:+.2f} ({'left' if steer > 0.05 else 'right' if steer < -0.05 else 'centre'})   ", end="", flush=True)
                time.sleep(0.05)
            print()
        elif mode == "hold":
            steer, thr = float(argv[2]), float(argv[3]) if len(argv) > 3 else 0.0
            print(f"  holding steering {steer:+.2f}, throttle {thr:+.2f} for 20 s")
            t0 = time.time()
            while time.time() - t0 < 20:
                send(steer, thr)
                time.sleep(0.05)
        else:
            print("mode is steer or hold"); return 1
    except KeyboardInterrupt:
        pass
    for _ in range(5):
        send(0.0, 0.0); time.sleep(0.05)
    s.close()
    print("centred and closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
