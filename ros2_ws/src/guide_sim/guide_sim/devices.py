"""The car's hardware links, simulated at the wire, so the real drivers run unchanged.

- ``PtyPort``: a pseudo-terminal. The serial bridge (pyserial) and the GPS
  driver open its device path exactly as they would open /dev/ttyACM0 and
  /dev/ttyUSB0; the simulator holds the other end.
- ``Phone``: posts Sensor Logger JSON to the phone bridge's HTTP port, with the
  phone's own clock, axes, gyro bias and noise, batched as the app does.
- ``nmea_sentences``: what the Adafruit GPS prints (GGA and RMC).
"""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import tty
import urllib.request
from datetime import datetime, timezone

from guide_bridge.phone_imu import quat_mul

class PtyPort:
    """A serial device for a driver to open; ``path`` is a stable symlink to it."""

    def __init__(self, link: str):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # no echo, no line editing: a plain byte pipe
        os.set_blocking(self.master, False)
        self.device = os.ttyname(self.slave)
        os.makedirs(os.path.dirname(link), exist_ok=True)
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(self.device, link)
        self.path = link

    def read(self) -> bytes:
        try:
            return os.read(self.master, 65536)
        except (BlockingIOError, OSError):
            return b""

    def write(self, data: bytes):
        if data:
            try:
                os.write(self.master, data)
            except OSError:
                pass  # nobody has the port open yet: the bytes are lost, as on a real wire

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(self.path):
            os.remove(self.path)


def _checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def _nmea(body: str) -> bytes:
    return f"${body}*{_checksum(body)}\r\n".encode("ascii")


def _dm(value: float, lat: bool) -> tuple[str, str]:
    hemi = ("N" if value >= 0 else "S") if lat else ("E" if value >= 0 else "W")
    v = abs(value)
    deg = int(v)
    minutes = (v - deg) * 60
    return (f"{deg:02d}{minutes:07.4f}" if lat else f"{deg:03d}{minutes:07.4f}"), hemi


def nmea_sentences(utc: datetime, lat: float | None, lon: float | None, speed: float = 0.0,
                   course_deg: float = 0.0, sats: int = 8, hdop: float = 1.2) -> bytes:
    """GGA + RMC as an MTK3339 prints them; ``lat is None`` means no fix (indoors)."""
    hms = utc.strftime("%H%M%S.") + f"{utc.microsecond // 10000:02d}"
    date = utc.strftime("%d%m%y")
    if lat is None:
        return _nmea(f"GPGGA,{hms},,,,,0,00,99.9,,M,,M,,") + _nmea(f"GPRMC,{hms},V,,,,,,,{date},,,N")
    la, ns = _dm(lat, True)
    lo, ew = _dm(lon, False)
    knots = speed / 0.514444
    return (_nmea(f"GPGGA,{hms},{la},{ns},{lo},{ew},1,{sats:02d},{hdop:.1f},120.0,M,-17.0,M,,")
            + _nmea(f"GPRMC,{hms},A,{la},{ns},{lo},{ew},{knots:.2f},{course_deg:.1f},{date},,,A"))


# Phone lies flat, screen up, top edge forward: phone = (-base_y, base_x, base_z).
_Q_BASE_PHONE = (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))  # inverse of guide_bridge's phone-from-base
PHONE_EPOCH = 1_790_000_000.0  # s: the phone's wall clock when the simulation starts


class Phone:
    """Sensor Logger on the strapped-on phone, pushing over WiFi in the background."""

    def __init__(self, url: str = "http://127.0.0.1:8765/data", rate_hz: float = 50.0, push_every: float = 0.2,
                 gyro_bias: float = 0.004, gyro_noise: float = 0.01, compass_error_deg: float = 20.0, seed: int = 1):
        import random

        self.url = url
        self.period = 1.0 / rate_hz
        self.push_every = push_every
        self.gyro_bias = gyro_bias
        self.gyro_noise = gyro_noise
        self.compass_error = math.radians(compass_error_deg)  # indoors steel bends the compass
        self.rand = random.Random(seed)
        self.batch: list[dict] = []
        self.next_sample = 0.0
        self.next_push = 0.0
        self.q: queue.Queue = queue.Queue(maxsize=50)
        self.posted = 0
        self.failed = 0
        self.enabled = True
        threading.Thread(target=self._sender, daemon=True).start()

    def _sender(self):
        while True:
            body = self.q.get()
            try:
                req = urllib.request.Request(self.url, data=body, headers={"Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=2.0).read()
                self.posted += 1
            except Exception:
                self.failed += 1  # the bridge is not up yet, or WiFi dropped: the app just carries on

    def update(self, t: float, yaw: float, w: float, accel_fwd: float, outdoors: bool,
               fix: tuple[float, float] | None = None):
        """Called every simulation step with the true body state."""
        while self.next_sample <= t:
            ts = int((PHONE_EPOCH + self.next_sample) * 1e9)
            gz = w + self.gyro_bias + self.rand.gauss(0, self.gyro_noise)
            # base -> phone axes: phone = (-base_y, base_x, base_z)
            gyro = {"x": self.rand.gauss(0, self.gyro_noise), "y": self.rand.gauss(0, self.gyro_noise), "z": gz}
            acc_b = (accel_fwd, 0.0, 9.81)
            acc = {"x": -acc_b[1] + self.rand.gauss(0, 0.05), "y": acc_b[0] + self.rand.gauss(0, 0.05), "z": acc_b[2] + self.rand.gauss(0, 0.05)}
            heading = yaw + (0.0 if outdoors else self.compass_error)
            q_world_base = (0.0, 0.0, math.sin(heading / 2), math.cos(heading / 2))
            qx, qy, qz, qw = quat_mul(q_world_base, _Q_BASE_PHONE)
            self.batch += [
                {"name": "gyroscope", "time": ts, "values": gyro},
                {"name": "totalacceleration", "time": ts, "values": acc},
                {"name": "orientation", "time": ts, "values": {"qx": qx, "qy": qy, "qz": qz, "qw": qw}},
            ]
            if fix is not None and int(self.next_sample * 1000) % 1000 < int(self.period * 1000):
                lat, lon = fix
                self.batch.append({"name": "location", "time": ts, "values": {
                    "latitude": lat, "longitude": lon, "altitude": 120.0,
                    "horizontalAccuracy": 5.0, "verticalAccuracy": 8.0}})
            self.next_sample += self.period
        if t >= self.next_push:
            self.next_push = t + self.push_every
            if self.batch and self.enabled:
                try:
                    self.q.put_nowait(json.dumps({"payload": self.batch}).encode())
                except queue.Full:
                    pass
            self.batch = []


def utc_at(t: float) -> datetime:
    return datetime.fromtimestamp(PHONE_EPOCH + t, tz=timezone.utc)
