"""Turn Sensor Logger HTTP pushes into IMU, magnetometer and GPS samples (spec section 3).

Sensor Logger posts JSON ``{"payload": [{"name": ..., "time": <ns>, "values": {...}}, ...]}``.
The phone lies flat, screen up, top edge pointing forward. Phone axes (Android
and Sensor Logger's normalised iOS output): x to the right edge, y to the top
edge, z out of the screen. base_link (REP-103): x forward, y left, z up. So
base = (phone_y, -phone_x, phone_z).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Rotation of +90 deg about z, taking base_link vectors to phone vectors.
_Q_PHONE_FROM_BASE = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # x, y, z, w


def phone_to_base(v):
    x, y, z = v
    return (y, -x, z)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def base_orientation(q_world_phone, world_yaw_offset: float = 0.0):
    """Orientation of base_link in ENU from the phone's orientation quaternion.

    Android's rotation vector is East-North-Up with *magnetic* north, so
    ``world_yaw_offset`` (minus the magnetic declination, see guide.yaml) turns
    it into true ENU; it also rotates other reference frames onto ENU.
    """
    q = quat_mul(q_world_phone, _Q_PHONE_FROM_BASE)
    if world_yaw_offset:
        q = quat_mul(yaw_quat(world_yaw_offset), q)
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


@dataclass
class ImuSample:
    t: float  # phone clock, seconds
    gyro: tuple | None = None  # rad/s, base frame
    accel: tuple | None = None  # m/s^2 including gravity, base frame
    orientation: tuple | None = None  # base in ENU, (x, y, z, w)


@dataclass
class MagSample:
    t: float
    field: tuple  # tesla, base frame


@dataclass
class FixSample:
    t: float
    lat: float
    lon: float
    alt: float
    horizontal_accuracy: float
    vertical_accuracy: float


def _num(v) -> float:
    x = float(v)
    if not math.isfinite(x):
        raise ValueError("non-finite")
    return x


def _vec(v: dict, keys=("x", "y", "z")) -> tuple:
    return tuple(_num(v[k]) for k in keys)


# Phone clock sanity: nanoseconds since 2000 and before 2100.
_T_MIN, _T_MAX = 946_684_800 * 10**9, 4_102_444_800 * 10**9


def parse_payload(doc, accel_scale: float = 1.0, world_yaw_offset: float = 0.0):
    """Group one push into samples. Returns (imu_samples, mag_samples, fix_samples).

    Anything malformed (wrong types, non-finite numbers, absurd timestamps) is
    skipped: the HTTP port is open to the whole WiFi network.
    """
    by_time: dict[int, dict] = {}
    mags, fixes = [], []
    payload = doc.get("payload") if isinstance(doc, dict) else None
    if not isinstance(payload, list):
        return [], [], []
    for item in payload:
        if not isinstance(item, dict) or not isinstance(item.get("name"), str) or not isinstance(item.get("values"), dict):
            continue
        name = item["name"].lower()
        v = item["values"]
        try:
            t_ns = int(item["time"])
            if not _T_MIN <= t_ns <= _T_MAX:
                continue
            t = t_ns * 1e-9
            if name == "gyroscope":
                by_time.setdefault(t_ns, {})["gyro"] = phone_to_base(_vec(v))
            elif name in ("totalacceleration", "accelerometer", "gravity"):
                by_time.setdefault(t_ns, {})[name] = _vec(v)
            elif name == "orientation":
                q = _vec(v, ("qx", "qy", "qz", "qw"))
                if sum(c * c for c in q) < 1e-6:
                    continue
                by_time.setdefault(t_ns, {})["orientation"] = base_orientation(q, world_yaw_offset)
            elif name == "magnetometer":
                mags.append(MagSample(t, tuple(c * 1e-6 for c in phone_to_base(_vec(v)))))
            elif name == "location":
                lat, lon = _num(v["latitude"]), _num(v["longitude"])
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                    continue
                fixes.append(
                    FixSample(
                        t,
                        lat,
                        lon,
                        _num(v.get("altitude", 0.0) or 0.0),
                        _num(v.get("horizontalAccuracy", 10.0) or 10.0),
                        _num(v.get("verticalAccuracy", 20.0) or 20.0),
                    )
                )
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    imus = []
    for t_ns, d in sorted(by_time.items()):
        accel = None
        if "totalacceleration" in d:
            accel = d["totalacceleration"]
        elif "accelerometer" in d and "gravity" in d:
            accel = tuple(a + g for a, g in zip(d["accelerometer"], d["gravity"]))
        if accel is not None:
            accel = phone_to_base(tuple(c * accel_scale for c in accel))
        if d.get("gyro") is None and accel is None and d.get("orientation") is None:
            continue
        imus.append(ImuSample(t_ns * 1e-9, d.get("gyro"), accel, d.get("orientation")))
    return imus, mags, fixes


class ClockOffset:
    """Phone clock to laptop clock (spec section 6: estimated once at start).

    Uses the minimum of (receive time - phone time of the newest sample in the
    push) over the first ``n`` pushes: the offset plus the smallest network
    delay seen. Using the newest sample keeps a batched push (Sensor Logger
    sends up to a second at once) from adding the batch length to the offset.
    """

    def __init__(self, n: int = 50):
        self.n = n
        self.count = 0
        self.offset: float | None = None

    def update(self, phone_t: float, recv_t: float) -> float:
        if self.count < self.n:
            d = recv_t - phone_t
            self.offset = d if self.offset is None else min(self.offset, d)
            self.count += 1
        return self.offset

    def to_local(self, phone_t: float) -> float:
        return phone_t + (self.offset or 0.0)


class Liveness:
    """True while pushes keep arriving; the EKF falls back after ``timeout`` (spec: 1 s)."""

    def __init__(self, timeout: float = 1.0):
        self.timeout = timeout
        self.last = -math.inf

    def seen(self, now: float):
        self.last = now

    def alive(self, now: float) -> bool:
        return now - self.last <= self.timeout
