"""The simulated building, cameras and devices, checked against the real parsers where they exist."""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

from guide_sim.devices import Phone, PtyPort, nmea_sentences, utc_at, xy_to_latlon
from guide_sim.render import CameraModel, Renderer
from guide_sim.world import Box, FloorWorld, Grid, Person

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "guide_bridge"))


def corridor(length=20.0, width=2.0, res=0.05):
    """A straight corridor along x from 0 to ``length``, centred on y = 0."""
    h, w = int(10 / res), int((length + 4) / res)
    occ = np.ones((h, w), dtype=bool)
    g = Grid(occ, res, -2.0, -5.0)
    rows, cols = np.mgrid[0:h, 0:w]
    x = g.ox + (cols + 0.5) * res
    y = g.oy + (h - rows - 0.5) * res
    occ[(x > 0) & (x < length) & (np.abs(y) < width / 2)] = False
    return FloorWorld("TEST", g)


def test_raycast_walls():
    fw = corridor()
    h = fw.raycast(1.0, 0.0, np.array([0.0, math.pi / 2, math.pi]), max_range=30.0)
    assert h.distance[0] == pytest.approx(19.0, abs=0.05)  # far end
    assert h.distance[1] == pytest.approx(1.0, abs=0.05)  # side wall
    assert h.distance[2] == pytest.approx(1.0, abs=0.05)  # near end


def test_people_and_boxes_block():
    fw = corridor()
    fw.people.append(Person((5.0, 0.0), (5.0, 0.0), speed=0.0))
    assert fw.raycast(1.0, 0.0, np.array([0.0])).distance[0] == pytest.approx(3.75, abs=0.01)
    fw.add_box(Box(3.0, 0.0, 0.2, 0.2))
    assert fw.raycast(1.0, 0.0, np.array([0.0])).distance[0] == pytest.approx(1.8, abs=0.06)
    assert fw.collides([(3.0, 0.0)], 0.0)


def test_render_stereo_disparity_matches_geometry():
    import cv2

    fw = corridor(length=12.0, width=3.0)
    cam = CameraModel(noise=0.0, pitch=0.0)
    r = Renderer(cam)
    L = r.render(fw, 0.5, 0.0, 0.0, "left")
    R = r.render(fw, 0.5, 0.0, 0.0, "right")
    d = cv2.StereoSGBM_create(0, 64, 7).compute(L, R).astype(float) / 16
    # The far wall at x = 12 is 12 - 0.5 - cam.x deep: disparity fx * B / depth.
    depth = 12.0 - 0.5 - cam.x
    centre = d[int(cam.cy) - 20:int(cam.cy) - 5, int(cam.cx) - 10:int(cam.cx) + 10]
    assert np.median(centre[centre > 0]) == pytest.approx(cam.fx * cam.baseline / depth, abs=0.6)


def test_phone_payload_parses_with_the_real_bridge():
    from guide_bridge.phone_imu import parse_payload, quat_yaw

    phone = Phone(url="http://127.0.0.1:9/none", gyro_bias=0.0, gyro_noise=0.0, compass_error_deg=0.0)
    phone.enabled = False
    for k in range(10):
        phone.update(k * 0.02, yaw=0.7, w=0.25, accel_fwd=0.0, outdoors=True)
    doc = {"payload": phone.batch}
    imus, _, _ = parse_payload(json.loads(json.dumps(doc)))
    assert imus and imus[0].gyro[2] == pytest.approx(0.25)
    assert quat_yaw(imus[0].orientation) == pytest.approx(0.7, abs=1e-6)
    assert imus[0].accel[2] == pytest.approx(9.81, abs=0.3)


def _nmea_ok(line: bytes) -> bool:
    body, cs = line.decode().strip()[1:].split("*")
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}" == cs


def test_nmea_fix_and_no_fix():
    lat, lon = xy_to_latlon(100.0, -50.0)
    lines = nmea_sentences(utc_at(10.0), lat, lon, speed=1.0, course_deg=45.0).split(b"\r\n")[:-1]
    assert [l[:6] for l in lines] == [b"$GPGGA", b"$GPRMC"] and all(_nmea_ok(l) for l in lines)
    f = lines[0].decode().split(",")
    got_lat = int(f[2][:2]) + float(f[2][2:]) / 60
    got_lon = -(int(f[4][:3]) + float(f[4][3:]) / 60)
    assert got_lat == pytest.approx(lat, abs=1e-6) and got_lon == pytest.approx(lon, abs=1e-6)
    none = nmea_sentences(utc_at(10.0), None, None).split(b"\r\n")[0].decode().split(",")
    assert none[6] == "0"  # GGA fix quality 0


def test_pty_is_a_serial_port(tmp_path):
    import serial

    port = PtyPort(str(tmp_path / "motor"))
    try:
        s = serial.Serial(port.path, 115200, timeout=0.5)
        s.write(b"V 0.300 0.000\n")
        import time

        time.sleep(0.05)
        assert port.read() == b"V 0.300 0.000\n"
        port.write(b"U 87\r\n")
        assert s.readline() == b"U 87\r\n"
        s.close()
    finally:
        port.close()
