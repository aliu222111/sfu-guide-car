"""The simulated building, cameras and devices, checked against the real parsers where they exist."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from guide_sim.devices import Phone, PtyPort, nmea_sentences, utc_at
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
    from sfu_map.geo import xy_to_lonlat

    lon, lat = (float(v) for v in xy_to_lonlat(100.0, -50.0))
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


def test_render_depth_matches_geometry():
    fw = corridor(length=12.0, width=3.0)
    cam = CameraModel(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0, baseline=0.0, noise=0.0, pitch=0.2)
    img, depth = Renderer(cam).render(fw, 0.5, 0.0, 0.0, "center", with_depth=True)
    assert img.shape == depth.shape == (480, 640)
    # Bottom centre pixel sees the floor: its ray meets z = 0 at forward distance
    # cam.z / tan(pitch + angle below the axis); depth is along the optical axis.
    below = math.atan2(479.5 - cam.cy, cam.fy)
    ahead = cam.z / math.tan(cam.pitch + below)
    assert depth[479, 320] == pytest.approx(ahead / math.cos(cam.pitch + below) * math.cos(below), rel=0.03)
    # The row looking level sees the far wall at x = 12.
    level_row = int(cam.cy - cam.fy * math.tan(cam.pitch))
    expect = (12.0 - 0.5 - cam.x) * math.cos(cam.pitch)
    assert depth[level_row, 320] == pytest.approx(expect, rel=0.03)


def test_iphone_driver_reads_the_simulated_phone(tmp_path, monkeypatch):
    """The real iPhone driver, through fake_record3d, turns the simulated stream into /vo and points."""
    import time

    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    from guide_bridge.arkit import arkit_camera_pose
    from guide_sim.devices import Record3DServer

    sock = str(tmp_path / "r3d.sock")
    monkeypatch.setenv("R3D_SIM_SOCKET", sock)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "guide_sim" / "fake_record3d"))
    sys.modules.pop("record3d", None)
    server = Record3DServer(sock)
    rclpy.init()
    try:
        from guide_bridge.iphone_node import IphoneNode

        node = IphoneNode()
        got = {"vo": [], "points": []}
        node.create_subscription(Odometry, "/vo", got["vo"].append, 50)
        node.create_subscription(PointCloud2, "front/points", got["points"].append, 5)
        deadline = time.monotonic() + 5.0
        while not server.clients and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert server.clients, "driver never connected"
        rgb = np.zeros((480, 640, 3), np.uint8)
        depth = np.full((192, 256), 2.0, np.float32)
        conf = np.full((192, 256), 2, np.uint8)
        yaw = 0.8
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:  # 0.4 m/s forward, phone 0.2 m ahead of base_link
            d = 0.4 * (time.monotonic() - t0)
            pose = arkit_camera_pose((d + 0.2) * math.cos(yaw), (d + 0.2) * math.sin(yaw), 0.45, yaw, 0.2)
            server.send(rgb, depth, conf, (500.0, 500.0, 320.0, 240.0), pose)
            end = time.monotonic() + 1 / 30
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.005)
        vx = [m.twist.twist.linear.x for m in got["vo"]][5:]
        assert vx and np.median(vx) == pytest.approx(0.4, abs=0.08)
        assert abs(np.median([m.twist.twist.linear.y for m in got["vo"]])) < 0.05
        assert got["points"] and got["points"][0].width > 1000
        node.destroy_node()
    finally:
        rclpy.shutdown()
        server.close()
