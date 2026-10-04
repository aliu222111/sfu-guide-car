import math

import cv2
import numpy as np
import pytest

from guide_perception.ground import optical_to_base_rotation, project_to_ground
from guide_perception.ground_odometry import GroundOdometry, compose, rigid_2d

W, H = 320, 240
K = np.array([[250.0, 0, W / 2], [0, 250.0, H / 2], [0, 0, 1]])
R_CAM = optical_to_base_rotation(pitch_down=math.radians(15))
T_CAM = np.array([0.1, 0.0, 0.45])


def floor_texture(seed=1, size=2048, res=0.005):
    rng = np.random.default_rng(seed)
    t = rng.random((size, size)).astype(np.float32)
    t = cv2.GaussianBlur(t, (0, 0), 2.0)
    t = (255 * (t - t.min()) / (t.max() - t.min())).astype(np.uint8)
    return t, res


TEX, RES = floor_texture()
V, U = np.mgrid[0:H, 0:W]
PTS, OK = project_to_ground(K, R_CAM, T_CAM, U.ravel(), V.ravel(), 30.0)


def render(x, y, yaw):
    """Grey image of a textured floor seen from car pose (x, y, yaw); sky above."""
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * PTS[:, 0] - s * PTS[:, 1]
    wy = y + s * PTS[:, 0] + c * PTS[:, 1]
    i = np.clip(((wx + 5) / RES).astype(int), 0, TEX.shape[1] - 1)
    j = np.clip(((wy + 5) / RES).astype(int), 0, TEX.shape[0] - 1)
    img = np.where(OK, TEX[j, i], 200).astype(np.uint8)
    return img.reshape(H, W)


def test_rigid_2d_recovers_motion():
    rng = np.random.default_rng(0)
    a = rng.random((20, 2))
    th = 0.3
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    b = a @ R.T + [0.2, -0.1]
    R2, t2 = rigid_2d(a, b)
    assert np.allclose(R2, R) and np.allclose(t2, [0.2, -0.1])


def test_straight_and_turning_drive():
    odo = GroundOdometry(K, R_CAM, T_CAM, (H, W))
    pose = (0.0, 0.0, 0.0)
    truth = (0.0, 0.0, 0.0)
    odo.update(render(*truth))
    # 0.1 m forward per frame, then an arc turning 4 deg per frame.
    for k in range(40):
        dyaw = 0.0 if k < 20 else math.radians(4)
        x, y, yaw = truth
        truth = (x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw), yaw + dyaw)
        step = odo.update(render(*truth))
        assert step is not None, f"lost at frame {k}"
        pose = compose(pose, step)
    err = math.hypot(pose[0] - truth[0], pose[1] - truth[1])
    assert err < 0.05 * 4.0  # under 5 % of the 4 m driven
    assert abs(pose[2] - truth[2]) < math.radians(3)


def test_pivot_in_place():
    odo = GroundOdometry(K, R_CAM, T_CAM, (H, W))
    odo.update(render(0, 0, 0))
    total = 0.0
    for k in range(1, 11):
        step = odo.update(render(0, 0, math.radians(5 * k)))
        assert step is not None
        total += step.dyaw
        assert math.hypot(step.dx, step.dy) < 0.02
    assert total == pytest.approx(math.radians(50), abs=math.radians(1.5))


def test_blank_floor_is_lost():
    odo = GroundOdometry(K, R_CAM, T_CAM, (H, W))
    blank = np.full((H, W), 120, np.uint8)
    odo.update(blank)
    assert odo.update(blank) is None


def test_camera_seeing_no_floor_is_rejected():
    up = optical_to_base_rotation(pitch_down=math.radians(-40))
    with pytest.raises(ValueError):
        GroundOdometry(K, up, T_CAM, (H, W))
