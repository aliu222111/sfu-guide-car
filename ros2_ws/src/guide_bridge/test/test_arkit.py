"""iPhone front camera: ARKit pose to base_link, velocities, depth registration."""

import math

import numpy as np
import pytest

from guide_bridge.arkit import (TwistTracker, arkit_camera_pose, base_pose, depth_points, prepare, quat_matrix,
                                rotate_image, rotate_K)


def test_arkit_start_pose_is_the_origin_heading():
    # ARKit's session starts with the camera looking along world -z: that is ROS yaw 0.
    assert base_pose(0, 0, 0, 1, 0, 0, 0) == pytest.approx((0, 0, 0))
    # Camera 1 m along ARKit -z (forward) and 0.5 m along +x (right).
    x, y, yaw = base_pose(0, 0, 0, 1, 0.5, 0, -1.0)
    assert (x, y, yaw) == pytest.approx((1.0, -0.5, 0.0))


def test_turning_left_is_positive_yaw():
    # Rotating +90 deg about ARKit's y (up) turns the view from -z to -x: left.
    q = (0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4))
    assert base_pose(*q, 0, 0, 0)[2] == pytest.approx(math.pi / 2)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.0, 3.0])
@pytest.mark.parametrize("pitch", [0.0, 0.2])
def test_round_trip_with_mount_offset(yaw, pitch):
    mx, my = 0.2, 0.03
    bx, by = 3.0, -1.5
    cx = bx + math.cos(yaw) * mx - math.sin(yaw) * my
    cy = by + math.sin(yaw) * mx + math.cos(yaw) * my
    pose = arkit_camera_pose(cx, cy, 0.45, yaw, pitch)
    R = quat_matrix(*pose[:4])
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9) and np.linalg.det(R) == pytest.approx(1.0)
    assert R[1, 1] > 0  # camera up has an upward component: the phone is upright
    assert base_pose(*pose, mx, my) == pytest.approx((bx, by, yaw), abs=1e-9)


def test_twist_in_body_frame_and_jump_reset():
    tr = TwistTracker(window=0.1)
    tw = None
    for k in range(20):  # 0.5 m/s forward along yaw 1.0 while turning at 0.2 rad/s
        t = k / 30
        yaw = 1.0 + 0.2 * t
        tw = tr.update(t, 0.5 * t * math.cos(1.0), 0.5 * t * math.sin(1.0), yaw) or tw
    assert tw.wz == pytest.approx(0.2, abs=1e-6)
    assert math.hypot(tw.vx, tw.vy) == pytest.approx(0.5, abs=1e-6)
    assert tw.vx > 0.49
    # A relocalisation jump of 3 m does not become a 90 m/s spike.
    assert tr.update(20 / 30, 3.0, 3.0, 1.1) is None and tr.jumps == 1


def test_prepare_registers_depth_and_scales_intrinsics():
    rgb = np.zeros((1440, 1920, 3), np.uint8)
    rgb[..., 0] = 255  # red in RGB
    depth = np.full((192, 256), 2.0, np.float32)
    depth[0, 0] = 9.0  # beyond max_depth
    conf = np.full((192, 256), 2, np.uint8)
    conf[1, 1] = 1  # not confident enough
    K = np.array([[1400.0, 0, 960], [0, 1400.0, 720], [0, 0, 1]])
    f = prepare(rgb, depth, conf, K, out_width=640, max_depth=5.0)
    assert f.bgr.shape == (480, 640, 3) and f.bgr[0, 0, 2] == 255  # BGR order
    assert f.depth.shape == (480, 640) and f.depth.dtype == np.float32
    assert f.K[0, 0] == pytest.approx(1400 / 3) and f.K[0, 2] == pytest.approx(320)
    assert f.K_small[0, 0] == pytest.approx(1400 * 256 / 1920) and f.K_small[1, 2] == pytest.approx(96)
    assert f.depth_small[0, 0] == 0 and f.depth_small[1, 1] == 0 and f.depth_small[5, 5] == 2.0


def test_points_of_a_wall_ahead():
    K = np.array([[200.0, 0, 128], [0, 200.0, 96], [0, 0, 1]])
    pts = depth_points(np.full((192, 256), 3.0, np.float32), K, stride=4)
    assert np.allclose(pts[:, 2], 3.0)
    assert pts[:, 0].min() == pytest.approx(-128 * 3 / 200) and pts[:, 1].min() == pytest.approx(-96 * 3 / 200)


def test_rotated_intrinsics_follow_the_pixels():
    img = np.zeros((40, 60), np.uint8)
    img[10, 50] = 1  # (u=50, v=10)
    K = np.array([[100.0, 0, 30], [0, 90.0, 20], [0, 0, 1]])
    for turns in range(4):
        r = rotate_image(img, turns)
        Kr = rotate_K(K, 60, 40, turns)
        v, u = map(int, np.argwhere(r == 1)[0])
        # The pixel's ray direction is the same before and after rotating.
        ray = ((50 - K[0, 2]) / K[0, 0], (10 - K[1, 2]) / K[1, 1])
        ray_r = ((u - Kr[0, 2]) / Kr[0, 0], (v - Kr[1, 2]) / Kr[1, 1])
        assert math.hypot(*ray) == pytest.approx(math.hypot(*ray_r), abs=0.02)
