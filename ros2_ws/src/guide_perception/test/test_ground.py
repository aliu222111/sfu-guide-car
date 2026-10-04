import math

import numpy as np

from guide_perception.ground import free_boundary, optical_to_base_rotation, project_to_ground
from guide_perception.people import Person, locate, nearest_ahead

K = np.array([[500.0, 0, 320], [0, 500.0, 180], [0, 0, 1]])


def test_centre_pixel_below_horizon_hits_ground_ahead():
    R = optical_to_base_rotation(pitch_down=math.radians(20))
    t = np.array([0.1, 0.0, 0.5])
    pts, ok = project_to_ground(K, R, t, np.array([320.0]), np.array([180.0]))
    assert ok[0]
    # Optical axis 20 deg down from 0.5 m: hits 0.5 / tan(20 deg) ahead of the camera.
    assert abs(pts[0, 0] - (0.1 + 0.5 / math.tan(math.radians(20)))) < 1e-6
    assert abs(pts[0, 1]) < 1e-9 and abs(pts[0, 2]) < 1e-9


def test_above_horizon_invalid_and_right_pixels_go_right():
    R = optical_to_base_rotation(0.0)
    t = np.array([0, 0, 0.5])
    _, ok = project_to_ground(K, R, t, np.array([320.0]), np.array([100.0]))
    assert not ok[0]
    pts, ok = project_to_ground(K, R, t, np.array([600.0]), np.array([350.0]))
    assert ok[0] and pts[0, 1] < 0  # image right = base_link -y


def test_free_boundary():
    m = np.ones((10, 8), dtype=bool)
    m[:6, :4] = False  # obstacle reaches down to row 5 on the left
    cols, rows, hit = free_boundary(m, start_row=0, step=2)
    assert list(cols) == [1, 3, 5, 7]
    assert list(rows[:2]) == [5, 5] and list(hit) == [True, True, False, False]


def test_people_distance():
    R = optical_to_base_rotation(math.radians(10))
    t = np.array([0.0, 0.0, 0.5])
    near = Person(1, (300, 100, 340, 300), 0.9)
    side = Person(2, (0, 100, 40, 260), 0.9)
    clipped = Person(3, (280, 50, 360, 360), 0.9)
    locate([near, side, clipped], K, R, t, 360)
    assert 1.0 < near.distance < 3.0 and abs(near.y) < 0.1
    assert side.y > 0.5
    assert clipped.x == 0.5
    assert nearest_ahead([near, side], half_width=0.5) == near.distance
    assert nearest_ahead([]) == math.inf


def test_pick_student_keeps_lock():
    from guide_perception.student_node import pick_student

    a = Person(1, (0, 0, 1, 1), 0.9, x=-2.0, y=0.0)
    b = Person(2, (0, 0, 1, 1), 0.9, x=-1.0, y=0.0)
    assert pick_student([a, b], None).track_id == 2
    assert pick_student([a, b], 1).track_id == 1
    assert pick_student([Person(3, (0, 0, 1, 1), 0.9, x=-5.0)], None) is None
    stranger = Person(3, (0, 0, 1, 1), 0.9, x=-1.0, y=0.0)
    assert pick_student([stranger], 1, lost_for=0.2, last_distance=2.0) is None  # occluded: no hand-over
    assert pick_student([stranger], 1, lost_for=5.0, last_distance=2.5) is None  # gone long, but not where the student was
    assert pick_student([stranger], 1, lost_for=5.0, last_distance=1.2).track_id == 3  # new id, same place: re-lock


def test_nominal_camera_places_feet_on_the_ground():
    """The fallback before calibration: the URDF left camera (28 cm up, 5 deg up) and a 70 deg lens."""
    from guide_perception.ground import mount_pose, nominal_K
    from guide_perception.people import Person, locate, nearest_ahead

    K = nominal_K(640, 360, 70.0)
    R, t = mount_pose(0.50, 0.05, 0.28, -0.09, 0.0)
    for ahead, left in ((2.0, 0.0), (3.5, 0.6), (6.0, -1.0)):
        foot = np.array([ahead, left, 0.0])
        cam = R.T @ (foot - t)  # base_link -> optical frame
        u, v = (K @ cam)[:2] / cam[2]
        p = locate([Person(None, (u - 20, v - 150, u + 20, v), 0.9)], K, R, t, 360)[0]
        assert abs(p.x - ahead) < 0.05 and abs(p.y - left) < 0.05
    # a person filling the bottom of the picture is right in front: stop
    near = locate([Person(None, (300, 50, 340, 359), 0.9)], K, R, t, 360)
    assert nearest_ahead(near) == 0.5


def test_rear_camera_mount_faces_backwards():
    from guide_perception.ground import mount_pose, nominal_K, project_to_ground

    K = nominal_K(640, 360)
    R, t = mount_pose(-0.12, 0.0, 0.28, 0.1, math.pi)
    pts, ok = project_to_ground(K, R, t, np.array([320.0]), np.array([300.0]))
    assert ok[0] and pts[0, 0] < -0.5 and abs(pts[0, 1]) < 0.01  # behind the car, on its centre line


def test_object_footprint_spans_its_width_where_it_stands():
    from guide_perception.ground import mount_pose, nominal_K
    from guide_perception.people import Person, locate, object_footprint

    K = nominal_K(640, 360, 70.0)
    R, t = mount_pose(0.50, 0.0, 0.28, -0.09, 0.0)
    # a chair 0.5 m wide standing 3 m ahead, 0.4 m left
    ahead, left, width = 3.0, 0.4, 0.5
    cam = R.T @ (np.array([ahead, left, 0.0]) - t)
    u, v = (K @ cam)[:2] / cam[2]
    half_px = width / 2 * K[0, 0] / cam[2]
    chair = locate([Person(None, (u - half_px, v - 60, u + half_px, v), 0.8, cls=56, label="chair")], K, R, t, 360)[0]
    pts = np.array(object_footprint(chair, K, t[:2]))
    assert abs(pts[:, 0].mean() - ahead) < 0.1 and abs(pts[:, 1].mean() - left) < 0.05
    assert abs((pts[:, 1].max() - pts[:, 1].min()) - width) < 0.08
    assert object_footprint(Person(None, (0, 0, 1, 1), 0.5), K) == []  # not placed: nothing


def test_carried_bags_are_not_floor_obstacles():
    from guide_perception.people import Person, carried

    walker = Person(1, (200, 40, 300, 340), 0.9)
    worn = Person(None, (230, 90, 290, 200), 0.7, label="backpack")
    on_floor = Person(None, (400, 250, 460, 330), 0.7, label="backpack")
    beside = Person(None, (280, 200, 380, 330), 0.7, label="chair")  # overlaps the walker a little
    assert carried(worn, [walker]) and not carried(on_floor, [walker]) and not carried(beside, [walker])
