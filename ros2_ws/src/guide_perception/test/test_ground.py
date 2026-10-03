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
