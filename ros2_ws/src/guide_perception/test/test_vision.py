import math
from types import SimpleNamespace

import cv2
import numpy as np

from guide_perception.segmentation import DRIVABLE, drivable_ids, labels_from_logits
from guide_perception.sign_reader import ClaudeBackend, extract_rooms, parse_claude_reply
from guide_perception.vanishing import absolute_yaw, detect_lines, find_vanishing_point, oblique, relative_yaw


def corridor_image(vp=(400, 170), w=640, h=360):
    img = np.full((h, w, 3), 200, np.uint8)
    for (x, y) in [(0, 360), (640, 360), (0, 0), (640, 0), (0, 300), (640, 300)]:
        # Draw each edge from the frame border most of the way to the point.
        ex = int(x + 0.85 * (vp[0] - x))
        ey = int(y + 0.85 * (vp[1] - y))
        cv2.line(img, (x, y), (ex, ey), (20, 20, 20), 3)
    return img


def test_vanishing_point_found():
    img = corridor_image()
    segs = oblique(detect_lines(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)))
    vp = find_vanishing_point(segs)
    assert vp is not None
    assert abs(vp.x - 400) < 8 and abs(vp.y - 170) < 8


def test_vanishing_point_rejects_one_sided_lines():
    segs = np.array([[0, 360, 300, 200], [0, 300, 300, 190], [0, 250, 300, 180], [0, 200, 300, 175]], float)
    assert find_vanishing_point(segs) is None


def test_heading_math():
    rel = relative_yaw(400, 500, 320)
    assert rel > 0  # corridor appears right of centre: the car is turned left of it
    assert abs(absolute_yaw(0.0, rel, 0.1) - rel) < 1e-9
    # Driving the other way along the same corridor axis.
    assert abs(absolute_yaw(0.0, rel, math.pi) - (rel - math.pi)) < 1e-9


def test_segformer_postprocess_and_ids(tmp_path):
    logits = np.zeros((1, 150, 2, 2), np.float32)
    logits[0, 3, 0, 0] = 5  # floor
    logits[0, 9, 1, 1] = 5  # grass
    lab = labels_from_logits(logits, (4, 4))
    assert lab[0, 0] == 3 and lab[3, 3] == 9
    assert set(drivable_ids()) == set(DRIVABLE.values())
    p = tmp_path / "labels.json"
    p.write_text('{"0": "wall", "3": "floor", "6": "road"}')
    assert list(drivable_ids(p)) == [3, 6]


def test_room_extraction():
    assert [r.text for r in extract_rooms("AQ 3150 Lecture")] == ["AQ 3150"]
    assert [r.text for r in extract_rooms("Room 3004b")] == ["3004B"]
    assert extract_rooms("EXIT 2024") == []
    assert [r.text for r in extract_rooms("wmc-2260")] == ["WMC 2260"]


def test_claude_backend_with_fake_client():
    calls = []

    class FakeMessages:
        def create(self, **kw):
            calls.append(kw)
            return SimpleNamespace(stop_reason="end_turn", content=[SimpleNamespace(type="text", text='{"rooms": ["AQ 3150", "3149"]}')])

    backend = ClaudeBackend(client=SimpleNamespace(messages=FakeMessages()))
    reads = backend.read(np.zeros((360, 640, 3), np.uint8))
    assert [r.text for r in reads] == ["AQ 3150", "3149"]
    content = calls[0]["messages"][0]["content"]
    assert calls[0]["model"] == "claude-haiku-4-5"
    assert content[0]["type"] == "image" and content[0]["source"]["media_type"] == "image/jpeg"
    assert parse_claude_reply("no json") == []


def test_claude_subscription_backend_parses_reader_reply():
    from guide_perception.sign_reader import ClaudeSubscriptionBackend

    class Reader:
        def read_text(self, jpeg):
            assert jpeg[:2] == b"\xff\xd8"
            return 'Rooms: {"rooms": ["WMC 2260"]}'

    reads = ClaudeSubscriptionBackend(reader=Reader()).read(np.zeros((360, 640, 3), np.uint8))
    assert [r.text for r in reads] == ["WMC 2260"]
