import math

from guide_bridge.protocol import MAX_LINEAR, Button, Encoders, LineBuffer, format_velocity, parse_line


def test_format_clamps_and_rounds():
    assert format_velocity(0.5, -0.2) == b"V 0.500 -0.200\n"
    assert format_velocity(99, 0) == f"V {MAX_LINEAR:.3f} 0.000\n".encode()
    assert format_velocity(float("nan"), 0) == b"V 0.000 0.000\n"


def test_parse():
    assert parse_line(b"E 10 -4\r\n") == Encoders(10, -4)
    assert parse_line("B 1") == Button(True)
    assert parse_line("B 0") == Button(False)
    assert parse_line("E 1") is None
    assert parse_line("E a b") is None
    assert parse_line("garbage") is None


def test_line_buffer_splits_and_drops_overlong():
    b = LineBuffer(limit=10)
    assert b.feed(b"E 1 2\nB ") == [b"E 1 2"]
    assert b.feed(b"1\n") == [b"B 1"]
    assert b.feed(b"x" * 20) == []
    assert b.buf == b""


def test_ultrasonic():
    from guide_bridge.protocol import Ultrasonic

    assert parse_line(b"U 87") == Ultrasonic(0.87)
    assert parse_line("U 0") == Ultrasonic(math.inf)
    assert parse_line("U -3") is None
    assert parse_line("U x") is None
