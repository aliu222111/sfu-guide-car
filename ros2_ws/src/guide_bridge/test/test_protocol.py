import math

from guide_bridge.protocol import Button, LineBuffer, Ultrasonic, format_drive, parse_line


def test_format_clamps_and_rounds():
    assert format_drive(0.5, -0.2) == b"C 0.500 -0.200\n"
    assert format_drive(3.0, -9.0) == b"C 1.000 -1.000\n"
    assert format_drive(float("nan"), 0.1) == b"C 0.000 0.100\n"


def test_parse():
    assert parse_line("B 1") == Button(True)
    assert parse_line("B 0") == Button(False)
    assert parse_line("B 2") is None
    assert parse_line("garbage") is None


def test_line_buffer_splits_and_drops_overlong():
    b = LineBuffer(limit=10)
    assert b.feed(b"U 87\nB ") == [b"U 87"]
    assert b.feed(b"1\n") == [b"B 1"]
    assert b.feed(b"x" * 20) == []
    assert b.buf == b""


def test_ultrasonic():
    assert parse_line(b"U 87") == Ultrasonic(0.87)
    assert parse_line("U 0") == Ultrasonic(math.inf)
    assert parse_line("U -3") is None
    assert parse_line("U x") is None
