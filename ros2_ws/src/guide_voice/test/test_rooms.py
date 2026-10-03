"""Rooms and answers in what speech recognition wrote."""

import pytest

from guide_voice.rooms import answer_in, rooms_in, spoken

KEYS = {"AQ 3046", "AQ 3150", "AQ 3003.1", "WMC 2830", "WMC 2260", "MBC 2290", "AQ 2830", "WMC 3150", "AQ 5005"}


@pytest.mark.parametrize("text, want", [
    ("Take me to AQ 3046", ["AQ 3046"]),
    ("A Q thirty forty six please", ["AQ 3046"]),
    ("academic quadrangle three oh four six", ["AQ 3046"]),
    ("I need to get to 8Q 3046.", ["AQ 3046"]),
    ("aq3046", ["AQ 3046"]),
    ("A.Q. three thousand forty-six", ["AQ 3046"]),
    ("West Mall twenty-eight thirty", ["WMC 2830"]),
    ("w m c twenty two sixty", ["WMC 2260"]),
    ("MBC 2,290", ["MBC 2290"]),
    ("room 3150 in the West Mall Centre", ["WMC 3150"]),
    ("AQ 3003 point one", ["AQ 3003.1"]),
    ("five thousand and five in AQ", []),  # "and" breaks the number: better to ask than to guess
    ("AQ five oh oh five", ["AQ 5005"]),
])
def test_rooms(text, want):
    assert rooms_in(text, KEYS) == want


def test_number_without_a_building_lists_every_building():
    assert sorted(rooms_in("room twenty-eight thirty", KEYS)) == ["AQ 2830", "WMC 2830"]


def test_no_room():
    assert rooms_in("where is the library", KEYS) == []
    assert rooms_in("I have 2 minutes", KEYS) == []


@pytest.mark.parametrize("text, want", [
    ("Yes.", "yes"), ("yeah it's open", "yes"), ("okay go", "yes"), ("No, wrong room", "no"),
    ("stop!", "stop"), ("wait a second", "stop"), ("take me to AQ 3046 then the library please", None), ("", None),
])
def test_answers(text, want):
    assert answer_in(text) == want


def test_spoken():
    dest = {"name": "Multifaith Centre", "floor_name": "3000 Level"}
    assert spoken("AQ 3046", dest) == "AQ 3046, the Multifaith Centre, on the 3000 level of the Academic Quadrangle"
    assert spoken("WMC 2830", {"name": "Classroom", "floor_name": ""}) == "WMC 2830, in the West Mall Centre"
