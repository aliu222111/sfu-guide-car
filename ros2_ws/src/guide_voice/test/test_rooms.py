"""Rooms and answers in what speech recognition wrote."""

import pytest

from guide_voice.rooms import answer_in, places_in, rooms_in, spoken, suggest_rooms

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


@pytest.mark.parametrize("said, want", [
    ("3064", "AQ 3046"),  # two digits swapped
    ("3056", "AQ 3046"),  # one digit misheard
    ("304", "AQ 3046"),  # one digit dropped
])
def test_closest_real_room(said, want):
    keys = {"AQ 3046", "AQ 3150", "AQ 3003.1", "WMC 2830"}
    assert suggest_rooms(["AQ"], said, keys)[0] == want


def test_same_number_in_another_building_first():
    assert suggest_rooms(["MBC"], "2830", {"WMC 2830", "MBC 2831"}) == ["WMC 2830", "MBC 2831"]


DESTS = {
    "AQ 3046": {"name": "Multifaith Centre", "type": "Office", "building": "AQ", "floor": "AQ-03"},
    "AQ 3044": {"name": "Women's Studies", "type": "Office", "building": "AQ", "floor": "AQ-03"},
    "AQ 3059": {"name": "", "type": "Washroom, men's, accessible", "building": "AQ", "floor": "AQ-03",
                "room_x": 0.0, "room_y": 0.0},
    "AQ 3010": {"name": "", "type": "Washroom, women's, accessible", "building": "AQ", "floor": "AQ-03",
                "room_x": 50.0, "room_y": 0.0},
    "TA 1": {"name": "TA OFFICE", "type": "Office", "building": "AQ", "floor": "AQ-03"},
}


def test_places_by_name_and_kind():
    here = {"floor": "AQ-03", "pose": [1.0, 0.0]}
    assert places_in("the multi faith centre", DESTS) == (["AQ 3046"], "")
    assert places_in("I need a washroom", DESTS, here) == (["AQ 3059"], "washroom")
    assert places_in("a women's washroom", DESTS, here) == (["AQ 3010"], "washroom")  # not Women's Studies
    assert places_in("a washroom", DESTS) == ([], "washroom")  # where is the car? ask the building
    assert places_in("the TA office", DESTS) == ([], "")
