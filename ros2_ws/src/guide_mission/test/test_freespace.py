"""Moving an impossible estimate (inside a wall) to where the car fits."""

import json
import math
from pathlib import Path

import pytest

from guide_mission.freespace import FloorSpace

MAPS = Path(__file__).resolve().parents[4] / "maps"


@pytest.fixture(scope="module")
def aq3():
    if not (MAPS / "floors" / "AQ-03.yaml").exists():
        pytest.skip("maps not built")
    nodes = {n["id"]: n for n in json.loads((MAPS / "route_graph.json").read_text())["nodes"]}
    return FloorSpace.load(MAPS / "floors" / "AQ-03.yaml"), nodes


def test_entrance_is_open(aq3):
    space, nodes = aq3
    n = nodes["entrance:aq_convocation_mall"]
    assert space.clearance(n["x"], n["y"]) > 0.4


def test_the_hil_failure_snaps_out_of_the_wall(aq3):
    # The estimate Nav2 refused ("Start occupied") after a sign fix from an unknown heading.
    space, nodes = aq3
    assert space.clearance(-64.63, 25.95) < 0.4
    x, y = space.nearest_fit(-64.63, 25.95, 0.4)
    assert space.clearance(x, y) >= 0.4 and math.hypot(x + 64.63, y - 25.95) < 2.0


def test_doors_are_on_the_corridor_side(aq3):
    space, nodes = aq3
    door = nodes["AQ-03:door:305.1"]
    assert space.nearest_fit(door["x"], door["y"], 0.4) is not None


def test_deep_in_a_wall_or_outside(aq3):
    space, _ = aq3
    assert space.clearance(-1e6, 0) == 0.0
    assert space.nearest_fit(-1e6, 0) is None
