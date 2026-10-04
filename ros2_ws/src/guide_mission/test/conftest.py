from pathlib import Path

import pytest

from sfu_map.route import RouteGraph

MAPS = Path(__file__).resolve().parents[4] / "maps"


@pytest.fixture(scope="session")
def campus():
    return RouteGraph.load(MAPS / "route_graph.json")
