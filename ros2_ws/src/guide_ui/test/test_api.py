import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from guide_ui.api import MapData
from guide_ui.server_node import make_handler

MAPS = Path(__file__).resolve().parents[4] / "maps"


@pytest.fixture(scope="module")
def server():
    srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(MapData(MAPS), {"rosbridge_port": 9090}))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def get(url):
    with urllib.request.urlopen(url) as r:
        return r.status, r.read()


def test_destinations_classrooms_first(server):
    _, body = get(server + "/api/destinations")
    d = json.loads(body)
    assert len(d) > 1000
    assert d[0]["type"] in ("Classroom", "Lecture Theatre", "Seminar")
    assert any(x["key"] == "AQ 3150" for x in d)


def test_floor_outline(server):
    _, body = get(server + "/api/floor/AQ-03")
    f = json.loads(body)
    assert f["name"] == "AQ 3000 Level" and len(f["edges"]) > 100


def test_static_and_traversal(server):
    status, body = get(server + "/")
    assert status == 200 and b"SFU Guide Car" in body
    for bad in ("/../setup.py", "/%2e%2e/setup.py", "/api/floor/NOPE"):
        with pytest.raises(urllib.error.HTTPError) as e:
            get(server + bad)
        assert e.value.code == 404
