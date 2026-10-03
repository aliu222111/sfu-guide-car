import pytest
from shapely.geometry import box

from sfu_map.gisdata import Floor, Room


def room(rm_id, geom, cat="OFFICE", typ="Office", floor="T-01"):
    return Room("T", floor, rm_id, typ, cat, "", True, geom)


@pytest.fixture
def t_floor():
    """A T-shaped corridor: 40 m east-west, a 20 m branch north from the middle,
    four offices along the south wall and a stair at the east end."""
    fl = Floor("T", "01", "Test Level", 1)
    fl.rooms = [
        room("C1", box(0, 0, 40, 3), "CIRCULATION", "Corridor"),
        room("C2", box(18.5, 3, 21.5, 23), "CIRCULATION", "Corridor"),
        room("101", box(0, -6.2, 9, -0.2)),
        room("102", box(10, -6.2, 19, -0.2)),
        room("103", box(20, -6.2, 29, -0.2), "CLASSROOM", "Classroom"),
        room("104", box(30, -6.2, 39, -0.2)),
        room("S1", box(40.2, 0, 44, 3), "PENETRATIONS", "Stairs"),
    ]
    return fl
