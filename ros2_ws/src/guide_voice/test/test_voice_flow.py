"""The voice node's conversation: read the room back, go on yes, a door number when lost."""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from guide_voice.cloud import Unheard
from guide_voice.voice_node import VoiceNode

MAPS = Path(__file__).resolve().parents[4] / "maps" / "route_graph.json"


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(getattr(m, "data", m))


@pytest.fixture
def node():
    if not MAPS.exists():
        pytest.skip("maps not built")
    from sfu_map.route import RouteGraph

    n = VoiceNode.__new__(VoiceNode)  # the logic only: no ROS, microphone or network
    n.rg = RouteGraph.load(str(MAPS))
    n.trip = {"state": "idle", "detail": ""}
    n.pending = None
    n.dest_pub, n.pause_pub, n.confirm_pub, n.sign_pub = Pub(), Pub(), Pub(), Pub()
    n.heard = []
    n.stt = lambda audio, extra_terms=(): n.heard.pop(0)
    n.agent = SimpleNamespace(handle=lambda text, ctx: (_ for _ in ()).throw(TimeoutError("no Gemini")))
    n.get_logger = lambda: SimpleNamespace(info=lambda *a: None, error=lambda *a: None)
    return n


def say(n, text):
    n.heard.append(text)
    return n.reply(None)


def test_room_is_read_back_and_starts_only_on_yes(node):
    reply = say(node, "A Q thirty forty six please")
    assert reply == "AQ 3046, the Multifaith Centre, on the 3000 level of the Academic Quadrangle. Is that right?"
    assert node.dest_pub.sent == [] and node.pending == "AQ 3046"
    assert say(node, "yes") == "Starting the trip to AQ 3046."
    assert node.dest_pub.sent == ["AQ 3046"] and node.pending is None


def test_no_asks_again(node):
    say(node, "AQ 3046")
    assert say(node, "no") == "Okay. Which room would you like?"
    assert node.pending is None and node.dest_pub.sent == []


def test_yes_at_a_door_is_a_confirm(node):
    node.trip = {"state": "waiting_for_help", "detail": "door"}
    assert say(node, "yeah it's open") == "Thanks."
    assert len(node.confirm_pub.sent) == 1


def test_lost_door_number_is_a_sign_read(node):
    node.trip = {"state": "lost", "detail": "localization lost"}
    assert say(node, "it says A Q three oh four six") == "Thanks. I'll look around AQ 3046."
    assert json.loads(node.sign_pub.sent[0])["text"] == "AQ 3046" and node.pending is None


def test_gemini_gets_the_rooms_heard(node):
    seen = {}
    node.agent = SimpleNamespace(handle=lambda text, ctx: seen.setdefault("ctx", ctx) and "ok")
    say(node, "eight Q thirty forty six")
    assert "rooms that match what was heard: AQ 3046" in seen["ctx"]


def test_no_network(node):
    def deaf(audio, extra_terms=()):
        raise Unheard("offline")

    node.stt = deaf
    assert "can't hear you" in node.reply(None)


def test_room_not_on_the_map_suggests_the_likely_one(node):
    reply = say(node, "take me to A Q thirty sixty four")
    assert reply == "I can't find AQ 3064 on my map. Did you mean AQ 3046 or AQ 3062?"
    assert node.pending is None
    assert say(node, "forty six, AQ 3046").endswith("Is that right?") and node.pending == "AQ 3046"


def test_a_place_by_name_is_proposed_and_yes_goes(node):
    reply = say(node, "can you take me to the multi faith centre")
    assert reply == "Do you mean AQ 3046, the Multifaith Centre, on the 3000 level of the Academic Quadrangle?"
    assert node.pending == "AQ 3046"
    assert say(node, "yes") == "Starting the trip to AQ 3046." and node.dest_pub.sent == ["AQ 3046"]


def test_nearest_washroom_from_where_the_car_is(node):
    node.trip = {"state": "idle", "detail": "", "floor": "AQ-03", "pose": [-60.0, 0.0, 0.0]}
    reply = say(node, "I need a washroom")
    assert reply.startswith("The nearest washroom I know is AQ 3059") and node.pending == "AQ 3059"


def test_a_kind_of_place_with_no_position_asks_the_building(node):
    assert say(node, "where's the nearest washroom").startswith("Which building should I look for a washroom in")
    assert node.pending is None


def test_lost_and_a_door_number_not_on_the_map(node):
    node.trip = {"state": "lost", "detail": ""}
    assert say(node, "it says AQ 3064") == "I can't find AQ 3064 on my map. Could you read me the number on another door?"
    assert node.sign_pub.sent == []
