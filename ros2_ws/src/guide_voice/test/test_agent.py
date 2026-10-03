from types import SimpleNamespace

import numpy as np

from guide_voice.agent import TOOLS, Actions, VoiceAgent
from guide_voice.speech import SAMPLE_RATE, trim_silence


def text(t):
    return SimpleNamespace(type="text", text=t)


def tool(name, args, id_="t1"):
    return SimpleNamespace(type="tool_use", name=name, input=args, id=id_)


class FakeClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []
        self.messages = self

    def create(self, **kw):
        self.calls.append({**kw, "messages": list(kw["messages"])})
        return self.responses.pop(0)


def make(responses):
    log = []
    actions = Actions(
        set_destination=lambda r: log.append(("dest", r)) or f"Guiding to {r}.",
        pause_or_resume=lambda: log.append(("pause",)) or "Paused.",
        confirm=lambda: log.append(("confirm",)) or "Confirmed.",
    )
    client = FakeClient(responses)
    return VoiceAgent(client, actions), client, log


def test_exactly_three_tools():
    assert [t["name"] for t in TOOLS] == ["set_destination", "pause_or_resume", "confirm"]


def test_destination_round_trip():
    agent, client, log = make([
        SimpleNamespace(stop_reason="tool_use", content=[tool("set_destination", {"room": "AQ 3150"})]),
        SimpleNamespace(stop_reason="end_turn", content=[text("Follow me to AQ 3150.")]),
    ])
    reply = agent.handle("take me to A Q thirty one fifty", "idle", now=0.0)
    assert reply == "Follow me to AQ 3150." and log == [("dest", "AQ 3150")]
    second = client.calls[1]["messages"]
    assert second[-1]["content"][0]["type"] == "tool_result" and second[-1]["content"][0]["tool_use_id"] == "t1"
    assert client.calls[0]["model"] == "claude-haiku-4-5"


def test_tool_errors_reported_not_raised():
    def boom(_):
        raise RuntimeError("bridge down")

    agent, client, _ = make([
        SimpleNamespace(stop_reason="tool_use", content=[tool("set_destination", {"room": "AQ 3150"}), tool("nope", {}, "t2")]),
        SimpleNamespace(stop_reason="end_turn", content=[text("Something went wrong.")]),
    ])
    agent.actions.set_destination = boom
    agent.handle("go", now=0.0)
    results = client.calls[1]["messages"][-1]["content"]
    assert [r["is_error"] for r in results] == [True, True]


def test_turn_cap_and_idle_reset():
    loop = [SimpleNamespace(stop_reason="tool_use", content=[tool("confirm", {}, f"t{i}")]) for i in range(4)]
    agent, client, log = make(loop + [SimpleNamespace(stop_reason="end_turn", content=[text("Hi.")])])
    assert agent.handle("yes", now=0.0) == "Sorry, could you say that again?"
    assert len(log) == 4
    agent.handle("hello", now=1000.0)
    assert len(client.calls[-1]["messages"]) == 1  # history dropped after idle


def test_history_trim_keeps_whole_exchanges():
    agent, _, _ = make([])
    agent.history_limit = 3
    agent.messages = [
        {"role": "user", "content": "a"}, {"role": "assistant", "content": []},
        {"role": "user", "content": [{"type": "tool_result"}]}, {"role": "assistant", "content": []},
        {"role": "user", "content": "b"}, {"role": "assistant", "content": []},
    ]
    agent._trim()
    assert agent.messages[0] == {"role": "user", "content": "b"}


def test_trim_silence():
    quiet = np.zeros(SAMPLE_RATE, np.float32)
    assert trim_silence(quiet).size == 0
    a = quiet.copy()
    a[8000:9000] = 0.5
    out = trim_silence(a)
    assert 1000 <= out.size < 8000
