import asyncio
import os

from claude_agent_sdk import AssistantMessage, ClaudeAgentOptions, ResultMessage, TextBlock, ToolUseBlock

from guide_claude.session import AgentSession, use_subscription_env
from guide_claude.signs import SubscriptionSignReader
from guide_claude.voice import SubscriptionVoiceAgent, make_tools


def result(is_error=False):
    return ResultMessage(subtype="error_during_execution" if is_error else "success", duration_ms=1, duration_api_ms=1,
                         is_error=is_error, num_turns=1, session_id="s")


class FakeClient:
    """Stands in for ClaudeSDKClient: records prompts, replies from a script."""

    instances = []

    def __init__(self, options):
        self.options = options
        self.prompts = []
        self.connected = False
        FakeClient.instances.append(self)

    async def connect(self):
        self.connected = True

    async def disconnect(self):
        self.connected = False

    async def query(self, prompt):
        if hasattr(prompt, "__aiter__"):
            prompt = [m async for m in prompt]
        self.prompts.append(prompt)

    async def receive_response(self):
        for m in FakeClient.script(self.prompts[-1]):  # read at call time
            yield m


def test_session_collects_text_tools_and_errors():
    FakeClient.script = lambda p: [AssistantMessage(content=[TextBlock("Okay."), ToolUseBlock("t1", "mcp__car__confirm", {})], model="m"), result()]
    s = AgentSession(lambda: ClaudeAgentOptions(), FakeClient)
    r = s.ask("yes")
    assert r.text == "Okay." and not r.is_error and r.tool_calls == [("mcp__car__confirm", {})]
    assert s.turns == 1
    FakeClient.script = lambda p: [result(is_error=True)]
    assert s.ask("x").is_error
    s.reset()
    assert s.client is None
    s.close()


def test_voice_agent_options_lock_it_down_and_reset_after_idle():
    FakeClient.instances.clear()
    FakeClient.script = lambda p: [AssistantMessage(content=[TextBlock("Follow me to AQ 3150.")], model="m"), result()]

    class A:
        set_destination = staticmethod(lambda r: f"Guiding to {r}.")
        pause_or_resume = staticmethod(lambda: "Paused.")
        confirm = staticmethod(lambda: "Confirmed.")

    agent = SubscriptionVoiceAgent(A(), "system", client_factory=FakeClient)
    assert agent.handle("AQ 3150 please", "idle", now=1.0) == "Follow me to AQ 3150."
    opts = FakeClient.instances[0].options
    assert opts.tools == [] and opts.permission_mode == "dontAsk" and opts.setting_sources == []
    assert opts.allowed_tools == ["mcp__car__set_destination", "mcp__car__pause_or_resume", "mcp__car__confirm"]
    assert "car" in opts.mcp_servers and opts.model == "claude-haiku-4-5"
    assert FakeClient.instances[0].prompts[-1] == "[Robot status: idle]\nAQ 3150 please"
    agent.handle("hello", now=500.0)  # idle for longer than idle_reset: fresh session
    assert len(FakeClient.instances) >= 2 and not FakeClient.instances[0].connected
    FakeClient.script = lambda p: [result(is_error=True)]
    import pytest

    with pytest.raises(RuntimeError):  # a usage limit or CLI error: the node logs it and says so
        agent.handle("x", now=501.0)


def test_tools_call_the_car_and_report_errors():
    log = []
    t_set, t_pause, t_confirm = make_tools(lambda r: log.append(r) or f"Guiding to {r}.", lambda: "Paused.", lambda: 1 / 0)
    out = asyncio.run(t_set.handler({"room": " AQ 3150 "}))
    assert log == ["AQ 3150"] and out["content"][0]["text"] == "Guiding to AQ 3150."
    assert asyncio.run(t_set.handler({}))["is_error"]
    assert asyncio.run(t_confirm.handler({}))["is_error"]
    assert [t.name for t in (t_set, t_pause, t_confirm)] == ["set_destination", "pause_or_resume", "confirm"]


def test_sign_reader_sends_image_and_rotates_sessions():
    FakeClient.instances.clear()
    FakeClient.script = lambda p: [AssistantMessage(content=[TextBlock('{"rooms": ["AQ 3150"]}')], model="m"), result()]
    reader = SubscriptionSignReader("read rooms", frames_per_session=3, client_factory=FakeClient)
    for _ in range(5):
        assert reader.read_text(b"\xff\xd8jpeg") == '{"rooms": ["AQ 3150"]}'
    msg = FakeClient.instances[0].prompts[0][0]
    content = msg["message"]["content"]
    assert msg["type"] == "user" and content[0]["type"] == "image" and content[0]["source"]["media_type"] == "image/jpeg"
    assert len(FakeClient.instances) >= 2  # a fresh session after frames_per_session


def test_subscription_env_removes_api_key(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-test")
    assert use_subscription_env() and "ANTHROPIC_API_KEY" not in os.environ
    assert not use_subscription_env()


def test_prewarm_then_ask_starts_one_client():
    """A slow connect overlapped by ask() once started a second CLI and orphaned the first."""
    import asyncio
    import time

    from claude_agent_sdk import ClaudeAgentOptions, ResultMessage

    from guide_claude.session import AgentSession

    made = []

    class Slow:
        def __init__(self, options):
            self.connected = False
            made.append(self)

        async def connect(self):
            await asyncio.sleep(0.3)
            self.connected = True

        async def disconnect(self):
            self.connected = False

        async def query(self, prompt):
            pass

        async def receive_response(self):
            yield ResultMessage(subtype="success", duration_ms=1, duration_api_ms=1, is_error=False, num_turns=1, session_id="s")

    s = AgentSession(lambda: ClaudeAgentOptions(), Slow)
    s.prewarm()
    s.ask("hi", timeout=5)
    s.reset()
    s.prewarm()
    s.ask("again", timeout=5)
    time.sleep(0.4)
    assert len(made) == 2 and [c.connected for c in made] == [False, True]
    s.close()
