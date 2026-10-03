"""Claude through the owner's Claude subscription, via the Claude Agent SDK.

The Agent SDK drives the Claude Code CLI (bundled with the ``claude-agent-sdk``
package), which signs in with the subscription. On the car laptop, once:

    claude setup-token                # prints a long-lived token for the subscription
    export CLAUDE_CODE_OAUTH_TOKEN=... # e.g. in ~/.bashrc or the launch environment

(or run ``claude`` once and log in). This is for the owner's own use of the
car: Anthropic does not allow offering claude.ai login to other people's
products. ``ANTHROPIC_API_KEY`` takes precedence over the subscription login
in Claude Code, so subscription mode removes it from this process.

One ``AgentSession`` keeps one Claude Code process alive in a background
thread, so each request costs a model call, not a CLI start-up.
"""

from __future__ import annotations

import asyncio
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, AsyncIterator, Callable


@dataclass
class Reply:
    text: str
    is_error: bool
    tool_calls: list[tuple[str, dict]]
    detail: str = ""


def subscription_status() -> str:
    """How the CLI will sign in, for the startup log."""
    if os.environ.get("CLAUDE_CODE_OAUTH_TOKEN"):
        return "CLAUDE_CODE_OAUTH_TOKEN (from `claude setup-token`)"
    if (Path.home() / ".claude" / ".credentials.json").exists() or (Path.home() / ".claude.json").exists():
        return "Claude Code login in ~/.claude"
    return "no subscription credentials found: run `claude setup-token` and set CLAUDE_CODE_OAUTH_TOKEN"


def use_subscription_env() -> bool:
    """Make sure the CLI uses the subscription, not an API key. Returns True if a key was removed."""
    return os.environ.pop("ANTHROPIC_API_KEY", None) is not None


def user_message(content: list[dict]) -> Callable[[], AsyncIterator[dict]]:
    """A streamed user turn with content blocks (text and images)."""

    async def gen():
        yield {"type": "user", "message": {"role": "user", "content": content}, "parent_tool_use_id": None}

    return gen


class AgentSession:
    """A persistent Claude Code session usable from synchronous ROS callbacks.

    ``options_factory`` builds ``ClaudeAgentOptions`` for each (re)connect;
    ``client_factory`` makes the client (``ClaudeSDKClient`` by default, a
    fake in tests).
    """

    def __init__(self, options_factory: Callable[[], Any], client_factory: Callable[[Any], Any] | None = None):
        self.options_factory = options_factory
        if client_factory is None:
            from claude_agent_sdk import ClaudeSDKClient

            client_factory = ClaudeSDKClient
        self.client_factory = client_factory
        self.client = None
        self.turns = 0
        # Connect and close are serialized: prewarm() then ask() would otherwise
        # start two CLI processes and orphan one (a leak per new student).
        self._lock = asyncio.Lock()
        self.loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self._thread.start()

    def _run(self, coro, timeout: float):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    async def _connect(self):
        async with self._lock:
            if self.client is None:
                client = self.client_factory(self.options_factory())
                await client.connect()
                self.client = client
                self.turns = 0

    async def _ask(self, prompt) -> Reply:
        from claude_agent_sdk import AssistantMessage, ResultMessage, TextBlock, ToolUseBlock

        await self._connect()
        await self.client.query(prompt() if callable(prompt) else prompt)
        texts, calls, result = [], [], None
        async for msg in self.client.receive_response():
            if isinstance(msg, AssistantMessage):
                for block in msg.content:
                    if isinstance(block, TextBlock):
                        texts.append(block.text)
                    elif isinstance(block, ToolUseBlock):
                        calls.append((block.name, dict(block.input or {})))
            elif isinstance(msg, ResultMessage):
                result = msg
        self.turns += 1
        if result is None:
            return Reply(" ".join(texts).strip(), True, calls, "no result message")
        detail = "" if not result.is_error else f"{result.subtype}: {result.errors or result.result or ''}"
        return Reply(" ".join(t.strip() for t in texts if t.strip()), bool(result.is_error), calls, detail)

    async def _close(self):
        async with self._lock:
            if self.client is not None:
                client, self.client = self.client, None
                try:
                    await client.disconnect()
                except Exception:
                    pass

    def ask(self, prompt, timeout: float = 30.0) -> Reply:
        """Send a string or ``user_message(...)``; blocks until the reply is complete."""
        fut = asyncio.run_coroutine_threadsafe(self._ask(prompt), self.loop)
        try:
            return fut.result(timeout)
        except Exception:
            # A stuck or dead CLI: stop this request and start a fresh one next time.
            fut.cancel()
            self.reset()
            raise

    def prewarm(self):
        """Start the CLI in the background so the first request is fast."""
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def reset(self, timeout: float = 10.0):
        """Forget the conversation (a new student) by starting a fresh session."""
        try:
            self._run(self._close(), timeout)
        except Exception:
            self.client = None

    def close(self):
        self.reset()
        self.loop.call_soon_threadsafe(self.loop.stop)
