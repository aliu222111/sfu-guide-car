"""Voice agent node (spec section 9).

Hold the push-to-talk button on the car (``voice/ptt`` from the serial bridge),
speak, release. The transcript goes to the agent; its reply and every mission
announcement (``guide/speech``) are spoken on the car's speaker.
"""

from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from sfu_map.route import RouteGraph

from .agent import SYSTEM, Actions, VoiceAgent
from .speech import Recorder, Speaker, Transcriber, trim_silence


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_agent")
        graph = self.declare_parameter("graph", "maps/route_graph.json").value
        self.rg = RouteGraph.load(graph)
        self.trip = {"state": "idle", "detail": ""}
        P = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        log = self.get_logger().warn
        piper = (P("piper_voice", "voices/en_US-lessac-medium.onnx"), P("piper_bin", "piper"), P("player", "aplay"))
        tts = P("tts", "piper")
        if tts == "elevenlabs":  # ElevenLabs voice, Piper when it cannot (ELEVENLABS_API_KEY)
            from .cloud import ElevenLabsSpeaker

            self.speaker = ElevenLabsSpeaker(*piper, voice_id=P("elevenlabs_voice_id", "21m00Tcm4TlvDq8ikWAM"),
                                             model=P("elevenlabs_tts_model", "eleven_flash_v2_5"), log=log)
        elif tts == "piper":
            self.speaker = Speaker(*piper)
        else:
            raise ValueError(f"tts must be piper or elevenlabs, not {tts!r}")
        self.recorder = Recorder(P("mic_device", "") or None)
        stt = P("stt", "whisper")
        whisper = Transcriber(P("whisper_model", "base"), P("whisper_device", "auto"), P("whisper_compute", "int8"))
        if stt == "elevenlabs":  # the student's voice goes to ElevenLabs; whisper when it cannot
            from .cloud import ElevenLabsTranscriber

            self.stt = ElevenLabsTranscriber(whisper, model=P("elevenlabs_stt_model", "scribe_v1"), log=log)
        elif stt == "whisper":
            self.stt = whisper
        else:
            raise ValueError(f"stt must be whisper or elevenlabs, not {stt!r}")
        actions = Actions(self.set_destination, self.pause_or_resume, self.confirm)
        llm = P("llm", "claude")
        model = self.declare_parameter("claude_model", "claude-haiku-4-5").value
        auth = self.declare_parameter("claude_auth", "subscription").value
        if llm == "gemini":  # Google Gemini (GEMINI_API_KEY)
            from google import genai
            from google.genai import types

            from .gemini_agent import GeminiVoiceAgent

            client = genai.Client(http_options=types.HttpOptions(timeout=15000))
            self.agent = GeminiVoiceAgent(client, actions, model=P("gemini_model", "gemini-flash-latest"))
            self.get_logger().info(f"voice agent: Gemini {self.agent.model}")
        elif llm != "claude":
            raise ValueError(f"llm must be claude or gemini, not {llm!r}")
        elif auth == "subscription":
            # The owner's Claude subscription through the Claude Agent SDK (guide_claude).
            from guide_claude.session import subscription_status, use_subscription_env
            from guide_claude.voice import SubscriptionVoiceAgent

            if use_subscription_env():
                self.get_logger().warn("ANTHROPIC_API_KEY ignored: claude_auth is subscription")
            self.get_logger().info(f"Claude via subscription: {subscription_status()}")
            self.agent = SubscriptionVoiceAgent(actions, SYSTEM, model=model)
        elif auth == "api_key":
            import anthropic

            self.agent = VoiceAgent(anthropic.Anthropic(timeout=15.0), actions, model=model)
        else:
            raise ValueError(f"claude_auth must be subscription or api_key, not {auth!r}")
        self.dest_pub = self.create_publisher(String, "guide/destination", 10)
        self.pause_pub = self.create_publisher(Bool, "guide/pause", 10)
        self.confirm_pub = self.create_publisher(Empty, "guide/confirm", 10)
        self.create_subscription(Bool, "voice/ptt", self.on_ptt, 10)
        self.create_subscription(String, "guide/speech", lambda m: self.speaker.say(m.data), 10)
        self.create_subscription(String, "guide/state", self.on_state, 10)
        self.busy = threading.Lock()

    def on_state(self, msg):
        try:
            self.trip = json.loads(msg.data)
        except ValueError:
            pass

    def on_ptt(self, msg: Bool):
        if msg.data:
            self.speaker.hold(True)  # stop talking and keep quiet while the student speaks
            self.recorder.start()
        else:
            audio = self.recorder.stop()
            self.speaker.hold(False)
            threading.Thread(target=self.respond, args=(audio,), daemon=True).start()

    def respond(self, audio):
        if not self.busy.acquire(blocking=False):
            self.speaker.say("One moment, I'm still thinking about the last thing you said.")
            return
        try:
            text = self.stt(trim_silence(audio))
            if not text:
                self.speaker.say("Sorry, I didn't catch that.")
                return
            self.get_logger().info(f"heard: {text}")
            context = f"{self.trip.get('state')}, {self.trip.get('detail', '')}".strip(", ")
            try:
                reply = self.agent.handle(text, context)
            except Exception as e:  # network loss or usage limit: voice pauses (spec section 16)
                self.get_logger().error(f"agent failed: {e}")
                reply = "Sorry, I can't reach my voice service right now. Please use the phone app."
            self.speaker.say(reply)
        finally:
            self.busy.release()

    # Tools ----------------------------------------------------------------------
    def set_destination(self, room: str) -> str:
        try:
            key = room
            node = self.rg.resolve(room)
            key = next((k for k, d in self.rg.destinations.items() if d["node"] == node), room)
        except Exception:
            hits = self.rg.search(room, limit=3)
            if hits:
                return f"No room called {room}. Close matches: {', '.join(hits)}. Ask the student which one."
            return f"No room called {room} in AQ, WMC or MBC. Ask the student to repeat it."
        self.dest_pub.publish(String(data=key))
        return f"Guiding to {key}."

    def pause_or_resume(self) -> str:
        paused = bool(self.trip.get("paused"))
        self.pause_pub.publish(Bool(data=not paused))
        return "Resumed." if paused else "Paused."

    def confirm(self) -> str:
        self.confirm_pub.publish(Empty())
        return "Confirmed."


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
