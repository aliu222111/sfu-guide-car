"""Voice agent node (spec section 9).

Hold the push-to-talk button on the car (``voice/ptt`` from the serial bridge),
speak, release. ElevenLabs Scribe writes down what was said; rooms in it are
found on the map (rooms.py) and given to the agent (Gemini) as hints. A room the
student names is read back ("AQ 3046, the Multifaith Centre, on the 3000 level
of the Academic Quadrangle. Is that right?") and the trip starts only on a yes.
When the car is lost and asks for the nearest door number, a room the student
reads out is a sign read (``signs``). The agent's reply and every mission
announcement (``guide/speech``: doors, elevators, turns, arrival) are spoken
with ElevenLabs, or Piper when the network is down.
"""

from __future__ import annotations

import json
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from sfu_map.route import RouteGraph

from .agent import SYSTEM, Actions, VoiceAgent
from .cloud import Unheard
from .rooms import answer_in, rooms_in, spoken, suggestion, understand
from .speech import Recorder, Speaker, audio_device, trim_silence


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_agent")
        graph = self.declare_parameter("graph", "maps/route_graph.json").value
        self.rg = RouteGraph.load(graph)
        self.trip = {"state": "idle", "detail": ""}
        P = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        log = self.get_logger().warn
        piper = (P("piper_voice", "voices/en_US-lessac-medium.onnx"), audio_device(P("speaker_device", "")))
        tts = P("tts", "elevenlabs")
        if tts == "elevenlabs":  # ElevenLabs voice (ELEVENLABS_API_KEY), Piper when it cannot
            from .cloud import TTS_MODEL, VOICE_ID, ElevenLabsSpeaker

            self.speaker = ElevenLabsSpeaker(*piper, voice_id=P("elevenlabs_voice_id", VOICE_ID),
                                             model=P("elevenlabs_tts_model", TTS_MODEL), log=log)
        elif tts == "piper":
            self.speaker = Speaker(*piper)
        else:
            raise ValueError(f"tts must be elevenlabs or piper, not {tts!r}")
        self.recorder = Recorder(audio_device(P("mic_device", "")))
        from .cloud import STT_MODEL, ElevenLabsTranscriber

        self.stt = ElevenLabsTranscriber(model=P("elevenlabs_stt_model", STT_MODEL), log=log)
        if not os.environ.get("ELEVENLABS_API_KEY"):
            log("ELEVENLABS_API_KEY is not set: the car cannot hear, and speaks with Piper")
        self.pending = None  # a destination read back to the student, waiting for their yes
        actions = Actions(self.set_destination, self.pause_or_resume, self.confirm)
        llm = P("llm", "gemini")
        model = self.declare_parameter("claude_model", "claude-haiku-4-5").value
        auth = self.declare_parameter("claude_auth", "subscription").value
        self.agent = None  # without one, offline() still understands rooms, yes, no and stop
        if llm == "gemini":  # Google Gemini (GEMINI_API_KEY)
            from google import genai
            from google.genai import types

            from .gemini_agent import MODEL as GEMINI_MODEL
            from .gemini_agent import GeminiVoiceAgent

            try:
                client = genai.Client(http_options=types.HttpOptions(timeout=15000))
                self.agent = GeminiVoiceAgent(client, actions, model=P("gemini_model", GEMINI_MODEL))
                self.get_logger().info(f"voice agent: Gemini {self.agent.model}")
            except Exception as e:  # no GEMINI_API_KEY: rooms, yes, no and stop still work
                self.get_logger().error(f"Gemini unavailable ({e}); answering with the built-in phrases only")
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
        self.sign_pub = self.create_publisher(String, "signs", 10)  # a door number the student read out
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
        try:
            if msg.data:
                self.speaker.hold(True)  # stop talking and keep quiet while the student speaks
                self.recorder.start()
            else:
                audio = self.recorder.stop()
                self.speaker.hold(False)
                threading.Thread(target=self.respond, args=(audio,), daemon=True).start()
        except Exception as e:  # no microphone, or it went away: say so instead of dying
            self.get_logger().error(f"microphone: {e}")
            self.speaker.hold(False)
            self.speaker.say("I can't hear you: my microphone isn't working. Please use the phone app.")

    def respond(self, audio):
        if not self.busy.acquire(blocking=False):
            self.speaker.say("One moment, I'm still thinking about the last thing you said.")
            return
        try:
            self.speaker.say(self.reply(trim_silence(audio)))
        finally:
            self.busy.release()

    def reply(self, audio) -> str:
        try:
            text = self.stt(audio, extra_terms=(self.pending,) if self.pending else ())
        except Unheard:
            return "Sorry, I can't hear you without the network right now. Please use the phone app."
        if not text:
            return "Sorry, I didn't catch that."
        self.get_logger().info(f"heard: {text}")
        heard = understand(text, self.rg.destinations, self.trip)  # the trip says the car's floor and pose
        rooms = heard.rooms
        if self.trip.get("state") == "lost":
            if rooms:
                # The car asked for the nearest door number: a room read out is where we are.
                self.sign_pub.publish(String(data=json.dumps({"text": rooms[0], "conf": 0.8, "source": "voice"})))
                return f"Thanks. I'll look around {rooms[0]}."
            if heard.missing:
                return f"I can't find {heard.missing} on my map. Could you read me the number on another door?"
        elif not rooms and answer_in(text) is None:
            # A room that is not on the map, or a place asked for by name or kind: say what it
            # could be, and a yes goes there (the same read-back-then-yes as a room).
            said, proposed = suggestion(heard, self.rg.destinations)
            if said:
                self.pending = proposed
                return said
        context = f"{self.trip.get('state')}, {self.trip.get('detail', '')}".strip(", ")
        if self.pending:
            context += f"; waiting for the student to confirm the destination {self.pending}"
        if rooms:
            context += f"; rooms that match what was heard: {', '.join(rooms[:3])}"
        if self.agent is None:
            return self.offline(text, rooms)
        try:
            return self.agent.handle(text, context)
        except Exception as e:  # Gemini unreachable or over its limit: the plain answers still work
            self.get_logger().error(f"agent failed: {e}")
            return self.offline(text, rooms)

    def offline(self, text: str, rooms: list[str]) -> str:
        answer = answer_in(text)
        if answer == "stop":
            return self.pause_or_resume()
        if answer == "yes":
            said = self.confirm()
            return said if self.dest_sent(said) else "Thanks."
        if answer == "no" and self.pending:
            self.pending = None
            return "Okay. Which room would you like?"
        if len(rooms) == 1:
            return self.set_destination(rooms[0], speak=True)
        return "Sorry, I can't reach my voice service right now. Please say just a room number, or use the phone app."

    # Tools ----------------------------------------------------------------------
    def set_destination(self, room: str, speak: bool = False) -> str:
        """Find the room and read it back; the trip starts on the student's yes (confirm)."""
        try:
            node = self.rg.resolve(room)
            key = next((k for k, d in self.rg.destinations.items() if d["node"] == node), room)
        except Exception:
            heard = understand(room, self.rg.destinations, self.trip)
            hits = heard.rooms or heard.suggestions or self.rg.search(room, limit=3)
            if len(hits) == 1 and not heard.missing:
                key = hits[0]
            elif hits:
                return (f"No room called {room} on the map. The closest real ones: {', '.join(hits[:3])}. "
                        "Ask the student if they meant one of those.")
            else:
                return f"No room called {room} in AQ, WMC or MBC. Ask the student to check the room number."
        self.pending = key
        where = spoken(key, self.rg.destinations.get(key))
        if speak:
            return f"{where}. Is that right?"
        return f"Found {where}. Read that back and ask the student if it is right; start only after they say yes."

    def pause_or_resume(self) -> str:
        paused = bool(self.trip.get("paused"))
        self.pause_pub.publish(Bool(data=not paused))
        return "Resumed." if paused else "Paused."

    @staticmethod
    def dest_sent(said: str) -> bool:
        return said.startswith("Starting the trip")

    def confirm(self) -> str:
        if self.pending:  # yes to the destination read back: go
            key, self.pending = self.pending, None
            self.dest_pub.publish(String(data=key))
            return f"Starting the trip to {key}."
        self.confirm_pub.publish(Empty())  # yes to a door, an elevator, "still with me?"
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
