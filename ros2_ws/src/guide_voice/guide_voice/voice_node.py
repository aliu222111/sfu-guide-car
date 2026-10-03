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

from .agent import Actions, VoiceAgent
from .speech import Recorder, Speaker, Transcriber, trim_silence


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_agent")
        graph = self.declare_parameter("graph", "maps/route_graph.json").value
        self.rg = RouteGraph.load(graph)
        self.trip = {"state": "idle", "detail": ""}
        self.speaker = Speaker(
            self.declare_parameter("piper_voice", "voices/en_US-lessac-medium.onnx").value,
            self.declare_parameter("piper_bin", "piper").value,
            self.declare_parameter("player", "aplay").value,
        )
        self.recorder = Recorder(self.declare_parameter("mic_device", "").value or None)
        self.stt = Transcriber(
            self.declare_parameter("whisper_model", "base").value,
            self.declare_parameter("whisper_device", "auto").value,
            self.declare_parameter("whisper_compute", "int8").value,
        )
        import anthropic

        self.agent = VoiceAgent(anthropic.Anthropic(timeout=15.0), Actions(self.set_destination, self.pause_or_resume, self.confirm),
                                model=self.declare_parameter("claude_model", "claude-haiku-4-5").value)
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
            except Exception as e:  # network loss: voice pauses (spec section 16)
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
