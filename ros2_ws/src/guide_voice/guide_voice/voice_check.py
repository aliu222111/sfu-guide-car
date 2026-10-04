"""Bench check of the car's voice, no ROS and no car needed (spec section 9).

    ros2 run guide_voice voice_check --devices          # list microphones and speakers
    ros2 run guide_voice voice_check                    # speak, listen 4 s, transcribe (Scribe), ask Gemini, speak the reply
    ros2 run guide_voice voice_check --talk             # a conversation: Enter starts and stops each recording
    ros2 run guide_voice voice_check --text "take me to AQ 3046"   # skip the microphone
    ros2 run guide_voice voice_check --file question.wav           # transcribe a recording instead
    ros2 run guide_voice voice_check --echo --talk      # ElevenLabs only: say back what Scribe heard
    ros2 run guide_voice voice_check --say "Could you hold the door, please?"   # just the speaker

Run from the repository root (voices/ and maps/ live there). ElevenLabs
(ELEVENLABS_API_KEY) hears and speaks, Piper speaks when it cannot; the
transcript goes to Gemini (GEMINI_API_KEY). The rooms found in each transcript,
and what the agent would do on the car (set a destination, pause, confirm), are
printed.
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

from .cloud import VOICE_ID, ElevenLabsSpeaker, ElevenLabsTranscriber, Unheard
from .rooms import answer_in, spoken, suggestion, understand
from .speech import SAMPLE_RATE, Recorder, Speaker, audio_device, read_wav, resample, trim_silence

GREETING = "Hello, I'm the SFU guide car. Where would you like to go?"


def say(speaker: Speaker, text: str):
    print(f"car: {text}")
    try:
        with tempfile.TemporaryDirectory() as d:
            wav = Path(d) / "say.wav"
            t = time.monotonic()
            speaker.synthesize(text, wav)
            print(f"  (spoken in {time.monotonic() - t:.1f} s)")
            speaker.play(wav)
    except Exception as e:
        print(f"  speaker failed: {type(e).__name__}: {e}")


def listen(rec: Recorder, seconds: float | None) -> np.ndarray:
    if seconds is None:
        input("Press Enter, speak, then press Enter again... ")
        rec.start()
        input("  recording, Enter to stop ")
    else:
        print(f"Speak now ({seconds:.0f} s)...")
        rec.start()
        time.sleep(seconds)
    audio = rec.stop()
    level = float(np.sqrt(np.mean(audio**2))) if audio.size else 0.0
    print(f"  {audio.size / SAMPLE_RATE:.1f} s recorded, level {level:.3f}" + ("  (silent: check the microphone)" if level < 0.003 else ""))
    return audio


def make_agent(model: str):
    if not os.environ.get("GEMINI_API_KEY") and not os.environ.get("GOOGLE_API_KEY"):
        print("No GEMINI_API_KEY: transcripts are not sent to Gemini.")
        return None
    from google import genai
    from google.genai import types

    from .agent import Actions
    from .gemini_agent import GeminiVoiceAgent

    def act(name):
        def f(*args):
            print(f"  [car would: {name}{'(' + ', '.join(map(str, args)) + ')' if args else ''}]")
            return f"Guiding to {args[0]}." if name == "set_destination" else "Done."
        return f

    actions = Actions(act("set_destination"), act("pause_or_resume"), act("confirm"))
    client = genai.Client(http_options=types.HttpOptions(timeout=15000))
    return GeminiVoiceAgent(client, actions, model=model)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--devices", action="store_true", help="list audio devices and exit")
    ap.add_argument("--talk", action="store_true", help="keep talking: Enter starts and stops each recording")
    ap.add_argument("--text", help="what the student says, instead of the microphone")
    ap.add_argument("--file", help="a WAV recording to transcribe, instead of the microphone")
    ap.add_argument("--echo", action="store_true", help="say back what was heard instead of asking Gemini")
    ap.add_argument("--say", help="speak this and exit (tests the speaker and the ElevenLabs voice)")
    ap.add_argument("--voice-id", default=VOICE_ID, help="ElevenLabs voice id (from its voice library)")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--voice", default="voices/en_US-lessac-medium.onnx")
    ap.add_argument("--gemini-model", default=None)
    ap.add_argument("--mic", default="", help="device name or index (see --devices)")
    ap.add_argument("--speaker", default="")
    args = ap.parse_args(argv)

    if args.devices:
        import sounddevice as sd

        print(sd.query_devices())
        return 0
    eleven = bool(os.environ.get("ELEVENLABS_API_KEY"))
    if not eleven:
        print("No ELEVENLABS_API_KEY: Piper speaks, and nothing can be heard.")
    speaker = (ElevenLabsSpeaker(args.voice, audio_device(args.speaker), voice_id=args.voice_id, log=print) if eleven
               else Speaker(args.voice, audio_device(args.speaker)))
    if not Path(args.voice).exists():
        print(f"No Piper voice at {args.voice}: run ros2_ws/src/guide_bringup/scripts/setup_voice.sh")
    if args.say:
        say(speaker, args.say)
        return 0
    say(speaker, GREETING)

    from .gemini_agent import MODEL

    agent = None if args.echo else make_agent(args.gemini_model or MODEL)
    stt = ElevenLabsTranscriber(log=print) if eleven and not args.text else None
    keys = {}
    if Path("maps/route_graph.json").exists():
        from sfu_map.route import RouteGraph

        keys = RouteGraph.load("maps/route_graph.json").destinations
    rec = Recorder(audio_device(args.mic))
    while True:
        if args.text:
            text = args.text
        else:
            if args.file:
                data, rate = read_wav(Path(args.file))
                audio = data[:, 0].astype(np.float32) / 32768.0
                audio = resample(audio, rate)
            else:
                try:
                    audio = listen(rec, None if args.talk else args.seconds)
                except Exception as e:
                    print(f"Microphone failed: {e}\nPick a device with input channels from --devices and pass --mic <number>.")
                    return 1
            if stt is None:
                return 1
            t = time.monotonic()
            try:
                text = stt(trim_silence(audio))
            except Unheard:
                text = ""
            print(f"heard ({time.monotonic() - t:.1f} s): {text!r}")
        heard = understand(text, keys) if keys and text else None
        said = ""
        if heard is not None:
            print(f"  rooms on the map: {heard.rooms or 'none'}"
                  + (f"; not on the map: {heard.missing}" if heard.missing else "")
                  + (f"; suggested: {heard.suggestions}" if heard.suggestions else ""))
            if heard.rooms:
                said = f"{spoken(heard.rooms[0], keys.get(heard.rooms[0]))}. Is that right?"
            elif answer_in(text) is None:
                said = suggestion(heard, keys)[0]
        if args.echo:
            say(speaker, f"I heard: {text}. {said}".strip() if text else "I didn't catch that.")
        elif said and not heard.rooms:
            say(speaker, said)  # what the car says itself, without asking Gemini
        elif agent is not None and text:
            t = time.monotonic()
            reply = agent.handle(text, "idle")
            print(f"  (Gemini {time.monotonic() - t:.1f} s)")
            say(speaker, reply)
        if not args.talk or args.text or args.file:
            return 0


if __name__ == "__main__":
    sys.exit(main())
