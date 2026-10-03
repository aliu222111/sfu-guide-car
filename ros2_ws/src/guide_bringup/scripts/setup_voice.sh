#!/usr/bin/env bash
# The car's voice on the laptop (spec section 9): ElevenLabs Scribe listens, Gemini answers,
# ElevenLabs speaks (Piper on the laptop when the network is down).
# Run once from the repository root, in the Python environment ROS uses:
#   bash ros2_ws/src/guide_bringup/scripts/setup_voice.sh
# sounddevice needs PortAudio: `sudo apt install libportaudio2` on Ubuntu, or
# `micromamba install -c conda-forge portaudio` in a RoboStack environment.
set -euo pipefail
python3 -m pip install --upgrade elevenlabs piper-tts sounddevice google-genai
mkdir -p voices
python3 -m piper.download_voices en_US-lessac-medium --data-dir voices
python3 -c "import sounddevice" || echo "sounddevice cannot find PortAudio: install it (see the top of this script)"
echo
echo "Done. Put the keys in the environment:"
echo "  export ELEVENLABS_API_KEY=...   # https://elevenlabs.io/app/settings/api-keys"
echo "  export GEMINI_API_KEY=...       # https://aistudio.google.com/apikey"
echo "then check microphone, speaker and Gemini:  ros2 run guide_voice voice_check"
