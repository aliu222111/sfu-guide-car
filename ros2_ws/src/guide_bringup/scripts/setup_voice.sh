#!/usr/bin/env bash
# The car's voice on the laptop (spec section 9): Whisper listens, Gemini answers, Piper speaks.
# Run once from the repository root, in the Python environment ROS uses:
#   bash ros2_ws/src/guide_bringup/scripts/setup_voice.sh
# sounddevice needs PortAudio: `sudo apt install libportaudio2` on Ubuntu, or
# `micromamba install -c conda-forge portaudio` in a RoboStack environment.
set -euo pipefail
python3 -m pip install --upgrade piper-tts faster-whisper sounddevice google-genai
mkdir -p voices
python3 -m piper.download_voices en_US-lessac-medium --data-dir voices
# Whisper downloads its model on first use; do it now, while there is internet.
python3 -c "from faster_whisper import WhisperModel; WhisperModel('base', device='cpu', compute_type='int8')"
python3 -c "import sounddevice" || echo "sounddevice cannot find PortAudio: install it (see the top of this script)"
echo
echo "Done. Put your Gemini key in the environment (https://aistudio.google.com/apikey):"
echo "  export GEMINI_API_KEY=..."
echo "then check microphone, speaker and Gemini:  ros2 run guide_voice voice_check"
