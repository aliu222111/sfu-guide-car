from setuptools import find_packages, setup

package_name = "guide_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # Plus the piper binary and a voice model, see README.
    install_requires=["setuptools", "elevenlabs", "google-genai", "piper-tts", "sounddevice", "numpy", "sfu_map"],
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Voice agent: ElevenLabs Scribe and voice, Gemini with three tools, Piper offline",
    license="MIT",
    entry_points={"console_scripts": ["voice_agent = guide_voice.voice_node:main",
                                      "voice_check = guide_voice.voice_check:main"]},
)
