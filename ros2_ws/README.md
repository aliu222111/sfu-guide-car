# ros2_ws: the car's software (phases 1 to 6)

ROS 2 Jazzy on Ubuntu 24.04, all Python. Design: `docs/superpowers/specs/2026-10-03-campus-guide-design.md`.
Maps come from phase 0 (`sfu_map/`, output in `maps/`).

| package | what | spec |
|---|---|---|
| `guide_bridge` | serial link to the motor microcontroller (+ Arduino firmware), phone IMU bridge, velocity gate | 3, 6, 10 |
| `guide_perception` | floor segmentation, person detection, sign reader, vanishing-point heading, rear-camera student tracker | 6, 7, 9 |
| `guide_mission` | trip state machine and mission node, localization manager, RTAB-Map to GIS alignment | 6, 8 |
| `guide_ui` | phone web app and its server, Bluetooth signal monitor | 9 |
| `guide_voice` | voice agent: faster-whisper, Claude Haiku 4.5 with three tools, Piper | 9 |
| `guide_claude` | Claude for the voice agent and the cloud sign reader through your Claude subscription (Claude Agent SDK) | 9, 16 |
| `guide_bringup` | robot description, launch files, Nav2 / EKF / navsat / camera parameters, simulator | 4 |

Logic lives in plain modules (`trip.py`, `velocity_gate.py`, `vanishing.py`, ...) with tests that need no ROS.
Nodes are thin wrappers.

## Install

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-robot-localization \
  ros-jazzy-rtabmap-ros ros-jazzy-usb-cam ros-jazzy-image-pipeline ros-jazzy-nmea-navsat-driver \
  ros-jazzy-rosbridge-server ros-jazzy-xacro python3-serial v4l-utils
pip install -e sfu_map                                   # route graph, used by mission and voice
pip install -r ros2_ws/src/guide_perception/requirements-perception.txt
pip install faster-whisper sounddevice bleak claude-agent-sdk   # phase 6 (anthropic too for API-key mode)
pip install google-genai elevenlabs                              # optional: Gemini, ElevenLabs
cd ros2_ws && colcon build && source install/setup.bash
```

Models, once (nothing is trained): `python ros2_ws/src/guide_perception/scripts/export_models.py --out models`
(`--segformer b2` with an NVIDIA GPU). Piper voice: download `en_US-lessac-medium.onnx` and its `.json` into `voices/`.

Launch everything from the repository root: `maps/`, `models/`, `voices/` and `rtabmap_db/` are found
relative to it (or set `GUIDE_CAR_ROOT`).

## Claude: your subscription, not an API key

The voice agent (and the sign reader with `backend: claude`) use Claude Haiku 4.5 through your own
Claude Pro/Max subscription via the Claude Agent SDK, which runs the Claude Code CLI bundled in
`claude-agent-sdk`. On the car laptop, once:

```bash
claude setup-token                      # sign in with your subscription; prints a long-lived token
echo 'export CLAUDE_CODE_OAUTH_TOKEN=<token>' >> ~/.bashrc
```

Nothing else: no API key. If `ANTHROPIC_API_KEY` is set it would take precedence in Claude Code, so
subscription mode removes it from the node's environment. The agent gets no Claude Code tools at all
(no shell, no files), only the three car tools, and ignores your personal Claude Code settings.
This is for your own use of the car; Anthropic does not allow offering claude.ai login to other
people's products. Subscription usage limits apply: the voice agent uses a few requests per trip;
the cloud sign reader should run at 0.2 Hz (`guide.yaml`), not the local OCR's 1 to 2 Hz. To use an
API key instead: `claude_auth: api_key` (voice) or `backend: claude_api` (signs).

## Gemini and ElevenLabs (optional)

Switches in `guide.yaml`, keys in the environment:

| what | setting | key |
|---|---|---|
| Voice agent's brain on Gemini | `voice_agent: llm: gemini` | `GEMINI_API_KEY` |
| Cloud sign reading on Gemini | `sign_reader: backend: gemini` (0.2 Hz) | `GEMINI_API_KEY` |
| ElevenLabs voice | `voice_agent: tts: elevenlabs` | `ELEVENLABS_API_KEY` |
| ElevenLabs speech-to-text | `voice_agent: stt: elevenlabs` | `ELEVENLABS_API_KEY` |

The voice and transcription fall back to Piper and faster-whisper on any failure, so keep those installed.
Use a paid Gemini key: on the free tier Google may use the prompts and camera frames. With
`stt: elevenlabs` students' voices leave the laptop; tell them (a sign on the car).

## Try it without hardware

```bash
ros2 launch guide_bringup sim.launch.py start_floor:=AQ-03 start:=entrance:aq_convocation_mall
# phone app: http://localhost:8080   or:
ros2 topic pub --once /guide/destination std_msgs/String "{data: 'AQ 3046'}"
```

`fake_robot` integrates `/cmd_vel`, fakes the phone heartbeat and an empty sensor world, and the real Nav2,
velocity gate, mission, localization manager, UI server and rosbridge run on the AQ-03 grid.

End-to-end check, playing the student (it answers door and elevator prompts and exits 0 on arrival):

```bash
ros2 run guide_bringup sim_trip "AQ 6106" --timeout 600   # AQ-03 -> elevator -> AQ-06 -> two doors
```

RTAB-Map without cameras: `stereo_sim` ray-traces a textured ring corridor (60 m round) as a
rectified stereo pair; `rtabmap_check` scores odometry drift, map nodes, loop closures and
localization against its ground truth.

```bash
ros2 launch guide_bringup rtabmap.launch.py mode:=mapping handheld:=true fresh:=true database:=/tmp/sim.db &
ros2 run guide_bringup stereo_sim --ros-args -p laps:=1.3 &
ros2 run guide_bringup rtabmap_check --mode mapping --duration 215
# then mode:=localization (same database), stereo_sim -p start_s:=20, rtabmap_check --mode localization --start-s 20
```

Tank steering in the sim: `sim.launch.py turn_slip:=1.5 pivot_stall:=0.2` makes the fake car turn at
two thirds of the asked rate and not pivot below 0.2 rad/s; add `turn_gain:=1.5 min_pivot_rate:=0.15` for
the serial bridge's compensation.

One camera (spec section 18): `ros2 run guide_perception mono_odometry` on the left image publishes `/vo`;
`rtabmap_check --mode odometry` scores it against `stereo_sim`.

Kill leftover nodes between runs (`pkill -f ros2_ws/install/`): a second mission node from an old run
fights the new one.

## Phase runbook

**1, handheld stereo rig.** Mount and level the cameras, then
`bash $(ros2 pkg prefix guide_bringup)/share/guide_bringup/scripts/set_exposure.sh 150`, calibrate with
`ros2 run camera_calibration cameracalibrator --approximate 0.05 --size 8x6 --square 0.025 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left`
and save `left.yaml`/`right.yaml` into `~/.ros/camera_info/`. Map a floor:
`ros2 launch guide_bringup mapping.launch.py floor:=AQ-03 fresh:=true`, carry the rig round the concourse
and close the loop. Align it to the GIS frame at three or more doors:
`ros2 run guide_mission align_floor --floor AQ-03 --graph maps/route_graph.json --out rtabmap_db/alignments.yaml --no-scale AQ-03:door:3150=x,y ...`
with x,y read from RTAB-Map at each door.

**2, chassis and outdoor driving.** Flash `guide_bridge/firmware/guide_motor/guide_motor.ino` (set pins,
track width, max track speed, `MIN_PWM_DRIVE`, `MIN_PWM_PIVOT`). Tank steering slips in turns: with the serial bridge and phone
IMU running (no Nav2, no gate), `ros2 run guide_bridge calibrate_turn --yes` spins the car in place and prints
`turn_gain` and `min_pivot_rate` for `guide.yaml`; rerun with `--current-gain <new gain>` to check. Phone: Sensor Logger, HTTP push to `http://<laptop>:8765/data` at 100 Hz.
Draw the outdoor route in `sfu_map/connectors.yaml` (`outdoor_links`) and rebuild maps, then
`ros2 launch guide_bringup guide_car.launch.py perception:=false` (without perception the gate does not
wait for the person detector; never run that way near people).

**3, perception.** `guide_car.launch.py` with the default `perception:=true`.

**4, indoor driving.** `guide_car.launch.py start_floor:=AQ-03` once the floor's database and alignment exist.

**5, mission and phone app.** Open `http://<laptop>:8080` on the phone (same WiFi).

**One camera.** If a webcam dies or the stereo mount will not hold calibration: calibrate the remaining
camera with `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/stereo/left/image_raw camera:=/stereo/left`,
save it as `left.yaml`, measure its height and tilt into the URDF (`cam_z`, `cam_pitch`; 1 cm of height is
2 % of distance), fit the ultrasonic bumper, then add `cameras:=mono` to any launch above. Spec section 18.

**6, voice and follow.** Calibrate the rear camera like the left one (`cameracalibrator ... image:=/rear/image_raw camera:=/rear`)
and save it as `rear.yaml` next to `left.yaml`: the follow tracker measures distance on rectified images. Then `voice:=true` (your Claude subscription, see above), `voice_mode:=true` for a student
without a phone, `rear_camera:=true`, `rssi:=true`.

## Topics

| topic | type | from -> to |
|---|---|---|
| `/guide/destination` | String | phone app, voice -> mission |
| `/guide/confirm`, `/guide/cancel` | Empty | phone app, voice -> mission |
| `/guide/pause` | Bool | phone app, voice -> mission |
| `/guide/heartbeat` | Empty | phone app (1 Hz) -> mission |
| `/guide/phone_rssi`, `/guide/student_distance` | Float32 | rssi monitor, rear camera -> mission |
| `/guide/state`, `/guide/speech`, `/guide/route` | String (JSON / text) | mission -> phone app, voice |
| `/guide/localization_mode` | String (JSON `{"floor": ...}`) | mission -> localization manager |
| `/guide/localized`, `/guide/outdoor` | Bool | localization manager -> mission, gate, sign reader |
| `/guide/set_pose`, `/guide/sign_fix` | PoseWithCovarianceStamped | mission -> localization manager |
| `/guide/indoor_pose`, `/guide/gps_odom`, `/guide/imu_abs` | pose / odom / imu | localization manager -> ekf_global |
| `/guide/corridor_axis` | Float64 | mission -> vp_heading |
| `/guide/vp_heading` | Imu (yaw only) | vp_heading -> ekf_global |
| `/cmd_vel_nav` | Twist | Nav2 -> velocity gate |
| `/guide/cmd_vel_manual`, `/guide/drive_enable`, `/guide/recovery_rotate` | Twist / Bool | mission -> velocity gate |
| `/cmd_vel` | Twist | velocity gate -> serial bridge |
| `/people/nearest` | Float32 | person detection -> velocity gate |
| `/floor/obstacles`, `/floor/free`, `/stereo/points2` | PointCloud2 | perception -> Nav2 costmaps |
| `/signs` | String (JSON) | sign reader -> mission |
| `/phone/imu`, `/phone/mag`, `/phone/fix`, `/phone/alive` | | phone bridge -> EKFs, gate |
| `/vo` | Odometry | RTAB-Map stereo odometry -> EKFs, localization manager |
| `/voice/ptt` | Bool | serial bridge (car button) -> voice |

TF: `map -> odom` from ekf_global, `odom -> base_link` from ekf_local, the rest from the URDF. RTAB-Map
publishes no TF on the car; its poses reach the EKF through the localization manager in the GIS frame.

## Tests

```bash
for p in guide_bridge guide_claude guide_mission guide_perception guide_ui guide_voice; do
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/$p:src/guide_claude python3 -m pytest -q src/$p/test; done
```
