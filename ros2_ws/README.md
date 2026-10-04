# ros2_ws: the car's software (phases 1 to 6)

ROS 2 Jazzy on Ubuntu 24.04, all Python. Design: `docs/superpowers/specs/2026-10-03-campus-guide-design.md`.
Maps come from phase 0 (`sfu_map/`, output in `maps/`).

| package | what | spec |
|---|---|---|
| `guide_bridge` | serial link to the ESP32 that works the RC hand controller (+ its firmware), the Ackermann maths, phone IMU bridge, velocity gate | 3, 6, 10 |
| `guide_perception` | floor segmentation, person detection, sign reader, vanishing-point heading, rear-camera student tracker | 6, 7, 9 |
| `guide_mission` | trip state machine and mission node, localization manager, RTAB-Map to GIS alignment | 6, 8 |
| `guide_ui` | phone web app and its server, Bluetooth signal monitor | 9 |
| `guide_voice` | voice agent: ElevenLabs Scribe, Gemini (or Claude) with three tools, ElevenLabs voice (Piper offline) | 9 |
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
pip install elevenlabs google-genai piper-tts sounddevice bleak   # phase 6 (claude-agent-sdk for llm: claude)
pip install google-genai elevenlabs                              # optional: Gemini, ElevenLabs
cd ros2_ws && colcon build && source install/setup.bash
```

Models, once (nothing is trained): `python ros2_ws/src/guide_perception/scripts/export_models.py --out models`
(`--segformer b2` with an NVIDIA GPU). Voice: `bash ros2_ws/src/guide_bringup/scripts/setup_voice.sh` (ElevenLabs, Gemini, sounddevice, and Piper with its voice in `voices/` for when the network is down).

Launch everything from the repository root: `maps/`, `models/`, `voices/` and `rtabmap_db/` are found
relative to it (or set `GUIDE_CAR_ROOT`).

## Voice: ElevenLabs hears and speaks, Gemini answers

The default (`guide.yaml`, `voice_agent`), over the laptop's campus WiFi: ElevenLabs Scribe writes
down what the student said, told the building names and codes as key terms. `rooms.py` finds the
rooms in it however they were said ("A Q thirty forty six", "West Mall twenty-eight thirty") and
gives them to Gemini (`llm: gemini`), which picks one of three tools (set destination, pause or
resume, confirm) and words the reply. The car reads the room back ("AQ 3046, the Multifaith Centre,
on the 3000 level of the Academic Quadrangle. Is that right?") and starts only on a yes. On the way
it asks out loud for doors to be held and elevator buttons pressed, and waits for a yes. If it gets
lost it asks for the number on the nearest door, and that becomes a position fix. ElevenLabs speaks;
without the network Piper does, and the car says it cannot hear just now. The student's voice goes
to ElevenLabs: put a sign on the car. Once:

```bash
bash ros2_ws/src/guide_bringup/scripts/setup_voice.sh     # packages and the offline Piper voice
export ELEVENLABS_API_KEY=...                              # https://elevenlabs.io/app/settings/api-keys
export GEMINI_API_KEY=...                                  # https://aistudio.google.com/apikey (paid tier: see below)
ros2 run guide_voice voice_check --devices                 # find the USB speaker and microphone
ros2 run guide_voice voice_check --say "Could you hold the door?" --speaker 3   # the ElevenLabs voice alone
ros2 run guide_voice voice_check --echo --talk --mic 2 --speaker 3   # ElevenLabs alone: it says back what Scribe heard
ros2 run guide_voice voice_check --talk --mic 2 --speaker 3   # Enter, speak, Enter: hear Gemini's reply
```

Put the device names or numbers that worked into `mic_device` and `speaker_device` in `guide.yaml`,
then `guide_car.launch.py voice:=true`. On the car the push-to-talk button (ESP32 GPIO4, `B 1`/`B 0`
on the serial line) starts and stops the recording; the car stops talking while it is held. Without
WiFi the car still speaks (Piper) and says it cannot hear just now; the phone app still works.

## Claude instead of Gemini: your subscription, not an API key

`voice_agent: llm: claude`.

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

## Gemini and ElevenLabs: the switches

Switches in `guide.yaml`, keys in the environment:

| what | setting | key |
|---|---|---|
| Voice agent's brain on Gemini | `voice_agent: llm: gemini` | `GEMINI_API_KEY` |
| Cloud sign reading on Gemini | `sign_reader: backend: gemini` (0.2 Hz) | `GEMINI_API_KEY` |
| ElevenLabs voice | `voice_agent: tts: elevenlabs` | `ELEVENLABS_API_KEY` |
| ElevenLabs speech-to-text | `voice_agent: stt: elevenlabs` | `ELEVENLABS_API_KEY` |

The voice falls back to Piper on any failure, so keep it installed.
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

The fake car in `sim.launch.py` is an Ackermann car (`wheelbase`, `max_steer` parameters of
`fake_robot`): it cannot turn on the spot, like the real one. In `hil.launch.py`, `understeer:=1.1`
is the simulated car's geometric / actual turn rate.

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

**Walking the campus with the rig** (optional, before or instead of the handheld mapping drive).
Mount the two front cameras on a rigid plate, 10 cm apart, at the SAME height and tilt they will
have on the car (RTAB-Map localizes by what the car later sees; a map taken from another height
matches poorly), level side to side. Calibrate the pair once (phase 1), then on the walking laptop:

```bash
bash ros2_ws/src/guide_bringup/scripts/record_walk.sh aq3-concourse 720p   # Ctrl-C to stop
bash ros2_ws/src/guide_bringup/scripts/map_from_walk.sh walks/<date>-aq3-concourse AQ-03
```

`record_walk.sh` records the raw pair, camera info, the phone's IMU and GPS (Sensor Logger pushing to
this laptop) and the Adafruit GPS if plugged in, into `walks/`; `720p` captures 1280x720 (1080p
cameras; about 11 GB an hour), `360p` the car's 640x360. Walk at the car's pace (0.5 m/s), no fast
turns, down every corridor of the floor and back to where you started (a loop closure), past each
door you will align at. `map_from_walk.sh` replays the bag into the same mapping pipeline the car
uses and writes `rtabmap_db/<floor>.db`; align it at three doors (phase 1). The same bags test
calibration, exposure and the perception nodes offline (`ros2 bag play`).

**2, the RC car.** The car is a stock RC car with its stock pistol-grip controller; the laptop drives
the controller. An ESP32-WROOM-32E's two DAC pins stand in for the controller's steering wheel and
trigger potentiometers.

*Wiring.* Open the controller. Find the steering and trigger potentiometers; each has three wires:
the two rails and the wiper (middle). Measure the rail voltage with the controller on: if it is above
3.3 V (many run the pots from the battery), feed the pots from the ESP32's 3V3 pin instead, or put an
op-amp level shifter between each DAC and wiper, because the ESP32 outputs at most 3.3 V and its
pins die above it. Lift each wiper wire off its potentiometer (desolder or cut), and connect GPIO25 to
the steering wiper wire and GPIO26 to the throttle wiper wire. Bias each wiper wire to its centre
voltage with two resistors (10 k to the rail, 10 k to ground, adjusted to give the centre voltage
measured below) so the car sits still while the board boots or resets. Tie the ESP32's ground to the
controller's ground. Push-to-talk button: GPIO4 to ground. HC-SR04 (one camera only): trigger GPIO18,
echo GPIO19 through a 1 k / 2 k divider (its echo is 5 V). Keep the controller's antenna clear of the
laptop and the car's chassis.

*Measure.* Before lifting the wipers, with the controller on and a multimeter on each wiper: the
voltage with the wheel and trigger at rest, at full left, full right, full trigger and full
brake/reverse. Put the six numbers at the top of `guide_bridge/firmware/guide_motor/guide_motor.ino`
(`STEER_CENTER_V` ... `THROTTLE_REVERSE_V`); the board refuses to run if any is above 3.3 V.

*Flash.* Arduino IDE, board "ESP32 Dev Module", 115200 baud. Power the board before switching the
controller on (both outputs sit at centre). Serial port: `ls -l /dev/serial/by-id/`; the ESP32 is the
CP2102 or CH340 entry (usually `/dev/ttyUSB0`, the GPS adapter `/dev/ttyUSB1`): pass
`motor_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102...` and `gps_port:=...` to the launch to be
safe. `sudo usermod -aG dialout $USER`, then log out and in.

*Bench test, wheels off the floor.* `ros2 run guide_bridge serial_bridge --ros-args -p port:=/dev/ttyUSB0`
then `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"`: the wheels must turn
forwards (else `throttle_reversed: true` in guide.yaml); `"{linear: {x: 0.3}, angular: {z: 0.5}}"`: the
front wheels must steer left (else `steer_reversed: true`). Unplug the USB: the ESP32 centres both
within 300 ms.

*Calibrate* (guide.yaml, `serial_bridge`): `wheelbase` (front axle to rear axle); `max_steer`:
drive a slow full-lock circle, measure its radius R to the car's centre line, `max_steer = atan(wheelbase / R)`;
`steer_trim`: the steering value at which it rolls straight (try 0.02 steps); `throttle_start`: the
smallest throttle at which it rolls; `throttle_limit`: keep 0.4 until everything else works;
`speed_at_limit`: time 10 m at `throttle_limit`. The serial bridge then trims the throttle on the
measured speed (`speed_trim_gain`), so the slow speeds near people are real.

*First drive.* Outdoors, an empty paved area, the controller in your hand (switching it off stops the
car): `ros2 launch guide_bringup guide_car.launch.py perception:=false` and a short destination.
Without perception the gate does not wait for the person detector: never run that way near people.

*Phone.* Sensor Logger on the iPhone, lying flat on the chassis, screen up, top edge forward: enable
accelerometer, gravity, gyroscope, orientation, magnetometer and location; turn ON "Standardise
Units & Frames" (without it the iOS axes are flipped and the heading turns the wrong way); HTTP push
to `http://<laptop>:8765/data` at 100 Hz. Check the compass once: with the phone's top edge pointing
east, `/phone/imu` yaw must read about 0; if it reads about 90 degrees, set `world_yaw_offset: 1.571`
under `phone_imu` in guide.yaml (iOS reports true-north heading with x north). Outdoor position still
needs the Adafruit GPS on the laptop: the phone's location is published on `/phone/fix` but not fused.

Draw the outdoor route in `sfu_map/connectors.yaml` (`outdoor_links`) and rebuild maps.

**3, perception.** `guide_car.launch.py` with the default `perception:=true`.

**4, indoor driving.** `guide_car.launch.py start_floor:=AQ-03` once the floor's database and alignment exist.

**5, mission and phone app.** Open `http://<laptop>:8080` on the phone (same WiFi).

**One camera.** If a webcam dies or the stereo mount will not hold calibration: calibrate the remaining
camera with `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/stereo/left/image_raw camera:=/stereo/left`,
save it as `left.yaml`, measure its height and tilt into the URDF (`cam_z`, `cam_pitch`; 1 cm of height is
2 % of distance), fit the ultrasonic bumper, then add `cameras:=mono` to any launch above. Spec section 18.

**iPhone in front, webcam behind** (spec section 19). On the phone: Record3D, Settings, USB Streaming
mode, then press record. On the laptop: `pip install record3d` (and `libusbmuxd`). Mount the phone
upright, landscape, rear camera forward, on the stereo mount, and set `mount_x`/`mount_y` under `iphone:`
in guide.yaml to match the URDF's `iphone_camera`. Check the axes once: push the car forward, `/vo`
`linear.x` must be positive; turn it left, `angular.z` positive. Map each floor by carrying the phone:
`ros2 launch guide_bringup mapping.launch.py floor:=AQ-03 fresh:=true cameras:=iphone`, align as in
phase 1, then add `cameras:=iphone rear_camera:=true` to `guide_car.launch.py` (the webcam is the rear
camera; calibrate it as `rear.yaml`, below). In the simulator: `hil.launch.py cameras:=iphone` with
`mode:=map` first, as for stereo.

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

Also: `/bumper/range` (sensor_msgs/Range, serial bridge -> velocity gate, from the HC-SR04) and
`/odometry/local` -> serial bridge (the throttle's speed trim).

## Tests

```bash
for p in guide_bridge guide_claude guide_mission guide_perception guide_ui guide_voice; do
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/$p:src/guide_claude python3 -m pytest -q src/$p/test; done
```
