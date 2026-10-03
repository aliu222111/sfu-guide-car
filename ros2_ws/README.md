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
pip install faster-whisper sounddevice bleak anthropic   # phase 6
cd ros2_ws && colcon build && source install/setup.bash
```

Models, once (nothing is trained): `python ros2_ws/src/guide_perception/scripts/export_models.py --out models`
(`--segformer b2` with an NVIDIA GPU). Piper voice: download `en_US-lessac-medium.onnx` and its `.json` into `voices/`.

Launch everything from the repository root: `maps/`, `models/`, `voices/` and `rtabmap_db/` are found
relative to it (or set `GUIDE_CAR_ROOT`).

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
wheel base, max wheel speed). Phone: Sensor Logger, HTTP push to `http://<laptop>:8765/data` at 100 Hz.
Draw the outdoor route in `sfu_map/connectors.yaml` (`outdoor_links`) and rebuild maps, then
`ros2 launch guide_bringup guide_car.launch.py perception:=false` (without perception the gate does not
wait for the person detector; never run that way near people).

**3, perception.** `guide_car.launch.py` with the default `perception:=true`.

**4, indoor driving.** `guide_car.launch.py start_floor:=AQ-03` once the floor's database and alignment exist.

**5, mission and phone app.** Open `http://<laptop>:8080` on the phone (same WiFi).

**6, voice and follow.** `voice:=true` (needs `ANTHROPIC_API_KEY`), `voice_mode:=true` for a student
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
for p in guide_bridge guide_mission guide_perception guide_ui guide_voice; do
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/$p python3 -m pytest -q src/$p/test; done
```
