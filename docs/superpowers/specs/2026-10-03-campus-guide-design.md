# SFU Campus Guide Car: navigation software design

Date: 2026-10-03
Status: approved in conversation, awaiting written review
Owner: Alex Liu (navigation software, chassis)

## 1. Goal

A small wheeled robot that leads a student from wherever they are on the SFU Burnaby campus to a classroom door in the Academic Quadrangle (AQ), West Mall Centre (WMC) or Maggie Benston Centre (MBC). Outdoor paths are the primary use. Indoor corridors, including level changes, are the extension. The student picks a destination on a phone web app, or speaks to the car if they have no phone. The car drives ahead at walking pace, waits if the student falls behind, speaks turn instructions, and asks the student to press elevator buttons and hold doors.

Success for version one:

- Any classroom in AQ, WMC or MBC is selectable and routable from any outdoor entrance or corridor node in the graph.
- The car completes an outdoor route of at least 200 m with one turn and one ramp without touching a curb, a person or grass.
- The car completes an indoor route on one AQ concourse level of at least 60 m with two turns, ending within 2 m of the correct classroom door.
- One elevator change with the student pressing the button, then correct relocalization on the new floor.
- Voice: a student with no phone can say the destination and be understood on the first or second attempt in a quiet corridor.

## 2. Decisions from the interview

Stated by Alex:

- Environment: door to room, outdoors and indoors, level changes included, outdoors used more.
- Buildings for version one: AQ, WMC, MBC.
- Compute: an NVIDIA laptop that can dual-boot Ubuntu, riding on the car. A Raspberry Pi 4 is in hand but is not enough for stereo SLAM and is not used for perception.
- Sensors in hand: two 720p USB webcams, Adafruit Ultimate GPS, WiFi, Bluetooth.
- Chassis: Alex is building it. Steering geometry is open. An IMU will be added. Wheel encoders will not, for now.
- Interaction: phone web app is the main controller. An LLM voice agent on the car serves students without a phone. Follow-aware behaviour is required. Elevators and doors are handled by asking the student.
- Learned vision wanted: floor segmentation, room-sign reading, person detection, and glass handling.
- Localization: Alex was not sure. RTAB-Map with IMU is the recommendation and is adopted here.
- Time: not a constraint. Build the general system.
- No models are trained in this project. Every model is pretrained.

Assumptions made by Claude and open to correction:

- Differential drive chassis, so the car can turn in place in corridors. If Alex builds Ackermann steering instead, Nav2 needs the Smac Hybrid-A* planner and the Regulated Pure Pursuit controller, and in-place rotation recovery is disabled. Everything else is unchanged.
- The car drives at 0.6 to 1.0 m/s outdoors and 0.4 to 0.6 m/s indoors.
- SFU WiFi is available for the voice agent's LLM calls. The phone app runs over the laptop's own hotspot, so it does not depend on campus WiFi.
- The laptop runs Ubuntu 24.04 and ROS 2 Jazzy.

## 3. Hardware

In hand: two 720p USB webcams, Adafruit Ultimate GPS (MTK3339, about 3 m accuracy, up to 10 Hz, uFL socket for an external antenna), NVIDIA laptop, Raspberry Pi 4.

To build (Alex): chassis, motor driver, microcontroller for motors, IMU.

Required additions:

- IMU: any 6-axis part with a ROS 2 driver or a trivial serial stream, for example BNO055 (fuses its own orientation, easiest), ICM-20948 or MPU-6050.
- External active GPS antenna on the uFL socket, mounted on top of the car. The car sits low next to concrete and the on-board patch antenna will lose lock.
- Stereo mount: the two webcams rigidly fixed 10 to 15 cm apart, level, facing forward, on a plate that does not flex. Any flex invalidates the calibration.
- Microcontroller (Arduino, ESP32 or the Pi 4) that takes velocity commands over USB serial and drives the motors, and streams IMU readings back.
- Speaker and USB microphone on the laptop for the voice agent.

Strongly recommended, not required: motors with magnetic encoders. Without encoders the position estimate freezes whenever the cameras lose texture (blank wall, glass, darkness, fast turn). With encoders the car coasts through several metres of blindness. Section 6 designs for cameras plus IMU only and treats encoders as the first upgrade.

Optional, only if tests show glass close calls: one ultrasonic sensor on the front bumper.

Rear camera for student tracking is phase 6, not required for version one.

## 4. Software architecture

Everything runs on the laptop under ROS 2 Jazzy. Each box below is a ROS 2 node or an existing package.

```
webcams ──> stereo_calibrated ──> stereo depth (OpenCV SGBM) ──┐
   │                                                          ├──> Nav2 costmaps ──> Nav2 planner/controller ──> cmd_vel ──> serial bridge ──> motors
   │        floor segmentation (SegFormer, ADE20K) ───────────┤
   │        person detection (YOLO11) ────────────────────────┘
   │
   ├──> RTAB-Map (stereo + IMU) ──> visual odometry, loop closure, indoor maps
   │
   └──> sign reader (PaddleOCR) ──> absolute position fixes at known doors

GPS ──> navsat_transform ──┐
IMU ───────────────────────┼──> robot_localization EKF ──> map -> odom -> base_link
RTAB-Map odometry ─────────┘

SFU GIS ──> map pipeline (offline) ──> per-floor occupancy grids + campus route graph
                                              │
                                              └──> mission node ──> Nav2 goals, speech, waits for student
                                                        ▲
phone web app (FastAPI + WebSocket) ────────────────────┤
voice agent (Whisper -> Claude tools -> Piper) ─────────┘
```

Packages: `rtabmap_ros`, `nav2_bringup`, `robot_localization`, `image_pipeline` (stereo calibration and rectification), `usb_cam` or `v4l2_camera`, `nmea_navsat_driver`, `rosbridge_server` for the phone app. Custom packages: `sfu_map` (offline pipeline, pure Python, no ROS dependency), `guide_perception` (floor, person, sign nodes), `guide_mission` (route graph walker and state machine), `guide_ui` (phone app server), `guide_voice` (voice agent), `guide_bridge` (serial to microcontroller).

## 5. Map layer

### Sources, all public, all verified on 2026-10-03

SFU runs a public ArcGIS REST server at `https://viewsfu.its.sfu.ca/fsgis/rest/services`. The Room Finder app reads from it. No login.

- `RoomFinder/RoomFinder2024_RoomDisplay/MapServer/0`: every room polygon on campus (18,879 features) with `bl_abbr`, `fl_name`, `rm_id`, `rm_type`, `rm_cat`. Corridors have `rm_cat = CIRCULATION`. Query by `bl_abbr='AQ'`, paginate with `resultOffset` in steps of 1000, request `outSR=4326&f=geojson`.
- `RoomFinder/RoomFinder2024_CampusMapB/MapServer/0`: building footprints.
- `Vertisee/Vertisee_Transportations/MapServer/6`: roads.
- `Vertisee/Vertisee_BuildingFloorplan_P_2020/MapServer`: wall polylines per level, but coverage for AQ, WMC, MBC is near zero. Not used.

Already pulled into `data/sfu_gis/sfu_rooms_AQ_WMC_MBC.geojson`: 2,441 rooms across all floors of the three buildings, 316 of them corridors. AQ 3000 level alone has 17 corridor polygons covering 4,412 m2 and the lecture halls 3003, 3005, 3149, 3150, 3153, 3154, 3159. See `data/sfu_gis/aq_3000_level_preview.png`.

`data/sfu_gis/campus-geometry.json` is the footprints and roads snapshot from the open-source student project DHu06/SFU-Burnaby-Interactive-map, already projected to local metres with origin latitude 49.278945, longitude -122.916558. Its indoor nodes are synthetic and are not used.

Outdoor footpaths: OpenStreetMap `highway=footway|path|steps|pedestrian` inside the campus bounding box (49.2735, -122.9270) to (49.2830, -122.9050). The Overpass query returned HTTP 406 during the session and still needs a working query form or a Geofabrik extract. `data/sfu_pdf/sfu_Accessibility_campus_map_2024.pdf` marks the step-free routes and is the manual check on which outdoor paths the car can take.

The building key plans at sfu.ca/fs are behind SFU CAS login and are not needed.

### Pipeline, `sfu_map`, offline, pure Python with shapely

Input: the GeoJSON files above and a hand-edited `connectors.yaml`.
Output, committed to the repo:

1. One occupancy grid per building floor in Nav2's PGM plus YAML format at 5 cm resolution. Corridor polygons are free, everything else occupied, glass walls included because they are room boundaries in the GIS. One outdoor occupancy grid at 10 cm resolution where walkable paths, widened by 2.5 m, are free.
2. One campus route graph as JSON. Node types: corridor junction, room door, elevator, stairs, entrance, outdoor junction. Edge attributes: length in metres, type (corridor, outdoor, elevator, door), and whether the car can traverse it alone. Stairs edges exist for the UI but are never traversable by the car.
3. A destination index: room id to door node.

Corridor junction nodes come from the medial axis of the corridor polygons, simplified to about one node per 5 m plus every junction. Room door nodes are placed on the corridor edge nearest each room polygon. Elevators, outdoor entrances and the links between floors and buildings are listed by hand in `connectors.yaml`, because the GIS does not encode them. Expect about 40 hand entries for three buildings.

Coordinate frame: one `map` frame for the whole campus, East-North-Up metres, origin at latitude 49.278945, longitude -122.916558 to match the existing snapshot. Floors share XY and are distinguished by a floor id, not by Z. Nav2 loads one floor grid at a time.

## 6. Localization

### Outdoors

`robot_localization` runs one EKF fusing IMU orientation and angular rate, RTAB-Map's visual odometry twist, and GPS position converted to the map frame by `navsat_transform_node`. GPS is a 3 m measurement, so the filter leans on odometry over seconds and on GPS over tens of seconds. Under the AQ overhangs and covered walkways GPS drops out and the filter coasts on vision plus IMU.

### Indoors

RTAB-Map in stereo mode with IMU, first in mapping mode on a drive of each corridor, then in localization mode. Each floor is one RTAB-Map database, aligned to the GIS frame once by driving to three known door nodes and solving a 2D similarity transform. The alignment is stored and reused.

Absolute fixes from signs: the sign reader (section 7) reads room numbers. When a confident read matches a door node on the current floor and the current position is within 8 m of that door, the mission node publishes a pose correction to RTAB-Map. This is how the car recovers after an elevator ride.

### Transitions

Crossing an entrance node switches the EKF between the GPS-fused outdoor configuration and the indoor configuration where GPS is ignored and RTAB-Map localization is enabled against that floor's database. After an elevator ride the car has no position until a sign read or an RTAB-Map loop closure on the new floor, so it rotates slowly in place until one arrives.

### Without encoders

Visual odometry needs texture. On blank walls, glass, darkness or motion blur it reports no motion and the EKF holds the last position, with the IMU still tracking heading. Mitigations: speed limits above, turn rate capped at 45 degrees per second, cameras set to fixed exposure, and a recovery behaviour that stops and rotates slowly when RTAB-Map reports lost. Adding encoders later is a one-line EKF change.

## 7. Perception

All nodes consume the rectified stereo pair at 640 x 360, which is enough for everything below and halves GPU load. Calibration uses `image_pipeline`'s stereo calibrator with a printed checkerboard, done once per mount.

- Stereo depth: OpenCV SGBM on CPU, output as a point cloud into Nav2's voxel or obstacle layer. Measured first on the real corridor. Fast-FoundationStereo on the GPU is the upgrade only if SGBM leaves holes on glass or plain walls that cause collisions.
- Floor segmentation: SegFormer pretrained on ADE20K (floor, sidewalk, path, road, grass, stairs classes). Pixels in the lower image that are not floor-like are projected to the ground plane and added to the costmap as obstacles. This is the sensor for curbs, grass edges, stair tops and the base of glass walls. Runs outdoors and indoors.
- Person detection: YOLO11 nano with ByteTrack. People within 3 m ahead reduce the speed limit; a person within 1 m stops the car. AGPL licence is fine for a student project and is noted for any later commercial use.
- Sign reading: PaddleOCR on the upper third of the left image at 2 Hz, filtered to the pattern `<building abbreviation> <three or four digits>`. Used only indoors.
- Glass: three layers. The GIS map keeps the planner off glass walls. Floor segmentation marks the base of glass as an obstacle. Glass doors are door nodes where the car stops and asks for help. An ultrasonic bumper sensor is the fallback if tests show close calls.

Webcam pitfalls that the build must handle: no hardware sync between the two cameras, so capture is software-timestamped and RTAB-Map's approximate sync is used; auto exposure is disabled and a fixed exposure is set per environment; rolling shutter smears on fast turns, handled by the turn rate cap.

## 8. Planning and mission

Nav2 does all driving within one floor or outdoors: global planner on the static grid plus live obstacle layers, local controller producing velocity commands. Default plugins: NavFn global planner and DWB controller for differential drive, with inflation radius set to half the car width plus 25 cm.

The mission node owns the trip:

1. Resolve start node (nearest node to current position) and destination door node.
2. Dijkstra on the route graph with edge lengths, excluding stairs and any edge marked not traversable.
3. Walk the resulting node list. Consecutive nodes on the same floor or outdoor segment become one Nav2 goal sequence. At an entrance node, switch localization mode. At an elevator node, stop, say which floor to press, wait for the student's confirmation on the phone or by voice, then wait for a sign read or loop closure on the new floor before continuing. At a door node, stop, ask the student to hold the door, wait for confirmation, drive through.
4. Announce each turn 5 m ahead. On arrival, announce the room and stop.

States: idle, routing, driving, waiting_for_student (heartbeat lost or follow check failed), waiting_for_help (elevator or door), lost (relocalizing), arrived, aborted. Every state is visible on the phone app and in RViz.

## 9. Interaction

### Phone web app, main controller

Served by the laptop over its own WiFi hotspot at a fixed address. Plain HTML and JavaScript, no build step, talking to ROS through `rosbridge_server` over WebSocket. Screens: search a room, see the route drawn on the floor plan outline, trip status, three buttons (I pressed it, I am holding the door, pause). Spoken instructions on the phone use the browser's built-in speech synthesis. Room search is a text box over the destination index.

Follow-aware: the page sends a heartbeat every second while open. Two missed heartbeats put the mission into waiting_for_student and the car stops and says so. Bluetooth signal strength from the phone, read by the laptop, is the second signal: when it falls below a calibrated threshold for 3 s the car waits. Phase 6 adds a rear camera with YOLO tracking the student directly.

### Voice agent on the car, for students without a phone

Push-to-talk button on the car or a wake word. Audio to text with faster-whisper on the GPU. The text goes to a Claude model with exactly three tools: `set_destination(room)`, `pause_or_resume()`, `confirm()`. The model's reply text goes to Piper text-to-speech on the car's speaker. The same spoken instructions the phone gives are also played on the car speaker. Follow-aware without a phone is the walking-pace limit plus a spoken "still with me?" at each turn that waits for a yes. Claude model: Haiku 4.5 for latency and cost; tool calls only, no free conversation beyond clarifying the room.

## 10. Chassis interface

`guide_bridge` is a ROS 2 node that opens one USB serial port. Downlink at 20 Hz: one line `V <linear m/s> <angular rad/s>\n`. Uplink at 50 Hz or more: `I <qw> <qx> <qy> <qz> <gx> <gy> <gz> <ax> <ay> <az>\n` from the IMU, and if encoders are added later, `E <left ticks> <right ticks>\n`. Any microcontroller that can parse those lines works. A watchdog on the microcontroller stops the motors if no `V` line arrives for 500 ms.

If the chassis is Ackermann, the `V` line stays the same and the microcontroller converts angular velocity to a steering angle. The Nav2 plugin change is noted in section 2.

## 11. Phases and exit criteria

Phase 0, map pipeline, no hardware. `sfu_map` turns the committed GIS data into floor grids and the route graph. Exit: a route from the Convocation Mall entrance node to AQ 3150's door node prints as a node list with correct floors, and the AQ 3000 grid loads in Nav2's map server and RViz.

Phase 1, handheld stereo rig. Mount, calibrate, rectify, SGBM depth, RTAB-Map mapping while carrying the rig down the AQ concourse. Exit: a loop of the concourse closes in RTAB-Map and the point cloud is recognisably the corridor.

Phase 2, chassis and outdoor driving. Serial bridge, IMU, GPS, EKF, Nav2 on the outdoor grid. Exit: the 200 m outdoor success criterion.

Phase 3, perception. Floor segmentation and YOLO feeding the costmap, sign reader publishing fixes. Exit: the car stops for a person, refuses to cross a curb onto grass, and reads 9 of 10 room signs it drives past.

Phase 4, indoor driving. RTAB-Map localization against the phase 1 map, aligned to the GIS frame. Exit: the 60 m indoor success criterion.

Phase 5, mission and phone app. Route graph walker, entrance transitions, elevator and door protocol, phone app with heartbeat. Exit: one door-to-room trip including an elevator ride.

Phase 6, voice and follow tracking. Voice agent, Bluetooth signal threshold, rear camera tracking. Exit: the voice success criterion.

Phase 0 starts immediately and needs nothing but the laptop. Phases 1 and 2 can run in parallel if a second person builds the chassis.

## 12. Risks

- Camera-only blind spots freeze the position estimate. Mitigated by speed limits and recovery rotation; fixed properly by encoders.
- Unsynchronised webcams degrade stereo on fast motion. Mitigated by the turn rate cap and approximate sync.
- SFU's terraced campus means most building-to-building walks use stairs. Car routes will be longer via ramps. The accessibility map is the source of truth for ramps until the OSM footpath pull is fixed.
- Rain and wet concrete glare. Not addressed in version one; the car stays indoors in rain.
- Hand-entered connectors (elevators, entrances) are the most error-prone data. Each one is verified on site before being marked traversable.
- GIS room polygons date from the 2024 edition. Renovated areas may differ. Verified by the phase 1 drive.

## 13. Reference projects and verdicts

Researched on 2026-10-02 and 2026-10-03.

- rpng/open_vins: visual-inertial odometry filter. Needs hardware-synchronised stereo and an IMU. Gives odometry only, no map or place recognition. Not used; RTAB-Map covers odometry plus mapping with these webcams.
- comma.ai openpilot: lane keeping for cars on comma's own hardware over CAN. Nothing transfers but the process-per-sensor architecture, which ROS 2 already provides.
- autowarefoundation/vision_pilot: monocular highway ADAS for CUDA GPUs. Wrong domain.
- autowarefoundation/autoware: full self-driving stack for LiDAR and workstation GPUs. Not deployable here; its ROS 2 conventions are the same ones Nav2 uses.
- utiasASRL/vtr3: visual teach and repeat. The simpler fallback if the Nav2 route proves too much; gives up multi-building routing.
- nvidia/LocateAnything-3B: vision-language object grounding, non-commercial licence. Not localization. Not used.
- amap-cvlab/ABot-Recon: monocular 3D reconstruction for A100-class GPUs. Not needed with stereo.
- bilawalsidhu/gods-eye-view: browser globe with live feeds and a voice agent. Its voice-agent-with-tools pattern is the model for section 9's voice agent. Nothing else transfers.
- cartesiancs/map3d: React Three Fiber city renderer from OpenStreetMap. Could skin the phone app's map later. Not navigation.
- DHu06/SFU-Burnaby-Interactive-map: source of the footprint snapshot and the proof that the SFU GIS server is public.
- "godseye": ambiguous name, five unrelated repos. The one Alex linked is gods-eye-view above.

Pretrained models chosen: SegFormer ADE20K (Hugging Face `nvidia/segformer-b2-finetuned-ade-512-512`), YOLO11n with ByteTrack (Ultralytics), PaddleOCR, faster-whisper small, Piper. Fallback upgrade for depth: NVlabs/Fast-FoundationStereo.

## 14. Repository layout

```
sfu-guide-car/
  docs/superpowers/specs/      this spec, later the implementation plan
  data/sfu_gis/                room polygons, footprints, previews
  data/sfu_pdf/                public SFU campus and accessibility maps
  sfu_map/                     phase 0, pure Python
  ros2_ws/src/
    guide_bridge/
    guide_perception/
    guide_mission/
    guide_ui/
    guide_voice/
    guide_bringup/             launch files and parameters
```

## 15. Not in scope for version one

Buildings other than AQ, WMC and MBC. Driving in rain. Stairs. Autonomous elevator button pressing. Student accounts, trip history, or any backend beyond the laptop. Training any model. Running perception on the Raspberry Pi 4.
