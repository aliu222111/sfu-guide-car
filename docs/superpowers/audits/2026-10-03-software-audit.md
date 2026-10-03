# Software audit, phases 0 to 6

Date: 2026-10-03
Scope: `sfu_map/` and every package in `ros2_ws/src/`, against the design spec
(`docs/superpowers/specs/2026-10-03-campus-guide-design.md`).

## How it was audited

1. **Unit tests.** 85 tests, all passing: sfu_map 20, guide_bridge 21, guide_mission 22,
   guide_perception 11, guide_ui 3, guide_voice 8. Logic modules are tested without ROS; the trip
   tests run against the real campus graph.
2. **Build.** `colcon build` of all six packages in ROS 2 Jazzy (RoboStack); launch files and
   parameter files load.
3. **End-to-end simulation.** `sim.launch.py` runs the real Nav2 servers on the AQ-03 grid, plus the
   velocity gate, mission node, localization manager, UI server and rosbridge. `fake_robot` stands
   in for motors, cameras and the phone. Each run sends a destination over ROS and waits for the trip
   to end.
4. **Code review.** Four independent reviews, one each for:
   - mission and localization;
   - bridge and safety gate;
   - perception;
   - UI, voice and map pipeline.

   Each finding was reproduced (a script or an exact code path) before being fixed. Every fix has a
   regression test where the logic allows one.

## Found by simulation

| # | Defect | Fix |
|---|---|---|
| S1 | Nav2's bt_navigator would not start. The default navigate-to-pose tree needs a BackUp server, which the car does not run. | Only the NavigateThroughPoses navigator is configured, with a custom tree that has no BackUp. |
| S2 | Door and elevator nodes sat 0.4 m from the wall, inside the car's footprint plus inflation. Nav2 could never reach them, so the car stalled 1.4 m short. | Nodes are inset 0.8 m from the wall (0.6 or 0.4 m in narrow corridors). |
| S4 | The route graph promised paths the Nav2 grid cannot drive. The skeleton was computed on a 10 cm raster, where GIS wall lines about 0.1 m thick can vanish; the 5 cm Nav2 grid keeps them. An elevator trip from Convocation Mall was routed through a wall. | The skeleton now uses the grid's 5 cm resolution. The build removes any edge whose ends are not connected on the grid once walls are grown by the car's 0.19 m half-width (14 edges, listed in the build report). A build check and a test keep "graph pieces the grid splits" at 0. |
| S5 | Leaving the elevator, the car crept forward again, deeper into the shaft, and could not plan on the new floor. | The exit creep turns 180° in place, then drives out forwards, so the cameras and person stop face the way the car moves. |
| S6 | Inferred and hand-entered doorways were edges in the graph but walls in the Nav2 grid, so the car could not pass a door the student was holding. | The build opens a 0.9 m strip in the floor grid along every same-floor door edge. A test checks every doorway is drivable. |
| S7 | The localization manager sent its first map load before map_server was active and never retried, so after a Nav2 restart a floor switch would be lost. | Loads are retried every second until map_server confirms. In simulation the first request failed and the retry succeeded one second later. |
| S3 | Every 5 m corridor node was a mandatory Nav2 waypoint. After cutting a corner the car turned back for the missed node and stalled. | The mission sends sparse waypoints: junctions, turns and transitions at least 6 m apart, at most 20 m between them, plus the goal. The behaviour tree counts a waypoint as passed within 2 m. |

Results on the final code and maps, run with `ros2 run guide_bringup sim_trip`:

| Trip | Route | Runs | Result |
|---|---|---|---|
| Convocation Mall entrance to AQ 3046 | 50 m, one turn | 5 | 4 arrived (80 s each, about 0.5 m from the door node, correct "on your right"). 1 run was stopped by Nav2 14 s in, mid-corridor. The mission handled it correctly ("I'm stuck..."; a confirm retries), but that run's Nav2 log was overwritten, so the cause is not known. Suspected: planner timeouts under CPU load in this 4-core container. |
| Convocation Mall entrance to AQ 3150 (the spec's phase 0 exit route) | 275 m, three turns | 2 | 2 arrived in about 7.5 min at the 0.6 m/s indoor cap, 0.56 m from the door, "on your left". |
| Convocation Mall entrance to AQ 6106 | AQ-03, elevator to AQ-06, two doorways | 1 | Arrived 0.53 m from the door. Sequence: call → board (creep) → ride → turn round and exit → AQ-06 map loaded and relocalized → two held doors → a "slight right". Six Nav2 goals succeeded. |

## Found by review, all fixed

### Safety: the car could move when it should not

| Finding | Where | Fix |
|---|---|---|
| A NaN velocity command became full speed and full turn rate: Python's `min`/`max` with NaN returns the cap. | velocity_gate.py | Non-finite commands are dropped. A NaN person distance counts as "person at 0 m". |
| Resuming a trip (localization regained, "try again", "still with me") ignored pause and follow-loss and sent Nav2 goals. | trip.py | One `_resume()` is the only way into driving. It checks pause, follow and localization first. |
| A localization loss that arrived while waiting was forgotten, so the car later drove blind. Starting a trip before the first fix also drove blind. | trip.py | `_resume()` enters `lost` when not localized. A floor change resets "localized" until the localization manager confirms it. |
| Pause or follow-loss during `lost` kept the recovery rotation spinning. | trip.py | The rotation stops while paused or alone, and restarts when both clear. |
| The elevator creep could not be cancelled or paused. A new trip right after a cancel replayed the stale creep ahead of Nav2. | mission_node.py | The creep pauses while paused or alone. It is cleared on cancel, abort and disable. |
| Enable, recovery and phone-liveness flags latched forever. If the mission node crashed during recovery, the car spun indefinitely; if the phone bridge died, the outdoor speed cap stayed at 1.0 m/s. | velocity_gate.py, mission_node.py | The gate times every flag out after 1.5 s, falling back to disabled, no rotation, and phone lost. The mission re-sends its flags at 2 Hz. |
| Reversing went from forward to full reverse in one 50 ms tick. | velocity_gate.py | A sign change outputs zero for one tick, then ramps up. |
| One malformed HTTP POST from anyone on the WiFi killed the phone IMU node. JSON NaN also reached the EKF. | phone_imu.py, phone_imu_node.py | Types, finiteness, coordinate ranges and timestamps are validated. A failing push is logged and dropped. |
| rosbridge on port 9090 let any WiFi client publish `drive_enable` and `cmd_vel_manual`. | launch | rosbridge only accepts the phone app's own topics, and no services or actions. |
| The firmware treated the tail of an overlong serial line as a new command. | guide_motor.ino | The rest of the line is discarded up to its newline. |

### Correctness

| Finding | Where | Fix |
|---|---|---|
| The ground plane was 8 cm above the floor, because `base_link` was at axle height. Every curb, grass edge and person distance came out about 14 % short. | URDF | `base_link` now lies on the floor, so costmap heights are heights above the floor. |
| The startup localization mode was published before discovery and lost. Indoors, the localization manager stayed in outdoor mode: it fused GPS and compass, dropped RTAB-Map and sign fixes, and the sign reader never switched on. | mission_node.py, localization_manager_node.py | The mode topic is transient-local. `guide/outdoor` is re-sent at 5 Hz. |
| The phone clock offset included the whole batch length, about 1 s late with batched pushes. | phone_imu_node.py | The offset uses the newest sample in each push. |
| 28 of 97 inferred doorways had zero length, with both ends in the same corridor piece, so the other piece stayed disconnected. | sfu_map graph.py | Each side is inset within its own piece. Gaps that cannot be placed are reported. |
| Threshold edges between a building and the outdoors were in neither occupancy grid. | sfu_map graph.py, trip.py | Thresholds are part of the outdoor grid. Door legs that cross floors are driven on it. |
| `camera_yaw_offset` was applied with the wrong sign. | vp_heading_node.py | Subtracted; the convention is documented. |
| PaddleOCR 3.x renamed its arguments and result format, so the default sign reader crashed at start. | requirements | Pinned to `paddleocr<3`, which is the API the code uses. |
| A tool call cut off by `max_tokens` left an orphan `tool_use`, which broke every later voice request with a 400. A refusal produced silence. | agent.py | Only text is stored from a final reply. An empty reply gets a spoken fallback. |
| The phone app could not return to search after a trip ended (1 Hz state re-sends forced the trip view). Pause toggles drifted between phone, voice and mission. | app.js, voice_node.py, mission_node.py | The trip view is forced only for active trips. A "New trip" button appears after the trip ends. `paused` is in the state message and the phone and voice both use it. |
| One lost heartbeat already stopped the car; the spec says two. | follow.py | The timeout is now 2.5 s. |
| A page reloaded mid-trip never drew the route. | mission_node.py | The route is transient-local and re-sent every 5 s. |
| An out-of-range phone sends no Bluetooth advertisements at all, so the signal check never tripped. | rssi_node.py | After 2 s of silence the node publishes -127 dBm. |
| The car's speaker talked over, and into the recording of, a student holding push-to-talk. A busy agent silently dropped speech. | speech.py, voice_node.py | Playback stops and holds while recording. A busy agent says "one moment". |
| Room names with letter prefixes or hyphens (`WMC S1300`, `AQ 5003-9`) did not resolve from compact input. | route.py | Buildings are matched against the known codes. All destination keys now round-trip. |
| A trip started after an aborted voice-mode trip inherited "still with me?". | trip.py | `start()` resets it. |

## Second pass: QA, safety audit and simplicity audit

After the hardware-in-the-loop simulator (`guide_sim`) existed, four reviews ran in parallel: a QA
pass (every test suite, build, launch files, docs against code), a safety audit of everything that
can move the car, an audit of the rest, and a simplicity audit (the ponytail checklist: delete,
use the standard library, reuse what exists, no speculative code). Every finding below was
reproduced or read in the code before it was fixed; all suites pass (bridge 36, claude 6,
mission 25, perception 18, ui 3, voice 13, sim 19, sfu_map 22).

Fixed:

- **Mission crash** when a sign fix reset the pose before any position was known (`None` unpacked).
  Now it waits 1 s for the reset to land, then replans from the pose.
- **Creep by time.** The "inch forward" recovery counted seconds, so a stalled car believed it had
  moved. It now counts metres and radians from odometry, and falls back to commanded motion only
  while the gate is in manual mode.
- **One PWM floor** for driving and pivoting. A pivot needs more (both tracks scrub), so the
  firmware has `MIN_PWM_DRIVE` 50 and `MIN_PWM_PIVOT` 55; the sim's slow straight now holds
  0.15 m/s within 0.02.
- **Dead ultrasonic failed open.** A disconnected sensor timed out and read as "nothing within
  4 m". The firmware now reports `U -1` for no echo (sensor missing) and `U 0` for out of range;
  the bridge drops `-1`, so the reading goes stale and, with one camera (`require_bumper`), the
  gate caps forward speed at the 0.15 m/s creep. The sim can unplug it.
- **Flags that never expired.** "Outdoor" and "localized" stayed set if their publisher died; both
  now time out (outdoor like the gate's other flags, localized after 1.5 s without an update).
- **Stale "localized" after a floor switch**: a `True` within 1 s of the mode change is ignored.
- **Claude session race**: prewarm and the first question could start two clients; connect and close
  now hold a lock, a timed-out request is cancelled, and an error reply is raised instead of spoken.
- **Student tracker** jumped to whoever was nearest. It now keeps its lock while the student is
  briefly hidden and re-locks only on someone near the last distance. The rear camera is rectified.
- **URDF** had both `base_footprint` and the EKF as parents of `base_link`; `base_footprint` removed.
- Smaller: `PIVOT_SPEED` threshold, `nav_failures` reset per trip, the sim's `pulseIn` returned
  twice the echo, and the judge looked up the destination door by name instead of the route graph.
- Simplicity: unused imports, parameters and state removed across sfu_map, the UI and the sim; the
  sim reuses the bridge's quaternion helper and the voice agent calls `run_tool` directly.

Found by the regression run after these fixes (hardware in the loop, one camera, entrance to AQ 3046):

- **A pivot floor set too high makes the car hunt.** With `MIN_PWM_PIVOT` 65 against tracks that
  break loose at 0.10 m/s per side, the slowest pivot the car could make was about 0.48 rad/s, so
  every small heading correction overshot; it spun in place near AQ 306 until localization timed
  out. Now 55 (slowest pivot about 0.2 rad/s), and the run arrives (194 s, 0 collisions, 0.15 m
  from the door, 0.41 m median localization error). On the real car, measure `MIN_PWM_PIVOT` with
  `calibrate_turn` rather than guessing high.
- **Paths hugged inside corners.** Wall cost ended 0.43 m out and fell steeply, so the car's centre
  came within 0.38 m of walls (its side within 0.19 m). Inflation now reaches 1 m on the global
  costmap (0.8 m local) with a gentler fall-off: closest approach 0.91 m, never under 0.5 m, and the
  trip still arrives (187 s, 0 collisions).

Found by the iPhone runs (`cameras:=iphone`, hardware in the loop, entrance to AQ 3046):

- **RTAB-Map relayed odometry as fixes.** By default it publishes a "localization" pose before it has
  matched anything; the localization manager reset the EKF to it on a cold start and the car began
  2.3 m off, inside a wall, where Nav2 cannot plan. RTAB-Map now publishes only real matches
  (`pub_loc_pose_only_when_localizing`), and a cold start needs two consecutive matches within
  0.5 m. Applies to stereo too.
- **The simulated serial bridge was uncalibrated.** Nav2's 0.1 rad/s pivots never broke the tracks
  loose and the car sat still until it gave up. `hil.launch.py` now passes what `calibrate_turn`
  would measure (`turn_gain` = turn slip, `min_pivot_rate` 0.2). On the car: run `calibrate_turn`
  before indoor driving.
- **The simulator's alignment used poses from before loop closures** (`hil_map`); now it reads the
  finished graph, as a person reading the finished map would. Alignment RMS 0.62 m to 0.22 m.
- Result: arrived in 156 s, 0.57 m from the door, median localization error 0.57 m, closest
  approach to a wall 0.70 m.

Open, from the same runs:

- **Side blind spot.** Twice a simulated person walking across the car's path from about 60 deg off
  its heading reached it before either the person detector or the depth cloud could see them (the
  front camera covers about +/-33 deg). The simulated people walk fixed paths and never yield, which
  real people mostly do, but a crowded concourse will test it. Options: slow to a crawl when
  starting or pivoting near people last seen at the edge of the view, side ultrasonic sensors, or a
  wider camera.
- **Pivot hunting.** With `min_pivot_rate` raising Nav2's small corrections to 0.2 rad/s, the car
  can swing back and forth in place for tens of seconds before it settles on a heading. A goal
  heading tolerance wider than the pivot overshoot, or a slower minimum pivot on a grippier floor,
  should fix it; tune on the real car.

Sensor fusion review (robot_localization, two EKFs), from the simulator traces:

| Camera setup | position error median / p95 | heading error median / p95 | mostly |
|---|---|---|---|
| stereo (RTAB-Map) | 0.18 / 0.86 m | 1.0 / 4.9 deg | balanced |
| one camera (sign fixes) | 0.41 / 1.18 m | 2.0 / 10.8 deg | along the corridor: distance drift between signs |
| iPhone (ARKit, RTAB-Map) | 0.57 / 1.02 m | 1.5 / 27 deg | heading before the first fix (no compass) |

- **Gyro bias, fixed.** The EKFs have no gyro bias state; a phone gyro is typically 0.1 to 1 deg/s
  off. The phone bridge now learns the yaw-rate bias whenever visual odometry has reported the car
  still for 2 s and subtracts it (`GyroBias`).
- **Kept as is.** An EKF, not a UKF: the motion is planar and gently non-linear, so a UKF costs CPU
  for nothing measurable. Visual odometry is fused as velocity (vx, vy, yaw rate), so the filters
  never see its drifting position; vy is fused because the car cannot slide sideways and the
  cameras confirm it. No IMU accelerations: a phone's are too noisy to integrate.
- **Open.** With one camera, error grows along the corridor between sign reads: more sign reads
  (the student can now say a door number) or wheel speed would bound it. The commanded speed is not
  used as a control input: the RC throttle is open loop, so it would be a poor measurement of speed.
  In iPhone mode there is no heading until the first RTAB-Map match; ARKit's heading-aligned mode
  (compass) is not exposed by Record3D.

Not fixed, by decision:

- **Nav2 death watchdog.** If Nav2 crashes mid-trip the mission waits for its action timeout.
  The velocity gate already stops the car when `cmd_vel` goes quiet, so this is a stall, not a hazard.
- **rosbridge has no authentication.** It listens on the laptop's hotspot only; add a token if the
  car ever joins a shared network.
- **Simulator limits.** The sim's world is the same grid the car plans on, so map errors cannot show
  up; the host Arduino core assumes 32-bit `int` where the Uno has 16-bit; firmware is rebuilt only
  when its source changes; the stereo pair has no separate per-camera bias; `stereo_sim` duplicates
  part of `guide_sim`.
- **Deferred simplifications** (worth doing, not worth the risk now): union-find in sfu_map could be
  `scipy.sparse.csgraph`; two Umeyama implementations and several quaternion helpers could share one
  module; the voice agents could share their tool list and preamble; launch files repeat include
  boilerplate. Encoder inputs stay as the specced extension point.

## Open items, not fixable without the hardware or site visits

- **Verify on site:** every connector in `sfu_map/connectors.yaml` (14 elevators, 1 entrance) and
  all 131 inferred doorways (door edges between corridor pieces) are `verified: false`.
- **Outdoor paths come from a drawing.** The walkways between AQ, MBC and WMC are traced from SFU's
  2024 accessibility map PDF (`sfu-map walkways`, georeferenced to the GIS footprints, accurate to a
  few metres), with 19 entrances from its level labels. Each entrance still needs a look on site.
- **Disconnected corridor pieces.** Open public rooms (lobbies, lounges, study areas, dining) now count
  as free space, and pieces facing one room across a thin wall are joined through it (two places,
  flagged "check on site"). From the AQ Convocation Mall entrance 1764 of 1923 destinations and 96
  of 98 classrooms are car-reachable; `maps/build_report.txt` lists the rest (WMC 2200 and 2202,
  MBC-04, some suites), each needing a `links` entry after a look on site.
- **RTAB-Map** is not packaged for RoboStack. Here it was built from source (core 0.23 without GUI,
  rtabmap_ros `ros2` branch, `rtabmap_util` patched not to require the GUI component); on Ubuntu
  `apt install ros-jazzy-rtabmap-ros` needs none of that. `stereo_sim` (a ray-traced textured 60 m
  ring corridor) and `rtabmap_check` test it against ground truth. Three launch bugs found and fixed:
  (1) `odom_frame_id` on the rtabmap node made it ignore the odometry topic and read TF at each image
  stamp, which fails for every frame stereo odometry skipped, so it kept 2 map nodes in 78 m (and on
  the car would have ignored `/odometry/local`); (2) `odom_sensor_sync` has the same failure on the
  handheld rig; (3) giving stereo odometry an IMU without orientation warped the first corner of
  every lap (3.4 m worst error), so `odom_imu` is now opt-in and the gyro reaches only the EKF.
  Result: mapping 1.3 laps, 0.2 % odometry drift, 208 map nodes, 46 loop closures; localization
  against that map from a different start, median error 0.47 m. Real webcams, real texture and real
  lighting are still untested.
- **Tank steering** (decided after the audit): `turn_gain` and `min_pivot_rate` in the serial bridge,
  `MIN_PWM_DRIVE` and `MIN_PWM_PIVOT` in the firmware, `calibrate_turn` to measure them, Rotation Shim controller. In the sim a
  car turning at 2/3 of the asked rate and stalling below 0.2 rad/s ends blocked without compensation
  and arrives with it (AQ 3046 in 86 s; AQ 6106 via elevator and two doors in 323 s). The real
  slip is floor-dependent and must be measured.
- **One camera** (spec section 18, `cameras:=mono`): floor-plane odometry drifts 3.3 % in the sim
  (stereo: 0.2 to 1 %), there is no RTAB-Map, and the ultrasonic bumper becomes required. Untested
  on a real floor.
- **Not exercised here:** usb_cam, nmea_navsat_driver, the perception models (SegFormer, YOLO11,
  PaddleOCR, Whisper, Piper) and the real Claude calls. Claude now runs through the owner's
  Claude subscription (Claude Agent SDK, `CLAUDE_CODE_OAUTH_TOKEN` from `claude setup-token`) with
  the API key as a fallback. Both are unit-tested with fakes; the real calls need the laptop and
  the token.
- **CPU cost.** Nav2's global costmap covers the whole building at 5 cm (3,594 × 3,613 cells for AQ).
  In this 4-core container the planner took about 1 s per plan and the controller ran at about 5 Hz.
  If the laptop struggles, crop grids to each floor's corridors, or plan on a 10 cm copy.
- **Firmware** is written for a PWM + direction driver. A TB6612FNG or L298N needs `drive()`
  adapted (noted in the file).
