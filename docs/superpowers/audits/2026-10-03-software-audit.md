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

## Open items, not fixable without the hardware or site visits

- **Verify on site:** every connector in `sfu_map/connectors.yaml` (14 elevators, 1 entrance) and
  all 92 inferred doorways are `verified: false`.
- **No outdoor paths.** There is no car route between buildings until `outdoor_links` are drawn or
  OSM footpaths are imported.
- **Disconnected corridor pieces.** Several floors still have pieces the GIS separates by more than
  0.45 m; `maps/build_report.txt` lists them. Each needs a `links` entry after a look on site.
- **RTAB-Map** is not packaged for RoboStack, so `rtabmap.launch.py` is untested here. Its parameter
  names follow the rtabmap_ros documentation.
- **Not exercised here:** usb_cam, nmea_navsat_driver, the perception models (SegFormer, YOLO11,
  PaddleOCR, Whisper, Piper) and the Claude API calls. Their logic is unit-tested with fakes, and the
  real calls need the laptop and an `ANTHROPIC_API_KEY`.
- **CPU cost.** Nav2's global costmap covers the whole building at 5 cm (3,594 × 3,613 cells for AQ).
  In this 4-core container the planner took about 1 s per plan and the controller ran at about 5 Hz.
  If the laptop struggles, crop grids to each floor's corridors, or plan on a 10 cm copy.
- **Firmware** is written for a PWM + direction driver. A TB6612FNG or L298N needs `drive()`
  adapted (noted in the file).
