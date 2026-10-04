# guide_api: the Scout rider app's backend

The Scout app (Uber-style: say where you are and where you are going, the car comes to you and leads
you there) talks to the car through this package. It adds nothing below the mission node: it speaks
the same `/guide/*` topics the car's own phone app uses.

```
phone (Scout app)
   |  HTTP + server-sent events, port 8090
guide_api  (this package, on the car's laptop)
   |  /guide/destination  /guide/confirm  /guide/cancel  /guide/pause  /guide/heartbeat
   |  /guide/state  /guide/route  /guide/speech
mission -> Nav2 -> velocity gate -> serial bridge --USB--> ESP32 -> the RC controller -> the car
```

Logic is in plain modules with tests that need no ROS; `api_node.py` is the thin ROS wrapper.

| file | what |
|---|---|
| `service.py` | trips: summon stage, guide stage, one car, one trip at a time |
| `phases.py` | mission state -> the app's phase and the prompt ("Hold the door") |
| `progress.py` | progress and time left from `/guide/route` and the car's pose |
| `frames.py`, `fit_frame.py` | the app's floor-plan coordinates <-> the map frame |
| `http_api.py` | the HTTP API (standard library only) |
| `api_node.py` | ROS node `scout_api`: publishers, subscribers, the server |
| `fake_car.py`, `dev.py` | a stand-in mission, to work on the app with no car and no ROS |
| `planner.py`, `straight_car.py`, `car.py` | the car with no steering: straight legs on the app's floor plan, driven through the ESP32 + L298N board, no ROS |
| `web/scout-api.js` | browser client with the method names the app already uses |
| `web/index.html` | a test page that drives the API by hand |

## Try it without the car (any OS, Python only)

```bash
cd ros2_ws/src/guide_api
python -m guide_api.dev --speedup 6        # add --door for a door stop
# open http://localhost:8090  : request "ASB 9896", or "ASB 10900" for an elevator ride
```

## On the car

`guide_car.launch.py` and `sim.launch.py` start it as `scout_api` beside `ui_server`; settings are
under `scout_api` in `guide.yaml` (port, `speed` for the time left, optional `token`). Phones reach it
at `http://<laptop>:8090` on the same WiFi.

## The API

| call | what |
|---|---|
| `GET /api/health` | is the mission answering, how many rooms, which buildings have an app frame |
| `GET /api/car` | `status` (`available`, `busy`, `offline`), floor, pose, position on the app's plan |
| `GET /api/destinations?q=98` | rooms from `maps/destinations.json` |
| `POST /api/trips` `{"destination": "ASB 9896", "pickup": "ASB 9700"}` | start a trip; without `pickup` the rider is already at the car |
| `POST /api/trips/<id>/start` | the rider met the car: begin guiding |
| `POST /api/trips/<id>/confirm` | the rider did what the car asked |
| `POST /api/trips/<id>/pause` `{"paused": true}` | pause or resume |
| `POST /api/trips/<id>/cancel` | end the trip |
| `GET /api/trips/<id>/events` | server-sent events, one JSON document per change |
| `GET /api/trips/current` | the trip under way, for a reloaded page |
| `POST /api/car/position` `{"room": "ASB 970.7", "headingDeg": 90}` | only for the straight-driving car: where it is standing |

Errors are `{"error": code, "message": text}`: `unknown_room` (404), `busy` (409), `offline` (503),
`not_ready` (409), `unknown_trip` (404).

One event:

```json
{"tripId": "trip-3", "stage": "guide", "phase": "help", "progress": 0.42, "etaSec": 95.0, "leftM": 17.5,
 "paused": false, "online": true, "say": "Could you hold the door for me?",
 "prompt": {"kind": "door", "title": "Hold the door", "line": "...", "button": "I'm holding the door"},
 "floor": "ASB-09", "pose": {"x": 170.1, "y": -150.2, "yaw": 1.2},
 "app": {"lv": "9", "x": 640.5, "y": 512.0, "heading": -81.0}}
```

`phase`: `requested` (sent, the mission has not taken it up yet), `enroute` (driving to the rider),
`arrived` (at the rider: call `start`), `guiding`, `help` (press `prompt.button`, then `confirm`),
`waiting`, `lost`, and the endings `done`, `aborted`, `cancelled`, `failed`. `route` and `legs` (the
route per floor, in the app's plan coordinates) come with the first event and whenever the route changes.

How a trip maps onto the mission:

- **Summon.** `/guide/destination` is set to the pickup room. Mission `arrived` becomes the app's `arrived`.
- **Guide.** On `start`, `/guide/destination` is set to the destination. Mission `arrived` becomes `done`.
- Mission `waiting_for_help` (`door`, `elevator_call`, `elevator_ride`, `blocked`) and
  `waiting_for_student` (`still_with_me`) become a `prompt`; `confirm` publishes `/guide/confirm`.
- While an app is listening to a trip's events, `/guide/heartbeat` goes out once a second.
- A trip the mission does not take up within 8 s ends as `failed`, with the car's last spoken line as the reason.

## The straight-driving car (ESP32 + L298N, no steering, no ROS)

Two drive motors on an L298N, no steering, no wheel sensors. `straight_car.py` stands where the mission
node would: it plans a route on the app's floor plan, cuts it into straight legs, and for each leg asks
for the car to be turned by hand, waits for a confirm (the app's button, or the button on the car), then
drives the leg's length. Distance is time at a calibrated speed.

Firmware: `guide_bridge/firmware/guide_l298n/guide_l298n.ino` (wiring and rules at the top of the file).
It takes the same `C <steering> <throttle>` line as `guide_motor.ino` and stops if none arrives for 300 ms.
Full throttle is PWM duty 120 of 255 (`MAX_DUTY`): above that the motors and the driver overheat.

```bash
pip install pyserial
cd ros2_ws/src/guide_api
python -m guide_api.car --port COM5 --jog 2          # wheels off the floor: both must turn forwards
python -m guide_api.car --port COM5 --jog 4          # on the floor: measure the distance, speed = metres / 4
python -m guide_api.car --port COM5 --speed 0.62 --start "ASB 970.7"
# http://<laptop>:8090 : set the car's room, request a trip, press the button at each turn
```

What it cannot do: steer, cross a floor, see obstacles (unless the HC-SR04 is fitted and
`HAVE_ULTRASONIC` is defined), or know where it really is. Its position is start + time driven, so it
drifts a little every leg and every hand turn; set its room again (`/api/car/position`) when it is off.
Someone has to walk with it.

## The app's floor plan and the map frame

The app draws ASB from its SFU keyplans, in plan units. `guide_api/config/app_frames.json` holds the
affine transform to the map frame, fitted from the 411 room numbers the plan and the GIS file share
(median error 7 cm). To redo it, or add a building:

```bash
python -m sfu_map.fetch ASB                  # data/sfu_gis/sfu_rooms_ASB.geojson
python -m guide_api.fit_frame ASB            # needs guide_api/config/asb_keyplan_rooms.json
```

A map floor id becomes the app's level by dropping the building and leading zeros: `ASB-09` is level `9`.

## Using it from the Scout app

```html
<script src="http://<laptop>:8090/scout-api.js"></script>
<script>
  const api = ScoutApi("http://<laptop>:8090");
  // same names as the app's this.api: getCar, requestTrip, onTripUpdate, startGuiding, continueTrip, cancelTrip
</script>
```

The app keeps its own floor plan, search and "find the nearest"; the car answers the rest. Draw the
car from `update.app`, the route from `update.legs`, and show `update.prompt` when it is not null.

## Not done yet

- **ASB is not in the built maps.** `maps/` has AQ, WMC and MBC. `data/sfu_gis/sfu_rooms_ASB.geojson` is
  fetched; until `sfu-map build` includes it (and the floor is mapped with the rig), the real mission
  answers "I can't find ASB ..." and the trip ends as `failed`. The fake car reads the GIS file directly.
- **The ROS node has not been run.** It was written on a Windows machine without ROS; the logic, the HTTP
  API and the fake car are tested, `api_node.py` and the two launch lines are not.
- **Summoning past a door or an elevator.** With no rider beside it the car stops at `help` and waits.
  Pick-ups work on the car's own floor with no closed doors in between.
- **No battery level.** Nothing on the car measures it; `battery` is null.
- **The app itself is not served from here yet.** A page on `https://` cannot call `http://<laptop>`, so
  the Scout app has to be exported and served from this server (`web/`) to reach the car.
- **Anyone on the WiFi can command the car**, as with rosbridge. Set `token` in `guide.yaml` to require one.

## Tests

```bash
cd ros2_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/guide_api python3 -m pytest -q src/guide_api/test
```
