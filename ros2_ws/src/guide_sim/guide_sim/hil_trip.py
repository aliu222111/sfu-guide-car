"""The student in the hardware-in-the-loop simulation, and the judge.

    ros2 run guide_sim hil_trip "AQ 3046" --timeout 900

Talks to the car the way the phone app does: a rosbridge websocket on port
9090, a heartbeat every second, the destination, and "confirm" when the car
asks for a door or an elevator. Meanwhile it reads the simulator's ground
truth to score the trip: arrival, time, collisions, how far the car's own
position estimate (map -> base_link) strayed from the truth, and where the
car stopped relative to the destination door.

Exit status 0: arrived, no collisions, and the car stopped within 2.5 m of the door.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import statistics
import sys
import os
import threading
from pathlib import Path

ANSWERED = ("elevator_call", "elevator_ride", "door", "still_with_me")


async def student(url: str, destination: str, state: dict, answer_after: float, log):
    import aiohttp

    async with aiohttp.ClientSession() as http:
        for _ in range(120):
            try:
                ws = await http.ws_connect(url)
                break
            except aiohttp.ClientError:
                await asyncio.sleep(1.0)
        else:
            raise RuntimeError(f"no rosbridge at {url}")
        send = lambda d: ws.send_str(json.dumps(d))  # noqa: E731
        for topic, typ in (("/guide/destination", "std_msgs/String"), ("/guide/confirm", "std_msgs/Empty"),
                           ("/guide/heartbeat", "std_msgs/Empty")):
            await send({"op": "advertise", "topic": topic, "type": typ})
        for topic in ("/guide/state", "/guide/speech"):
            await send({"op": "subscribe", "topic": topic, "type": "std_msgs/String"})

        async def heartbeat():
            while not state.get("done"):
                await send({"op": "publish", "topic": "/guide/heartbeat", "msg": {}})
                await asyncio.sleep(1.0)

        hb = asyncio.create_task(heartbeat())
        sent_dest = False
        asked, asked_at = None, 0.0
        while not state.get("done"):
            try:
                msg = await asyncio.wait_for(ws.receive(), timeout=1.0)
            except asyncio.TimeoutError:
                msg = None
            if msg is not None and msg.type == aiohttp.WSMsgType.TEXT:
                d = json.loads(msg.data)
                if d.get("op") == "publish":
                    data = d["msg"]["data"]
                    if d["topic"] == "/guide/speech":
                        log(f"  car: {data}")
                    else:
                        state["doc"] = json.loads(data)
            elif msg is not None and msg.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                raise RuntimeError("rosbridge closed the connection")
            doc = state.get("doc")
            # Ask once the car reports a state (mission up and the phone seen).
            if doc and not sent_dest and state.get("sim_t", 0) > 5.0:
                await send({"op": "publish", "topic": "/guide/destination", "msg": {"data": destination}})
                log(f"student: take me to {destination}")
                sent_dest = True
            if doc and doc["state"] in ("waiting_for_help", "waiting_for_student") and doc.get("detail") in ANSWERED:
                key = (doc["state"], doc.get("detail"))
                if asked != key:
                    asked, asked_at = key, state.get("sim_t", 0.0)
                elif state.get("sim_t", 0.0) - asked_at > answer_after:
                    log("student: confirm")
                    await send({"op": "publish", "topic": "/guide/confirm", "msg": {}})
                    asked = None
        hb.cancel()
        await ws.close()


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("destination")
    ap.add_argument("--timeout", type=float, default=900.0, help="simulated seconds")
    ap.add_argument("--answer-after", type=float, default=3.0, help="simulated seconds before the student confirms")
    ap.add_argument("--rosbridge", default="ws://127.0.0.1:9090")
    ap.add_argument("--report", help="write the result as JSON here")
    ap.add_argument("--trace", help="write t, truth and estimated pose, speed and state as CSV here")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("hil_trip", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    tf = Buffer()
    TransformListener(tf, node)
    state: dict = {"sim_t": 0.0}
    errors: list[float] = []
    status: dict = {}
    truth: dict = {}
    log_lines: list[str] = []

    def log(line: str):
        stamp = f"{state['sim_t']:7.1f} s  "
        print(stamp + line, flush=True)
        log_lines.append(stamp + line)

    trace: list[str] = []

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_truth(m: PoseStamped):
        truth["x"], truth["y"] = m.pose.position.x, m.pose.position.y
        state["sim_t"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        tyaw = yaw_of(m.pose.orientation)
        try:
            tr = tf.lookup_transform("map", "base_link", Time()).transform
        except Exception:
            tr = None
        doc_now = state.get("doc") or {}
        if doc_now.get("state") in ("driving", "waiting_for_help", "waiting_for_student", "arrived"):
            state["placed"] = True  # scored from the first fix on: before it the car does not know
        if tr is not None and state.get("placed"):
            e = tr.translation
            errors.append(math.hypot(e.x - truth["x"], e.y - truth["y"]))
        doc = state.get("doc") or {}
        trace.append(",".join(str(v) for v in (
            round(state["sim_t"], 2), round(truth["x"], 3), round(truth["y"], 3), round(tyaw, 3),
            "" if tr is None else round(tr.translation.x, 3), "" if tr is None else round(tr.translation.y, 3),
            "" if tr is None else round(yaw_of(tr.rotation), 3), status.get("v", ""), status.get("w", ""),
            doc.get("state", ""), doc.get("detail", ""),
            *(round(v, 3) for name in ("vo", "local", "global") for v in odo.get(name, (math.nan,) * 3)),
            round(vp.get("yaw", math.nan), 3) if state["sim_t"] - vp.get("t", -9) < 0.5 else "",
            round(vp.get("axis", math.nan), 3),
            cmd.get("gate", ""), *(round(v, 3) for k in ("nav", "out") for v in cmd.get(k, (math.nan, math.nan))))))

    from nav_msgs.msg import Odometry

    odo: dict = {}

    def keep(name):
        def f(m):
            odo[name] = (m.twist.twist.linear.x, m.twist.twist.angular.z, m.pose.covariance[0])
        return f

    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64

    from geometry_msgs.msg import Twist

    # What each layer asked for: Nav2's controller, then the velocity gate's output and reason.
    cmd: dict = {}
    node.create_subscription(Twist, "/cmd_vel_nav", lambda m: cmd.update(nav=(m.linear.x, m.angular.z)), 10)
    node.create_subscription(Twist, "/cmd_vel", lambda m: cmd.update(out=(m.linear.x, m.angular.z)), 10)
    node.create_subscription(String, "/guide/gate_reason", lambda m: cmd.update(gate=m.data), 10)

    vp: dict = {}
    node.create_subscription(Imu, "/guide/vp_heading", lambda m: vp.update(yaw=yaw_of(m.orientation), t=state["sim_t"]), 10)
    node.create_subscription(Float64, "/guide/corridor_axis", lambda m: vp.update(axis=m.data), 10)
    node.create_subscription(Odometry, "/vo", keep("vo"), 10)
    node.create_subscription(Odometry, "/odometry/local", keep("local"), 10)
    node.create_subscription(Odometry, "/odometry/global", keep("global"), 10)
    node.create_subscription(PoseStamped, "/sim/truth", on_truth, 10)
    node.create_subscription(String, "/sim/status", lambda m: status.update(json.loads(m.data)), 10)
    stop = threading.Event()
    threading.Thread(target=lambda: [rclpy.spin_once(node, timeout_sec=0.1) for _ in iter(lambda: stop.is_set(), True)],
                     daemon=True).start()

    loop = asyncio.new_event_loop()
    task = loop.create_task(student(args.rosbridge, args.destination, state, args.answer_after, log))
    last = None
    result = "timeout"
    try:
        while state["sim_t"] < args.timeout:
            loop.run_until_complete(asyncio.sleep(0.2))
            if task.done():
                task.result()  # raises the student's error
                break
            doc = state.get("doc")
            if not doc:
                continue
            key = (doc["state"], doc.get("detail", ""))
            if key != last:
                last = key
                log(f"{key[0]} {key[1]}".strip())
            if doc["state"] == "arrived":
                result = "arrived"
                break
            if doc["state"] == "aborted" or key == ("waiting_for_help", "blocked"):
                result = f"{key[0]} {key[1]}".strip()
                break
    finally:
        state["done"] = True
        try:
            loop.run_until_complete(asyncio.wait_for(task, 5.0))
        except Exception:
            pass
        stop.set()

    door = None
    try:
        root = Path(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()))
        g = json.loads((root / "maps" / "route_graph.json").read_text())
        nodes = {n["id"]: n for n in g["nodes"]}
        from sfu_map.route import RouteGraph

        door = nodes[RouteGraph.load(root / "maps" / "route_graph.json").resolve(args.destination)]
    except Exception:
        pass
    stop_dist = math.hypot(truth["x"] - door["x"], truth["y"] - door["y"]) if door and truth else None
    report = {
        "destination": args.destination, "result": result, "sim_time_s": round(state["sim_t"], 1),
        "collisions": status.get("collisions"), "collision_at": status.get("collision_at"),
        "stopped_from_door_m": None if stop_dist is None else round(stop_dist, 2),
        "localization_error_m": None if not errors else {  # from the first fix on
            "median": round(statistics.median(errors), 2), "p95": round(sorted(errors)[int(0.95 * (len(errors) - 1))], 2),
            "max": round(max(errors), 2)},
        "phone_posts": status.get("phone_posts"), "final_floor": status.get("floor"),
    }
    print(json.dumps(report, indent=2))
    if args.trace:
        with open(args.trace, "w") as f:
            f.write("t,x,y,yaw,est_x,est_y,est_yaw,v,w,state,detail,vo_v,vo_w,vo_cov,local_v,local_w,local_cov,"
                    "global_v,global_w,global_cov,vp_yaw,corridor_axis,gate,nav_v,nav_w,out_v,out_w\n" + "\n".join(trace) + "\n")
    if args.report:
        with open(args.report, "w") as f:
            json.dump({**report, "log": log_lines}, f, indent=2)
    node.destroy_node()
    rclpy.try_shutdown()
    ok = result == "arrived" and status.get("collisions", 1) == 0 and stop_dist is not None and stop_dist <= 2.5
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
