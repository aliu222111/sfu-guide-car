"""End-to-end trip check against sim.launch.py: send a destination, play the
student (answer every door and elevator prompt), report how the trip ended.

    ros2 launch guide_bringup sim.launch.py start_floor:=AQ-03 &
    ros2 run guide_bringup sim_trip "AQ 6106" --timeout 600

Exit status 0 when the car arrives, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String

ANSWERED = ("elevator_call", "elevator_ride", "door", "still_with_me")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("destination")
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--answer-after", type=float, default=3.0, help="seconds before the student confirms")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("sim_trip")
    state = {"doc": None}
    node.create_subscription(String, "guide/state", lambda m: state.__setitem__("doc", json.loads(m.data)), 10)
    node.create_subscription(String, "guide/speech", lambda m: print(f"  car: {m.data}", flush=True), 10)
    dest = node.create_publisher(String, "guide/destination", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    confirm = node.create_publisher(Empty, "guide/confirm", 10)

    # Wait for the mission node before sending.
    t0 = time.time()
    while dest.get_subscription_count() == 0 and time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.2)
    dest.publish(String(data=args.destination))
    print(f"destination: {args.destination}", flush=True)

    started = time.time()
    asked, asked_at = None, 0.0
    last = None
    result = 1
    while time.time() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        doc = state["doc"]
        if not doc:
            continue
        key = (doc["state"], doc.get("detail", ""))
        if key != last:
            last = key
            print(f"{time.time() - started:6.0f} s  {key[0]} {key[1]}", flush=True)
        if doc["state"] in ("waiting_for_help", "waiting_for_student") and key[1] in ANSWERED:
            if asked != key:
                asked, asked_at = key, time.time()
            elif time.time() - asked_at > args.answer_after:
                print("  student: confirm", flush=True)
                confirm.publish(Empty())
                asked = None
        if doc["state"] == "arrived":
            result = 0
            break
        if doc["state"] == "aborted" or key == ("waiting_for_help", "blocked"):
            break
    print("result:", "arrived" if result == 0 else f"{last}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
