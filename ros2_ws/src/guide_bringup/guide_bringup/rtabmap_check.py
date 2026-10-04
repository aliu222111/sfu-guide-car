"""Score RTAB-Map against the stereo simulator's ground truth.

    ros2 run guide_bringup rtabmap_check --mode mapping --duration 200
    ros2 run guide_bringup rtabmap_check --mode localization --start-s 20 --duration 120
    ros2 run guide_bringup rtabmap_check --mode odometry --duration 200     # /vo only (one camera)

mapping:      visual odometry (/vo) drift against truth, frames where odometry
              was lost, map nodes and loop closures RTAB-Map reports
              (/rtabmap/info). Past one lap there must be a loop closure.
odometry:     the mapping checks on /vo alone, for mono_odometry without RTAB-Map.
localization: error of every /rtabmap/localization_pose against truth. The
              map frame is the pose where mapping started (start_s = 0).

Exit status 0 when the run meets the thresholds below.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.time import Time

from .stereo_sim import centreline, perimeter

MAX_DRIFT_FRACTION = 0.05  # odometry error per metre driven
MAX_LOCALIZATION_ERROR = 1.0  # m
MIN_NODES_PER_METRE = 0.3  # RTAB-Map adds a node about every 0.4 m at 1 Hz and 0.4 m/s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def relative(origin, pose):
    """``pose`` expressed in the frame of ``origin`` (both x, y, yaw)."""
    ox, oy, oyaw = origin
    dx, dy = pose[0] - ox, pose[1] - oy
    c, s = math.cos(-oyaw), math.sin(-oyaw)
    return c * dx - s * dy, s * dx + c * dy, pose[2] - oyaw


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["mapping", "odometry", "localization"], required=True)
    ap.add_argument("--duration", type=float, default=180.0)
    ap.add_argument("--start-s", type=float, default=0.0, help="the simulator's start_s")
    ap.add_argument("--csv", help="write every scored pose (t, expected x y, got x y, error) here")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node("rtabmap_check")
    tf = Buffer()
    TransformListener(tf, node)
    rows = []
    stats = {"nodes": set(), "vo": 0, "lost": 0, "drift": [], "loops": set(), "loc": [], "dist": 0.0}
    last_truth = [None]

    def truth_at(stamp):
        try:
            t = tf.lookup_transform("sim_truth", "base_link_truth", Time.from_msg(stamp)).transform
        except Exception:
            return None
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    start = centreline(args.start_s)
    map_origin = centreline(0.0)

    def on_vo(m):
        stats["vo"] += 1
        if m.pose.covariance[0] >= 9999:
            stats["lost"] += 1
            return
        tr = truth_at(m.header.stamp)
        if tr is None:
            return
        if last_truth[0] is not None:
            stats["dist"] += math.hypot(tr[0] - last_truth[0][0], tr[1] - last_truth[0][1])
        last_truth[0] = tr
        expect = relative(start, tr)
        p = m.pose.pose.position
        stats["drift"].append((stats["dist"], math.hypot(p.x - expect[0], p.y - expect[1])))
        rows.append(("vo", Time.from_msg(m.header.stamp).nanoseconds * 1e-9, *expect[:2], p.x, p.y, stats["drift"][-1][1]))

    def on_loc(m):
        tr = truth_at(m.header.stamp)
        if tr is None:
            return
        expect = relative(map_origin, tr)
        p = m.pose.pose.position
        stats["loc"].append(math.hypot(p.x - expect[0], p.y - expect[1]))
        rows.append(("loc", Time.from_msg(m.header.stamp).nanoseconds * 1e-9, *expect[:2], p.x, p.y, stats["loc"][-1]))

    node.create_subscription(Odometry, "/vo", on_vo, 50)
    node.create_subscription(PoseWithCovarianceStamped, "/rtabmap/localization_pose", on_loc, 10)
    try:
        from rtabmap_msgs.msg import Info

        def on_info(m):
            if m.ref_id > 0:
                stats["nodes"].add(m.ref_id)
            if m.loop_closure_id > 0:
                stats["loops"].add((m.ref_id, m.loop_closure_id))

        node.create_subscription(Info, "/rtabmap/info", on_info, 10)
    except ImportError:
        print("rtabmap_msgs not importable: loop closures not counted")

    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    if args.csv:
        with open(args.csv, "w") as f:
            f.write("kind,t,expect_x,expect_y,x,y,error\n")
            f.writelines(",".join(str(v) for v in r) + "\n" for r in rows)
    ok = True
    if args.mode in ("mapping", "odometry"):
        dist = stats["dist"]
        final = stats["drift"][-1][1] if stats["drift"] else math.inf
        worst = max((e for _, e in stats["drift"]), default=math.inf)
        print(f"odometry: {stats['vo']} messages, {stats['lost']} lost, {dist:.1f} m driven")
        print(f"odometry error: final {final:.2f} m, worst {worst:.2f} m ({100 * final / max(dist, 1e-6):.1f}% of distance)")
        ok = dist > 10 and final <= MAX_DRIFT_FRACTION * dist + 0.3 and stats["lost"] <= 0.05 * max(stats["vo"], 1)
        if args.mode == "mapping":
            print(f"map nodes: {len(stats['nodes'])}, loop closures: {len(stats['loops'])}")
            ok = (ok and len(stats["nodes"]) >= MIN_NODES_PER_METRE * dist
                  and (dist < perimeter() * 1.1 or stats["loops"]))  # past a lap it must close the loop
    else:
        errs = stats["loc"]
        if errs:
            errs_sorted = sorted(errs)
            print(f"localization poses: {len(errs)}, median error {errs_sorted[len(errs) // 2]:.2f} m, worst {errs_sorted[-1]:.2f} m")
        else:
            print("localization poses: 0")
        ok = bool(errs) and sorted(errs)[len(errs) // 2] <= MAX_LOCALIZATION_ERROR
    print("RESULT:", "PASS" if ok else "FAIL")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
