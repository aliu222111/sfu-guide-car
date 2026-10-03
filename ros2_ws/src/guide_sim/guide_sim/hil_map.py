"""Phase 1 in simulation: build a floor's RTAB-Map database and align it to the GIS frame.

    ros2 launch guide_sim hil.launch.py mode:=map rig_destination:="AQ 3046" sim_root:=/tmp/guide_sim_root
    ros2 run guide_sim hil_map --floor AQ-03 --root /tmp/guide_sim_root

The simulator carries the stereo rig along the route there and back while
RTAB-Map runs in mapping mode, exactly as rtabmap.launch.py does on the
handheld rig. Each time the rig passes a route node this records where
RTAB-Map thinks it is (map -> base_link), relative to RTAB-Map's nearest graph
node. At the end it re-reads those graph nodes after every loop closure has
been applied, as a person reading door positions off the finished map would,
and runs the real ``align_floor`` with those pairs, which writes
``rtabmap_db/alignments.yaml`` and reports the fit's RMS: how far the map is
from the building's shape.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def main(argv=None):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rtabmap_msgs.msg import MapGraph
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--floor", required=True)
    ap.add_argument("--root", required=True, help="the simulation root (sim_root): maps/ and rtabmap_db/")
    ap.add_argument("--timeout", type=float, default=3600.0, help="simulated seconds")
    ap.add_argument("--every", type=int, default=2, help="use every n-th route node as an alignment pair")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("hil_map", parameter_overrides=[Parameter("use_sim_time", value=True)])
    tf = Buffer()
    TransformListener(tf, node)
    # route node -> (graph node id, the rig's offset from that graph node in its own frame)
    pairs: dict[str, tuple[int, float, float]] = {}
    seen = []
    status: dict = {}
    graph: dict[int, tuple[float, float, float]] = {}

    def on_graph(m: MapGraph):
        graph.clear()
        for i, p in zip(m.poses_id, m.poses):
            q = p.orientation
            graph[i] = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_rig(m: String):
        seen.append(m.data)
        if len(seen) % args.every:
            return
        try:
            t = tf.lookup_transform("map", "base_link", Time()).transform.translation
        except Exception:
            return
        if m.data in pairs or not graph:  # first pass only: the way out
            return
        gid, (gx, gy, gyaw) = min(graph.items(), key=lambda kv: math.hypot(kv[1][0] - t.x, kv[1][1] - t.y))
        c, s = math.cos(gyaw), math.sin(gyaw)
        dx, dy = t.x - gx, t.y - gy
        pairs[m.data] = (gid, c * dx + s * dy, -s * dx + c * dy)

    node.create_subscription(String, "/sim/rig_node", on_rig, 10)
    node.create_subscription(MapGraph, "/rtabmap/mapGraph", on_graph, 5)
    node.create_subscription(String, "/sim/status", lambda m: status.update(json.loads(m.data)), 10)
    while rclpy.ok() and not status.get("rig_done") and status.get("t", 0.0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    final = dict(graph)  # the last graph: every loop closure applied
    node.destroy_node()
    rclpy.try_shutdown()
    points = {}
    for name, (gid, ox, oy) in pairs.items():
        if gid in final:
            gx, gy, gyaw = final[gid]
            points[name] = (gx + math.cos(gyaw) * ox - math.sin(gyaw) * oy, gy + math.sin(gyaw) * ox + math.cos(gyaw) * oy)
    print(f"rig passed {len(seen)} route nodes; {len(points)} alignment pairs")
    if len(points) < 3:
        print("too few pairs to align")
        return 1
    from guide_mission.alignment import main as align_floor

    root = Path(args.root)
    out = root / "rtabmap_db" / "alignments.yaml"
    cli = ["--floor", args.floor, "--graph", str(root / "maps" / "route_graph.json"), "--out", str(out), "--no-scale"]
    cli += [f"{n}={x:.3f},{y:.3f}" for n, (x, y) in points.items()]
    return align_floor(cli)


if __name__ == "__main__":
    sys.exit(main())
