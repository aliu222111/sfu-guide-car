"""Phase 1 in simulation: build a floor's RTAB-Map database and align it to the GIS frame.

    ros2 launch guide_sim hil.launch.py mode:=map rig_destination:="AQ 3046" sim_root:=/tmp/guide_sim_root
    ros2 run guide_sim hil_map --floor AQ-03 --root /tmp/guide_sim_root

The simulator carries the stereo rig along the route there and back while
RTAB-Map runs in mapping mode, exactly as rtabmap.launch.py does on the
handheld rig. Each time the rig passes a route node this records where
RTAB-Map thinks it is (map -> base_link); at the end it runs the real
``align_floor`` with those pairs, which writes ``rtabmap_db/alignments.yaml``
and reports the fit's RMS: how far the map is from the building's shape.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main(argv=None):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.time import Time
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
    pairs: dict[str, tuple[float, float]] = {}
    seen = []
    status: dict = {}

    def on_rig(m: String):
        seen.append(m.data)
        if len(seen) % args.every:
            return
        try:
            t = tf.lookup_transform("map", "base_link", Time()).transform.translation
        except Exception:
            return
        pairs.setdefault(m.data, (t.x, t.y))  # first pass only: the way out

    node.create_subscription(String, "/sim/rig_node", on_rig, 10)
    node.create_subscription(String, "/sim/status", lambda m: status.update(json.loads(m.data)), 10)
    while rclpy.ok() and not status.get("rig_done") and status.get("t", 0.0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.try_shutdown()
    print(f"rig passed {len(seen)} route nodes; {len(pairs)} alignment pairs")
    if len(pairs) < 3:
        print("too few pairs to align")
        return 1
    from guide_mission.alignment import main as align_floor

    root = Path(args.root)
    out = root / "rtabmap_db" / "alignments.yaml"
    cli = ["--floor", args.floor, "--graph", str(root / "maps" / "route_graph.json"), "--out", str(out), "--no-scale"]
    cli += [f"{n}={x:.3f},{y:.3f}" for n, (x, y) in pairs.items()]
    return align_floor(cli)


if __name__ == "__main__":
    sys.exit(main())
