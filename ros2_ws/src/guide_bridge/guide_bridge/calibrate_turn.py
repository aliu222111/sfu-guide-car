"""Measure a tank chassis's turn slip with the phone gyro: spins in place, prints turn_gain.

    ros2 run guide_bridge serial_bridge &      # with its current turn_gain
    ros2 run guide_bridge phone_imu &
    ros2 run guide_bridge calibrate_turn --current-gain 1.0 --yes

The car spins in place, both ways, at each rate in turn (about 2 minutes), on
the floor it will drive on. Give it 1 m of clear space all round. It
publishes /cmd_vel itself, so the velocity gate and Nav2 must not be running.
Ctrl-C stops it; the motor watchdog stops the car within 0.5 s if this dies.
Put the printed turn_gain and min_pivot_rate in guide.yaml (serial_bridge),
then run it again with --current-gain set to the new gain to check: the gain
should come out near the one in use. If every slow spin stalls, raise MIN_PWM_PIVOT
in the firmware.
"""

from __future__ import annotations

import argparse
import sys
import time

from .skid import SpinResult, mean_rate, new_turn_gain, stall_rate

RATES = (0.15, 0.25, 0.35, 0.45)  # rad/s; the gate caps turning at 0.52
SLOWEST_USED = 0.26  # rad/s: the velocity gate's recovery rotation
SPIN_S = 6.0
PAUSE_S = 1.5


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--current-gain", type=float, default=1.0, help="the serial bridge's turn_gain during this run")
    ap.add_argument("--yes", action="store_true", help="confirm the car has room to spin")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:])
    if not args.yes:
        print("The car will spin in place for about 2 minutes. Clear 1 m round it, then rerun with --yes.")
        return 2

    rclpy.init()
    node = rclpy.create_node("calibrate_turn")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    samples: list[tuple[float, float]] = []
    node.create_subscription(Imu, "phone/imu", lambda m: samples.append((time.monotonic(), m.angular_velocity.z)), 50)

    def hold(w: float, seconds: float):
        end = time.monotonic() + seconds
        cmd = Twist()
        cmd.angular.z = w
        while time.monotonic() < end and rclpy.ok():
            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        time.sleep(1.0)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        others = [i for i in node.get_publishers_info_by_topic("/cmd_vel") if i.node_name != "calibrate_turn"]
        if others:
            print("refusing: something else publishes /cmd_vel:", ", ".join(i.node_name for i in others))
            return 1
        if not samples:
            print("no /phone/imu: start phone_imu and the Sensor Logger app first")
            return 1
        results = []
        for rate in RATES:
            for w in (rate, -rate):
                start = time.monotonic()
                hold(w, SPIN_S)
                results.append(SpinResult(w, mean_rate(samples, start)))
                hold(0.0, PAUSE_S)
                r = results[-1]
                print(f"asked {r.commanded:+.2f} rad/s, turned {r.measured:+.2f} rad/s")
    except KeyboardInterrupt:
        print("stopped")
        return 1
    finally:
        stop = Twist()
        for _ in range(5):
            pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()

    stall = stall_rate(results)
    try:
        gain = new_turn_gain(results, args.current_gain)
    except ValueError as e:
        print(e)
        return 1
    if stall == float("inf"):
        print("every rate stalled at least one way: raise MIN_PWM_PIVOT in the firmware and run again")
        return 1
    # The stall was measured through the current gain; express it at the new one.
    min_pivot = stall * args.current_gain / gain
    print("guide.yaml, serial_bridge:")
    print(f"    turn_gain: {gain:.2f}")
    print(f"    min_pivot_rate: {min_pivot:.2f}" + ("   (no spin stalled)" if stall == 0.0 else ""))
    if min_pivot > SLOWEST_USED:
        print(f"note: pivots slower than {min_pivot:.2f} rad/s stall, so the lost-recovery rotation "
              f"({SLOWEST_USED} rad/s) will run faster; raise MIN_PWM_PIVOT in the firmware to avoid that")
    return 0


if __name__ == "__main__":
    sys.exit(main())
