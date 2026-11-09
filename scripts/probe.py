"""Debug probe: prints ground truth vs. odometry-in-world and the nearest lidar return.

    python scripts/probe.py --duration 40
"""
import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self, sx, sy, syaw):
        super().__init__("probe", parameter_overrides=[])
        self.s = (sx, sy, syaw)
        self.gt = self.od = self.scan = None
        self.create_subscription(Odometry, "/ground_truth/odom", lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "od", m), 10)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)

    def report(self):
        if not (self.gt and self.od):
            print("waiting...")
            return
        g = self.gt.pose.pose
        o = self.od.pose.pose
        sx, sy, sth = self.s
        ox = sx + math.cos(sth) * o.position.x - math.sin(sth) * o.position.y
        oy = sy + math.sin(sth) * o.position.x + math.cos(sth) * o.position.y
        smin = min((r for r in self.scan.ranges if math.isfinite(r)), default=float("nan")) if self.scan else float("nan")
        print(f"t={self.gt.header.stamp.sec:4d}  gt=({g.position.x:6.3f},{g.position.y:6.3f},{math.degrees(yaw_of(g.orientation)):6.1f})"
              f"  odom=({ox:6.3f},{oy:6.3f},{math.degrees(sth + yaw_of(o.orientation)):6.1f})"
              f"  err={math.hypot(ox - g.position.x, oy - g.position.y):.3f}  scan_min={smin:.3f}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=40)
    ap.add_argument("--period", type=float, default=4)
    ap.add_argument("--start", type=float, nargs=3, default=[-3.0, -3.0, 0.0])
    a = ap.parse_args()
    rclpy.init()
    n = Probe(*a.start)
    n.create_timer(a.period, n.report)
    end = n.get_clock().now().nanoseconds + int(a.duration * 1e9)
    while rclpy.ok() and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
