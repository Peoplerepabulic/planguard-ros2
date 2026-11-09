"""Evaluator for one trial. Ground truth only: it never sees the lidar.

Outcome rules (fixed before any reported run):
  COLLISION  clearance between the collision footprint (chassis 0.40 x 0.30 plus wheel
             spheres r = 0.06 at y = +-0.17) at the ground-truth pose and any obstacle or
             wall footprint is <= 0.01 m. Checked first; ends the trial.
  GOAL       ground-truth base centre within 0.30 m of the goal.
  TIMEOUT    90 s of simulation time after the first ground-truth message.
  ERROR      no ground truth within the startup window, or the simulation stalls.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from shapely import affinity
from shapely.geometry import Point, box
from shapely.ops import unary_union
from std_msgs.msg import Bool, UInt32

FOOTPRINT = unary_union([box(-0.20, -0.15, 0.20, 0.15),
                         Point(0.0, 0.17).buffer(0.06, 32),
                         Point(0.0, -0.17).buffer(0.06, 32)])


def obstacle_shape(o: dict):
    if o["type"] == "box":
        g = box(-o["sx"] / 2, -o["sy"] / 2, o["sx"] / 2, o["sy"] / 2)
        g = affinity.rotate(g, o.get("yaw", 0.0), origin=(0, 0), use_radians=True)
        return affinity.translate(g, o["x"], o["y"])
    return Point(o["x"], o["y"]).buffer(o["r"], 64)


def footprint_at(x: float, y: float, yaw: float):
    return affinity.translate(affinity.rotate(FOOTPRINT, yaw, origin=(0, 0), use_radians=True), x, y)


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self, spec: dict, timeout: float, goal_tol: float, collision_tol: float):
        super().__init__("trial_monitor", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.obstacles = [(o["name"], obstacle_shape(o)) for o in spec["obstacles"]]
        self.goal = spec["goal"]
        self.timeout, self.goal_tol, self.collision_tol = timeout, goal_tol, collision_tol
        self.t0 = None
        self.last = None
        self.outcome = None
        self.result = {}
        self.min_clear = math.inf
        self.min_clear_with = ""
        self.path = 0.0
        self.traj = []
        self.interventions = 0
        self.active_n = 0
        self.active_total = 0
        self.raw_cmds = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 50)
        self.create_subscription(UInt32, "/shield/interventions", self.on_count, 10)
        self.create_subscription(Bool, "/shield/active", self.on_active, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.on_raw, 10)

    def on_count(self, m):
        self.interventions = m.data

    def on_active(self, m):
        self.active_total += 1
        self.active_n += int(m.data)

    def on_raw(self, _):
        self.raw_cmds += 1

    def on_gt(self, m: Odometry):
        if self.outcome:
            return
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        p = m.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if self.t0 is None:
            self.t0 = t
        if self.last is not None:
            self.path += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y)

        fp = footprint_at(x, y, yaw)
        clear, who = min((fp.distance(g), n) for n, g in self.obstacles)
        if clear < self.min_clear:
            self.min_clear, self.min_clear_with = clear, who
        el = t - self.t0
        if not self.traj or el - self.traj[-1][0] >= 0.1:
            self.traj.append((round(el, 3), round(x, 4), round(y, 4), round(yaw, 4), round(clear, 4)))

        dgoal = math.hypot(x - self.goal[0], y - self.goal[1])
        if clear <= self.collision_tol:
            self.finish("COLLISION", el, x, y, who)
        elif dgoal <= self.goal_tol:
            self.finish("GOAL", el, x, y, "")
        elif el >= self.timeout:
            self.finish("TIMEOUT", el, x, y, "")

    def finish(self, outcome, el, x, y, hit):
        self.outcome = outcome
        self.traj.append((round(el, 3), round(x, 4), round(y, 4), 0.0, round(self.min_clear, 4)))
        self.result = {
            "outcome": outcome, "t_end": round(el, 3), "x_end": round(x, 4), "y_end": round(y, 4),
            "hit": hit, "min_clearance": round(self.min_clear, 4), "min_clearance_with": self.min_clear_with,
            "path_length": round(self.path, 4), "interventions": self.interventions,
            "active_frac": round(self.active_n / self.active_total, 4) if self.active_total else 0.0,
            "raw_cmds": self.raw_cmds,
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world-json", required=True)
    ap.add_argument("--result-out", required=True)
    ap.add_argument("--traj-out", required=True)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--goal-tol", type=float, default=0.30)
    ap.add_argument("--collision-tol", type=float, default=0.01)
    ap.add_argument("--startup-wall", type=float, default=90.0)
    a = ap.parse_args()

    spec = json.load(open(a.world_json))
    rclpy.init()
    mon = Monitor(spec, a.timeout, a.goal_tol, a.collision_tol)
    w0 = time.time()
    t_first_wall = None
    while rclpy.ok() and not mon.outcome:
        rclpy.spin_once(mon, timeout_sec=0.1)
        now = time.time()
        if mon.t0 is None and now - w0 > a.startup_wall:
            mon.result = {"outcome": "ERROR", "error": "no ground truth"}
            break
        if mon.t0 is not None and t_first_wall is None:
            t_first_wall = now
        if t_first_wall and now - t_first_wall > 4 * a.timeout + 60:
            mon.result = {"outcome": "ERROR", "error": "simulation stalled"}
            break
    wall = time.time() - (t_first_wall or w0)
    mon.result["wall_s"] = round(wall, 2)
    if mon.result.get("t_end"):
        mon.result["rtf"] = round(mon.result["t_end"] / max(wall, 1e-6), 3)
    with open(a.traj_out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "yaw", "clearance"])
        w.writerows(mon.traj)
    json.dump(mon.result, open(a.result_out, "w"))
    print(json.dumps(mon.result))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
