"""Arena generator: fixed layout + seeded random obstacles near the start-goal diagonal.

Writes a Gazebo SDF world and a JSON sidecar with exact obstacle geometry, which the
trial monitor uses as ground truth for clearance/collision. The controller and shields
never read the JSON.

    python scripts/worldgen.py --seed 3 --out results/worlds/seed_03
    python scripts/worldgen.py --fixed-only --out src/planguard_description/worlds/arena
"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

ARENA_HALF = 4.0          # free space is [-4, 4]^2
WALL_T = 0.10
OBST_H = 0.50             # all obstacles taller than the lidar plane (0.26 m)
START = (-3.0, -3.0)
GOAL = (3.0, 3.0)

# Fixed obstacles: every one is >= 0.85 m (edge) from the start-goal diagonal,
# so the straight path is only ever blocked by the seeded random obstacles.
FIXED = [
    {"name": "fixed_box_1", "type": "box", "x": -1.5, "y": 1.0, "sx": 0.6, "sy": 0.6, "yaw": 0.0},
    {"name": "fixed_cyl_1", "type": "cylinder", "x": 0.5, "y": 2.5, "r": 0.25},
    {"name": "fixed_box_2", "type": "box", "x": -2.8, "y": 0.5, "sx": 0.5, "sy": 0.8, "yaw": 0.0},
    {"name": "fixed_cyl_2", "type": "cylinder", "x": 2.0, "y": -1.0, "r": 0.30},
    {"name": "fixed_box_3", "type": "box", "x": 1.0, "y": -2.5, "sx": 0.7, "sy": 0.4, "yaw": 0.0},
    {"name": "fixed_cyl_3", "type": "cylinder", "x": -0.5, "y": -2.0, "r": 0.20},
    {"name": "fixed_box_4", "type": "box", "x": 2.6, "y": 0.8, "sx": 0.5, "sy": 0.5, "yaw": 0.0},
]

# Random obstacles: centre at fraction s along the diagonal, lateral offset u (m),
# perpendicular to the diagonal (positive = left of travel direction).
RANDOM_SLOTS = [(0.25, 0.45), (0.55, 0.75)]
LATERAL_RANGE = 1.00


def walls() -> list[dict]:
    a = ARENA_HALF + WALL_T / 2
    L = 2 * ARENA_HALF + 2 * WALL_T
    return [
        {"name": "wall_n", "type": "box", "x": 0.0, "y": a, "sx": L, "sy": WALL_T, "yaw": 0.0},
        {"name": "wall_s", "type": "box", "x": 0.0, "y": -a, "sx": L, "sy": WALL_T, "yaw": 0.0},
        {"name": "wall_e", "type": "box", "x": a, "y": 0.0, "sx": WALL_T, "sy": L, "yaw": 0.0},
        {"name": "wall_w", "type": "box", "x": -a, "y": 0.0, "sx": WALL_T, "sy": L, "yaw": 0.0},
    ]


def random_obstacles(seed: int) -> list[dict]:
    rng = random.Random(seed)
    dx, dy = GOAL[0] - START[0], GOAL[1] - START[1]
    length = math.hypot(dx, dy)
    ux, uy = dx / length, dy / length          # along-path unit vector
    nx, ny = -uy, ux                           # left normal
    out = []
    for i, (lo, hi) in enumerate(RANDOM_SLOTS):
        s = rng.uniform(lo, hi)
        u = rng.uniform(-LATERAL_RANGE, LATERAL_RANGE)
        cx = START[0] + s * dx + u * nx
        cy = START[1] + s * dy + u * ny
        if rng.random() < 0.5:
            side = rng.uniform(0.40, 0.70)
            o = {"type": "box", "sx": side, "sy": side, "yaw": rng.uniform(0.0, math.pi / 2)}
        else:
            o = {"type": "cylinder", "r": rng.uniform(0.15, 0.35)}
        o.update({"name": f"rand_{i}", "x": round(cx, 4), "y": round(cy, 4),
                  "s": round(s, 4), "lateral": round(u, 4)})
        out.append(o)
    return out


def _model_sdf(o: dict, color: str) -> str:
    if o["type"] == "box":
        geom = f"<box><size>{o['sx']} {o['sy']} {OBST_H}</size></box>"
        yaw = o.get("yaw", 0.0)
    else:
        geom = f"<cylinder><radius>{o['r']}</radius><length>{OBST_H}</length></cylinder>"
        yaw = 0.0
    return f"""
    <model name="{o['name']}">
      <static>true</static>
      <pose>{o['x']} {o['y']} {OBST_H / 2} 0 0 {yaw}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual"><geometry>{geom}</geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>"""


def world_sdf(obstacles: list[dict], name: str = "arena") -> str:
    models = []
    for o in obstacles:
        if o["name"].startswith("wall"):
            color = "0.55 0.55 0.6 1"
        elif o["name"].startswith("rand"):
            color = "0.8 0.15 0.15 1"
        else:
            color = "0.25 0.35 0.55 1"
        models.append(_model_sdf(o, color))
    goal_marker = f"""
    <model name="goal_marker">
      <static>true</static>
      <pose>{GOAL[0]} {GOAL[1]} 0.001 0 0 0</pose>
      <link name="link"><visual name="visual">
        <geometry><cylinder><radius>0.3</radius><length>0.002</length></cylinder></geometry>
        <material><ambient>0.1 0.7 0.2 1</ambient><diffuse>0.1 0.7 0.2 1</diffuse></material>
      </visual></link>
    </model>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.85 0.9 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>12 12</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>12 12</size></plane></geometry>
          <material><ambient>0.85 0.85 0.8 1</ambient><diffuse>0.85 0.85 0.8 1</diffuse></material></visual>
      </link>
    </model>{''.join(models)}{goal_marker}
  </world>
</sdf>
"""


def build(seed: int | None) -> dict:
    obstacles = walls() + FIXED + ([] if seed is None else random_obstacles(seed))
    return {"seed": seed, "start": START, "goal": GOAL, "arena_half": ARENA_HALF,
            "obstacle_height": OBST_H, "obstacles": obstacles}


def write(spec: dict, out_stem: Path) -> None:
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    out_stem.with_suffix(".sdf").write_text(world_sdf(spec["obstacles"]))
    out_stem.with_suffix(".json").write_text(json.dumps(spec, indent=1))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int)
    ap.add_argument("--fixed-only", action="store_true")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    write(build(None if a.fixed_only else a.seed), Path(a.out))


if __name__ == "__main__":
    main()
