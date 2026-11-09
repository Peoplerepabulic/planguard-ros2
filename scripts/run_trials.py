"""Batch runner: seeds x shield modes, headless, one fresh simulator per trial.

    python scripts/run_trials.py --seeds 0-19 --tag main          # the reported experiment
    python scripts/run_trials.py --seeds 100-104 --tag pilot      # tuning seeds (never reported)

Every trial is appended to results/<tag>_trials.csv as soon as it finishes; no trial is
ever dropped or re-run selectively. Modes are interleaved within each seed.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import worldgen  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
FIELDS = ["tag", "seed", "mode", "outcome", "t_end", "min_clearance", "min_clearance_with", "hit",
          "path_length", "interventions", "active_frac", "x_end", "y_end", "wall_s", "rtf",
          "raw_cmds", "error"]


def parse_seeds(s: str) -> list[int]:
    out = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def kill_group(p: subprocess.Popen) -> None:
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGKILL, 5)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        try:
            p.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def sweep_strays() -> None:
    for pat in ("gz sim", "parameter_bridge", "planguard_control/lib", "robot_state_publisher"):
        subprocess.run(["pkill", "-9", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def run_one(seed: int, mode: str, tag: str, out: Path, timeout: float, idx: int,
            extra: list[str] | None = None) -> dict:
    stem = out / "worlds" / f"seed_{seed:03d}"
    if not stem.with_suffix(".json").exists():
        worldgen.write(worldgen.build(seed), stem)
    logs = out / "logs"
    trajs = out / "traj"
    logs.mkdir(parents=True, exist_ok=True)
    trajs.mkdir(parents=True, exist_ok=True)
    name = f"{tag}_s{seed:03d}_{mode}"

    env = dict(os.environ)
    env["GZ_PARTITION"] = f"pg_{name}_{os.getpid()}"
    env["ROS_DOMAIN_ID"] = str(1 + idx % 100)       # isolate DDS graphs between trials
    launch = subprocess.Popen(
        ["ros2", "launch", "planguard_description", "sim.launch.py",
         f"world:={stem.with_suffix('.sdf').resolve()}", f"shield_mode:={mode}", *(extra or [])],
        stdout=open(logs / f"{name}.log", "w"), stderr=subprocess.STDOUT,
        env=env, start_new_session=True)
    res_path = logs / f"{name}.result.json"
    try:
        subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "trial_monitor.py"),
             "--world-json", str(stem.with_suffix(".json")), "--result-out", str(res_path),
             "--traj-out", str(trajs / f"{name}.csv"), "--timeout", str(timeout)],
            env=env, timeout=6 * timeout + 180, check=False, stdout=subprocess.DEVNULL)
        res = json.load(open(res_path))
    except Exception as e:  # noqa: BLE001
        res = {"outcome": "ERROR", "error": repr(e)}
    finally:
        kill_group(launch)
        sweep_strays()
    res.update({"tag": tag, "seed": seed, "mode": mode})
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-19")
    ap.add_argument("--modes", default="off,brake,cbf")
    ap.add_argument("--tag", default="main")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--out", default=str(ROOT / "results"))
    ap.add_argument("--launch-arg", action="append", default=[], help="extra launch arg, e.g. cbf_alpha:=1.0")
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cfg = (ROOT / "src" / "planguard_control" / "config" / "control.yaml").read_bytes()
    meta = {"tag": a.tag, "seeds": a.seeds, "modes": a.modes, "timeout": a.timeout, "launch_args": a.launch_arg,
            "control_yaml_sha256": hashlib.sha256(cfg).hexdigest()[:16],
            "started": time.strftime("%Y-%m-%d %H:%M:%S"), "host": platform.node(),
            "cpus": os.cpu_count()}
    try:
        meta["git"] = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip()
    except Exception:  # noqa: BLE001
        pass
    json.dump(meta, open(out / f"{a.tag}_meta.json", "w"), indent=1)

    csv_path = out / f"{a.tag}_trials.csv"
    new = not csv_path.exists()
    sweep_strays()
    idx = 0
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        for seed in parse_seeds(a.seeds):
            for mode in a.modes.split(","):
                t = time.time()
                r = run_one(seed, mode, a.tag, out, a.timeout, idx, a.launch_arg)
                idx += 1
                w.writerow(r)
                f.flush()
                print(f"[{time.strftime('%H:%M:%S')}] seed={seed:3d} mode={mode:5s} {r.get('outcome'):9s} "
                      f"t={r.get('t_end', '-')} minclr={r.get('min_clearance', '-')} "
                      f"int={r.get('interventions', '-')} wall={time.time() - t:.0f}s", flush=True)


if __name__ == "__main__":
    main()
