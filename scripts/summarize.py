"""Summary table, paired tests and figures for a batch of trials.

    python scripts/summarize.py --tag main

Writes results/<tag>_summary.md, results/<tag>_summary.json and docs/figures/*.png.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.stats import binomtest, wilcoxon  # noqa: E402
from shapely.geometry import LineString  # noqa: E402

import trial_monitor as tm  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
MODES = ["off", "brake", "cbf"]
MODE_LABEL = {"off": "No shield", "brake": "Brake shield", "cbf": "CBF-QP shield"}
OUTCOMES = ["GOAL", "TIMEOUT", "COLLISION"]
HALF_WIDTH = 0.23  # collision footprint half-width

# Reference palette (dataviz skill): categorical slots 1-3 for modes, status colors for outcomes.
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
MODE_COLOR = {"off": "#2a78d6", "brake": "#eb6834", "cbf": "#1baf7a"}
OUT_COLOR = {"GOAL": "#0ca30c", "TIMEOUT": "#fab219", "COLLISION": "#d03b3b"}
OUT_TEXT = {"GOAL": "#ffffff", "TIMEOUT": "#0b0b0b", "COLLISION": "#ffffff"}
OUT_MARK = {"GOAL": "o", "TIMEOUT": "s", "COLLISION": "X"}


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def mcnemar_exact(a: pd.Series, b: pd.Series) -> dict:
    """Paired binary outcomes a, b (aligned by seed). Exact two-sided test on discordant pairs."""
    n10 = int(((a == 1) & (b == 0)).sum())
    n01 = int(((a == 0) & (b == 1)).sum())
    p = binomtest(n10, n10 + n01, 0.5).pvalue if n10 + n01 else 1.0
    return {"a_only": n10, "b_only": n01, "p": p}


def blocker_features(spec: dict) -> dict:
    """Geometry of the seeded obstacles relative to the straight start-goal corridor."""
    corridor = LineString([spec["start"], spec["goal"]]).buffer(HALF_WIDTH)
    rand = [o for o in spec["obstacles"] if o["name"].startswith("rand")]
    blockers = [o for o in rand if corridor.intersects(tm.obstacle_shape(o))]
    return {"n_blockers": len(blockers),
            "min_abs_lateral": min((abs(o["lateral"]) for o in blockers), default=float("nan"))}


def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelcolor=INK2)


def fig_outcomes(df: pd.DataFrame, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 2.9), dpi=200)
    fig.patch.set_facecolor(SURF)
    style(ax)
    n = df.groupby("mode").size().max()
    for i, m in enumerate(reversed(MODES)):
        left = 0
        for o in OUTCOMES:
            k = int(((df["mode"] == m) & (df["outcome"] == o)).sum())
            if k == 0:
                continue
            ax.barh(i, k - 0.08, left=left + 0.04, height=0.56, color=OUT_COLOR[o], edgecolor=SURF, linewidth=0)
            if k >= 2:
                ax.text(left + k / 2, i, f"{o.title()} {k}", ha="center", va="center",
                        fontsize=8.5, color=OUT_TEXT[o], fontweight="bold")
            left += k
    ax.set_yticks(range(len(MODES)))
    ax.set_yticklabels([MODE_LABEL[m] for m in reversed(MODES)], color=INK, fontsize=10)
    ax.set_xlim(0, n)
    ax.set_xlabel(f"trials (n = {n} paired seeds per shield)", color=INK2, fontsize=9)
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    handles = [plt.Rectangle((0, 0), 1, 1, color=OUT_COLOR[o]) for o in OUTCOMES]
    ax.legend(handles, [o.title() for o in OUTCOMES], ncol=3, frameon=False, fontsize=8.5,
              loc="lower center", bbox_to_anchor=(0.5, 1.0), labelcolor=INK2)
    ax.set_title("Outcome per trial", color=INK, fontsize=11, loc="left", pad=22)
    fig.tight_layout()
    fig.savefig(out, facecolor=SURF)
    plt.close(fig)


def fig_offset(df: pd.DataFrame, feats: pd.DataFrame, out: Path) -> None:
    d = df.merge(feats, on="seed")
    fig, ax = plt.subplots(figsize=(7.2, 3.0), dpi=200)
    fig.patch.set_facecolor(SURF)
    style(ax)
    for i, m in enumerate(reversed(MODES)):
        sub = d[(d["mode"] == m) & (d["n_blockers"] > 0)]
        for o in OUTCOMES:
            s = sub[sub["outcome"] == o]
            ax.scatter(s["min_abs_lateral"], [i] * len(s), s=46, marker=OUT_MARK[o], color=OUT_COLOR[o],
                       edgecolor=SURF, linewidth=1.2, zorder=3, label=o.title() if i == 0 else None)
    ax.set_yticks(range(len(MODES)))
    ax.set_yticklabels([MODE_LABEL[m] for m in reversed(MODES)], color=INK, fontsize=10)
    ax.set_ylim(-0.6, len(MODES) - 0.4)
    ax.set_xlabel("|lateral offset| of the most centred blocking obstacle from the start-goal line (m)",
                  color=INK2, fontsize=8.5)
    ax.xaxis.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    ax.legend(ncol=3, frameon=False, fontsize=8.5, loc="lower center", bbox_to_anchor=(0.5, 1.0),
              labelcolor=INK2)
    nb = int((feats["n_blockers"] == 0).sum())
    ax.set_title(f"Where each shield fails (seeds with a blocked straight path; {nb} unblocked seeds omitted)",
                 color=INK, fontsize=10, loc="left", pad=22)
    fig.tight_layout()
    fig.savefig(out, facecolor=SURF)
    plt.close(fig)


def fig_traj(seed: int, tag: str, res: Path, df: pd.DataFrame, out: Path, title: str) -> None:
    spec = json.load(open(res / "worlds" / f"seed_{seed:03d}.json"))
    fig, ax = plt.subplots(figsize=(5.2, 5.2), dpi=200)
    fig.patch.set_facecolor(SURF)
    style(ax)
    for o in spec["obstacles"]:
        g = tm.obstacle_shape(o)
        fc = "#d9d8d2" if not o["name"].startswith("rand") else "#898781"
        ax.fill(*g.exterior.xy, color=fc, linewidth=0, zorder=1)
    ax.add_patch(plt.Circle(spec["goal"], 0.3, fill=False, ls="--", color=MUTED, lw=1))
    ax.text(spec["goal"][0], spec["goal"][1] + 0.42, "goal", ha="center", color=INK2, fontsize=8)
    ax.text(spec["start"][0], spec["start"][1] - 0.45, "start", ha="center", color=INK2, fontsize=8)
    for m in MODES:
        p = res / "traj" / f"{tag}_s{seed:03d}_{m}.csv"
        if not p.exists():
            continue
        t = pd.read_csv(p)
        row = df[(df["seed"] == seed) & (df["mode"] == m)].iloc[0]
        ax.plot(t["x"], t["y"], color=MODE_COLOR[m], lw=2, zorder=3,
                label=f"{MODE_LABEL[m]}: {row['outcome'].lower()} ({row['t_end']:.0f} s)")
        ax.scatter(t["x"].iloc[-1], t["y"].iloc[-1], s=42, color=MODE_COLOR[m], marker=OUT_MARK[row["outcome"]],
                   edgecolor=SURF, linewidth=1.2, zorder=4)
    ax.set_aspect("equal")
    ax.set_xlim(-4.1, 4.1)
    ax.set_ylim(-4.1, 4.1)
    ax.set_xticks(range(-4, 5, 2))
    ax.set_yticks(range(-4, 5, 2))
    ax.grid(True, color=GRID, linewidth=0.5)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=8, loc="upper left", labelcolor=INK2)
    ax.set_title(title, color=INK, fontsize=10, loc="left")
    fig.tight_layout()
    fig.savefig(out, facecolor=SURF)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="main")
    ap.add_argument("--results", default=str(ROOT / "results"))
    ap.add_argument("--figures", default=str(ROOT / "docs" / "figures"))
    a = ap.parse_args()
    res, figs = Path(a.results), Path(a.figures)
    figs.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(res / f"{a.tag}_trials.csv")
    errors = df[df["outcome"] == "ERROR"]
    seeds = sorted(df["seed"].unique())
    feats = pd.DataFrame([{"seed": s, **blocker_features(json.load(open(res / "worlds" / f"seed_{s:03d}.json")))}
                          for s in seeds])

    rows, summ = [], {"tag": a.tag, "n_seeds": len(seeds), "errors": len(errors), "modes": {}}
    for m in MODES:
        d = df[df["mode"] == m]
        n = len(d)
        k = {o: int((d["outcome"] == o).sum()) for o in OUTCOMES}
        goal_t = d.loc[d["outcome"] == "GOAL", "t_end"]
        s = {"n": n, **{o.lower(): k[o] for o in OUTCOMES},
             "collision_rate": k["COLLISION"] / n, "collision_ci95": wilson(k["COLLISION"], n),
             "goal_rate": k["GOAL"] / n, "goal_ci95": wilson(k["GOAL"], n),
             "median_time_to_goal": float(goal_t.median()) if len(goal_t) else float("nan"),
             "median_min_clearance": float(d["min_clearance"].median()),
             "mean_interventions": float(d["interventions"].mean()),
             "mean_active_frac": float(d["active_frac"].mean())}
        summ["modes"][m] = s
        c_lo, c_hi = s["collision_ci95"]
        g_lo, g_hi = s["goal_ci95"]
        rows.append(f"| {MODE_LABEL[m]} | {k['COLLISION']}/{n} ({c_lo:.2f}–{c_hi:.2f}) | "
                    f"{k['GOAL']}/{n} ({g_lo:.2f}–{g_hi:.2f}) | {k['TIMEOUT']}/{n} | "
                    f"{s['median_time_to_goal']:.1f} | {s['median_min_clearance']:.3f} | "
                    f"{s['mean_interventions']:.1f} |")

    piv = df.pivot(index="seed", columns="mode", values="outcome")
    coll = (piv == "COLLISION").astype(int)
    goal = (piv == "GOAL").astype(int)
    tests = {
        "collision off vs brake": mcnemar_exact(coll["off"], coll["brake"]),
        "collision off vs cbf": mcnemar_exact(coll["off"], coll["cbf"]),
        "goal cbf vs brake": mcnemar_exact(goal["cbf"], goal["brake"]),
        "goal cbf vs off": mcnemar_exact(goal["cbf"], goal["off"]),
    }
    clr = df.pivot(index="seed", columns="mode", values="min_clearance")
    diff = clr["cbf"] - clr["brake"]
    try:
        w = wilcoxon(clr["cbf"], clr["brake"]) if (diff != 0).any() else None
        tests["min_clearance cbf vs brake (Wilcoxon)"] = {
            "median_diff": float(diff.median()), "p": float(w.pvalue) if w else 1.0}
    except ValueError:
        pass
    tt = df[df["outcome"] == "GOAL"].pivot(index="seed", columns="mode", values="t_end").dropna()
    if {"cbf", "off"} <= set(tt.columns) and len(tt) >= 3:
        both = tt[["cbf", "off"]].dropna()
        if len(both) >= 3 and (both["cbf"] != both["off"]).any():
            w = wilcoxon(both["cbf"], both["off"])
            tests["time_to_goal cbf vs off, seeds where both reach (Wilcoxon)"] = {
                "n": len(both), "median_diff": float((both["cbf"] - both["off"]).median()), "p": float(w.pvalue)}
    summ["tests"] = tests

    by_block = df.merge(feats, on="seed")
    blk = by_block.groupby(["mode", by_block["n_blockers"] > 0])["outcome"].value_counts().unstack(fill_value=0)
    summ["by_blocked"] = {f"{m}|{'blocked' if b else 'clear'}": r.to_dict() for (m, b), r in blk.iterrows()}

    md = [f"# Results: `{a.tag}` ({len(seeds)} paired seeds x {len(MODES)} shields)", "",
          f"ERROR trials: {len(errors)}", "",
          "| Shield | Collisions (95% Wilson CI) | Reached goal (95% CI) | Timeouts | Median time to goal (s) "
          "| Median min clearance (m) | Mean interventions |",
          "|---|---|---|---|---|---|---|", *rows, "",
          "## Paired tests (same seed = same world)", "",
          "| Comparison | Only first | Only second | p (exact) |", "|---|---|---|---|"]
    for name, t in tests.items():
        if "a_only" in t:
            md.append(f"| {name} | {t['a_only']} | {t['b_only']} | {t['p']:.4f} |")
    for name, t in tests.items():
        if "median_diff" in t:
            md.append(f"\n{name}: median difference {t['median_diff']:+.3f}, p = {t['p']:.4f}"
                      + (f", n = {t['n']}" if "n" in t else ""))
    md += ["", "## Outcomes split by whether a seeded obstacle blocks the straight path", "",
           "| Shield | Path | Goal | Timeout | Collision |", "|---|---|---|---|---|"]
    for (m, b), r in blk.iterrows():
        md.append(f"| {MODE_LABEL[m]} | {'blocked' if b else 'clear'} | {r.get('GOAL', 0)} | "
                  f"{r.get('TIMEOUT', 0)} | {r.get('COLLISION', 0)} |")
    md += ["", "## Per-seed outcomes", "", "| Seed | blockers | min abs lateral (m) | off | brake | cbf |",
           "|---|---|---|---|---|---|"]
    for s in seeds:
        f = feats[feats["seed"] == s].iloc[0]
        md.append(f"| {s} | {int(f['n_blockers'])} | {f['min_abs_lateral']:.3f} | "
                  + " | ".join(str(piv.loc[s, m]) for m in MODES) + " |")
    (res / f"{a.tag}_summary.md").write_text("\n".join(md) + "\n")
    json.dump(summ, open(res / f"{a.tag}_summary.json", "w"), indent=1, default=float)

    fig_outcomes(df, figs / "outcomes.png")
    fig_offset(df, feats, figs / "failure_vs_offset.png")
    # Example seeds chosen by fixed rules (lowest seed number satisfying each), not by eye.
    ex = [s for s in seeds if piv.loc[s, "off"] == "COLLISION" and piv.loc[s, "brake"] == "TIMEOUT"
          and piv.loc[s, "cbf"] == "GOAL"]
    if ex:
        fig_traj(ex[0], a.tag, res, df, figs / "traj_cbf_passes.png",
                 f"Seed {ex[0]}: off-centre obstacle (lowest such seed)")
    dl = [s for s in seeds if piv.loc[s, "cbf"] == "TIMEOUT"]
    if dl:
        fig_traj(dl[0], a.tag, res, df, figs / "traj_cbf_deadlock.png",
                 f"Seed {dl[0]}: CBF deadlock (lowest such seed)")
    print("\n".join(md))


if __name__ == "__main__":
    main()
