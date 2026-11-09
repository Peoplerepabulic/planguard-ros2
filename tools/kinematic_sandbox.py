"""Fast kinematic replica of the Gazebo setup (unicycle + accel limits, 10 Hz ray-cast lidar,
same controller/shield math in Python). Used only on pilot seeds to choose CBF parameters.

    python tools/kinematic_sandbox.py '{"two_circles": {"cx":[0.12,-0.12],"cr":[0.26,0.26],"margin":0.05,"alpha":1.5,"ww":0.0144}}' 100 110
"""
import math, sys, json
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
import worldgen
from shapely.geometry import Point, box
from shapely import affinity
from shapely.ops import unary_union

FP = unary_union([box(-0.2, -0.15, 0.2, 0.15), Point(0, 0.17).buffer(0.06, 32), Point(0, -0.17).buffer(0.06, 32)])


def shape(o):
    if o["type"] == "box":
        g = box(-o["sx"] / 2, -o["sy"] / 2, o["sx"] / 2, o["sy"] / 2)
        g = affinity.rotate(g, o.get("yaw", 0.0), origin=(0, 0), use_radians=True)
        return affinity.translate(g, o["x"], o["y"])
    return Point(o["x"], o["y"]).buffer(o["r"], 64)


def segments(obs):
    segs = []
    for o in obs:
        g = shape(o)
        c = list(g.exterior.coords)
        for a, b in zip(c[:-1], c[1:]):
            segs.append((a[0], a[1], b[0], b[1]))
    return np.array(segs)


ANG = -math.pi + np.arange(360) * (2 * math.pi / 360)


def scan(segs, x, y, th, rng):
    d = np.stack([np.cos(ANG + th), np.sin(ANG + th)], 1)  # 360x2
    ax, ay, bx, by = segs.T
    ex, ey = bx - ax, by - ay
    # solve p + t d = a + s e
    den = d[:, 0:1] * ey[None] - d[:, 1:2] * ex[None]
    wx, wy = ax[None] - x, ay[None] - y
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (wx * ey[None] - wy * ex[None]) / den
        s = (wx * d[:, 1:2] - wy * d[:, 0:1]) / den
    ok = (t > 0) & (s >= 0) & (s <= 1)
    t = np.where(ok, t, np.inf)
    r = t.min(1) + rng.normal(0, 0.01, 360)
    r[(r < 0.12) | (r > 8)] = np.inf
    return r


def qp2d(ref, w0, w1, cons):
    s0, s1 = math.sqrt(w0), math.sqrt(w1)
    zr = np.array([ref[0] * s0, ref[1] * s1])
    L = []
    for a0, a1, b in cons:
        n0, n1 = a0 / s0, a1 / s1
        nn = math.hypot(n0, n1)
        if nn < 1e-12:
            if b < -1e-9:
                return None
            continue
        L.append((n0 / nn, n1 / nn, b / nn))
    L = np.array(L)
    N, C = L[:, :2], L[:, 2]
    cand = [zr]
    viol = N @ zr - C
    cand += list(zr[None] - viol[:, None] * N)
    m = len(L)
    for i in range(m):
        for j in range(i + 1, m):
            det = N[i, 0] * N[j, 1] - N[i, 1] * N[j, 0]
            if abs(det) < 1e-12:
                continue
            cand.append(np.array([(C[i] * N[j, 1] - N[i, 1] * C[j]) / det, (N[i, 0] * C[j] - C[i] * N[j, 0]) / det]))
    cand = np.array(cand)
    cost = ((cand - zr) ** 2).sum(1)
    for k in np.argsort(cost):
        if (N @ cand[k] <= C + 1e-9).all():
            return cand[k][0] / s0, cand[k][1] / s1
    return None


def cbf(ref, pts, P):
    cons = []
    for c, rho in zip(P["cx"], P["cr"]):
        d = rho + P["margin"]
        sv = 0.5 + abs(c) * 1.5
        R = (sv + math.sqrt(sv * sv + P["alpha"] ** 2 * d * d)) / P["alpha"]
        r = pts - np.array([c, 0])
        n = np.hypot(r[:, 0], r[:, 1])
        r, n = r[n <= R], n[n <= R]
        if len(r) == 0:
            continue
        key = np.floor(np.arctan2(r[:, 1], r[:, 0]) / math.radians(5)).astype(int)
        best = {}
        for kk, rr, nn in zip(key, r, n):
            if kk not in best or nn < best[kk][1]:
                best[kk] = (rr, nn)
        for rr, nn in best.values():
            cons.append((rr[0], c * rr[1], 0.5 * P["alpha"] * (nn * nn - d * d)))
    cons += [(1, 0, 0.5), (-1, 0, 0.2), (0, 1, 1.5), (0, -1, 1.5)]
    u = qp2d(ref, 1.0, P["ww"], cons)
    return (0.0, 0.0) if u is None else u


def brake(ref, pts):
    m = (pts[:, 0] > 0.2) & (np.abs(pts[:, 1]) <= 0.28)
    c = (pts[m, 0] - 0.2).min() if m.any() else np.inf
    lim = 0 if c <= 0.2 else (0.5 if c >= 0.7 else 0.5 * (c - 0.2) / 0.5)
    return (min(ref[0], lim) if ref[0] > 0 else ref[0], ref[1])


def cruise(x, y, th, goal=(3, 3)):
    dx, dy = goal[0] - x, goal[1] - y
    d = math.hypot(dx, dy)
    if d < 0.3:
        return (0, 0), True
    e = math.atan2(math.sin(math.atan2(dy, dx) - th), math.cos(math.atan2(dy, dx) - th))
    w = max(-1.5, min(1.5, 1.5 * e))
    v = 0 if abs(e) > 0.8 else min(0.5, 0.5 * d) * math.cos(e)
    return (v, w), False


def run(seed, mode, P=None, T=90.0):
    spec = worldgen.build(seed)
    obs = spec["obstacles"]
    shapes = [shape(o) for o in obs]
    segs = segments(obs)
    rng = np.random.default_rng(seed)
    x, y, th, v, w = -3.0, -3.0, 0.0, 0.0, 0.0
    dt = 0.05
    pts = None
    t = 0.0
    minc = 9
    while t < T:
        k = int(round(t / dt))
        if k % 2 == 0:  # 10 Hz scan
            r = scan(segs, x, y, th, rng)
            ok = np.isfinite(r)
            pts = np.stack([r[ok] * np.cos(ANG[ok]), r[ok] * np.sin(ANG[ok])], 1)
        ref, done = cruise(x, y, th)
        if math.hypot(x - 3, y - 3) <= 0.3:
            return "GOAL", t, minc
        u = ref if mode == "off" else (brake(ref, pts) if mode == "brake" else cbf(ref, pts, P))
        # accel limits
        v += max(-1.0 * dt, min(1.0 * dt, u[0] - v))
        w += max(-3.0 * dt, min(3.0 * dt, u[1] - w))
        for _ in range(5):
            x += v * math.cos(th) * dt / 5
            y += v * math.sin(th) * dt / 5
            th += w * dt / 5
        fp = affinity.translate(affinity.rotate(FP, th, origin=(0, 0), use_radians=True), x, y)
        c = min(fp.distance(g) for g in shapes)
        minc = min(minc, c)
        if c <= 0.01:
            return "COLLISION", t, minc
        t += dt
    return "TIMEOUT", t, minc


if __name__ == "__main__":
    variants = json.loads(sys.argv[1])
    seeds = range(int(sys.argv[2]), int(sys.argv[3]))
    for name, P in variants.items():
        res = [run(s, "cbf", P) for s in seeds]
        cnt = {o: sum(r[0] == o for r in res) for o in ("GOAL", "TIMEOUT", "COLLISION")}
        print(name, cnt, "minclr_med %.3f" % np.median([r[2] for r in res]), flush=True)
