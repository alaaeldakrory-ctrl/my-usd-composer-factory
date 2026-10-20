"""Camera sightlines against the warehouse: occluder boxes from the environment and a "wide" director built from a
demo's close director keys (pulled back and up to show the building) that never looks through a column, a fire
box or a pile of crates.

    occ = occluders(stage)                         # world AABBs of the environment's meshes (min, max arrays)
    keys = widen(keys, occ)                        # same timing, wider framing, every sightline checked
    blocked(eye, targets, occ)                     # True if any target is hidden
"""
import math

import numpy as np
from pxr import Usd, UsdGeom


def occluders(stage, root="/World/Environment", y_band=(25.0, 450.0), max_span=450.0, pad=15.0):
    """(lo, hi) arrays of the environment meshes that can stand between a camera and the work: above the floor,
    below the roof trusses, smaller than a wall."""
    prim = stage.GetPrimAtPath(root)
    if not prim:
        return np.zeros((0, 3)), np.zeros((0, 3))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    lo, hi = [], []
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        r = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        a, b = np.array(r.GetMin()), np.array(r.GetMax())
        if b[1] < y_band[0] or a[1] > y_band[1] or (b - a)[0] > max_span or (b - a)[2] > max_span:
            continue
        lo.append(a - pad)
        hi.append(b + pad)
    return np.array(lo).reshape(-1, 3), np.array(hi).reshape(-1, 3)


def _hits(eye, target, occ):
    """Does the segment eye -> target pass through any occluder box (slab test, all boxes at once)?"""
    lo, hi = occ
    if not len(lo):
        return False
    e = np.asarray(eye, float)
    d = np.asarray(target, float) - e
    d = np.where(np.abs(d) < 1e-9, 1e-9, d)
    t1, t2 = (lo - e) / d, (hi - e) / d
    tmin = np.minimum(t1, t2).max(axis=1)
    tmax = np.maximum(t1, t2).min(axis=1)
    return bool(np.any((tmax >= np.maximum(tmin, 0.0)) & (tmin <= 0.97)))


def blocked(eye, targets, occ):
    return any(_hits(eye, t, occ) for t in targets)


def inside(eye, occ):
    lo, hi = occ
    e = np.asarray(eye, float)
    return bool(len(lo) and np.any(np.all((e > lo) & (e < hi), axis=1)))


def widen(keys, occ, scale=1.85, lift=110.0, room=1350.0, spread=70.0, min_gap=5.0, extra_targets=(), targets_fn=None):
    """Wide version of director keys [(t, eye, at)]: each eye pulled back `scale` x from its look-at and raised;
    if that view is blocked, the camera swings round its look-at (10 deg steps either way, then a little closer)
    until the look-at, two points `spread` cm either side of it and any extra targets are all in clear view.
    Keys closer than min_gap seconds are merged so the wide camera moves slowly."""
    merged = []
    for k in keys:
        if merged and k[0] - merged[-1][0] < min_gap:
            continue
        merged.append(k)
    if keys and merged[-1][0] != keys[-1][0]:
        merged.append(keys[-1])
    out = []
    for j, (t, eye, at) in enumerate(merged):
        ta = merged[j - 1][0] if j else t
        tb = merged[j + 1][0] if j + 1 < len(merged) else t
        window = [np.asarray(x, float) for tt in np.linspace((ta + t) / 2, (t + tb) / 2, 7)
                  for x in (targets_fn(tt) if targets_fn else ())]
        at = np.asarray(at, float)
        d = np.asarray(eye, float) - at
        dist = math.hypot(d[0], d[2]) * scale
        az0 = math.atan2(d[0], d[2])
        best = None
        for shrink in (1.0, 0.85, 0.72):
            for k in range(0, 13):
                for sgn in ((1,) if k == 0 else (1, -1)):
                    az = az0 + sgn * math.radians(10.0 * k)
                    r = dist * shrink
                    e = at + np.array([math.sin(az) * r, 0.0, math.cos(az) * r])
                    e[1] = eye[1] + lift
                    if abs(e[0]) > room or abs(e[2]) > room or inside(e, occ):
                        continue
                    side = np.array([math.cos(az), 0.0, -math.sin(az)]) * spread
                    targets = [at, at + side, at - side] + [np.asarray(x, float) for x in extra_targets] + window
                    if not blocked(e, targets, occ):
                        best = e
                        break
                if best is not None:
                    break
            if best is not None:
                break
        out.append((t, tuple(best if best is not None else np.asarray(eye, float)), tuple(at)))
    return out


def repair(keys, occ, at_fn, t_end, targets_fn, rounds=6, **kw):
    """widen() then, where the interpolated camera still loses sight, add a key mid-way through each blocked spell
    (solved the same way) until it is clear or `rounds` run out."""
    wide = widen(keys, occ, targets_fn=targets_fn, **kw)
    for _ in range(rounds):
        bad = report(wide, occ, at_fn, t_end, targets_fn=targets_fn)
        if not bad:
            break
        spells, cur = [], [bad[0]]
        for t in bad[1:]:
            if t - cur[-1] > 0.3:
                spells.append(cur)
                cur = [t]
            else:
                cur.append(t)
        spells.append(cur)
        extra = []
        for sp in spells:
            tm = (sp[0] + sp[-1]) / 2
            eye, at = at_fn(tm, keys)
            extra.append((tm, eye, at))
        fixed = widen(sorted(extra), occ, targets_fn=targets_fn, min_gap=0.0, **{k: v for k, v in kw.items() if k != "min_gap"})
        wide = sorted(wide + [k for k in fixed if all(abs(k[0] - w[0]) > 1.0 for w in wide)])
    return wide


def report(keys, occ, at_fn, t_end, dt=0.25, targets_fn=None):
    """Seconds (sampled) when the interpolated camera's view of the look-at (or targets_fn(t)) is blocked."""
    bad = []
    t = 0.0
    while t <= t_end:
        eye, at = at_fn(t, keys)
        tg = targets_fn(t) if targets_fn else [at]
        if blocked(eye, tg, occ) or inside(eye, occ):
            bad.append(round(t, 2))
        t += dt
    return bad
