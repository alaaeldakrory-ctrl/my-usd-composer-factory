"""Wide director camera for the rack demo: solve, then report blocked sightlines to the workers (no Kit)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdSkel   # noqa: E402
from assembly import choreo_rack as CR, director as D, rig as RG, scene_rack as SR, sightline as SL   # noqa: E402

st = Usd.Stage.CreateInMemory()
parts, chars = SR.build(st)
occ = SL.occluders(st)
rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
tr, wk, end = CR.script(parts, rigs)
targets = CR.worker_targets(wk)
print("close director blocked s:", len(SL.report(CR.DIRECTOR_KEYS, occ, D.at, end, targets_fn=targets)) * 0.25)
wide, _ = CR.wide_keys(st, wk, end)
bad = SL.report(wide, occ, D.at, end, targets_fn=targets)
print("wide keys", len(wide), "blocked s:", len(bad) * 0.25, bad[:60])

import numpy as np   # noqa: E402
lo, hi = occ
for t in (14.5, 57.5, 122.0, 137.0, 146.5, 154.5, 158.0):
    eye, at = D.at(t, wide)
    for k, tg in enumerate(targets(t)):
        e = np.asarray(eye, float); d = np.asarray(tg, float) - e
        d = np.where(np.abs(d) < 1e-9, 1e-9, d)
        t1, t2 = (lo - e) / d, (hi - e) / d
        tmin = np.minimum(t1, t2).max(axis=1); tmax = np.maximum(t1, t2).min(axis=1)
        h = np.where((tmax >= np.maximum(tmin, 0.0)) & (tmin <= 0.97))[0]
        if len(h):
            print(t, "target", k, [round(v) for v in tg], "eye", [round(v) for v in eye], "box", lo[h[0]].round(), hi[h[0]].round())
            break
