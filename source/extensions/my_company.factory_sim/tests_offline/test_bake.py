"""Offline end-to-end: scan -> plan -> bake -> drivers on a test layout, then read the result back from USD.
Run:  python test_bake.py [today|two]"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdGeom          # noqa: E402
from forklift_ops import bake as B         # noqa: E402
from forklift_ops import driver as D       # noqa: E402
from forklift_ops import planner as P      # noqa: E402
from forklift_ops import scene as S        # noqa: E402
import make_test_stage                      # noqa: E402

layout = sys.argv[1] if len(sys.argv) > 1 else "two"
st = make_test_stage.build(layout)
sc = S.scan(st)
plan = P.Planner(sc, P.Settings(), log=lambda m: None).plan()
t0 = time.time()
end_f, per_agent = B.bake(st, sc, plan)
t1 = time.time()
notes = D.add_drivers(st, sc, plan, per_agent)
t2 = time.time()
print(f"baked {end_f} frames in {t1 - t0:.1f}s, drivers in {t2 - t1:.1f}s")
for n in notes:
    print(" ", n)

bbox_at = lambda prim, f: UsdGeom.BBoxCache(Usd.TimeCode(f), ["default", "render"]).ComputeWorldBound(prim).ComputeAlignedRange()
for name, tr in plan.pallets.items():
    prim = st.GetPrimAtPath("/World/" + name)
    r0, r1 = bbox_at(prim, 0), bbox_at(prim, end_f)
    det = [e for e in tr.events if e[1] == "detach"][0]
    print(f"  {name}: start center {[round(v) for v in r0.GetMidpoint()]} bottom {r0.GetMin()[1]:.1f} -> "
          f"end center {[round(v) for v in r1.GetMidpoint()]} bottom {r1.GetMin()[1]:.1f} (planned {[round(v) for v in det[2]]}, shelf {det[3]:.1f})")
for a in plan.agents:
    prim = a.fl.prim
    m0 = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
    m1 = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(end_f)
    print(f"  {a.fl.name}: start {[round(v) for v in m0.ExtractTranslation()]} end {[round(v) for v in m1.ExtractTranslation()]}")
    drv = st.GetPrimAtPath(prim.GetPath().AppendChild("Driver"))
    if drv:
        # driver must stay inside the cab footprint at a few times
        for f in (0, end_f // 3, 2 * end_f // 3):
            bb = bbox_at(drv, f)
            fb = bbox_at(prim, f)
            inside = all(fb.GetMin()[i] - 5 <= bb.GetMidpoint()[i] <= fb.GetMax()[i] + 5 for i in (0, 2))
            print(f"    frame {f}: driver center {[round(v) for v in bb.GetMidpoint()]}, top {bb.GetMax()[1]:.0f}, inside forklift {inside}")
out = os.path.join(os.environ.get("TEMP", "."), f"factory_ops_{layout}.usda")
st.GetRootLayer().Export(out)
print("exported", out)
