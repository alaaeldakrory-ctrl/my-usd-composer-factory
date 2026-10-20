"""Checks the baked v2 demo through UsdSkel: hand contact, planted-foot slide, clearance from the lift, parts seated."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdGeom, UsdSkel   # noqa: E402
from assembly import choreo as C, rig as RG, scene as SC   # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\OmniverseData\RailAssembly_ES2632.usd"
CARLIFT = "CarLift" in PATH
RACK = "Rack_" in PATH
st = Usd.Stage.Open(PATH)
cache = UsdSkel.Cache()
rigs, queries = {}, {}
for n in "AB":
    prim = st.GetPrimAtPath(f"/World/Worker{n}")
    skel = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))
    rigs[n] = RG.CCRig(st, skel)
    root = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Root))
    cache.Populate(UsdSkel.Root(root), Usd.TraverseInstanceProxies())
    queries[n] = cache.GetSkelQuery(UsdSkel.Skeleton(skel))
if RACK:
    from assembly import choreo_rack as CC, scene_rack as SCL
    SCL.configure("improved" if "Improved" in PATH else "current")
    parts = SCL.make_parts()
    tracks, workers, end = CC.script(parts, rigs)
elif CARLIFT:
    from assembly import choreo_carlift as CC, scene_carlift as SCL
    parts = SCL.make_parts()
    tracks, workers, end = CC.script(parts, rigs)
else:
    parts = SC.make_parts()
    tracks, workers, end = C.script(parts, rigs)
ctx = C.Ctx(tracks, workers)
n_frames = int(st.GetEndTimeCode()) + 1
worst = {n: [] for n in "AB"}
slide = {n: 0.0 for n in "AB"}
slips = {n: [] for n in "AB"}
clear = {n: (1e9, None) for n in "AB"}
prev = {n: None for n in "AB"}
UP = Gf.Vec3d(0, 1, 0)
near_ab = (1e9, None)
for f in range(0, n_frames, 2):
    t = f / C.FPS
    hips = {}
    for n, W in workers.items():
        xf = queries[n].ComputeJointWorldTransforms(UsdGeom.XformCache(Usd.TimeCode(f)))
        J = {rigs[n].names[i]: Gf.Vec3d(m.ExtractTranslation()) for i, m in enumerate(xf)}
        for side in "LR":
            sp = C.hand_spec_at(W, side, t, ctx)
            if not sp or sp[0] != "free" or sp[2] < 1.0:
                continue
            spec = sp[1]
            if spec["kind"] != "tube":
                continue
            palm = J[side + "_Hand"] + (J[side + "_Mid1"] - J[side + "_Hand"]) * 0.75
            d = palm - spec["c"]
            ax = spec["axis"].GetNormalized()
            radial = (d - ax * Gf.Dot(d, ax)).GetLength()
            worst[n].append((abs(radial - (spec["r"] + 2.0)), round(t, 1), side))
        hips[n] = J["Hip"]
        feet = {s: J[s + "_ToeBase"] for s in "LR"}
        if prev[n]:
            for s in "LR":
                if feet[s][1] < 3.0 and prev[n][s][1] < 3.0:
                    d = math.hypot(feet[s][0] - prev[n][s][0], feet[s][2] - prev[n][s][2])
                    slide[n] = max(slide[n], d)
                    if d > 2.0:
                        slips[n].append(round(t, 1))
        prev[n] = feet
        for j in ("Hip", "L_Calf", "R_Calf", "Spine02"):
            p = J[j]
            dx, dz = (max(abs(p[0]) - 100.0, 0), max(abs(p[2]) - 51.3, 0)) if RACK else                 (max(abs(p[0]) - 223.0, 0), max(abs(p[2]) - 105.0, 0)) if CARLIFT else (max(abs(p[0] - 1.7) - 119.8, 0), max(abs(p[2]) - 41.3, 0))
            dd = math.hypot(dx, dz) if (dx or dz) else -1
            if dd < clear[n][0]:
                clear[n] = (dd, (j, round(t, 1)))
    dab = math.hypot(hips["A"][0] - hips["B"][0], hips["A"][2] - hips["B"][2])
    if dab < near_ab[0]:
        near_ab = (dab, round(t, 1))
print(f"closest A-B (hips) {near_ab[0]:.0f} cm at {near_ab[1]}s")
for n in "AB":
    w = sorted(worst[n], reverse=True)
    print(f"Worker {n}: grip error worst {w[0][0]:.1f} cm at {w[0][1]}s {w[0][2]}, p95 {w[len(w)//20][0]:.1f} cm; "
          f"max planted-toe slide per 2 frames {slide[n]:.2f} cm; closest body to lift {clear[n][0]:.1f} cm {clear[n][1]}")
    print("   toe slips > 2 cm at:", slips[n][:40])
    bad = sorted({int(e[1]) for e in w if e[0] > 3.0})
    print("   seconds with >3 cm:", bad)
for name, p in parts.items():
    if name == "StepStand":
        continue
    m = tracks[name].matrix(1e9, workers)
    if name == "CrossBar":                    # fitted end-for-end (symmetric): compare its pivot
        m = Gf.Matrix4d().SetTranslate(m.Transform(p.pivot) - p.pivot)
    print(name, "seated" if (m.ExtractTranslation().GetLength() < 1e-3) else "NOT seated", [round(v, 2) for v in m.ExtractTranslation()])
