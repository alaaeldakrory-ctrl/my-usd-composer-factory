"""Checks a baked assembly demo by reading joint positions back through UsdSkel (what Kit will render)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdGeom, UsdSkel   # noqa: E402
from assembly import choreo as C, rig as RG, scene as SC   # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\alaae\AppData\Local\Temp\RailAssembly_test.usda"
st = Usd.Stage.Open(PATH)
cache = UsdSkel.Cache()
rigs, queries = {}, {}
for n in ("A", "B"):
    prim = st.GetPrimAtPath(f"/World/Worker{n}")
    skel = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))
    rigs[n] = RG.CCRig(st, skel)
    root = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Root))
    cache.Populate(UsdSkel.Root(root), Usd.TraverseInstanceProxies())
    queries[n] = cache.GetSkelQuery(UsdSkel.Skeleton(skel))

# rebuild the script to know the intended targets
parts_def = {}
for side in "LR":
    parts_def["Side" + side] = SC.Part(f"/World/Rails/Side{side}", {"A": (Gf.Vec3d(SC.POST_X[0], SC.GRIP_LO, 0), Gf.Vec3d(SC.POST_X[0], SC.GRIP_HI, 0)),
                                       "B": (Gf.Vec3d(SC.POST_X[-1], SC.GRIP_LO, 0), Gf.Vec3d(SC.POST_X[-1], SC.GRIP_HI, 0))},
                                       (Gf.Vec3d(0, SC.DECK_Y, (1 if side == "L" else -1) * SC.SOCKET_Z), 0.0),
                                       (Gf.Vec3d(0, SC.STAND_Y, (1 if side == "L" else -1) * 255.0), 0.0))
h = SC.END_W / 2
for end, x0 in (("F", 250.0), ("B", 280.0)):
    sgn = 1 if end == "F" else -1
    parts_def["End" + end] = SC.Part(f"/World/Rails/End{end}", {"one": (Gf.Vec3d(0, SC.MID_RAIL, h * 0.55), Gf.Vec3d(0, SC.MID_RAIL, -h * 0.55))},
                                     (Gf.Vec3d(sgn * SC.END_X, SC.DECK_Y, 0), 0.0), (Gf.Vec3d(x0, 78.0 - SC.MID_RAIL + 6, 0), 0.0))
parts, workers, end = C.script(parts_def, rigs)
ctx = C.Ctx(parts, workers)
n_frames = int(st.GetEndTimeCode()) + 1


def world_joints(n, f):
    q = queries[n]
    xf = q.ComputeJointWorldTransforms(UsdGeom.XformCache(Usd.TimeCode(f)))
    names = rigs[n].names
    return {names[i]: Gf.Vec3d(m.ExtractTranslation()) for i, m in enumerate(xf)}


worst_hand = {n: (0, None) for n in workers}
slide = {n: 0.0 for n in workers}
clear = {n: 1e9 for n in workers}
prev_feet = {n: None for n in workers}
for f in range(0, n_frames, 3):
    t = f / C.FPS
    for n, W in workers.items():
        J = world_joints(n, f)
        specs = C.hand_specs(W, t, ctx)
        for side in "LR":
            s = specs[side]
            if s is None or (isinstance(s, tuple) and s[0] == "blend"):
                continue
            grip = s[0]
            palm = (J[side + "_Hand"] + J[side + "_Mid1"]) / 2
            err = (palm - grip).GetLength()
            if err > worst_hand[n][0]:
                worst_hand[n] = (err, round(t, 1))
        # foot sliding: a foot that is low (planted) should not move between samples
        feet = {s: J[s + "_Foot"] for s in "LR"}
        if prev_feet[n]:
            for s in "LR":
                if feet[s][1] < rigs[n].ankle_h + 1.0 and prev_feet[n][s][1] < rigs[n].ankle_h + 1.0:
                    d = math.hypot(feet[s][0] - prev_feet[n][s][0], feet[s][2] - prev_feet[n][s][2])
                    slide[n] = max(slide[n], d) if d < 30 else slide[n]
        prev_feet[n] = feet
        # clearance from the lift body (box |x|<114, |z|<39) for the pelvis
        p = J["Hip"]
        dx, dz = max(abs(p[0]) - 114, 0), max(abs(p[2]) - 39, 0)
        clear[n] = min(clear[n], math.hypot(dx, dz) if (dx or dz) else -1)
for n in workers:
    print(f"Worker {n}: worst palm-to-grip {worst_hand[n][0]:.1f} cm at {worst_hand[n][1]}s, "
          f"max planted-foot slide per 0.1s {slide[n]:.1f} cm, closest pelvis to lift {clear[n]:.1f} cm")
for name, p in parts_def.items():
    m = parts[name].end_matrix(workers)
    print(f"{name}: final position {[round(v, 1) for v in m.ExtractTranslation()]} (install {[round(v, 1) for v in p.install[0]]})")
