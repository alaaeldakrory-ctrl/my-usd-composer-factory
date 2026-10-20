"""Walk quality lab (no Kit): gait numbers vs the Mixamo reference and a side-view filmstrip of one worker walking.

    python walk_lab.py OUT_DIR [worker A|B] [t0] [t1]     (rack demo script; defaults: A, 40-45 s)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
sys.path.insert(0, HERE)
from PIL import Image   # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt   # noqa: E402
from assembly import choreo as C, choreo_rack as CR, gait as GT, rig as RG, scene_rack as SR   # noqa: E402
import preview as PV, skin_preview as SK   # noqa: E402


def main(out, who="A", t0=40.0, t1=45.0):
    os.makedirs(out, exist_ok=True)
    st = Usd.Stage.CreateInMemory()
    parts, chars = SR.build(st)
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    tracks, workers, end = CR.script(parts, rigs)
    ctx = C.Ctx(tracks, workers)
    W, rig = workers[who], rigs[who]
    g = GT.Gait(rig, W.root_at, end, floor=C.floor_fn(ctx))
    steps = sorted([s for side in "LR" for s in g.steps[side] if t0 <= s.t0 <= t1], key=lambda s: s.t0)
    if len(steps) > 1:
        per = (steps[-1].t0 - steps[0].t0) / (len(steps) - 1)
        lens = [(s.b[0] - s.a[0]).GetLength() / 2 for s in steps]
        print(f"{who}: {len(steps)} steps, {per:.2f} s/step ({60 / per:.0f} steps/min), step length "
              f"{sum(lens) / len(lens):.0f} cm, swing {sum(s.t1 - s.t0 for s in steps) / len(steps):.2f} s")
    frames = [int(round(t * C.FPS)) for t in [t0 + 0.1 * k for k in range(int((t1 - t0) / 0.1) + 1)]]
    poses = C.pose_worker(rig, W, ctx, g, C.Posture(rig), frames)
    anim = UsdSkel.Animation.Define(st, f"/Anim{who}")
    anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
    knee_max, ankle_max = 0.0, 0.0
    for f, (pose, w) in zip(frames, poses):
        for a, v in zip((anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr()), rig.arrays(pose, w)):
            a.Set(v, f)
        for s in "LR":
            th, kn, an = rig.pos(w, s + "_Thigh"), rig.pos(w, s + "_Calf"), rig.pos(w, s + "_Foot")
            v1, v2 = th - kn, an - kn
            ang = 180 - math.degrees(math.acos(max(-1, min(1, Gf.Dot(v1, v2) / v1.GetLength() / v2.GetLength()))))
            knee_max = max(knee_max, ang)
            ankle_max = max(ankle_max, an[1])
    print(f"  max knee flexion {knee_max:.0f} deg (mocap ~69), max ankle height {ankle_max:.0f} cm (mocap ~26)")
    UsdSkel.BindingAPI.Apply(rig.prim).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    other = "B" if who == "A" else "A"
    UsdGeom.Imageable(st.GetPrimAtPath(f"/World/Worker{other}")).MakeInvisible()
    ims = []
    for f in frames[::2]:
        p, h = W.root_at(f / C.FPS)
        side = RG.left_of(h)
        eye = p + side * 330 + Gf.Vec3d(0, 95, 0)
        geo = SK.skinned(st, f)
        path = os.path.join(out, f"w{f:05d}.png")
        PV.render(geo, path, eye, p + Gf.Vec3d(0, 85, 0), size=(220, 330), fov=40)
        ims.append(Image.open(path))
    cols = 9
    sheet = Image.new("RGB", (220 * cols, 330 * ((len(ims) + cols - 1) // cols)), (255, 255, 255))
    for k, im in enumerate(ims):
        sheet.paste(im, ((k % cols) * 220, (k // cols) * 330))
    sheet.save(os.path.join(out, f"walk_{who}.png"))


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0], a[1] if len(a) > 1 else "A", float(a[2]) if len(a) > 2 else 40.0, float(a[3]) if len(a) > 3 else 45.0)
