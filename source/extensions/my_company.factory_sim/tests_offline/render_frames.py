"""Render chosen moments of the v2 assembly offline (z-buffer preview) without baking the whole clip.

    python render_frames.py out_dir t1 t2 ... [--cam auto|overview] [--who A|B]
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
sys.path.insert(0, HERE)
import numpy as np   # noqa: E402
from PIL import Image   # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt   # noqa: E402
from assembly import choreo as C, gait as GT, rig as RG, scene as SC   # noqa: E402
import preview as PV   # noqa: E402
import skin_preview as SK   # noqa: E402


def main(out, times, cam="auto", who=None, size=(640, 480)):
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    st = Usd.Stage.CreateInMemory()
    parts, chars = SC.build(st)
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    tracks, workers, end = C.script(parts, rigs)
    ctx = C.Ctx(tracks, workers)
    frames = [int(round(t * C.FPS)) for t in times]
    for name, tr in tracks.items():
        op = UsdGeom.Xformable(st.GetPrimAtPath(parts[name].path)).GetOrderedXformOps()[0]
        for f in frames:
            op.Set(tr.matrix(f / C.FPS, workers), f)
    for name, W in workers.items():
        rig = rigs[name]
        g = GT.Gait(rig, W.root_at, end, floor=C.floor_fn(ctx))
        poses = C.pose_worker(rig, W, ctx, g, C.Posture(rig), frames)
        anim = UsdSkel.Animation.Define(st, f"/Anim{name}")
        anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
        for f, (pose, w) in zip(frames, poses):
            for a, v in zip((anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr()),
                            rig.arrays(pose, w)):
                a.Set(v, f)
        UsdSkel.BindingAPI.Apply(rig.prim).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
        C.write_wrench(st, name, W, ctx, max(frames) + 1)
    print(f"posed {len(frames)} frames in {time.time() - t0:.1f}s")
    paths = []
    for f, t in zip(frames, times):
        geo = PV.collect(st, Usd.TimeCode(f), skip=("/World/Worker", "/World/Environment")) + SK.skinned(st, f)
        if cam == "overview":
            eye, at = (520, 330, 600), (0, 110, 0)
        elif cam == "side":
            p, h = workers[who or "A"].root_at(t)
            sd = RG.left_of(h)
            if Gf.Dot(sd, p) < 0:                   # keep the camera on the open side, away from the lift
                sd = -sd
            eye, at = p + sd * 330 + RG.fwd_of(h) * 60 + Gf.Vec3d(0, 100, 0), p + Gf.Vec3d(0, 85, 0)
        else:
            pts = []
            for name in ([who] if who else workers):
                p, h = workers[name].root_at(t)
                pts.append(p)
            c = sum(pts, Gf.Vec3d(0, 0, 0)) / len(pts)
            fwd = RG.fwd_of(workers[who or "A"].root_at(t)[1])
            side = RG.left_of(workers[who or "A"].root_at(t)[1])
            ov = Gf.Vec3d(c[0], 0, c[2])
            ov = ov.GetNormalized() if ov.GetLength() > 1 else Gf.Vec3d(0, 0, 1)
            eye = c + (ov * 1.0 + side * 0.55).GetNormalized() * 270 + Gf.Vec3d(0, 150, 0) if who else c + Gf.Vec3d(330, 230, 380)
            at = c + Gf.Vec3d(0, 95, 0)
        p = os.path.join(out, f"t{t:05.1f}.png")
        PV.render(geo, p, eye, at, size=size, fov=40)
        paths.append(p)
        print("  rendered", p, f"{time.time() - t0:.0f}s")
    return paths


if __name__ == "__main__":
    args = sys.argv[1:]
    cam, who = "auto", None
    if "--cam" in args:
        k = args.index("--cam")
        cam = args[k + 1]
        del args[k:k + 2]
    if "--who" in args:
        k = args.index("--who")
        who = args[k + 1]
        del args[k:k + 2]
    out = args[0]
    paths = main(out, [float(a) for a in args[1:]], cam, who)
    ims = [Image.open(p) for p in paths]
    cols = min(3, len(ims))
    rows = (len(ims) + cols - 1) // cols
    sheet = Image.new("RGB", (ims[0].width * cols, ims[0].height * rows), (255, 255, 255))
    for k, im in enumerate(ims):
        sheet.paste(im, ((k % cols) * im.width, (k // cols) * im.height))
    sheet.save(os.path.join(out, "sheet.png"))
