"""Offline preview frames of the rack assembly (no Kit)."""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
sys.path.insert(0, HERE)
from PIL import Image
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt
from assembly import choreo as C, choreo_rack as CC, gait as GT, rig as RG, scene_rack as SCL, director as D
import preview as PV, skin_preview as SK

def main(out, times, size=(560, 420)):
    os.makedirs(out, exist_ok=True)
    st = Usd.Stage.CreateInMemory()
    parts, chars = SCL.build(st)
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    tracks, workers, end = CC.script(parts, rigs)
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
            for a, v in zip((anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr()), rig.arrays(pose, w)):
                a.Set(v, f)
        UsdSkel.BindingAPI.Apply(rig.prim).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    ims = []
    for f, t in zip(frames, times):
        geo = PV.collect(st, Usd.TimeCode(f), skip=("/World/Worker", "/World/Environment")) + SK.skinned(st, f)
        eye, at = D.at(t, CC.DIRECTOR_KEYS)
        p = os.path.join(out, f"t{t:05.1f}.png")
        PV.render(geo, p, eye, at, size=size, fov=45)
        ims.append(Image.open(p))
    cols = 3
    sheet = Image.new("RGB", (size[0] * cols, size[1] * ((len(ims) + cols - 1) // cols)), (255, 255, 255))
    for k, im in enumerate(ims):
        sheet.paste(im, ((k % cols) * size[0], (k // cols) * size[1]))
    sheet.save(os.path.join(out, "sheet.png"))

if __name__ == "__main__":
    main(sys.argv[1], [float(a) for a in sys.argv[2:]])
