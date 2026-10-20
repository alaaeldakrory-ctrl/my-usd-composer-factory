"""Preview frames straight from a baked demo file (parts, bolts, tools, ergonomics markers as baked), seen through one
of its cameras, optionally with the live ergonomics panel beside each frame.

    python render_baked.py out_dir t1 t2 ... [--cam Director|DirectorWide] [--hud] [--file path.usd]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
sys.path.insert(0, HERE)
from PIL import Image   # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdSkel   # noqa: E402
import preview as PV   # noqa: E402
import skin_preview as SK   # noqa: E402

args = sys.argv[1:]
out = args[0]
cam = args[args.index("--cam") + 1] if "--cam" in args else "Director"
path = args[args.index("--file") + 1] if "--file" in args else r"C:\OmniverseData\Rack_Assembly.usd"
times = [float(a) for a in args[1:] if a.replace(".", "", 1).isdigit()]
os.makedirs(out, exist_ok=True)
st = Usd.Stage.Open(path)
scores = steps = None
if "--hud" in args:
    from assembly import choreo_rack as CR, ergo as E, ergo_report as ER, rig as RG, scene_rack as SR
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(st.GetPrimAtPath(f"/World/Worker{n}")) if p.IsA(UsdSkel.Skeleton)))
            for n in "AB"}
    SR.configure("improved" if "Improved" in path else "current")
    _, wk, _ = CR.script(SR.make_parts(), rigs)
    scores, steps = E.analyse(st, rigs, wk, CR.LOADS, int(st.GetEndTimeCode()) + 1), CR.STEPS
camp = st.GetPrimAtPath(f"/World/Cameras/{cam}")
size = (960, 640)
ims = []
for t in times:
    f = int(round(t * 30))
    m = UsdGeom.Xformable(camp).ComputeLocalToWorldTransform(Usd.TimeCode(f))
    eye = m.ExtractTranslation()
    fwd = -Gf.Vec3d(m[2][0], m[2][1], m[2][2]).GetNormalized()
    if os.environ.get("RACK_EYE"):
        e, a = [Gf.Vec3d(*[float(v) for v in q.split(",")]) for q in os.environ["RACK_EYE"].split(";")]
        eye, fwd = e, (a - e).GetNormalized()
    geo = PV.collect(st, Usd.TimeCode(f), skip=("/World/Worker", "/World/Environment"))
    if "--noskin" not in args:
        geo += SK.skinned(st, f)
    p = os.path.join(out, f"frame{f:05d}.png")
    PV.render(geo, p, eye, eye + fwd * 300, size=size, fov=55)
    im = Image.open(p).convert("RGB")
    if scores:
        panel = ER.hud(scores, steps, f, (440, size[1]))
        c = Image.new("RGB", (size[0] + 440, size[1]))
        c.paste(im, (0, 0))
        c.paste(panel, (size[0], 0))
        im = c
        im.save(p)
    ims.append(im)
cols = 2
w, h = ims[0].size
sheet = Image.new("RGB", (w * cols, h * ((len(ims) + cols - 1) // cols)), (255, 255, 255))
for k, im in enumerate(ims):
    sheet.paste(im, ((k % cols) * w, (k // cols) * h))
sheet.save(os.path.join(out, "sheet.png"))
