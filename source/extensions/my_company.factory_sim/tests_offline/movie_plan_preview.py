"""Offline look at the movie plan's camera (no environment, quick z-buffer preview) at a few output frames."""
import json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from PIL import Image
from pxr import Gf, Usd
import preview as PV, skin_preview as SK
plan = json.load(open(r"C:\OmniverseData\RackMovie\plan.json"))
out = sys.argv[1]
os.makedirs(out, exist_ok=True)
st = Usd.Stage.Open(sys.argv[2] if len(sys.argv) > 2 else r"C:\OmniverseData\Rack_Assembly.usd")
picks = [int(len(plan["times"]) * u) for u in (0.03, 0.15, 0.3, 0.42, 0.55, 0.7, 0.85, 0.97)]
fov = math.degrees(2 * math.atan(20.955 / 2 / plan["focal"]))      # horizontal, Kit default aperture
ims = []
for k in picks:
    tc = plan["times"][k]
    m = Gf.Matrix4d(*plan["cams"][k])
    eye = m.ExtractTranslation()
    fwd = -Gf.Vec3d(m[2][0], m[2][1], m[2][2])
    geo = PV.collect(st, Usd.TimeCode(tc), skip=("/World/Worker", "/World/Environment")) + SK.skinned(st, tc)
    p = os.path.join(out, f"p{k:05d}.png")
    PV.render(geo, p, eye, eye + fwd * 300, size=(640, 360), fov=fov)
    ims.append(Image.open(p))
sheet = Image.new("RGB", (1280, 360 * 4))
for i, im in enumerate(ims):
    sheet.paste(im, ((i % 2) * 640, (i // 2) * 360))
sheet.save(os.path.join(out, "sheet.png"))
