"""Plan the before/after rack movie (no Kit): the time remap (faster while nobody is working) and a cinematic camera
(18 mm, slow orbit round the action, sightlines checked against the warehouse), one entry per output frame.

    python movie_plan.py [out.json]
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
import numpy as np   # noqa: E402
from pxr import Gf, Usd, UsdSkel   # noqa: E402
from assembly import choreo_rack as CR, director as D, ergo as E, rig as RG, scene_rack as SR, sightline as SL   # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else r"C:\OmniverseData\RackMovie\plan.json"
FPS = 30.0
SPEED_WORK, SPEED_IDLE = 2.0, 3.2
FOCAL = 15.0                                  # mm on the default 20.955 mm aperture: ~70 deg wide
RADIUS, HEIGHT = 600.0, 255.0

SR.configure("current")
st = Usd.Stage.Open(r"C:\OmniverseData\Rack_Assembly.usd")
rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(st.GetPrimAtPath(f"/World/Worker{n}")) if p.IsA(UsdSkel.Skeleton)))
        for n in "AB"}
_, wk, end = CR.script(SR.make_parts(), rigs)
n_src = int(st.GetEndTimeCode()) + 1
scores = E.analyse(st, rigs, wk, CR.LOADS, n_src)
occ = SL.occluders(st)

# ---------------------------------------------------------------- time remap
task = np.zeros(n_src)
for who in "AB":
    for r in scores[who]:
        if r["task"]:
            task[int(r["frame"])] = 1
k = int(FPS)                                         # 1 s smoothing of the speed
speed = np.convolve(np.pad(np.where(task > 0, SPEED_WORK, SPEED_IDLE), k, mode="edge"), np.ones(2 * k + 1) / (2 * k + 1), "valid")
times = [0.0]
while times[-1] < (n_src - 1) / FPS:
    s = times[-1]
    times.append(s + speed[min(int(s * FPS), n_src - 1)] / FPS)
times[-1] = (n_src - 1) / FPS
print(f"source {end:.1f} s -> movie {len(times) / FPS:.1f} s ({len(times)} frames)")

# ---------------------------------------------------------------- camera keys
targets = CR.worker_targets(wk)


def centroid(t, w=3.0):
    pts = [wk[n].root_at(tt)[0] for n in "AB" for tt in np.linspace(max(0, t - w), min(end, t + w), 9)]
    c = sum(pts, Gf.Vec3d(0, 0, 0)) / len(pts)
    return np.array([c[0] * 0.9, 95.0, c[2] * 0.9])


def ok(eye, t):
    if abs(eye[0]) > 1350 or abs(eye[2]) > 1350 or (abs(eye[0]) < 175 and abs(eye[2]) < 125) or SL.inside(eye, occ):
        return False
    for tt in np.linspace(max(0, t - 2.5), min(end, t + 2.5), 6):
        for n in "AB":
            p = wk[n].root_at(tt)[0]
            if math.hypot(eye[0] - p[0], eye[2] - p[2]) < 190:
                return False
        if SL.blocked(eye, targets(tt), occ):
            return False
    return True


keys = []
az_prev = math.radians(-140.0)
for t in np.arange(0.0, end + 0.01, 3.0):
    tg = centroid(t)
    want = az_prev + math.radians(1.1)                # slow orbit
    best = None
    for dk in range(0, 16):
        for sgn in ((1,) if dk == 0 else (1, -1)):
            az = want + sgn * math.radians(10.0 * dk)
            for r in (RADIUS, RADIUS * 0.88, RADIUS * 1.12):
                e = tg + np.array([math.sin(az) * r, 0.0, math.cos(az) * r])
                e[1] = HEIGHT
                if ok(e, t):
                    best = (az, e)
                    break
            if best:
                break
        if best:
            break
    if best is None:
        best = (az_prev, keys[-1][1] if keys else tg + np.array([0, HEIGHT, RADIUS]))
    az_prev = best[0]
    keys.append((float(t), tuple(float(v) for v in best[1]), tuple(float(v) for v in tg)))

# per output frame: interpolate, then smooth the eye and look-at paths (1.5 s) for a steady, cinematic move
eyes, ats = [], []
for s in times:
    e, a = D.at(s, keys)
    eyes.append(e)
    ats.append(a)
eyes, ats = np.array(eyes, float), np.array(ats, float)
r = int(1.5 * FPS)
ker = np.ones(2 * r + 1) / (2 * r + 1)
sm = lambda A: np.stack([np.convolve(np.pad(A[:, i], r, mode="edge"), ker, "valid") for i in range(3)], axis=1)
eyes, ats = sm(eyes), sm(ats)
bad = [round(s, 1) for s, e in zip(times[::15], eyes[::15]) if SL.blocked(e, targets(s), occ) or SL.inside(e, occ)]
print("camera keys", len(keys), " sampled frames with a worker hidden:", len(bad), bad[:20])
cams = [list(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*e), Gf.Vec3d(*a), Gf.Vec3d(0, 1, 0)).GetInverse()[i][j]
             for i in range(4) for j in range(4)) for e, a in zip(eyes, ats)]
os.makedirs(os.path.dirname(OUT), exist_ok=True)
json.dump({"fps": FPS, "focal": FOCAL, "res": [1280, 720], "times": [round(s * FPS, 3) for s in times],
           "cams": [[round(v, 4) for v in c] for c in cams]}, open(OUT, "w"))
print("->", OUT)
