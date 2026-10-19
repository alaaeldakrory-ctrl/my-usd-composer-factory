"""A director camera for the assembly demo: smooth Catmull-Rom moves between hand-picked framings of each phase."""
from pxr import Gf, UsdGeom

# (seconds, eye, look-at) — follows the script in choreo.py
KEYS = [
    (0.0, (430, 270, 640), (0, 100, 160)),
    (4.0, (-60, 165, 560), (-20, 75, 240)),
    (9.0, (60, 190, 480), (0, 105, 150)),
    (14.0, (-10, 195, 340), (0, 130, 40)),
    (18.5, (-300, 175, 330), (-60, 120, 60)),
    (22.0, (-540, 230, 220), (-160, 100, 0)),
    (26.5, (-500, 230, -260), (-110, 100, -150)),
    (30.5, (-60, 165, -560), (-20, 75, -240)),
    (36.0, (60, 190, -480), (0, 105, -150)),
    (41.0, (0, 195, -340), (0, 130, -40)),
    (45.0, (300, 175, -330), (60, 120, -60)),
    (50.0, (-700, 260, -330), (-330, 90, -150)),       # 3: extension U-frame
    (59.0, (-560, 210, 120), (-330, 80, -110)),
    (65.0, (-470, 230, 300), (-230, 110, 0)),
    (71.0, (-430, 230, 240), (-150, 140, 0)),
    (78.0, (-330, 190, 150), (-125, 125, 0)),
    (86.0, (420, 240, -330), (200, 110, 0)),            # 4: crossbar hand-over and fit
    (95.0, (430, 220, 250), (175, 130, 0)),
    (100.0, (380, 230, 230), (160, 150, 0)),
    (105.0, (330, 270, 180), (130, 190, 0)),
    (110.0, (290, 280, -170), (110, 210, 0)),
    (116.0, (430, 210, -260), (170, 90, -40)),          # stand moved, gate, controller
    (122.0, (300, 200, -450), (0, 100, -120)),
    (127.0, (420, 200, 200), (130, 120, 0)),
    (132.0, (330, 190, 120), (115, 125, 0)),
    (136.5, (-330, 230, -380), (-90, 170, -70)),
    (141.0, (-360, 240, -330), (-90, 160, -60)),
    (148.0, (-430, 265, 330), (-60, 110, 40)),
    (153.5, (640, 330, 330), (0, 115, 0)),
]


def _cr(p0, p1, p2, p3, u):
    u2, u3 = u * u, u * u * u
    return [0.5 * ((2 * b) + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u2 + (-a + 3 * b - 3 * c + d) * u3)
            for a, b, c, d in zip(p0, p1, p2, p3)]


def at(t, keys=None):
    k = keys or KEYS
    if t <= k[0][0]:
        return k[0][1], k[0][2]
    if t >= k[-1][0]:
        return k[-1][1], k[-1][2]
    i = max(j for j in range(len(k) - 1) if k[j][0] <= t)
    u = (t - k[i][0]) / (k[i + 1][0] - k[i][0])
    u = u * u * (3 - 2 * u) * 0.35 + u * 0.65            # linger a little on each framing
    q = [k[max(0, i - 1)], k[i], k[i + 1], k[min(len(k) - 1, i + 2)]]
    return _cr(*(x[1] for x in q), u), _cr(*(x[2] for x in q), u)


def write(stage, n_frames, fps=30.0, path="/World/Cameras/Director", keys=None):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 100000.0))
    op = UsdGeom.Xformable(cam).AddTransformOp()
    for f in range(n_frames):
        eye, look = at(f / fps, keys)
        op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*look), Gf.Vec3d(0, 1, 0)).GetInverse(), f)
    return path
