"""Pose lab: pose one character with the rig solvers and render close-ups offline (no Kit needed).

    python pose_lab.py <case> [out_dir]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
sys.path.insert(0, HERE)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt   # noqa: E402
from assembly import rig as RG   # noqa: E402
import preview as PV   # noqa: E402
import skin_preview as SK   # noqa: E402

CH = r"C:\Nvidia_Assets\Characters_NVD\Assets\Characters\Reallusion\ActorCore"
CHARS = {"M": CH + r"\Uniform_M_0001\uniform_m_0001.usd", "F": CH + r"\Uniform_F_0001\uniform_f_0001.usd"}
UP = Gf.Vec3d(0, 1, 0)


def make_stage(who="M"):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, "Y")
    p = st.DefinePrim("/W", "Xform")
    p.GetReferences().AddReference(CHARS[who])
    sk = next(q for q in st.Traverse() if q.IsA(UsdSkel.Skeleton))
    rig = RG.CCRig(st, sk)
    anim = UsdSkel.Animation.Define(st, "/Anim")
    anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
    UsdSkel.BindingAPI.Apply(sk).CreateAnimationSourceRel().SetTargets(["/Anim"])
    return st, rig, anim


def write(anim, rig, pose, w, frame=0):
    tr, ro, sc = rig.arrays(pose, w)
    anim.CreateTranslationsAttr().Set(tr, frame)
    anim.CreateRotationsAttr().Set(ro, frame)
    anim.CreateScalesAttr().Set(sc, frame)


def tube_mesh(a, b, r, rgb=(0.9, 0.35, 0.05), n=12):
    import numpy as np
    a, b = np.array(a, float), np.array(b, float)
    d = b - a
    d /= np.linalg.norm(d)
    u = np.cross(d, [1, 0, 0] if abs(d[0]) < 0.9 else [0, 1, 0])
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    pts, tris = [], []
    for k in range(n):
        ang = 2 * np.pi * k / n
        o = (np.cos(ang) * u + np.sin(ang) * v) * r
        pts += [a + o, b + o]
    for k in range(n):
        i, j = 2 * k, 2 * ((k + 1) % n)
        tris += [(i, j, i + 1), (j, j + 1, i + 1)]
    return np.array(pts), np.array(tris), rgb


def solve_basic(rig, pose, specs, look=None, bend=0.0, crouch=0.0):
    w = rig.fk(pose)
    f, l = rig.heading_dirs(pose)
    rig.set_hip(pose, w, Gf.Vec3d(0, -crouch, 0) - f * RG.hip_back(bend))
    rig.bend(pose, w, bend)
    for s in "LR":
        fo, le = rig.ankle_off[s]
        h = rig.rest_heading + pose.root_yaw
        a = pose.root_pos + RG.fwd_of(h) * fo + RG.left_of(h) * le + UP * rig.ankle_h
        rig.leg(pose, w, s, a, h, toe_up=0.0)
    errs = {}
    for s, spec in specs.items():
        errs[s] = rig.arm(pose, w, s, spec)
    if look is not None:
        rig.look(pose, w, look, 0.8)
    return w, errs


if __name__ == "__main__":
    case = sys.argv[1] if len(sys.argv) > 1 else "post"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "_lab")
    os.makedirs(out, exist_ok=True)
    who = os.environ.get("WHO", "M")
    st, rig, anim = make_stage(who)
    pose = RG.Pose()
    pose.root_yaw = 0.0 - rig.rest_heading         # face +Z
    extra = []
    sh = rig.shoulder_h
    if case == "post":           # both hands on one vertical post in front (thumb up)
        c = Gf.Vec3d(0, 0, 32)
        specs = {"R": {"kind": "tube", "c": c + UP * (sh - 30), "axis": UP, "thumb": UP, "r": 1.6, "curl": 1.0},
                 "L": {"kind": "tube", "c": c + UP * (sh - 48), "axis": UP, "thumb": UP, "r": 1.6, "curl": 1.0}}
        extra.append(tube_mesh((0, 40, 32), (0, 200, 32), 1.6))
        cams = [((60, sh - 20, 80), (0, sh - 40, 25)), ((-50, sh - 30, 70), (0, sh - 40, 25)), ((0, sh, 120), (0, sh - 40, 20))]
    elif case == "bar":          # overhand on a horizontal bar
        y = sh - 38
        specs = {s: {"kind": "tube", "c": Gf.Vec3d(k * 19, y, 26), "axis": Gf.Vec3d(1, 0, 0), "thumb": Gf.Vec3d(-k, 0, 0),
                     "r": 1.6, "curl": 1.0} for s, k in (("L", 1), ("R", -1))}
        extra.append(tube_mesh((-60, y, 26), (60, y, 26), 1.6))
        cams = [((60, y + 40, 90), (0, y, 30)), ((0, y + 70, 110), (0, y, 30)), ((-40, y - 20, 90), (0, y, 30))]
    else:
        raise SystemExit("unknown case")
    w, errs = solve_basic(rig, pose, specs)
    print("wrist errors", {k: round(v, 2) for k, v in errs.items()})
    write(anim, rig, pose, w)
    M = SK.skinned(st, 0) + extra
    cams = []
    for s in "LR":
        hp = rig.pos(w, s + "_Hand") + (rig.pos(w, s + "_Mid1") - rig.pos(w, s + "_Hand")) * 0.6
        f, l = rig.heading_dirs(pose)
        side = l if s == "L" else -l
        cams += [(hp + side * 30 + UP * 12 + f * 20, hp), (hp - side * 25 + f * 30 + UP * 8, hp), (hp + UP * 35 + f * 8, hp)]
    for k, (eye, at) in enumerate(cams):
        PV.render(M, os.path.join(out, f"_c{k}.png"), eye, at, size=(420, 360), fov=40)
    PV.render(M, os.path.join(out, "_full.png"), (180, 120, 260), (0, 90, 0), size=(420, 720), fov=35)
    from PIL import Image
    sheet = Image.new("RGB", (420 * 3 + 420, 720))
    for k in range(6):
        sheet.paste(Image.open(os.path.join(out, f"_c{k}.png")), ((k % 3) * 420, (k // 3) * 360))
    sheet.paste(Image.open(os.path.join(out, "_full.png")), (1260, 0))
    sheet.save(os.path.join(out, f"{case}_{who}.png"))
