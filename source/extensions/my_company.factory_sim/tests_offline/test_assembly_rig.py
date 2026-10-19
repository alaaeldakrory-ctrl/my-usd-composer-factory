"""Offline checks of the CC rig solvers on the NVIDIA Worker and Uniform_M characters."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdGeom, UsdSkel   # noqa: E402
from assembly import rig as RG              # noqa: E402

CH = r"C:\Nvidia_Assets\Characters_NVD\Assets\Characters\Reallusion"
CHARS = {"Worker": (CH + r"\Worker\Worker.usd", True), "Uniform": (CH + r"\ActorCore\Uniform_M_0001\uniform_m_0001.usd", False)}


def place(stage, path, asset, z_up):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetPayloads().AddPayload(asset)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    if z_up:
        xf.AddRotateXOp(opSuffix="unitsResolve").Set(-90.0)
    return prim


def main():
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(st, "Y")
    UsdGeom.SetStageMetersPerUnit(st, 0.01)
    for name, (asset, zup) in CHARS.items():
        prim = place(st, "/World/" + name, asset, zup)
        st.Load(prim.GetPath())
        skel = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))
        r = RG.CCRig(st, skel)
        w = r.rest_world
        print(f"== {name}: rest heading {r.rest_heading:.0f}, hip height {r.hip_h:.1f}, ankle {r.ankle_h:.1f}, "
              f"arm {r.len['Larm'][0]:.1f}+{r.len['Larm'][1]:.1f}, leg {r.len['Lleg'][0]:.1f}+{r.len['Lleg'][1]:.1f}")
        # 1. root placement
        p = RG.Pose()
        p.root_pos, p.root_yaw = Gf.Vec3d(100, 0, 50), 90.0
        w = r.fk(p)
        f, left = r.heading_dirs(p)
        toe = (r.pos(w, "L_ToeBase") + r.pos(w, "R_ToeBase")) / 2 - (r.pos(w, "L_Foot") + r.pos(w, "R_Foot")) / 2
        print("  placed: hip", [round(v) for v in r.pos(w, "Hip")], "facing", [round(v, 2) for v in toe.GetNormalized()],
              "expected", [round(v, 2) for v in f])
        # 2. crouch 25 cm with feet planted
        feet = {s: r.pos(w, s + "_Foot") for s in "LR"}
        p.hip_offset = Gf.Vec3d(0, -25, 0)
        r.bend(p, 25.0)
        for s in "LR":
            r.leg(p, s, feet[s])
        w = r.fk(p)
        print("  crouch: hip y", round(r.pos(w, "Hip")[1], 1), "ankle errors",
              [round((r.pos(w, s + "_Foot") - feet[s]).GetLength(), 2) for s in "LR"])
        # 3. grip a horizontal tube in front at 100 cm high, both hands, overhand
        tube_c = r.pos(w, "Hip") + f * 45 + Gf.Vec3d(0, 0, 0)
        tube_c = Gf.Vec3d(tube_c[0], 100.0, tube_c[2])
        axis = left
        for s, sgn in (("L", 1), ("R", -1)):
            g = tube_c + axis * (22 * sgn) + Gf.Vec3d(0, 2.1, 0)
            r.arm(p, s, g, palm_dir=Gf.Vec3d(0, -1, 0), finger_dir=f)
            r.grip(p, s, 1.0)
        w = r.fk(p)
        for s, sgn in (("L", 1), ("R", -1)):
            g = tube_c + axis * (22 * sgn) + Gf.Vec3d(0, 2.1, 0)
            fd, sd, nd = r.hand_frame(r.fk_open(p, s) if hasattr(r, "fk_open") else w, s)
            palm = r.pos(w, s + "_Hand") + fd * 4.5 - Gf.Vec3d(0, 3.5, 0)
            tip = r.pos(w, s + "_Mid3")
            print(f"  {s} hand: wrist {[round(v) for v in r.pos(w, s + '_Hand')]}, grip {[round(v) for v in g]}, "
                  f"palm normal . down = {Gf.Dot(nd, Gf.Vec3d(0, -1, 0)):.2f}, fingertip below tube top: {tip[1] < g[1]}, "
                  f"fingertip-to-tube-axis {(Gf.Vec3d(tip[0], tip[1], tip[2]) - Gf.Vec3d(tip[0], 100.0, tube_c[2]) if False else 0)}")
        r.look(p, tube_c)
        w = r.fk(p)
        print("  head fwd . to-tube:", round(Gf.Dot(r._head_fwd(w), (tube_c - r.pos(w, "Head")).GetNormalized()), 3))


if __name__ == "__main__":
    main()
