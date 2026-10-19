"""Turns the NVIDIA factory sample's in-ground vehicle lift (car_lift.usd, from SolidWorks, "Tall" configuration)
into an assembly asset: flattened meshes grouped into the parts that get installed, sunk into the floor so the
runways stand at waist height (the column disappears below the floor as in a real in-ground lift).

    python carlift_asset.py [C:/OmniverseData/CarLift/CarLift_kit.usd]

Layout (cm, Y up, long axis X, column on the origin):
    /CarLift/Fixed/Column        base plate + column (already set in the floor)
    /CarLift/Kit/Head            welded head frame (end beams + runway support beams + hub + cover plate)
    /CarLift/Kit/RunwayL|R       runways (+Z / -Z): two rails, end caps, 32 tread bars each
"""
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ifc_to_usd   # noqa: E402  (smooth_normals)
import pbr   # noqa: E402

SRC = r"C:/Nvidia_Assets/USD_Explorer_Sample_NVD/Usd_Explorer/Samples/Examples/2023_2/Factory/Source_Files/ALL_ASSETS/car_lift.usd"
OUT = r"C:/OmniverseData/CarLift/CarLift_kit.usd"
SINK = -62.0                     # runway tops end up at ~111 cm


def group_of(name, lo, hi):
    k = int(name.split("_")[1])
    if k in (142, 143, 144, 145):
        return "Fixed/Column", "lift_blue"
    if k == 140:
        return "Kit/Head", "lift_blue"
    if k == 141:
        return "Kit/Head", "steel_tread"              # centre cover plate rides with the head
    side = "RunwayL" if (lo[2] + hi[2]) > 0 else "RunwayR"
    rail = (hi[0] - lo[0]) > 400 or (hi[2] - lo[2]) > 38 or max(hi - lo) < 4    # rails / end caps / corner blocks
    return "Kit/" + side, ("lift_blue" if rail else "steel_tread")


def build(out=OUT):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    src = Usd.Stage.Open(SRC)
    xc = UsdGeom.XformCache()
    if os.path.exists(out):
        os.remove(out)
    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(st, "Y")
    UsdGeom.SetStageMetersPerUnit(st, 0.01)
    root = UsdGeom.Xform.Define(st, "/CarLift")
    st.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(st, "/CarLift/Looks")
    looks = {}
    count = {}
    for p in Usd.PrimRange(src.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh) or not p.IsInstanceProxy():       # the real-size instance only
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        if not pts:
            continue
        P = np.array(pts, float)
        M = np.array(xc.GetLocalToWorldTransform(p))
        W = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        W[:, 1] += SINK
        lo, hi = W.min(0), W.max(0)
        grp, look = group_of(p.GetName(), lo, hi)
        if look not in looks:
            looks[look] = pbr.material(st, f"/CarLift/Looks/{look}", look)
        parent = "/CarLift/" + grp
        for seg_path in ("/CarLift/" + grp.split("/")[0], parent):
            if not st.GetPrimAtPath(seg_path):
                UsdGeom.Xform.Define(st, seg_path)
        k = count.get(grp, 0)
        count[grp] = k + 1
        dst = UsdGeom.Mesh.Define(st, f"{parent}/{p.GetName()}")
        pts_w = [Gf.Vec3f(*v) for v in W]
        cnt = list(m.GetFaceVertexCountsAttr().Get())
        idx = list(m.GetFaceVertexIndicesAttr().Get())
        dst.CreatePointsAttr(Vt.Vec3fArray(pts_w))
        dst.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
        dst.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        dst.CreateSubdivisionSchemeAttr("none")
        dst.CreateNormalsAttr(Vt.Vec3fArray(ifc_to_usd.smooth_normals(pts_w, cnt, idx, 30.0)))
        dst.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        dst.CreateDisplayColorAttr([Gf.Vec3f(*pbr.LOOKS[look]["color"])])
        UsdShade.MaterialBindingAPI.Apply(dst.GetPrim()).Bind(looks[look])
    for g in ("Head", "RunwayL", "RunwayR"):
        UsdGeom.Xformable(st.GetPrimAtPath(f"/CarLift/Kit/{g}")).AddTransformOp().Set(Gf.Matrix4d(1))
    st.GetRootLayer().Save()
    return count


if __name__ == "__main__":
    print(build(sys.argv[1] if len(sys.argv) > 1 else OUT))
