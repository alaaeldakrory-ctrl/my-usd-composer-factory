"""Turns the NVIDIA Industrial pack's two-level shelving rack (RackSmallEmpty_A2: 2.0 m bay, 1.08 m deep, 3.0 m frames)
into an assembly asset: its three meshes are split into connected pieces and regrouped into the parts that get
installed, keeping the pack's textured OmniPBR material (UDIM albedo / normal / ORM).

    python rack_asset.py [C:/OmniverseData/Rack/Rack_kit.usd]

Layout (cm, Y up; source Z up is turned so the bay runs along X and the aisle (front) is +Z):
    /Rack/Kit/FrameL|R        end frames at x -93.5 / +93.5: two uprights, three horizontals, two diagonal panels
    /Rack/Kit/BeamLowF|LowR   lower beams (front +Z / rear -Z), tops at 53
    /Rack/Kit/BeamUpF|UpR     upper beams, tops at 151 (the pack has them at 191; set one 40 cm pitch lower so both
                              workers can fit them from the floor; the uprights take beams at any pitch)
    /Rack/Kit/DeckLow|DeckUp  decks sitting in the beams' steps
The pack's own beam hardware (pins / nuts) is left out: the demo bolts the beams on itself.
"""
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

SRC = r"C:/Nvidia_Assets/Industrial_NVD/Assets/ArchVis/Industrial/Shelves/RackSmallEmpty_A2.usd"
OUT = r"C:/OmniverseData/Rack/Rack_kit.usd"
# source (x depth, y bay, z up) -> world (x bay, y up, z depth): a proper rotation (cyclic permutation)
PERM = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)      # world = src @ PERM
UP_SHIFT = -40.0                                                # upper level moved down (cm)


def components(cnt, idx, n_pts, pts):
    """Face index lists of the mesh's connected pieces (shared vertices or coincident points)."""
    par = np.arange(n_pts)

    def find(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a

    seen = {}
    for i, q in enumerate(map(tuple, np.round(pts, 2))):
        if q in seen:
            a, b = find(i), find(seen[q])
            if a != b:
                par[a] = b
        else:
            seen[q] = i
    starts = np.r_[0, np.cumsum(cnt)[:-1]]
    for s, c in zip(starts, cnt):
        r0 = find(idx[s])
        for v in idx[s + 1:s + c]:
            r = find(v)
            if r != r0:
                par[r] = r0
    groups = {}
    for f, s in enumerate(starts):
        groups.setdefault(find(idx[s]), []).append(f)
    return list(groups.values()), starts


def classify(lo, hi):
    """Part group of a piece from its world bounds, or None (hardware)."""
    size = hi - lo
    c = (lo + hi) / 2
    if size.max() < 8.0:
        return None
    if size[1] > 250 or (size[0] < 6 and size[2] > 60):           # uprights / frame bracing (thin in x)
        return "FrameL" if c[0] < 0 else "FrameR"
    level = "Low" if c[1] < 100 else "Up"
    if size[2] < 15:                                                # beams: 187 long, 10 deep
        return f"Beam{level}" + ("F" if c[2] > 0 else "R")
    return "Deck" + level


def build(out=OUT):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    src = Usd.Stage.Open(SRC)
    xc = UsdGeom.XformCache()
    if os.path.exists(out):
        os.remove(out)
    st = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(st, "Y")
    UsdGeom.SetStageMetersPerUnit(st, 0.01)
    root = UsdGeom.Xform.Define(st, "/Rack")
    st.SetDefaultPrim(root.GetPrim())
    # the pack's material, texture paths made absolute
    UsdGeom.Scope.Define(st, "/Rack/Looks")
    Sdf.CopySpec(src.GetRootLayer(), "/World/Looks/Shelf", st.GetRootLayer(), "/Rack/Looks/Shelf")
    base = os.path.dirname(SRC)
    for p in Usd.PrimRange(st.GetPrimAtPath("/Rack/Looks/Shelf")):
        if p.IsA(UsdShade.Shader):
            for inp in UsdShade.Shader(p).GetInputs():
                v = inp.Get()
                if isinstance(v, Sdf.AssetPath) and v.path.startswith("./"):
                    inp.Set(Sdf.AssetPath(os.path.normpath(os.path.join(base, v.path)).replace("\\", "/")))
    mat = UsdShade.Material(st.GetPrimAtPath("/Rack/Looks/Shelf"))
    UsdGeom.Xform.Define(st, "/Rack/Kit")
    count = {}
    for p in src.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        P = np.array(m.GetPointsAttr().Get(), float)
        M = np.array(xc.GetLocalToWorldTransform(p))
        W = (np.c_[P, np.ones(len(P))] @ M)[:, :3] @ PERM
        R3 = M[:3, :3] @ PERM
        cnt = np.array(m.GetFaceVertexCountsAttr().Get())
        idx = np.array(m.GetFaceVertexIndicesAttr().Get())
        st_pv = UsdGeom.PrimvarsAPI(p).GetPrimvar("st")
        uv = np.array(st_pv.Get(), float) if st_pv and st_pv.HasValue() else None
        uv_idx = np.array(st_pv.GetIndices()) if (uv is not None and st_pv.IsIndexed()) else None
        nrm = m.GetNormalsAttr().Get()
        N = np.array(nrm, float) @ R3 if nrm else None
        if N is not None:
            N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
        n_interp = m.GetNormalsInterpolation()
        faces_list, starts = components(cnt, idx, len(P), W)
        for faces in faces_list:
            fv = np.concatenate([np.arange(starts[f], starts[f] + cnt[f]) for f in faces])   # face-vertex slots
            verts = idx[fv]
            lo, hi = W[verts].min(0), W[verts].max(0)
            grp = classify(lo, hi)
            if grp is None:
                continue
            shift = np.array([0.0, UP_SHIFT if "Up" in grp else 0.0, 0.0])
            gpath = f"/Rack/Kit/{grp}"
            if not st.GetPrimAtPath(gpath):
                UsdGeom.Xform.Define(st, gpath).AddTransformOp().Set(Gf.Matrix4d(1))
            k = count.get(grp, 0)
            count[grp] = k + 1
            used, remap = np.unique(verts, return_inverse=True)
            dst = UsdGeom.Mesh.Define(st, f"{gpath}/Piece{k:02d}")
            dst.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*v) for v in W[used] + shift]))
            dst.CreateFaceVertexCountsAttr(Vt.IntArray([int(cnt[f]) for f in faces]))
            dst.CreateFaceVertexIndicesAttr(Vt.IntArray([int(v) for v in remap]))
            dst.CreateSubdivisionSchemeAttr("none")
            dst.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*(lo + shift)), Gf.Vec3f(*(hi + shift))]))
            if N is not None:
                nn = N[fv] if n_interp == UsdGeom.Tokens.faceVarying else N[used]
                dst.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*v) for v in nn]))
                dst.SetNormalsInterpolation(n_interp if n_interp == UsdGeom.Tokens.faceVarying else UsdGeom.Tokens.vertex)
            if uv is not None:
                uvs = uv[uv_idx[fv]] if uv_idx is not None else uv[fv]
                pv = UsdGeom.PrimvarsAPI(dst.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                                     UsdGeom.Tokens.faceVarying)
                pv.Set(Vt.Vec2fArray([Gf.Vec2f(*v) for v in uvs]))
            dst.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.2, 0.5) if grp.startswith("Frame") else
                                        Gf.Vec3f(0.85, 0.35, 0.05) if grp.startswith("Beam") else Gf.Vec3f(0.6, 0.6, 0.62)])
            UsdShade.MaterialBindingAPI.Apply(dst.GetPrim()).Bind(mat)
    st.GetRootLayer().Save()
    return count


if __name__ == "__main__":
    print(build(sys.argv[1] if len(sys.argv) > 1 else OUT))
