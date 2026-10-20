"""Turns the NVIDIA Industrial pack's two-level shelving rack (RackSmallEmpty_A2: 2.0 m bay, 1.08 m deep, 3.0 m frames)
into an assembly asset: its three meshes are split into connected pieces and regrouped into the parts that get
installed, keeping the pack's textured OmniPBR material (UDIM albedo / normal / ORM).

    python rack_asset.py [C:/OmniverseData/Rack/Rack_kit.usd]

Layout (cm, Y up; source Z up is turned so the bay runs along X and the aisle (front) is +Z):
    /Rack/Kit/FrameL|R        end frames at x -93.5 / +93.5: two uprights, three horizontals, two diagonal panels
    /Rack/Kit/BeamLowF|LowR   lower beams (front +Z / rear -Z), tops at 53
    /Rack/Kit/BeamUpF|UpR     upper beams, tops at 151 (the pack has them at 191; set one 40 cm pitch lower so both
                              workers can fit them from the floor; the uprights take beams at any pitch)
    /Rack/Kit/DeckLow|DeckUp  galvanised wire-mesh decks sitting in the beams' steps (modelled here: the pack's decks are
                              solid panels): 50 x 100 mm mesh of 5 mm wire, perimeter rods, three U support channels
The pack's own beam hardware (pins / nuts) is left out: the demo bolts the beams on itself.
Each upright gets a base plate (the pack's uprights stop flush at the floor) with an anchor hole on its outer tab;
the demo drills and anchors them to the floor.
"""
import math
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ifc_to_usd   # noqa: E402  (smooth_normals)
import pbr   # noqa: E402

SRC = r"C:/Nvidia_Assets/Industrial_NVD/Assets/ArchVis/Industrial/Shelves/RackSmallEmpty_A2.usd"
OUT = r"C:/OmniverseData/Rack/Rack_kit.usd"
# source (x depth, y bay, z up) -> world (x bay, y up, z depth): a proper rotation (cyclic permutation)
PERM = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)      # world = src @ PERM
UP_SHIFT = -40.0                                                # upper level moved down (cm)
DECK_TOP = {"Low": 50.6, "Up": 148.3}
DECK_X, DECK_Z = 87.5, 46.0                                     # deck half extents
DECK_THICK = 4.6                                                # mesh top to channel bottoms
# base plates: 8 mm steel, 25 x 26 cm, under each upright, the outer tab carrying the anchor
PLATE_X = (81.0, 106.1)                                         # |x| range
PLATE_Z = (26.0, 52.0)                                          # |z| range
PLATE_T = 0.8


def _rod(a, b, r, n=6):
    """Prism (n sides, capped) from a to b: points, counts, indices."""
    a, b = np.array(a, float), np.array(b, float)
    ax = (b - a) / np.linalg.norm(b - a)
    t = np.cross(ax, [0, 1, 0] if abs(ax[1]) < 0.9 else [1, 0, 0])
    t /= np.linalg.norm(t)
    u = np.cross(ax, t)
    ring = [r * (math.cos(2 * math.pi * k / n) * t + math.sin(2 * math.pi * k / n) * u) for k in range(n)]
    pts = [a + q for q in ring] + [b + q for q in ring]
    counts = [4] * n + [n, n]
    idx = []
    for k in range(n):
        k2 = (k + 1) % n
        idx += [k, k2, n + k2, n + k]
    idx += list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    return pts, counts, idx


def _box(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    pts = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    idx = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 2, 3, 7, 6, 1, 2, 6, 5, 0, 4, 7, 3]
    return [np.array(p, float) for p in pts], [4] * 6, idx


def mesh_from(st, path, pieces, mat, colour):
    pts, counts, idx = [], [], []
    for p, c, i in pieces:
        base = len(pts)
        pts += p
        counts += c
        idx += [base + v for v in i]
    dst = UsdGeom.Mesh.Define(st, path)
    P = [Gf.Vec3f(*v) for v in pts]
    dst.CreatePointsAttr(Vt.Vec3fArray(P))
    dst.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    dst.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    dst.CreateSubdivisionSchemeAttr("none")
    dst.CreateNormalsAttr(Vt.Vec3fArray(ifc_to_usd.smooth_normals(P, counts, idx, 50.0)))
    dst.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    dst.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    if mat:
        UsdShade.MaterialBindingAPI.Apply(dst.GetPrim()).Bind(mat)
    return dst


def base_plates(st, mat):
    for side, s in (("L", -1.0), ("R", 1.0)):
        for zs in (-1.0, 1.0):
            x0, x1 = sorted((s * PLATE_X[0], s * PLATE_X[1]))
            z0, z1 = sorted((zs * PLATE_Z[0], zs * PLATE_Z[1]))
            mesh_from(st, f"/Rack/Kit/Frame{side}/BasePlate{'F' if zs > 0 else 'R'}",
                      [_box((x0, 0.0, z0), (x1, PLATE_T, z1))], mat, (0.62, 0.63, 0.64))


def wire_deck(st, level, mat):
    """A galvanised wire deck in its installed place: top wires flush with DECK_TOP."""
    T = DECK_TOP[level]
    pieces = []
    r = 0.25
    for z in np.arange(-DECK_Z + 1.0, DECK_Z - 0.9, 5.0):                       # long wires (along X), on top
        pieces.append(_rod((-DECK_X, T - r, z), (DECK_X, T - r, z), r))
    for x in np.arange(-DECK_X + 2.5, DECK_X - 2.4, 10.0):                      # cross wires (along Z), under them
        pieces.append(_rod((x, T - 3 * r, -DECK_Z), (x, T - 3 * r, DECK_Z), r))
    for zs in (-1.0, 1.0):                                                       # perimeter rods
        pieces.append(_rod((-DECK_X, T - 0.6, zs * (DECK_Z - 0.4)), (DECK_X, T - 0.6, zs * (DECK_Z - 0.4)), 0.4, 8))
    for xs in (-1.0, 1.0):
        pieces.append(_rod((xs * (DECK_X - 0.4), T - 0.6, -DECK_Z), (xs * (DECK_X - 0.4), T - 0.6, DECK_Z), 0.4, 8))
    yb, yt = T - DECK_THICK, T - 1.0
    for x in (-58.0, 0.0, 58.0):                                                 # inverted-U support channels
        pieces.append(_box((x - 1.9, yt - 0.2, -DECK_Z), (x + 1.9, yt, DECK_Z)))
        for xs in (-1.0, 1.0):
            pieces.append(_box((x + xs * 1.9 - (0.2 if xs > 0 else 0), yb, -DECK_Z),
                               (x + xs * 1.9 + (0.0 if xs > 0 else 0.2), yt, DECK_Z)))
    pts, counts, idx = [], [], []
    for p, c, i in pieces:
        base = len(pts)
        pts += p
        counts += c
        idx += [base + v for v in i]
    gpath = f"/Rack/Kit/Deck{level}"
    UsdGeom.Xform.Define(st, gpath).AddTransformOp().Set(Gf.Matrix4d(1))
    dst = UsdGeom.Mesh.Define(st, f"{gpath}/WireDeck")
    P = [Gf.Vec3f(*v) for v in pts]
    dst.CreatePointsAttr(Vt.Vec3fArray(P))
    dst.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    dst.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    dst.CreateSubdivisionSchemeAttr("none")
    dst.CreateNormalsAttr(Vt.Vec3fArray(ifc_to_usd.smooth_normals(P, counts, idx, 50.0)))
    dst.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    dst.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.73, 0.75)])
    UsdShade.MaterialBindingAPI.Apply(dst.GetPrim()).Bind(mat)
    return len(counts)


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
            if grp is None or grp.startswith("Deck"):          # the pack's solid decks are replaced by wire decks
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
    galv = pbr.material(st, "/Rack/Looks/Galvanised", "galvanised")
    base_plates(st, galv)
    for level in ("Low", "Up"):
        count["Deck" + level] = wire_deck(st, level, galv)
    st.GetRootLayer().Save()
    return count


if __name__ == "__main__":
    print(build(sys.argv[1] if len(sys.argv) > 1 else OUT))
