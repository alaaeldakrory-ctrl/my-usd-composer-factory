"""3D ergonomics visuals written into the demo stage (all under /World/Ergo, on top of ergo.write_markers):

    skeleton     glowing "X-ray" bones per worker, each segment coloured by its own load: back by RULA trunk and the
                 L5/S1 compression (green < 2000 N, yellow < 3400 N NIOSH action limit, red above), neck, upper /
                 lower arms by the upper-arm score, legs by the legs score. One mesh per worker, time-sampled points
                 and per-face colours (UsdPreviewSurface reading displayColor, emissive).
    load arrows  an arrow hanging from the hands while something is carried, its length set by the kilograms.
    floor heat   where the risky work happened: a heat map on the floor of time x (RULA - 4), revealed at the end.
    x-ray look   variant set "ergoView" on /World/Ergo: "markers" (default) or "xray" (bodies see-through, bones on).
"""
import math
import os

import numpy as np
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

from . import ergo as E

UP = Gf.Vec3d(0, 1, 0)
LEVEL = {0: (0.12, 0.85, 0.3), 1: (1.0, 0.8, 0.05), 2: (1.0, 0.12, 0.08)}
NEUTRAL = (0.35, 0.75, 1.0)
BONES = [   # (from, to, colour source)
    ("Hip", "Waist", "back"), ("Waist", "Spine01", "back"), ("Spine01", "Spine02", "back"),
    ("Spine02", "NeckTwist01", "neck"), ("NeckTwist01", "Head", "neck"),
    ("L_Thigh", "R_Thigh", "frame"), ("L_Upperarm", "R_Upperarm", "frame"),
] + [b for s in "LR" for b in (
    (f"{s}_Clavicle", f"{s}_Upperarm", f"upper_arm_{s}"), (f"{s}_Upperarm", f"{s}_Forearm", f"upper_arm_{s}"),
    (f"{s}_Forearm", f"{s}_Hand", f"upper_arm_{s}"), (f"{s}_Thigh", f"{s}_Calf", "legs"),
    (f"{s}_Calf", f"{s}_Foot", "legs"), (f"{s}_Foot", f"{s}_ToeBase", "legs"))]
SIDES = 6


def _vertex_colour_material(stage, path, emissive=1.0, opacity=1.0):
    """UsdPreviewSurface whose colour (and glow) is the mesh's displayColor primvar."""
    mat = UsdShade.Material.Define(stage, path)
    rd = UsdShade.Shader.Define(stage, path + "/Color")
    rd.CreateIdAttr("UsdPrimvarReader_float3")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
    rd_out = rd.CreateOutput("result", Sdf.ValueTypeNames.Float3)
    sh = UsdShade.Shader.Define(stage, path + "/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rd_out)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rd_out)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _prism(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax = b - a
    L = np.linalg.norm(ax)
    ax = ax / L if L > 1e-6 else np.array([0, 1, 0.0])
    t = np.cross(ax, [0, 1, 0] if abs(ax[1]) < 0.9 else [1, 0, 0])
    t /= np.linalg.norm(t)
    u = np.cross(ax, t)
    ring = [r * (math.cos(2 * math.pi * k / SIDES) * t + math.sin(2 * math.pi * k / SIDES) * u) for k in range(SIDES)]
    tip = 0.6 * r
    return [a - ax * tip] + [a + q for q in ring] + [b + q for q in ring] + [b + ax * tip]


def _prism_topology(n_bones):
    cnt, idx = [], []
    n = 2 * SIDES + 2
    for k in range(n_bones):
        o = k * n
        for i in range(SIDES):
            j = (i + 1) % SIDES
            cnt.append(3)
            idx += [o, o + 1 + j, o + 1 + i]
            cnt.append(4)
            idx += [o + 1 + i, o + 1 + j, o + 1 + SIDES + j, o + 1 + SIDES + i]
            cnt.append(3)
            idx += [o + 1 + SIDES + i, o + 1 + SIDES + j, o + n - 1]
    return cnt, idx


def _level(source, sm, k, rows):
    if source == "frame":
        return None
    if source == "back":
        lv = E._part_level("trunk", sm["trunk"][k])
        n = rows[k]["l5s1_N"]
        return max(lv, 2 if n >= E.NIOSH_AL else (1 if n >= 2000 else 0))
    if source.startswith("upper_arm"):
        return E._part_level("upper_arm", sm[source][k])
    return E._part_level(source, sm[source][k])


def write_skeleton(stage, scores, root="/World/Ergo", r=1.5):
    mat = _vertex_colour_material(stage, f"{root}/Looks/Bones")
    cnt, idx = _prism_topology(len(BONES))
    per_bone_faces = 3 * SIDES
    for who, rows in scores.items():
        sm = {k: E.smooth_scores(rows, k) for k in ("trunk", "neck", "legs", "upper_arm_L", "upper_arm_R")}
        m = UsdGeom.Mesh.Define(stage, f"{root}/{who}/Skeleton")
        m.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr("none")
        pts_attr = m.CreatePointsAttr()
        col = UsdGeom.PrimvarsAPI(m).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.uniform)
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        last = None
        for k, row in enumerate(rows):
            J = row["joints"]
            pts = []
            for a, b, _ in BONES:
                pts += _prism(J[a], J[b], r * (0.7 if a.endswith("Thigh") and b.endswith("Thigh") else 1.0))
            pts_attr.Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]), row["frame"])
            levels = tuple(_level(src, sm, k, rows) for _, _, src in BONES)
            if levels != last:
                cols = []
                for lv in levels:
                    cols += [Gf.Vec3f(*(NEUTRAL if lv is None else LEVEL[lv]))] * per_bone_faces
                col.Set(Vt.Vec3fArray(cols), row["frame"])
                last = levels


def write_load_arrows(stage, scores, root="/World/Ergo", cm_per_kg=3.2):
    """A downward arrow from the middle of the hands while carrying, length ~ kg (the worker's share)."""
    mat = _vertex_colour_material(stage, f"{root}/Looks/Arrows")
    for who, rows in scores.items():
        m = UsdGeom.Mesh.Define(stage, f"{root}/{who}/LoadArrow")
        cnt, idx = _prism_topology(2)
        m.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        m.CreateSubdivisionSchemeAttr("none")
        pts_attr = m.CreatePointsAttr()
        vis = m.CreateVisibilityAttr()
        col = UsdGeom.PrimvarsAPI(m).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
        col.Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.55, 0.1)]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        shown = None
        for row in rows:
            f, kg = row["frame"], row["kg"]
            on = kg >= 2.0
            if on:
                J = row["joints"]
                top = (J["L_Hand"] + J["R_Hand"]) * 0.5 - UP * 4.0
                L = kg * cm_per_kg
                bot = top - UP * L
                head = _prism(bot + UP * 6.0, bot, 4.5)
                head = [head[0]] + [bot + UP * 6.0 + (q - (bot + UP * 6.0)) for q in head[1:SIDES + 1]] + \
                       [bot + (q - bot) * 0.05 for q in head[SIDES + 1:2 * SIDES + 1]] + [bot - UP * 0.5]
                pts = _prism(top, bot + UP * 6.0, 1.2) + head
                pts_attr.Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]), f)
            if on != shown:
                vis.Set("inherited" if on else "invisible", f)
                shown = on


def floor_heat(scores, extent=((-600.0, 600.0), (-520.0, 520.0)), cell=5.0, sigma=28.0):
    """(heat grid [z, x] normalised 0..1, extent): time spent at RULA >= 5, weighted (score - 4), splatted at the
    worker's feet."""
    (x0, x1), (z0, z1) = extent
    nx, nz = int((x1 - x0) / cell), int((z1 - z0) / cell)
    H = np.zeros((nz, nx))
    k = int(3 * sigma / cell)
    g = np.exp(-((np.arange(-k, k + 1) * cell) ** 2) / (2 * sigma ** 2))
    ker = np.outer(g, g)
    for who, rows in scores.items():
        sm = E.smooth_scores(rows)
        dt = (rows[1]["t"] - rows[0]["t"]) if len(rows) > 1 else 1 / E.FPS
        for row, s in zip(rows, sm):
            w = (s - 4) * dt
            if w <= 0:
                continue
            J = row["joints"]
            p = (J["L_Foot"] + J["R_Foot"]) * 0.5
            ix, iz = int((p[0] - x0) / cell), int((p[2] - z0) / cell)
            if not (k <= ix < nx - k and k <= iz < nz - k):
                continue
            H[iz - k:iz + k + 1, ix - k:ix + k + 1] += w * ker
    total = H.sum() * 0 + H.max()
    return (H / total if total > 0 else H), extent


def heat_rgba(H):
    """Transparent -> yellow -> orange -> red, alpha with the heat."""
    c = np.clip(H, 0, 1)
    stops = np.array([[255, 230, 60], [255, 140, 30], [225, 30, 25]], float)
    x = c * 2
    lo = np.clip(np.floor(x).astype(int), 0, 1)
    u = (x - lo)[..., None]
    rgb = stops[lo] * (1 - u) + stops[lo + 1] * u
    a = np.clip(c ** 0.6 * 1.25, 0, 0.85)
    a[c < 0.04] = 0
    return np.dstack([rgb, a * 255]).astype(np.uint8)


def write_floor_heat(stage, scores, out_dir, reveal_t, root="/World/Ergo", cell=8.0, floor_y=0.35):
    """The heat as floor geometry: a quad per grid cell with heat, coloured per vertex (no transparency needed, which
    RTX real-time ignores on this material), revealed at reveal_t. Also writes the heat images for the report."""
    H, ((x0, x1), (z0, z1)) = floor_heat(scores)
    rgba = heat_rgba(H)
    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(rgba[::-1], "RGBA").save(os.path.join(out_dir, "floor_heat_rgba.png"))
    nz, nx = H.shape
    step = max(1, int(round(cell / ((x1 - x0) / nx))))
    Hc = H[::step, ::step]
    cz, cx = Hc.shape
    dx, dz = (x1 - x0) / cx, (z1 - z0) / cz
    stops = np.array([[1.0, 0.86, 0.2], [1.0, 0.5, 0.08], [0.85, 0.08, 0.06]])

    def colour(v):
        x = min(max(v, 0.0), 1.0) * 2
        k = min(int(x), 1)
        u = x - k
        return stops[k] * (1 - u) + stops[k + 1] * u
    vid, pts, cols = {}, [], []

    def vert(i, j):
        key = (i, j)
        if key not in vid:
            vid[key] = len(pts)
            pts.append(Gf.Vec3f(x0 + j * dx, floor_y, z0 + i * dz))
            ii, jj = min(i, cz - 1), min(j, cx - 1)
            cols.append(Gf.Vec3f(*colour(float(Hc[ii, jj]) ** 0.7)))
        return vid[key]
    idx = []
    thr = 0.06
    for i in range(cz):
        for j in range(cx):
            if Hc[i, j] >= thr:
                idx += [vert(i, j), vert(i + 1, j), vert(i + 1, j + 1), vert(i, j + 1)]
    m = UsdGeom.Mesh.Define(stage, f"{root}/FloorHeat")
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (len(idx) // 4)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 1, 0)] * len(pts)))
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    UsdGeom.PrimvarsAPI(m).CreatePrimvar("displayColor", Sdf.ValueTypeNames.Color3fArray,
                                         UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray(cols))
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(_vertex_colour_material(stage, f"{root}/Looks/FloorHeat"))
    vis = m.CreateVisibilityAttr()
    fr = int(reveal_t * E.FPS)
    vis.Set("invisible", 0)
    vis.Set("invisible", max(0, fr - 1))
    vis.Set("inherited", fr)
    return H


def add_view_variants(stage, worker_paths, root="/World/Ergo"):
    """Variant set ergoView on /World/Ergo: 'markers' (bodies normal, dots + halos) or 'xray' (bodies 30 % opaque,
    glowing bones, dots hidden)."""
    prim = stage.GetPrimAtPath(root)
    vs = prim.GetVariantSets().AddVariantSet("ergoView")
    for v in ("markers", "xray"):
        vs.AddVariant(v)
    vs.SetVariantSelection("xray")
    with vs.GetVariantEditContext():
        for who in ("A", "B"):
            for mk in E.MARKERS:
                stage.OverridePrim(f"{root}/{who}/{mk}").CreateAttribute("visibility", Sdf.ValueTypeNames.Token).Set("invisible")
    vs.SetVariantSelection("markers")
    with vs.GetVariantEditContext():
        for who in ("A", "B"):
            stage.OverridePrim(f"{root}/{who}/Skeleton").CreateAttribute("visibility", Sdf.ValueTypeNames.Token).Set("invisible")
    vs.SetVariantSelection("markers")
    # the see-through bodies follow the variant through a small switch prim the shader overrides read
    for wp in worker_paths:
        for p in Usd.PrimRange(stage.GetPrimAtPath(wp)):
            if p.IsA(UsdShade.Shader) and p.GetParent().GetName() == "Character_0":
                XRAY_SHADERS.append(str(p.GetPath()))


XRAY_SHADERS = []


def write_xray_layer(stage_path, out_path):
    """A small layer that, sublayered over the demo file, makes the bodies see-through (opacity 0.3) and switches
    /World/Ergo to the xray variant. Kept separate because character shader overrides cannot live inside a variant
    on /World/Ergo."""
    lay = Sdf.Layer.CreateNew(out_path) if not os.path.exists(out_path) else Sdf.Layer.FindOrOpen(out_path)
    lay.Clear()
    lay.subLayerPaths.append(stage_path.replace("\\", "/"))
    st = Usd.Stage.Open(lay)
    st.SetEditTarget(Usd.EditTarget(lay))
    for sp in XRAY_SHADERS:
        sh = UsdShade.Shader(st.OverridePrim(sp))
        sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(False)
        sh.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(0.28)
    st.OverridePrim("/World/Ergo").GetVariantSets().AddVariantSet("ergoView").SetVariantSelection("xray")
    src = Sdf.Layer.FindOrOpen(stage_path)                  # the playback range lives on the root layer
    lay.startTimeCode, lay.endTimeCode = src.startTimeCode, src.endTimeCode
    lay.timeCodesPerSecond, lay.framesPerSecond = src.timeCodesPerSecond, src.framesPerSecond
    lay.defaultPrim = src.defaultPrim
    lay.pseudoRoot.SetInfo("upAxis", src.pseudoRoot.GetInfo("upAxis"))
    lay.pseudoRoot.SetInfo("metersPerUnit", src.pseudoRoot.GetInfo("metersPerUnit"))
    lay.Save()
    return out_path
