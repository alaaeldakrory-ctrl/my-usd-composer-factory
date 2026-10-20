"""Hi-vis safety vests for the two NVIDIA ActorCore workers.

Both characters share one 4K texture sheet per character (Character_0), so the vest is repainted in place on a copy of
the sheet: its UV islands are rasterised to a mask, the cloth keeps its shading (value relative to the vest's mean) but
takes a fluorescent colour, and two silver reflective bands go round the torso at fixed heights (heights come from
the vest mesh in its bind pose, interpolated across each UV triangle).

    python hivis.py            writes C:/OmniverseData/Characters/<name>_BaseColor_hivis.jpg
Scenes then point the character's shader at the copy (apply()).
"""
import os

import numpy as np
from PIL import Image
from pxr import Sdf, Usd, UsdGeom, UsdShade

ROOT = "C:/Nvidia_Assets/Characters_NVD/Assets/Characters/Reallusion/ActorCore/"
OUT = "C:/OmniverseData/Characters/"
VESTS = {   # character: (fluorescent colour sRGB, band centres as fractions of the vest's height)
    "uniform_m_0001": ((214, 232, 28), (0.22, 0.52)),     # yellow-green
    "uniform_f_0001": ((255, 112, 22), (0.22, 0.52)),     # orange
}
BAND = 0.075                 # band width as a fraction of the vest height
SILVER = np.array([206, 208, 210], float)


def _paths(name):
    folder = ROOT + name.replace("uniform_m", "Uniform_M").replace("uniform_f", "Uniform_F") + "/"
    return folder + name + ".usd", folder + f"Actor/{name}/Materials/Textures/Character_0/Character_0_BaseColor.jpg"


def _raster(uv, cnt, heights, W, H):
    """Per-pixel vest mask and bind-pose height over the vest's UV triangles."""
    mask = np.zeros((H, W), bool)
    hmap = np.zeros((H, W), np.float32)
    o = 0
    for c in cnt:
        for k in range(1, c - 1):                          # fan-triangulate
            ids = (o, o + k, o + k + 1)
            P = np.array([(uv[i][0] * W, (1 - uv[i][1]) * H) for i in ids])
            hv = np.array([heights[i] for i in ids])
            x0, y0 = np.floor(P.min(0)).astype(int) - 1
            x1, y1 = np.ceil(P.max(0)).astype(int) + 1
            x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, W - 1), min(y1, H - 1)
            if x1 <= x0 or y1 <= y0:
                continue
            xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            (ax, ay), (bx, by), (cx, cy) = P
            den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
            if abs(den) < 1e-9:
                continue
            l1 = ((by - cy) * (xs - cx) + (cx - bx) * (ys - cy)) / den
            l2 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / den
            l3 = 1 - l1 - l2
            pad = 1.5 / max(x1 - x0, y1 - y0, 1)            # a little dilation over the seams
            inside = (l1 >= -pad) & (l2 >= -pad) & (l3 >= -pad)
            sub = mask[y0:y1 + 1, x0:x1 + 1]
            hsub = hmap[y0:y1 + 1, x0:x1 + 1]
            hsub[inside] = (l1 * hv[0] + l2 * hv[1] + l3 * hv[2])[inside]
            sub[inside] = True
        o += c
    return mask, hmap


def make(name):
    usd, tex = _paths(name)
    colour, bands = VESTS[name]
    st = Usd.Stage.Open(usd)
    vest = next(p for p in st.Traverse() if p.GetName() == "Safety_vest" and p.IsA(UsdGeom.Mesh))
    mesh = UsdGeom.Mesh(vest)
    uv = np.array(UsdGeom.PrimvarsAPI(vest).GetPrimvar("st0").ComputeFlattened())
    cnt = list(mesh.GetFaceVertexCountsAttr().Get())
    idx = list(mesh.GetFaceVertexIndicesAttr().Get())
    pts = np.array(mesh.GetPointsAttr().Get(), float)
    up = int(np.argmax(pts.max(0) - pts.min(0)))       # the vest is tallest along the body's up axis
    if np.median(pts[:, up]) < 0:                          # (in its bind frame, which may lie on its side)
        pts[:, up] *= -1
    heights = pts[idx][:, up]
    img = np.array(Image.open(tex).convert("RGB"), float)
    H, W = img.shape[:2]
    mask, hmap = _raster(uv, cnt, heights, W, H)
    lo, hi = heights.min(), heights.max()
    rel = (hmap - lo) / max(hi - lo, 1e-6)
    v = img.max(axis=2)
    cloth = mask & (v > 12)
    shade = np.clip(v / max(np.median(v[cloth]), 1.0), 0.55, 1.25)
    out = img.copy()
    out[cloth] = np.clip(np.array(colour, float)[None, :] * shade[cloth][:, None], 0, 255)
    band = np.zeros_like(mask)
    for b in bands:
        band |= mask & (np.abs(rel - b) < BAND / 2)
    out[band] = np.clip(SILVER[None, :] * (0.9 + 0.12 * shade[band][:, None]), 0, 255)
    os.makedirs(OUT, exist_ok=True)
    path = OUT + f"{name}_BaseColor_hivis.jpg"
    Image.fromarray(out.astype(np.uint8)).save(path, quality=93)
    return path


def apply(stage, worker_path, name):
    """Point a referenced / payloaded character's shader at its hi-vis sheet (if built)."""
    path = OUT + f"{name}_BaseColor_hivis.jpg"
    if not os.path.exists(path):
        return False
    root = stage.GetPrimAtPath(worker_path)
    shaders = [p for p in Usd.PrimRange(root) if p.IsA(UsdShade.Shader) and p.GetParent().GetName() == "Character_0"]
    for sh in shaders:
        UsdShade.Shader(sh).CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(path))
    return bool(shaders)


if __name__ == "__main__":
    for n in VESTS:
        print(make(n))
