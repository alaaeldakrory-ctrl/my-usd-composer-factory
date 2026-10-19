"""Minimal IFC (STEP) -> USD converter for faceted-B-rep models such as manufacturer BIM downloads.

Handles IFCFACETEDBREP / IFCCLOSEDSHELL / IFCFACE / IFCFACEOUTERBOUND / IFCPOLYLOOP / IFCCARTESIANPOINT
geometry with IFCSTYLEDITEM colours. One USD mesh per B-rep, millimetres Z-up -> centimetres Y-up.

    python ifc_to_usd.py input.ifc output.usd
"""
import re
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

_LINE = re.compile(r"^#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\);\s*$", re.S)


def _split_args(s):
    """Parse a STEP argument list into python values: refs as ('#', id), lists, floats, strings, enums."""
    pos = 0
    n = len(s)

    def parse():
        nonlocal pos
        out = []
        cur = None
        while pos < n:
            c = s[pos]
            if c == "(":
                pos += 1
                cur = parse()
            elif c == ")":
                pos += 1
                out.append(cur)
                return out
            elif c == ",":
                out.append(cur)
                cur = None
                pos += 1
            elif c == "'":
                j = pos + 1
                while j < n:
                    if s[j] == "'" and (j + 1 >= n or s[j + 1] != "'"):
                        break
                    j += 2 if s[j] == "'" else 1
                cur = s[pos + 1:j]
                pos = j + 1
            elif c == "#":
                j = pos + 1
                while j < n and s[j].isdigit():
                    j += 1
                cur = ("#", int(s[pos + 1:j]))
                pos = j
            elif c in " \r\n\t":
                pos += 1
            else:
                j = pos
                while j < n and s[j] not in ",()":
                    j += 1
                tok = s[pos:j].strip()
                pos = j
                if pos < n and s[pos] == "(":          # typed value: IFCREAL(1.0)
                    pos += 1
                    inner = parse()
                    cur = (tok, inner)
                else:
                    try:
                        cur = float(tok)
                    except ValueError:
                        cur = tok
        out.append(cur)
        return out

    return parse()


def read(path):
    ents = {}
    with open(path, "r", encoding="latin-1") as fh:
        buf = ""
        in_data = False
        for line in fh:
            if not in_data:
                if line.strip() == "DATA;":
                    in_data = True
                continue
            buf += line
            if not buf.rstrip().endswith(";"):
                continue
            m = _LINE.match(buf.strip())
            buf = ""
            if m:
                ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents


def smooth_normals(pts, counts, idx, crease_deg=35.0):
    """Face-varying normals: each corner averages the (area-weighted) normals of the faces around its point that are
    within crease_deg of its own face, so curved tubes shade smooth while hard edges stay sharp."""
    import math
    face_n, face_of_corner = [], []
    o = 0
    for f, c in enumerate(counts):
        nx = ny = nz = 0.0
        for k in range(c):                         # Newell's method (handles n-gons)
            a, b = pts[idx[o + k]], pts[idx[o + (k + 1) % c]]
            nx += (a[1] - b[1]) * (a[2] + b[2])
            ny += (a[2] - b[2]) * (a[0] + b[0])
            nz += (a[0] - b[0]) * (a[1] + b[1])
        face_n.append((nx, ny, nz))
        face_of_corner += [f] * c
        o += c
    unit = []
    for n in face_n:
        L = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) or 1.0
        unit.append((n[0] / L, n[1] / L, n[2] / L))
    around = {}
    for corner, p in enumerate(idx):
        around.setdefault(p, []).append(face_of_corner[corner])
    cos_t = math.cos(math.radians(crease_deg))
    out = []
    for corner, p in enumerate(idx):
        f = face_of_corner[corner]
        u = unit[f]
        sx = sy = sz = 0.0
        for g in set(around[p]):
            v = unit[g]
            if u[0] * v[0] + u[1] * v[1] + u[2] * v[2] >= cos_t:
                w = face_n[g]
                sx += w[0]
                sy += w[1]
                sz += w[2]
        L = math.sqrt(sx * sx + sy * sy + sz * sz) or 1.0
        out.append(Gf.Vec3f(sx / L, sy / L, sz / L))
    return out


def convert(ifc_path, usd_path, root="/ES2632", group_fn=None):
    """group_fn(k, rgb, lo, hi) -> sub-path under root for B-rep k (e.g. "Kit/SideL"), or "" for the root."""
    ents = read(ifc_path)
    cache = {}

    def args(i):
        if i not in cache:
            cache[i] = _split_args(ents[i][1])
        return cache[i]

    def ref(v):
        return v[1] if isinstance(v, tuple) and v and v[0] == "#" else None

    # colours: styled item -> brep
    colour = {}
    for i, (t, _) in ents.items():
        if t != "IFCSTYLEDITEM":
            continue
        a = args(i)
        item = ref(a[0])
        try:
            psa = ref(a[1][0])
            ss = ref(args(psa)[0][0])
            rend = ref(args(ss)[2][0])
            ra = args(rend)
            rgb = args(ref(ra[0]))
            transp = ra[1] if isinstance(ra[1], float) else 0.0
            lin = [((c + 0.055) / 1.055) ** 2.4 if c > 0.04045 else c / 12.92 for c in rgb[1:4]]   # sRGB -> linear
            colour[item] = (Gf.Vec3f(*lin), transp)
        except Exception:
            continue

    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, "Y")
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    top = UsdGeom.Xform.Define(stage, root)
    stage.SetDefaultPrim(top.GetPrim())
    UsdGeom.Scope.Define(stage, root + "/Looks")
    mats = {}

    def material(rgb, transp):
        key = (round(rgb[0], 3), round(rgb[1], 3), round(rgb[2], 3), round(transp, 2))
        if key not in mats:
            p = f"{root}/Looks/M{len(mats)}"
            m = UsdShade.Material.Define(stage, p)
            sh = UsdShade.Shader.Define(stage, p + "/PBR")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
            if transp > 0:
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0 - transp)
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            mats[key] = m
        return mats[key]

    pt_cache = {}

    def point(i):
        if i not in pt_cache:
            c = args(i)[0]
            x, y, z = (c + [0.0, 0.0, 0.0])[:3]
            pt_cache[i] = Gf.Vec3f(x / 10.0, z / 10.0, -y / 10.0)     # mm Z-up -> cm Y-up
        return pt_cache[i]

    breps = [i for i, (t, _) in ents.items() if t == "IFCFACETEDBREP"]
    meshes = []
    for k, b in enumerate(sorted(breps)):
        shell = ref(args(b)[0])
        faces = args(shell)[0]
        pts, counts, idx = [], [], []
        index = {}
        for fr in faces:
            fa = args(ref(fr))
            for br in fa[0]:
                bt = ents[ref(br)][0]
                ba = args(ref(br))
                loop = args(ref(ba[0]))[0]
                orient = ba[1] if len(ba) > 1 else ".T."
                ids = [ref(p) for p in loop]
                if orient == ".F.":
                    ids.reverse()
                if bt == "IFCFACEBOUND":          # holes: skipped (none in typical BIM exports)
                    continue
                for pid in ids:
                    if pid not in index:
                        index[pid] = len(pts)
                        pts.append(point(pid))
                    idx.append(index[pid])
                counts.append(len(ids))
        if not counts:
            continue
        rgb, tr = colour.get(b, (Gf.Vec3f(0.6, 0.6, 0.6), 0.0))
        lo = Gf.Vec3f(*[min(p[a] for p in pts) for a in range(3)])
        hi = Gf.Vec3f(*[max(p[a] for p in pts) for a in range(3)])
        meshes.append((k, pts, counts, idx, rgb, tr, lo, hi))
    info = []
    for k, pts, counts, idx, rgb, tr, lo, hi in meshes:
        sub = group_fn(k, rgb, lo, hi) if group_fn else ""
        parent = root
        for seg in [x for x in sub.split("/") if x]:
            parent += "/" + seg
            if not stage.GetPrimAtPath(parent):
                UsdGeom.Xform.Define(stage, parent)
        mesh = UsdGeom.Mesh.Define(stage, f"{parent}/Brep_{k:04d}")
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateNormalsAttr(Vt.Vec3fArray(smooth_normals(pts, counts, idx)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        mesh.CreateDisplayColorAttr(Vt.Vec3fArray([rgb]))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(rgb, tr))
        info.append((k, tuple(round(v, 2) for v in rgb), lo, hi, len(counts), sub))
    stage.GetRootLayer().Save()
    return info


if __name__ == "__main__":
    info = convert(sys.argv[1], sys.argv[2])
    print(f"{len(info)} meshes written to {sys.argv[2]}")
