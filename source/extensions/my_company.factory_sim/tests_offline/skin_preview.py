"""Skinned-character meshes for preview.render (linear blend skinning through UsdSkel queries)."""
import numpy as np
from pxr import Usd, UsdGeom, UsdSkel, Vt


def skinned(stage, time, skip=()):
    t = Usd.TimeCode(time)
    out = []
    cache = UsdSkel.Cache()
    xc = UsdGeom.XformCache(t)
    for prim in stage.Traverse():
        if not prim.IsA(UsdSkel.Root) or any(str(prim.GetPath()).startswith(s) for s in skip):
            continue
        root = UsdSkel.Root(prim)
        cache.Populate(root, Usd.TraverseInstanceProxies())
        for binding in cache.ComputeSkelBindings(root, Usd.TraverseInstanceProxies()):
            sq = cache.GetSkelQuery(binding.GetSkeleton())
            xforms = sq.ComputeSkinningTransforms(t)
            skel_w = np.array(xc.GetLocalToWorldTransform(binding.GetSkeleton().GetPrim()))
            for tq in binding.GetSkinningTargets():
                mp = tq.GetPrim()
                if not mp.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(mp).ComputeVisibility(t) == "invisible":
                    continue
                m = UsdGeom.Mesh(mp)
                pts = m.GetPointsAttr().Get(t)
                if not pts:
                    continue
                pts = Vt.Vec3fArray(pts)
                if tq.HasJointInfluences() and not tq.ComputeSkinnedPoints(xforms, pts, t):
                    continue
                P = np.array(pts, dtype=np.float64)
                P = (np.c_[P, np.ones(len(P))] @ skel_w)[:, :3]
                cnt, idx = m.GetFaceVertexCountsAttr().Get(), np.array(m.GetFaceVertexIndicesAttr().Get())
                tris, o = [], 0
                for c in cnt:
                    for k in range(1, c - 1):
                        tris.append((idx[o], idx[o + k], idx[o + k + 1]))
                    o += c
                name = mp.GetName().lower()
                rgb = (0.85, 0.65, 0.5)
                for key, col in (("vest", (0.95, 0.6, 0.05)), ("glove", (0.25, 0.25, 0.22)), ("pant", (0.2, 0.25, 0.35)),
                                 ("shoe", (0.1, 0.1, 0.1)), ("cap", (0.15, 0.2, 0.5)), ("casual", (0.3, 0.35, 0.45)),
                                 ("_h", (0.25, 0.18, 0.12)), ("eye", (0.9, 0.9, 0.9)), ("teeth", (0.9, 0.9, 0.9)),
                                 ("tongue", (0.6, 0.3, 0.3))):
                    if key in name:
                        rgb = col
                        break
                out.append((P, np.array(tris, dtype=np.int64), rgb))
    return out
