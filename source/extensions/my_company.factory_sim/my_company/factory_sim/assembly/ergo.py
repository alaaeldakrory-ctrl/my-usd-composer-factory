"""Ergonomics of an assembly animation: RULA (McAtamney & Corlett 1993) for every frame of every worker, from the
baked skeleton, plus what the worker handles (choreo LOADS: weight, tool use) — then markers on the body, a CSV and a
per-step summary.

Scoring uses my_company.ergo_sim's RULA tables (core/rula.py). Angles come from CC joint positions:
    trunk      Hip -> Spine02 against vertical (signed forward), side bend, twist (shoulder vs hip line)
    neck       NeckTwist01 -> Head against the trunk
    upper arm  Upperarm -> Forearm against the trunk's down axis (signed forward), abducted, shoulder raised
    lower arm  elbow flexion (60-100 deg is good), working across the midline / out to the side
    wrist      forearm vs hand direction; twist near end of range while turning a ratchet
    legs       unbalanced / deep squat (knee flexion > 60 deg)
    no task    walking or waiting with nothing in the hands is not a task posture and is capped at 2 (acceptable)
    load       per worker share of what is carried; repeated work (ratchet, hammer drill) = muscle use
Caveat: the scores describe our animated method, not measured people (good for comparing methods and spotting bad
steps), and the CC characters' arms are slightly short, which inflates trunk bend a little.
"""
import csv
import math
import os
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdSkel, Vt

def _load_rula():
    """my_company.ergo_sim's RULA tables, loaded from its file (its package imports omni.* at import time)."""
    import importlib.util
    path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
                                         "my_company.ergo_sim", "my_company", "ergo_sim", "core", "rula.py"))
    spec = importlib.util.spec_from_file_location("ergo_sim_rula", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


R = _load_rula()

UP = Gf.Vec3d(0, 1, 0)
FPS = 30.0
G = 9.81
BODY_KG = {"A": 82.0, "B": 63.0}           # worker body masses (the characters' build)
NIOSH_AL, NIOSH_MPL = 3400.0, 6400.0       # L5/S1 compression: NIOSH action limit / maximum permissible limit (N)
ERECTOR_ARM = 0.05                          # m, erector spinae moment arm about L5/S1
# risk bands of the RULA grand score / colours (sRGB)
BANDS = [(2, "acceptable", (46, 170, 80)), (4, "investigate", (240, 200, 30)), (6, "change soon", (245, 120, 30)),
         (7, "change now", (215, 40, 40))]
PART_LIMITS = {"trunk": (2, 3), "neck": (2, 3), "upper_arm": (2, 3), "legs": (1, 1)}     # green <=, yellow <=


def band(score):
    for hi, name, rgb in BANDS:
        if score <= hi:
            return name, rgb
    return BANDS[-1][1], BANDS[-1][2]


def _ang(a, b):
    c = Gf.Dot(a.GetNormalized(), b.GetNormalized())
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _active(loads, who, t):
    kg, labels = 0.0, set()
    for w, t0, t1, m, lab in loads:
        if w == who and t0 <= t <= t1:
            kg = max(kg, m)
            labels.add(lab)
    return kg, labels


def neutral(rig):
    """The rig's own rest-pose angles (wrist, neck, trunk), so a relaxed joint scores as neutral."""
    J = {rig.names[i]: Gf.Vec3d(m.ExtractTranslation()) for i, m in enumerate(rig.rest_world)}
    up_t = (J["Spine02"] - J["Hip"]).GetNormalized()
    out = {"trunk": _ang(up_t, UP), "neck": _ang(J["Head"] - J["NeckTwist01"], up_t)}
    for s in "LR":
        out["wrist_" + s] = _ang(J[s + "_Hand"] - J[s + "_Forearm"], J[s + "_Mid1"] - J[s + "_Hand"])
    return out


def spine_compression(J, body_kg, load_kg):
    """Static estimate of the L5/S1 compression (N), the usual single-equivalent-muscle model: the moment of the
    upper body (trunk + head 0.50, each arm 0.05 of body weight) and the load in the hands about the L5/S1 joint is
    held by the back muscles 5 cm behind it, plus the weight pressing along the spine."""
    l5 = J["Waist"] * 0.6 + J["Hip"] * 0.4
    trunk_com = l5 + (J["Head"] - l5) * 0.45
    parts = [(0.50 * body_kg, trunk_com)]
    for s in "LR":
        parts.append((0.05 * body_kg, (J[s + "_Upperarm"] + J[s + "_Hand"]) * 0.5))
    if load_kg > 0:
        parts.append((load_kg, (J["L_Hand"] + J["R_Hand"]) * 0.5))
    moment = 0.0
    for m, p in parts:
        d = p - l5
        moment += m * G * math.hypot(d[0], d[2]) / 100.0          # cm -> m
    axis = (J["Spine02"] - l5).GetNormalized()
    weight = sum(m for m, _ in parts) * G
    return moment / ERECTOR_ARM + weight * max(0.0, Gf.Dot(axis, UP))


def frame_score(J, heading, kg, labels, walking, rest=None):
    """RULA for one worker in one frame. J: joint name -> world position; rest: neutral() of the rig."""
    rest = rest or {"trunk": 0.0, "neck": 0.0, "wrist_L": 0.0, "wrist_R": 0.0}
    hip, sp, neck, head = J["Hip"], J["Spine02"], J["NeckTwist01"], J["Head"]
    up_t = (sp - hip).GetNormalized()
    fwd = Gf.Vec3d(math.sin(math.radians(heading)), 0, math.cos(math.radians(heading)))
    left = Gf.Vec3d(math.cos(math.radians(heading)), 0, -math.sin(math.radians(heading)))
    trunk = max(0.0, _ang(up_t, UP) - rest["trunk"]) * (1 if Gf.Dot(up_t, fwd) >= -0.05 else -1)
    trunk_side = abs(math.degrees(math.asin(max(-1, min(1, Gf.Dot(up_t, left)))))) > 12
    sh = J["R_Upperarm"] - J["L_Upperarm"]
    hp = J["R_Thigh"] - J["L_Thigh"]
    twist = abs(math.degrees(math.atan2(sh[0], sh[2]) - math.atan2(hp[0], hp[2])))
    twist = min(twist, 360 - twist) > 15
    n = head - neck
    neck_fl = max(0.0, _ang(n, up_t) - rest["neck"])
    neck_fl = neck_fl if Gf.Dot(n - up_t * Gf.Dot(n, up_t), fwd) >= 0 else -neck_fl
    fwd_t = (fwd - up_t * Gf.Dot(fwd, up_t)).GetNormalized()
    left_t = Gf.Cross(up_t, fwd_t).GetNormalized()
    knee = max(180 - _ang(J[s + "_Thigh"] - J[s + "_Calf"], J[s + "_Foot"] - J[s + "_Calf"]) for s in "LR")
    legs_ok = knee < 60                  # RULA: unbalanced = one leg / deep squat (walking normally counts as balanced)
    ratchet = "ratchet" in labels
    repeated = ratchet or "hammer drill" in labels
    sides = {}
    for s in "LR":
        u = J[s + "_Forearm"] - J[s + "_Upperarm"]
        f = J[s + "_Hand"] - J[s + "_Forearm"]
        h = J[s + "_Mid1"] - J[s + "_Hand"]
        ua = _ang(u, -up_t) * (1 if Gf.Dot(u, fwd_t) >= 0 else -1)
        abducted = abs(Gf.Dot(u.GetNormalized(), left_t)) > 0.62
        raised = J[s + "_Hand"][1] > J[s + "_Upperarm"][1] + 15.0
        elbow = 180 - _ang(J[s + "_Upperarm"] - J[s + "_Forearm"], J[s + "_Hand"] - J[s + "_Forearm"])
        out = abs(Gf.Dot(J[s + "_Hand"] - hip, left_t)) > 45.0 or Gf.Dot(J[s + "_Hand"] - hip, left_t) * (1 if s == "L" else -1) < -8.0
        wrist = max(0.0, _ang(f, h) - rest["wrist_" + s])
        res = R.rula_from_angles(ua, elbow, wrist, neck_fl, max(trunk, 0.0), mass_kg=kg, is_repeated=repeated,
                                 repeats_per_minute=10.0 if repeated else 0.0, shoulder_raised=raised,
                                 upper_arm_abducted=abducted, arm_across_midline=out, wrist_twist_near_end=ratchet,
                                 neck_twisted=False, trunk_twisted=twist, trunk_side_bent=trunk_side,
                                 legs_supported=legs_ok)
        sides[s] = (res, ua)
    worst = max(sides, key=lambda k: sides[k][0].grand_score)
    res = sides[worst][0]
    return {"score": res.grand_score, "trunk": res.trunk, "neck": res.neck, "legs": res.legs,
            "upper_arm_L": sides["L"][0].upper_arm, "upper_arm_R": sides["R"][0].upper_arm,
            "trunk_deg": trunk, "neck_deg": neck_fl, "ua_L_deg": sides["L"][1], "ua_R_deg": sides["R"][1],
            "knee_deg": knee, "kg": kg, "worst_side": worst, "force": res.force, "muscle": res.muscle_use}


def analyse(stage, rigs, workers, loads, n_frames, step=1, body_kg=None):
    """Per-frame scores {worker: [dict]} from the baked skeleton animation on `stage`."""
    cache = UsdSkel.Cache()
    out = {}
    for who, rig in rigs.items():
        prim = rig.prim
        root = prim
        while root and not root.IsA(UsdSkel.Root):
            root = root.GetParent()
        if not root:
            root = next(p for p in Usd.PrimRange(stage.GetPrimAtPath(f"/World/Worker{who}")) if p.IsA(UsdSkel.Root))
        cache.Populate(UsdSkel.Root(root), Usd.TraverseInstanceProxies())
        q = cache.GetSkelQuery(UsdSkel.Skeleton(prim))
        rows = []
        W = workers[who]
        rest = neutral(rig)
        for f in range(0, n_frames, step):
            t = f / FPS
            xf = q.ComputeJointWorldTransforms(UsdGeom.XformCache(Usd.TimeCode(f)))
            J = {rig.names[i]: Gf.Vec3d(m.ExtractTranslation()) for i, m in enumerate(xf)}
            p0, h = W.root_at(t)
            p1, _ = W.root_at(t + 0.2)
            walking = (p1 - p0).GetLength() > 4.0
            kg, labels = _active(loads, who, t)
            r = frame_score(J, h, kg, labels, walking, rest)
            hands = any(t0 - 0.2 <= t <= t1 + 0.2 for side in "LR" for (t0, t1, _) in W.holds[side])
            r["walking"] = walking and kg < 0.5
            r["task"] = hands or kg >= 0.5
            if not r["task"]:                  # RULA rates task postures: walking / waiting empty-handed is not a task
                r["score"] = min(r["score"], 2)
            r.update(frame=f, t=round(t, 3), joints=J)
            r["l5s1_N"] = spine_compression(J, (body_kg or BODY_KG).get(who, 75.0), kg)
            rows.append(r)
        out[who] = rows
    return out


def smooth_scores(rows, key="score", r=6):
    """Median over +-r frames: the displayed score does not flicker."""
    v = np.array([x[key] for x in rows], float)
    pad = np.pad(v, r, mode="edge")
    return [int(np.median(pad[i:i + 2 * r + 1])) for i in range(len(v))]


def write_csv(path, scores, steps):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["worker", "frame", "t", "step", "rula", "band", "trunk", "neck", "upper_arm_L", "upper_arm_R",
                    "legs", "trunk_deg", "neck_deg", "ua_L_deg", "ua_R_deg", "knee_deg", "load_kg", "l5s1_N"])
        for who, rows in scores.items():
            sm = smooth_scores(rows)
            for r, s in zip(rows, sm):
                st = next((lab for t0, t1, lab in steps if t0 <= r["t"] < t1), "")
                w.writerow([who, r["frame"], r["t"], st, s, band(s)[0], r["trunk"], r["neck"], r["upper_arm_L"],
                            r["upper_arm_R"], r["legs"], round(r["trunk_deg"], 1), round(r["neck_deg"], 1),
                            round(r["ua_L_deg"], 1), round(r["ua_R_deg"], 1), round(r["knee_deg"], 1), r["kg"],
                            round(r.get("l5s1_N", 0.0))])


def summary(scores, steps):
    """[(step, worker, peak score, mean score, seconds per band, dominant cause)]"""
    out = []
    for t0, t1, lab in steps:
        for who, rows in scores.items():
            sm = smooth_scores(rows)
            sel = [(r, s) for r, s in zip(rows, sm) if t0 <= r["t"] < t1]
            if not sel:
                continue
            dt = (sel[1][0]["t"] - sel[0][0]["t"]) if len(sel) > 1 else 1 / FPS
            secs = {b[1]: 0.0 for b in BANDS}
            for r, s in sel:
                secs[band(s)[0]] += dt
            worst = max(sel, key=lambda x: x[1])[0]
            causes = []
            if worst["trunk"] >= 3:
                causes.append(f"back bent {worst['trunk_deg']:.0f} deg")
            if max(worst["upper_arm_L"], worst["upper_arm_R"]) >= 3:
                causes.append(f"arm raised {max(abs(worst['ua_L_deg']), abs(worst['ua_R_deg'])):.0f} deg")
            if worst["neck"] >= 3:
                causes.append(f"neck {worst['neck_deg']:.0f} deg")
            if worst["legs"] >= 2 and worst["knee_deg"] > 60:
                causes.append(f"deep squat (knee {worst['knee_deg']:.0f} deg)")
            if worst["kg"] >= 2:
                causes.append(f"{worst['kg']:.1f} kg load")
            out.append((lab, who, max(s for _, s in sel), float(np.mean([s for _, s in sel])), secs,
                        ", ".join(causes) or "-"))
    return out


# ------------------------------------------------------------------------------------------- markers in the scene
MARKERS = {   # marker: (joint, offset to the front of the body, offset to its left (cm), which score)
    "Back": ("Spine01", -11.0, 0.0, "trunk"), "Neck": ("NeckTwist01", -8.0, 0.0, "neck"),
    "ShoulderL": ("L_Upperarm", 0.0, 7.0, "upper_arm_L"), "ShoulderR": ("R_Upperarm", 0.0, -7.0, "upper_arm_R"),
    "KneeL": ("L_Calf", 7.0, 0.0, "legs"), "KneeR": ("R_Calf", 7.0, 0.0, "legs"),
}
LEVEL_RGB = {0: (0.10, 0.75, 0.25), 1: (1.0, 0.78, 0.05), 2: (0.95, 0.12, 0.08)}


def _part_level(part, score):
    g, y = PART_LIMITS["upper_arm" if part.startswith("upper_arm") else part]
    return 0 if score <= g else (1 if score <= y else 2)


def _emissive(stage, path, rgb):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("enable_emission", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("emissive_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("emissive_intensity", Sdf.ValueTypeNames.Float).Set(2500.0)
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    pv = UsdShade.Shader.Define(stage, path + "/Preview")
    pv.CreateIdAttr("UsdPreviewSurface")
    pv.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    pv.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    mat.CreateSurfaceOutput().ConnectToSource(pv.ConnectableAPI(), "surface")
    return mat


def _ball(stage, path, r, mat, rgb):
    """A small mesh sphere (meshes also show in the offline previews)."""
    pts, idx, cnt = [], [], []
    nu, nv = 10, 6
    for j in range(nv + 1):
        th = math.pi * j / nv
        for i in range(nu):
            ph = 2 * math.pi * i / nu
            pts.append(Gf.Vec3f(r * math.sin(th) * math.cos(ph), r * math.cos(th), r * math.sin(th) * math.sin(ph)))
    for j in range(nv):
        for i in range(nu):
            a, b = j * nu + i, j * nu + (i + 1) % nu
            idx += [a, b, b + nu, a + nu]
            cnt.append(4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateSubdivisionSchemeAttr("catmullClark")
    m.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def write_markers(stage, scores, n_frames, root="/World/Ergo", r=3.6):
    """Glowing markers that ride on each worker's back, neck, shoulders and knees and turn green / yellow / red with
    that body part's RULA score, plus a halo over the head in the colour of the overall score. One child per colour
    with time-sampled visibility (renders the same in RTX and Storm)."""
    UsdGeom.Scope.Define(stage, root)
    mats = {k: _emissive(stage, f"{root}/Looks/L{k}", rgb) for k, rgb in LEVEL_RGB.items()}
    band_mats = {name: _emissive(stage, f"{root}/Looks/B{k}", tuple(c / 255 for c in rgb))
                 for k, (_, name, rgb) in enumerate(BANDS)}
    for who, rows in scores.items():
        smooth = {key: smooth_scores(rows, key) for key in ("trunk", "neck", "legs", "upper_arm_L", "upper_arm_R", "score")}
        for mk, (joint, fwd_off, side_off, key) in MARKERS.items():
            base = f"{root}/{who}/{mk}"
            xf = UsdGeom.Xform.Define(stage, base)
            op = xf.AddTranslateOp()
            vis = {}
            for lv in LEVEL_RGB:
                _ball(stage, f"{base}/L{lv}", r, mats[lv], LEVEL_RGB[lv])
                vis[lv] = UsdGeom.Imageable(stage.GetPrimAtPath(f"{base}/L{lv}")).CreateVisibilityAttr()
            last = None
            for k, row in enumerate(rows):
                f = row["frame"]
                J = row["joints"]
                hip, sp = J["Hip"], J["Spine02"]
                lft = (J["L_Thigh"] - J["R_Thigh"]).GetNormalized()
                fwd = Gf.Cross(lft, (sp - hip).GetNormalized()).GetNormalized() * -1.0
                if Gf.Dot(fwd, J["L_ToeBase"] + J["R_ToeBase"] - J["L_Foot"] - J["R_Foot"]) < 0:
                    fwd = -fwd                                     # toes point forward: settles the sign
                op.Set(Gf.Vec3d(J[joint]) + fwd * fwd_off + lft * side_off, f)
                lv = _part_level(key, smooth[key][k])
                if lv != last:
                    for l2, a in vis.items():
                        a.Set("inherited" if l2 == lv else "invisible", f)
                    last = lv
        # halo over the head: overall score
        base = f"{root}/{who}/Halo"
        xf = UsdGeom.Xform.Define(stage, base)
        op = xf.AddTranslateOp()
        vis = {}
        for k, (_, name, rgb) in enumerate(BANDS):
            _ring(stage, f"{base}/B{k}", band_mats[name], tuple(c / 255 for c in rgb))
            vis[k] = UsdGeom.Imageable(stage.GetPrimAtPath(f"{base}/B{k}")).CreateVisibilityAttr()
        last = None
        for k, row in enumerate(rows):
            f = row["frame"]
            op.Set(Gf.Vec3d(row["joints"]["Head"]) + UP * 32.0, f)
            s = smooth["score"][k]
            b = next(i for i, (hi, _, _) in enumerate(BANDS) if s <= hi or i == len(BANDS) - 1)
            if b != last:
                for k2, a in vis.items():
                    a.Set("inherited" if k2 == b else "invisible", f)
                last = b


def _ring(stage, path, mat, rgb, R=11.0, r=1.1):
    pts, idx, cnt = [], [], []
    nu, nv = 24, 6
    for i in range(nu):
        a = 2 * math.pi * i / nu
        for j in range(nv):
            b = 2 * math.pi * j / nv
            pts.append(Gf.Vec3f((R + r * math.cos(b)) * math.cos(a), r * math.sin(b), (R + r * math.cos(b)) * math.sin(a)))
    for i in range(nu):
        for j in range(nv):
            a0, a1 = i * nv + j, ((i + 1) % nu) * nv + j
            b0, b1 = i * nv + (j + 1) % nv, ((i + 1) % nu) * nv + (j + 1) % nv
            idx += [a0, a1, b1, b0]
            cnt.append(4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(cnt))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateSubdivisionSchemeAttr("catmullClark")
    m.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
