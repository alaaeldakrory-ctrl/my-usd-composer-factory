"""Procedural reach animation for a Mixamo-style skeleton (T-pose rest, Y up, facing +Z, centimeters).

Given where the hands must be (and optionally where to look), the solver finds the SMALLEST forward trunk lean
that lets the arms reach comfortably, solves each arm with two-bone IK, tips the neck/head toward the work and
returns joint rotations. Legs and feet stay planted. The result can be written out as a SkelAnimation.

Targets are given in the operator's frame in METERS as (right, up, forward) measured from the floor point
between the feet; `to_skel_cm` converts them. Everything here needs only `pxr`.
"""
import math

from pxr import Gf, Usd, UsdSkel, Vt

try:
    from . import pose as P, skeleton as SK
except (ImportError, ValueError):
    import pose as P
    import skeleton as SK

X_AXIS = Gf.Vec3d(1.0, 0.0, 0.0)
DOWN = Gf.Vec3d(0.0, -1.0, 0.0)


def to_skel_cm(right, up, forward):
    """Operator frame (meters: right, up, forward) -> skeleton space (cm). The rig faces +Z, so its right is -X."""
    return Gf.Vec3d(-right * 100.0, up * 100.0, forward * 100.0)


def two_bone_ik(shoulder, target, l1, l2, pole):
    """Elbow and wrist positions for a two-bone arm. Returns (elbow, wrist, reached)."""
    d_vec = target - shoulder
    d = d_vec.GetLength()
    u = d_vec / d if d > 1e-9 else Gf.Vec3d(0, 0, 1)
    d_c = min(max(d, abs(l1 - l2) + 1e-3), l1 + l2 - 1e-3)
    a = (l1 * l1 - l2 * l2 + d_c * d_c) / (2.0 * d_c)
    h = math.sqrt(max(l1 * l1 - a * a, 0.0))
    perp = pole - u * Gf.Dot(pole, u)
    perp = perp.GetNormalized() if perp.GetLength() > 1e-9 else Gf.Vec3d(0, -1, 0)
    elbow = shoulder + u * a + perp * h
    wrist = shoulder + u * d_c
    return elbow, wrist, d <= l1 + l2 - 1e-3


class Overrides(dict):
    """Joint index -> local Gf.Rotation. `.trans` optionally maps joint index -> local translation (cm)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.trans = {}


class Rig:
    """Rest data and forward kinematics of a skeleton."""

    def __init__(self, skeleton_prim, meters_per_unit=0.01):
        sk = UsdSkel.Skeleton(skeleton_prim)
        self.prim = skeleton_prim
        self.meters_per_unit = meters_per_unit
        self.joints = [str(j) for j in sk.GetJointsAttr().Get()]
        topo = UsdSkel.Topology(sk.GetJointsAttr().Get())
        self.parent = [topo.GetParent(i) for i in range(len(self.joints))]
        rest = list(sk.GetRestTransformsAttr().Get())
        self.rest_t = [Gf.Vec3d(m.ExtractTranslation()) for m in rest]
        self.rest_r = [m.ExtractRotation() for m in rest]
        self.idx = {}
        for key, ending in SK.JOINT_ENDINGS.items():
            for i, name in enumerate(self.joints):
                if name.split("/")[-1].lower().endswith(ending.lower()):
                    self.idx.setdefault(key, i)
        self.rest_world = self.fk(Overrides())

    def fk(self, overrides):
        """World (skeleton space) matrices; `overrides` maps joint index -> local Gf.Rotation."""
        world = [None] * len(self.joints)
        trans = getattr(overrides, "trans", {})
        for i in range(len(self.joints)):
            m = Gf.Matrix4d(overrides.get(i, self.rest_r[i]), trans.get(i, self.rest_t[i]))
            p = self.parent[i]
            world[i] = m if p < 0 else m * world[p]
        return world

    def pos(self, world, key):
        return Gf.Vec3d(world[self.idx[key]].ExtractTranslation())

    def key_positions_m(self, world):
        out = {}
        for key, i in self.idx.items():
            t = world[i].ExtractTranslation()
            out[key] = (t[0] * self.meters_per_unit, t[1] * self.meters_per_unit, t[2] * self.meters_per_unit)
        return out

    def rest_offsets(self, side="r"):
        """Rest pose counts as neutral; pass the result as `offsets` to pose.pose_angles."""
        return P.pose_angles(self.key_positions_m(self.rest_world), side)


class ReachSolver:
    def __init__(self, rig, max_reach_fraction=0.92, lean_max_deg=80.0, wrist_flex_deg=0.0,
                 head_flexion_limit_deg=60.0, head_extension_limit_deg=25.0):
        self.rig = rig
        self.max_reach_fraction = max_reach_fraction
        self.lean_max = lean_max_deg
        self.wrist_flex = wrist_flex_deg
        self.head_flex_limit = head_flexion_limit_deg
        self.head_ext_limit = head_extension_limit_deg
        self.offsets = {s: rig.rest_offsets(s) for s in ("r", "l")}
        w = rig.rest_world
        self.len = {}
        for s in ("r", "l"):
            self.len[s] = (
                (rig.pos(w, f"{s}_forearm") - rig.pos(w, f"{s}_arm")).GetLength(),
                (rig.pos(w, f"{s}_hand") - rig.pos(w, f"{s}_forearm")).GetLength(),
            )

    # ------------------------------------------------------------ trunk lean
    HINGE_SHARE = 0.6        # share of the forward bend taken at the hips (the rest is spread over the spine)

    def lean_overrides(self, alpha_deg):
        """Forward bend of `alpha_deg`: a hip hinge (pelvis rotates about the hip joints, legs and feet stay
        exactly planted) plus a smooth spine bend. The measured trunk angle comes out near 0.8 * alpha."""
        r = self.rig
        hips = r.idx["hips"]
        chain = [r.idx["spine"], r.idx["spine1"], r.idx["spine2"]]
        beta = alpha_deg * self.HINGE_SHARE
        spine_total = alpha_deg - beta
        ov = Overrides()
        l_hip, r_hip = r.pos(r.rest_world, "l_up_leg"), r.pos(r.rest_world, "r_up_leg")
        pivot = (l_hip + r_hip) / 2.0                                                          # midpoint of the hip joints
        axis = (l_hip - r_hip).GetNormalized()                                                 # hinge along the line through both hips
        if axis[0] < 0:
            axis = -axis                                                                       # same sense as +X (forward bend)
        delta = Gf.Rotation(axis, beta)
        ov.trans[hips] = pivot + delta.TransformDir(r.rest_t[hips] - pivot)
        hips_new = r.rest_world[hips].ExtractRotation() * delta
        ov[hips] = hips_new
        for leg in (r.idx["l_up_leg"], r.idx["r_up_leg"]):                                    # keep the legs where they were
            ov[leg] = r.rest_world[leg].ExtractRotation() * hips_new.GetInverse()
        prev = hips_new
        for k, j in enumerate(chain):
            new_w = r.rest_world[j].ExtractRotation() * Gf.Rotation(X_AXIS, beta + spine_total * (k + 1) / 3.0)
            ov[j] = new_w * prev.GetInverse()
            prev = new_w
        return ov

    # ------------------------------------------------------------ one arm
    def _solve_arm(self, ov, side, target):
        r = self.rig
        iA, iF, iH, iM = r.idx[f"{side}_arm"], r.idx[f"{side}_forearm"], r.idx[f"{side}_hand"], r.idx[f"{side}_hand_mid"]
        l1, l2 = self.len[side]
        out = -1.0 if side == "r" else 1.0                              # which way "outward" is along x
        world = r.fk(ov)
        shoulder = Gf.Vec3d(world[iA].ExtractTranslation())
        pole = Gf.Vec3d(out * 0.35, -1.0, -0.2).GetNormalized()
        elbow, wrist, reached = two_bone_ik(shoulder, target, l1, l2, pole)

        d_cur = Gf.Vec3d(world[iF].ExtractTranslation()) - shoulder
        new_a = world[iA].ExtractRotation() * Gf.Rotation(d_cur.GetNormalized(), (elbow - shoulder).GetNormalized())
        ov[iA] = new_a * world[r.parent[iA]].ExtractRotation().GetInverse()

        world = r.fk(ov)
        e = Gf.Vec3d(world[iF].ExtractTranslation())
        d_cur = Gf.Vec3d(world[iH].ExtractTranslation()) - e
        new_f = world[iF].ExtractRotation() * Gf.Rotation(d_cur.GetNormalized(), (wrist - e).GetNormalized())
        ov[iF] = new_f * world[iA].ExtractRotation().GetInverse()

        if abs(self.wrist_flex) > 1e-6:                                   # bend the wrist about the forearm's lateral axis
            world = r.fk(ov)
            fore_dir = (Gf.Vec3d(world[iH].ExtractTranslation()) - Gf.Vec3d(world[iF].ExtractTranslation())).GetNormalized()
            hand_dir = (Gf.Vec3d(world[iM].ExtractTranslation()) - Gf.Vec3d(world[iH].ExtractTranslation())).GetNormalized()
            axis = Gf.Cross(fore_dir, hand_dir)                           # bend further in the plane the hand already bends in
            if axis.GetLength() < 1e-3:
                axis = Gf.Cross(fore_dir, DOWN)
            if axis.GetLength() > 1e-6:
                new_h = world[iH].ExtractRotation() * Gf.Rotation(axis.GetNormalized(), self.wrist_flex)
                ov[iH] = new_h * world[iF].ExtractRotation().GetInverse()
        return reached

    # ------------------------------------------------------------ one frame
    def solve(self, targets, look=None):
        """targets: {"r": Vec3d cm or None, "l": ...}. look: Vec3d cm or None.
        Returns (joint rotation overrides, info dict)."""
        r = self.rig
        active = [s for s in ("r", "l") if targets.get(s) is not None]
        alpha, lean_ok = 0.0, True
        if active:
            lean_ok = False
            def ok(a):
                world = r.fk(self.lean_overrides(a))
                return all((Gf.Vec3d(targets[s]) - r.pos(world, f"{s}_arm")).GetLength()
                           <= self.max_reach_fraction * sum(self.len[s]) for s in active)

            coarse = 4.0                                                  # coarse scan, then 0.5 degree steps inside the last bracket
            a = 0.0
            while a <= self.lean_max + 1e-9 and not ok(a):
                a += coarse
            if a <= self.lean_max + 1e-9:
                lean_ok = True
                alpha = max(0.0, a - coarse)
                while alpha < a and not ok(alpha):
                    alpha += 0.5
                alpha = min(alpha, a)
            else:
                alpha = self.lean_max
        ov = self.lean_overrides(alpha)
        reached = {}
        world = r.fk(ov)
        for s in ("r", "l"):
            if targets.get(s) is not None:
                tgt = Gf.Vec3d(targets[s])
            else:                                                         # relaxed arm hanging at the side
                out = -1.0 if s == "r" else 1.0
                tgt = r.pos(world, f"{s}_arm") + Gf.Vec3d(out * 6.0, -0.95 * sum(self.len[s]), 4.0)
            reached[s] = self._solve_arm(ov, s, tgt)

        head_extra = 0.0
        if look is not None:
            world = r.fk(ov)
            eye = r.pos(world, "head") + Gf.Vec3d(0.0, 7.0, 9.0)
            v = Gf.Vec3d(look) - eye
            pitch_down = math.degrees(math.atan2(-v[1], math.hypot(v[0], v[2])))
            head_extra = max(-self.head_ext_limit, min(self.head_flex_limit, pitch_down - alpha))
            iN, iHd, iS2 = r.idx["neck"], r.idx["head"], r.idx["spine2"]
            new_n = world[iN].ExtractRotation() * Gf.Rotation(X_AXIS, 0.65 * head_extra)
            ov[iN] = new_n * world[iS2].ExtractRotation().GetInverse()
            world = r.fk(ov)
            new_h = world[iHd].ExtractRotation() * Gf.Rotation(X_AXIS, 0.35 * head_extra)
            ov[iHd] = new_h * world[iN].ExtractRotation().GetInverse()

        world = r.fk(ov)
        err = {}
        for s in active:
            err[s] = (r.pos(world, f"{s}_hand") - Gf.Vec3d(targets[s])).GetLength()
        info = {"lean_deg": alpha, "lean_ok": lean_ok, "reached": reached, "hand_error_cm": err, "head_extra_deg": head_extra}
        return ov, info

    def angles(self, overrides, side="r"):
        """RULA body angles (calibrated to the rest pose) for a solved pose."""
        world = self.rig.fk(overrides)
        return P.pose_angles(self.rig.key_positions_m(world), side, offsets=self.offsets[side])


# ---------------------------------------------------------------- writing an animation
def _arrays(rig, overrides):
    rots, trans, scales = [], [], []
    for i in range(len(rig.joints)):
        q = overrides.get(i, rig.rest_r[i]).GetQuat()
        rots.append(Gf.Quatf(q.GetReal(), *q.GetImaginary()).GetNormalized())
        trans.append(Gf.Vec3f(getattr(overrides, "trans", {}).get(i, rig.rest_t[i])))
        scales.append(Gf.Vec3h(1, 1, 1))
    return Vt.Vec3fArray(trans), Vt.QuatfArray(rots), Vt.Vec3hArray(scales)


def write_animation(stage, anim_path, rig, frames):
    """frames: list of (timecode, overrides). Writes a UsdSkel.Animation prim and returns it."""
    anim = UsdSkel.Animation.Define(stage, anim_path)
    anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
    t_attr, r_attr, s_attr = anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr()
    for tc, ov in frames:
        trans, rots, scales = _arrays(rig, ov)
        t_attr.Set(trans, tc)
        r_attr.Set(rots, tc)
        s_attr.Set(scales, tc)
    return anim.GetPrim()


def bind_animation(skeleton_prim, anim_path):
    """Point the skeleton at an animation (e.g. the one just written)."""
    UsdSkel.BindingAPI.Apply(skeleton_prim).GetAnimationSourceRel().SetTargets([anim_path])
