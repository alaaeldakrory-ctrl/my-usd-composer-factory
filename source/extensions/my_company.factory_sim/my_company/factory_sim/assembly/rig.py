"""A Reallusion CC skeleton (RL_BoneRoot, Hip, Pelvis, L_Thigh, ... R_Pinky3) that can be posed in WORLD space.

Everything is solved in the stage's world frame (Y up, cm). The skeleton's own space may be Z-up or rotated;
`M` (skeleton -> world, at rest) converts. A pose is a dict joint index -> local Gf.Rotation plus a root
transform (position on the floor and heading), which moves RL_BoneRoot.

Solvers share one list of world matrices `w` and refresh only the joints below the one they change, so a full
body solve costs a handful of forward-kinematics passes.
"""
import math

from pxr import Gf, Usd, UsdGeom, UsdSkel

UP = Gf.Vec3d(0, 1, 0)


def _rot(m):
    return Gf.Matrix4d(m).ExtractRotation()


def _vec(m):
    return Gf.Vec3d(Gf.Matrix4d(m).ExtractTranslation())


def fwd_of(h):
    a = math.radians(h)
    return Gf.Vec3d(math.sin(a), 0, math.cos(a))


def left_of(h):
    a = math.radians(h)
    return Gf.Vec3d(math.cos(a), 0, -math.sin(a))


def two_bone(a, target, l1, l2, pole):
    """Mid joint and end positions for a two-bone chain from `a` toward `target`."""
    d = target - a
    dist = d.GetLength()
    u = d / dist if dist > 1e-6 else Gf.Vec3d(0, -1, 0)
    dc = min(max(dist, abs(l1 - l2) + 0.1), l1 + l2 - 0.05)
    x = (l1 * l1 - l2 * l2 + dc * dc) / (2 * dc)
    h = math.sqrt(max(l1 * l1 - x * x, 0.0))
    p = pole - u * Gf.Dot(pole, u)
    p = p.GetNormalized() if p.GetLength() > 1e-6 else Gf.Vec3d(1, 0, 0)
    return a + u * x + p * h, a + u * dc


def _signed_angle(a, b, axis):
    """Angle (deg) from a to b about axis, after projecting both onto the plane normal to axis."""
    a = a - axis * Gf.Dot(a, axis)
    b = b - axis * Gf.Dot(b, axis)
    if a.GetLength() < 1e-9 or b.GetLength() < 1e-9:
        return 0.0
    a, b = a.GetNormalized(), b.GetNormalized()
    return math.degrees(math.atan2(Gf.Dot(Gf.Cross(a, b), axis), Gf.Dot(a, b)))


class Pose:
    def __init__(self):
        self.rot = {}                 # joint index -> local Gf.Rotation (skeleton-space semantics)
        self.root_pos = Gf.Vec3d(0, 0, 0)   # world floor point under the character
        self.root_yaw = 0.0           # world heading offset from the character's rest facing
        self.hip_offset = Gf.Vec3d(0, 0, 0)  # world offset of the Hip joint (crouch / bob / sway)


class CCRig:
    def __init__(self, stage, skel_prim):
        sk = UsdSkel.Skeleton(skel_prim)
        self.prim = skel_prim
        self.joints = [str(j) for j in sk.GetJointsAttr().Get()]
        self.names = [j.split("/")[-1] for j in self.joints]
        n = len(self.joints)
        topo = UsdSkel.Topology(sk.GetJointsAttr().Get())
        self.parent = [topo.GetParent(i) for i in range(n)]
        self.desc = [[] for _ in range(n)]
        for k in range(n):                    # joints are ordered parents-first
            p = self.parent[k]
            while p >= 0:
                self.desc[p].append(k)
                p = self.parent[p]
        rest = list(sk.GetRestTransformsAttr().Get())
        self.rest_t = [_vec(m) for m in rest]
        self.rest_r = [_rot(m) for m in rest]
        self.M = UsdGeom.Xformable(skel_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.Minv = self.M.GetInverse()
        self.i = {nm: k for k, nm in enumerate(self.names)}
        self.root = self.i["RL_BoneRoot"]
        self.hip = self.i["Hip"]
        w = self.fk(Pose(), rest_root=True)
        self.rest_world = w
        P = lambda nm: self.pos(w, nm)
        toe, heel = (P("L_ToeBase") + P("R_ToeBase")) / 2, (P("L_Foot") + P("R_Foot")) / 2
        f = toe - heel
        f = Gf.Vec3d(f[0], 0, f[2]).GetNormalized()
        self.rest_fwd = f
        self.rest_heading = math.degrees(math.atan2(f[0], f[2]))
        self.rest_ground = P("RL_BoneRoot")
        lf = left_of(self.rest_heading)
        self.len, self.foot, self.ankle_off = {}, {}, {}
        for s in "LR":
            self.len[s + "arm"] = ((P(s + "_Forearm") - P(s + "_Upperarm")).GetLength(),
                                    (P(s + "_Hand") - P(s + "_Forearm")).GetLength())
            self.len[s + "leg"] = ((P(s + "_Calf") - P(s + "_Thigh")).GetLength(),
                                    (P(s + "_Foot") - P(s + "_Calf")).GetLength())
            a, t = P(s + "_Foot"), P(s + "_ToeBase")
            toe_fwd = Gf.Dot(t - a, f)
            self.foot[s] = {"toe_fwd": toe_fwd, "heel_back": 0.42 * toe_fwd, "toe_h": t[1] - self.rest_ground[1]}
            d = a - self.rest_ground
            self.ankle_off[s] = (Gf.Dot(d, f), Gf.Dot(d, lf))       # ankle vs root: (forward, left)
        self.ankle_h = P("L_Foot")[1] - self.rest_ground[1]
        self.hip_h = P("Hip")[1] - self.rest_ground[1]
        self.shoulder_h = P("L_Upperarm")[1] - self.rest_ground[1]
        self.palm = {s: 0.55 * (P(s + "_Mid1") - P(s + "_Hand")).GetLength() for s in "LR"}
        self.sigma = {}
        for s in "LR":
            f0, s0, n0 = self.hand_frame(w, s)
            self.sigma[s] = 1.0 if Gf.Dot(n0, Gf.Cross(f0, s0)) > 0 else -1.0
        self.arm_len = {s: sum(self.len[s + "arm"]) + self.palm[s] for s in "LR"}

    # ------------------------------------------------------------ forward kinematics (world)
    def _local(self, pose, k, w):
        if k == self.root:
            return self._root_local(pose)
        r = pose.rot.get(k, self.rest_r[k])
        t = self.rest_t[k]
        if k == self.hip and pose.hip_offset.GetLength() > 0:
            t = t + w[self.parent[k]].GetInverse().TransformDir(pose.hip_offset)
        return Gf.Matrix4d(r, t)

    def fk(self, pose, rest_root=False):
        """World matrices for every joint."""
        out = [None] * len(self.joints)
        for k in range(len(self.joints)):
            p = self.parent[k]
            if p < 0:
                m = self._root_local(pose) if not rest_root else Gf.Matrix4d(self.rest_r[k], self.rest_t[k])
                out[k] = m * self.M
            else:
                out[k] = self._local(pose, k, out) * out[p]
        return out

    def refresh(self, pose, w, k):
        """Recompute joint k and everything below it after pose.rot[k] (or the hip offset) changed."""
        for j in [k] + self.desc[k]:
            p = self.parent[j]
            w[j] = self._local(pose, j, w) * (w[p] if p >= 0 else self.M)

    def _root_local(self, pose):
        """RL_BoneRoot local transform that puts the character at root_pos facing rest_heading + root_yaw."""
        R = Gf.Rotation(UP, pose.root_yaw)
        want = Gf.Matrix4d(self.rest_r[self.root], self.rest_t[self.root]) * self.M
        T = Gf.Matrix4d().SetTranslate(-self.rest_ground) * Gf.Matrix4d().SetRotate(R) * \
            Gf.Matrix4d().SetTranslate(pose.root_pos)
        return want * T * self.Minv

    def pos(self, w, n):
        return _vec(w[self.i[n]])

    # ------------------------------------------------------------ joint helpers
    def set_world_rot(self, pose, w, n, world_rot):
        k = self.i[n] if isinstance(n, str) else n
        prot = w[self.parent[k]].ExtractRotation()
        pose.rot[k] = world_rot * prot.GetInverse()
        self.refresh(pose, w, k)

    def rotate(self, pose, w, n, delta):
        """Apply a world-space rotation to joint n (about its own position)."""
        k = self.i[n]
        self.set_world_rot(pose, w, k, w[k].ExtractRotation() * delta)

    def aim(self, pose, w, n, child, target_dir):
        """Rotate joint n (minimal rotation) so the bone toward `child` points along target_dir (world)."""
        k = self.i[n]
        cur = (self.pos(w, child) - self.pos(w, n)).GetNormalized()
        self.set_world_rot(pose, w, k, w[k].ExtractRotation() * Gf.Rotation(cur, target_dir.GetNormalized()))

    def heading_dirs(self, pose):
        h = self.rest_heading + pose.root_yaw
        return fwd_of(h), left_of(h)

    def set_hip(self, pose, w, offset):
        pose.hip_offset = Gf.Vec3d(offset)
        self.refresh(pose, w, self.hip)

    # ------------------------------------------------------------ trunk
    def bend(self, pose, w, deg, side_deg=0.0, twist_deg=0.0, hip_share=0.6, pelvis_yaw=0.0, pelvis_roll=0.0):
        """Hinge forward at the hips (+deg bows forward) and through the spine; legs are re-solved later.

        twist_deg turns the shoulders relative to the hips; pelvis_yaw / pelvis_roll move only the pelvis
        (and the thighs it carries), as in walking."""
        f, left = self.heading_dirs(pose)
        self.rotate(pose, w, "Hip", Gf.Rotation(left, deg * hip_share))
        for c in ("Waist", "Spine01", "Spine02"):
            extra = Gf.Rotation(left, deg * (1 - hip_share) / 3) * Gf.Rotation(f, side_deg / 3) * \
                Gf.Rotation(UP, twist_deg / 3)
            self.rotate(pose, w, c, extra)
        if pelvis_yaw or pelvis_roll:
            self.rotate(pose, w, "Pelvis", Gf.Rotation(UP, pelvis_yaw) * Gf.Rotation(f, pelvis_roll))

    # ------------------------------------------------------------ legs
    def leg(self, pose, w, side, ankle_target, foot_heading, pitch_up=0.0, toe_up=None, knee_out=0.12):
        """Two-bone leg IK to an ankle position; the foot points along foot_heading, pitched by pitch_up
        degrees (toes up > 0). toe_up: world pitch of the toes (0 = flat on the floor), None = follow the foot."""
        ff, fl = fwd_of(foot_heading), left_of(foot_heading)
        out = fl if side == "L" else -fl
        hip = self.pos(w, side + "_Thigh")
        l1, l2 = self.len[side + "leg"]
        knee, ankle = two_bone(hip, ankle_target, l1, l2, ff + out * knee_out)
        self.aim(pose, w, side + "_Thigh", side + "_Calf", knee - hip)
        self.aim(pose, w, side + "_Calf", side + "_Foot", ankle - self.pos(w, side + "_Calf"))
        yaw = Gf.Rotation(UP, foot_heading - self.rest_heading)
        k = self.i[side + "_Foot"]
        self.set_world_rot(pose, w, k, self.rest_world[k].ExtractRotation() * yaw * Gf.Rotation(fl, -pitch_up))
        if toe_up is not None:
            kt = self.i[side + "_ToeBase"]
            self.set_world_rot(pose, w, kt, self.rest_world[kt].ExtractRotation() * yaw * Gf.Rotation(fl, -toe_up))

    def leg_reach_drop(self, w, targets, slack=0.996):
        """How far the hips must drop so both ankle targets are within `slack` of full leg length."""
        drop = 0.0
        for s, a in targets.items():
            hip = self.pos(w, s + "_Thigh")
            L = sum(self.len[s + "leg"]) * slack
            d = a - hip
            hz = math.hypot(d[0], d[2])
            if hz >= L:
                continue
            v = math.sqrt(L * L - hz * hz)
            drop = max(drop, (hip[1] - a[1]) - v)
        return drop

    # ------------------------------------------------------------ hands
    def hand_frame(self, w, side):
        """(finger direction, thumb-side direction, palm normal) of a hand in world."""
        P = lambda n: self.pos(w, n)
        f = (P(side + "_Mid1") - P(side + "_Hand")).GetNormalized()
        s = (P(side + "_Index1") - P(side + "_Pinky1"))
        s = (s - f * Gf.Dot(s, f)).GetNormalized()
        n = Gf.Cross(f, s) if side == "L" else Gf.Cross(s, f)
        th = P(side + "_Thumb1") - P(side + "_Hand")
        if Gf.Dot(n, th - f * Gf.Dot(th, f)) < 0:
            n = -n
        return f, s, n.GetNormalized()

    def hand_basis(self, side, f_hint, s_dir):
        """Target (fingers, palm normal) for a hand whose thumb side points along s_dir."""
        s = s_dir.GetNormalized()
        f = f_hint - s * Gf.Dot(f_hint, s)
        f = f.GetNormalized() if f.GetLength() > 1e-6 else Gf.Vec3d(0, -1, 0)
        n = Gf.Cross(f, s) * self.sigma[side]
        return f, n.GetNormalized()

    def shrug(self, pose, w, side, target, amount=0.3, max_deg=16.0):
        """Let the clavicle follow a reach a little (raise / protract the shoulder)."""
        c = self.pos(w, side + "_Clavicle")
        b0 = (self.pos(w, side + "_Upperarm") - c).GetNormalized()
        d = (target - c).GetNormalized()
        tgt = b0 + (d - b0 * Gf.Dot(d, b0)) * amount
        ang = math.degrees(math.acos(max(-1.0, min(1.0, Gf.Dot(b0, tgt.GetNormalized())))))
        if ang > max_deg:
            tgt = b0 + (tgt.GetNormalized() - b0) * (max_deg / ang)
        self.aim(pose, w, side + "_Clavicle", side + "_Upperarm", tgt)

    def hand_target(self, pose, w, side, spec):
        """Resolve a hand spec into a palm target:
            {"kind": "tube", "c": axis point, "axis": dir, "thumb": hint, "r": radius, "curl": 0..1}
            {"kind": "flat", "c": contact point, "n": palm normal, "f": finger hint, "curl": 0..1}
        -> {"palm", "f", "n", "tube", "r", "curl", "shrug", "pole"}"""
        c = Gf.Vec3d(spec["c"])
        shoulder = self.pos(w, side + "_Upperarm")
        out = {"curl": spec.get("curl", 1.0 if spec["kind"] == "tube" else 0.3), "shrug": spec.get("shrug", 0.3),
               "pole": spec.get("pole"), "r": spec.get("r", 1.6), "tube": None}
        if spec["kind"] == "tube":
            axis = spec["axis"].GetNormalized()
            s_dir = axis if Gf.Dot(axis, spec.get("thumb", UP)) >= 0 else -axis
            f_t, n_t = self.hand_basis(side, spec.get("f") or (c - shoulder), s_dir)
            out.update(palm=c - n_t * (out["r"] + 2.0) - f_t * self.palm[side] * 0.25, f=f_t, n=n_t, tube=(c, axis))
        else:
            n_t = spec["n"].GetNormalized()
            s_dir = Gf.Cross(n_t, spec["f"]) * self.sigma[side]
            f_t, n_t = self.hand_basis(side, spec["f"], s_dir)
            out.update(palm=c - n_t * 2.0, f=f_t, n=n_t)
        return out

    def reach(self, pose, w, side, tg):
        """Shoulder, two-bone arm, hand orientation, forearm twist and fingers for a hand_target(). Returns the
        wrist error (cm)."""
        fwd, left = self.heading_dirs(pose)
        out = left if side == "L" else -left
        f_t, n_t = tg["f"], tg["n"]
        if tg.get("shrug", 0) > 0:
            self.shrug(pose, w, side, tg["palm"], tg["shrug"])
        shoulder = self.pos(w, side + "_Upperarm")
        wrist = tg["palm"] - f_t * self.palm[side]
        l1, l2 = self.len[side + "arm"]
        up_pull = max(0.0, min(1.0, (wrist[1] - shoulder[1] + 10.0) / 30.0))     # high reach: elbow goes out
        pole = tg.get("pole") or (out * (0.45 + 0.6 * up_pull) - UP * 1.0 - fwd * 0.3)
        elbow, wr = two_bone(shoulder, wrist, l1, l2, pole)
        self.aim(pose, w, side + "_Upperarm", side + "_Forearm", elbow - shoulder)
        self.aim(pose, w, side + "_Forearm", side + "_Hand", wr - self.pos(w, side + "_Forearm"))
        f0, s0, n0 = self.hand_frame(w, side)
        delta = (_basis(f0, n0).GetInverse() * _basis(f_t, n_t)).ExtractRotation()
        k = self.i[side + "_Hand"]
        self.set_world_rot(pose, w, k, w[k].ExtractRotation() * delta)
        self._forearm_twist(pose, w, side)
        if tg.get("tube"):
            self.fingers(pose, w, side, tg["curl"], tg["tube"], tg["r"])
        else:
            self.fingers(pose, w, side, tg["curl"])
        return (self.pos(w, side + "_Hand") - wrist).GetLength()

    def arm(self, pose, w, side, spec):
        return self.reach(pose, w, side, self.hand_target(pose, w, side, spec))

    def _forearm_twist(self, pose, w, side):
        """Spread the hand's roll about the forearm over the forearm twist bones (no candy-wrapper wrist)."""
        kf, kh = self.i[side + "_Forearm"], self.i[side + "_Hand"]
        axis = (self.pos(w, side + "_Hand") - self.pos(w, side + "_Forearm")).GetNormalized()
        if not hasattr(self, "_thumb_local"):
            self._thumb_local = {s: self.rest_world[self.i[s + "_Hand"]].ExtractRotation().GetInverse().TransformDir(
                self.hand_frame(self.rest_world, s)[1]) for s in "LR"}
        unrolled = (self.rest_r[kh] * w[kf].ExtractRotation()).TransformDir(self._thumb_local[side])
        th = _signed_angle(unrolled, self.hand_frame(w, side)[1], axis)
        for n, share in ((side + "_ForearmTwist01", 0.25), (side + "_ForearmTwist02", 0.4)):
            if n in self.i:
                self.rotate(pose, w, n, Gf.Rotation(axis, th * share))
        return th

    def fingers(self, pose, w, side, amount, tube=None, r=1.6):
        """Curl the fingers. With tube=(axis point, axis dir) each joint closes until its segment meets the
        tube surface (a real wrap); otherwise a relaxed curl scaled by amount (0 open .. 1 fist)."""
        f0, s0, n0 = self.hand_frame(w, side)
        nat = Gf.Cross(f0, n0).GetNormalized()            # rotating fingers about this moves them toward the palm
        if tube is None:
            for finger, spread in (("Index", 0.9), ("Mid", 1.0), ("Ring", 1.08), ("Pinky", 1.15)):
                for seg, ang in zip((1, 2, 3), (30.0, 42.0, 28.0)):
                    n = f"{side}_{finger}{seg}"
                    if n in self.i:
                        self.rotate(pose, w, n, Gf.Rotation(nat, ang * amount * spread))
            sgn = -1.0 if side == "L" else 1.0
            for seg, (across, curl) in zip((1, 2, 3), ((14.0, 4.0), (4.0, 14.0), (0.0, 16.0))):
                n = f"{side}_Thumb{seg}"
                if n in self.i:
                    self.rotate(pose, w, n, Gf.Rotation(f0, sgn * across * amount) * Gf.Rotation(nat, curl * amount))
            return
        c, ax = tube
        ax = ax.GetNormalized()
        surf = r + 1.7                                    # glove + half a finger thickness

        def radial(p):
            d = p - c
            return (d - ax * Gf.Dot(d, ax)).GetLength()

        def close(joint, child, axis, max_deg, step=4.0):
            """Largest-wrap angle (deg) that keeps `child` outside the tube, stopping where it touches."""
            j = self.pos(w, joint)
            v = child - j
            best = 0.0
            a = 0.0
            if radial(child) < surf:              # already inside (hand short of its target): just close
                return 0.6 * max_deg
            while a <= max_deg:
                p = j + Gf.Rotation(axis, a).TransformDir(v)
                if radial(p) < surf:
                    break
                best = a
                a += step
            return best

        for finger in ("Index", "Mid", "Ring", "Pinky"):
            for seg in (1, 2, 3):
                n = f"{side}_{finger}{seg}"
                if n not in self.i:
                    continue
                j = self.pos(w, n)
                if seg < 3:
                    child = self.pos(w, f"{side}_{finger}{seg + 1}")
                else:
                    prev = self.pos(w, f"{side}_{finger}2")
                    child = j + (j - prev) * 0.8           # virtual fingertip
                ang = close(n, child, nat, (75.0, 95.0, 70.0)[seg - 1]) * amount
                self.rotate(pose, w, n, Gf.Rotation(nat, ang))
        # thumb: wrap round the front of the tube, its tip over the index finger (two-bone solve from Thumb1)
        f0, s0, n0 = self.hand_frame(w, side)
        j1, j2, j3 = (self.pos(w, f"{side}_Thumb{k}") for k in (1, 2, 3))
        l12, l23 = (j2 - j1).GetLength(), (j3 - j2).GetLength() * 1.8     # Thumb3 + its tip
        d1 = j1 - c
        axial = Gf.Dot(d1, ax)
        fp = (f0 - ax * Gf.Dot(f0, ax)).GetNormalized()
        np_ = (n0 - ax * Gf.Dot(n0, ax)).GetNormalized()
        phi = math.radians(35.0)
        tip = c + ax * (axial - 0.8) + (-fp * math.cos(phi) + np_ * math.sin(phi)) * (r + 1.9)
        tip = j1 + (tip - j1) * amount + (j3 + (j3 - j2) * 0.8 - j1) * (1 - amount)
        mid_dir = ((j1 + tip) * 0.5 - c)
        mid_dir = mid_dir - ax * Gf.Dot(mid_dir, ax)
        knee, end = two_bone(j1, tip, l12, l23, mid_dir)
        self.aim(pose, w, f"{side}_Thumb1", f"{side}_Thumb2", knee - j1)
        self.aim(pose, w, f"{side}_Thumb2", f"{side}_Thumb3", end - self.pos(w, f"{side}_Thumb2"))

    # ------------------------------------------------------------ head
    def look(self, pose, w, target, weight=1.0):
        """Turn neck and head toward a world point."""
        head = self.pos(w, "Head")
        d = (target - head).GetNormalized()
        cur = self._head_fwd(w)
        full = Gf.Rotation(cur, d)
        axis, ang = full.GetAxis(), full.GetAngle() * weight
        ang = max(-70.0, min(70.0, ang))
        for n, share in (("NeckTwist01", 0.35), ("NeckTwist02", 0.25), ("Head", 0.4)):
            self.rotate(pose, w, n, Gf.Rotation(axis, ang * share))

    def _head_fwd(self, w):
        k = self.i["Head"]
        delta = self.rest_world[k].ExtractRotation().GetInverse() * w[k].ExtractRotation()
        return delta.TransformDir(self.rest_fwd).GetNormalized()

    # ------------------------------------------------------------ posture table
    def shoulder_table(self, bends):
        """Shoulder (L, R) positions in the root frame (fwd, up, left) for each trunk bend, hips shifted back."""
        out = []
        for b in bends:
            pose = Pose()
            w = self.fk(pose)
            f, l = self.heading_dirs(pose)
            self.set_hip(pose, w, -f * hip_back(b))
            self.bend(pose, w, b)
            row = []
            for s in "LR":
                d = self.pos(w, s + "_Upperarm") - self.rest_ground
                row.append((Gf.Dot(d, f), d[1], Gf.Dot(d, l)))
            out.append(row)
        return out

    # ------------------------------------------------------------ output
    def arrays(self, pose, w=None):
        """translations, rotations, scales for a SkelAnimation (skeleton-local)."""
        from pxr import Vt
        tr, ro, sc = [], [], []
        for k in range(len(self.joints)):
            if k == self.root:
                m = self._root_local(pose)
                t, r = _vec(m), m.ExtractRotation()
            elif k == self.hip and pose.hip_offset.GetLength() > 0:
                w = w or self.fk(pose)
                off = w[self.parent[k]].GetInverse().TransformDir(pose.hip_offset)
                t, r = self.rest_t[k] + off, pose.rot.get(k, self.rest_r[k])
            else:
                t, r = self.rest_t[k], pose.rot.get(k, self.rest_r[k])
            q = r.GetQuat()
            tr.append(Gf.Vec3f(t))
            ro.append(Gf.Quatf(q.GetReal(), *q.GetImaginary()).GetNormalized())
            sc.append(Gf.Vec3h(1, 1, 1))
        return Vt.Vec3fArray(tr), Vt.QuatfArray(ro), Vt.Vec3hArray(sc)


def hip_back(bend_deg):
    """How far the hips sit back (cm) to balance a forward trunk bend."""
    return min(16.0, 0.26 * max(0.0, bend_deg))


def _basis(f, n):
    """Rotation matrix whose rows are (f, n, f x n)."""
    f = f.GetNormalized()
    n = (n - f * Gf.Dot(n, f)).GetNormalized()
    b = Gf.Cross(f, n)
    return Gf.Matrix4d(f[0], f[1], f[2], 0, n[0], n[1], n[2], 0, b[0], b[1], b[2], 0, 0, 0, 0, 1)
