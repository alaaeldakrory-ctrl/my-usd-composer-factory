"""Mocap layer: Mixamo walk cycles retargeted onto the CC rig by bone directions.

The footstep planner (gait.py) still owns the feet and the path; this layer adds what a planner cannot invent —
pelvis rotation and bob, spine / neck motion and free-arm swing from real motion capture — phase-locked to the
planned footsteps (each planned heel strike lines up with the clip's heel strike of the same foot).

Retargeting is direction based (rest poses do not need to match): each CC joint is turned so its bone points
where the matching Mixamo bone points, in the walker's heading frame, with a second axis fixing the roll.
"""
import math
import os

from pxr import Gf

from . import fbx_anim
from .rig import _basis, fwd_of, left_of

UP = Gf.Vec3d(0, 1, 0)
MIX = "mixamorig1:"
CLIPS = r"C:\OmniverseData\mixamo\fbx"

# CC joint: (child for the bone direction, Mixamo from, Mixamo to, roll axis CC pair, roll axis Mixamo pair)
TRUNK = [
    ("Hip", "Waist", "Hips", "Spine", ("L_Thigh", "R_Thigh"), ("LeftUpLeg", "RightUpLeg")),
    ("Waist", "Spine01", "Spine", "Spine1", None, None),
    ("Spine01", "Spine02", "Spine1", "Spine2", None, None),
    ("Spine02", "NeckTwist01", "Spine2", "Neck", ("L_Upperarm", "R_Upperarm"), ("LeftArm", "RightArm")),
    ("NeckTwist01", "Head", "Neck", "Head", None, None),
]
ARM = [  # per side, with {s} = L/R for CC and {S} = Left/Right for Mixamo
    ("{s}_Clavicle", "{s}_Upperarm", "{S}Shoulder", "{S}Arm", None, None),
    ("{s}_Upperarm", "{s}_Forearm", "{S}Arm", "{S}ForeArm", ("{s}_Hand", "{s}_Forearm"), ("{S}Hand", "{S}ForeArm")),
    ("{s}_Forearm", "{s}_Hand", "{S}ForeArm", "{S}Hand", None, None),
    ("{s}_Hand", "{s}_Mid1", "{S}Hand", "{S}HandMiddle1", ("{s}_Index1", "{s}_Pinky1"), ("{S}HandIndex1", "{S}HandPinky1")),
]


LEG = [
    ("{s}_Thigh", "{s}_Calf", "{S}UpLeg", "{S}Leg", None, None),
    ("{s}_Calf", "{s}_Foot", "{S}Leg", "{S}Foot", None, None),
    ("{s}_Foot", "{s}_ToeBase", "{S}Foot", "{S}ToeBase", None, None),
]


class Cycle:
    """One walk cycle in the walker's frame: per frame, joint positions as (forward, up, left) from the
    progression point; plus the phases of each heel strike."""

    def __init__(self, path, fps=30.0, span=None):
        clip = fbx_anim.FbxClip(path, fps)
        a, b = span or (0, clip.frames - 1)
        n = b - a + 1
        G = [clip.global_mats(f) for f in range(a, b + 1)]
        P = [{k[len(MIX):] if k.startswith(MIX) else k: Gf.Vec3d(m.ExtractTranslation()) for k, m in g.items()} for g in G]
        h0, h1 = P[0]["Hips"], P[-1]["Hips"]
        travel = Gf.Vec3d(h1[0] - h0[0], 0, h1[2] - h0[2])
        self.speed = travel.GetLength() / max((n - 1) / fps, 1e-6)
        f = travel.GetNormalized() if travel.GetLength() > 1 else Gf.Vec3d(0, 0, 1)
        l = Gf.Cross(UP, f)
        self.n = n - 1                                     # last frame repeats the first
        self.frames = []
        for k in range(self.n):
            o = Gf.Vec3d(h0[0], 0, h0[2]) + travel * (k / (n - 1))
            self.frames.append({j: (Gf.Dot(p - o, f), p[1], Gf.Dot(p - o, l)) for j, p in P[k].items()})
        hips = [fr["Hips"] for fr in self.frames]
        self.hip_mean = tuple(sum(h[i] for h in hips) / len(hips) for i in range(3))
        self.strike = {s: self._strike(S) for s, S in (("L", "Left"), ("R", "Right"))}
        self.name = os.path.splitext(os.path.basename(path))[0]

    def _strike(self, S):
        """Phase (0..1) where the foot lands: its forward speed (in the walker frame) turns from fastest to rest."""
        fw = [fr[S + "Foot"][0] for fr in self.frames]
        n = self.n
        vel = [(fw[(k + 1) % n] - fw[k - 1]) for k in range(n)]
        k_fast = max(range(n), key=lambda k: vel[k])
        k = k_fast
        for _ in range(n):
            k = (k + 1) % n
            if vel[k] < 0.25 * vel[k_fast]:
                return k / n
        return 0.0

    def sample(self, phase):
        """Joint positions (forward, up, left) at a cycle phase, interpolated."""
        x = (phase % 1.0) * self.n
        a = int(math.floor(x)) % self.n
        b = (a + 1) % self.n
        u = x - math.floor(x)
        A, B = self.frames[a], self.frames[b]
        return {j: tuple(A[j][i] + (B[j][i] - A[j][i]) * u for i in range(3)) for j in A}


SPANS = {"Walkt_carrying": (36, 80)}      # one clean cycle out of a longer take


def load(name):
    return Cycle(os.path.join(CLIPS, name + ".fbx"), span=SPANS.get(name))


class Phase:
    """Gait phase over time, keyed so that each planned heel strike of a walking step hits the clip's strike."""

    def __init__(self, gait, cycle, min_roll=0.5):
        events = sorted((stp.t1, s) for s in "LR" for stp in gait.steps[s] if stp.k >= min_roll)
        self.keys = []
        ph = None
        for t, s in events:
            target = cycle.strike[s]
            if ph is None:
                ph = target
            else:
                step = (target - ph) % 1.0
                if step < 0.2:                     # same foot twice in a row: a whole cycle
                    step += 1.0
                ph += step
            self.keys.append((t, ph))

    def at(self, t):
        k = self.keys
        if not k:
            return 0.0
        if t <= k[0][0]:
            return k[0][1] - (k[0][0] - t) / 1.05
        if t >= k[-1][0]:
            return k[-1][1] + min(t - k[-1][0], 0.6) / 1.05
        lo, hi = 0, len(k) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if k[m][0] <= t:
                lo = m
            else:
                hi = m
        (ta, pa), (tb, pb) = k[lo], k[hi]
        if tb - ta > 1.2:                           # a pause between walks: hold, then wind up into the next
            return pa + min(t - ta, 0.5) / 1.05 if t - ta < 0.5 else pb - max(tb - t, 0.0) / 1.05 if tb - t < 0.5 else pa + 0.5 / 1.05
        return pa + (pb - pa) * (t - ta) / (tb - ta)


def _world(v, f, l):
    return f * v[0] + UP * v[1] + l * v[2]


def _dir(S, a, b, f, l):
    return _world(tuple(S[b][i] - S[a][i] for i in range(3)), f, l)


def apply(rig, pose, w, sample, heading, weight, arms=(), legs=False, trunk=True):
    """Turn trunk (and the listed free arms) toward the mocap sample, blended by weight."""
    if weight <= 1e-3:
        return
    f, l = fwd_of(heading), left_of(heading)
    rows = list(TRUNK) if trunk else []
    for s in arms:
        S = "Left" if s == "L" else "Right"
        rows += [tuple(x.format(s=s, S=S) if isinstance(x, str) else
                       (tuple(y.format(s=s, S=S) for y in x) if x else None) for x in row) for row in ARM]
    if legs:
        for s, S in (("L", "Left"), ("R", "Right")):
            rows += [tuple(x.format(s=s, S=S) if isinstance(x, str) else None for x in row) for row in LEG]
    for joint, child, ma, mb, cc_roll, mx_roll in rows:
        if joint not in rig.i or ma not in sample or mb not in sample:
            continue
        k = rig.i[joint]
        tgt = _dir(sample, ma, mb, f, l).GetNormalized()
        cur = (rig.pos(w, child) - rig.pos(w, joint)).GetNormalized()
        if cc_roll and mx_roll and all(m in sample for m in mx_roll):
            cs = rig.pos(w, cc_roll[0]) - rig.pos(w, cc_roll[1])
            ts = _dir(sample, mx_roll[1], mx_roll[0], f, l)
            delta = (_basis(cur, cs).GetInverse() * _basis(tgt, ts)).ExtractRotation()
        else:
            delta = Gf.Rotation(cur, tgt)
        R0 = w[k].ExtractRotation()
        R1 = R0 * delta
        q = Gf.Slerp(min(1.0, weight), R0.GetQuat(), R1.GetQuat())
        rig.set_world_rot(pose, w, k, Gf.Rotation(q))


def hip_offset(cycle, sample, heading, scale):
    """Mocap pelvis bob / sway / surge (world vector, cm) relative to the cycle mean."""
    h = sample["Hips"]
    m = cycle.hip_mean
    d = (0.3 * (h[0] - m[0]), h[1] - m[1], h[2] - m[2])
    return _world(d, fwd_of(heading), left_of(heading)) * scale


class Clip:
    """A one-shot clip (turn on the spot, start walking) in a slowly turning pelvis frame: per frame, joint positions
    as (forward, up, left) from the hips' floor point, plus how far the hips have travelled and turned."""

    def __init__(self, path, fps=30.0, smooth=6):
        clip = fbx_anim.FbxClip(path, fps)
        n = clip.frames
        P = []
        for f in range(n):
            g = clip.global_mats(f)
            P.append({k[len(MIX):] if k.startswith(MIX) else k: Gf.Vec3d(m.ExtractTranslation()) for k, m in g.items()})
        raw = []
        for p in P:
            l = p["LeftUpLeg"] - p["RightUpLeg"]
            fw = Gf.Cross(Gf.Vec3d(l[0], 0, l[2]), UP)
            raw.append(math.degrees(math.atan2(fw[0], fw[2])))
        for k in range(1, n):                                    # unwrap
            while raw[k] - raw[k - 1] > 180:
                raw[k] -= 360
            while raw[k] - raw[k - 1] < -180:
                raw[k] += 360
        yaw = [sum(raw[max(0, k - smooth):k + smooth + 1]) / len(raw[max(0, k - smooth):k + smooth + 1]) for k in range(n)]
        self.n, self.fps = n, fps
        self.frames, self.travel, self.turned = [], [], []
        h0 = P[0]["Hips"]
        f0 = fwd_of(yaw[0])
        for k in range(n):
            f, l = fwd_of(yaw[k]), left_of(yaw[k])
            o = Gf.Vec3d(P[k]["Hips"][0], 0, P[k]["Hips"][2])
            self.frames.append({j: (Gf.Dot(p - o, f), p[1], Gf.Dot(p - o, l)) for j, p in P[k].items()})
            self.travel.append(Gf.Dot(P[k]["Hips"] - h0, f0))
            self.turned.append(yaw[k] - yaw[0])
        hips = [fr["Hips"] for fr in self.frames]
        self.hip_mean = tuple(sum(h[i] for h in hips) / len(hips) for i in range(3))
        self.move0 = next((k for k in range(n) if self.travel[k] > 2.0), 0)
        self.name = os.path.splitext(os.path.basename(path))[0]

    def sample_frame(self, x):
        x = min(max(x, 0.0), self.n - 1.0)
        a = int(math.floor(x))
        b = min(a + 1, self.n - 1)
        u = x - a
        A, B = self.frames[a], self.frames[b]
        return {j: tuple(A[j][i] + (B[j][i] - A[j][i]) * u for i in range(3)) for j in A}

    def frame_at_travel(self, d):
        """Clip frame (fractional) where the hips have moved d cm (clip units) since the start."""
        if d <= 0:
            return float(self.move0)
        for k in range(self.move0, self.n - 1):
            a, b = self.travel[k], self.travel[k + 1]
            if b >= d:
                return k + (d - a) / max(b - a, 1e-6)
        return float(self.n - 1)


def load_clip(name):
    return Clip(os.path.join(CLIPS, name + ".fbx"))
