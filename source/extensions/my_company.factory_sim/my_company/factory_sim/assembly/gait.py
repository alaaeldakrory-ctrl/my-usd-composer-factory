"""Footstep-planned walking for a scripted root path, shaped after a motion-captured walk.

The worker's script only moves a floor point (root) and a heading. This module plans discrete footsteps that
follow it (strict left/right alternation while moving, corrective steps when standing), then evaluates each foot
with a heel-off -> swing -> heel-strike roll, plus the pelvis bob / sway / rotation and arm swing a walk needs.

Reference (Uniform_M "carry" mocap, 60 fps): a step every ~0.65 s, ~60 cm per step, the foot planted dead still
in stance, ~0.5 s swing, ankle rising ~16 cm (of which ~8 from the heel lifting), hips moving 1-2 cm.

Cadence follows walking speed like people do (about 100 steps/min at 0.9 m/s, 110 at 1.3 m/s; ~78 % of a step is
swing), and the swing foot follows the Mixamo "Walking" clip's swing (SWING_* tables below, sampled from toe-off to
heel strike): the heel kicks up and back right after toe-off with the toes hanging, then the leg reaches forward and
the toes come up before the heel lands. Without this the walk reads stiff and heavy (short quick steps, the foot
skimming the floor flat).
"""
import math

from pxr import Gf

from .rig import fwd_of, left_of

UP = Gf.Vec3d(0, 1, 0)
DS = 0.07                   # double-support pause between corrective (standing) steps (s)
HEEL_OFF = -38.0            # foot pitch at toe-off (deg) for the procedural swing (stairs)
HEEL_LEAD = 0.22            # the heel starts rising this long before the foot leaves the floor (s)
# Mixamo Walking, left foot, toe-off -> heel strike in 12 equal time slices (151 cm of foot travel, hips 91 cm):
SWING_S = (0.0, 0.049, 0.111, 0.185, 0.269, 0.364, 0.466, 0.568, 0.673, 0.779, 0.875, 0.952, 1.0)   # travel fraction
SWING_H = (0.0, 3.48, 6.39, 8.28, 8.32, 6.69, 4.38, 1.57, -0.77, -1.66, -0.98, 0.3, 0.0)           # ankle rise (cm)
SWING_P = (-42.7, -50.1, -39.7, -36.9, -45.9, -46.7, -34.4, -19.5, -4.7, 7.3, 16.3, 23.3, 27.2)    # foot pitch (deg)
CLIP_TRAVEL, CLIP_HIP = 151.6, 90.9


def _table(tab, v):
    x = min(max(v, 0.0), 1.0) * (len(tab) - 1)
    k = min(int(x), len(tab) - 2)
    return tab[k] + (tab[k + 1] - tab[k]) * (x - k)


def step_period(speed, scale):
    """Seconds per step at a walking speed (cm/s): ~0.62 s at 0.85 m/s, ~0.54 s at 1.3 m/s; shorter legs step quicker."""
    return 60.0 / (70.0 + 0.32 * min(max(speed, 0.0), 160.0)) * scale


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class Step:
    __slots__ = ("side", "t0", "t1", "a", "b", "k")

    def __init__(self, side, t0, t1, a, b, k):
        self.side, self.t0, self.t1, self.a, self.b, self.k = side, t0, t1, a, b, k


class Gait:
    def __init__(self, rig, root_at, t_end, dt=1.0 / 60.0, floor=None):
        """floor(x, z, t) -> floor height (cm) under a point, for steps / stands; None = flat floor."""
        self.rig, self.root_at = rig, root_at
        self.floor = floor
        self.steps = {"L": [], "R": []}
        self.scale = math.sqrt(sum(rig.len["Lleg"]) / 87.3)      # shorter legs -> quicker steps
        self._plan(t_end, dt)

    # ------------------------------------------------------------------ planning
    def ground(self, x, z, t):
        return self.floor(x, z, t) if self.floor else 0.0

    def ideal(self, p, h, s, t=0.0):
        fo, le = self.rig.ankle_off[s]
        q = p + fwd_of(h) * fo + left_of(h) * le
        return (Gf.Vec3d(q[0], self.ground(q[0], q[2], t), q[2]), h)

    def _moving(self, t):
        p0, h0 = self.root_at(t)
        p1, h1 = self.root_at(t + 0.35)
        return (p1 - p0).GetLength() > 2.5 or abs(wrap(h1 - h0)) > 5.0

    def _plan(self, t_end, dt):
        p, h = self.root_at(0.0)
        plant = {s: self.ideal(p, h, s, 0.0) for s in "LR"}
        last, t_land, t = "R", -1.0, 0.0
        ds = DS
        while t < t_end + 1.0:
            if t - t_land >= ds:
                moving = self._moving(t)
                t_pred = t + 0.40 + 0.2              # nominal swing + until the foot is under the hips (mocap: ~0.2 s)
                pp, hh = self.root_at(t_pred)
                err = {}
                for s in "LR":
                    ip, ih = self.ideal(pp, hh, s, t_pred)
                    err[s] = ((plant[s][0] - ip).GetLength(), abs(wrap(plant[s][1] - ih)))
                lim_d, lim_a = (1.5, 4.0) if moving else (5.0, 9.0)
                need = {s: err[s][0] > lim_d or err[s][1] > lim_a for s in "LR"}
                cand = "L" if last == "R" else "R"
                side = cand if need[cand] else (last if need[last] and not moving else
                                                (cand if need[last] else None))
                if side:
                    other = "R" if side == "L" else "L"
                    a = plant[side]
                    d0 = (self.ideal(pp, hh, side, t_pred)[0] - a[0]).GetLength()
                    speed = (self.root_at(t + 0.5)[0] - self.root_at(t)[0]).GetLength() / 0.5
                    if moving and speed > 20.0:              # walking: swing ~78 % of the step period
                        P = step_period(speed, self.scale)
                        T, ds = 0.78 * P, 0.22 * P
                    else:                                    # shuffles / turns on the spot
                        T, ds = min(0.46, max(0.32, 0.30 + d0 / 450.0)) * self.scale, DS
                    pp, hh = self.root_at(t + T + 0.2)
                    b = self.ideal(pp, hh, side, t + T)
                    b = self._limit(b, plant[other], side, hh, t + T)
                    dv = b[0] - a[0]
                    d = math.hypot(dv[0], dv[2]) * (0.4 if abs(dv[1]) > 5 else 1.0)   # little roll on stairs
                    k = max(0.0, min(1.0, (d - 8.0) / 45.0))     # how much heel-toe roll this step gets
                    self.steps[side].append(Step(side, t, t + T, a, b, k))
                    plant[side] = b
                    last, t_land = side, t + T
                    t = t + T
                    continue
            t += dt

    def _limit(self, b, other, side, h, t=0.0):
        """Keep a landing within a sane stride of the other foot and never across it."""
        pos, yaw = b
        d = pos - other[0]
        L = d.GetLength()
        if L > 80.0:
            pos = other[0] + d * (80.0 / L)
        lft = left_of(h)
        lat = Gf.Dot(pos - other[0], lft) * (1 if side == "L" else -1)
        if lat < 9.0:                             # nudge apart (capped: on sharp turns the ideal spot rules)
            pos = pos + lft * (min(9.0 - lat, 6.0) * (1 if side == "L" else -1))
        return (Gf.Vec3d(pos[0], self.ground(pos[0], pos[2], t), pos[2]), yaw)

    # ------------------------------------------------------------------ evaluation
    def _find(self, s, t):
        """(step in swing at t or None, planted (pos, yaw))."""
        st = self.steps[s]
        lo, hi = 0, len(st)
        while lo < hi:
            m = (lo + hi) // 2
            if st[m].t0 <= t:
                lo = m + 1
            else:
                hi = m
        k = lo - 1
        if k < 0:
            p, h = self.root_at(0.0)
            return None, self.ideal(p, h, s, 0.0)
        stp = st[k]
        if t <= stp.t1:
            return stp, None
        return None, stp.b

    def _next(self, s, t):
        for stp in self.steps[s]:
            if stp.t0 > t:
                return stp
        return None

    def _toe_pivot(self, s, pa, ya, theta):
        geo, ah = self.rig.foot[s], self.rig.ankle_h
        f = fwd_of(ya)
        toe = pa + f * geo["toe_fwd"] + UP * geo["toe_h"]
        vx, vy = -geo["toe_fwd"], ah - geo["toe_h"]
        c, sn = math.cos(math.radians(theta)), math.sin(math.radians(theta))
        return toe + f * (vx * c - vy * sn) + UP * (vx * sn + vy * c)

    def foot(self, s, t):
        """Ankle position, foot heading, pitch (toes up > 0), toe world pitch (0 flat / None follow), swing u."""
        rig = self.rig
        ah = rig.ankle_h
        geo = rig.foot[s]
        stp, planted = self._find(s, t)
        if stp is None:
            pos, yaw = planted
            nxt = self._next(s, t)
            if nxt is not None and nxt.k > 0 and nxt.t0 - t < HEEL_LEAD:
                th0 = self._clip_pitch(nxt)[0] if self._clip_ok(nxt) else HEEL_OFF * nxt.k * 0.85
                th = th0 * ease(1 - (nxt.t0 - t) / HEEL_LEAD)
                return self._toe_pivot(s, pos, yaw, th), yaw, th, 0.0, None
            return pos + UP * ah, yaw, 0.0, 0.0, None
        u = (t - stp.t0) / max(stp.t1 - stp.t0, 1e-6)
        k = stp.k
        pa, ya = stp.a
        pb, yb = stp.b
        clip = self._clip_ok(stp)
        if clip:
            heel_off, strike, amp = self._clip_pitch(stp)
            U1, U3 = 0.0, 1.0 - 0.18 * k
            h0 = heel_off
        else:
            heel_off, strike = HEEL_OFF * k, 14.0 * k
            U1, U3 = 0.15 * k, 1.0 - 0.18 * k
            h0 = HEEL_OFF * k * 0.85                      # already lifted during late stance

        def toe_pivot(theta):
            return self._toe_pivot(s, pa, ya, theta)

        def heel_pivot(theta):
            f = fwd_of(yb)
            heel = pb - f * geo["heel_back"]
            vx, vy = geo["heel_back"], ah
            c, sn = math.cos(math.radians(theta)), math.sin(math.radians(theta))
            return heel + f * (vx * c - vy * sn) + UP * (vx * sn + vy * c)

        if k > 0 and u < U1:
            th = h0 + (heel_off - h0) * ease(u / U1)
            return toe_pivot(th), ya, th, 0.0, u
        if k > 0 and u > U3:
            th = strike * (1 - ease((u - U3) / (1 - U3)))
            return heel_pivot(th), yb, th, None, u
        v = (u - U1) / max(U3 - U1, 1e-6)
        P1 = toe_pivot(heel_off) if k > 0 else pa + UP * ah
        P3 = heel_pivot(strike) if k > 0 else pb + UP * ah
        if clip:                                     # the mocap swing, fitted between the two footprints
            sv = _table(SWING_S, v)
            travel = (pb - pa).GetLength()
            hs = (rig.hip_h / CLIP_HIP) * min(1.0, max(0.35, travel / CLIP_TRAVEL)) ** 0.5 * k
            pos = P1 + (P3 - P1) * sv + UP * (_table(SWING_H, v) * hs + 1.5 * k * math.sin(math.pi * v))
            th = _table(SWING_P, v) * amp
            if v < 0.22:                             # still rolling over the toes as it leaves the floor
                wv = ease(v / 0.22)
                pos = toe_pivot(th) * (1 - wv) + pos * wv
            elif v > 0.85:                           # and landing onto the heel without skidding
                wv = ease((v - 0.85) / 0.15)
                pos = pos * (1 - wv) + heel_pivot(th) * wv
            toe = th * ease(min(1.0, v / 0.3)) if v < 0.3 else None      # the toes peel off the floor
            if toe is not None and th < 0:
                toe = max(th, toe)
            return pos, ya + wrap(yb - ya) * ease(sv), th, toe, u
        e = ease(v)
        hor = P1 + (P3 - P1) * e
        rise = P3[1] - P1[1]
        if rise > 3.0:                               # stepping up: lift early and clear the nosing
            vy = ease(min(1.0, v * 1.7))
            bump = 6.0
        elif rise < -3.0:                            # stepping down: stay over the step, drop late
            vy = ease(max(0.0, (v - 0.35) / 0.65))
            bump = 4.0
        else:
            vy, bump = v, 0.0
        lift = (4.5 + 4.0 * k + bump) * math.sin(math.pi * v) + (P1[1] + rise * vy - hor[1])
        th = heel_off + (strike - heel_off) * ease(min(1.0, v * 1.25))
        toe = heel_off * (1 - ease(min(1.0, v * 2.0))) if k > 0 else None
        toe = None if toe is None else max(th, toe) if th < 0 else None
        return hor + UP * lift, ya + wrap(yb - ya) * e, th, toe, u

    def _clip_ok(self, stp):
        dy = stp.b[0][1] - stp.a[0][1]
        return stp.k >= 0.5 and abs(dy) <= 3.0           # real walking steps on a flat floor (turns, shuffles, stairs: procedural)

    def _clip_pitch(self, stp):
        """(toe-off pitch, heel-strike pitch, pitch amplitude) for a mocap-shaped step."""
        travel = (stp.b[0] - stp.a[0]).GetLength()
        amp = stp.k * min(1.0, max(0.4, travel / CLIP_TRAVEL)) ** 0.35
        return SWING_P[0] * amp, SWING_P[-1] * amp, amp

    def body(self, t):
        """Gait terms for the trunk: bob (cm), sway (world vec), pelvis yaw (deg), lean (deg), arm swing {side: cm},
        walking weight 0..1."""
        p, h = self.root_at(t)
        f, l = fwd_of(h), left_of(h)
        rel = {}
        swing = None
        for s in "LR":
            stp, planted = self._find(s, t)
            if stp is not None:
                u = (t - stp.t0) / max(stp.t1 - stp.t0, 1e-6)
                pos = stp.a[0] + (stp.b[0] - stp.a[0]) * ease(u)
                swing = (s, u, stp.k)
            else:
                pos = planted[0]
            d = pos - p
            rel[s] = (Gf.Dot(d, f), Gf.Dot(d, l))
        bob, sway = 0.0, Gf.Vec3d(0, 0, 0)
        weight = 0.0
        if swing:
            s, u, k = swing
            weight = k
            bob = (1.6 * math.sin(math.pi * u) - 0.7) * k
            support = "R" if s == "L" else "L"
            sway = l * max(-3.0, min(3.0, rel[support][1] * 0.28 * k))
        yaw = max(-8.0, min(8.0, -0.12 * (rel["L"][0] - rel["R"][0])))
        arms = {s: max(-16.0, min(16.0, 0.36 * rel["R" if s == "L" else "L"][0])) for s in "LR"}
        return {"bob": bob, "sway": sway, "yaw": yaw, "arms": arms, "walk": weight}
