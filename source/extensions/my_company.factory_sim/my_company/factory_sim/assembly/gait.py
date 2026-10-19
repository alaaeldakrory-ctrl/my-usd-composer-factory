"""Footstep-planned walking for a scripted root path, shaped after a motion-captured walk.

The worker's script only moves a floor point (root) and a heading. This module plans discrete footsteps that
follow it (strict left/right alternation while moving, corrective steps when standing), then evaluates each foot
with a heel-off -> swing -> heel-strike roll, plus the pelvis bob / sway / rotation and arm swing a walk needs.

Reference (Uniform_M "carry" mocap, 60 fps): a step every ~0.65 s, ~60 cm per step, the foot planted dead still
in stance, ~0.5 s swing, ankle rising ~16 cm (of which ~8 from the heel lifting), hips moving 1-2 cm.
"""
import math

from pxr import Gf

from .rig import fwd_of, left_of

UP = Gf.Vec3d(0, 1, 0)
DS = 0.07                   # double-support pause between steps (s)
HEEL_OFF = -38.0            # foot pitch at toe-off (deg)
HEEL_LEAD = 0.22            # the heel starts rising this long before the foot leaves the floor (s)


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
        while t < t_end + 1.0:
            if t - t_land >= DS:
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
                    T = min(0.46, max(0.32, 0.30 + d0 / 450.0)) * self.scale
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
                th = HEEL_OFF * nxt.k * 0.85 * ease(1 - (nxt.t0 - t) / HEEL_LEAD)
                return self._toe_pivot(s, pos, yaw, th), yaw, th, 0.0, None
            return pos + UP * ah, yaw, 0.0, 0.0, None
        u = (t - stp.t0) / max(stp.t1 - stp.t0, 1e-6)
        k = stp.k
        pa, ya = stp.a
        pb, yb = stp.b
        heel_off, strike = HEEL_OFF * k, 14.0 * k
        U1, U3 = 0.15 * k, 1.0 - 0.18 * k
        h0 = HEEL_OFF * k * 0.85                          # already lifted during late stance

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
        lift = (2.5 + 6.0 * k + bump) * math.sin(math.pi * v) + (P1[1] + rise * vy - hor[1])
        th = heel_off + (strike - heel_off) * ease(min(1.0, v * 1.25))
        toe = heel_off * (1 - ease(min(1.0, v * 2.0))) if k > 0 else None
        toe = None if toe is None else max(th, toe) if th < 0 else None
        return hor + UP * lift, ya + wrap(yb - ya) * e, th, toe, u

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
