"""Choreography for the ES2632 guardrail assembly: part motion, worker scripts, posture and baking.

Times in seconds, positions in cm (world, Y up). Headings in degrees: forward = (sin h, 0, cos h).
Six steps, as the kit really goes together (see script()): both main side rails, the extension U-frame, the front top
crossbar (from a step stand), the entry gate on its hinges and the platform controller.
"""
import math

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

from . import gait as GT
from . import mocap as MC
from . import rig as RG
from . import scene as SC
from .rig import fwd_of, left_of

FPS = 30.0
UP = Gf.Vec3d(0, 1, 0)
BLEND = 0.5                     # hand transition time (s)


def ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def lerp_angle(a, b, u):
    return a + wrap(b - a) * u


def heading_to(d):
    return math.degrees(math.atan2(d[0], d[2]))


# --------------------------------------------------------------------------------------------- parts
class PartTrack:
    """Segments: ("hold", t0, t1, m) / ("move", t0, t1, m0, m1) / ("path", t0, t1, fn) / ("carry", t0, t1, worker, rel)."""

    def __init__(self, m0):
        self.segs = []
        self.m0 = m0

    def matrix(self, t, workers):
        m = self.m0
        for s in self.segs:
            if t < s[1]:
                break
            kind = s[0]
            if kind == "move":
                if t <= s[2]:
                    return _blend_m(s[3], s[4], ease((t - s[1]) / max(s[2] - s[1], 1e-6)))
                m = s[4]
            elif kind == "path":
                if t <= s[2]:
                    return s[3](t)
                m = s[3](s[2])
            elif kind == "carry":
                if t <= s[2]:
                    return s[4] * workers[s[3]].frame(t)
                m = s[4] * workers[s[3]].frame(s[2])
            else:
                m = s[3]
        return m


def _blend_m(a, b, u):
    ta, tb = a.ExtractTranslation(), b.ExtractTranslation()
    q = Gf.Slerp(u, a.ExtractRotation().GetQuat(), b.ExtractRotation().GetQuat())
    return Gf.Matrix4d().SetRotate(Gf.Rotation(q)) * Gf.Matrix4d().SetTranslate(ta + (tb - ta) * u)


# --------------------------------------------------------------------------------------------- workers
class Worker:
    """A worker's script: dense root keys (floor point + heading), hand holds and look targets over time."""
    DT = 1.0 / FPS

    def __init__(self, name, rig, pos, heading):
        self.name, self.rig = name, rig
        self.t = 0.0
        self.pos, self.h = Gf.Vec3d(pos[0], 0, pos[1]), heading
        self.root = [(0.0, Gf.Vec3d(self.pos), heading)]
        self.holds = {"L": [], "R": []}       # (t0, t1, spec fn)
        self.looks = []
        self.turns = []                        # (t0, t1, degrees) turns on the spot
        self.starts = []                       # (t0, start position) the moment a walk sets off
        self.pace = sum(rig.len["Lleg"]) / 87.3          # walking speeds below are for a 175 cm worker

    def _key(self, t, pos, h):
        self.root.append((t, Gf.Vec3d(pos[0], pos[1], pos[2]), h))

    def climb(self, p, dur=0.9):
        """Step up / down to the floor point p (x, y, z) keeping the heading (the gait plans the footfalls)."""
        q = Gf.Vec3d(p)
        p0 = Gf.Vec3d(self.pos)
        n = max(2, int(dur / self.DT))
        for k in range(1, n + 1):
            u = ease(k / n)
            pt = p0 + (q - p0) * u
            pt[1] = p0[1] + (q[1] - p0[1]) * ease(min(1.0, (k / n) * 1.25))
            self._key(self.t + dur * k / n, pt, self.h)
        self.t += dur
        self.pos = q

    def stand(self, dur):
        if dur <= 1e-6:
            return
        self.t += dur
        self._key(self.t, self.pos, self.h)

    def until(self, t):
        self.stand(t - self.t)

    def turn(self, h, speed=130.0):
        d = wrap(h - self.h)
        if abs(d) < 1:
            return
        dur = max(0.6, abs(d) / speed)
        n = max(2, int(dur / self.DT))
        h0 = self.h
        self.turns.append((self.t, self.t + dur, d))
        for k in range(1, n + 1):
            self._key(self.t + dur * k / n, self.pos, h0 + d * ease(k / n))
        self.t += dur
        self.h = h0 + d

    def follow(self, t0, t1, fn):
        """Root follows fn(t) -> ((x, z), heading) between t0 and t1 (e.g. carrying a panel with someone)."""
        self.until(t0)
        n = max(2, int((t1 - t0) / self.DT))
        for k in range(1, n + 1):
            t = t0 + (t1 - t0) * k / n
            (x, z), h = fn(t)
            self._key(t, Gf.Vec3d(x, self.pos[1], z), h)
        self.t = t1
        (x, z), self.h = fn(t1)
        self.pos = Gf.Vec3d(x, self.pos[1], z)

    def strafe(self, p, speed=40.0):
        """Shuffle to (x, z) keeping the heading."""
        q = Gf.Vec3d(p[0], self.pos[1], p[1])
        L = (q - self.pos).GetLength()
        if L < 1:
            return
        dur = max(0.7, L / (speed * self.pace))
        p0 = Gf.Vec3d(self.pos)
        n = max(2, int(dur / self.DT))
        for k in range(1, n + 1):
            self._key(self.t + dur * k / n, p0 + (q - p0) * ease(k / n), self.h)
        self.t += dur
        self.pos = q

    def walk(self, pts, end_h=None, speed=85.0, accel=110.0, radius=45.0):
        """Walk a rounded path through (x, z) points with a smooth start / stop, turning into the walk."""
        path = [Gf.Vec3d(self.pos)]
        for p in pts:
            q = Gf.Vec3d(p[0], 0, p[1])
            if (q - path[-1]).GetLength() > 1:
                path.append(q)
        if len(path) < 2:
            if end_h is not None:
                self.turn(end_h)
            return
        dense = _rounded(path, radius)
        S = [0.0]
        for a, b in zip(dense, dense[1:]):
            S.append(S[-1] + (b - a).GetLength())
        L = S[-1]
        h_start = heading_to(dense[min(len(dense) - 1, 8)] - dense[0])
        if abs(wrap(h_start - self.h)) > 40:                  # turn on the spot, finish the turn while walking
            self.turn(h_start - math.copysign(25.0, wrap(h_start - self.h)))
        h0 = self.h
        self.starts.append((self.t, Gf.Vec3d(self.pos)))
        tail = heading_to(dense[-1] - dense[max(0, len(dense) - 9)])
        blend_end = end_h is not None and abs(wrap(end_h - tail)) <= 100
        v = speed * self.pace
        d_acc = v * v / (2 * accel)
        if L < 2 * d_acc:
            v = math.sqrt(accel * L)
            d_acc = L / 2
        t_acc = v / accel
        T = 2 * t_acc + (L - 2 * d_acc) / v
        n = max(2, int(T / self.DT))
        j = 0
        for k in range(1, n + 1):
            tt = T * k / n
            if tt < t_acc:
                s = 0.5 * accel * tt * tt
            elif tt > T - t_acc:
                r = T - tt
                s = L - 0.5 * accel * r * r
            else:
                s = d_acc + v * (tt - t_acc)
            s = min(max(s, 0.0), L)
            while j < len(S) - 2 and S[j + 1] < s:
                j += 1
            u = (s - S[j]) / max(S[j + 1] - S[j], 1e-9)
            p = dense[j] + (dense[j + 1] - dense[j]) * u
            a = dense[max(0, j - 5)]
            b = dense[min(len(dense) - 1, j + 6)]
            h = heading_to(b - a) if (b - a).GetLength() > 1e-6 else h0
            h = lerp_angle(h0, h, ease(s / 35.0))
            if blend_end:
                h = lerp_angle(h, end_h, ease((s - (L - 45.0)) / 45.0))
            self._key(self.t + tt, p, h)
        self.t += T
        self.pos = Gf.Vec3d(dense[-1])
        self.h = self.root[-1][2]
        if end_h is not None:
            self.turn(end_h)

    def root_at(self, t):
        keys = self.root
        if t <= keys[0][0]:
            return keys[0][1], keys[0][2]
        if t >= keys[-1][0]:
            return keys[-1][1], keys[-1][2]
        lo, hi = 0, len(keys) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if keys[m][0] <= t:
                lo = m
            else:
                hi = m
        a, b = keys[lo], keys[hi]
        u = (t - a[0]) / max(b[0] - a[0], 1e-9)
        return a[1] + (b[1] - a[1]) * u, lerp_angle(a[2], b[2], u)

    def frame(self, t):
        """World matrix of the worker's floor frame (rows: left, up, forward, position)."""
        p, h = self.root_at(t)
        f, l = fwd_of(h), left_of(h)
        return Gf.Matrix4d(l[0], l[1], l[2], 0, 0, 1, 0, 0, f[0], f[1], f[2], 0, p[0], p[1], p[2], 1)

    def hold(self, side, t0, t1, fn):
        self.holds[side].append((t0, t1, fn))

    def look_at(self, t0, t1, target):
        self.looks.append((t0, t1, target))


def _rounded(path, radius, step=2.0):
    """Dense points along a polyline whose corners are rounded with quadratic arcs."""
    pts = [Gf.Vec3d(path[0])]
    for i in range(1, len(path) - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        r = min(radius, (b - a).GetLength() * 0.45, (c - b).GetLength() * 0.45)
        p0 = b + (a - b).GetNormalized() * r
        p2 = b + (c - b).GetNormalized() * r
        _line(pts, p0, step)
        n = max(2, int(2 * r / step))
        for k in range(1, n + 1):
            u = k / n
            pts.append(p0 * (1 - u) ** 2 + b * (2 * u * (1 - u)) + p2 * u * u)
    _line(pts, path[-1], step)
    return pts


def _line(pts, q, step):
    a = pts[-1]
    L = (q - a).GetLength()
    n = int(L / step)
    for k in range(1, n + 1):
        pts.append(a + (q - a) * (k / max(n, 1)))
    if (pts[-1] - q).GetLength() > 1e-6:
        pts.append(Gf.Vec3d(q))


# --------------------------------------------------------------------------------------------- hand specs
# A hand spec fn(t, ctx) returns a rig hand spec dict (world) or None (free hand).
def grip(part_name, hold, curl=1.0, load=True):
    """A tube hold on a part, carried along with the part."""
    def f(t, ctx):
        m = ctx.part_m(part_name, t)
        return {"kind": "tube", "c": m.Transform(hold["c"]), "axis": m.TransformDir(hold["axis"]),
                "thumb": m.TransformDir(hold["thumb"]), "r": hold["r"], "curl": curl, "load": load}
    return f


def flat(point, normal, fingers=UP, curl=0.15):
    """An open hand braced flat on a surface (palm toward `normal`)."""
    def f(t, ctx):
        return {"kind": "flat", "c": Gf.Vec3d(point), "n": Gf.Vec3d(normal), "f": Gf.Vec3d(fingers), "curl": curl}
    return f


def wrench_spec(worker_name, bolt, axis, t_start, handle_dir, period=0.9):
    """Hand on a ratchet whose head sits on `bolt`; the handle points along handle_dir (out to the working hand's
    side, a little down) and swings +-30 deg about the bolt axis; the hand holds the grip end."""
    def f(t, ctx):
        th = 30.0 * math.sin(2 * math.pi * (t - t_start) / period)
        out = Gf.Rotation(axis, th).TransformDir(handle_dir)
        out = (out - axis * Gf.Dot(out, axis)).GetNormalized()
        origin = bolt + out * 23.0                          # handle end; the head (z = 23) is on the bolt
        d = -out
        ctx.wrench[worker_name] = (t, origin, d, axis)
        return {"kind": "tube", "c": origin + d * 6.5 - axis * 0.5, "axis": d, "thumb": d, "r": 1.45, "curl": 1.0}
    return f


class Ctx:
    def __init__(self, parts, workers):
        self.parts, self.workers = parts, workers
        self.wrench = {}

    def part_m(self, name, t):
        return self.parts[name].matrix(t, self.workers)


def floor_fn(ctx):
    """Floor height under (x, z) at time t: the step stand's treads, else 0."""
    def f(x, z, t):
        if "StepStand" not in ctx.parts:
            return 0.0
        return SC.step_height(ctx.part_m("StepStand", t), Gf.Vec3d(x, 0, z))
    return f


# --------------------------------------------------------------------------------------------- the script
BOLT_EVENTS = []          # (part, bolt head point, axis, t0, t1) filled by script()


def script(parts_def, rigs):
    """Six assembly steps on the stowed ES2632:
    1-2 main side rails L / R (two people, from floor stands), 3 extension U-frame (two people, slid in from the back),
    4 front top crossbar (A, from a step stand), 5 entry gate hung on its hinges and swung shut (B),
    6 platform controller hooked onto the right rail (A, step stand moved to the side)."""
    BOLT_EVENTS.clear()
    A = Worker("A", rigs["A"], (-215, 345), 180.0)
    B = Worker("B", rigs["B"], (225, 335), 180.0)
    workers = {"A": A, "B": B}
    tracks = {n: PartTrack(p.at(*p.start)) for n, p in parts_def.items()}
    ctx0 = Ctx(tracks, workers)
    BACK = {"A": 31.0, "B": 27.0}                      # pelvis to the held post
    px = {"A": SC.SIDE_POSTS[0], "B": SC.SIDE_POSTS[1]}
    sx = {w: px[w] + SC.SIDE_STANCE[w] for w in "AB"}

    def bolt(W, part, bp, axis, t0, hand, steady=None, dur=2.5):
        """Ratchet in `hand` (the one on the bolt's side), handle out to that side; the other hand steadies."""
        other = "R" if hand == "L" else "L"
        _, h = W.root_at(t0)
        side = left_of(h) * (1.0 if hand == "L" else -1.0)
        side = side - axis * Gf.Dot(side, axis)
        side = side.GetNormalized() if side.GetLength() > 1e-6 else -UP
        a = math.radians(25.0)
        handle = side * math.cos(a) - UP * math.sin(a)
        handle = (handle - axis * Gf.Dot(handle, axis)).GetNormalized()
        W.hold(hand, t0, t0 + dur, wrench_spec(W.name, bp, axis, t0, handle))
        if steady:
            W.hold(other, t0 - 0.3, t0 + dur, steady)
        W.look_at(t0, t0 + dur, bp)
        BOLT_EVENTS.append((part, bp, axis, t0, t0 + dur))
        return t0 + dur

    def hand_toward(W, t, point):
        """'L' if the point is on the worker's left at time t, else 'R'."""
        p, h = W.root_at(t)
        return "L" if Gf.Dot(Gf.Vec3d(point) - p, left_of(h)) > 0 else "R"

    def carry_segment(name, W, t0, t1):
        rel = tracks[name].matrix(t0, workers) * W.frame(t0).GetInverse()
        tracks[name].segs.append(("carry", t0, t1, W.name, rel))
        return rel

    # ---------------------------------------------------------------- 1-2: main side rails
    def side_rail(side):
        zs = 1.0 if side == "L" else -1.0
        name = "Side" + side
        P = parts_def[name]
        face = 180.0 if side == "L" else 0.0
        t = max(A.t, B.t) + 0.2
        for W in (A, B):
            W.until(t)
        x0 = P.pivot[0]
        bottom0, carry_y, raise_y = SC.STAND_Y, 94.0, SC.SIDE_PIVOT[1] + 5.5
        z_stand, z_near, z_in = zs * SC.STAND_Z, zs * 63.0, zs * SC.SIDE_Z
        t_grip, t_lift0 = t, t + 1.4
        t_lift1 = t_lift0 + 1.7
        t_carry0 = t_lift1 + 0.3
        t_carry1 = t_carry0 + abs(z_stand - z_near) / 52.0
        t_raise1 = t_carry1 + 1.7
        t_in1 = t_raise1 + 1.1
        t_seat = t_in1 + 0.9
        m = lambda y, z: P.at(Gf.Vec3d(x0, y, z))
        tracks[name].segs += [("hold", 0.0, t_lift0, m(bottom0, z_stand)),
                              ("move", t_lift0, t_lift1, m(bottom0, z_stand), m(carry_y, z_stand)),
                              ("hold", t_lift1, t_carry0, m(carry_y, z_stand)),
                              ("move", t_carry0, t_carry1, m(carry_y, z_stand), m(carry_y, z_near)),
                              ("move", t_carry1, t_raise1, m(carry_y, z_near), m(raise_y, z_near)),
                              ("move", t_raise1, t_in1, m(raise_y, z_near), m(raise_y, z_in)),
                              ("move", t_in1, t_seat, m(raise_y, z_in), P.installed()),
                              ("hold", t_seat, 1e9, P.installed())]
        for W in (A, B):
            off = BACK[W.name]
            W.follow(t_carry0, t_in1, lambda tt, W=W, off=off: (
                (sx[W.name], ctx0.part_m(name, tt).Transform(P.pivot)[2] + zs * off), face))
            holds = P.holds[W.name]
            near = "L" if (px[W.name] - sx[W.name]) * left_of(face)[0] > 0 else "R"
            far = "R" if near == "L" else "L"
            W.hold(near, t_grip, t_seat + 0.4, grip(name, holds["lo"]))
            W.hold(far, t_grip, t_seat + 0.4, grip(name, holds["hi"]))
            W.look_at(t_grip, t_seat, lambda tt, ctx, n=name: ctx.part_m(n, tt).Transform(P.pivot + UP * 40))
        # bolting: square up to the post; brace one palm on the deck side toward the rail's middle,
        # ratchet in the other hand with the handle out past the end of the rail
        for W in (A, B):
            W.until(t_seat + 0.45)
            W.strafe((px[W.name], W.pos[2]), speed=35.0)
            inward = 1.0 if P.pivot[0] > px[W.name] else -1.0
            brace_hand = hand_toward(W, W.t, Gf.Vec3d(px[W.name] + inward * 20, 0, W.pos[2] - zs * 30))
            ratchet = "R" if brace_hand == "L" else "L"
            brace = flat(Gf.Vec3d(px[W.name] + inward * 15.0, 127.0, zs * 33.6), Gf.Vec3d(0, 0, -zs))
            tt = W.t + 0.35
            for k in (0, 1):
                bp, axis = SC.side_bolt(side, px[W.name], k)
                tt = bolt(W, name, bp, axis, tt, ratchet, brace) + 0.3
            W.until(tt)
        # step back from the deck
        for W in (A, B):
            W.strafe((W.pos[0], W.pos[2] + zs * 24.0))

    A.walk([(-150, 300), (sx["A"], SC.STAND_Z + BACK["A"])], end_h=180.0, speed=85.0)
    B.walk([(150, 300), (sx["B"], SC.STAND_Z + BACK["B"])], end_h=180.0, speed=85.0)
    side_rail("L")
    A.walk([(-185, 110), (-185, -150), (-170, -300), (sx["A"], -SC.STAND_Z - BACK["A"])], end_h=0.0)
    B.walk([(205, 110), (205, -150), (175, -300), (sx["B"], -SC.STAND_Z - BACK["B"])], end_h=0.0)
    side_rail("R")

    # ---------------------------------------------------------------- 3: extension U-frame, from behind the back end
    PE = parts_def["ExtFrame"]
    s0 = PE.start[0]                                   # pivot in storage (bottom 40 on its stands)
    a_off, b_x = -34.0, (-51.0 - PE.pivot[0])          # A behind the end posts; B beside the left short rail
    b_z = PE.pivot[2] + SC.EXT_MID[1] + 33.0
    A.walk([(-250, -270), (-470, -250), (s0[0] + a_off - 40, s0[2]), (s0[0] + a_off, s0[2])], end_h=90.0, speed=85.0)
    B.walk([(-150, -110), (s0[0] + b_x, s0[2] + b_z)], end_h=180.0, speed=85.0)
    t = max(A.t, B.t) + 0.3
    for W in (A, B):
        W.until(t)
    y_carry, y_raise = 68.0, PE.pivot[1] + 2.0
    t_l0, t_l1 = t + 1.3, t + 2.7
    t_s1 = t_l1 + 0.3 + abs(s0[2]) / 40.0              # side-step clear of the storage stands
    t_c1 = t_s1 + 0.2 + 110.0 / 45.0                   # carry toward the back end
    t_r1 = t_c1 + 0.4 + 1.8                            # A lifts it to deck height
    t_i1 = t_r1 + 0.2 + 150.0 / 38.0                   # slides it in along the main rails
    t_seat = t_i1 + 0.7
    P0 = PE.at(s0)
    P1 = PE.at(Gf.Vec3d(s0[0], y_carry, s0[2]))
    P2 = PE.at(Gf.Vec3d(s0[0], y_carry, 0.0))
    P3 = PE.at(Gf.Vec3d(s0[0] + 110.0, y_carry, 0.0))
    P4 = PE.at(Gf.Vec3d(s0[0] + 110.0, y_raise, 0.0))
    P5 = PE.at(Gf.Vec3d(PE.pivot[0], y_raise, 0.0))
    tracks["ExtFrame"].segs += [("hold", 0.0, t_l0, P0), ("move", t_l0, t_l1, P0, P1), ("move", t_l1 + 0.3, t_s1, P1, P2),
                                ("move", t_s1 + 0.2, t_c1, P2, P3), ("move", t_c1 + 0.4, t_r1, P3, P4),
                                ("move", t_r1 + 0.2, t_i1, P4, P5), ("move", t_i1, t_seat, P5, PE.installed()),
                                ("hold", t_seat, 1e9, PE.installed())]
    piv = lambda tt: ctx0.part_m("ExtFrame", tt).Transform(PE.pivot)
    A.follow(t_l1 + 0.3, t_i1, lambda tt: ((piv(tt)[0] + a_off, piv(tt)[2]), 90.0))
    B.follow(t_l1 + 0.3, t_c1, lambda tt: ((piv(tt)[0] + b_x, piv(tt)[2] + b_z), 180.0))
    hA = PE.holds["A"]
    A.hold("L", t, t_seat + 0.3, grip("ExtFrame", hA["post1"]))     # post at -Z is on his left (facing +X)
    A.hold("R", t, t_seat + 0.3, grip("ExtFrame", hA["post0"]))
    A.look_at(t, t_seat, lambda tt, ctx: ctx.part_m("ExtFrame", tt).Transform(PE.pivot + UP * 50))
    B.hold("L", t, t_c1 + 0.3, grip("ExtFrame", PE.holds["B"]["front"]))
    B.hold("R", t, t_c1 + 0.3, grip("ExtFrame", PE.holds["B"]["back"]))
    B.look_at(t, t_c1 + 0.3, lambda tt, ctx: ctx.part_m("ExtFrame", tt).Transform(PE.pivot + Gf.Vec3d(60, 60, 0)))
    B.until(t_c1 + 0.4)
    B.strafe((B.pos[0] - 10.0, B.pos[2] + 30.0))       # hands off, step clear while A lifts it in
    A.until(t_seat + 0.4)
    tt = A.t + 0.2
    for z, other in ((SC.EXT_POSTS[0], "post1"), (SC.EXT_POSTS[1], "post0")):
        hand = hand_toward(A, tt, Gf.Vec3d(PE.pivot[0], 0, z))
        for k in (0, 1):
            bp, axis = SC.ext_bolt(z, k)
            tt = bolt(A, "ExtFrame", bp, axis, tt, hand, grip("ExtFrame", PE.holds["steady"][other], 0.9, False)) + 0.3
    A.until(tt + 0.2)
    A.strafe((A.pos[0] - 25.0, A.pos[2]))

    # ---------------------------------------------------------------- 4: front top crossbar: B brings it, A fits it
    PB = parts_def["CrossBar"]
    stand_low = SC.tread_point(SC.STEP_FRONT, 0)
    stand_top = SC.tread_point(SC.STEP_FRONT, 1)
    # A goes up the step stand at the entry end and turns round to take the bar
    A.walk([(-185, -120), (150, -125), (stand_low[0] + 38.0, 0.0)], end_h=-90.0, speed=90.0)
    A.climb(stand_low, 0.9)
    A.stand(0.15)
    A.climb(stand_top, 0.9)
    A.turn(90.0, speed=90.0)
    # B fetches it from the cart
    B.walk([(-160, 140), (190, 140), (330, 90), (342, 0)], end_h=-90.0, speed=90.0)
    t0 = B.t + 0.1
    bar0 = PB.at(*PB.start)
    bar_up = PB.at(PB.start[0] + UP * 14.0, PB.start[1])
    tracks["CrossBar"].segs += [("hold", 0.0, t0 + 1.2, bar0), ("move", t0 + 1.2, t0 + 2.1, bar0, bar_up)]
    B.until(t0 + 2.3)
    c0 = B.t
    B.walk([(330, 95), (240, 95), (stand_low[0] + 52.0, 72.0)], end_h=-120.0, speed=70.0)
    B.until(max(B.t, A.t - 0.8))                        # waits aside until he is up on the stand
    B.walk([(stand_low[0] + 32.0, 28.0)], end_h=-90.0, speed=40.0)
    c1 = B.t
    rel = carry_segment("CrossBar", B, c0, c1)
    m_c1 = rel * B.frame(c1)
    th = max(c1, A.t) + 0.2
    hand_over = PB.at(Gf.Vec3d(stand_low[0] + 4.0, 150.0, 10.0))
    B.until(th)
    h1 = th + 1.2                                       # B lifts it up to him
    tracks["CrossBar"].segs += [("hold", c1, th, m_c1), ("move", th, h1, m_c1, hand_over)]
    hbB, hbA = PB.holds["B"], PB.holds["A"]
    B.hold("L", t0, h1 + 0.8, grip("CrossBar", hbB["l"]))
    B.hold("R", t0, h1 + 0.8, grip("CrossBar", hbB["r"]))
    B.look_at(t0, h1 + 0.8, lambda tt, ctx: ctx.part_m("CrossBar", tt).Transform(PB.pivot))
    A.until(h1)
    # he takes it (his hands inboard of hers), she lets go, he turns back to the lift with it
    A.stand(0.9)
    c2 = A.t
    A.turn(-90.0, speed=80.0)
    c3 = A.t
    tracks["CrossBar"].segs += [("hold", h1, c2, hand_over)]
    rel = carry_segment("CrossBar", A, c2, c3)
    m_c3 = rel * A.frame(c3)
    # he turned round with it, so it goes on end-for-end (the bar is symmetric): no spin in his hands
    over = PB.at(PB.pivot + Gf.Vec3d(8.0, 6.0, 0.0), 180.0)
    above = PB.at(PB.pivot + Gf.Vec3d(0.0, 6.0, 0.0), 180.0)
    seated = PB.at(PB.pivot, 180.0)
    r1, i1, s1 = c3 + 1.3, c3 + 2.1, c3 + 2.8
    tracks["CrossBar"].segs += [("move", c3, r1, m_c3, over), ("move", r1, i1, over, above),
                                ("move", i1, s1, above, seated), ("hold", s1, 1e9, seated)]
    # facing +X at the hand-over his left hand is on the bar's -Z side
    A.hold("L", h1 - 0.3, s1 + 0.3, grip("CrossBar", hbA["r"]))
    A.hold("R", h1 - 0.3, s1 + 0.3, grip("CrossBar", hbA["l"]))
    A.look_at(h1 - 0.5, s1, lambda tt, ctx: ctx.part_m("CrossBar", tt).Transform(PB.pivot))
    B.until(h1 + 0.9)
    B.strafe((B.pos[0] + 20.0, B.pos[2] + 25.0))
    A.until(s1 + 0.4)
    tt = A.t + 0.2
    for zs in (1.0, -1.0):
        bp, axis = SC.bar_bolt(zs)
        hand = hand_toward(A, tt, bp)
        tt = bolt(A, "CrossBar", bp, axis, tt, hand, grip("CrossBar", PB.holds["steady"], 0.9, False)) + 0.3
    A.until(tt)
    A.climb(stand_low, 0.8)
    A.stand(0.1)
    A.climb(Gf.Vec3d(stand_low[0] + 30.0, 0.0, 0.0), 0.8)
    t_off_stand = A.t

    # ---------------------------------------------------------------- A moves the step stand to the right side
    PS = parts_def["StepStand"]
    sm0 = PS.at(*SC.STEP_FRONT)
    grab = sm0.Transform(PS.holds["L"]["c"])
    A.walk([(grab[0] + 28.0, 0.0)], end_h=-90.0, speed=40.0)
    t0 = A.t + 0.1
    lifted = PS.at(SC.STEP_FRONT[0] + UP * 63.0, SC.STEP_FRONT[1])
    tracks["StepStand"].segs += [("hold", 0.0, t0 + 1.1, sm0), ("move", t0 + 1.1, t0 + 2.0, sm0, lifted)]
    A.until(t0 + 2.0)
    c0 = A.t
    side_place = SC.STEP_SIDE
    put = PS.at(side_place[0] + UP * 63.0, side_place[1])
    drop_x, drop_z = side_place[0][0], side_place[0][2] - 14.0 - 28.0
    A.walk([(grab[0] + 28.0, -125.0), (60.0, -140.0), (drop_x, drop_z)], end_h=0.0, speed=65.0)
    t_front_clear = c0 + 2.5
    c1 = A.t
    rel = carry_segment("StepStand", A, c0, c1)
    m_c1 = rel * A.frame(c1)
    sd = PS.at(*side_place)
    tracks["StepStand"].segs += [("move", c1, c1 + 0.5, m_c1, put), ("move", c1 + 0.5, c1 + 1.3, put, sd),
                                 ("hold", c1 + 1.3, 1e9, sd)]
    for k in ("L", "R"):                                # each hand takes the side rail on its own side
        hand = hand_toward(A, t0, sm0.Transform(PS.holds[k]["c"]))
        A.hold(hand, t0, c1 + 1.4, grip("StepStand", PS.holds[k], 1.0, True))
    A.until(c1 + 1.6)

    # ---------------------------------------------------------------- 6 (A): platform controller from the parts table
    PC = parts_def["Controller"]
    cs = PC.start[0]
    handle0 = PC.at(*PC.start).Transform(SC.CTRL_HANDLE)
    A.walk([(handle0[0] - 40.0, -150.0), (handle0[0], handle0[2] + 42.0)], end_h=180.0, speed=70.0)
    t0 = A.t + 0.1
    ctrl0 = PC.at(*PC.start)
    ctrl_up = PC.at(cs + UP * 8.0, PC.start[1])
    tracks["Controller"].segs += [("hold", 0.0, t0 + 1.0, ctrl0), ("move", t0 + 1.0, t0 + 1.6, ctrl0, ctrl_up)]
    A.until(t0 + 1.8)
    c0 = A.t
    low = SC.tread_point(side_place, 0)
    top = SC.tread_point(side_place, 1)
    A.walk([(low[0] + 15.0, low[2] - 50.0), (low[0], low[2] - 36.0)], end_h=0.0, speed=60.0)
    A.climb(low, 0.9)
    A.stand(0.15)
    A.climb(top, 0.9)
    c1 = A.t
    rel = carry_segment("Controller", A, c0, c1)
    m_c1 = rel * A.frame(c1)
    hook_out = PC.at(PC.pivot + Gf.Vec3d(0.0, 6.0, -9.0))
    hook_up = PC.at(PC.pivot + Gf.Vec3d(0.0, 6.0, 0.0))
    r1, i1, s1 = c1 + 1.1, c1 + 1.8, c1 + 2.4
    tracks["Controller"].segs += [("move", c1, r1, m_c1, hook_out), ("move", r1, i1, hook_out, hook_up),
                                  ("move", i1, s1, hook_up, PC.installed()), ("hold", s1, 1e9, PC.installed())]
    A.hold("R", t0, s1 + 0.3, grip("Controller", PC.holds["handle"], 1.0, False))
    A.look_at(t0, s1 + 0.5, lambda tt, ctx: ctx.part_m("Controller", tt).Transform(PC.pivot + UP * 25))
    A.until(s1 + 0.8)
    A.climb(low, 0.8)
    A.stand(0.1)
    A.climb(Gf.Vec3d(low[0], 0.0, low[2] - 32.0), 0.8)

    # ---------------------------------------------------------------- 5 (B): entry gate, hung on its hinges and swung shut
    PG = parts_def["Gate"]
    B.walk([(262, 100)], end_h=-150.0, speed=60.0)       # out of his way while he takes the step stand
    B.until(max(B.t, t_front_clear - 0.5))
    B.walk([(232, 75), (226, 0)], end_h=90.0, speed=70.0)
    t0 = B.t + 0.1
    g0 = PG.at(*PG.start)
    g_up = PG.at(PG.start[0] + UP * 16.0, PG.start[1])
    tracks["Gate"].segs += [("hold", 0.0, t0 + 1.3, g0), ("move", t0 + 1.3, t0 + 2.3, g0, g_up)]
    B.until(t0 + 2.5)
    c0 = B.t
    B.turn(-90.0, speed=110.0)
    hinge = SC.GATE_HINGE

    def swing(theta, lift=0.0):
        return Gf.Matrix4d().SetTranslate(-hinge) * Gf.Matrix4d().SetRotate(Gf.Rotation(UP, theta)) * \
            Gf.Matrix4d().SetTranslate(hinge + UP * lift)
    OPEN = 25.0
    middle = Gf.Vec3d(SC.GATE_PIVOT[0], 0.0, SC.GATE_PIVOT[2])
    latch = Gf.Vec3d(SC.GATE_PIVOT[0], 0.0, SC.GATE_UPRIGHTS[0])
    stance = lambda m: m.Transform(middle) + m.TransformDir(Gf.Vec3d(1, 0, 0)) * 33.0
    m_open_up = swing(OPEN, 4.0)
    st_open = stance(m_open_up)
    B.walk([(st_open[0], st_open[2])], end_h=heading_to(m_open_up.Transform(middle) - st_open), speed=50.0)
    c1 = B.t
    rel = carry_segment("Gate", B, c0, c1)
    m_c1 = rel * B.frame(c1)
    h1, h2 = c1 + 1.4, c1 + 2.2
    sw0, sw1 = h2 + 0.6, h2 + 0.6 + 2.2
    tracks["Gate"].segs += [("move", c1, h1, m_c1, m_open_up), ("move", h1, h2, m_open_up, swing(OPEN)),
                            ("hold", h2, sw0, swing(OPEN)),
                            ("path", sw0, sw1, lambda tt: swing(OPEN * (1 - ease((tt - sw0) / (sw1 - sw0))))),
                            ("hold", sw1, 1e9, PG.installed())]
    hg = PG.holds["B"]
    lv = left_of(90.0)
    order = sorted(hg, key=lambda k: -Gf.Dot(g0.Transform(hg[k]["c"]), lv))
    B.hold("L", t0, h2 + 0.3, grip("Gate", hg[order[0]]))
    B.hold("R", t0, h2 + 0.3, grip("Gate", hg[order[1]]))
    push_hand = hand_toward(B, h2, swing(OPEN).Transform(latch))
    B.hold(push_hand, h2 + 0.5, sw1 + 0.6, grip("Gate", PG.holds["push"], 1.0, False))
    B.look_at(t0, sw1 + 0.5, lambda tt, ctx: ctx.part_m("Gate", tt).Transform(PG.pivot + UP * 30))
    B.follow(sw0, sw1, lambda tt: (lambda m, s: ((s[0], s[2]), heading_to(m.Transform(middle) - s)))(
        ctx0.part_m("Gate", tt), stance(ctx0.part_m("Gate", tt))))
    B.until(sw1 + 0.8)
    B.strafe((B.pos[0] + 25.0, B.pos[2]))

    # ---------------------------------------------------------------- wrap up: both step back and look the platform over
    B.walk([(250, 150)], end_h=-130.0, speed=70.0)
    A.walk([(-120, -200), (-260, -60), (-250, 150)], end_h=130.0, speed=75.0)
    end = max(A.t, B.t) + 0.3
    for W in (A, B):
        W.look_at(W.t, end + 5, Gf.Vec3d(0, 170, 0))
    end += 3.5
    for W in (A, B):
        W.until(end)
    return tracks, workers, end


# --------------------------------------------------------------------------------------------- hand targets
def free_target(rig, w, pose, side, swing, walk):
    """Relaxed arm at the side; swings with the gait."""
    f, l = rig.heading_dirs(pose)
    out = l if side == "L" else -l
    sh = rig.pos(w, side + "_Upperarm")
    L = sum(rig.len[side + "arm"])
    palm = sh - UP * (0.9 * L + rig.palm[side] * 0.9) + out * 7.0 + f * (3.0 + swing) + UP * (abs(swing) * 0.12)
    n = (-out * 0.9 + f * 0.1).GetNormalized()
    fd = (-UP + f * (0.12 + swing * 0.012)).GetNormalized()
    return {"kind": "flat", "c": palm + n * 2.0, "n": n, "f": fd, "curl": 0.32 + 0.1 * walk, "shrug": 0.0}


def hand_spec_at(W, side, t, ctx):
    """None (free), ("free", spec, weight) blended with the free arm, or ("mix", specA, specB, u) for a
    straight hand-over from one hold to the next."""
    hs = W.holds[side]
    cur = None
    for (t0, t1, fn) in hs:
        if t0 - BLEND <= t <= t1 + BLEND and (cur is None or t0 >= cur[0]):
            cur = (t0, t1, fn)
    if cur is None:
        return None
    t0, t1, fn = cur
    for (u0, u1, gn) in hs:
        if gn is fn:
            continue
        if abs(u0 - t1) < 0.35 and t1 - BLEND * 0.5 <= t <= u0 + BLEND * 0.5:
            u = ease((t - (t1 - BLEND * 0.5)) / (u0 - t1 + BLEND))
            return ("mix", fn(min(t, t1), ctx), gn(max(t, u0), ctx), u)
        if abs(u1 - t0) < 0.35 and u1 - BLEND * 0.5 <= t <= t0 + BLEND * 0.5:
            u = ease((t - (u1 - BLEND * 0.5)) / (t0 - u1 + BLEND))
            return ("mix", gn(min(t, u1), ctx), fn(max(t, t0), ctx), u)
    if t < t0:
        return ("free", fn(t0, ctx), ease(1 - (t0 - t) / BLEND))
    if t > t1:
        return ("free", fn(t1, ctx), ease(1 - (t - t1) / BLEND))
    return ("free", fn(t, ctx), 1.0)


def _mix_targets(a, b, u):
    """Blend two solved hand targets (palm point, finger dir, palm normal, curl)."""
    q = Gf.Slerp(u, _basis_q(a["f"], a["n"]), _basis_q(b["f"], b["n"]))
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(q))
    out = {"palm": a["palm"] + (b["palm"] - a["palm"]) * u, "f": Gf.Vec3d(R[0][0], R[0][1], R[0][2]),
           "n": Gf.Vec3d(R[1][0], R[1][1], R[1][2]), "curl": a["curl"] + (b["curl"] - a["curl"]) * u,
           "shrug": a["shrug"] + (b["shrug"] - a["shrug"]) * u, "pole": None}
    main = b if u >= 0.5 else a
    near = u > 0.8 or u < 0.2
    out["tube"] = main.get("tube") if near else None
    out["r"] = main.get("r", 1.6)
    if out["tube"] is None:
        out["curl"] = min(out["curl"], 0.55)
    return out


def _basis_q(f, n):
    return RG._basis(f, n).ExtractRotation().GetQuat()


# --------------------------------------------------------------------------------------------- posture
BENDS = np.arange(0.0, 72.0, 3.0)
CROUCH = np.arange(0.0, 44.0, 2.0)


class Posture:
    """Fast trunk-bend / crouch search: shoulder positions are tabulated per bend, crouch shifts them."""

    def __init__(self, rig):
        self.rig = rig
        tab = rig.shoulder_table(list(BENDS))
        self.sh = {}
        for k, s in enumerate("LR"):
            fo = np.array([row[k][0] for row in tab])[:, None] - 0.15 * CROUCH[None, :]
            up = np.array([row[k][1] for row in tab])[:, None] - CROUCH[None, :]
            le = np.array([row[k][2] for row in tab])[:, None] + 0 * CROUCH[None, :]
            self.sh[s] = (fo, up, le)
        self.comfy = {s: 0.93 * sum(rig.len[s + "arm"]) for s in "LR"}
        self.cmax = 0.42 * rig.hip_h

    def solve(self, p, h, wrists, load):
        """wrists: {side: wrist target}; returns (bend deg, crouch cm)."""
        if not wrists:
            return 0.0, 0.0
        f, l = fwd_of(h), left_of(h)
        short = np.zeros((len(BENDS), len(CROUCH)))
        for s, wr in wrists.items():
            d = wr - p
            df, du, dl = Gf.Dot(d, f), d[1], Gf.Dot(d, l)
            fo, up, le = self.sh[s]
            dist = np.sqrt((df - fo) ** 2 + (du - up) ** 2 + (dl - le) ** 2)
            short += np.maximum(0.0, dist - self.comfy[s])
        cost = 12.0 * short + (0.10 if load else 0.05) * BENDS[:, None] + 0.035 * CROUCH[None, :] + \
            0.0006 * CROUCH[None, :] ** 2
        cost[:, CROUCH > self.cmax] += 1e6
        i, j = np.unravel_index(np.argmin(cost), cost.shape)
        return float(BENDS[i]), float(CROUCH[j])


# --------------------------------------------------------------------------------------------- per-frame posing
def hand_target_for(rig, w, pose, W, side, t, ctx, swing, walk):
    free = free_target(rig, w, pose, side, swing, walk)
    sp = hand_spec_at(W, side, t, ctx)
    if sp is None:
        return rig.hand_target(pose, w, side, free)
    if sp[0] == "mix":
        return _mix_targets(rig.hand_target(pose, w, side, sp[1]), rig.hand_target(pose, w, side, sp[2]), sp[3])
    _, spec, u = sp
    tg = rig.hand_target(pose, w, side, spec)
    if u >= 1.0:
        return tg
    return _mix_targets(rig.hand_target(pose, w, side, free), tg, u)


def look_target(W, t, ctx):
    for (t0, t1, tg) in reversed(W.looks):
        if t0 <= t <= t1:
            return tg(t, ctx) if callable(tg) else tg
    p, h = W.root_at(t)
    p2, h2 = W.root_at(t + 0.8)
    if (p2 - p).GetLength() > 5:
        return p2 + fwd_of(h2) * 250 + UP * 140
    return None


def _smooth(vals, r):
    a = np.asarray(vals, dtype=float)
    if r <= 0 or len(a) == 0:
        return a
    k = np.ones(2 * r + 1) / (2 * r + 1)
    pad = np.pad(a, ((r, r),) + ((0, 0),) * (a.ndim - 1), mode="edge")
    if a.ndim == 1:
        return np.convolve(pad, k, mode="valid")
    return np.stack([np.convolve(pad[:, c], k, mode="valid") for c in range(a.shape[1])], axis=1)


_CYCLES = {}
_CLIPS = {}


def one_shot(W, t, loaded):
    """(clip, fractional frame, weight) for a turn on the spot or the first steps of a walk, else None."""
    for (t0, t1, d) in W.turns:
        if t0 - 0.1 <= t <= t1 + 0.1 and abs(d) >= 20:
            clip = _CLIPS.get("left" if d > 0 else "right")
            if not clip:
                return None
            u = min(max((t - t0) / (t1 - t0), 0.0), 1.0)
            k = next((i for i in range(clip.n) if abs(clip.turned[i]) >= abs(clip.turned[-1]) * 0.98), clip.n - 1)
            w = min(1.0, (t - t0 + 0.1) / 0.25, (t1 + 0.1 - t) / 0.25)
            return clip, u * k, max(0.0, w) * min(1.0, abs(d) / 60.0)
    if loaded:
        return None
    for (t0, p0) in reversed(W.starts):
        if t0 - 0.35 <= t <= t0 + 2.5:
            clip = _CLIPS.get("start_f" if W.rig.hip_h < 88 else "start_m")
            if not clip:
                return None
            p, _ = W.root_at(t)
            d = (p - p0).GetLength() * (clip.hip_mean[1] / W.rig.hip_h)
            if d > 95:
                return None
            x = clip.frame_at_travel(d) if t >= t0 else clip.move0 - (t0 - t) * clip.fps
            w = min(1.0, (t - t0 + 0.35) / 0.3) * min(1.0, max(0.0, (95 - d) / 40.0))
            return clip, x, w
    return None


def cycles():
    """Mixamo walk cycles (free walking, carrying a load), or {} when the clips are not on disk."""
    if not _CYCLES:
        try:
            _CYCLES.update(walk=MC.load("Walking"), carry=MC.load("Walkt_carrying"))
            _CLIPS.update(left=MC.load_clip("Left_Turn_90"), right=MC.load_clip("Right_Turn_90"),
                          start_m=MC.load_clip("Start_Walking"), start_f=MC.load_clip("Female_Start_Walking"))
        except (OSError, ValueError) as e:
            print("[assembly] mocap clips unavailable, procedural walk only:", e)
            _CYCLES["none"] = None
    return {k: v for k, v in _CYCLES.items() if v is not None}


def _loaded(W, t, ctx):
    for side in "LR":
        sp = hand_spec_at(W, side, t, ctx)
        if sp and sp[0] == "free" and sp[2] > 0.5 and sp[1].get("load"):
            return True
        if sp and sp[0] == "mix" and (sp[1].get("load") or sp[2].get("load")):
            return True
    return False


def pose_worker(rig, W, ctx, gait, posture, frames, progress=None, use_mocap=True):
    """Poses for one worker at the given frame numbers: list of (Pose, w)."""
    cyc = cycles() if use_mocap else {}
    phases = {k: MC.Phase(gait, c) for k, c in cyc.items()}
    mscale = {k: rig.hip_h / c.hip_mean[1] for k, c in cyc.items()}
    # pass 1: posture and gait trunk terms (over every frame so the smoothing is the same for any subset)
    n_all = max(frames) + 8
    raw_pc, body = [], []
    for fr in range(n_all):
        t = fr / FPS
        p, h = W.root_at(t)
        pose = RG.Pose()
        pose.root_pos, pose.root_yaw = p, h - rig.rest_heading
        w = None
        wr, load = {}, False
        for side in "LR":
            sp = hand_spec_at(W, side, t, ctx)
            if sp is None:
                continue
            spec = sp[2] if sp[0] == "mix" else sp[1]
            weight = 1.0 if sp[0] == "mix" else sp[2]
            if weight < 0.35:
                continue
            w = w or rig.fk(pose)
            tg = rig.hand_target(pose, w, side, spec)
            wr[side] = tg["palm"] - tg["f"] * rig.palm[side]
            load = load or bool(spec.get("load"))
        raw_pc.append(posture.solve(p, h, wr, load))
        g = gait.body(t)
        body.append([g["bob"], g["sway"][0], g["sway"][2], g["yaw"], g["arms"]["L"], g["arms"]["R"], g["walk"]])
    pc = _smooth(raw_pc, 7)
    body = _smooth(body, 4)

    def trunk(fr):
        t = fr / FPS
        bend, crouch = float(pc[fr][0]), float(pc[fr][1])
        bob, sx, sz, gyaw, armL, armR, walk = (float(v) for v in body[fr])
        p, h = W.root_at(t)
        pose = RG.Pose()
        pose.root_pos, pose.root_yaw = p, h - rig.rest_heading
        w = rig.fk(pose)
        f, l = rig.heading_dirs(pose)
        idle = (1.0 - walk) * 0.5
        key = "carry" if _loaded(W, t, ctx) else "walk"
        cy = cyc.get(key)
        wm = min(1.0, walk * 1.25) if cy else 0.0
        sample = cy.sample(phases[key].at(t)) if cy else None
        moc = MC.hip_offset(cy, sample, h, mscale[key]) if cy else Gf.Vec3d(0, 0, 0)
        sway = (Gf.Vec3d(sx, 0, sz) + UP * bob) * (1 - wm) + moc * wm + l * (idle * math.sin(2 * math.pi * t / 4.7))
        hip = UP * (-crouch - 1.5 * walk) + sway - f * (RG.hip_back(bend) + 0.15 * crouch)
        rig.set_hip(pose, w, hip)
        if cy:
            MC.apply(rig, pose, w, sample, h, wm)
        shot = one_shot(W, t, key == "carry")
        if shot:
            clip, x, ws = shot
            MC.apply(rig, pose, w, clip.sample_frame(x), h, ws)
            wm = max(wm, ws)
        lean = 3.0 * walk * (1 - wm) + 0.6 * math.sin(2 * math.pi * t / 3.9) * (1 - walk)
        rig.bend(pose, w, bend + lean, twist_deg=-gyaw * 1.5 * (1 - wm), pelvis_yaw=gyaw * (1 - wm))
        feet = {s: gait.foot(s, t) for s in "LR"}
        return pose, w, hip, feet, (armL, armR, walk, wm, sample, key, shot)

    # the hips sink where the legs could not reach the feet: smooth that so the pelvis eases down and up
    lo, hi = max(0, min(frames) - 10), min(n_all - 1, max(frames) + 10)
    drops = np.zeros(n_all)
    for fr in range(lo, hi + 1):
        pose, w, hip, feet, _ = trunk(fr)
        drops[fr] = rig.leg_reach_drop(w, {s: feet[s][0] for s in "LR"})
    mx = np.array([drops[max(0, i - 2):i + 3].max() for i in range(n_all)])
    drops_s = _smooth(mx, 2)
    out = []
    for k, fr in enumerate(frames):
        t = fr / FPS
        pose, w, hip, feet, (armL, armR, walk, wm, sample, key, shot) = trunk(fr)
        hip = hip - UP * float(drops_s[fr])
        rig.set_hip(pose, w, hip)
        drop = rig.leg_reach_drop(w, {s: feet[s][0] for s in "LR"})
        if drop > 0:
            rig.set_hip(pose, w, hip - UP * drop)
        for s in "LR":
            a, fh, pitch, toe, _ = feet[s]
            rig.leg(pose, w, s, a, fh, pitch, toe)
        for side in "LR":
            tg = hand_target_for(rig, w, pose, W, side, t, ctx, armL if side == "L" else armR, walk)
            rig.reach(pose, w, side, tg)
            if hand_spec_at(W, side, t, ctx) is None:
                hh = W.root_at(t)[1]
                if sample is not None and key == "walk" and wm > 0.02:
                    MC.apply(rig, pose, w, sample, hh, min(1.0, walk * 1.25), arms=(side,), trunk=False)
                if shot:
                    MC.apply(rig, pose, w, shot[0].sample_frame(shot[1]), hh, shot[2], arms=(side,), trunk=False)
        lt = look_target(W, t, ctx)
        if lt is not None:
            rig.look(pose, w, lt, 0.8)
        out.append((pose, w))
        if progress and k % 300 == 0:
            progress(f"  {W.name}: frame {fr}")
    return out


# --------------------------------------------------------------------------------------------- bake
def bake(stage, parts_def, rigs, progress=print, script_fn=None, kit=None):
    """Bake a product's assembly: script_fn(parts_def, rigs) -> (tracks, workers, end) (default: the ES2632 rails);
    bolts are written under `kit` (default: the ES2632 kit path)."""
    tracks, workers, end = (script_fn or script)(parts_def, rigs)
    ctx = Ctx(tracks, workers)
    n = int(end * FPS) + 1
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(n - 1)
    layer = stage.GetEditTarget().GetLayer()
    for name, tr in tracks.items():
        op = UsdGeom.Xformable(stage.GetPrimAtPath(parts_def[name].path)).GetOrderedXformOps()[0]
        for f in range(n):
            op.Set(tr.matrix(f / FPS, workers), f)
    SC.write_bolts(stage, list(BOLT_EVENTS), n, kit=kit or SC.KIT)
    results = {}
    for name, W in workers.items():
        rig = rigs[name]
        gait = GT.Gait(rig, W.root_at, end, floor=floor_fn(ctx))
        poses = pose_worker(rig, W, ctx, gait, Posture(rig), list(range(n)), progress)
        results[name] = gait
        anim_path = rig.prim.GetParent().GetPath().AppendChild("AssemblyAnim")
        scratch = Sdf.Layer.CreateAnonymous("assembly")
        tmp = Usd.Stage.Open(scratch)
        anim = UsdSkel.Animation.Define(tmp, anim_path)
        anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
        attrs = [anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr()]
        for f, (pose, w) in enumerate(poses):
            for a, v in zip(attrs, rig.arrays(pose, w)):
                scratch.SetTimeSample(a.GetPath(), float(f), v)
        Sdf.CreatePrimInLayer(layer, anim_path.GetParentPath())
        Sdf.CopySpec(scratch, anim_path, layer, anim_path)
        UsdSkel.BindingAPI.Apply(rig.prim).GetAnimationSourceRel().SetTargets([anim_path])
        write_wrench(stage, name, W, ctx, n)
    return end, n, results


def write_wrench(stage, name, W, ctx, n):
    """The ratchet: visible only while in use, its head on the bolt."""
    wr = stage.GetPrimAtPath(f"/World/Tools/Wrench{name}")
    xf = UsdGeom.Xformable(wr)
    ops = xf.GetOrderedXformOps()
    wr_op = ops[0] if ops else xf.AddTransformOp()
    vis = UsdGeom.Imageable(wr).GetVisibilityAttr()
    for f in range(n):
        t = f / FPS
        ctx.wrench.pop(name, None)
        for (t0, t1, fn) in W.holds["L"] + W.holds["R"]:     # the ratchet can be in either hand
            if t0 <= t <= t1:
                fn(t, ctx)
        wv = ctx.wrench.get(name)
        if wv:
            _, origin, d, axis = wv
            x = axis.GetNormalized()
            z = d.GetNormalized()
            y = Gf.Cross(z, x)
            wr_op.Set(Gf.Matrix4d(x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0,
                                  origin[0], origin[1], origin[2], 1), f)
            vis.Set("inherited", f)
        else:
            vis.Set("invisible", f)
