"""Choreography for the in-ground vehicle lift: head frame onto the column, runway L, runway R (each bolted), check.
Built from the same pieces as the ES2632 demo (choreo.Worker / PartTrack / grips / ratchet rule / bolts)."""
import math

from pxr import Gf

from . import choreo as C
from . import scene_carlift as CL
from .rig import left_of

UP = Gf.Vec3d(0, 1, 0)
ease = C.ease

DIRECTOR_KEYS = [
    (0.0, (560, 330, -780), (0, 90, -200)),
    (6.0, (300, 210, -690), (0, 80, -250)),
    (11.0, (560, 230, -300), (0, 90, -120)),
    (16.5, (470, 230, 140), (0, 95, -20)),
    (23.0, (-450, 260, 240), (0, 90, 150)),
    (26.5, (-360, 240, 660), (0, 90, 280)),
    (30.0, (-280, 210, 740), (0, 85, 330)),
    (36.0, (390, 230, 520), (0, 100, 150)),
    (41.5, (-330, 200, 330), (-145, 110, 100)),
    (44.5, (330, 200, 330), (145, 110, 100)),
    (51.0, (-660, 260, -260), (0, 90, -200)),
    (60.0, (150, 210, -730), (0, 85, -350)),
    (66.0, (-400, 230, -540), (0, 100, -150)),
    (74.0, (-330, 210, -330), (-145, 110, -100)),
    (77.0, (330, 210, -330), (145, 110, -100)),
    (85.0, (650, 280, 200), (0, 100, 0)),
    (92.0, (-640, 300, 260), (0, 100, 0)),
    (98.7, (690, 330, 300), (0, 95, 0)),
]


def script(parts_def, rigs):
    C.BOLT_EVENTS.clear()
    A = C.Worker("A", rigs["A"], (-320, -520), 0.0)
    B = C.Worker("B", rigs["B"], (320, -520), 0.0)
    workers = {"A": A, "B": B}
    tracks = {n: C.PartTrack(p.at(*p.start)) for n, p in parts_def.items()}
    ctx0 = C.Ctx(tracks, workers)

    def hand_toward(W, t, point):
        p, h = W.root_at(t)
        return "L" if Gf.Dot(Gf.Vec3d(point) - p, left_of(h)) > 0 else "R"

    def bolt(W, part, bp, axis, t0, hand, steady=None, dur=2.5):
        other = "R" if hand == "L" else "L"
        _, h = W.root_at(t0)
        side = left_of(h) * (1.0 if hand == "L" else -1.0)
        side = side - axis * Gf.Dot(side, axis)
        side = side.GetNormalized() if side.GetLength() > 1e-6 else -UP
        a = math.radians(20.0)
        handle = side * math.cos(a) - UP * math.sin(a)
        handle = (handle - axis * Gf.Dot(handle, axis)).GetNormalized()
        W.hold(hand, t0, t0 + dur, C.wrench_spec(W.name, bp, axis, t0, handle))
        if steady:
            W.hold(other, t0 - 0.3, t0 + dur, steady)
        W.look_at(t0, t0 + dur, bp)
        C.BOLT_EVENTS.append((part, bp, axis, t0, t0 + dur))
        return t0 + dur

    def hands_on(W, name, holds, t0, t1, face):
        """Two holds on a part: each hand takes the one on its own side."""
        lv = left_of(face)
        m = tracks[name].matrix(t0, workers)
        ka, kb = sorted(("a", "b"), key=lambda k: -Gf.Dot(m.Transform(holds[k]["c"]), lv))
        W.hold("L", t0, t1, C.grip(name, holds[ka]))
        W.hold("R", t0, t1, C.grip(name, holds[kb]))

    # ---------------------------------------------------------------- 1: head frame onto the column (both)
    PH = parts_def["Head"]
    hp = PH.start[0]
    off = CL.HEAD_END_X + 40.0                            # carriers stand outside the end beams
    A.walk([(-330, -420), (-off, hp[2])], end_h=90.0, speed=85.0)
    B.walk([(330, -420), (off, hp[2])], end_h=-90.0, speed=85.0)
    t = max(A.t, B.t) + 0.3
    for W in (A, B):
        W.until(t)
    y_carry = CL.COLUMN_TOP + 8.0
    t_l0, t_l1 = t + 1.4, t + 3.1
    t_s1 = t_l1 + 0.3 + abs(hp[2]) / 40.0
    t_seat = t_s1 + 1.2
    m = lambda y, z: PH.at(Gf.Vec3d(0.0, y, z))
    tracks["Head"].segs += [("hold", 0.0, t_l0, m(hp[1], hp[2])), ("move", t_l0, t_l1, m(hp[1], hp[2]), m(y_carry, hp[2])),
                            ("move", t_l1 + 0.3, t_s1, m(y_carry, hp[2]), m(y_carry, 0.0)),
                            ("move", t_s1, t_seat, m(y_carry, 0.0), PH.installed()), ("hold", t_seat, 1e9, PH.installed())]
    piv = lambda tt: ctx0.part_m("Head", tt).Transform(PH.pivot)
    A.follow(t_l1 + 0.3, t_s1, lambda tt: ((-off, piv(tt)[2]), 90.0))
    B.follow(t_l1 + 0.3, t_s1, lambda tt: ((off, piv(tt)[2]), -90.0))
    hands_on(A, "Head", PH.holds["A"], t, t_seat + 0.3, 90.0)
    hands_on(B, "Head", PH.holds["B"], t, t_seat + 0.3, -90.0)
    for W in (A, B):
        W.look_at(t, t_seat, lambda tt, ctx: ctx.part_m("Head", tt).Transform(PH.pivot + UP * 10))
        W.until(t_seat + 0.6)
    A.strafe((A.pos[0] - 20.0, A.pos[2]))
    B.strafe((B.pos[0] + 20.0, B.pos[2]))

    # ---------------------------------------------------------------- 2-3: runways (both), bolted as they go on
    def runway(side, a_route, b_route):
        zs = 1.0 if side == "L" else -1.0
        name = "Runway" + side
        P = parts_def[name]
        rp = P.start[0]
        face = 180.0 if side == "L" else 0.0
        outer = CL.RAIL_OUT - CL.RUNWAY_Z
        stand_z = lambda z: z + zs * (outer + 30.0)
        A.walk(a_route + [(-CL.GRIP_X, stand_z(rp[2]))], end_h=face, speed=85.0)
        B.walk(b_route + [(CL.GRIP_X, stand_z(rp[2]))], end_h=face, speed=85.0)
        t = max(A.t, B.t) + 0.3
        for W in (A, B):
            W.until(t)
        y0, y_carry, y_raise = rp[1], 88.0, 109.0
        z_near, z_in = zs * 152.0, zs * CL.RUNWAY_Z
        t_l0, t_l1 = t + 1.4, t + 3.0
        t_c0 = t_l1 + 0.3
        t_c1 = t_c0 + abs(rp[2] - z_near) / 50.0
        t_r1 = t_c1 + 1.4
        t_i1 = t_r1 + 1.2
        t_seat = t_i1 + 0.8
        mm = lambda y, z: P.at(Gf.Vec3d(0.0, y, z))
        tracks[name].segs += [("hold", 0.0, t_l0, mm(y0, rp[2])), ("move", t_l0, t_l1, mm(y0, rp[2]), mm(y_carry, rp[2])),
                              ("move", t_c0, t_c1, mm(y_carry, rp[2]), mm(y_carry, z_near)),
                              ("move", t_c1, t_r1, mm(y_carry, z_near), mm(y_raise, z_near)),
                              ("move", t_r1, t_i1, mm(y_raise, z_near), mm(y_raise, z_in)),
                              ("move", t_i1, t_seat, mm(y_raise, z_in), P.installed()), ("hold", t_seat, 1e9, P.installed())]
        zp = lambda tt: ctx0.part_m(name, tt).Transform(P.pivot)[2]
        A.follow(t_c0, t_i1, lambda tt: ((-CL.GRIP_X, stand_z(zp(tt))), face))
        B.follow(t_c0, t_i1, lambda tt: ((CL.GRIP_X, stand_z(zp(tt))), face))
        for W in (A, B):
            hands_on(W, name, P.holds[W.name], t, t_seat + 0.3, face)
            W.look_at(t, t_seat, lambda tt, ctx, n=name: ctx.part_m(n, tt).Transform(P.pivot + UP * 5))
        # bolts: each squares up between his / her two bolts; ratchet in the hand on the bolt's side, the other palm
        # flat on the rail top beside it (never across the body)
        for W, xs in ((A, (-170.0, -120.0)), (B, (120.0, 170.0))):
            W.until(t_seat + 0.45)
            W.strafe(((xs[0] + xs[1]) / 2.0, W.pos[2]), speed=35.0)
            tt = W.t + 0.3
            for x in xs:
                bp, axis = CL.runway_bolt(side, x)
                hand = hand_toward(W, tt, bp)
                _, h = W.root_at(tt)
                away = left_of(h) * (-1.0 if hand == "L" else 1.0)
                brace = bp + away * 24.0 + UP * 0.4
                tt = bolt(W, name, bp, axis, tt, hand, C.flat(brace, -UP, Gf.Vec3d(0, 0, -zs))) + 0.3
            W.until(tt)
        for W in (A, B):
            W.strafe((W.pos[0], W.pos[2] + zs * 22.0))

    runway("L", [(-330, -60), (-330, 300)], [(330, -60), (330, 300)])
    runway("R", [(-345, 120), (-345, -300)], [(345, 120), (345, -300)])

    # ---------------------------------------------------------------- 4: check: walk the runways' length and look them over
    A.walk([(-300, -140), (-300, 0)], end_h=90.0, speed=70.0)
    B.walk([(300, -140), (300, 0)], end_h=-90.0, speed=70.0)
    tt = max(A.t, B.t)
    for W in (A, B):
        W.look_at(tt, tt + 3.0, Gf.Vec3d(0, 105, 0))
        W.until(tt + 3.0)
    A.walk([(-320, 260)], end_h=140.0, speed=70.0)
    B.walk([(320, 260)], end_h=-140.0, speed=70.0)
    end = max(A.t, B.t) + 0.3
    for W in (A, B):
        W.look_at(W.t, end + 5, Gf.Vec3d(0, 110, 0))
    end += 3.0
    for W in (A, B):
        W.until(end)
    return tracks, workers, end
