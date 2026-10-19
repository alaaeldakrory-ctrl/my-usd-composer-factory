"""Choreography for the two-level shelving rack (NVIDIA RackSmallEmpty_A2): the way a bay really goes up.

    1-2  end frames L / R walked up from the floor (both workers, one per upright, hand over hand up the uprights)
    3    lower beams hooked onto the uprights' front faces, dropped onto their hooks and bolted (A front, B rear)
    4    lower deck: A swings it up by its front edge, rests the back edge on the front beam and slides it across,
         B takes the back edge at the rear and both lower it into the beams' steps
    5    upper beams (as 3)
    6    upper deck (as 4), then both step back and look the bay over
Built from the ES2632 pieces (choreo.Worker / PartTrack / grips / ratchet rule / bolts).
"""
import math

from pxr import Gf

from . import choreo as C
from . import scene_rack as SR
from .rig import left_of

UP = Gf.Vec3d(0, 1, 0)
ease = C.ease
DIRECTOR_KEYS = []          # filled by script(): (t, eye, look-at)
HAND_H = {"A": 166.0, "B": 142.0}     # hand height while walking a frame up
LIFT_Z = {"A": 76.0, "B": -71.0}       # each lifter walks beside his / her upright, outboard
REACH = {"A": 30.0, "B": 24.0}          # pelvis to the hands, horizontally


def theta_profile(t0):
    """Frame angle (deg) over time while walked up: off the block, overhead, hand over hand, last push."""
    t1, t2, t3 = t0 + 2.4, t0 + 7.6, t0 + 9.0
    th1, th2 = 31.0, 82.0

    def th(t):
        if t <= t0:
            return SR.THETA0
        if t <= t1:
            return SR.THETA0 + (th1 - SR.THETA0) * ease((t - t0) / (t1 - t0))
        if t <= t2:
            u = (t - t1) / (t2 - t1)
            return th1 + (th2 - th1) * (0.5 * u + 0.5 * ease(u))
        if t <= t3:
            return th2 + (90.0 - th2) * ease((t - t2) / (t3 - t2))
        return 90.0
    return th, t3


def script(parts_def, rigs):
    C.BOLT_EVENTS.clear()
    DIRECTOR_KEYS.clear()
    A = C.Worker("A", rigs["A"], (-330, 150), -90.0)
    B = C.Worker("B", rigs["B"], (-330, -150), -90.0)
    workers = {"A": A, "B": B}
    tracks = {n: C.PartTrack(SR.start_matrix(n, p)) for n, p in parts_def.items()}
    ctx0 = C.Ctx(tracks, workers)

    def shot(t, eye, at):
        DIRECTOR_KEYS.append((t, eye, at))

    def hand_toward(W, t, point):
        p, h = W.root_at(t)
        return "L" if Gf.Dot(Gf.Vec3d(point) - p, left_of(h)) > 0 else "R"

    def carry_segment(name, W, t0, t1):
        rel = tracks[name].matrix(t0, workers) * W.frame(t0).GetInverse()
        tracks[name].segs.append(("carry", t0, t1, W.name, rel))
        return rel

    def bolt(W, part, bp, axis, t0, hand, steady=None, dur=2.2):
        other = "R" if hand == "L" else "L"
        _, h = W.root_at(t0)
        side = left_of(h) * (1.0 if hand == "L" else -1.0)
        side = side - axis * Gf.Dot(side, axis)
        side = side.GetNormalized() if side.GetLength() > 1e-6 else -UP
        a = math.radians(22.0)
        handle = side * math.cos(a) - UP * math.sin(a)
        handle = (handle - axis * Gf.Dot(handle, axis)).GetNormalized()
        W.hold(hand, t0, t0 + dur, C.wrench_spec(W.name, bp, axis, t0, handle))
        if steady:
            W.hold(other, t0 - 0.3, t0 + dur, steady)
        W.look_at(t0, t0 + dur, bp)
        C.BOLT_EVENTS.append((part, bp, axis, t0, t0 + dur))
        return t0 + dur

    def hands_on(W, name, ha, hb, t0, t1, face, curl=1.0, load=True):
        """Two holds on a part: each hand takes the one on its own side."""
        lv = left_of(face)
        m = tracks[name].matrix(t0, workers)
        a, b = sorted((ha, hb), key=lambda hd: -Gf.Dot(m.Transform(hd["c"]), lv))
        W.hold("L", t0, t1, C.grip(name, a, curl, load))
        W.hold("R", t0, t1, C.grip(name, b, curl, load))

    # ---------------------------------------------------------------- 1-2: walk the end frames up
    def frame_up(side, routes):
        s = -1.0 if side == "L" else 1.0
        name = "Frame" + side
        face = 90.0 if side == "L" else -90.0             # lifters face the foot of the frame
        d_max = {"A": 284.0, "B": 278.0}

        def d_of(who, th):
            r = math.radians(th)
            return min(d_max[who], (HAND_H[who] - 6.4 * math.cos(r)) / max(math.sin(r), 1e-3))

        def root(who, th):
            c = SR.frame_m(side, th).Transform(SR.upright_point(side, d_of(who, th), 0.0))
            return (c[0] + s * REACH[who], LIFT_Z[who])

        for W in (A, B):
            W.walk(routes[W.name] + [root(W.name, SR.THETA0)], end_h=face, speed=85.0)
        t = max(A.t, B.t) + 0.4
        for W in (A, B):
            W.until(t)
        th, t_up = theta_profile(t + 0.9)
        tracks[name].segs += [("hold", 0.0, t + 0.9, SR.frame_m(side, SR.THETA0)),
                              ("path", t + 0.9, t_up, lambda tt: SR.frame_m(side, th(tt))),
                              ("hold", t_up, 1e9, SR.frame_m(side, 90.0))]
        for W in (A, B):
            W.follow(t + 0.9, t_up, lambda tt, w=W.name: (root(w, th(tt)), face))
        for W in (A, B):
            zc = math.copysign(SR.UPRIGHT_Z[1] - 5.0, LIFT_Z[W.name])
            on_left = Gf.Dot(Gf.Vec3d(0, 0, zc - LIFT_Z[W.name]), left_of(face)) > 0
            upper, lower = ("R", "L") if on_left else ("L", "R")
            for hand, back in ((upper, 0.0), (lower, 24.0)):
                def spec(tt, ctx, who=W.name, back=back, zc=zc):
                    m = ctx.part_m(name, tt)
                    d = d_of(who, th(tt)) - back
                    return {"kind": "tube", "c": m.Transform(SR.upright_point(side, d, zc)), "axis": m.TransformDir(UP),
                            "thumb": -m.TransformDir(UP), "r": 6.0, "curl": 1.0, "load": True}
                W.hold(hand, t, t_up + 0.5, spec)
            W.look_at(t, t_up + 0.5, lambda tt, ctx: ctx.part_m(name, tt).Transform(
                SR.upright_point(side, 200.0, 0.0)))
        for W in (A, B):
            W.until(t_up + 0.8)
            W.strafe((W.pos[0] + s * 18.0, W.pos[2] + math.copysign(12.0, W.pos[2])))
        return t, t_up

    t1a, t1b = frame_up("L", {"A": [], "B": []})
    shot(0.0, (-560, 260, 520), (-260, 60, 0))
    shot(t1a, (-640, 210, 360), (-250, 80, 0))
    shot(t1b, (-520, 240, 470), (-120, 150, 0))
    t2a, t2b = frame_up("R", {"A": [(-60, 88), (330, 88)], "B": [(-60, -88), (330, -88)]})
    shot(t1b + 4.0, (180, 260, 620), (150, 90, 0))
    shot(t2a, (640, 210, 380), (250, 80, 0))
    shot(t2b, (520, 250, 480), (100, 150, 0))

    # ---------------------------------------------------------------- 3 / 5: beams (A front, B rear, at the same time)
    def beam_job(W, name, route):
        P = parts_def[name]
        zs = 1.0 if name.endswith("F") else -1.0
        face = 180.0 if zs > 0 else 0.0
        x_s, z_s = SR.BEAM_STORE[name]
        W.walk(route + [(x_s, z_s + zs * 40.0)], end_h=face, speed=85.0)
        t0 = W.t + 0.1
        m0 = SR.start_matrix(name, P)
        lifted = P.at(P.start[0] + UP * 12.0)
        tracks[name].segs += [("hold", 0.0, t0 + 0.9, m0), ("move", t0 + 0.9, t0 + 1.7, m0, lifted)]
        W.until(t0 + 1.8)
        c0 = W.t
        W.walk([(x_s * 0.4, zs * (SR.BEAM_Z + 110.0)), (0.0, zs * (SR.BEAM_Z + 38.0))], end_h=face, speed=60.0)
        c1 = W.t
        rel = carry_segment(name, W, c0, c1)
        m_c1 = rel * W.frame(c1)
        over = P.at(P.pivot + Gf.Vec3d(0.0, 6.0, zs * 9.0))
        above = P.at(P.pivot + Gf.Vec3d(0.0, 6.0, 0.0))
        r1, i1, s1 = c1 + 1.4, c1 + 2.1, c1 + 2.7
        tracks[name].segs += [("move", c1, r1, m_c1, over), ("move", r1, i1, over, above),
                              ("move", i1, s1, above, P.installed()), ("hold", s1, 1e9, P.installed())]
        hands_on(W, name, P.holds["a"], P.holds["b"], t0, s1 + 0.3, face)
        W.look_at(t0, s1, lambda tt, ctx: ctx.part_m(name, tt).Transform(P.pivot))
        W.until(s1 + 0.4)
        # each end: square up beside it, ratchet in the hand on the bolt's side, other palm flat on the beam top
        for xs in (-1.0, 1.0):
            W.strafe((xs * 64.0, W.pos[2]), speed=40.0)
            tt = W.t + 0.25
            bp, axis = SR.beam_bolt(name, xs)
            hand = hand_toward(W, tt, bp)
            brace = P.holds["brace"][xs] + Gf.Vec3d(-xs * 24.0, 0, 0)
            tt = bolt(W, name, bp, axis, tt, hand, C.flat(brace, -UP, Gf.Vec3d(0, 0, -zs))) + 0.3
            W.until(tt)
        W.strafe((W.pos[0] * 0.5, W.pos[2] + zs * 25.0))
        return s1

    # ---------------------------------------------------------------- 4 / 6: decks (A swings it up and slides it, B takes the back)
    def deck_job(level, a_route, b_route):
        name = "Deck" + level
        P = parts_def[name]
        D = SR.deck_pivot(level)
        hx, hz = SR.DECK_STORE[name]
        A.walk(a_route + [(hx, hz + 12.0 + 30.0)], end_h=180.0, speed=85.0)
        t0 = A.t + 0.1
        m0 = SR.start_matrix(name, P)
        m_up = SR.deck_m(level, Gf.Vec3d(hx, SR.DECK_STORE_Y + 8.0, hz), 0.0)
        tracks[name].segs += [("hold", 0.0, t0 + 0.9, m0), ("move", t0 + 0.9, t0 + 1.6, m0, m_up)]
        A.until(t0 + 1.7)
        c0 = A.t
        hinge_z = 140.0
        A.walk([(hx + 80.0, 220.0), (0.0, hinge_z + 30.0)], end_h=180.0, speed=55.0)
        c1 = A.t
        rel = carry_segment(name, A, c0, c1)
        m_c1 = rel * A.frame(c1)
        top = SR.BEAM_TOP[level]
        hy_swing = top + 38.0 if level == "Up" else 110.0
        y_back = lambda al: SR.deck_m(level, Gf.Vec3d(0, 0, 0), al).Transform(D + Gf.Vec3d(0, -12.0, -SR.DECK_DEPTH))[1]
        hy_land = top + 1.0 - y_back(85.0)
        s0 = c1 + 1.2
        s1 = s0 + 1.8
        l1 = s1 + 0.8
        sl1 = l1 + 3.4
        seat = sl1 + 1.3
        swing = lambda tt: SR.deck_m(level, Gf.Vec3d(0, hy_swing, hinge_z), 85.0 * ease((tt - s0) / (s1 - s0)))

        def slide(tt):
            u = ease((tt - l1) / (sl1 - l1))
            al = 85.0 + 5.0 * u
            z = hinge_z + (SR.DECK_EDGE - hinge_z) * u
            return SR.deck_m(level, Gf.Vec3d(0, top + 1.0 - y_back(al), z), al)
        tracks[name].segs += [("move", c1, s0, m_c1, SR.deck_m(level, Gf.Vec3d(0, hy_swing, hinge_z), 0.0)),
                              ("path", s0, s1, swing),
                              ("move", s1, l1, swing(s1), SR.deck_m(level, Gf.Vec3d(0, hy_land, hinge_z), 85.0)),
                              ("path", l1, sl1, slide),
                              ("move", sl1, seat, slide(sl1), P.installed()), ("hold", seat, 1e9, P.installed())]
        hinge = lambda tt: ctx0.part_m(name, tt).Transform(D)
        A.follow(c1, seat, lambda tt: ((0.0, hinge(tt)[2] + 30.0), 180.0))
        hands_on(A, name, P.holds["front_a"], P.holds["front_b"], t0, seat + 0.3, 180.0)
        A.look_at(t0, seat, lambda tt, ctx: ctx.part_m(name, tt).Transform(D + Gf.Vec3d(0, 0, -40.0)))
        # B waits at the back and takes the back edge as it comes over the rear beam
        B.walk(b_route + [(0.0, -(SR.DECK_EDGE + (27.0 if level == "Up" else 34.0)))], end_h=0.0, speed=80.0)
        tb = l1 + 0.72 * (sl1 - l1)
        B.until(max(B.t, l1))
        B.look_at(l1, seat, lambda tt, ctx: ctx.part_m(name, tt).Transform(D + Gf.Vec3d(0, 0, -SR.DECK_DEPTH)))
        hands_on(B, name, P.holds["rear_a"], P.holds["rear_b"], tb, seat + 0.3, 0.0)
        for W in (A, B):
            W.until(seat + 0.6)
        A.strafe((A.pos[0], A.pos[2] + 25.0))
        B.strafe((B.pos[0], B.pos[2] - 25.0))
        return c1, seat

    t3 = max(A.t, B.t)
    sA = beam_job(A, "BeamLowF", [(240, 160), (250, 300)])
    sB = beam_job(B, "BeamLowR", [(240, -160), (250, -300)])
    shot(t3 + 3.0, (520, 300, 680), (0, 60, 100))
    shot(sA - 4.0, (-330, 170, 380), (0, 50, 30))
    shot(sA + 4.0, (440, 250, 280), (0, 60, -10))
    shot(max(A.t, B.t) - 6.0, (400, 180, -390), (30, 50, -40))
    t4 = max(A.t, B.t)
    c4, seat4 = deck_job("Low", [(-150, 180)], [(150, -160)])
    shot(t4 - 1.0, (560, 260, 120), (0, 70, 40))
    shot(t4 + 3.0, (-60, 280, 760), (-200, 90, 220))
    shot(c4, (-520, 260, 520), (0, 70, 60))
    shot(seat4 - 2.0, (-330, 180, 380), (0, 60, 0))
    t5 = max(A.t, B.t)
    beam_job(A, "BeamUpF", [(260, 150), (260, 300)])
    beam_job(B, "BeamUpR", [(220, -140), (250, -300)])
    shot(t5 + 6.0, (560, 300, 160), (0, 110, 40))
    shot(max(A.t, B.t) - 12.0, (-330, 220, 380), (-40, 130, 40))
    shot(max(A.t, B.t) - 8.0, (-460, 240, 0), (0, 130, 0))
    shot(max(A.t, B.t) - 4.0, (-330, 220, -390), (40, 130, -40))
    t6 = max(A.t, B.t)
    c6, seat6 = deck_job("Up", [(-150, 180)], [(150, -150)])
    shot(t6 - 1.0, (-520, 260, -120), (0, 110, 0))
    shot(t6 + 3.0, (-150, 300, 780), (-220, 110, 220))
    shot(c6, (-520, 280, 520), (0, 150, 60))
    shot(seat6 - 2.0, (340, 250, 380), (0, 130, 0))

    # ---------------------------------------------------------------- wrap up: both step out front and look the bay over
    A.walk([(-120, 200)], end_h=160.0, speed=70.0)
    B.walk([(200, -130), (220, 120), (130, 210)], end_h=-160.0, speed=75.0)
    end = max(A.t, B.t) + 0.3
    for W in (A, B):
        W.look_at(W.t, end + 5, Gf.Vec3d(0, 110, 0))
    end += 3.5
    for W in (A, B):
        W.until(end)
    shot(seat6 + 3.0, (480, 230, 560), (0, 110, 0))
    shot(end, (620, 260, 700), (0, 120, 0))
    DIRECTOR_KEYS.sort(key=lambda k: k[0])
    return tracks, workers, end
