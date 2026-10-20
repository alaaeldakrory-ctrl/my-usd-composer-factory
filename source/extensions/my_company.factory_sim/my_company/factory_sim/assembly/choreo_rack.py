"""Choreography for the two-level shelving rack (NVIDIA RackSmallEmpty_A2): the way a bay really goes up.

    1-2  end frames L / R walked up from the floor (both workers, one per upright, hand over hand up the uprights)
    3    lower beams hooked onto the uprights' front faces, dropped onto their hooks and bolted (A front, B rear)
    4    base plates anchored to the floor with a hammer drill (drill through the plate, drive the anchor), the bay
         square on its lower beams
    5    lower wire deck: both carry it on edge by its front edge, tip it up, rest its back edge on the front beam and
         slide it across onto the rear beam (the beams carry the weight), then drop the front edge into the step
    6    upper beams (as 3), upper deck (as 5), then both step back and look the bay over
Built from the ES2632 pieces (choreo.Worker / PartTrack / grips / ratchet rule / bolts).
"""
import math

from pxr import Gf, UsdGeom

from . import choreo as C
from . import scene_rack as SR
from .rig import fwd_of, left_of

UP = Gf.Vec3d(0, 1, 0)
ease = C.ease
DIRECTOR_KEYS = []          # filled by script(): (t, eye, look-at)
DRILL_EVENTS = []           # (worker, anchor point, heading, t0, t1)
# for the ergonomics (ergo.py): what each worker handles and when, and the assembly steps
LOADS = []                  # (worker, t0, t1, kg carried by this worker, label)
STEPS = []                  # (t0, t1, label)
MASS = {"frame": 25.0, "beam": 9.0, "deck": 12.0, "drill": 5.5}     # kg (pack-size parts, steel)
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


def worker_targets(workers):
    """What the camera must keep in view at time t: both workers (head and hips)."""
    def f(t):
        return [tuple(workers[w].root_at(t)[0] + Gf.Vec3d(0, y, 0)) for w in "AB" for y in (150.0, 70.0)]
    return f


def wide_keys(stage, workers, end, dist=-980.0, height=340.0, step=8.0):
    """The wide director: a high camera ~10 m out in the rear aisle (the front one has the deck stand in the way),
    looking down the aisle between the central columns (so no column ever stands between it and the work) through
    the open bay, drifting slowly sideways after the two workers so the warehouse around them stays in the frame."""
    import numpy as np
    from . import director as D
    from . import sightline as SL
    occ = SL.occluders(stage)
    times = list(np.arange(0.0, end, step)) + [end]
    cx = []
    for t in times:
        xs = [workers[w].root_at(tt)[0][0] for w in "AB" for tt in np.linspace(max(0, t - step / 2), min(end, t + step / 2), 5)]
        cx.append(float(np.mean(xs)))
    cx = list(np.convolve(np.pad(cx, 2, mode="edge"), np.ones(5) / 5, mode="valid"))
    keys = []
    for t, x in zip(times, cx):
        ex = max(-330.0, min(330.0, 130.0 + 0.5 * x))         # a little off-centre: the deck stand stays aside
        keys.append((float(t), (ex, height, dist), (0.75 * x, 85.0, 0.0)))
    bad = SL.report(keys, occ, D.at, end, targets_fn=worker_targets(workers))
    return keys, bad


def drill_m(k, t):
    """World matrix of the hammer drill for DRILL_EVENTS[k] at time t: set down on the anchor point, drilling
    through the plate (bit sinking 5 cm, hammering), lifted, then driving the anchor home."""
    who, ap, face, t0, t1 = DRILL_EVENTS[k]
    t = min(max(t, t0), t1)
    r = -left_of(face)
    f = fwd_of(face)
    d1, d2 = t1 - 1.2, t1 - 0.9
    if t < t0 + 0.4:
        y = 4.0 * (1 - ease((t - t0) / 0.4))
    elif t < d1:
        y = -5.0 * ease((t - t0 - 0.4) / (d1 - t0 - 0.4))
    elif t < d2:
        y = -5.0 + 8.5 * ease((t - d1) / (d2 - d1))
    else:
        y = 3.5 - 2.0 * ease((t - d2) / (t1 - d2))
    buzz = 0.18 * math.sin(2 * math.pi * 23.0 * t) if t0 + 0.4 < t < t1 - 0.05 and not (d1 < t < d2) else 0.0
    o = ap + UP * y + r * buzz + f * (0.5 * buzz)
    return Gf.Matrix4d(r[0], r[1], r[2], 0, 0, 1, 0, 0, f[0], f[1], f[2], 0, o[0], o[1], o[2], 1)


def drill_grip(k, local):
    def f(t, ctx):
        m = drill_m(k, t)
        return {"kind": "tube", "c": m.Transform(local), "axis": m.TransformDir(Gf.Vec3d(1, 0, 0)),
                "thumb": m.TransformDir(Gf.Vec3d(1, 0, 0)), "r": 1.6, "curl": 1.0, "load": False}   # it stands on the floor
    return f


def write_tools(stage, n_frames):
    """The hammer drills: shown only while in use."""
    for who in "AB":
        prim = stage.GetPrimAtPath(f"/World/Tools/Drill{who}")
        op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
        vis = UsdGeom.Imageable(prim).GetVisibilityAttr()
        ev = [k for k, e in enumerate(DRILL_EVENTS) if e[0] == who]
        for f in range(n_frames):
            t = f / C.FPS
            k = next((k for k in ev if DRILL_EVENTS[k][3] - 0.2 <= t <= DRILL_EVENTS[k][4] + 0.2), None)
            if k is None:
                vis.Set("invisible", f)
                continue
            op.Set(drill_m(k, t), f)
            vis.Set("inherited", f)


def script(parts_def, rigs):
    C.BOLT_EVENTS.clear()
    DIRECTOR_KEYS.clear()
    DRILL_EVENTS.clear()
    LOADS.clear()
    STEPS.clear()
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
        LOADS.append((W.name, t0, t0 + dur, 0.6, "ratchet"))
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
            off = SR.FRAME_OUT - SR.UPRIGHT_X
            return min(d_max[who], (HAND_H[who] - off * math.cos(r)) / max(math.sin(r), 1e-3))

        def root(who, th):
            c = SR.frame_m(side, th).Transform(SR.upright_point(side, d_of(who, th), 0.0))
            return (c[0] + s * REACH[who], LIFT_Z[who])

        # each lifter turns ~30 deg toward his / her upright so both hands work in front of the chest, the hand on
        # the upright's side the higher one (no arms crossing)
        turn, upper_hand = {}, {}
        for W in (A, B):
            zc = math.copysign(SR.UPRIGHT_Z[1] - 5.0, LIFT_Z[W.name])
            on_left = Gf.Dot(Gf.Vec3d(0, 0, zc - LIFT_Z[W.name]), left_of(face)) > 0
            turn[W.name] = face + (30.0 if on_left else -30.0)
            upper_hand[W.name] = "L" if on_left else "R"
        for W in (A, B):
            W.walk(routes[W.name] + [root(W.name, SR.THETA0)], end_h=turn[W.name], speed=85.0)
        t = max(A.t, B.t) + 0.4
        for W in (A, B):
            W.until(t)
        th, t_up = theta_profile(t + 0.9)
        tracks[name].segs += [("hold", 0.0, t + 0.9, SR.frame_m(side, SR.THETA0)),
                              ("path", t + 0.9, t_up, lambda tt: SR.frame_m(side, th(tt))),
                              ("hold", t_up, 1e9, SR.frame_m(side, 90.0))]
        for W in (A, B):
            W.follow(t + 0.9, t_up, lambda tt, w=W.name: (root(w, th(tt)), turn[w]))
        for W in (A, B):
            zc = math.copysign(SR.UPRIGHT_Z[1] - 5.0, LIFT_Z[W.name])
            upper = upper_hand[W.name]
            lower = "R" if upper == "L" else "L"
            for hand, back in ((upper, 0.0), (lower, 24.0)):
                def spec(tt, ctx, who=W.name, back=back, zc=zc):
                    m = ctx.part_m(name, tt)
                    d = d_of(who, th(tt)) - back
                    return {"kind": "tube", "c": m.Transform(SR.upright_point(side, d, zc)), "axis": m.TransformDir(UP),
                            "thumb": -m.TransformDir(UP), "r": 6.0, "curl": 1.0, "load": True}
                W.hold(hand, t, t_up + 0.5, spec)
            LOADS.append((W.name, t + 0.9, t_up, MASS["frame"] / 2, "walk frame up"))
            W.look_at(t, t_up + 0.5, lambda tt, ctx: ctx.part_m(name, tt).Transform(
                SR.upright_point(side, 200.0, 0.0)))
        for W in (A, B):
            W.until(t_up + 0.8)
            W.strafe((W.pos[0] + s * 18.0, W.pos[2] + math.copysign(12.0, W.pos[2])))
        return t, t_up

    t1a, t1b = frame_up("L", {"A": [], "B": []})
    STEPS.append((0.0, t1b + 0.8, "1 End frame L walked up"))
    shot(0.0, (-560, 260, 520), (-260, 60, 0))
    shot(t1a, (-640, 210, 360), (-250, 80, 0))
    shot(t1b, (-520, 240, 470), (-120, 150, 0))
    t2a, t2b = frame_up("R", {"A": [(-60, 88), (330, 88)], "B": [(-60, -88), (330, -88)]})
    STEPS.append((t1b + 0.8, t2b + 0.8, "2 End frame R walked up"))
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
        lifted = P.at(P.start[0] + UP * (12.0 if SR.TRESTLE_TOP < 60 else 3.0))     # off waist-high trestles: barely lifted
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
        LOADS.append((W.name, t0 + 0.9, s1, MASS["beam"], "carry / hook beam"))
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

    # ---------------------------------------------------------------- 4: anchors (A the front plates, B the rear ones)
    def anchor_job(W, plan):
        """plan: [(route, frame name, anchor point)]; the drill goes in vertically through the plate's outer tab."""
        for route, frame, ap in plan:
            s = 1.0 if ap[0] > 0 else -1.0
            face = -90.0 * s                                # facing the plate, from outside the bay
            W.walk(route + [(ap[0] + s * SR.DRILL_STANCE, ap[2])], end_h=face, speed=80.0)
            t0 = W.t + 0.5
            t1 = t0 + 4.2
            DRILL_EVENTS.append((W.name, ap, face, t0, t1))
            k = len(DRILL_EVENTS) - 1
            C.BOLT_EVENTS.append((frame, ap, UP, t1 - 0.9, t1))
            W.hold("R", t0 - 0.2, t1 + 0.2, drill_grip(k, SR.DRILL_TOP))
            LOADS.append((W.name, t0, t1, MASS["drill"], "hammer drill"))
            W.hold("L", t0 - 0.2, t1 + 0.2, drill_grip(k, SR.DRILL_SIDE))
            W.look_at(t0 - 0.5, t1 + 0.2, ap)
            W.until(t1 + 0.6)

    # ---------------------------------------------------------------- 5 / 6: decks, both workers from the front
    def deck_job(level, a_route, b_route):
        name = "Deck" + level
        P = parts_def[name]
        D = SR.deck_pivot(level)
        hx, hz = SR.DECK_STORE[name]
        top = SR.BEAM_TOP[level]
        rear_local = D + Gf.Vec3d(0, -SR.DECK_THICK, -SR.DECK_DEPTH)
        rear_off = lambda al: SR.deck_m(level, Gf.Vec3d(0, 0, 0), al).Transform(rear_local)
        GAP = {"A": -43.0, "B": 43.0}                      # where each stands along the deck (x from its middle)
        BACK = 30.0                                         # in front of the edge they hold
        for W, route in ((A, a_route), (B, b_route)):
            W.walk(route + [(hx + GAP[W.name], hz + SR.DECK_THICK + BACK)], end_h=180.0, speed=85.0)
        t0 = max(A.t, B.t) + 0.3
        for W in (A, B):
            W.until(t0)
        m0 = SR.start_matrix(name, P)
        carry_al = 18.0                                     # carried on edge, the bottom swung out clear of the knees
        h_carry = Gf.Vec3d(hx, 100.0, hz)
        # at the bay: tip it up (back edge rising out in front of the beams), rest the back edge on the front beam,
        # slide it across onto the rear beam, drop the front edge into the step
        land_al = 82.0 if level == "Low" else 98.0          # front edge a bit above / below the back edge
        land = Gf.Vec3d(0, top + 0.6, SR.BEAM_Z + 1.0) - rear_off(land_al)
        clear = land + Gf.Vec3d(0, 7.0, 13.0)
        h_bay = Gf.Vec3d(0.0, 100.0, clear[2] + 8.0)
        t_l = t0 + 1.4
        t_c0 = t_l + 0.2
        t_c1 = t_c0 + (h_carry - h_bay).GetLength() / 42.0
        t_up = t_c1 + 2.0
        t_land = t_up + 0.9
        t_sl = t_land + 3.4
        seat = t_sl + 1.1

        def slide(tt):
            u = ease((tt - t_land) / (t_sl - t_land))
            al = land_al + (90.0 - land_al) * u
            z = land[2] + (SR.DECK_EDGE - land[2]) * u
            return SR.deck_m(level, Gf.Vec3d(0, top + 0.6 - rear_off(al)[1], z), al)
        tracks[name].segs += [("hold", 0.0, t0 + 0.6, m0),
                              ("move", t0 + 0.6, t_l, m0, SR.deck_m(level, h_carry, carry_al)),
                              ("move", t_c0, t_c1, SR.deck_m(level, h_carry, carry_al), SR.deck_m(level, h_bay, carry_al)),
                              ("move", t_c1, t_up, SR.deck_m(level, h_bay, carry_al), SR.deck_m(level, clear, land_al)),
                              ("move", t_up, t_land, SR.deck_m(level, clear, land_al), slide(t_land)),
                              ("path", t_land, t_sl, slide),
                              ("move", t_sl, seat, slide(t_sl), P.installed()), ("hold", seat, 1e9, P.installed())]
        hinge = lambda tt: ctx0.part_m(name, tt).Transform(D)
        for W in (A, B):
            W.follow(t0 + 0.6, seat, lambda tt, g=GAP[W.name]: ((hinge(tt)[0] + g, max(hinge(tt)[2], 50.0) + BACK), 180.0))
            k1, k2 = ("front_A1", "front_A2") if W.name == "A" else ("front_B1", "front_B2")
            hands_on(W, name, P.holds[k1], P.holds[k2], t0, seat + 0.3, 180.0)
            LOADS.append((W.name, t0 + 0.6, seat, MASS["deck"] / 2, "carry / slide deck"))
            W.look_at(t0, seat, lambda tt, ctx: ctx.part_m(name, tt).Transform(D + Gf.Vec3d(0, 0, -45.0)))
            W.until(seat + 0.6)
        for W in (A, B):
            W.strafe((W.pos[0], W.pos[2] + 30.0))
        return t_c0, t_land, seat

    t3 = max(A.t, B.t)
    sA = beam_job(A, "BeamLowF", [(240, 160), (250, 300)])
    sB = beam_job(B, "BeamLowR", [(240, -160), (250, -300)])
    shot(t3 + 3.0, (520, 300, 680), (0, 60, 100))
    shot(sA - 4.0, (-330, 170, 380), (0, 50, 30))
    shot(sA + 4.0, (440, 250, 280), (0, 60, -10))
    shot(max(A.t, B.t) - 6.0, (400, 180, -390), (30, 50, -40))
    t4 = max(A.t, B.t)
    STEPS.append((t2b + 0.8, t4, "3 Lower beams hooked + bolted"))
    an = {f + ("F" if p[2] > 0 else "R"): (f, p) for f, p in SR.anchors()}
    anchor_job(A, [([(170, 110)],) + an["FrameRF"], ([(170, 120), (-170, 120)],) + an["FrameLF"]])
    anchor_job(B, [([(-170, -110)],) + an["FrameLR"], ([(-170, -120), (170, -120)],) + an["FrameRR"]])
    shot(t4 + 1.0, (470, 200, 400), (120, 20, 40))
    shot(t4 + 8.0, (60, 280, 560), (0, 30, 0))
    shot(max(A.t, B.t) - 2.0, (-470, 200, 400), (-120, 20, 40))
    t4b = max(A.t, B.t)
    STEPS.append((t4, t4b, "4 Base plates anchored (hammer drill)"))
    c4, land4, seat4 = deck_job("Low", [(-200, 160), (-60, 300)], [(170, -115), (-200, -115), (-200, 160), (60, 300)])
    shot(t4b + 1.0, (-420, 230, 600), (0, 80, 330))
    shot(c4 + 1.0, (-380, 190, 420), (0, 80, 200))
    shot(land4, (-330, 170, 330), (0, 60, 40))
    shot(seat4 - 1.0, (330, 180, 330), (0, 55, 0))
    t5 = max(A.t, B.t)
    STEPS.append((t4b, t5, "5 Lower wire deck"))
    beam_job(A, "BeamUpF", [(-200, 160), (-150, 300)])
    beam_job(B, "BeamUpR", [(200, 120), (230, -140), (250, -300)])
    shot(t5 + 6.0, (560, 300, 160), (0, 110, 40))
    shot(max(A.t, B.t) - 12.0, (-330, 220, 380), (-40, 130, 40))
    shot(max(A.t, B.t) - 8.0, (-460, 240, 0), (0, 130, 0))
    shot(max(A.t, B.t) - 4.0, (-330, 220, -390), (40, 130, -40))
    t6 = max(A.t, B.t)
    STEPS.append((t5, t6, "6 Upper beams hooked + bolted"))
    c6, land6, seat6 = deck_job("Up", [(-160, 150), (-60, 300)], [(220, -150), (220, 200), (60, 300)])
    shot(t6 + 1.0, (-420, 260, 620), (0, 100, 330))
    shot(c6 + 1.0, (-400, 220, 440), (0, 110, 200))
    shot(land6, (-340, 230, 340), (0, 140, 40))
    shot(seat6 - 1.0, (340, 250, 380), (0, 140, 0))

    # ---------------------------------------------------------------- wrap up: both step out front and look the bay over
    STEPS.append((t6, seat6 + 0.6, "7 Upper wire deck"))
    A.walk([(-120, 200)], end_h=160.0, speed=70.0)
    B.walk([(130, 210)], end_h=-160.0, speed=75.0)
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
