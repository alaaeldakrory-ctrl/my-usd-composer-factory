"""Seated drivers: a Mixamo character in each forklift that has a seat layout, hands on the knob and rim,
looking up at raised forks and back over the shoulder when reversing. Uses the ergo_sim reach solver."""
import math
import os
import sys

from pxr import Gf, Usd, UsdGeom

from . import bake as B

def _find_ergo_core():
    """ergo_sim's core folder, next to this extension in the source tree (Kit runs a linked copy under _build)."""
    here = os.path.realpath(os.path.dirname(__file__))
    rel = os.path.join("my_company.ergo_sim", "my_company", "ergo_sim", "core")
    cands = [os.path.normpath(os.path.join(here, "..", "..", "..", "..", rel))]
    marker = os.sep + "_build" + os.sep
    if marker in here:                                        # .../kit-app-template/_build/... -> .../source/extensions
        cands.append(os.path.join(here.split(marker)[0], "source", "extensions", rel))
    for c in cands:
        if os.path.isfile(os.path.join(c, "reach.py")):
            return c
    return cands[-1]


_ERGO_CORE = _find_ergo_core()
DEFAULT_CHARACTER = r"C:\Users\alaae\Dev\omniverse-learning\exercises\assets\maximo\Snatch.usd"
DRIVER = "Driver"
STEP = 4                          # pose keys every 4 frames
LEFT_GRIP, LEFT_FOLLOW = -60.0, 35.0
PALM_UP, PALM_BACK = 4.0, 6.0
LOOK_BACK_DEG = 75.0
HELMET = 6.4                      # helmet height above the skull joint (Mixamo Ch17)


def _reach():
    if _ERGO_CORE not in sys.path:
        sys.path.insert(0, _ERGO_CORE)
    import reach                   # noqa: E402  (ergo_sim/core/reach.py)
    return reach


def _ease(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def rim_point(sw, clock_deg, wheel_deg=0.0, lift=0.0, radius=None):
    """Forklift-frame point on the rim (clockwise degrees from 12 as the driver sees it), turned with the wheel."""
    a = math.radians(clock_deg)
    right = Gf.Vec3d(-1, 0, 0)
    p = sw.center + (sw.top * math.cos(a) + right * math.sin(a)) * (radius or sw.radius) + sw.axis * lift
    rot = Gf.Rotation(sw.axis, -wheel_deg)
    return sw.center + rot.TransformDir(p - sw.center)


class Seating:
    def __init__(self, skeleton_prim, fl):
        R = _reach()
        self.R, self.fl, self.seat = R, fl, fl.seat
        self.rig = R.Rig(skeleton_prim)
        self.solver = R.ReachSolver(self.rig, max_reach_fraction=0.92, lean_max_deg=30.0, head_extension_limit_deg=6.0)
        names = [j.split("/")[-1] for j in self.rig.joints]
        self.j = {n.split("_", 1)[-1] if n.startswith("mixamorig") else n: i for i, n in enumerate(names)}
        w = self.rig.rest_world
        up_leg = Gf.Vec3d(w[self.j["LeftUpLeg"]].ExtractTranslation())
        knee = Gf.Vec3d(w[self.j["LeftLeg"]].ExtractTranslation())
        ankle = Gf.Vec3d(w[self.j["LeftFoot"]].ExtractTranslation())
        self.thigh, self.shin = (knee - up_leg).GetLength(), (ankle - knee).GetLength()
        self.root = Gf.Vec3d(self.seat["x"], 0.0, self.seat["hip_z"] - up_leg[2])
        self.raise_y = self.seat["hip_y"] - up_leg[1]

    flip = False                      # True: the character faces the back of the forklift

    def to_skel(self, p):
        d = Gf.Vec3d(p) - self.root - Gf.Vec3d(0.0, self.raise_y, 0.0)
        return Gf.Vec3d(-d[0], d[1], -d[2]) if self.flip else d

    def has_controls(self):
        """Something to hold: the steering wheel (seated) or the console (standing)."""
        return self.fl.steering is not None

    def grips(self, wheel_deg):
        sw = self.fl.steering
        palm = sw.axis * PALM_UP + Gf.Vec3d(0.0, 0.0, -PALM_BACK)
        left = rim_point(sw, LEFT_GRIP + max(-LEFT_FOLLOW, min(LEFT_FOLLOW, wheel_deg)))
        if sw.knob:
            right = rim_point(sw, sw.knob[0], wheel_deg, sw.knob[2], sw.knob[1])
        else:
            right = rim_point(sw, 60.0 + max(-LEFT_FOLLOW, min(LEFT_FOLLOW, wheel_deg)))
        return {"l": self.to_skel(left + palm), "r": self.to_skel(right + palm)}

    def _legs(self, ov):
        R, r, s = self.R, self.rig, self.seat
        for side, out in (("Left", 1.0), ("Right", -1.0)):
            iU, iK, iF = self.j[side + "UpLeg"], self.j[side + "Leg"], self.j[side + "Foot"]
            world = r.fk(ov)
            hip = Gf.Vec3d(world[iU].ExtractTranslation())
            target = self.to_skel(Gf.Vec3d(s["x"] + out * 13.0, s["ankle_y"], s["ankle_z"]))
            pole = Gf.Vec3d(out * 0.15, 0.5, 1.0).GetNormalized()
            knee, ankle, _ = R.two_bone_ik(hip, target, self.thigh, self.shin, pole)
            d_cur = Gf.Vec3d(world[iK].ExtractTranslation()) - hip
            new_u = world[iU].ExtractRotation() * Gf.Rotation(d_cur.GetNormalized(), (knee - hip).GetNormalized())
            ov[iU] = new_u * world[r.parent[iU]].ExtractRotation().GetInverse()
            world = r.fk(ov)
            k = Gf.Vec3d(world[iK].ExtractTranslation())
            d_cur = Gf.Vec3d(world[iF].ExtractTranslation()) - k
            new_k = world[iK].ExtractRotation() * Gf.Rotation(d_cur.GetNormalized(), (ankle - k).GetNormalized())
            ov[iK] = new_k * world[iU].ExtractRotation().GetInverse()
            world = r.fk(ov)
            ov[iF] = r.rest_world[iF].ExtractRotation() * world[iK].ExtractRotation().GetInverse()

    def pose(self, wheel_deg=0.0, look=None, look_back=0.0):
        targets = self.grips(wheel_deg) if self.has_controls() else {"l": None, "r": None}
        ov, info = self.solver.solve(targets, look=None if look is None else self.to_skel(look))
        self._legs(ov)
        if look_back:
            r = self.rig
            world = r.fk(ov)
            iN, iH = self.j["Neck"], self.j["Head"]
            new_n = world[iN].ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 1, 0), -look_back * 0.6)
            ov[iN] = new_n * world[r.parent[iN]].ExtractRotation().GetInverse()
            world = r.fk(ov)
            new_h = world[iH].ExtractRotation() * Gf.Rotation(Gf.Vec3d(0, 1, 0), -look_back * 0.4)
            ov[iH] = new_h * world[iN].ExtractRotation().GetInverse()
        hips = self.j["Hips"]
        ov.trans[hips] = Gf.Vec3d(ov.trans.get(hips, self.rig.rest_t[hips])) + Gf.Vec3d(0.0, self.raise_y, 0.0)
        return ov, info


class Standing(Seating):
    """Operator standing on an order picker's platform, hands on the front console. The platform (and so the
    operator) rides up with the forks; that is done by moving the driver prim, see add_drivers_steps."""

    def __init__(self, skeleton_prim, fl):
        R = _reach()
        self.R, self.fl, self.seat, self.st = R, fl, None, fl.stand
        self.rig = R.Rig(skeleton_prim)
        self.solver = R.ReachSolver(self.rig, max_reach_fraction=0.92, lean_max_deg=25.0, head_extension_limit_deg=20.0)
        names = [j.split("/")[-1] for j in self.rig.joints]
        self.j = {n.split("_", 1)[-1] if n.startswith("mixamorig") else n: i for i, n in enumerate(names)}
        self.root = Gf.Vec3d(self.st["x"], 0.0, self.st["z"])
        self.raise_y = self.st["floor_y"]                      # feet on the platform deck
        self.flip = self.st.get("face") == "back"

    def grips(self, wheel_deg=0.0):
        st, out = self.st, {}
        toward = 1.0 if self.flip else -1.0                     # the panel's operator side
        for side, sgn in (("l", 1.0), ("r", -1.0)):            # his left is +x, or -x when facing back
            x = st["x"] + sgn * st["hand_spread"] * (-1.0 if self.flip else 1.0)
            p = Gf.Vec3d(x, st["console_y"] + 4.0, st["console_z"] + toward * 4.0)
            out[side] = self.to_skel(p)
        return out

    def has_controls(self):
        return True

    def _legs(self, ov):
        pass                                                    # standing: the rest pose legs are already straight


def _place_driver(stage, fl, character):
    """Create (or reuse) the driver prim under the forklift, sitting on the seat facing forward."""
    path = fl.prim.GetPath().AppendChild(DRIVER)
    prim = stage.DefinePrim(path, "Xform")
    prim.GetPayloads().ClearPayloads()
    prim.GetPayloads().AddPayload(character)
    stage.Load(path)
    old = stage.GetPrimAtPath(fl.prim.GetPath().AppendChild("Operator"))      # from the first forklift scripts
    if old and old.IsActive():
        old.SetActive(False)
    return prim


def _orient(stage, prim, fl, root_F, flip=False):
    """Put the character root at forklift-frame point `root_F`, facing the forklift's forward direction
    (or its back, with `flip`)."""
    d = Usd.TimeCode.Default()
    l = (math.cos(math.radians(fl.heading)), -math.sin(math.radians(fl.heading)))
    f = (math.sin(math.radians(fl.heading)), math.cos(math.radians(fl.heading)))
    world_pos = Gf.Vec3d(fl.origin[0] + l[0] * root_F[0] + f[0] * root_F[2], fl.place.origin0[1] + root_F[1],
                         fl.origin[1] + l[1] * root_F[0] + f[1] * root_F[2])
    s = -1.0 if flip else 1.0
    want = Gf.Matrix4d(s * l[0], 0, s * l[1], 0,
                       0, 1, 0, 0,
                       s * f[0], 0, s * f[1], 0,
                       world_pos[0], world_pos[1], world_pos[2], 1)
    parent = UsdGeom.Xformable(fl.prim).ComputeLocalToWorldTransform(d)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(want * parent.GetInverse())


def add_drivers(stage, scene, plan, per_agent, character=DEFAULT_CHARACTER, progress=None):
    """Seat and animate a driver in every forklift with a known seat. Returns notes for the log."""
    notes = []
    for kind, val in add_drivers_steps(stage, scene, plan, per_agent, character):
        if kind == "progress" and progress:
            progress(val)
        elif kind == "done":
            notes = val
    return notes


def _write_anim(stage, anim_path, rig, keys):
    """SkelAnimation with one sample per key, written straight into the edit-target layer in one change block."""
    from pxr import Sdf, UsdSkel, Vt
    # build it in a private layer (fast even inside Kit), then copy the finished prim into the real layer
    scratch = Sdf.Layer.CreateAnonymous("driver_anim")
    tmp = Usd.Stage.Open(scratch)
    anim = UsdSkel.Animation.Define(tmp, anim_path)
    anim.CreateJointsAttr(Vt.TokenArray(rig.joints))
    attrs = (anim.CreateTranslationsAttr(), anim.CreateRotationsAttr(), anim.CreateScalesAttr())
    paths = [a.GetPath() for a in attrs]
    with Sdf.ChangeBlock():
        for f, arrays in keys:
            for path, value in zip(paths, arrays):
                scratch.SetTimeSample(path, float(f), value)
    layer = stage.GetEditTarget().GetLayer()
    Sdf.CreatePrimInLayer(layer, anim_path.GetParentPath())
    Sdf.CopySpec(scratch, anim_path, layer, anim_path)
    return stage.GetPrimAtPath(anim_path)


def add_drivers_steps(stage, scene, plan, per_agent, character=DEFAULT_CHARACTER):
    """Generator: yields ("progress", text) while posing, then ("done", notes). Poses repeat a lot (wheel
    centered, looking ahead), so each distinct pose is solved once and reused."""
    notes = []
    try:
        R = _reach()
    except ImportError:
        yield "done", ["drivers skipped: ergo_sim reach solver not found at " + _ERGO_CORE]
        return
    fps = stage.GetTimeCodesPerSecond() or 60.0
    for agent in plan.agents:
        fl = agent.fl
        if not fl.seat and not fl.stand:
            continue
        yield "progress", f"Posing the driver of {fl.name}..."
        prim = _place_driver(stage, fl, character)
        skel = next((p for p in Usd.PrimRange(prim) if p.GetTypeName() == "Skeleton"), None)
        if skel is None:
            notes.append(f"{fl.name}: character has no skeleton")
            continue
        seat = Seating(skel, fl) if fl.seat else Standing(skel, fl)
        _orient(stage, prim, fl, seat.root, seat.flip)
        frames, samples = per_agent[fl.name]
        step = max(1, STEP // max(1, (frames[1] - frames[0]) if len(frames) > 1 else 1))
        reverse = [(s.t0, s.t1) for s in agent.segs if s.kind == "path" and s.direction < 0 and s.dur > 1.0]
        cache, keys, worst, solved = {}, [], 0.0, 0
        for i in range(0, len(samples), step):
            f = frames[i]
            pos, h, L, odo, delta = samples[i]
            t = f / fps
            wheel = 2.0 * round(-B.STEER_RATIO * delta / 2.0) if fl.seat else 0.0
            back = round(20 * max([_ease((t - t0 + 0.4) / 0.6) * _ease((t1 + 0.2 - t) / 0.6)
                                   for t0, t1 in reverse] or [0.0])) / 20.0 if fl.seat else 0.0
            high = _ease((L - 40.0) / 60.0)
            Lq = 20.0 * round(L / 20.0) if high > 0.01 and fl.seat else None   # standing: forks rise with him
            key = (wheel, back, Lq)
            if key not in cache:
                look = Gf.Vec3d(0.0, fl.blade_top + Lq + 20.0, fl.fork_tip) if Lq is not None else None
                ov, info = seat.pose(wheel, look, LOOK_BACK_DEG * back)
                worst = max([worst] + list(info["hand_error_cm"].values()))
                cache[key] = R._arrays(seat.rig, ov)
                solved += 1
                if solved % 40 == 0:
                    yield "progress", f"Posing the driver of {fl.name}... {100 * i // len(samples)}%"
            keys.append((f, cache[key]))
        anim_path = skel.GetParent().GetPath().AppendChild("DriverAnim")
        if stage.GetPrimAtPath(anim_path):
            stage.RemovePrim(anim_path)
        _write_anim(stage, anim_path, seat.rig, keys)
        R.bind_animation(skel, anim_path)
        if fl.stand:                                            # ride up and down with the platform
            op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
            local0 = Gf.Matrix4d(op.Get(Usd.TimeCode.Default()))
            parent = UsdGeom.Xformable(fl.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            up_local = parent.GetInverse().TransformDir(Gf.Vec3d(0, 1, 0))
            W = B._Writer(stage)
            for i in range(0, len(samples)):
                m = Gf.Matrix4d(local0)
                m.SetTranslateOnly(local0.ExtractTranslation() + up_local * samples[i][2])
                W.add(op.GetAttr(), frames[i], m)
            W.flush()
        notes.append(f"{fl.name}: driver {'seated' if fl.seat else 'standing'} ({len(keys)} keys, "
                     f"{solved} distinct poses, worst hand error {worst:.1f} cm)")
    yield "done", notes
