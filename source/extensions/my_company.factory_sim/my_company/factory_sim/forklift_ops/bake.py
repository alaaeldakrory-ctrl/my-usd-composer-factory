"""Write a plan into the stage as time samples (and remove them again)."""
import math

from pxr import Gf, Sdf, Usd, UsdGeom

from . import planner as P
from .scene import wrap180

STEER_RATIO = 3.5            # steering-wheel degrees per degree of road-wheel angle
STEER_SMOOTH_S = 0.3
PIVOT_LOCK = 35.0            # equivalent road-wheel angle shown while turning on the spot
KEY_STEP = 2                 # key every 2nd frame (30 keys/s at 60 fps): smooth, and half the data to write
STEER_OP = "xformOp:transform:steer"
ROLL_OP = "xformOp:transform:roll"


def _ops(prim):
    return {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}


def _clear(attr):
    for tc in attr.GetTimeSamples():
        attr.ClearAtTime(tc)


class _Writer:
    """Collects keyframes per attribute and writes them straight into the edit-target layer in one change
    block. Much faster inside Kit than UsdAttribute.Set per frame, which notifies listeners every time."""

    def __init__(self, stage):
        self.stage = stage
        self.layer = stage.GetEditTarget().GetLayer()
        self.data = {}

    def add(self, attr, frame, value):
        self.data.setdefault(attr, ([], []))
        self.data[attr][0].append(frame)
        self.data[attr][1].append(value)

    def _spec(self, attr):
        path = attr.GetPath()
        spec = self.layer.GetAttributeAtPath(path)
        if spec is None:
            prim_spec = Sdf.CreatePrimInLayer(self.layer, path.GetPrimPath())
            spec = Sdf.AttributeSpec(prim_spec, path.name, attr.GetTypeName())
        return spec

    def flush(self):
        """Build every attribute's samples in a private anonymous layer (nothing listens to it, so it is fast
        even inside Kit), then copy each finished attribute spec into the real layer in one call."""
        scratch = Sdf.Layer.CreateAnonymous("factory_ops_keys")
        for attr in self.data:
            self._spec(attr)                                             # make sure the target spec exists
            path = attr.GetPath()
            Sdf.CreatePrimInLayer(scratch, path.GetPrimPath())
            Sdf.CopySpec(self.layer, path, scratch, path)                # keep its default, type, metadata
        with Sdf.ChangeBlock():
            for attr, (frames, values) in self.data.items():
                path = attr.GetPath()
                spec = scratch.GetAttributeAtPath(path)
                if spec.HasInfo("timeSamples"):
                    spec.ClearInfo("timeSamples")
                for f, v in zip(frames, values):
                    scratch.SetTimeSample(path, float(f), v)
        with Sdf.ChangeBlock():
            for attr in self.data:
                Sdf.CopySpec(scratch, attr.GetPath(), self.layer, attr.GetPath())
        self.data = {}


PHYS_MARK = "factory:physicsBefore"      # remembers what we switched off, so Clear can restore it


def physics_off(prim):
    """Keyframed objects must not also be rigid bodies (PhysX converts them on Play and can crash building
    their colliders). SimReady assets carry a PhysicsVariant (None / RigidBody): switch it to None. Other
    rigid bodies get physics:rigidBodyEnabled = false. The previous state is stored on the prim."""
    if prim.HasAttribute(PHYS_MARK):
        return False
    vsets = prim.GetVariantSets()
    if "PhysicsVariant" in vsets.GetNames():
        vs = vsets.GetVariantSet("PhysicsVariant")
        sel = vs.GetVariantSelection()
        if sel and sel != "None" and "None" in vs.GetVariantNames():
            prim.CreateAttribute(PHYS_MARK, Sdf.ValueTypeNames.String, custom=True).Set("variant:" + sel)
            vs.SetVariantSelection("None")
            return True
    bodies = [p for p in Usd.PrimRange(prim) if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas()]
    if bodies:
        changed = []
        for b in bodies:
            a = b.GetAttribute("physics:rigidBodyEnabled")
            if a and a.Get() is not False:
                a.Set(False)
                changed.append(str(b.GetPath()))
        prim.CreateAttribute(PHYS_MARK, Sdf.ValueTypeNames.String, custom=True).Set("bodies:" + ";".join(changed))
        return True
    return False


def physics_restore(prim):
    a = prim.GetAttribute(PHYS_MARK)
    if not a:
        return
    val = a.Get() or ""
    stage = prim.GetStage()
    if val.startswith("variant:"):
        prim.GetVariantSets().GetVariantSet("PhysicsVariant").SetVariantSelection(val[len("variant:"):])
    elif val.startswith("bodies:"):
        for path in filter(None, val[len("bodies:"):].split(";")):
            b = stage.GetPrimAtPath(path)
            if b:
                b.GetAttribute("physics:rigidBodyEnabled").Set(True)
    prim.RemoveProperty(PHYS_MARK)


def steering_op(stage, fl, create=True):
    """The extra transform op that turns the steering wheel mesh (appended last, so it acts in mesh space)."""
    if fl.steering is None:
        return None
    prim = stage.GetPrimAtPath(fl.steering.path)
    op = _ops(prim).get(STEER_OP)
    if op is None and create:
        op = UsdGeom.Xformable(prim).AddTransformOp(opSuffix="steer")
        op.Set(Gf.Matrix4d(1.0))
    return op


def roll_op(stage, wheel, create=True):
    """Extra transform op that rolls (and steers) a wheel mesh that has no rotate op of its own."""
    prim = stage.GetPrimAtPath(wheel.path)
    op = _ops(prim).get(ROLL_OP)
    if op is None and create:
        op = UsdGeom.Xformable(prim).AddTransformOp(opSuffix="roll")
        op.Set(Gf.Matrix4d(1.0))
    return op


def roll_matrix(wheel, spin_deg, steer_deg):
    c = Gf.Vec3d(wheel.center_local)
    rot = Gf.Rotation(wheel.axle_local, spin_deg) * Gf.Rotation(wheel.up_local, steer_deg)
    return Gf.Matrix4d().SetTranslate(-c) * Gf.Matrix4d().SetRotate(rot) * Gf.Matrix4d().SetTranslate(c)


def steer_matrix(sw, wheel_deg):
    c = Gf.Vec3d(sw.local_center)
    return Gf.Matrix4d().SetTranslate(-c) * Gf.Matrix4d().SetRotate(Gf.Rotation(sw.local_axis, -wheel_deg)) * \
        Gf.Matrix4d().SetTranslate(c)


def sample_agent(agent, frames, fps):
    """Per frame: (pos, heading, L, odo, delta) where delta is the front-equivalent steer angle (+ = left)."""
    fl = agent.fl
    raw = []
    odo = 0.0
    prev = None
    for f in frames:
        t = f / fps
        pos, h, L, _, seg = agent.state(t)
        if prev is not None:
            ds = math.dist(pos, prev[0])
            direction = seg.direction if (seg is not None and seg.kind == "path") else 1
            odo += ds * direction
            dh = math.radians(wrap180(h - prev[1]))
            if seg is not None and seg.kind == "pivot":
                kappa = None if abs(dh) < 1e-6 else (1.0 if dh > 0 else -1.0) * 1e9
            elif ds > 0.05:
                kappa = dh / (ds * direction)
            else:
                kappa = 0.0
        else:
            kappa = 0.0
        raw.append([pos, h, L, odo, kappa])
        prev = (pos, h)
    # steering angle from curvature, smoothed so the driver turns a little before each curve
    lock = math.radians(PIVOT_LOCK)
    ang = []
    for r in raw:
        k = r[4]
        if k is None:
            ang.append(0.0)
        elif abs(k) > 1e8:
            ang.append(math.copysign(lock, k))
        else:
            ang.append(max(-lock, min(lock, math.atan(fl.wheelbase * k))))
    step = (frames[1] - frames[0]) if len(frames) > 1 else 1
    sigma = max(1e-3, STEER_SMOOTH_S * fps / step)
    half = int(3 * sigma)
    w = [math.exp(-0.5 * (k / sigma) ** 2) for k in range(-half, half + 1)]
    n = len(ang)
    out = []
    for i in range(n):
        acc = ws = 0.0
        for k, wk in zip(range(-half, half + 1), w):
            j = min(max(i + k, 0), n - 1)
            acc += wk * ang[j]
            ws += wk
        pos, h, L, odo, _ = raw[i]
        out.append((pos, h, L, odo, math.degrees(acc / ws)))
    return out


def bake(stage, scene, plan, progress=None):
    """Author time samples for every forklift and moved pallet.
    Returns (last frame, {forklift name: (frames, samples)})."""
    result = None
    for kind, val in bake_steps(stage, scene, plan):
        if kind == "progress" and progress:
            progress(val)
        elif kind == "done":
            result = val
    return result


def bake_steps(stage, scene, plan):
    """Generator version of bake(): yields ("progress", text) between chunks so a UI can stay responsive,
    then ("done", (last frame, per-forklift samples))."""
    fps = stage.GetTimeCodesPerSecond() or 60.0
    end_f = int(math.ceil(plan.end_t * fps))
    frames = list(range(0, end_f + 1, KEY_STEP))
    if frames[-1] != end_f:
        frames.append(end_f)
    per_agent = {}
    pallet_prims = {p.name: p.prim for p in scene.pallets}
    off = [str(p.GetPath()) for p in [a.fl.prim for a in plan.agents] + [pallet_prims[n] for n in plan.pallets]
           if physics_off(p)]
    if off:
        yield "progress", f"Physics switched off on {len(off)} animated object(s) (restored by Clear)."
    for ai, agent in enumerate(plan.agents):
        fl = agent.fl
        yield "progress", f"Writing {fl.name} ({ai + 1}/{len(plan.agents)})..."
        samples = sample_agent(agent, frames, fps)
        per_agent[fl.name] = (frames, samples)
        place = fl.place
        y0 = place.origin0[1]
        lift_ops = [(frac, _ops(stage.GetPrimAtPath(path))["xformOp:translate"], rest, up)
                    for frac, path, rest, up in fl.lift_parts]
        wheel_ops = [(w, _ops(stage.GetPrimAtPath(w.path))["xformOp:rotateXYZ"] if w.mode == "rotate"
                      else roll_op(stage, w)) for w in fl.wheels]
        sop = steering_op(stage, fl)
        attrs = [place.translate, place.rotate] + [o for _, o, _, _ in lift_ops] + [o for _, o in wheel_ops]
        if sop is not None:
            attrs.append(sop)
        W = _Writer(stage)
        rot_f3 = place.rotate.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Float3
        tr_f3 = place.translate.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Float3
        for f, (pos, h, L, odo, delta) in zip(frames, samples):
            t, r = place.ops_for(pos[0], y0, pos[1], h)
            W.add(place.translate.GetAttr(), f, Gf.Vec3f(t) if tr_f3 else t)
            W.add(place.rotate.GetAttr(), f, Gf.Vec3f(r) if rot_f3 else r)
            for frac, op, rest, up in lift_ops:
                d = L if frac >= 1.0 else frac * max(0.0, L)
                v = Gf.Vec3d(rest) + up * d
                W.add(op.GetAttr(), f, Gf.Vec3f(v) if op.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Float3 else v)
            for w, op in wheel_ops:
                if w.mode == "roll":
                    W.add(op.GetAttr(), f, roll_matrix(w, math.degrees(odo / w.radius), -delta if w.rear else 0.0))
                    continue
                rot = list(w.rest)
                rot[w.spin_idx] += w.spin_sign * math.degrees(odo / w.radius)
                if w.rear and w.steer_idx > w.spin_idx:
                    rot[w.steer_idx] += w.steer_sign * -delta          # rear wheels steer the other way
                W.add(op.GetAttr(), f, Gf.Vec3d(*rot) if op.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Double3
                      else Gf.Vec3f(*rot))
            if sop is not None:
                W.add(sop.GetAttr(), f, steer_matrix(fl.steering, -STEER_RATIO * delta))
        W.flush()

    # pallets
    yield "progress", "Writing pallets..."
    pallets = {p.name: p for p in scene.pallets}
    for name, track in plan.pallets.items():
        pal = pallets[name]
        place = pal.place
        c0 = pal.box.c
        o0 = (place.origin0[0], place.origin0[2])
        off = (c0[0] - o0[0], c0[1] - o0[1])          # origin -> footprint center at rest
        dy = place.origin0[1] - pal.base_y
        W = _Writer(stage)
        rot_f3 = place.rotate.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Float3
        tr_f3 = place.translate.GetAttr().GetTypeName() == Sdf.ValueTypeNames.Float3
        for f in frames:
            st = P.pallet_state(track, f / fps)
            if st is None:
                c, y, dh = c0, pal.base_y, 0.0
            else:
                c, y, dh = st
            a = math.radians(dh)
            ox = off[0] * math.cos(a) + off[1] * math.sin(a)
            oz = off[1] * math.cos(a) - off[0] * math.sin(a)
            t, r = place.ops_for(c[0] - ox, y + dy, c[1] - oz, place.r0[1] + dh)
            W.add(place.translate.GetAttr(), f, Gf.Vec3f(t) if tr_f3 else t)
            W.add(place.rotate.GetAttr(), f, Gf.Vec3f(r) if rot_f3 else r)
        W.flush()
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(end_f)
    yield "done", (end_f, per_agent)


def clear(stage, scene):
    """Remove every time sample this tool writes (forklifts, their parts, pallets, drivers)."""
    n = 0
    for fl in scene.forklifts:
        attrs = [fl.place.translate.GetAttr(), fl.place.rotate.GetAttr()]
        for _, path, _, _ in fl.lift_parts:
            attrs.append(_ops(stage.GetPrimAtPath(path))["xformOp:translate"].GetAttr())
        for w in fl.wheels:
            op = _ops(stage.GetPrimAtPath(w.path))["xformOp:rotateXYZ"] if w.mode == "rotate" else roll_op(stage, w, False)
            if op is not None:
                attrs.append(op.GetAttr())
        sop = steering_op(stage, fl, create=False)
        if sop is not None:
            attrs.append(sop.GetAttr())
        for a in attrs:
            n += a.GetNumTimeSamples()
            _clear(a)
        drv = stage.GetPrimAtPath(fl.prim.GetPath().AppendChild("Driver"))
        if drv:
            stage.RemovePrim(drv.GetPath())
        physics_restore(fl.prim)
    for pal in scene.pallets:
        for a in (pal.place.translate.GetAttr(), pal.place.rotate.GetAttr()):
            n += a.GetNumTimeSamples()
            _clear(a)
        physics_restore(pal.prim)
    return n
