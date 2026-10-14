"""Find and measure forklifts, racks and pallets in a stage.

World frame: Y up, cm. Plan positions are (x, z) tuples. A heading h (degrees) means forward = (sin h, cos h),
so heading 0 faces +Z. "Left" of a heading is (cos h, -sin h)... see `left_of`.

Forklift frame F = (left, up, forward) measured from the forklift prim's origin; this is also the frame a
Y-up, +Z-facing character uses (its +X is its left), which keeps the driver code simple.
"""
import math
import re
from dataclasses import dataclass, field

from pxr import Gf, Usd, UsdGeom

ROLE_ATTR = "factory:role"
ROLES = ("forklift", "rack", "pallet", "dock", "obstacle")


# ----------------------------------------------------------------------------------------------- math helpers
def fwd_of(h):
    a = math.radians(h)
    return (math.sin(a), math.cos(a))


def left_of(h):
    a = math.radians(h)
    return (math.cos(a), -math.sin(a))


def heading_of(dx, dz):
    return math.degrees(math.atan2(dx, dz))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def corners(rng):
    lo, hi = rng.GetMin(), rng.GetMax()
    return [Gf.Vec3d(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]


# ----------------------------------------------------------------------------------------------- tagging
def set_role(prim, role):
    """Tag a prim (role None clears the tag)."""
    if role is None:
        if prim.HasAttribute(ROLE_ATTR):
            prim.RemoveProperty(ROLE_ATTR)
        return
    from pxr import Sdf
    prim.CreateAttribute(ROLE_ATTR, Sdf.ValueTypeNames.Token, custom=True).Set(role)


def get_role(prim):
    attr = prim.GetAttribute(ROLE_ATTR)
    return attr.Get() if attr and attr.HasAuthoredValue() else None


_NAME_RULES = (("forklift", re.compile(r"forklift", re.I)),
               ("rack", re.compile(r"rack", re.I)),
               ("pallet", re.compile(r"pallet", re.I)),
               ("dock", re.compile(r"dock", re.I)))


def discover(stage):
    """{role: [prim, ...]}. Tagged prims win; with no tags at all, direct children of the default prim are
    matched by name (Forklift*, *Rack*, *Pallet*) so assets dropped in from the browsers just work."""
    found = {r: [] for r in ROLES}
    tagged = False
    for prim in stage.Traverse():
        role = get_role(prim)
        if role in found:
            found[role].append(prim)
            tagged = True
    if tagged:
        return found, "tags"
    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    for prim in root.GetChildren():
        for role, rx in _NAME_RULES:
            if rx.search(prim.GetName()):
                found[role].append(prim)
                break
    return found, "names"


# ----------------------------------------------------------------------------------------------- transforms
class Placement:
    """Read/write a prim's plan pose through its translate + rotateXYZ ops (what Kit gives dropped assets)."""

    def __init__(self, prim, heading_offset=0.0):
        self.prim = prim
        xf = UsdGeom.Xformable(prim)
        ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
        self.translate = ops.get("xformOp:translate")
        self.rotate = ops.get("xformOp:rotateXYZ")
        if self.translate is None or self.rotate is None:
            raise ValueError(f"{prim.GetPath()} needs translate + rotateXYZ ops (drag it once with the Move tool, "
                             f"or reset its transform)")
        d = Usd.TimeCode.Default()
        self.t0 = Gf.Vec3d(self.translate.Get(d))
        self.r0 = Gf.Vec3d(self.rotate.Get(d))
        parent = prim.GetParent()
        self.parent_inv = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(d).GetInverse() \
            if parent and parent.IsA(UsdGeom.Xformable) else Gf.Matrix4d(1.0)
        self.world0 = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(d)
        self.origin0 = self.world0.ExtractTranslation()
        self.heading_offset = heading_offset           # world heading of "forward" when rotateXYZ.y == 0

    def heading0(self):
        return wrap180(self.r0[1] + self.heading_offset)

    def ops_for(self, x, y, z, heading):
        """translate, rotateXYZ values that put the origin at world (x, y, z) facing `heading`."""
        t = self.parent_inv.Transform(Gf.Vec3d(x, y, z))
        return Gf.Vec3d(t), Gf.Vec3d(self.r0[0], heading - self.heading_offset, self.r0[2])


# ----------------------------------------------------------------------------------------------- forklifts
# Known part layouts. Keys are matched against the forklift asset/prim name.
KNOWN_PROFILES = {
    "Forklift_C01": {
        "lift_groups": {
            1.0: ["Fork01", "Fork02", "MastStage04", "MastStage05", "MastStageWheel05", "MastStageWheel06",
                  "MastStageWheel07", "MastStageWheel08", "MastStageWheel09"],
            0.75: ["MastStage03", "MastStageWheel03", "MastStageWheel04", "MastStageWheel10", "MastStageWheel11"],
            0.5: ["MastChain01", "MastChain02", "Hose"],
            0.4: ["MastStage01", "MastStageWheel01", "MastStageWheel02"],
        },
        "max_lift": 400.0,
        "seat": {"x": 4.5, "hip_y": 111.0, "hip_z": -70.0, "ankle_y": 66.0, "ankle_z": -36.0, "guard_y": 203.6},
    },
    # Order picker: the operator platform and the forks are one mesh (OperatorCab) that rides up the mast.
    "Forklift_B01": {
        "lift_groups": {
            1.0: ["OperatorCab", "OperatorCabGlass", "OperatorCabDecal", "OperatorCabPedal",
                  "OperatorCabTopWheels", "OperatorCabBotWheels"],
        },
        "max_lift": 190.0,                  # cab rollers reach the top of the (fixed) mast
        "forks_in": {"mesh": "OperatorCab", "max_up": 40.0, "min_fwd": 105.0},
        # the control panel is at the back of the platform, against the mast: the operator faces it
        "stand": {"x": 0.0, "floor_y": 23.0, "z": 80.0, "face": "back", "console_y": 142.0, "console_z": 52.0,
                  "hand_spread": 18.0, "canopy_y": 225.0},
    },
}

_FORK_RX = re.compile(r"fork(?!lift)", re.I)      # "Fork01", "Fork_A01" but not "Forklift"/"Forkliftt"


@dataclass
class Wheel:
    path: str
    radius: float
    rear: bool
    spin_idx: int
    spin_sign: float
    steer_idx: int
    steer_sign: float
    rest: tuple
    mode: str = "rotate"              # "rotate": its own rotateXYZ op; "roll": an extra transform op about its axle
    center_local: Gf.Vec3d = None     # roll mode: wheel center / axle / up in the mesh's own space
    axle_local: Gf.Vec3d = None
    up_local: Gf.Vec3d = None


@dataclass
class SteeringWheel:
    path: str
    center: Gf.Vec3d          # forklift frame F
    axis: Gf.Vec3d            # rim normal, toward the driver
    top: Gf.Vec3d             # 12 o'clock in the rim plane
    radius: float
    knob: tuple = None        # (clock degrees, in-plane radius, lift above rim) or None
    local_center: Gf.Vec3d = None   # same circle in the mesh's own space (for the steering op)
    local_axis: Gf.Vec3d = None


@dataclass
class Forklift:
    prim: Usd.Prim
    name: str
    place: Placement
    origin: tuple             # plan (x, z) at rest
    heading: float
    half_width: float
    rear: float               # body extent behind the origin (positive cm)
    front: float              # mast / fork heel ahead of the origin
    fork_heel: float
    fork_tip: float
    fork_spread: float
    blade_top: float
    blade_thick: float
    max_lift: float
    lift_parts: list = field(default_factory=list)    # (fraction, path, rest translate, up vector in part space)
    wheels: list = field(default_factory=list)
    wheelbase: float = 110.0
    steering: SteeringWheel = None
    seat: dict = None                 # seated driver layout (counterbalance trucks)
    stand: dict = None                # standing operator layout (order pickers: rides up with the platform)
    warnings: list = field(default_factory=list)

    def to_F(self, p_world):
        """World point -> forklift frame (left, up, forward) at the rest pose."""
        d = Gf.Vec3d(p_world) - Gf.Vec3d(self.origin[0], 0.0, self.origin[1])
        f, l = fwd_of(self.heading), left_of(self.heading)
        return Gf.Vec3d(d[0] * l[0] + d[2] * l[1], p_world[1], d[0] * f[0] + d[2] * f[1])


def _profile_for(prim):
    names = [prim.GetName()]
    for spec in prim.GetPrimStack():
        for ref in list(spec.referenceList.prependedItems) + list(spec.payloadList.prependedItems):
            names.append(str(ref.assetPath))
    for key, prof in KNOWN_PROFILES.items():
        if any(key.lower() in n.lower() for n in names):
            return key, prof
    return None, None


_SKIP_CHILDREN = ("Driver", "Operator")      # characters this tool (or the first scripts) seated in a forklift


def _meshes(prim):
    out = []
    it = iter(Usd.PrimRange(prim))
    for p in it:
        if p != prim and p.GetParent() == prim and p.GetName() in _SKIP_CHILDREN:
            it.PruneChildren()
            continue
        if p.IsA(UsdGeom.Mesh):
            out.append(p)
    return out


def _world_points(mesh):
    m = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pts = UsdGeom.Mesh(mesh).GetPointsAttr().Get() or []
    return [m.Transform(Gf.Vec3d(p)) for p in pts]


def _short(name):
    """'SM_Forklift_C01_Fork01_01' -> 'Fork01' style key for profile matching."""
    return name


def measure_forklift(prim, bbox):
    d = Usd.TimeCode.Default()
    meshes = _meshes(prim)
    if not meshes:
        raise ValueError(f"{prim.GetPath()}: no meshes (is its payload loaded?)")
    key, prof = _profile_for(prim)
    forks = [m for m in meshes if _FORK_RX.search(m.GetName())]
    world0 = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(d)
    o = world0.ExtractTranslation()
    origin = (o[0], o[2])

    total = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
    fork_pts = None                    # world points of forks that live inside a bigger mesh (profile "forks_in")
    if forks:
        fc = Gf.Vec3d(0, 0, 0)
        for m in forks:
            fc += bbox.ComputeWorldBound(m).ComputeAlignedRange().GetMidpoint()
        fc /= len(forks)
    else:
        fc = total.GetMidpoint()
    heading = heading_of(fc[0] - o[0], fc[2] - o[2])
    fin = (prof or {}).get("forks_in")
    if not forks and fin:
        host = next((m for m in meshes if re.search(r"(^|_)" + re.escape(fin["mesh"]) + r"(_|$)", m.GetName())), None)
        if host is not None:
            f0, l0 = fwd_of(heading), left_of(heading)
            sel = []
            for p in _world_points(host):
                dx, dz = p[0] - o[0], p[2] - o[2]
                if p[1] < fin["max_up"] and dx * f0[0] + dz * f0[1] > fin["min_fwd"]:
                    sel.append(p)
            if sel:
                fork_pts = sel
                fc = sum(sel, Gf.Vec3d(0, 0, 0)) / len(sel)
                heading = heading_of(fc[0] - o[0], fc[2] - o[2])
    rot = UsdGeom.Xformable(prim).GetOrderedXformOps()
    rot_y = next((op.Get(d)[1] for op in rot if op.GetOpName() == "xformOp:rotateXYZ"), 0.0)
    place = Placement(prim, heading_offset=wrap180(heading - rot_y))

    fl = Forklift(prim=prim, name=prim.GetName(), place=place, origin=origin, heading=heading,
                  half_width=0, rear=0, front=0, fork_heel=0, fork_tip=0, fork_spread=60, blade_top=14,
                  blade_thick=4, max_lift=(prof or {}).get("max_lift", 300.0), seat=(prof or {}).get("seat"),
                  stand=(prof or {}).get("stand"))
    if not forks and fork_pts is None:
        fl.warnings.append("no fork meshes found: pallets will be carried but the forks won't move")

    # footprint in the forklift frame
    lefts, fwds, body_fwds = [], [], []
    for m in meshes:
        for c in _local_corners_world(prim, m, bbox):          # boxes in the forklift's own frame: tight at any angle
            F = fl.to_F(c)
            lefts.append(F[0])
            fwds.append(F[2])
            if m not in forks:
                body_fwds.append(F[2])
    fl.half_width = max(abs(min(lefts)), abs(max(lefts)))
    fl.rear = -min(fwds)

    # forks: heel, tip, blade top
    if forks or fork_pts:
        pts = [fl.to_F(p) for p in fork_pts] if fork_pts else [fl.to_F(p) for m in forks for p in _world_points(m)]
        a_min, a_max = min(p[2] for p in pts), max(p[2] for p in pts)
        tip_zone = [p for p in pts if p[2] > a_min + 0.7 * (a_max - a_min)]
        mid_zone = [p for p in pts if a_min + 0.4 * (a_max - a_min) < p[2] < a_min + 0.7 * (a_max - a_min)]
        fl.fork_heel, fl.fork_tip = a_min + 2.0, a_max
        fl.blade_top = max(p[1] for p in tip_zone)
        fl.blade_thick = max(p[1] for p in mid_zone) - min(p[1] for p in mid_zone) if mid_zone else 4.0
        fl.fork_spread = max(p[0] for p in pts) - min(p[0] for p in pts)
    else:
        fl.fork_heel = max(body_fwds)
        fl.fork_tip = fl.fork_heel + 110.0
    fl.front = fl.fork_heel

    # lifting parts
    up_world = Gf.Vec3d(0, 1, 0)
    groups = (prof or {}).get("lift_groups") or {1.0: [m.GetName() for m in forks]}
    for frac, keys in groups.items():
        for m in meshes:
            n = m.GetName()
            if any(re.search(r"(^|_)" + re.escape(k) + r"(_|$)", n) for k in keys) or (prof is None and m in forks):
                ops = {op.GetOpName(): op for op in UsdGeom.Xformable(m).GetOrderedXformOps()}
                t = ops.get("xformOp:translate")
                if t is None:
                    fl.warnings.append(f"{n}: no translate op, it won't lift")
                    continue
                parent_inv = UsdGeom.Xformable(m.GetParent()).ComputeLocalToWorldTransform(d).GetInverse()
                fl.lift_parts.append((frac, str(m.GetPath()), Gf.Vec3d(t.Get(d)), parent_inv.TransformDir(up_world)))

    # road wheels (touch the ground), front/rear split by forward position
    cand = []
    for m in meshes:
        n = m.GetName().lower()
        if "wheel" not in n or "steering" in n:
            continue
        r = bbox.ComputeWorldBound(m).ComputeAlignedRange()
        if r.IsEmpty() or r.GetMin()[1] > 3.0 or r.GetSize()[1] < 8.0:
            continue
        cand.append((m, r))
    if cand:
        fpos = [fl.to_F(r.GetMidpoint())[2] for _, r in cand]
        mid = (min(fpos) + max(fpos)) / 2.0
        rear_f = [f for f in fpos if f < mid] or [min(fpos)]
        front_f = [f for f in fpos if f >= mid] or [max(fpos)]
        fl.wheelbase = max(30.0, sum(front_f) / len(front_f) - sum(rear_f) / len(rear_f))
        l_vec = left_of(heading)
        left_w = Gf.Vec3d(l_vec[0], 0, l_vec[1])
        for (m, r), f in zip(cand, fpos):
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(m).GetOrderedXformOps()}
            rop = ops.get("xformOp:rotateXYZ")
            mw = UsdGeom.Xformable(m).ComputeLocalToWorldTransform(d)
            if rop is None:                       # no rotate op (e.g. translate + pivot): roll it with an extra op
                inv = mw.GetInverse()
                fl.wheels.append(Wheel(
                    path=str(m.GetPath()), radius=r.GetSize()[1] / 2.0, rear=f < mid,
                    spin_idx=0, spin_sign=1.0, steer_idx=2, steer_sign=1.0, rest=(0.0, 0.0, 0.0), mode="roll",
                    center_local=inv.Transform(r.GetMidpoint()), axle_local=inv.TransformDir(left_w).GetNormalized(),
                    up_local=inv.TransformDir(up_world).GetNormalized()))
                continue
            axes = [mw.TransformDir(Gf.Vec3d(*e)).GetNormalized() for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
            spin_idx = max(range(3), key=lambda i: abs(Gf.Dot(axes[i], left_w)))
            steer_idx = max(range(3), key=lambda i: abs(Gf.Dot(axes[i], up_world)))
            fl.wheels.append(Wheel(
                path=str(m.GetPath()), radius=r.GetSize()[1] / 2.0, rear=f < mid,
                spin_idx=spin_idx, spin_sign=1.0 if Gf.Dot(axes[spin_idx], left_w) > 0 else -1.0,
                steer_idx=steer_idx, steer_sign=1.0 if Gf.Dot(axes[steer_idx], up_world) > 0 else -1.0,
                rest=tuple(rop.Get(d))))

    # steering wheel
    sw = next((m for m in meshes if "steeringwheel" in m.GetName().lower().replace("_", "")), None)
    if sw is not None:
        try:
            fl.steering = _fit_steering_wheel(fl, sw)
        except Exception as e:   # odd geometry: animate without it
            fl.warnings.append(f"steering wheel not fitted: {e}")
    if fl.seat is None and fl.stand is None:
        fl.warnings.append("no driver layout for this model: no driver will be added")
    return fl


def _sym_eigvec_smallest(C):
    tr = C[0][0] + C[1][1] + C[2][2]
    M = [[(tr if i == j else 0.0) - C[i][j] for j in range(3)] for i in range(3)]
    v = Gf.Vec3d(0.3, 0.8, -0.5)
    for _ in range(300):
        v = Gf.Vec3d(*[sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]).GetNormalized()
    return v


def _circle_fit(uv):
    n = len(uv)
    Su = sum(u for u, v in uv); Sv = sum(v for u, v in uv)
    Suu = sum(u * u for u, v in uv); Svv = sum(v * v for u, v in uv); Suv = sum(u * v for u, v in uv)
    Suuu = sum(u ** 3 for u, v in uv); Svvv = sum(v ** 3 for u, v in uv)
    Suvv = sum(u * v * v for u, v in uv); Svuu = sum(v * u * u for u, v in uv)
    M = [[Suu, Suv, Su], [Suv, Svv, Sv], [Su, Sv, n]]
    b = [-(Suuu + Suvv), -(Svvv + Svuu), -(Suu + Svv)]

    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    D = det(M)
    sol = []
    for k in range(3):
        Mk = [row[:] for row in M]
        for i in range(3):
            Mk[i][k] = b[i]
        sol.append(det(Mk) / D)
    A, B, C = sol
    uc, vc = -A / 2.0, -B / 2.0
    return uc, vc, math.sqrt(max(uc * uc + vc * vc - C, 1e-9))


def _fit_steering_wheel(fl, mesh):
    pts = [fl.to_F(p) for p in _world_points(mesh)]
    c0 = sum(pts, Gf.Vec3d(0, 0, 0)) / len(pts)
    C = [[sum((p[i] - c0[i]) * (p[j] - c0[j]) for p in pts) / len(pts) for j in range(3)] for i in range(3)]
    axis = _sym_eigvec_smallest(C)
    axis = Gf.Vec3d(0.0, axis[1], axis[2]).GetNormalized()       # symmetric about the forklift centerline
    if axis[1] < 0:
        axis = -axis
    if axis[2] > 0:                                                 # must face back toward the driver
        axis = Gf.Vec3d(axis[0], axis[1], -abs(axis[2])).GetNormalized()
    top = Gf.Vec3d(0, 1, 0) - axis * axis[1]
    top.Normalize()
    right = Gf.Vec3d(-1, 0, 0)

    def polar(p):
        d = p - c0
        h = Gf.Dot(d, axis)
        u, v = Gf.Dot(d, right), Gf.Dot(d, top)
        return u, v, h, math.hypot(u, v), math.degrees(math.atan2(u, v))
    P = [polar(p) for p in pts]
    rmax = max(r for *_, r, _a in P)
    # knob: an angular sector with points standing clearly above the rim plane
    sectors = {}
    for u, v, h, r, a in P:
        if r > 0.6 * rmax:
            k = int((a + 180) // 15)
            sectors.setdefault(k, []).append(h)
    # a knob is a sector with far more geometry than the plain rim, standing above it
    counts = sorted(len(hs) for hs in sectors.values())
    median = counts[len(counts) // 2] if counts else 0
    knob_k = max(sectors, key=lambda k: len(sectors[k])) if sectors else None
    knob_ok = knob_k is not None and len(sectors[knob_k]) > 4 * max(median, 1) and max(sectors[knob_k]) > 2.0
    lo_a = (knob_k * 15 - 180 - 15) if knob_ok else 999
    hi_a = lo_a + 45
    rim = [(u, v) for u, v, h, r, a in P if 0.75 * rmax < r and not (lo_a < a < hi_a) and abs(h) < 2.5]
    if len(rim) < 20:
        raise ValueError("too few rim points")
    uc, vc, R = _circle_fit(rim)
    hs = [h for u, v, h, r, a in P if 0.75 * rmax < r and not (lo_a < a < hi_a) and abs(h) < 2.5]
    center = c0 + right * uc + top * vc + axis * (sum(hs) / len(hs))
    knob = None
    if knob_ok:
        kp = [p for p, (u, v, h, r, a) in zip(pts, P) if lo_a < a < hi_a and h > 2.0]
        kc = sum(kp, Gf.Vec3d(0, 0, 0)) / len(kp)
        d = kc - center
        kh = Gf.Dot(d, axis)
        kpl = d - axis * kh
        knob = (math.degrees(math.atan2(Gf.Dot(kpl, right), Gf.Dot(kpl, top))), kpl.GetLength(), kh)
    # the same circle in the mesh's own space, for the steering op
    d = Usd.TimeCode.Default()
    mw_inv = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(d).GetInverse()
    f, l = fwd_of(fl.heading), left_of(fl.heading)

    def F_to_world(p):
        return Gf.Vec3d(fl.origin[0] + l[0] * p[0] + f[0] * p[2], p[1], fl.origin[1] + l[1] * p[0] + f[1] * p[2])
    lc = mw_inv.Transform(F_to_world(center))
    la = mw_inv.TransformDir(F_to_world(center + axis) - F_to_world(center)).GetNormalized()
    return SteeringWheel(path=str(mesh.GetPath()), center=center, axis=axis, top=top, radius=R, knob=knob,
                         local_center=lc, local_axis=la)


# ----------------------------------------------------------------------------------------------- frames for racks / pallets
def _plan_frame(prim):
    """World matrix plus the prim's horizontal local axes as plan unit vectors and the local up axis index."""
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    axes = [m.TransformDir(Gf.Vec3d(*e)) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    up = max(range(3), key=lambda i: abs(axes[i][1]) / max(axes[i].GetLength(), 1e-9))
    horiz = [i for i in range(3) if i != up]
    return m, axes, up, horiz


def _local_corners_world(prim, child, bbox):
    rng = bbox.ComputeRelativeBound(child, prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return []
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return [m.Transform(c) for c in corners(rng)]


@dataclass
class Box2:
    """Oriented rectangle on the floor: center, unit axis u, half sizes (along u, along the perpendicular)."""
    c: tuple
    u: tuple
    hu: float
    hv: float
    y0: float = 0.0
    y1: float = 0.0

    @property
    def v(self):
        return (-self.u[1], self.u[0])

    def corners(self):
        u, v = self.u, self.v
        return [(self.c[0] + su * self.hu * u[0] + sv * self.hv * v[0],
                 self.c[1] + su * self.hu * u[1] + sv * self.hv * v[1]) for su, sv in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _box_from_points(pts_world, u):
    v = (-u[1], u[0])
    us = [p[0] * u[0] + p[2] * u[1] for p in pts_world]
    vs = [p[0] * v[0] + p[2] * v[1] for p in pts_world]
    cu, cv = (min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2
    return Box2(c=(cu * u[0] + cv * v[0], cu * u[1] + cv * v[1]), u=u, hu=(max(us) - min(us)) / 2,
                hv=(max(vs) - min(vs)) / 2, y0=min(p[1] for p in pts_world), y1=max(p[1] for p in pts_world))


# ----------------------------------------------------------------------------------------------- pallets
@dataclass
class Pallet:
    prim: Usd.Prim
    name: str
    place: Placement
    box: Box2                 # footprint, u = the pallet's first horizontal local axis
    height: float
    opening_top: float        # top of the fork pockets above the pallet base
    base_y: float             # bottom of the pallet in world
    rack: object = None       # Rack / Slot it sits in, if it is stored
    slot: object = None

    def half_along(self, d):
        """Half extent of the footprint along plan direction d (unit)."""
        b = self.box
        return abs(b.hu * (b.u[0] * d[0] + b.u[1] * d[1])) + abs(b.hv * (b.v[0] * d[0] + b.v[1] * d[1]))


def measure_pallet(prim, bbox):
    m, axes, up, horiz = _plan_frame(prim)
    a = axes[horiz[0]]
    u = (a[0], a[2])
    n = math.hypot(*u)
    u = (u[0] / n, u[1] / n)
    pts = []
    for mesh in _meshes(prim) or [prim]:
        pts += _local_corners_world(prim, mesh, bbox)
    if not pts:
        raise ValueError(f"{prim.GetPath()}: empty bounds (is its payload loaded?)")
    box = _box_from_points(pts, u)
    # pallet base = lowest mesh whose name says pallet, else the lowest mesh
    base_h = None
    for mesh in _meshes(prim):
        r = bbox.ComputeWorldBound(mesh).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        if "pallet" in mesh.GetName().lower() and r.GetSize()[1] < 25 and r.GetMin()[1] <= box.y0 + 1.0:
            base_h = r.GetSize()[1]
    opening = (base_h - 2.0) if base_h else 10.0
    return Pallet(prim=prim, name=prim.GetName(), place=Placement(prim), box=box, height=box.y1 - box.y0,
                  opening_top=max(6.0, opening), base_y=box.y0)


# ----------------------------------------------------------------------------------------------- racks
@dataclass
class Slot:
    rack: str
    level: int
    y: float                  # shelf top (pallet base sits here)
    center: tuple             # plan point on the rack's centerline (depth 0) at this slot's width position
    clearance: float          # free height above the shelf
    occupant: str = None


@dataclass
class Rack:
    prim: Usd.Prim
    name: str
    box: Box2                 # u = depth direction (the open faces are at +/- u)
    levels: list
    slots: list


def measure_rack(prim, bbox, pallet_width=90.0):
    m, axes, up, horiz = _plan_frame(prim)
    meshes = _meshes(prim)
    allpts = []
    per_mesh = []
    for mesh in meshes:
        c = _local_corners_world(prim, mesh, bbox)
        if c:
            per_mesh.append((mesh, c))
            allpts += c
    if not allpts:
        raise ValueError(f"{prim.GetPath()}: empty bounds (is its payload loaded?)")
    a0, a1 = axes[horiz[0]], axes[horiz[1]]
    u0 = (a0[0] / math.hypot(a0[0], a0[2]), a0[2] / math.hypot(a0[0], a0[2]))
    b0 = _box_from_points(allpts, u0)
    if b0.hu > b0.hv:                                  # make u the shorter side (depth)
        u1 = (a1[0] / math.hypot(a1[0], a1[2]), a1[2] / math.hypot(a1[0], a1[2]))
        b0 = _box_from_points(allpts, u1)
    box = b0
    # shelves: thin, wide meshes (or named like shelves)
    levels = []
    for mesh, c in per_mesh:
        b = _box_from_points(c, box.u)
        thick = b.y1 - b.y0
        named = re.search(r"shelf|beam|deck|level", mesh.GetName(), re.I)
        wide = b.hu > 0.7 * box.hu and b.hv > 0.7 * box.hv
        if (named and wide and thick < 60) or (wide and thick < 15):
            levels.append(b.y1)
    levels = sorted(set(round(y, 1) for y in levels))
    rack_top = box.y1
    slots = []
    inner = 2 * box.hv - 8.0                          # minus the uprights
    n_cols = max(1, int((inner + 4.0) // (pallet_width + 2.0)))
    pitch = inner / n_cols
    v = box.v
    for li, y in enumerate(levels):
        above = [z - 40.0 for z in levels if z > y + 1.0]      # next shelf top minus its beam depth (~40)
        clearance = (min(above) if above else rack_top + 100.0) - y
        for ci in range(n_cols):
            off = -inner / 2 + pitch * (ci + 0.5)
            slots.append(Slot(rack=prim.GetName(), level=li, y=y, clearance=clearance,
                              center=(box.c[0] + v[0] * off, box.c[1] + v[1] * off)))
    return Rack(prim=prim, name=prim.GetName(), box=box, levels=levels, slots=slots)


# ----------------------------------------------------------------------------------------------- whole scene
@dataclass
class Dock:
    """A floor area where retrieved pallets are put down."""
    prim: Usd.Prim
    name: str
    box: Box2


def measure_dock(prim, bbox):
    m, axes, up, horiz = _plan_frame(prim)
    a = axes[horiz[0]]
    n = math.hypot(a[0], a[2]) or 1.0
    pts = []
    for mesh in _meshes(prim) or [prim]:
        pts += _local_corners_world(prim, mesh, bbox)
    if not pts:
        raise ValueError(f"{prim.GetPath()}: a dock needs some geometry (e.g. a plane) to mark its area")
    return Dock(prim=prim, name=prim.GetName(), box=_box_from_points(pts, (a[0] / n, a[2] / n)))


@dataclass
class Scene:
    forklifts: list
    racks: list
    pallets: list
    obstacles: list           # Box2
    source: str
    warnings: list
    docks: list = field(default_factory=list)


def scan(stage):
    if UsdGeom.GetStageUpAxis(stage) != "Y":
        raise ValueError("this tool expects a Y-up stage")
    found, source = discover(stage)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    warnings = []
    forklifts, racks, pallets, obstacles = [], [], [], []
    for p in found["pallet"]:
        try:
            pallets.append(measure_pallet(p, bbox))
        except ValueError as e:
            warnings.append(str(e))
    pw = max([max(p.box.hu, p.box.hv) * 2 for p in pallets] or [90.0])
    for p in found["rack"]:
        try:
            racks.append(measure_rack(p, bbox, pallet_width=pw))
        except ValueError as e:
            warnings.append(str(e))
    for p in found["forklift"]:
        try:
            fl = measure_forklift(p, bbox)
            forklifts.append(fl)
            warnings += [f"{fl.name}: {w}" for w in fl.warnings]
        except ValueError as e:
            warnings.append(str(e))
    for p in found["obstacle"]:
        pts = []
        for mesh in _meshes(p) or [p]:
            r = bbox.ComputeWorldBound(mesh).ComputeAlignedRange()
            if not r.IsEmpty():
                pts += corners(r)
        if pts:
            obstacles.append(_box_from_points(pts, (1.0, 0.0)))
    docks = []
    for p in found["dock"]:
        try:
            docks.append(measure_dock(p, bbox))
        except ValueError as e:
            warnings.append(str(e))
    # pallets already sitting in a rack slot
    for pal in pallets:
        if pal.base_y < 5.0:
            continue
        best = None
        for rack in racks:
            for s in rack.slots:
                d = math.dist(s.center, pal.box.c)
                if abs(s.y - pal.base_y) < 15 and d < pw and (best is None or d < best[0]):
                    best = (d, s)
        if best:
            best[1].occupant = pal.name
            pal.slot = best[1]
            pal.rack = next(r for r in racks if best[1] in r.slots)
    return Scene(forklifts, racks, pallets, obstacles, source, warnings, docks)


def summary(scene):
    lines = [f"Found by {scene.source}: {len(scene.forklifts)} forklift(s), {len(scene.racks)} rack(s), "
             f"{len(scene.pallets)} pallet(s), {len(scene.obstacles)} obstacle(s)."]
    for fl in scene.forklifts:
        lines.append(f"  {fl.name}: forks {fl.fork_tip - fl.fork_heel:.0f} cm, max lift {fl.max_lift:.0f} cm, "
                     f"{len(fl.lift_parts)} lifting parts, {len(fl.wheels)} wheels, "
                     f"steering {'yes' if fl.steering else 'no'}, "
                     f"driver {'seated' if fl.seat else 'standing' if fl.stand else 'none'}")
    for r in scene.racks:
        free = sum(1 for s in r.slots if s.occupant is None)
        lines.append(f"  {r.name}: {len(r.levels)} shelves at {', '.join(f'{y:.0f}' for y in r.levels)} cm, "
                     f"{len(r.slots)} slots ({free} free)")
    floor = [p for p in scene.pallets if p.base_y < 5.0]
    stored = [p for p in scene.pallets if p.slot is not None]
    lines.append(f"  pallets on the floor: {len(floor)}, in racks: {len(stored)}")
    for d in scene.docks:
        lines.append(f"  dock {d.name}: {2 * d.box.hu / 100:.1f} x {2 * d.box.hv / 100:.1f} m")
    lines += ["  ! " + w for w in scene.warnings]
    return "\n".join(lines)
