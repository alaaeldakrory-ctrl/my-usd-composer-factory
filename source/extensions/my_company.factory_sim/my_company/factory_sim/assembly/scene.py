"""Builds the assembly-demo stage around the ES2632 BIM model converted to USD (see es2632_asset.py): the machine,
its six guardrail components as movable parts, floor stands, a parts cart, wrenches, lights and two NVIDIA characters.

World: Y up, cm. The lift sits on the origin with its long axis along X (+X = entry-gate end); the deck top is at
DECK_Y. Every guardrail part is animated by the transform op on /World/ScissorLift/Kit/<name>; identity = installed.
"""
import math

import os

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt

from . import pbr

CHARS = r"C:\Nvidia_Assets\Characters_NVD\Assets\Characters\Reallusion\ActorCore"
WORKER_A = CHARS + r"\Uniform_M_0001\uniform_m_0001.usd"      # vest, gloves
WORKER_B = CHARS + r"\Uniform_F_0001\uniform_f_0001.usd"      # vest, cap, gloves
LIFT_ASSET = r"C:\OmniverseData\ES2632\ES2632_kit.usd"
ENV_ASSET = r"C:\OmniverseData\ES2632\Warehouse_Env.usd"      # warehouse_env.py; optional
KIT = "/World/ScissorLift/Kit"

DECK_Y = 119.4
POST_R = 1.6                                   # guardrail tube radius
SIDE_POSTS = (-108.2, 102.5)                   # x of the side panels' corner posts (rear, front)
SIDE_Z = 34.4                                  # |z| of the side panels' post centerline
SIDE_PIVOT = Gf.Vec3d(-2.7, 118.7, SIDE_Z)     # bottom center of the +Z panel (mirror z for -Z)
SIDE_BOLT_Y = (121.0, 130.7)                   # bracket bolts on each corner post
STAND_Y = 45.0                                 # side rails wait on stands with their post feet this high
STAND_Z = 230.0
CART = ((250.0, 316.0), (-60.0, 60.0), 78.0)   # x range, z range, top height
# entry gate: uprights at z +23.8 (latch side) / -20.8 (hinge side), x 101.5; hinge axis on the right front post
GATE_UPRIGHTS = (23.8, -20.8)
GATE_PIVOT = Gf.Vec3d(101.5, 119.2, 1.5)
GATE_HINGE = Gf.Vec3d(99.6, 0.0, -24.8)
GATE_START = (Gf.Vec3d(261.0, 78.0, 0.0), 180.0)
# extension-deck U-frame: end posts at z +-19.3 (x -115.3), short side rails at z +-34.45 (mid rail y 179.35)
EXT_POSTS = (19.3, -19.3)
EXT_PIVOT = Gf.Vec3d(-115.3, 120.0, 0.0)
EXT_BOLT_Y = (122.7, 131.3)
EXT_MID = (179.35, 34.45)
EXT_START = (Gf.Vec3d(-375.3, 40.0, -140.0), 0.0)
# front top crossbar between the front corner posts (x 102.5, y 226.7..229.9, z -31..31)
BAR_PIVOT = Gf.Vec3d(102.5, 226.7, 0.0)
BAR_START = (Gf.Vec3d(305.0, 78.0, 0.0), 0.0)
BAR_BOLT_Z = 30.9
# platform controller (box + hanger): bottom center, hanger top edge
CTRL_PIVOT = Gf.Vec3d(-91.2, 166.0, -26.2)
PARTS_TABLE = ((-50.0, 10.0), (-232.0, -182.0), 78.0)   # small table beside the right rail
CTRL_START = (Gf.Vec3d(-20.0, 78.0, -200.0), 180.0)
CTRL_HANDLE = Gf.Vec3d(-89.0, 211.0, -34.2)
# aluminium 2-step stand (Warehouse_NVD AluminumStepStand_A02): treads at 25 / 50 cm
STEP_ASSET = r"C:/Nvidia_Assets/Warehouse_NVD/Assets/DigitalTwin/Assets/Warehouse/Equipment/Ladders/AlumStepStand_A/AluminumStepStand_A02_PR_NVD_01.usd"
STEP_FRONT = (Gf.Vec3d(150.0, 0.0, 0.0), 90.0)       # at the entry end, low step toward +X
STEP_SIDE = (Gf.Vec3d(-89.0, 0.0, -70.0), 180.0)     # beside the right rail, low step toward -Z


# --------------------------------------------------------------------------------------------- materials / prims
def _mat(stage, name, rgb, rough=0.45, metal=0.0):
    path = f"/World/Looks/{name}"
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


def _bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def box(stage, path, lo, hi, mat):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
    _bind(c.GetPrim(), mat)
    c.CreateDisplayColorAttr([Gf.Vec3f(*mat_colour(mat))])
    return c


def tube(stage, path, a, b, r, mat):
    """Cylinder from point a to point b (local coordinates of its parent)."""
    a, b = Gf.Vec3d(*a), Gf.Vec3d(*b)
    d = b - a
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(d.GetLength())
    c.CreateAxisAttr("Y")
    rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), d.GetNormalized())
    UsdGeom.Xformable(c).AddTransformOp().Set(Gf.Matrix4d().SetRotate(rot) * Gf.Matrix4d().SetTranslate((a + b) / 2))
    _bind(c.GetPrim(), mat)
    c.CreateDisplayColorAttr([Gf.Vec3f(*mat_colour(mat))])
    return c


_COLOURS = {}


def mat_colour(mat):
    return _COLOURS.get(str(mat.GetPath()), (0.5, 0.5, 0.5))


# --------------------------------------------------------------------------------------------- parts
def tube_grip(c, axis=(0, 1, 0), thumb=(0, 1, 0), r=POST_R):
    """A hand hold on a tube, in the part's installed (= world when seated) frame."""
    return {"kind": "tube", "c": Gf.Vec3d(*c), "axis": Gf.Vec3d(*axis), "thumb": Gf.Vec3d(*thumb), "r": r}


class Part:
    """A guardrail group: prim path, pivot (bottom center when installed), hand holds and start placement."""

    def __init__(self, name, pivot, holds, start, path=None):
        self.name, self.pivot, self.holds, self.start = name, Gf.Vec3d(pivot), holds, start
        self.path = path or f"{KIT}/{name}"

    def at(self, pos, yaw=0.0):
        """Matrix that moves the part's pivot to `pos`, turned by `yaw` degrees about +Y."""
        return Gf.Matrix4d().SetTranslate(-self.pivot) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 1, 0), yaw)) * \
            Gf.Matrix4d().SetTranslate(Gf.Vec3d(pos))

    def installed(self):
        return Gf.Matrix4d(1)

    def lifted(self, dy, dz=0.0, dx=0.0):
        return self.at(self.pivot + Gf.Vec3d(dx, dy, dz))


def bar_grip(c, axis, thumb, r=POST_R):
    return tube_grip(c, axis, thumb, r)


# where each worker stands beside a side rail's corner post (outboard, clear of the stands) and grip heights
SIDE_STANCE = {"A": -28.0, "B": 28.0}
SIDE_GRIPS = {"A": (10.0, 28.0), "B": (3.0, 15.0)}


def make_parts():
    parts = {}
    for side, zs in (("L", 1.0), ("R", -1.0)):
        holds = {}
        for who, x in (("A", SIDE_POSTS[0]), ("B", SIDE_POSTS[1])):
            lo, hi = SIDE_GRIPS[who]
            base = Gf.Vec3d(x, SIDE_PIVOT[1], zs * SIDE_Z)
            holds[who] = {"lo": tube_grip(base + Gf.Vec3d(0, lo, 0)), "hi": tube_grip(base + Gf.Vec3d(0, hi, 0))}
        pv = Gf.Vec3d(SIDE_PIVOT[0], SIDE_PIVOT[1], zs * SIDE_Z)
        parts["Side" + side] = Part("Side" + side, pv, holds, (Gf.Vec3d(pv[0], STAND_Y, zs * STAND_Z), 0.0))
    ext = {"A": {"post%d" % k: tube_grip((EXT_PIVOT[0], EXT_PIVOT[1] + 10.0, z)) for k, z in enumerate(EXT_POSTS)},
           "steady": {"post%d" % k: tube_grip((EXT_PIVOT[0], EXT_PIVOT[1] + 30.0, z)) for k, z in enumerate(EXT_POSTS)},
           "B": {"front": bar_grip((-40.0, EXT_MID[0], EXT_MID[1]), (1, 0, 0), (-1, 0, 0)),
                 "back": bar_grip((-62.0, EXT_MID[0], EXT_MID[1]), (1, 0, 0), (1, 0, 0))}}
    parts["ExtFrame"] = Part("ExtFrame", EXT_PIVOT, ext, EXT_START)
    parts["Gate"] = Part("Gate", GATE_PIVOT,
                         {"B": {"up%d" % k: tube_grip((GATE_PIVOT[0], GATE_PIVOT[1] + 8.0, z)) for k, z in enumerate(GATE_UPRIGHTS)},
                          "push": tube_grip((GATE_PIVOT[0], GATE_PIVOT[1] + 30.0, GATE_UPRIGHTS[0]))}, GATE_START)
    bc = Gf.Vec3d(BAR_PIVOT[0], BAR_PIVOT[1] + 1.6, 0.0)
    parts["CrossBar"] = Part("CrossBar", BAR_PIVOT,
                             {"A": {"l": bar_grip(bc + Gf.Vec3d(0, 0, 13), (0, 0, 1), (0, 0, -1)),
                                    "r": bar_grip(bc + Gf.Vec3d(0, 0, -13), (0, 0, 1), (0, 0, 1))},
                              "B": {"l": bar_grip(bc + Gf.Vec3d(0, 0, -24), (0, 0, 1), (0, 0, 1)),
                                    "r": bar_grip(bc + Gf.Vec3d(0, 0, 24), (0, 0, 1), (0, 0, -1))},
                              "steady": bar_grip(bc, (0, 0, 1), (0, 0, 1))}, BAR_START)
    parts["Controller"] = Part("Controller", CTRL_PIVOT, {"handle": bar_grip(CTRL_HANDLE, (1, 0, 0), (1, 0, 0), 1.2)},
                               CTRL_START)
    # the step stand (a prop, Y-up wrapper around the Z-up asset): grab its side rails near the low step
    parts["StepStand"] = Part("StepStand", Gf.Vec3d(0, 0, 0),
                              {"L": tube_grip((31.0, 44.0, 14.0), (0, 0, 1), (0, 0, -1)),
                               "R": tube_grip((-31.0, 44.0, 14.0), (0, 0, 1), (0, 0, -1))},
                              STEP_FRONT, path="/World/Props/StepStand")
    return parts


def step_height(m, p):
    """Floor height (cm) at world point p over the step stand placed by matrix m (0 if not on it)."""
    q = m.GetInverse().Transform(Gf.Vec3d(p[0], 0, p[2]))
    x, y = q[0], -q[2]                       # back to the asset's (Z-up) plan: width x, depth y
    if abs(x) > 32.0 or abs(y) > 26.0:
        return 0.0
    return 25.0 if y < -6.0 else 50.0


def tread_point(placement, tread):
    """World floor point at the middle of the low (0) or top (1) tread for a stand placement (pos, heading)."""
    pos, h = placement
    local = Gf.Vec3d(0, 0, 16.5) if tread == 0 else Gf.Vec3d(0, 0, -10.5)
    w = Gf.Rotation(Gf.Vec3d(0, 1, 0), h).TransformDir(local)
    return Gf.Vec3d(pos[0] + w[0], 25.0 if tread == 0 else 50.0, pos[2] + w[2])


def side_bolt(side, x, k):
    """World point of a side-post bracket bolt head and the bolt axis (pointing out of the deck)."""
    zs = 1.0 if side == "L" else -1.0
    return Gf.Vec3d(x, SIDE_BOLT_Y[k], zs * (SIDE_Z + 2.3)), Gf.Vec3d(0, 0, zs)


def ext_bolt(z, k):
    return Gf.Vec3d(EXT_PIVOT[0] - 1.6, EXT_BOLT_Y[k], z), Gf.Vec3d(-1, 0, 0)


def bar_bolt(zs):
    return Gf.Vec3d(BAR_PIVOT[0] + 2.2, BAR_PIVOT[1] + 1.6, zs * BAR_BOLT_Z), Gf.Vec3d(1, 0, 0)


# --------------------------------------------------------------------------------------------- bolt hardware
def _prism(n, r, z0, z1, phase=0.0):
    """Closed n-gon prism along +Z: points, counts, indices."""
    import math
    pts = []
    for z in (z0, z1):
        for k in range(n):
            a = phase + 2 * math.pi * k / n
            pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), z))
    counts, idx = [n, n], list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    for k in range(n):
        j = (k + 1) % n
        counts.append(4)
        idx += [k, j, n + j, n + k]
    return pts, counts, idx


def bolt_mesh(stage, path, mat):
    """M10 hex bolt as seen from outside: hex head (16 mm A/F) on a washer, short shank; local +Z = out of the joint."""
    import math
    parts = [_prism(6, 0.924, 0.2, 0.85, math.pi / 6), _prism(18, 1.05, 0.0, 0.2), _prism(12, 0.5, -2.2, 0.0)]
    pts, counts, idx = [], [], []
    for p, c, i in parts:
        base = len(pts)
        pts += p
        counts += c
        idx += [base + v for v in i]
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.71, 0.72)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def write_bolts(stage, events, n_frames, fps=30.0, kit=KIT):
    """Bolts appear when the ratchet goes on and screw in (1.4 cm, two turns) while it works."""
    import math
    zinc = UsdShade.Material(stage.GetPrimAtPath("/World/Looks/Zinc"))
    for k, (part, bp, axis, t0, t1) in enumerate(events):
        path = f"{kit}/{part}/Bolts/Bolt{k:02d}"
        UsdGeom.Xform.Define(stage, f"{kit}/{part}/Bolts")
        holder = UsdGeom.Xform.Define(stage, path)
        bolt_mesh(stage, path + "/Mesh", zinc)
        op = holder.AddTransformOp()
        vis = holder.CreateVisibilityAttr()
        ax = Gf.Vec3d(axis).GetNormalized()
        base = Gf.Rotation(Gf.Vec3d(0, 0, 1), ax)
        f0, f1 = int(t0 * fps), int(t1 * fps) + 1
        for f in [0] + list(range(max(1, f0 - 1), min(n_frames, f1 + 2))):
            u = min(max((f / fps - t0) / max(t1 - t0, 1e-6), 0.0), 1.0)
            spin = Gf.Rotation(Gf.Vec3d(0, 0, 1), 720.0 * u)
            m = Gf.Matrix4d().SetRotate(spin * base) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(bp) + ax * (1.4 * (1 - u) - 0.85))
            op.Set(m, f)
        vis.Set("invisible", 0)
        vis.Set("invisible", max(0, f0 - 1))
        vis.Set("inherited", f0)


# --------------------------------------------------------------------------------------------- props
def wrench(stage, path, mats):
    UsdGeom.Xform.Define(stage, path)
    tube(stage, f"{path}/Handle", (0, 0, 0), (0, 0, 22), 1.0, mats["steel"])
    tube(stage, f"{path}/Head", (-1.8, 0, 23), (1.8, 0, 23), 2.0, mats["steel"])
    tube(stage, f"{path}/Grip", (0, 0, 1), (0, 0, 12), 1.45, mats["black"])
    UsdGeom.Imageable(stage.GetPrimAtPath(path)).CreateVisibilityAttr("invisible")


def character(stage, path, asset):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetPayloads().AddPayload(asset)
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    stage.Load(prim.GetPath())
    return prim


def build(stage):
    UsdGeom.SetStageUpAxis(stage, "Y")
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.SetTimeCodesPerSecond(30)
    stage.SetFramesPerSecond(30)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    UsdGeom.Scope.Define(stage, "/World/Looks")
    spec = {"black": ((0.04, 0.04, 0.045), "black_powder"), "grey": ((0.45, 0.46, 0.48), "bright_steel"),
            "steel": ((0.62, 0.63, 0.66), "bright_steel"), "floor": ((0.32, 0.33, 0.34), None),
            "cart": ((0.18, 0.2, 0.23), "steel_tread"), "yellow": ((0.95, 0.75, 0.05), "yellow_tape"),
            "rubber": ((0.05, 0.05, 0.05), "tyre_rubber"), "zinc": ((0.7, 0.71, 0.72), "zinc_bolt")}
    mats = {}
    for k, (rgb, look) in spec.items():
        path = f"/World/Looks/{k.capitalize()}"
        mats[k] = pbr.material(stage, path, look) if look else _mat(stage, k.capitalize(), rgb, 0.85)
        _COLOURS[str(mats[k].GetPath())] = rgb
    env = os.path.exists(ENV_ASSET)
    if env:
        stage.DefinePrim("/World/Environment", "Xform").GetReferences().AddReference(ENV_ASSET)
    else:
        box(stage, "/World/Floor", (-1500, -1, -1500), (1500, 0, 1500), mats["floor"])
    for k, (lo, hi) in enumerate((((-190, 0, -175), (190, 0.3, -171)), ((-190, 0, 171), (190, 0.3, 175)),
                                  ((-194, 0, -175), (-190, 0.3, 175)), ((190, 0, -175), (194, 0.3, 175)))):
        box(stage, f"/World/Marking{k}", (lo[0], 0.05, lo[2]), (hi[0], 0.25, hi[2]), mats["yellow"])

    lift = stage.DefinePrim("/World/ScissorLift", "Xform")
    lift.GetReferences().AddReference(LIFT_ASSET)
    parts = make_parts()
    for side, zs in (("L", 1.0), ("R", -1.0)):          # stands under the rails' post feet, reaching inboard
        for j, x in enumerate(SIDE_POSTS):
            lo_x, hi_x = (x - 6, x + 26) if j == 0 else (x - 26, x + 6)
            box(stage, f"/World/Stands/Stand{side}{j}", (lo_x, 0, zs * STAND_Z - 9), (hi_x, STAND_Y, zs * STAND_Z + 9), mats["black"])
    # extension U-frame storage: a block under its end posts and two pedestals under the short side rails
    ex, ey, ez = EXT_START[0]
    box(stage, "/World/Stands/ExtPosts", (ex - 6, 0, ez - 27), (ex + 6, ey, ez + 27), mats["black"])
    for k, zs in enumerate((1.0, -1.0)):
        x = ex + (-17.1 - EXT_PIVOT[0]) - 22.0
        box(stage, f"/World/Stands/ExtRail{k}", (x - 6, 0, ez + zs * EXT_MID[1] - 6),
            (x + 6, ey + (EXT_MID[0] - EXT_PIVOT[1]) - 1.6, ez + zs * EXT_MID[1] + 6), mats["black"])
    stand = stage.DefinePrim("/World/Props/StepStand", "Xform")
    UsdGeom.Xformable(stand).AddTransformOp()
    inner = stage.DefinePrim("/World/Props/StepStand/Asset", "Xform")
    inner.GetReferences().AddReference(STEP_ASSET)
    UsdGeom.Xformable(inner).AddRotateXOp(opSuffix="zup").Set(-90.0)
    for p in parts.values():
        UsdGeom.Xformable(stage.GetPrimAtPath(p.path)).GetOrderedXformOps()[0].Set(p.at(*p.start))
    (x0, x1), (z0, z1), top = PARTS_TABLE
    UsdGeom.Xform.Define(stage, "/World/PartsTable")
    box(stage, "/World/PartsTable/Top", (x0, top - 4, z0), (x1, top, z1), mats["cart"])
    for j, (x, z) in enumerate(((x0 + 3, z0 + 3), (x1 - 3, z0 + 3), (x0 + 3, z1 - 3), (x1 - 3, z1 - 3))):
        tube(stage, f"/World/PartsTable/Leg{j}", (x, 0, z), (x, top - 4, z), 1.6, mats["grey"])
    (x0, x1), (z0, z1), top = CART
    UsdGeom.Xform.Define(stage, "/World/Cart")
    box(stage, "/World/Cart/Top", (x0, top - 4, z0), (x1, top, z1), mats["cart"])
    for j, (x, z) in enumerate(((x0 + 4, z0 + 4), (x1 - 4, z0 + 4), (x0 + 4, z1 - 4), (x1 - 4, z1 - 4))):
        tube(stage, f"/World/Cart/Leg{j}", (x, 8, z), (x, top - 4, z), 1.6, mats["grey"])
        tube(stage, f"/World/Cart/Caster{j}", (x, 4, z - 2), (x, 4, z + 2), 4.0, mats["rubber"])
    UsdGeom.Xform.Define(stage, "/World/Tools")
    for n in ("A", "B"):
        wrench(stage, f"/World/Tools/Wrench{n}", mats)

    if not env:                                   # studio lighting only without the warehouse (it brings its own)
        dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
        dome.CreateIntensityAttr(700.0)
        key = UsdLux.RectLight.Define(stage, "/World/KeyLight")
        key.CreateIntensityAttr(9000.0)
        key.CreateWidthAttr(400.0)
        key.CreateHeightAttr(400.0)
        UsdGeom.Xformable(key).AddTransformOp().Set(
            Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), -70) * Gf.Rotation(Gf.Vec3d(0, 1, 0), 30)) *
            Gf.Matrix4d().SetTranslate(Gf.Vec3d(250, 600, 400)))
    stage.SetMetadataByDictKey("customLayerData", "assembly:zinc", str(mats["zinc"].GetPath()))
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Overview")
    cam.CreateFocalLengthAttr(24.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 100000.0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(560, 360, 640), Gf.Vec3d(0, 110, 0), Gf.Vec3d(0, 1, 0)).GetInverse())

    a = character(stage, "/World/WorkerA", WORKER_A)
    b = character(stage, "/World/WorkerB", WORKER_B)
    return parts, {"A": a, "B": b}
