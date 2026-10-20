"""Stage for the two-level shelving rack assembly (NVIDIA Industrial pack RackSmallEmpty_A2, see rack_asset.py): the
two end frames lying on the floor ready to be walked up, beams on trestles in front of / behind the bay, both decks
standing in a stillage, the warehouse, wrenches and the two NVIDIA characters. Reuses the ES2632 demo's helpers.

World: Y up, cm. Bay centred on the origin, long axis X (frames at x -+93.5), aisle (front) +Z.
"""
import math
import os

from pxr import Gf, UsdGeom

from . import hivis
from . import pbr
from . import scene as SC

RACK_ASSET = r"C:\OmniverseData\Rack\Rack_kit.usd"
KIT = "/World/Rack/Kit"
UP = Gf.Vec3d(0, 1, 0)

FRAME_OUT = 106.1                   # |x| of the base plates' outer edges: what the frames pivot on as they are walked up
UPRIGHT_X = 93.5                    # |x| of the upright centre lines
FRAME_IN = 87.2
UPRIGHT_Z = (28.0, 50.0)            # each upright spans |z| 28..50
FRAME_TOP = 301.0
BLOCK_D = 280.0                     # the frames' top ends rest on a timber block this far from the pivot ...
BLOCK_H = 34.0                      # ... this high
THETA0 = math.degrees(math.asin(BLOCK_H / BLOCK_D))
BEAM_Z = 46.45                      # beam centre lines (+Z front)
BEAM_FACE = 51.3                    # |z| of the beams' outer faces (bolt heads)
BEAM_TOP = {"Low": 53.1, "Up": 150.7}
BOLT_X = 90.5                       # |x| of the end-connector bolt
BOLT_Y = {"Low": 42.0, "Up": 140.0}
DECK_TOP = {"Low": 50.6, "Up": 148.3}
DECK_EDGE = 46.0                    # |z| of the decks' long edges
DECK_DEPTH = 92.0
DECK_THICK = 4.6                    # wire decks: mesh top to the support channels' bottoms
TRESTLE_TOP = 40.0
ANCHOR_X, ANCHOR_Z, PLATE_T = 103.1, 39.0, 0.8           # one anchor in each base plate's outer tab
BEAM_STORE = {"BeamLowF": (-130.0, 250.0), "BeamUpF": (130.0, 250.0),
              "BeamLowR": (-130.0, -250.0), "BeamUpR": (130.0, -250.0)}
DECK_STORE = {"DeckLow": (0.0, 356.0), "DeckUp": (0.0, 340.0)}     # hinge-edge (x, z); standing on edge, out front
DECK_STORE_Y = 106.0                # their top edges (they stand on a 14 cm pallet)


def frame_m(side, theta):
    """FrameL / FrameR turned up to `theta` degrees about its outer foot edge (90 = standing = installed)."""
    s = -1.0 if side == "L" else 1.0
    piv = Gf.Vec3d(s * FRAME_OUT, 0.0, 0.0)
    phi = -s * (90.0 - theta)                      # L: +90 lays it down toward -X, R: -90 toward +X
    return Gf.Matrix4d().SetTranslate(-piv) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), phi)) * \
        Gf.Matrix4d().SetTranslate(piv)


def upright_point(side, d, z):
    """Frame-local point d up the outer flange of the upright at z (installed coordinates)."""
    s = -1.0 if side == "L" else 1.0
    return Gf.Vec3d(s * UPRIGHT_X, d, z)


def deck_pivot(level):
    return Gf.Vec3d(0.0, DECK_TOP[level], DECK_EDGE)


def deck_m(level, hinge, alpha):
    """Deck held by its front edge at `hinge` (world), turned `alpha` degrees from hanging (0) to level (90)."""
    D = deck_pivot(level)
    return Gf.Matrix4d().SetTranslate(-D) * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), -(90.0 - alpha))) * \
        Gf.Matrix4d().SetTranslate(Gf.Vec3d(hinge))


def beam_pivot(name):
    level = "Low" if "Low" in name else "Up"
    zs = 1.0 if name.endswith("F") else -1.0
    return Gf.Vec3d(0.0, BEAM_TOP[level], zs * BEAM_Z)


def beam_start(name):
    level = "Low" if "Low" in name else "Up"
    x, z = BEAM_STORE[name]
    height = BEAM_TOP[level] - (16.0 if level == "Low" else 113.6)
    return Gf.Vec3d(x, TRESTLE_TOP + height, z), 0.0


def beam_bolt(name, xs):
    level = "Low" if "Low" in name else "Up"
    zs = 1.0 if name.endswith("F") else -1.0
    return Gf.Vec3d(xs * BOLT_X, BOLT_Y[level], zs * BEAM_FACE), Gf.Vec3d(0, 0, zs)


def make_parts():
    parts = {}
    for side in "LR":
        holds = {}
        for who, z in (("A", UPRIGHT_Z[1] - 5.0), ("B", -(UPRIGHT_Z[1] - 5.0))):
            holds[who] = z                                         # each lifter walks one upright up
        parts["Frame" + side] = SC.Part("Frame" + side, Gf.Vec3d(0, 0, 0), holds, (Gf.Vec3d(0, 0, 0), 0.0),
                                        path=f"{KIT}/Frame{side}")
    for name in BEAM_STORE:
        pv = beam_pivot(name)
        holds = {k: SC.tube_grip(pv + Gf.Vec3d(xs * 30.0, -2.5, 0.0), (1, 0, 0), (-xs, 0, 0), 3.0)
                 for k, xs in (("a", -1.0), ("b", 1.0))}
        holds["brace"] = {xs: pv + Gf.Vec3d(xs * 62.0, 0.4, 0.0) for xs in (-1.0, 1.0)}
        parts[name] = SC.Part(name, pv, holds, beam_start(name), path=f"{KIT}/{name}")
    for level in ("Low", "Up"):
        D = deck_pivot(level)
        holds = {}
        for k, x in (("A1", -64.0), ("A2", -22.0), ("B1", 22.0), ("B2", 64.0)):     # both carry it by its front edge
            holds["front_" + k] = SC.tube_grip(D + Gf.Vec3d(x, -1.2, -0.6), (1, 0, 0), (1, 0, 0), 1.4)
        x, z = DECK_STORE["Deck" + level]
        parts["Deck" + level] = SC.Part("Deck" + level, D, holds, (Gf.Vec3d(x, DECK_STORE_Y, z), 0.0),
                                        path=f"{KIT}/Deck{level}")
    return parts


def anchors():
    """(frame, anchor point on the plate top) for the four base plates."""
    return [("Frame" + side, Gf.Vec3d(s * ANCHOR_X, PLATE_T, zs * ANCHOR_Z))
            for side, s in (("L", -1.0), ("R", 1.0)) for zs in (1.0, -1.0)]


# hammer drill, local frame: bit tip on the origin, +Y up the tool, +X to the operator's right, +Z away from him
DRILL_TOP = Gf.Vec3d(0.0, 45.0, -2.0)          # rear D-handle grip (right hand)
DRILL_SIDE = Gf.Vec3d(-13.0, 25.0, 0.0)        # side handle (left hand)
DRILL_EXT = 0.0                                # stand-up drilling extension between bit and drill (cm)
DRILL_STANCE = 38.0                            # worker's pelvis from the anchor
METHOD = "current"


def configure(method="current"):
    """The work method the scene and choreography are built for:
    current   beams on 40 cm trestles, frame tops on a 34 cm block, anchors drilled crouching
    improved  beams on 80 cm trestles (waist height), frame tops on an 80 cm sawhorse, a stand-up drilling
              extension (72 cm, handles at waist height) so the anchors are drilled standing"""
    global TRESTLE_TOP, BLOCK_H, THETA0, DRILL_TOP, DRILL_SIDE, DRILL_EXT, DRILL_STANCE, METHOD
    METHOD = method
    imp = method == "improved"
    TRESTLE_TOP = 80.0 if imp else 40.0
    BLOCK_H = 80.0 if imp else 34.0
    THETA0 = math.degrees(math.asin(BLOCK_H / BLOCK_D))
    DRILL_EXT = 72.0 if imp else 0.0
    DRILL_TOP = Gf.Vec3d(0.0, 45.0 + DRILL_EXT, -2.0)
    DRILL_SIDE = Gf.Vec3d(-13.0, 25.0 + DRILL_EXT, 0.0)
    DRILL_STANCE = 23.0 if imp else 38.0


def drill(stage, path, mats):
    from .rack_asset import _box, _rod, mesh_from
    UsdGeom.Xform.Define(stage, path)
    e = DRILL_EXT
    mesh_from(stage, f"{path}/Bit", [_rod((0, 0, 0), (0, 17, 0), 0.55, 8)] +
              ([_rod((0, 17, 0), (0, 17 + e, 0), 1.3, 8), _rod((0, 10 + e, 0), (0, 17 + e, 0), 2.4, 10)] if e else []),
              mats["steel"], (0.6, 0.6, 0.62))
    mesh_from(stage, f"{path}/Body", [_rod((0, 17 + e, 0), (0, 22 + e, 0), 2.2, 10), _box((-4, 22 + e, -4), (4, 40 + e, 5)),
                                      _box((-3.5, 30 + e, 5), (3.5, 38 + e, 12))], mats["yellow"], (0.95, 0.75, 0.05))
    mesh_from(stage, f"{path}/Grips", [_box((-1.6, 40 + e, -6), (1.6, 48 + e, 2)), _rod((-7, 45 + e, -2), (7, 45 + e, -2), 1.5, 8),
                                       _rod((-4, 25 + e, 0), (-18, 25 + e, 0), 1.6, 8)], mats["black"], (0.05, 0.05, 0.05))
    UsdGeom.Imageable(stage.GetPrimAtPath(path)).CreateVisibilityAttr("invisible")
    UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTransformOp()


def start_matrix(name, p):
    if name.startswith("Frame"):
        return frame_m(name[-1], THETA0)
    if name.startswith("Deck"):
        return deck_m(name[4:], p.start[0], 0.0)
    return p.at(*p.start)


def build(stage):
    UsdGeom.SetStageUpAxis(stage, "Y")
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.SetTimeCodesPerSecond(30)
    stage.SetFramesPerSecond(30)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    UsdGeom.Scope.Define(stage, "/World/Looks")
    mats = {}
    for k, (rgb, look) in {"black": ((0.04, 0.04, 0.045), "black_powder"), "grey": ((0.45, 0.46, 0.48), "bright_steel"),
                           "steel": ((0.62, 0.63, 0.66), "bright_steel"), "yellow": ((0.95, 0.75, 0.05), "yellow_tape"),
                           "zinc": ((0.7, 0.71, 0.72), "zinc_bolt"), "wood": ((0.62, 0.47, 0.3), "timber")}.items():
        mats[k] = pbr.material(stage, f"/World/Looks/{k.capitalize()}", look)
        SC._COLOURS[str(mats[k].GetPath())] = rgb
    if os.path.exists(SC.ENV_ASSET):
        stage.DefinePrim("/World/Environment", "Xform").GetReferences().AddReference(SC.ENV_ASSET)
        stage.OverridePrim("/World/Environment/Lifts").SetActive(False)     # no branded machines in this one
    else:
        SC.box(stage, "/World/Floor", (-1500, -1, -1500), (1500, 0, 1500), mats["grey"])
    # floor marking around the bay's footprint
    for k, (lo, hi) in enumerate((((-112, 0.05, -66), (112, 0.25, -62)), ((-112, 0.05, 62), (112, 0.25, 66)),
                                  ((-116, 0.05, -66), (-112, 0.25, 66)), ((112, 0.05, -66), (116, 0.25, 66)))):
        SC.box(stage, f"/World/Marking{k}", lo, hi, mats["yellow"])
    rack = stage.DefinePrim("/World/Rack", "Xform")
    rack.GetReferences().AddReference(RACK_ASSET)
    parts = make_parts()
    # timber blocks under the frames' top ends
    for side, s in (("L", -1.0), ("R", 1.0)):
        x = s * (FRAME_OUT + BLOCK_D * math.cos(math.radians(THETA0)))
        if BLOCK_H > 50:                                   # a sawhorse: top rail on two A legs
            SC.box(stage, f"/World/Stands/Block{side}", (x - 6, BLOCK_H - 9, -60), (x + 6, BLOCK_H - 0.3, 60), mats["wood"])
            for k, z in enumerate((-52.0, 52.0)):
                SC.box(stage, f"/World/Stands/Block{side}Leg{k}", (x - 18, 0, z - 4), (x + 18, BLOCK_H - 9, z + 4), mats["black"])
        else:
            SC.box(stage, f"/World/Stands/Block{side}", (x - 7, 0, -60), (x + 7, BLOCK_H - 0.3, 60), mats["wood"])
    # trestle pairs under the beams
    for name, (x, z) in BEAM_STORE.items():
        for k, dx in enumerate((-60.0, 60.0)):
            SC.box(stage, f"/World/Stands/{name}{k}", (x + dx - 6, 0, z - 14), (x + dx + 6, TRESTLE_TOP, z + 14), mats["black"])
    # deck stillage out front: pallet-height base and a back frame the decks stand against
    xs = DECK_STORE["DeckLow"][0]
    zs = [z for _, z in DECK_STORE.values()]
    SC.box(stage, "/World/Stands/DeckBase", (xs - 95, 0, min(zs) - 6), (xs + 95, DECK_STORE_Y - DECK_DEPTH, max(zs) + 12),
           mats["wood"])
    for k, dx in enumerate((-85.0, 85.0)):
        SC.box(stage, f"/World/Stands/DeckPost{k}", (xs + dx - 3, 0, min(zs) - 6), (xs + dx + 3, 95, min(zs) - 2), mats["black"])
    for p_name, p in parts.items():
        UsdGeom.Xformable(stage.GetPrimAtPath(p.path)).GetOrderedXformOps()[0].Set(start_matrix(p_name, p))
    UsdGeom.Xform.Define(stage, "/World/Tools")
    for n in ("A", "B"):
        SC.wrench(stage, f"/World/Tools/Wrench{n}", mats)
        drill(stage, f"/World/Tools/Drill{n}", mats)
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Overview")
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 100000.0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(520, 300, 620), Gf.Vec3d(0, 110, 0), Gf.Vec3d(0, 1, 0)).GetInverse())
    a = SC.character(stage, "/World/WorkerA", SC.WORKER_A)
    b = SC.character(stage, "/World/WorkerB", SC.WORKER_B)
    hivis.apply(stage, "/World/WorkerA", "uniform_m_0001")       # hi-vis vests (hivis.py builds the textures)
    hivis.apply(stage, "/World/WorkerB", "uniform_f_0001")
    return parts, {"A": a, "B": b}
