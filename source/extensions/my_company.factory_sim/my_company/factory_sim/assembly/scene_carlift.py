"""Stage for the in-ground vehicle lift assembly (NVIDIA factory sample CAD, see carlift_asset.py): the lift with its
column set in the floor, the head frame and two runways waiting on stands, the warehouse, wrenches and the two
NVIDIA characters. Reuses the ES2632 demo's helpers (materials, boxes, bolts, characters, Part).

World: Y up, cm. Column on the origin, long axis X; runway tops at RUNWAY_TOP once installed.
"""
import os

from pxr import Gf, UsdGeom

from . import pbr
from . import scene as SC

LIFT_ASSET = r"C:\OmniverseData\CarLift\CarLift_kit.usd"
KIT = "/World/CarLift/Kit"

COLUMN_TOP = 82.7
HEAD_PIVOT = Gf.Vec3d(0.0, 82.7, 0.0)
HEAD_END_X = 175.0                       # middle of the head's end beams (they run along Z)
HEAD_START = (Gf.Vec3d(0.0, 50.0, -250.0), 0.0)
RUNWAY_TOP = 110.7
RUNWAY_Z = 84.95                          # runway centre lines (+Z = L)
RAIL_OUT = 103.4                          # outer rail centre (|z|)
RUNWAY_START = {"L": (Gf.Vec3d(0.0, 50.0, 360.0), 0.0), "R": (Gf.Vec3d(0.0, 50.0, -380.0), 0.0)}
STAND_Y = 50.0
GRIP_X = 165.0                            # where the carriers hold a runway's outer rail
BOLT_X = (-170.0, -120.0, 120.0, 170.0)  # vertical bolts through each runway's outer rail into the head


def make_parts():
    parts = {}
    holds = {}
    for who, xs in (("A", -1.0), ("B", 1.0)):
        c = Gf.Vec3d(xs * HEAD_END_X, 99.0, 0.0)
        holds[who] = {"a": SC.tube_grip(c + Gf.Vec3d(0, 0, 18), (0, 0, 1), (0, 0, -1), 4.0),
                      "b": SC.tube_grip(c + Gf.Vec3d(0, 0, -18), (0, 0, 1), (0, 0, 1), 4.0)}
    parts["Head"] = SC.Part("Head", HEAD_PIVOT, holds, HEAD_START, path=f"{KIT}/Head")
    for side, zs in (("L", 1.0), ("R", -1.0)):
        pv = Gf.Vec3d(0.0, 103.0, zs * RUNWAY_Z)
        h = {}
        for who, xs in (("A", -1.0), ("B", 1.0)):
            c = Gf.Vec3d(xs * GRIP_X, 107.0, zs * RAIL_OUT)
            h[who] = {"a": SC.tube_grip(c + Gf.Vec3d(-12, 0, 0), (1, 0, 0), (1, 0, 0), 2.6),
                      "b": SC.tube_grip(c + Gf.Vec3d(12, 0, 0), (1, 0, 0), (-1, 0, 0), 2.6),
                      "brace": Gf.Vec3d(xs * 140.0, RUNWAY_TOP + 0.5, zs * RAIL_OUT)}
        parts["Runway" + side] = SC.Part("Runway" + side, pv, h, RUNWAY_START[side], path=f"{KIT}/Runway{side}")
    return parts


def runway_bolt(side, x):
    zs = 1.0 if side == "L" else -1.0
    return Gf.Vec3d(x, RUNWAY_TOP, zs * RAIL_OUT), Gf.Vec3d(0, 1, 0)


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
                           "zinc": ((0.7, 0.71, 0.72), "zinc_bolt")}.items():
        mats[k] = pbr.material(stage, f"/World/Looks/{k.capitalize()}", look)
        SC._COLOURS[str(mats[k].GetPath())] = rgb
    if os.path.exists(SC.ENV_ASSET):
        stage.DefinePrim("/World/Environment", "Xform").GetReferences().AddReference(SC.ENV_ASSET)
        stage.OverridePrim("/World/Environment/Lifts").SetActive(False)     # no branded machines in this one
    else:
        SC.box(stage, "/World/Floor", (-1500, -1, -1500), (1500, 0, 1500), mats["grey"])
    # bay markings and the pit cover ring around the column
    for k, (lo, hi) in enumerate((((-300, 0.05, -175), (300, 0.25, -171)), ((-300, 0.05, 171), (300, 0.25, 175)),
                                  ((-304, 0.05, -175), (-300, 0.25, 175)), ((300, 0.05, -175), (304, 0.25, 175)))):
        SC.box(stage, f"/World/Marking{k}", lo, hi, mats["yellow"])
    SC.tube(stage, "/World/PitCover", (0, 0.0, 0), (0, 0.6, 0), 38.0, mats["black"])
    lift = stage.DefinePrim("/World/CarLift", "Xform")
    lift.GetReferences().AddReference(LIFT_ASSET)
    parts = make_parts()
    # stands: under the head's runway-support beams (inboard of its ends) and under each runway (inboard)
    hp, _ = HEAD_START
    for k, (x, z) in enumerate(((-60, -83), (60, -83), (-60, 83), (60, 83))):
        SC.box(stage, f"/World/Stands/Head{k}", (x - 12, 0, hp[2] + z - 12), (x + 12, STAND_Y, hp[2] + z + 12), mats["black"])
    for side in "LR":
        rp, _ = RUNWAY_START[side]
        for k, x in enumerate((-110, 110)):
            SC.box(stage, f"/World/Stands/Runway{side}{k}", (x - 12, 0, rp[2] - 25), (x + 12, STAND_Y, rp[2] + 25), mats["black"])
    for p in parts.values():
        UsdGeom.Xformable(stage.GetPrimAtPath(p.path)).GetOrderedXformOps()[0].Set(p.at(*p.start))
    UsdGeom.Xform.Define(stage, "/World/Tools")
    for n in ("A", "B"):
        SC.wrench(stage, f"/World/Tools/Wrench{n}", mats)
    cam = UsdGeom.Camera.Define(stage, "/World/Cameras/Overview")
    cam.CreateFocalLengthAttr(22.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 100000.0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(620, 330, 520), Gf.Vec3d(0, 90, 0), Gf.Vec3d(0, 1, 0)).GetInverse())
    a = SC.character(stage, "/World/WorkerA", SC.WORKER_A)
    b = SC.character(stage, "/World/WorkerB", SC.WORKER_B)
    return parts, {"A": a, "B": b}
