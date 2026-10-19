"""Builds a small manufacturing warehouse around the assembly station from NVIDIA asset packs (Warehouse_NVD modular
building, Industrial_NVD racks / piles, Warehouse_NVD furniture, shipping and safety props).

    python warehouse_env.py [C:/OmniverseData/ES2632/Warehouse_Env.usd]

World: Y up, cm. The station (lift at the origin) sits in the middle of a 3 x 3 grid of 10 m modules (30 x 30 m hall,
10 m high). Module files are Z-up with their tile spanning x 0..1000, y -1000..0; "Straight" carries its wall on
the x=0 edge, "Straight90" on the y=0 edge, "CornerIn" on both. Pieces are rotated about the tile center to put
their walls on the hall's outside edges. Keep-out: the workers use |x| < 420, |z| < 390 around the lift.
"""
import math
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

NV = r"C:/Nvidia_Assets"
MOD = NV + "/Warehouse_NVD/Assets/DigitalTwin/Assets/Warehouse/Modular/Warehouse_A/"
WH = NV + "/Warehouse_NVD/Assets/DigitalTwin/Assets/Warehouse/"
IND = NV + "/Industrial_NVD/Assets/ArchVis/Industrial/"
HDRI = IND + "Stages/HDRI/CloudyDay_04_04.hdr"
LIFT = r"C:/OmniverseData/ES2632/ES2632_kit.usd"
OUT = r"C:/OmniverseData/ES2632/Warehouse_Env.usd"
TILE = 1000.0

ASSETS = {
    "rack_large": IND + "Racks/RackLarge_A1.usd", "rack_large2": IND + "Racks/RackLarge_A4.usd",
    "rack_long": IND + "Racks/RackLong_A3.usd", "rack_small": IND + "Racks/RackSmall_A2.usd",
    "pile_big": IND + "Piles/WarehousePile_A1.usd", "pile_small": IND + "Piles/WarehousePile_A4.usd",
    "pallets": IND + "Piles/Pallets_A2.usd", "fence": IND + "Railing/MetalFencing_A1.usd",
    "cabinet_tall": WH + "Furnishing/Cabinets/ClosedIndSteel_A/ClosedIndustrialSteelCabinet_A03_PR_NVD_01.usd",
    "cabinet_feet": WH + "Furnishing/Cabinets/SteelCompartmentWithFeet_A/SteelCompartmentCabinetWithFeet_A02_PR_NVD_01.usd",
    "parts_bin": WH + "Furnishing/Cabinets/WeldedParts_A/WeldedPartsCabinet_A01_PR_NVD_01.usd",
    "crate": WH + "Shipping/Wood_Crate_on_Pallet/HeavyDuty_A/HeavyDutyWoodCrateAssembly_A02_PR_NVD_01.usd",
    "pallet": WH + "Shipping/Pallets/Wood/Block_A/BlockPallet_A01_PR_NVD_01.usd",
    "conveyor": WH + "Equipment/Conveyors/ConveyorBelt_A/ConveyorBelt_A01_PR_NVD_01.usd",
    "ladder": WH + "Equipment/Ladders/AlumStep_A/AluminumStepLadder_A03_PR_NVD_01.usd",
    "handtruck": WH + "Equipment/Hand_Trucks/Convertible_Steel_A/ConvertibleSteelHandTruck_A01_PR_V_NVD_01.usd",
    "cone": WH + "Safety/Cones/Heavy-Duty_Traffic/HeavyDutyTrafficCone_A02_71cm_PR_V_NVD_01.usd",
    "sign": WH + "Safety/Floor_Signs/FoldingWarning_A/FoldingWarningSign_A01_PR_NVD_01.usd",
    "trolley": WH + "Facilities/Cleaning/Trolley_A/CleaningTrolley_A01_01.usd",
    "shelving": WH + "Shelving/Industrial/IndustrialSteel_A/IndustrialSteelShelving_A05_PR_NVD_01.usd",
}

# (asset, x, z, heading deg about +Y) — heading 0 keeps the asset's +X along world +X
PROPS = [
    # back (north, -Z) wall: pallet racking, a full bay
    ("rack_large", -1180, -1330, 90), ("rack_large", -760, -1330, 90), ("rack_large2", -340, -1330, 90),
    ("rack_large", 80, -1330, 90), ("rack_large2", 500, -1330, 90), ("rack_large", 920, -1330, 90),
    # west (-X) wall: goods-in, piles and pallets
    ("pile_big", -1260, -650, 0), ("pile_small", -1290, -300, 15), ("pallets", -1280, 60, 0),
    ("pallets", -1280, 290, 90), ("crate", -1050, 520, 10), ("crate", -1050, 680, -5), ("pile_small", -1290, 900, 0),
    ("conveyor", -1350, 1150, 90),
    # east (+X) wall: tool crib — cabinets and shelving
    ("cabinet_tall", 1390, -700, -90), ("cabinet_tall", 1390, -560, -90), ("cabinet_feet", 1405, -420, -90),
    ("cabinet_feet", 1405, -350, -90), ("shelving", 1400, -150, -90), ("shelving", 1400, 30, -90),
    ("rack_small", 1360, 400, 0), ("rack_small", 1360, 620, 0),
    ("ladder", 1430, 230, -90), ("trolley", 1250, 1250, 30),
    # near the station: parts on pallets beside the cart, hand truck, safety kit
    ("pallet", 560, -120, 0), ("parts_bin", 560, -130, 90), ("crate", 590, 170, 0),
    ("handtruck", 450, 420, -40), ("sign", 300, 520, 35),
    ("cone", -380, 470, 0), ("cone", 380, 470, 0), ("cone", -380, -470, 0), ("cone", 380, -470, 0),
    ("fence", -520, -700, 0), ("fence", -440, -700, 0), ("fence", -360, -700, 0),
]

# extra ES2632 units: a finished one waiting for shipping and the next one in the queue (no rails yet)
LIFTS = [("Finished_ES2632", (-760, 0, 820), 15.0, True), ("Next_ES2632", (820, 0, -720), -90.0, False)]


def _tile_layout():
    """(name, module file, i, j, turn deg) for the 3 x 3 grid; i: -1 west .. 1 east, j: -1 north .. 1 south."""
    want = {}
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            edges = set()
            if i == -1:
                edges.add("W")
            if i == 1:
                edges.add("E")
            if j == -1:
                edges.add("N")
            if j == 1:
                edges.add("S")
            want[(i, j)] = edges
    rows = []
    for (i, j), edges in sorted(want.items()):
        if not edges:
            rows.append((f"Tile_{i + 1}{j + 1}", MOD + "Warehouse_A01_H10m_Center_01.usd", i, j, 0.0))
        elif len(edges) == 1:
            e = next(iter(edges))
            if e in "WE":
                f = "Warehouse_A05_H10m_Straight_01.usd" if e == "W" else "Warehouse_A01_H10m_Straight_01.usd"
                rows.append((f"Tile_{i + 1}{j + 1}", MOD + f, i, j, 0.0 if e == "W" else 180.0))
            else:
                f = "Warehouse_A05_H10m_Straight90_01.usd" if e == "N" else "Warehouse_A01_H10m_Straight90_01.usd"
                rows.append((f"Tile_{i + 1}{j + 1}", MOD + f, i, j, 0.0 if e == "N" else 180.0))
        else:
            # CornerIn carries walls on its west + north edges; turn it so they face the two outside edges
            turn = {frozenset("WN"): 0.0, frozenset("NE"): -90.0, frozenset("ES"): 180.0, frozenset("SW"): 90.0}[frozenset(edges)]
            rows.append((f"Tile_{i + 1}{j + 1}", MOD + "Warehouse_A01_H10m_CornerIn_01.usd", i, j, turn))
    return rows


def _place(stage, path, asset, pos, heading, z_up=True):
    """Outer Xform carries our placement; the asset (with its own transforms) is referenced underneath."""
    prim = stage.DefinePrim(path, "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateYOp().Set(float(heading))
    inner = stage.DefinePrim(path + "/Asset", "Xform")
    inner.GetReferences().AddReference(asset)
    if z_up:
        UsdGeom.Xformable(stage.DefinePrim(path, "Xform")).AddRotateXOp(opSuffix="zup").Set(-90.0)
    return inner


def build(out=OUT):
    if os.path.exists(out):
        os.remove(out)
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, "Y")
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    env = UsdGeom.Xform.Define(stage, "/Env").GetPrim()
    stage.SetDefaultPrim(env)
    UsdGeom.Xform.Define(stage, "/Env/Building")
    for name, f, i, j, turn in _tile_layout():
        # tile footprint after Z-up -> Y-up is x 0..1000, z 0..1000; turn about its center, then move into the grid
        tile = stage.DefinePrim(f"/Env/Building/{name}", "Xform")
        xf = UsdGeom.Xformable(tile)
        cx, cz = TILE * i, TILE * j
        xf.AddTranslateOp().Set(Gf.Vec3d(cx, 0, cz))
        xf.AddRotateYOp().Set(turn)
        xf.AddTranslateOp(opSuffix="center").Set(Gf.Vec3d(-TILE / 2, 0, -TILE / 2))
        xf.AddRotateXOp(opSuffix="zup").Set(-90.0)
        stage.DefinePrim(f"/Env/Building/{name}/Module", "Xform").GetReferences().AddReference(f)
    UsdGeom.Xform.Define(stage, "/Env/Props")
    count = {}
    for asset, x, z, h in PROPS:
        k = count.get(asset, 0)
        count[asset] = k + 1
        _place(stage, f"/Env/Props/{asset}_{k:02d}", ASSETS[asset], (x, 0, z), h)
    UsdGeom.Xform.Define(stage, "/Env/Lifts")
    for name, pos, h, rails in LIFTS:
        p = _place(stage, f"/Env/Lifts/{name}", LIFT, pos, h, z_up=False)
        if not rails:
            for g in ("SideL", "SideR", "ExtFrame", "CrossBar", "Gate", "Controller"):
                stage.OverridePrim(p.GetPath().AppendPath(f"Kit/{g}")).SetActive(False)
    # daylight through the window walls; the modules bring their own high-bay and column lights
    sky = UsdLux.DomeLight.Define(stage, "/Env/Sky")
    sky.CreateTextureFileAttr().Set(Sdf.AssetPath(HDRI))
    sky.CreateIntensityAttr(1000.0)
    UsdGeom.Xformable(sky).AddRotateXOp().Set(-90.0)
    sun = UsdLux.DistantLight.Define(stage, "/Env/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(0.6)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.88))
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-38.0, -60.0, 0.0))
    stage.GetRootLayer().Save()
    return stage


if __name__ == "__main__":
    st = build(sys.argv[1] if len(sys.argv) > 1 else OUT)
    print("wrote", st.GetRootLayer().realPath)
