"""Builds a small test warehouse the way Kit does when assets are dropped in (Y-up, cm, translate/rotateXYZ/scale
plus rotateX:unitsResolve for Z-up assets). Used by the offline tests; needs usd-core and the local NVIDIA assets."""
import os

from pxr import Gf, Usd, UsdGeom

FORKLIFT = r"C:\Nvidia_Assets\Warehouse_NVD\Assets\DigitalTwin\Assets\Warehouse\Equipment\Forklifts\Forklift_C\Forklift_C01_PR_V_NVD_01.usd"
FORKLIFT_B01 = r"C:\Nvidia_Assets\Warehouse_NVD\Assets\DigitalTwin\Assets\Warehouse\Equipment\Forklifts\Forklift_B\Forklift_B01_PR_V_NVD_01.usd"
RACK = r"C:\Nvidia_Assets\Industrial_NVD\Assets\ArchVis\Industrial\Shelves\RackSmallEmpty_A1.usd"
PALLET = os.environ.get("TEST_PALLET_USD", r"C:\Users\alaae\AppData\Local\Temp\claude\C--Users-alaae-OneDrive-Documents-AntiGravityProjects-Camping\017af5f4-d59d-4c55-bdd1-6bd4a2debe5d\scratchpad\pallet.usd")


def drop(stage, path, asset, pos, yaw=0.0, z_up=True):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(asset)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0, yaw, 0))
    xf.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
    if z_up:
        xf.AddRotateXOp(opSuffix="unitsResolve").Set(-90.0)
    return prim


def build(layout="two"):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, "Y")
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    stage.SetTimeCodesPerSecond(60)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    if layout == "today":           # the India Demo Warehouse layout
        drop(stage, "/World/Forklift_C01_PR_V_NVD_01", FORKLIFT, (1314.4895, 0, -6004.5322))
        drop(stage, "/World/Pallet_Asm_A03_84x86x49cm_PR_V_NVD_01", PALLET, (997.2614, 0, -5221.4014))
        drop(stage, "/World/RackSmallEmpty_A1", RACK, (668.4652, 0, -4862.2002))
        return stage
    if layout == "mixed":           # an order picker (B01) and a counterbalance truck (C01) sharing two racks
        drop(stage, "/World/Forklift_B01", FORKLIFT_B01, (1300, 0, -6000), yaw=-20)
        drop(stage, "/World/Forklift_C", FORKLIFT, (300, 0, -6000), yaw=90)
        drop(stage, "/World/RackSmallEmpty_A1", RACK, (700, 0, -4700))
        drop(stage, "/World/RackSmallEmpty_A2", RACK, (-200, 0, -4300), yaw=90)
        drop(stage, "/World/Pallet_01", PALLET, (1000, 0, -5200))
        drop(stage, "/World/Pallet_02", PALLET, (700, 0, -5500), yaw=30)
        drop(stage, "/World/Pallet_03", PALLET, (200, 0, -5300))
        return stage
    if layout == "retrieve":        # pallets already on the shelves, and a dock floor area to bring them to
        drop(stage, "/World/Forklift_A", FORKLIFT, (1300, 0, -6000))
        drop(stage, "/World/Forklift_B", FORKLIFT, (300, 0, -6000), yaw=90)
        drop(stage, "/World/RackSmallEmpty_A1", RACK, (700, 0, -4700))
        drop(stage, "/World/RackSmallEmpty_A2", RACK, (-200, 0, -4300), yaw=90)
        drop(stage, "/World/Pallet_R1", PALLET, (708, 53.1, -4748))
        drop(stage, "/World/Pallet_R2", PALLET, (708, 253.4, -4652))
        drop(stage, "/World/Pallet_R3", PALLET, (-152, 253.4, -4308), yaw=90)
        dock = UsdGeom.Mesh.Define(stage, "/World/Dock")
        x0, x1, z0, z1 = 1600, 2200, -5600, -5000
        dock.CreatePointsAttr([(x0, 0.2, z0), (x1, 0.2, z0), (x1, 0.2, z1), (x0, 0.2, z1)])
        dock.CreateFaceVertexCountsAttr([4])
        dock.CreateFaceVertexIndicesAttr([0, 3, 2, 1])
        return stage
    drop(stage, "/World/Forklift_A", FORKLIFT, (1300, 0, -6000))
    drop(stage, "/World/Forklift_B", FORKLIFT, (300, 0, -6000), yaw=90)
    drop(stage, "/World/RackSmallEmpty_A1", RACK, (700, 0, -4700))
    drop(stage, "/World/RackSmallEmpty_A2", RACK, (-200, 0, -4300), yaw=90)
    drop(stage, "/World/Pallet_01", PALLET, (1000, 0, -5200))
    drop(stage, "/World/Pallet_02", PALLET, (700, 0, -5500), yaw=30)
    drop(stage, "/World/Pallet_03", PALLET, (200, 0, -5300))
    drop(stage, "/World/Pallet_04", PALLET, (-300, 0, -5600), yaw=90)
    return stage


if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
    from forklift_ops import scene as S
    st = build(sys.argv[1] if len(sys.argv) > 1 else "two")
    sc = S.scan(st)
    print(S.summary(sc))
    for fl in sc.forklifts:
        print(fl.name, "heading", round(fl.heading, 1), "half_w", round(fl.half_width, 1), "rear", round(fl.rear, 1),
              "heel", round(fl.fork_heel, 1), "tip", round(fl.fork_tip, 1), "blade", round(fl.blade_top, 1),
              round(fl.blade_thick, 1), "wheelbase", round(fl.wheelbase, 1))
        if fl.steering:
            s = fl.steering
            print("  steering", [round(v, 2) for v in s.center], [round(v, 3) for v in s.axis], round(s.radius, 2),
                  "knob", s.knob and [round(v, 1) for v in s.knob])
        for w in fl.wheels:
            print("  wheel", w.path.split("/")[-1], round(w.radius, 1), "rear" if w.rear else "front", w.spin_idx,
                  w.spin_sign, w.steer_idx, w.steer_sign)
    for r in sc.racks:
        print(r.name, "box c", [round(v) for v in r.box.c], "u", [round(v, 2) for v in r.box.u], round(r.box.hu), round(r.box.hv),
              [(s.level, round(s.y), [round(v) for v in s.center], round(s.clearance)) for s in r.slots])
    for p in sc.pallets:
        print(p.name, [round(v) for v in p.box.c], round(p.box.hu), round(p.box.hv), "h", round(p.height), "opening", p.opening_top)
