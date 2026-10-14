"""Open the user's warehouse offline with online (S3) payloads swapped for local copies in the session layer,
so the forklift tool can be profiled on the real layout. Usage:  python open_user_scene.py  (prints a timed plan)"""
import os
import sys
import time

from pxr import Sdf, Usd

SCENE = r"C:\OmniverseData\India Demo Warehouse.usd"
SIMREADY = r"C:\Nvidia_Assets\SimReady_Warehouse_02\Assets\simready_content\common_assets\props"
LOCAL = {
    "Forklift_C01_PR_V_NVD_01.usd": r"C:\Nvidia_Assets\Warehouse_NVD\Assets\DigitalTwin\Assets\Warehouse\Equipment\Forklifts\Forklift_C\Forklift_C01_PR_V_NVD_01.usd",
    "RackSmallEmpty_A1.usd": r"C:\Nvidia_Assets\Industrial_NVD\Assets\ArchVis\Industrial\Shelves\RackSmallEmpty_A1.usd",
    "Pallet_Asm_A03_84x86x49cm_PR_V_NVD_01.usd": os.environ.get(
        "TEST_PALLET_USD", r"C:\Users\alaae\AppData\Local\Temp\claude\C--Users-alaae-OneDrive-Documents-AntiGravityProjects-Camping\017af5f4-d59d-4c55-bdd1-6bd4a2debe5d\scratchpad\pallet.usd"),
}


def local_for(url):
    name = url.replace("\\", "/").split("/")[-1]
    if name in LOCAL:
        return LOCAL[name]
    cand = os.path.join(SIMREADY, name[:-4], name)
    return cand if os.path.isfile(cand) else None


def open_scene():
    stage = Usd.Stage.Open(SCENE, Usd.Stage.LoadNone)
    session = stage.GetSessionLayer()
    with Usd.EditContext(stage, session):
        for prim in stage.GetDefaultPrim().GetAllChildren():
            for spec in prim.GetPrimStack():
                for pl in spec.payloadList.prependedItems:
                    url = str(pl.assetPath)
                    if url.startswith("http"):
                        loc = local_for(url)
                        if loc:
                            prim.GetPayloads().SetPayloads([Sdf.Payload(loc)])
    stage.Load()
    return stage


if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
    from forklift_ops import planner as P
    from forklift_ops import scene as S
    st = open_scene()
    t = time.time()
    sc = S.scan(st)
    print(f"scan {time.time() - t:.1f}s")
    print(S.summary(sc))
    for p in sc.pallets:
        print(" ", p.name, [round(v) for v in p.box.c], round(p.box.hu), round(p.box.hv), "base", round(p.base_y, 1),
              "h", round(p.height), "opening", round(p.opening_top, 1))
    import cProfile
    import pstats
    pr = cProfile.Profile()
    pr.enable()
    t = time.time()
    plan = P.Planner(sc, P.Settings()).plan()
    pr.disable()
    print(f"plan {time.time() - t:.1f}s; clip {plan.end_t:.1f}s; warnings {plan.warnings}")
    pstats.Stats(pr).sort_stats("cumulative").print_stats(12)
