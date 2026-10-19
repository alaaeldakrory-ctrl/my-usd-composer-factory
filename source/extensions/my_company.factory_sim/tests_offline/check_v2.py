"""Quick checks on the v2 assembly: script timing, footsteps, foot sliding, hand errors on sampled frames."""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
from pxr import Gf, Usd, UsdGeom, UsdSkel   # noqa: E402
from assembly import choreo as C, gait as GT, rig as RG, scene as SC   # noqa: E402


def setup():
    st = Usd.Stage.CreateInMemory()
    parts, chars = SC.build(st)
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    return st, parts, rigs


if __name__ == "__main__":
    t0 = time.time()
    st, parts, rigs = setup()
    tracks, workers, end = C.script(parts, rigs)
    print(f"script: {end:.1f}s  ({time.time() - t0:.1f}s to build)")
    for n, W in workers.items():
        g = GT.Gait(rigs[n], W.root_at, end)
        print(n, "steps L/R", len(g.steps["L"]), len(g.steps["R"]), "root keys", len(W.root))
