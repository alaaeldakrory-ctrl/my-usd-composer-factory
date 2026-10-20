"""RULA analysis of the baked rack demo (no Kit): per-step summary, CSV."""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
from pxr import Usd, UsdSkel   # noqa: E402
from assembly import choreo_rack as CR, ergo as E, rig as RG, scene_rack as SR   # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\OmniverseData\Rack_Assembly.usd"
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 2
t0 = time.time()
st = Usd.Stage.Open(PATH)
rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(st.GetPrimAtPath(f"/World/Worker{n}")) if p.IsA(UsdSkel.Skeleton)))
        for n in "AB"}
SR.configure("improved" if "Improved" in PATH else "current")
tr, wk, end = CR.script(SR.make_parts(), rigs)
sc = E.analyse(st, rigs, wk, CR.LOADS, int(st.GetEndTimeCode()) + 1, step=STEP)
print(f"analysed in {time.time() - t0:.0f}s")
for lab, who, peak, mean, secs, cause in E.summary(sc, CR.STEPS):
    print(f"{lab:38s} {who} peak {peak} mean {mean:.1f}  " + " ".join(f"{k[:6]}:{v:4.1f}s" for k, v in secs.items()) + f"  | {cause}")

if os.environ.get("ERGO_DEBUG"):
    import math as _m
    for who in "AB":
        for r in sc[who]:
            if abs(r["t"] - float(os.environ["ERGO_DEBUG"])) < 0.02:
                J = r["joints"]
                print(who, {k: (round(v, 1) if isinstance(v, float) else v) for k, v in r.items() if k != "joints"})
                for s in "LR":
                    el = 180 - E._ang(J[s + "_Upperarm"] - J[s + "_Forearm"], J[s + "_Hand"] - J[s + "_Forearm"])
                    wr = E._ang(J[s + "_Hand"] - J[s + "_Forearm"], J[s + "_Mid1"] - J[s + "_Hand"])
                    print("  ", s, "elbow", round(el), "wrist raw", round(wr))
                print("  rest", {k: round(v, 1) for k, v in E.neutral(rigs[who]).items()})
