"""Build the rail-kit assembly demo as a USD file (runs offline with usd-core, or inside Kit).

    python build_demo.py [output.usd]
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pxr import Usd, UsdSkel   # noqa: E402
from assembly import choreo, director, rig, scene   # noqa: E402

OUT = r"C:\OmniverseData\RailAssembly_ES2632.usd"


def main(out=OUT):
    """Builds into a temp file and swaps it in at the end: never rewrite a .usd that Kit has open (crate files are
    read lazily, so Kit would read a half-new file and crash). Close the stage in Kit first."""
    t0 = time.time()
    final, out = out, os.path.splitext(out)[0] + ".building" + os.path.splitext(out)[1]
    stage = Usd.Stage.CreateNew(out)
    parts, chars = scene.build(stage)
    rigs = {}
    for name, prim in chars.items():
        skel = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))
        rigs[name] = rig.CCRig(stage, skel)
    print(f"scene built in {time.time() - t0:.1f}s")
    end, n, _ = choreo.bake(stage, parts, rigs)
    director.write(stage, n)
    stage.GetRootLayer().Save()
    del stage
    try:
        os.replace(out, final)
    except OSError as e:
        print(f"could not replace {final} ({e}); is it open in Kit? New build left at {out}")
        return
    print(f"baked {n} frames ({end:.1f}s) in {time.time() - t0:.1f}s -> {final}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else OUT)
