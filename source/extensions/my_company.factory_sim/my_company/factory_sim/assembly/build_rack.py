"""Build the two-level shelving rack assembly demo (NVIDIA Industrial pack rack) as a USD file.

    python build_rack.py [output.usd]
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pxr import Usd, UsdSkel   # noqa: E402
from assembly import choreo, choreo_rack, director, rig, scene_rack   # noqa: E402

OUT = r"C:\OmniverseData\Rack_Assembly.usd"


def main(out=OUT):
    """Builds into a temp file and swaps it in (never rewrite a .usd that Kit has open)."""
    t0 = time.time()
    final, tmp = out, os.path.splitext(out)[0] + ".building" + os.path.splitext(out)[1]
    stage = Usd.Stage.CreateNew(tmp)
    parts, chars = scene_rack.build(stage)
    rigs = {n: rig.CCRig(stage, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    end, n, _ = choreo.bake(stage, parts, rigs, script_fn=choreo_rack.script, kit=scene_rack.KIT)
    director.write(stage, n, keys=choreo_rack.DIRECTOR_KEYS)
    stage.GetRootLayer().Save()
    del stage
    try:
        os.replace(tmp, final)
    except OSError as e:
        print(f"could not replace {final} ({e}); is it open in Kit? New build left at {tmp}")
        return
    print(f"baked {n} frames ({end:.1f}s) in {time.time() - t0:.1f}s -> {final}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else OUT)
