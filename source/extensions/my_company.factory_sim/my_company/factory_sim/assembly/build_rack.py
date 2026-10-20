"""Build the two-level shelving rack assembly demo (NVIDIA Industrial pack rack) as a USD file.

    python build_rack.py [output.usd]
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pxr import Usd, UsdSkel   # noqa: E402
from assembly import choreo, choreo_rack, director, ergo, ergo_report, ergo_viz, rig, scene_rack, sightline   # noqa: E402

OUT = r"C:\OmniverseData\Rack_Assembly.usd"
ERGO_OUT = r"C:\OmniverseData\Rack_Ergo"


def main(out=OUT, method="current"):
    """Builds into a temp file and swaps it in (never rewrite a .usd that Kit has open)."""
    t0 = time.time()
    final, tmp = out, os.path.splitext(out)[0] + ".building" + os.path.splitext(out)[1]
    scene_rack.configure(method)
    ergo_out = ERGO_OUT if method == "current" else ERGO_OUT + "_" + method.capitalize()
    stage = Usd.Stage.CreateNew(tmp)
    parts, chars = scene_rack.build(stage)
    rigs = {n: rig.CCRig(stage, next(p for p in Usd.PrimRange(prim) if p.IsA(UsdSkel.Skeleton))) for n, prim in chars.items()}
    end, n, _ = choreo.bake(stage, parts, rigs, script_fn=choreo_rack.script, kit=scene_rack.KIT)
    choreo_rack.write_tools(stage, n)
    # cameras: the close director with its views swung clear of the columns, and a wide one showing the warehouse
    _, workers, _ = choreo_rack.script(parts, rigs)          # same plan again: the workers' paths for the sightlines
    occ = sightline.occluders(stage)
    targets = choreo_rack.worker_targets(workers)
    close = sightline.repair(choreo_rack.DIRECTOR_KEYS, occ, director.at, end, targets, scale=1.0, lift=0.0,
                             min_gap=0.0, room=1350.0)
    wide, _ = choreo_rack.wide_keys(stage, workers, end)
    for name, keys in (("Director", close), ("DirectorWide", wide)):
        bad = sightline.report(keys, occ, director.at, end, targets_fn=targets)
        print(f"camera {name}: {len(keys)} keys, workers hidden {len(bad) * 0.25:.1f} s")
    director.write(stage, n, keys=close)
    director.write(stage, n, path="/World/Cameras/DirectorWide", keys=wide)
    # ergonomics: RULA per frame, markers on the bodies (/World/Ergo; deactivate it to hide them), CSV + report
    scores = ergo.analyse(stage, rigs, workers, choreo_rack.LOADS, n)
    ergo.write_markers(stage, scores, n)
    ergo_viz.write_skeleton(stage, scores)
    ergo_viz.write_load_arrows(stage, scores)
    heat = ergo_viz.write_floor_heat(stage, scores, ergo_out, reveal_t=choreo_rack.STEPS[-1][1] + 1.0)
    ergo_report.floor_png(heat, ((-600.0, 600.0), (-520.0, 520.0)), os.path.join(ergo_out, "floor_hotspots.png"),
                          outlines=[((-106, -52), (106, 52), "bay"), ((-95, 334), (95, 372), "decks"),
                                    ((-223, 236), (223, 264), "front beams"), ((-223, -264), (223, -236), "rear beams")],
                          labels=[(-300, 0, "frame L tip-up"), (300, 0, "frame R tip-up")])
    ergo_viz.add_view_variants(stage, ["/World/WorkerA", "/World/WorkerB"])
    os.makedirs(ergo_out, exist_ok=True)
    ergo.write_csv(os.path.join(ergo_out, "rula_per_frame.csv"), scores, choreo_rack.STEPS)
    ergo_report.timeline_png(scores, choreo_rack.STEPS, os.path.join(ergo_out, "rula_timeline.png"))
    ergo_report.summary_md(scores, choreo_rack.STEPS, os.path.join(ergo_out, "rula_by_step.md"))
    print(f"ergonomics -> {ergo_out}")
    stage.GetRootLayer().Save()
    del stage
    try:
        os.replace(tmp, final)
    except OSError as e:
        print(f"could not replace {final} ({e}); is it open in Kit? New build left at {tmp}")
        return
    xray = os.path.splitext(final)[0] + "_XRay.usda"
    ergo_viz.write_xray_layer(final, xray)
    print(f"baked {n} frames ({end:.1f}s) in {time.time() - t0:.1f}s -> {final}  (see-through bodies + bones: {xray})")


if __name__ == "__main__":
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    m = "improved" if "--improved" in sys.argv else "current"
    main(a[0] if a else (OUT if m == "current" else OUT.replace(".usd", "_Improved.usd")), m)
