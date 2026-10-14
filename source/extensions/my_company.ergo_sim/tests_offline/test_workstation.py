"""Offline tests for workstation.py. Run:  python test_workstation.py [converted Mixamo .usd]"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "my_company", "ergo_sim", "core")))
os.environ["PXR_USDC_EMIT_DEPRECATION_WARNINGS"] = "0"

from pxr import Usd, UsdSkel  # noqa: E402
import reach as RC  # noqa: E402
import skeleton as SK  # noqa: E402
import workstation as W  # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\alaae\Dev\omniverse-learning\exercises\assets\maximo\Snatch.usd"
checks = 0


def check(c, m):
    global checks
    checks += 1
    assert c, m


stage = Usd.Stage.Open(PATH)
root = next(p for p in stage.Traverse() if p.IsA(UsdSkel.Root))
skel = next(p for p in Usd.PrimRange(root) if p.IsA(UsdSkel.Skeleton))
rig = RC.Rig(skel)
solver = RC.ReachSolver(rig, wrist_flex_deg=10.0)

# targets
for kind in ("weld", "assembly"):
    tg = W.build_targets(W.TaskSpec(kind=kind, bench_height=0.95), fps=12)
    check(len(tg) == 8 * 12 + 1 and tg[0].t == 0 and abs(tg[-1].t - 8.0) < 1e-9, f"{kind}: frame count/time")
    check(all(t.right is not None for t in tg), f"{kind}: right hand always has a target")
weld = W.build_targets(W.TaskSpec(kind="weld"), 12)
check(all(t.is_static and t.mass_kg == 1.5 for t in weld), "torch is held static at 1.5 kg")
asm = W.build_targets(W.TaskSpec(kind="assembly"), 12)
check({t.phase for t in asm} >= {"reach", "grasp", "carry", "place", "release"}, "assembly goes through all phases")
check(any(t.mass_kg == 0.5 for t in asm) and any(t.mass_kg == 0 for t in asm), "part is carried only part of the time")
check(all(t.is_repeated for t in asm), "assembly is repetitive")

# analysis
t0 = time.time()
fr = W.analyse(solver, W.build_targets(W.TaskSpec(kind="weld", bench_height=1.0), 6))
dt = time.time() - t0
print(f"   analysed {len(fr)} frames in {dt:.1f} s")
sm = W.summarize(fr)
check(1 <= sm["max_score"] <= 7, "score in range")
check(sm["all_reachable"] and sm["worst_hand_error_cm"] < 0.3, f"weld at 1.0 m reachable: {sm}")
check(abs(sm["pct_1_2"] + sm["pct_3_4"] + sm["pct_5_6"] + sm["pct_7"] - 100) < 1e-9, "percentages add to 100")
check(all(f.rula.muscle_use == 1 for f in fr), "a held (static) torch counts as static muscle use")
check(all(f.rula.force == 0 for f in fr), "1.5 kg is below the 2 kg load threshold")
heavy = W.analyse(solver, W.build_targets(W.TaskSpec(kind="weld", bench_height=1.0, mass_kg=4.0), 6))
check(all(f.rula.force == 2 for f in heavy), "a 4 kg static load scores force 2")
check(W.summarize(heavy)["mean_score"] >= sm["mean_score"], "a heavier tool never scores better")

# sweep: lower bench -> more lean and no better score
rows = W.sweep_bench_height(solver, "weld", heights=(1.10, 1.00, 0.90, 0.80), fps=4)
for r in rows:
    print(f"   bench {r['bench_height']:.2f}: max RULA {r['max_score']}, mean {r['mean_score']:.2f}, max trunk {r['max_trunk_deg']:.0f}, "
          f"max upper arm {r['max_upper_arm_deg']:.0f}, reachable {r['all_reachable']}")
check(all(rows[i]["max_trunk_deg"] <= rows[i + 1]["max_trunk_deg"] + 1e-6 for i in range(3)), "lower bench -> trunk flexion never decreases")
check(all(rows[i]["mean_score"] <= rows[i + 1]["mean_score"] + 0.3 for i in range(3)), "lower bench -> score does not improve")
check(rows[0]["mean_score"] < rows[-1]["mean_score"], "the lowest bench scores clearly worse than the highest")

# csv
out = os.path.join(os.environ.get("TEMP", "."), "ergo_test")
os.makedirs(out, exist_ok=True)
W.write_summary_csv(rows, os.path.join(out, "sweep.csv"))
W.write_frames_csv(fr, os.path.join(out, "frames.csv"))
check(open(os.path.join(out, "sweep.csv")).read().count("\n") == 5, "summary csv has header + 4 rows")
check(open(os.path.join(out, "frames.csv")).read().count("\n") == len(fr) + 1, "frames csv has a row per frame")

# animation
work = Usd.Stage.Open(PATH)
root2 = next(p for p in work.Traverse() if p.IsA(UsdSkel.Root))
skel2 = next(p for p in Usd.PrimRange(root2) if p.IsA(UsdSkel.Skeleton))
rig2 = RC.Rig(skel2)
anim = str(root2.GetPath()) + "/ErgoWeld"
W.frames_to_animation(work, anim, rig2, fr)
rd = SK.SkeletonReader(work, animation_path=anim)
t_a, t_b = rd.time_range()
check(abs((t_b - t_a) / 24.0 - 8.0) < 1e-6, "animation lasts one cycle (8 s)")
print(f"ALL {checks} CHECKS PASSED")
