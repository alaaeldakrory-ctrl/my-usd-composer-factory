"""Offline tests for reach.py on a real Mixamo skeleton. Run:  python test_reach.py [converted Mixamo .usd]"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "my_company", "ergo_sim", "core")))
os.environ["PXR_USDC_EMIT_DEPRECATION_WARNINGS"] = "0"

from pxr import Gf, Usd, UsdSkel  # noqa: E402
import pose as P  # noqa: E402
import reach as RC  # noqa: E402
import skeleton as SK  # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\alaae\Dev\omniverse-learning\exercises\assets\maximo\Snatch.usd"
checks = 0


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


stage = Usd.Stage.Open(PATH)
skel_prim = next(p for p in stage.Traverse() if p.IsA(UsdSkel.Skeleton))
rig = RC.Rig(skel_prim)
solver = RC.ReachSolver(rig)
rest = rig.rest_world
print(f"{os.path.basename(PATH)}: {len(rig.joints)} joints; upper arm {solver.len['r'][0]:.1f} cm, forearm {solver.len['r'][1]:.1f} cm")

# ---------------------------------------------------------------- rig basics
check(abs(rig.pos(rest, "hips")[1] - 94.75) < 0.1, "hips height from the rest pose")
rest_angles = solver.offsets["r"]
a0 = P.pose_angles(rig.key_positions_m(rest), "r", offsets=solver.offsets["r"])
check(abs(a0.trunk_deg) < 1e-9 and abs(a0.neck_deg) < 1e-9 and a0.wrist_deg < 1e-9, "the rest pose reads as neutral after calibration")


def lengths(world):
    return {s: ((rig.pos(world, f"{s}_forearm") - rig.pos(world, f"{s}_arm")).GetLength(),
                (rig.pos(world, f"{s}_hand") - rig.pos(world, f"{s}_forearm")).GetLength()) for s in ("r", "l")}


# ---------------------------------------------------------------- lean alone
prev = -1e9
for alpha in (0, 15, 30, 45, 60):
    w = rig.fk(solver.lean_overrides(alpha))
    t = P.pose_angles(rig.key_positions_m(w), "r", offsets=solver.offsets["r"]).trunk_deg
    check(t > prev, "trunk angle grows with the spine bend")
    prev = t
    if alpha == 30:
        print(f"   spine bend 30 deg -> measured trunk flexion {t:.1f} deg")
    # feet and hips do not move
    check(((rig.pos(w, "l_up_leg") + rig.pos(w, "r_up_leg")) / 2 - (rig.pos(rest, "l_up_leg") + rig.pos(rest, "r_up_leg")) / 2).GetLength() < 0.05,
          "the hip joints stay where they were")
    check(all((Gf.Vec3d(w[i].ExtractTranslation()) - Gf.Vec3d(rest[i].ExtractTranslation())).GetLength() < 0.05
              for i, n in enumerate(rig.joints) if "Leg" in n or "Foot" in n or "Toe" in n), "legs and feet stay planted")

# ---------------------------------------------------------------- reaching
def solve_at(right, up, forward, look=None, **kw):
    s = RC.ReachSolver(rig, **kw) if kw else solver
    tgt = RC.to_skel_cm(right, up, forward)
    ov, info = s.solve({"r": tgt}, look=RC.to_skel_cm(*look) if look else None)
    return s, ov, info, tgt


s, ov, info, tgt = solve_at(0.15, 1.25, 0.25)                 # chest-high and close (about 34 cm from the shoulder): no lean needed
check(info["lean_ok"] and info["lean_deg"] == 0.0, f"close reach needs no lean: {info}")
check(info["hand_error_cm"]["r"] < 0.3, f"hand reaches the target: {info['hand_error_cm']}")
w = rig.fk(ov)
for side, (l1, l2) in lengths(w).items():
    check(abs(l1 - solver.len[side][0]) < 1e-3 and abs(l2 - solver.len[side][1]) < 1e-3, f"bone lengths preserved ({side})")

leans = []
for h in (1.10, 0.95, 0.85, 0.75, 0.65):
    s, ov, info, tgt = solve_at(0.0, h, 0.38)
    leans.append(info["lean_deg"])
    if h >= 0.85:
        check(info["lean_ok"] and info["hand_error_cm"]["r"] < 0.3, f"bench {h} m reachable: {info}")
    else:                                                     # very low work on this short-armed rig: reported as beyond comfortable reach
        check(info["lean_deg"] >= leans[-2] if len(leans) > 1 else True, "lower work never needs less lean")
print("   lean needed for work at 1.10 / 0.95 / 0.85 / 0.75 / 0.65 m:", [f"{v:.1f}" for v in leans])
check(all(leans[i] <= leans[i + 1] for i in range(4)), "lower work needs at least as much lean")
check(leans[0] < leans[-1], "and strictly more at the lowest height")

s, ov, info, tgt = solve_at(0.0, 0.9, 1.20)                    # 1.2 m in front: beyond reach
check((not info["lean_ok"]) and info["lean_deg"] == solver.lean_max and info["hand_error_cm"]["r"] > 5, f"out of reach is reported: {info}")

# both hands, and elbows stay in a sensible range
s = solver
ov, info = s.solve({"r": RC.to_skel_cm(0.12, 0.95, 0.40), "l": RC.to_skel_cm(-0.12, 0.95, 0.40)})
check(max(info["hand_error_cm"].values()) < 0.3, "both hands reach")
for side in ("r", "l"):
    a = s.angles(ov, side)
    check(30 < a.elbow_deg < 175, f"elbow angle sensible ({side}): {a.elbow_deg:.0f}")

# looking down at the work tips the neck forward; extension is limited
s, ov, info, tgt = solve_at(0.0, 0.85, 0.38, look=(0.0, 0.85, 0.38))
a = s.angles(ov, "r")
print(f"   bench 0.85 m: lean {info['lean_deg']:.1f}, trunk {a.trunk_deg:.1f}, neck {a.neck_deg:.1f}, upper arm {a.upper_arm_deg:.1f}, elbow {a.elbow_deg:.0f}")
check(-0.65 * 25 - 1 <= a.neck_deg <= 0.65 * 60 + 1, "neck stays within its limits")
s1, ov1, info1, _ = solve_at(0.0, 0.8, 0.25, look=(0.0, 0.8, 0.25), wrist_flex_deg=0.0)   # work close to the body, below the chest
a1 = s1.angles(ov1, "r")
print(f"   close low work: lean {info1['lean_deg']:.1f}, trunk {a1.trunk_deg:.1f}, neck {a1.neck_deg:.1f}")
s0, ov0, _, _ = solve_at(0.0, 1.0, 0.30, look=(0.0, 1.0, 0.30))
check(s0.angles(ov0, "r").neck_deg > 5, f"looking down at work below eye level tips the neck forward: {s0.angles(ov0, 'r').neck_deg:.1f}")
s2, ov2, info2, _ = solve_at(0.0, 1.5, 0.38, look=(0.0, 1.5, 0.38))
check(s2.angles(ov2, "r").neck_deg > -0.65 * 25 - 1, "neck extension is limited")

# wrist flex parameter
s3, ov3, _, _ = solve_at(0.1, 1.0, 0.35, wrist_flex_deg=25.0)
check(abs(s3.angles(ov3, "r").wrist_deg - 25.0) < 4.0, f"wrist flex ~25: {s3.angles(ov3, 'r').wrist_deg:.1f}")

# ---------------------------------------------------------------- write an animation, read it back with the reader
work = Usd.Stage.Open(PATH)
root = next(p for p in work.Traverse() if p.IsA(UsdSkel.Root))
skel2 = next(p for p in Usd.PrimRange(root) if p.IsA(UsdSkel.Skeleton))
rig2 = RC.Rig(skel2)
sol2 = RC.ReachSolver(rig2)
frames, expected = [], []
for k, h in enumerate((1.0, 0.9, 0.8, 0.9, 1.0)):
    ov, info = sol2.solve({"r": RC.to_skel_cm(0.1, h, 0.38)}, look=RC.to_skel_cm(0.1, h, 0.38))
    frames.append((float(k * 6), ov))
    expected.append(rig2.key_positions_m(rig2.fk(ov)))
anim_path = str(root.GetPath()) + "/GeneratedWork"
RC.write_animation(work, anim_path, rig2, frames)
rd = SK.SkeletonReader(work, animation_path=anim_path)
check(rd.time_range() == (0.0, 24.0), f"time range {rd.time_range()}")
worst = 0.0
for (tc, ov), exp in zip(frames, expected):
    got = rd.key_positions(tc)
    for key, v in exp.items():
        worst = max(worst, math.dist(v, got[key]))
check(worst < 0.005, f"animation written and read back: worst joint difference {worst * 100:.3f} cm")
a_ref = sol2.angles(frames[2][1], "r")
a_rd = P.pose_angles(rd.key_positions(frames[2][0]), "r", up=rd.up_vector(), offsets=sol2.offsets["r"])
check(abs(a_ref.trunk_deg - a_rd.trunk_deg) < 0.1 and abs(a_ref.upper_arm_deg - a_rd.upper_arm_deg) < 0.1, "angles from the written animation match the solver")
RC.bind_animation(rd.skeleton, anim_path)
check(str(rd.skeleton.GetRelationship("skel:animationSource").GetTargets()[0]) == anim_path, "skeleton bound to the new animation")

print(f"ALL {checks} CHECKS PASSED")
