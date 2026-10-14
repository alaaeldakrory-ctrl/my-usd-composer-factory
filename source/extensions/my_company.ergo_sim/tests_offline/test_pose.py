"""Offline tests for pose.py (angles) and skeleton.py (reader). Run:  python test_pose.py   (needs usd-core)
Pass a converted Mixamo .usd as the first argument to also test on your own file."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "my_company", "ergo_sim", "core")))

import pose as P          # noqa: E402
import rula as R          # noqa: E402

checks = 0


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


def near(a, b, tol=0.6):
    return abs(a - b) <= tol


def body(trunk_lean_deg=0.0, neck_extra_deg=0.0, arm_vec=(0.0, -0.3, 0.0), forearm_vec=(0.0, -0.25, 0.0), hand_vec=(0.0, -0.08, 0.0),
         turn_shoulders_deg=0.0):
    """A person facing +Z (Y up). Right shoulder is at -X. Trunk can lean forward, arm vectors are given in the world."""
    lean = math.radians(trunk_lean_deg)
    fwd_lean = (0.0, math.cos(lean), math.sin(lean))                 # trunk direction (forward lean = +Z)
    hips = (0.0, 1.0, 0.0)
    neck = tuple(hips[i] + 0.5 * fwd_lean[i] for i in range(3))
    ang = math.radians(trunk_lean_deg + neck_extra_deg)
    head = (neck[0], neck[1] + 0.2 * math.cos(ang), neck[2] + 0.2 * math.sin(ang))
    t = math.radians(turn_shoulders_deg)
    half = 0.2
    r_arm = (neck[0] - half * math.cos(t), neck[1] - 0.05, neck[2] + half * math.sin(t))
    l_arm = (neck[0] + half * math.cos(t), neck[1] - 0.05, neck[2] - half * math.sin(t))
    forearm = tuple(r_arm[i] + arm_vec[i] for i in range(3))
    hand = tuple(forearm[i] + forearm_vec[i] for i in range(3))
    mid = tuple(hand[i] + hand_vec[i] for i in range(3))
    return {"hips": hips, "neck": neck, "head": head, "l_up_leg": (0.1, 0.95, 0.0), "r_up_leg": (-0.1, 0.95, 0.0),
            "l_arm": l_arm, "r_arm": r_arm, "r_forearm": forearm, "r_hand": hand, "r_hand_mid": mid,
            "l_forearm": forearm, "l_hand": hand, "l_hand_mid": mid}


# ---------------------------------------------------------------- synthetic bodies with known answers
a = P.pose_angles(body(), "r")
check(near(a.trunk_deg, 0) and near(a.neck_deg, 0) and near(a.upper_arm_deg, 0), f"standing straight: {a}")
check(near(a.elbow_deg, 180) and near(a.wrist_deg, 0), "straight arm: elbow 180, wrist 0")
check(not (a.trunk_twisted or a.trunk_side_bent or a.upper_arm_abducted), "no flags when standing")

for lean in (15, 30, 45, 80):
    a = P.pose_angles(body(trunk_lean_deg=lean), "r")
    check(near(a.trunk_deg, lean), f"trunk leaning {lean}: got {a.trunk_deg:.1f}")
a = P.pose_angles(body(trunk_lean_deg=-20), "r")
check(near(a.trunk_deg, -20), "leaning backward is negative")

a = P.pose_angles(body(trunk_lean_deg=30, neck_extra_deg=25), "r")
check(near(a.trunk_deg, 30) and near(a.neck_deg, 25), f"head bent 25 more than the trunk: {a}")

fwd = P.pose_angles(body(arm_vec=(0.0, 0.0, 0.3)), "r")               # arm straight forward
check(near(fwd.upper_arm_deg, 90), f"arm forward = 90: {fwd.upper_arm_deg}")
up = P.pose_angles(body(arm_vec=(0.0, 0.3, 0.0)), "r")
check(near(up.upper_arm_deg, 180), f"arm overhead = 180: {up.upper_arm_deg}")
back = P.pose_angles(body(arm_vec=(0.0, -0.2, -0.2)), "r")
check(near(back.upper_arm_deg, -45), f"arm 45 backward = -45: {back.upper_arm_deg}")
raised_back = P.pose_angles(body(arm_vec=(-0.25, 0.1, -0.06)), "r")     # out to the side, a little behind the body
check(raised_back.upper_arm_deg > 0, f"a sideways raise slightly behind the body is not extension: {raised_back.upper_arm_deg}")
side = P.pose_angles(body(arm_vec=(-0.3, 0.0, 0.0)), "r")              # out to the side
check(near(abs(side.upper_arm_deg), 90) and side.upper_arm_abducted, f"arm out to the side: {side}")
check(not P.pose_angles(body(arm_vec=(0.0, -0.2, 0.2)), "r").upper_arm_abducted, "arm 45 forward is not abducted")
# leaning the trunk 30 deg and letting the arm hang straight down (world) means the arm is 30 deg forward of the trunk's line
hang = P.pose_angles(body(trunk_lean_deg=30), "r")
check(near(hang.upper_arm_deg, 30), f"arm hangs while leaning: {hang.upper_arm_deg}")

lean_hang = P.pose_angles(body(trunk_lean_deg=48), "r")                # arm hangs vertically while the trunk leans 48 forward
check(near(lean_hang.upper_arm_deg, 48), f"hanging arm + 48 deg lean is 48 forward of the trunk line: {lean_hang.upper_arm_deg}")
lean_back = P.pose_angles(body(trunk_lean_deg=48, arm_vec=(0.0, -0.2, -0.25)), "r")
check(lean_back.upper_arm_deg < 0, f"arm swung back relative to a leaning trunk is extension: {lean_back.upper_arm_deg}")
el = P.pose_angles(body(arm_vec=(0.0, -0.3, 0.0), forearm_vec=(0.0, 0.0, 0.25)), "r")   # forearm forward = right angle
check(near(el.elbow_deg, 90), f"right-angle elbow: {el.elbow_deg}")
wr = P.pose_angles(body(forearm_vec=(0.0, 0.0, 0.25), hand_vec=(0.0, 0.0707, 0.0707)), "r")
check(near(wr.wrist_deg, 45), f"wrist bent 45: {wr.wrist_deg}")
tw = P.pose_angles(body(turn_shoulders_deg=35), "r")
check(tw.trunk_twisted, "shoulders turned 35 degrees against the hips counts as twisted")
check(not P.pose_angles(body(turn_shoulders_deg=10), "r").trunk_twisted, "10 degrees is not")

# z-up frame gives the same answers
def swap(j):   # (x, y, z) Y-up  ->  (x, -z, y) Z-up, keeping handedness
    return {k: (v[0], -v[2], v[1]) for k, v in j.items()}
# trunk leaning 30 forward, arm held horizontally forward: 90 + 30 = 120 degrees from the trunk's downward line
y_case = P.pose_angles(body(trunk_lean_deg=30, arm_vec=(0.0, 0.0, 0.3)), "r")
z = P.pose_angles(swap(body(trunk_lean_deg=30, arm_vec=(0.0, 0.0, 0.3))), "r", up=(0.0, 0.0, 1.0))
check(near(y_case.trunk_deg, 30) and near(y_case.upper_arm_deg, 120), f"y-up frame: {y_case}")
check(near(z.trunk_deg, y_case.trunk_deg, 1e-6) and near(z.upper_arm_deg, y_case.upper_arm_deg, 1e-6)
      and near(z.elbow_deg, y_case.elbow_deg, 1e-6), f"z-up frame should match y-up: {z}")

# end to end: a person bending forward 50 degrees to lift a 12 kg box, arms reaching down and forward
j = body(trunk_lean_deg=50, arm_vec=(0.0, -0.25, 0.25), forearm_vec=(0.0, -0.2, 0.2))
a = P.pose_angles(j, "r")
res = R.rula_from_angles(a.upper_arm_deg, a.elbow_deg, a.wrist_deg, a.neck_deg, a.trunk_deg, mass_kg=12, is_static=True,
                         trunk_twisted=a.trunk_twisted, trunk_side_bent=a.trunk_side_bent, upper_arm_abducted=a.upper_arm_abducted)
light = R.rula_from_angles(a.upper_arm_deg, a.elbow_deg, a.wrist_deg, a.neck_deg, a.trunk_deg, mass_kg=0)
check(res.force == 3 and res.grand_score >= light.grand_score and res.trunk == 3, f"lift example {res}")

# ---------------------------------------------------------------- the reader on a real animated skeleton
path = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\alaae\Downloads\Climbing To Top.usd"
if os.path.exists(path):
    os.environ["PXR_USDC_EMIT_DEPRECATION_WARNINGS"] = "0"
    from pxr import Usd, UsdGeom, UsdSkel, Sdf  # noqa: E402
    import skeleton as S  # noqa: E402

    stage = Usd.Stage.Open(path)
    reader = S.SkeletonReader(stage)
    t0, t1 = reader.time_range()
    print(f"{os.path.basename(path)}: {len(reader.joints)} joints, animation '{reader.animation.GetName()}', "
          f"keys {t0}..{t1} timecodes ({(t1 - t0) / reader.timecodes_per_second:.2f} s), up={reader.up_axis}")
    check(reader.animation.GetName() != "Take_001" or len(UsdSkel.Animation(reader.animation).GetRotationsAttr().GetTimeSamples()) > 0,
          "picked an animation with keyframes")
    need = {"hips", "neck", "head", "l_up_leg", "r_up_leg", "l_arm", "r_arm", "r_forearm", "r_hand", "r_hand_mid", "l_forearm", "l_hand", "l_hand_mid"}
    check(need <= set(reader._by_key), f"missing joints: {need - set(reader._by_key)}")

    # compare with USD's own evaluation, after pointing the skeleton at the animation that has the keys
    binding = UsdSkel.BindingAPI(reader.skeleton)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        binding.GetAnimationSourceRel().SetTargets([reader.animation.GetPath()])
    cache = UsdSkel.Cache()
    cache.Populate(UsdSkel.Root(reader.root), Usd.PrimDefaultPredicate)
    query = cache.GetSkelQuery(UsdSkel.Skeleton(reader.skeleton))
    for t in (t0, (t0 + t1) / 2, t1, t0 + 17.3):
        mine = reader.joint_skel_transforms(t)
        ref = query.ComputeJointSkelTransforms(Usd.TimeCode(t))
        worst = max(abs(mine[i][r][c] - ref[i][r][c]) for i in range(len(mine)) for r in range(4) for c in range(4))
        check(worst < 1e-3, f"joint transforms differ from USD's own at t={t}: {worst}")
    print("joint transforms match USD's own skeleton evaluation")

    # body angles through the clip
    print("   time(s)  trunk  neck  upper-arm(R)  elbow(R)  wrist(R)   hand height(m)")
    rows = []
    for k in range(9):
        t = t0 + (t1 - t0) * k / 8
        pos = reader.key_positions(t)
        pa = P.pose_angles(pos, "r", up=reader.up_vector())
        up_i = 1 if reader.up_axis == "Y" else 2
        rows.append((pa, pos["r_hand"][up_i]))
        print(f"   {(t - t0) / reader.timecodes_per_second:6.2f}  {pa.trunk_deg:6.1f} {pa.neck_deg:6.1f} {pa.upper_arm_deg:11.1f} {pa.elbow_deg:9.1f} {pa.wrist_deg:9.1f}   {pos['r_hand'][up_i]:8.2f}")
    check(all(math.isfinite(v) for pa, h in rows for v in (pa.trunk_deg, pa.neck_deg, pa.upper_arm_deg, pa.elbow_deg, pa.wrist_deg, h)), "angles are finite")
    heights = [h for _, h in rows]
    check(max(heights) - min(heights) > 0.5, "the hand moves through the clip")
    check(0.5 < reader.key_positions(t0)["head"][1 if reader.up_axis == "Y" else 2] < 2.2, "head height is a plausible number of meters")
else:
    print("(no converted Mixamo USD found; skipped the reader tests)")

print(f"ALL {checks} CHECKS PASSED")
