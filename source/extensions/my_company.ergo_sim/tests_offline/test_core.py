"""Offline tests for the ergonomics core. Run with:  python test_core.py   (needs: pip install usd-core)
These run outside Omniverse; they import the core modules directly."""
import os
import sys

CORE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "my_company", "ergo_sim", "core")
sys.path.insert(0, os.path.normpath(CORE))

import rula as R          # noqa: E402
import mass as M          # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

checks = 0


def check(condition, message):
    global checks
    checks += 1
    assert condition, message


# ---------------------------------------------------------------- table structure
check(len(R.TABLE_A) == 6 and all(len(v) == 3 for v in R.TABLE_A.values()), "Table A should be 6 x 3 rows")
check(all(len(row) == 8 for ua in R.TABLE_A.values() for row in ua.values()), "Table A rows need 8 values")
check(len(R.TABLE_B) == 6 and all(len(v) == 12 for v in R.TABLE_B.values()), "Table B should be 6 rows of 12")
check(len(R.TABLE_C) == 8 and all(len(v) == 7 for v in R.TABLE_C.values()), "Table C should be 8 rows of 7")
cells = 6 * 3 * 8 + 6 * 12 + 8 * 7
check(cells == 272, "cell count")

# scores never go down as a posture gets worse (catches most single-cell typos)
for ua in range(1, 7):
    for la in range(1, 4):
        row = R.TABLE_A[ua][la]
        check(all(row[i] <= row[i + 1] for i in range(7)), f"Table A row {ua}/{la} not non-decreasing across wrist/twist")
        if la < 3:
            check(all(R.TABLE_A[ua][la][i] <= R.TABLE_A[ua][la + 1][i] for i in range(8)), f"Table A {ua}: lower arm {la}->{la + 1}")
    if ua < 6:
        for la in range(1, 4):
            check(all(R.TABLE_A[ua][la][i] <= R.TABLE_A[ua + 1][la][i] for i in range(8)), f"Table A upper arm {ua}->{ua + 1}")
for legs in (0, 1):
    for neck in range(1, 7):
        seq = [R.TABLE_B[neck][t * 2 + legs] for t in range(6)]
        check(all(seq[i] <= seq[i + 1] for i in range(5)), f"Table B neck {neck} legs {legs + 1} across trunk")
    for trunk in range(6):
        seq = [R.TABLE_B[n][trunk * 2 + legs] for n in range(1, 7)]
        check(all(seq[i] <= seq[i + 1] for i in range(5)), f"Table B trunk {trunk + 1} legs {legs + 1} down neck")
for r in range(1, 9):
    check(all(R.TABLE_C[r][c] <= R.TABLE_C[r][c + 1] for c in range(6)), f"Table C row {r} across")
for c in range(7):
    check(all(R.TABLE_C[r][c] <= R.TABLE_C[r + 1][c] for r in range(1, 8)), f"Table C column {c + 1} down")

# spot checks read directly off the worksheet image
check(R.table_a(1, 1, 1, 1) == 1 and R.table_a(1, 1, 1, 2) == 2 and R.table_a(1, 3, 4, 2) == 4, "Table A UA1")
check(R.table_a(3, 2, 3, 1) == 4 and R.table_a(4, 3, 2, 2) == 5 and R.table_a(5, 3, 4, 2) == 8, "Table A UA3-5")
check(R.table_a(6, 1, 4, 2) == 9 and R.table_a(6, 3, 1, 1) == 9 and R.table_a(2, 3, 4, 1) == 5, "Table A UA6")
check(R.table_b(1, 1, 1) == 1 and R.table_b(1, 1, 2) == 3 and R.table_b(3, 3, 2) == 5, "Table B")
check(R.table_b(4, 6, 2) == 8 and R.table_b(6, 4, 2) == 9 and R.table_b(5, 3, 2) == 8, "Table B high")
check(R.table_c(1, 1) == 1 and R.table_c(8, 7) == 7 and R.table_c(5, 5) == 6 and R.table_c(3, 7) == 6, "Table C")
check(R.table_c(12, 12) == 7 and R.table_c(7, 3) == 6, "Table C clamps to the 8+ row and 7+ column")

# ---------------------------------------------------------------- body part scores
check([R.score_upper_arm(a) for a in (-30, -20, 0, 20, 21, 45, 46, 90, 91)] == [2, 1, 1, 1, 2, 2, 3, 3, 4], "upper arm bands")
check(R.score_upper_arm(30, shoulder_raised=True, abducted=True) == 4 and R.score_upper_arm(0, supported=True) == 1, "upper arm adjust")
check([R.score_lower_arm(a) for a in (30, 59, 60, 100, 101)] == [2, 2, 1, 1, 2] and R.score_lower_arm(80, True) == 2, "lower arm")
check([R.score_wrist(a) for a in (0, 10, -10, 16, -30)] == [1, 2, 2, 3, 3] and R.score_wrist(0, True) == 2, "wrist")
check([R.score_neck(a) for a in (-5, 5, 15, 25)] == [4, 1, 2, 3] and R.score_neck(5, True, True) == 3, "neck")
check([R.score_trunk(a) for a in (0, 10, 40, 70)] == [1, 2, 3, 4] and R.score_trunk(10, True, True) == 4, "trunk")
check(R.score_legs(True) == 1 and R.score_legs(False) == 2, "legs")
check(R.score_trunk(170, True, True) == 6 and R.score_neck(-5, True, True) == 6, "scores clamp at the table maximum")

# ---------------------------------------------------------------- muscle use and force
check(R.muscle_use_score(30, 1) == 0 and R.muscle_use_score(61, 0) == 1 and R.muscle_use_score(0, 4) == 1, "muscle use")
check(R.muscle_use_score(300, 0, static_threshold_s=600) == 0, "static threshold is a parameter")
f = R.calculate_force_score
check(f(0) == 0 and f(1.9) == 0 and f(1.9, True, True) == 0, "under 2 kg is 0")
check(f(2.0) == 1 and f(10.0) == 1, "2-10 kg intermittent is 1 (both ends inclusive)")
check(f(2.0, is_repeated=True) == 2 and f(5, is_static=True) == 2 and f(10.0, True, True) == 2, "2-10 kg static or repeated is 2")
check(f(10.1) == 3 and f(25, True, True) == 3, "over 10 kg is 3")
check(f(0.5, is_shock=True) == 3 and f(None) == 0, "shock is 3; unknown mass handled")

# ---------------------------------------------------------------- full worked examples (computed by hand from the tables)
res = R.rula_from_angles(10, 80, 0, 5, 0, mass_kg=0)           # everything neutral, no load
check((res.posture_a, res.posture_b, res.grand_score) == (1, 1, 1), "neutral posture should score 1")
check(res.action.startswith("Acceptable"), "neutral posture action")

# upper arm 60 -> 3, lower arm 30 -> 2, wrist 20 -> 3, twist 1  => Table A = 4
# neck 25 -> 3, trunk 30 -> 3, legs unsupported -> 2            => Table B = 5
# 5 kg repeated => muscle +1, force +2:  wrist&arm = 4+1+2 = 7, neck/trunk/leg = 5+1+2 = 8 (columns cap at 7+)
# Table C row 7, column 7+ = 7
res = R.rula_from_angles(60, 30, 20, 25, 30, mass_kg=5, is_repeated=True, legs_supported=False)
check(res.posture_a == 4 and res.posture_b == 5, f"example posture scores {res.posture_a}/{res.posture_b}")
check(res.muscle_use == 1 and res.force == 2, "example muscle/force")
check(res.wrist_arm_score == 7 and res.neck_trunk_leg_score == 8 and res.grand_score == 7, f"example grand {res}")
without_load = R.rula_from_angles(60, 30, 20, 25, 30, mass_kg=0, legs_supported=False)
check(without_load.grand_score < res.grand_score, "adding a load should raise the final score here")
# Table C row 4 is (3, 3, 3, 4, 5, 6, 6); column 5 gives 5
check(without_load.wrist_arm_score == 4 and without_load.neck_trunk_leg_score == 5 and without_load.grand_score == 5, "same posture, no load")
# the force score goes through Table C, it is NOT simply added to the final score
light = R.rula_from_angles(60, 30, 20, 25, 30, mass_kg=1.0, legs_supported=False)
heavy = R.rula_from_angles(60, 30, 20, 25, 30, mass_kg=12.0, legs_supported=False)
check(light.grand_score <= heavy.grand_score and heavy.force == 3 and light.force == 0, "heavier never scores lower")
# 12 kg adds +3 force to both posture scores: 4+3 = 7 and 5+3 = 8 -> Table C (7, 7+) = 7, which is +2 over 5, not +3
check(heavy.wrist_arm_score == 7 and heavy.neck_trunk_leg_score == 8 and heavy.grand_score == 7, "heavy example by hand")
check(heavy.grand_score - without_load.grand_score != heavy.force, "force must not be added to the grand score directly")

# ---------------------------------------------------------------- mass from USD
stage = Usd.Stage.CreateInMemory()
UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
box = UsdGeom.Cube.Define(stage, "/World/Box").GetPrim()
UsdPhysics.MassAPI.Apply(box).GetMassAttr().Set(7.5)
r = M.get_mass_kg(box)
check(r.known and r.kg == 7.5 and r.source == "physics:mass", f"physics mass {r}")

override = UsdGeom.Cube.Define(stage, "/World/Override").GetPrim()
UsdPhysics.MassAPI.Apply(override).GetMassAttr().Set(7.5)
override.CreateAttribute("ergo:mass_kg", __import__("pxr").Sdf.ValueTypeNames.Float).Set(12.0)
r = M.get_mass_kg(override)
check(r.kg == 12.0 and r.source == "ergo:mass_kg", "the ergo:mass_kg override wins")

bare = UsdGeom.Cube.Define(stage, "/World/Bare").GetPrim()
r = M.get_mass_kg(bare)
check((not r.known) and r.kg == 0.0 and r.source == "unknown", "no mass anywhere -> unknown, default 0")
check(M.get_mass_kg(bare, default_kg=1.0).kg == 1.0, "default_kg is configurable")

zero = UsdGeom.Cube.Define(stage, "/World/Zero").GetPrim()
UsdPhysics.MassAPI.Apply(zero).GetMassAttr().Set(0.0)
check(not M.get_mass_kg(zero).known, "physics:mass = 0 means 'computed later', so it counts as unknown")

crate = UsdGeom.Xform.Define(stage, "/World/Crate").GetPrim()
for i, kg in enumerate((3.0, 4.5)):
    part = UsdGeom.Cube.Define(stage, f"/World/Crate/Item{i}").GetPrim()
    UsdPhysics.MassAPI.Apply(part).GetMassAttr().Set(kg)
r = M.get_mass_kg(crate)
check(r.known and r.kg == 7.5 and r.source == "descendants", f"descendant masses add up {r}")

check(not M.get_mass_kg(None).known and not M.get_mass_kg(stage.GetPrimAtPath("/World/Missing")).known, "missing prim is safe")
check(M.get_mass_kg_at_path(stage, "/World/Box").kg == 7.5, "lookup by path")

grams = Usd.Stage.CreateInMemory()
UsdPhysics.SetStageKilogramsPerUnit(grams, 0.001)        # stage mass unit = gram
UsdGeom.Xform.Define(grams, "/World")
item = UsdGeom.Cube.Define(grams, "/World/Item").GetPrim()
UsdPhysics.MassAPI.Apply(item).GetMassAttr().Set(2500.0)  # 2500 g
check(abs(M.get_mass_kg(item).kg - 2.5) < 1e-9, "stage mass units are converted to kg")

print(f"ALL {checks} CHECKS PASSED")
