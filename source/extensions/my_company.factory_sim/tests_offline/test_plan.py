"""Offline planning test: scan a test layout, plan, and check the result.
Run:  python test_plan.py [today|two]"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "my_company", "factory_sim"))
from forklift_ops import planner as P   # noqa: E402
from forklift_ops import scene as S     # noqa: E402
import make_test_stage                   # noqa: E402


def main(layout):
    st = make_test_stage.build(layout)
    sc = S.scan(st)
    print(S.summary(sc))
    t0 = time.time()
    plan = P.Planner(sc, P.Settings()).plan()
    print(f"planned in {time.time() - t0:.1f}s; clip length {plan.end_t:.1f}s")
    for w in plan.warnings:
        print("  WARNING", w)

    # 1. forklifts never overlap each other
    worst = 1e9
    t = 0.0
    while t < plan.end_t:
        states = [(a, a.state(t)) for a in plan.agents]
        for i in range(len(states)):
            for j in range(i + 1, len(states)):
                a, (pa, ha, *_r) = states[i]
                b, (pb, hb, *_r2) = states[j]
                ca = P._footprint_circles(a.fl, pa, ha, None)
                cb = P._footprint_circles(b.fl, pb, hb, None)
                for x, z, r in ca:
                    for x2, z2, r2 in cb:
                        worst = min(worst, math.hypot(x - x2, z - z2) - r - r2)
        t += 0.25
    print("closest approach between forklifts (cm, >0 = clear):", round(worst, 1) if worst < 1e9 else "n/a")

    # 2. every pallet ends where planned, at shelf height
    for name, tr in plan.pallets.items():
        c, y, dh = P.pallet_state(tr, plan.end_t + 1)
        det = [e for e in tr.events if e[1] == "detach"][0]
        print(f"  {name}: final {[round(v) for v in c]} y={y:.1f} (slot y {det[3]:.1f}), turned {dh:.0f} deg")
    # 3. path continuity: no teleports between segments
    for a in plan.agents:
        jumps = 0
        prev = None
        for s in a.segs:
            p0 = s.state(s.t0)
            if prev is not None and (math.dist(prev[0], p0[0]) > 1.0 or abs(S.wrap180(prev[1] - p0[1])) > 1.0):
                jumps += 1
                print(f"    jump before '{s.note}' ({s.kind}) at {s.t0:.1f}s: pos {math.dist(prev[0], p0[0]):.1f} cm, "
                      f"heading {S.wrap180(prev[1] - p0[1]):.1f} deg")
            prev = s.state(s.t1)
        print(f"  {a.fl.name}: {len(a.segs)} segments, ends {a.end_t:.1f}s, discontinuities {jumps}, "
              f"back home {math.dist(a.pos, a.fl.origin) < 2}")
    return plan


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "two")
