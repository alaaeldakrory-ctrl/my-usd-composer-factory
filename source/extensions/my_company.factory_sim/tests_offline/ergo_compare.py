"""Before / after: RULA and spine load of the current vs the improved rack method, by step (no Kit).

    python ergo_compare.py [out_dir]
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "my_company", "factory_sim"))
import numpy as np   # noqa: E402
from pxr import Usd, UsdSkel   # noqa: E402
from assembly import choreo_rack as CR, ergo as E, ergo_report as ER, rig as RG, scene_rack as SR   # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else r"C:\OmniverseData\Rack_Ergo_Compare"
FILES = {"current": r"C:\OmniverseData\Rack_Assembly.usd", "improved": r"C:\OmniverseData\Rack_Assembly_Improved.usd"}
os.makedirs(OUT, exist_ok=True)
res = {}
for method, path in FILES.items():
    SR.configure(method)
    st = Usd.Stage.Open(path)
    rigs = {n: RG.CCRig(st, next(p for p in Usd.PrimRange(st.GetPrimAtPath(f"/World/Worker{n}")) if p.IsA(UsdSkel.Skeleton)))
            for n in "AB"}
    _, wk, _ = CR.script(SR.make_parts(), rigs)
    res[method] = (E.analyse(st, rigs, wk, CR.LOADS, int(st.GetEndTimeCode()) + 1), list(CR.STEPS))


def stats(scores, steps, who, lab):
    t0, t1 = next((a, b) for a, b, l in steps if l == lab)
    rows = scores[who]
    sm = E.smooth_scores(rows)
    sel = [(r, s) for r, s in zip(rows, sm) if t0 <= r["t"] < t1]
    dt = 1 / E.FPS
    return {"peak": max(s for _, s in sel), "red": sum(dt for _, s in sel if s >= 5),
            "now": sum(dt for _, s in sel if s >= 7), "spine": max(r["l5s1_N"] for r, _ in sel),
            "over_al": sum(dt for r, _ in sel if r["l5s1_N"] >= E.NIOSH_AL)}


labels = [l for _, _, l in res["current"][1]]
lines = ["# Rack assembly: current vs improved method (RULA + spine load)", "",
         "Improved method: beams stored on 80 cm trestles (was 40 cm), frame tops on an 80 cm sawhorse (was a 34 cm "
         "block), anchors drilled standing with a stand-up drilling extension (was crouching). Same workers, same "
         "steps, same parts.", "",
         "| Step | Worker | RULA peak | Time at RULA 5-7 | Time at RULA 7 | Spine peak (N) | Time over 3400 N |",
         "|---|---|---|---|---|---|---|"]
tot = {m: {"red": 0.0, "now": 0.0, "over": 0.0} for m in FILES}
for lab in labels:
    for who in "AB":
        a = stats(*res["current"], who, lab)
        b = stats(*res["improved"], who, lab)
        for m, x in (("current", a), ("improved", b)):
            tot[m]["red"] += x["red"]
            tot[m]["now"] += x["now"]
            tot[m]["over"] += x["over_al"]
        lines.append(f"| {lab} | {who} | {a['peak']} -> {b['peak']} | {a['red']:.1f} -> {b['red']:.1f} s | "
                     f"{a['now']:.1f} -> {b['now']:.1f} s | {a['spine']:,.0f} -> {b['spine']:,.0f} | "
                     f"{a['over_al']:.1f} -> {b['over_al']:.1f} s |")
c, i = tot["current"], tot["improved"]
lines += ["", f"**Whole job, both workers:** time at RULA 5-7 {c['red']:.0f} s -> {i['red']:.0f} s "
          f"({100 * (1 - i['red'] / max(c['red'], 1e-6)):.0f}% less); at RULA 7 {c['now']:.0f} s -> {i['now']:.0f} s; "
          f"spine over the NIOSH action limit {c['over']:.1f} s -> {i['over']:.1f} s."]
open(os.path.join(OUT, "compare_by_step.md"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
merged = {}
for who in "AB":
    for m in FILES:
        merged[f"{who} {m}"] = res[m][0][who]
ER.timeline_png(merged, res["current"][1], os.path.join(OUT, "compare_timeline.png"), row=120)
