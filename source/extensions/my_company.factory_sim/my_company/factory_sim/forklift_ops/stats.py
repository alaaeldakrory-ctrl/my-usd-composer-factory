"""Throughput numbers for a plan: per-forklift time budget and distances, overall pallets/hour and cycle
times, plus a timestamped event log (CSV) for Excel or a DES tool."""
import csv
import math

_ACTIVE = {"path": "driving", "pivot": "turning", "lift": "lifting"}


def compute(plan):
    """Dict with "forklifts" (list of per-forklift dicts) and "overall"."""
    clip = plan.end_t
    rows = []
    for agent in plan.agents:
        name = agent.fl.name
        jobs = [j for j in plan.jobs if j["forklift"] == name]
        loaded = [(j["picked"], j["placed"]) for j in jobs]
        t = {"driving": 0.0, "turning": 0.0, "lifting": 0.0, "waiting": 0.0}
        dist_loaded = dist_empty = 0.0
        for s in agent.segs:
            if s.kind == "wait":
                if s.note == "give way":
                    t["waiting"] += s.dur
                continue
            t[_ACTIVE[s.kind]] += s.dur
            if s.kind == "path":
                mid = s.t0 + s.dur / 2
                if any(a <= mid <= b for a, b in loaded):
                    dist_loaded += s.lens[-1]
                else:
                    dist_empty += s.lens[-1]
        active = t["driving"] + t["turning"] + t["lifting"]
        rows.append({
            "forklift": name, "jobs": len(jobs),
            "driving_s": t["driving"], "turning_s": t["turning"], "lifting_s": t["lifting"],
            "waiting_s": t["waiting"], "idle_s": max(0.0, clip - active - t["waiting"]),
            "utilization": active / clip if clip > 0 else 0.0,
            "distance_loaded_m": dist_loaded / 100.0, "distance_empty_m": dist_empty / 100.0,
        })
    moved = len(plan.jobs)
    last_place = max([j["placed"] for j in plan.jobs] or [0.0])
    first_start = min([j["start"] for j in plan.jobs] or [0.0])
    span = max(last_place - first_start, 1e-6)
    cycles = [j["end"] - j["start"] for j in plan.jobs]
    overall = {
        "pallets_moved": moved,
        "clip_s": clip,
        "makespan_s": last_place,
        "throughput_per_hour": moved / span * 3600.0 if moved else 0.0,
        "avg_cycle_s": sum(cycles) / len(cycles) if cycles else 0.0,
        "max_cycle_s": max(cycles) if cycles else 0.0,
        "avg_wait_per_job_s": sum(j["waited"] for j in plan.jobs) / moved if moved else 0.0,
        "fleet_utilization": sum(r["utilization"] for r in rows) / len(rows) if rows else 0.0,
        "distance_total_m": sum(r["distance_loaded_m"] + r["distance_empty_m"] for r in rows),
    }
    return {"forklifts": rows, "overall": overall}


def summary_text(st):
    o = st["overall"]
    lines = [f"Throughput: {o['pallets_moved']} pallet(s) in {o['makespan_s']:.0f} s = "
             f"{o['throughput_per_hour']:.0f} pallets/hour",
             f"  cycle time avg {o['avg_cycle_s']:.0f} s (max {o['max_cycle_s']:.0f} s), "
             f"waiting avg {o['avg_wait_per_job_s']:.0f} s per job, fleet utilization {100 * o['fleet_utilization']:.0f}%"]
    for r in st["forklifts"]:
        lines.append(f"  {r['forklift']}: {r['jobs']} job(s), busy {100 * r['utilization']:.0f}% "
                     f"(drive {r['driving_s']:.0f} s, lift {r['lifting_s']:.0f} s, turn {r['turning_s']:.0f} s), "
                     f"gave way {r['waiting_s']:.0f} s, idle {r['idle_s']:.0f} s, "
                     f"{r['distance_loaded_m']:.0f} m loaded / {r['distance_empty_m']:.0f} m empty")
    return "\n".join(lines)


def events(plan):
    """[(time s, forklift, event, pallet, from, to, detail)] sorted by time."""
    out = []
    for j in plan.jobs:
        f, p = j["forklift"], j["pallet"]
        out.append((j["start"], f, "job start", p, j["from"], j["to"], f"ready at {j['ready']:.1f} s"))
        out.append((j["picked"], f, "pick", p, j["from"], j["to"], ""))
        out.append((j["placed"], f, "place", p, j["from"], j["to"], ""))
        out.append((j["end"], f, "job end", p, j["from"], j["to"], f"cycle {j['end'] - j['start']:.1f} s"))
    for agent in plan.agents:
        f = agent.fl.name
        for s in agent.segs:
            if s.kind == "wait" and s.note == "give way":
                out.append((s.t0, f, "give way", "", "", "", f"{s.dur:.1f} s"))
            elif s.note == "home" and (not out or out[-1][2] != "drive home" or out[-1][1] != f):
                out.append((s.t0, f, "drive home", "", "", "", ""))
    out.sort(key=lambda e: e[0])
    return out


def write_csv(plan, st, path):
    """Two sections in one file: the event log, then the per-forklift and overall numbers."""
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "forklift", "event", "pallet", "from", "to", "detail"])
        for e in events(plan):
            w.writerow([f"{e[0]:.2f}"] + list(e[1:]))
        w.writerow([])
        keys = ["forklift", "jobs", "utilization", "driving_s", "lifting_s", "turning_s", "waiting_s", "idle_s",
                "distance_loaded_m", "distance_empty_m"]
        w.writerow(keys)
        for r in st["forklifts"]:
            w.writerow([r[k] if isinstance(r[k], (str, int)) else f"{r[k]:.3f}" for k in keys])
        w.writerow([])
        for k, v in st["overall"].items():
            w.writerow([k, f"{v:.3f}" if isinstance(v, float) else v])
    return path
