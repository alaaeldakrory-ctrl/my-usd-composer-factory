"""Operator tasks at a bench, scored with RULA frame by frame, plus bench-height "what-if" sweeps.

Pure Python + pxr (no omni imports). Coordinates for task points are (right, up, forward) in meters, measured
from the spot on the floor between the operator's feet (+forward = the way the operator faces).
"""
import csv
import math
from dataclasses import dataclass, field

try:
    from . import reach as RC
    from . import rula as R
except (ImportError, ValueError):
    import reach as RC
    import rula as R


# ---------------------------------------------------------------- task description
@dataclass
class TaskSpec:
    kind: str = "weld"                 # "weld" or "assembly"
    bench_height: float = 0.90         # top of the bench (m)
    work_forward: float = 0.38         # how far in front of the feet the work sits (m)
    seconds: float = 8.0               # length of one cycle (weld: one pass; assembly: two parts)
    mass_kg: float = None              # load in the hand (default: torch 1.5 kg / part 0.5 kg)
    seam_length: float = 0.30          # weld seam length (m)
    wrist_flex_deg: float = 10.0
    look_at_work: bool = True


def _smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def _lerp(a, b, u):
    return tuple(a[i] + (b[i] - a[i]) * u for i in range(3))


@dataclass
class FrameTargets:
    t: float                           # seconds
    right: tuple = None                # right-hand target (right, up, forward) m, or None = relaxed
    left: tuple = None
    look: tuple = None
    mass_kg: float = 0.0               # load held by the working hand this frame
    is_static: bool = False
    is_repeated: bool = False
    phase: str = ""


def weld_targets(spec: TaskSpec, fps=12):
    """Right hand drags a torch along a straight seam on the bench; left hand steadies the work. Torch is held (static)."""
    mass = 1.5 if spec.mass_kg is None else spec.mass_kg
    z = spec.bench_height + 0.04
    half = spec.seam_length / 2.0
    start = (-half + 0.0, z, spec.work_forward)
    end = (half, z, spec.work_forward)
    n = max(2, int(spec.seconds * fps))
    out = []
    for k in range(n + 1):
        t = k / fps
        u = k / n
        if u < 0.15:                                    # reach to the start of the seam
            p = _lerp((0.20, z + 0.12, spec.work_forward - 0.05), start, _smooth(u / 0.15))
            phase = "reach"
        elif u < 0.85:                                  # weld along the seam
            p = _lerp(start, end, (u - 0.15) / 0.70)
            phase = "weld"
        else:                                           # withdraw
            p = _lerp(end, (0.20, z + 0.12, spec.work_forward - 0.05), _smooth((u - 0.85) / 0.15))
            phase = "withdraw"
        support = (-0.10, z, spec.work_forward - 0.04)
        out.append(FrameTargets(t, p, support, p if spec.look_at_work else None, mass, True, False, phase))
    return out


def assembly_targets(spec: TaskSpec, fps=12):
    """Two repeated pick-carry-place-release cycles: a 0.5 kg part from a tray at the right to a fixture in front."""
    mass = 0.5 if spec.mass_kg is None else spec.mass_kg
    z = spec.bench_height + 0.04
    tray = (0.28, z, spec.work_forward - 0.06)
    fixture = (0.0, z, spec.work_forward)
    n = max(2, int(spec.seconds * fps))
    out = []
    for k in range(n + 1):
        t = k / fps
        u = (k / n) * 2.0
        u = u - int(u) if u < 2.0 else 1.0               # 0..1 within a cycle
        lift = 0.10
        if u < 0.25:
            p, held, phase = _lerp((fixture[0], z + lift, fixture[2]), tray, _smooth(u / 0.25)), 0.0, "reach"
        elif u < 0.35:
            p, held, phase = tray, mass, "grasp"
        elif u < 0.70:
            v = _smooth((u - 0.35) / 0.35)
            p = _lerp(tray, fixture, v)
            p = (p[0], p[1] + lift * math.sin(math.pi * v), p[2])
            held, phase = mass, "carry"
        elif u < 0.85:
            p, held, phase = fixture, mass, "place"
        else:
            p, held, phase = _lerp(fixture, (fixture[0], z + lift, fixture[2]), _smooth((u - 0.85) / 0.15)), 0.0, "release"
        out.append(FrameTargets(t, p, None, p if spec.look_at_work else None, held, False, True, phase))
    return out


def build_targets(spec: TaskSpec, fps=12):
    return weld_targets(spec, fps) if spec.kind == "weld" else assembly_targets(spec, fps)


# ---------------------------------------------------------------- per-frame analysis
@dataclass
class FrameResult:
    t: float
    phase: str
    overrides: object
    lean_deg: float
    reachable: bool
    hand_error_cm: float
    mass_kg: float
    trunk: float
    neck: float
    upper_arm: float
    elbow: float
    wrist: float
    rula: R.RulaResult = None


def analyse(solver, targets, side="r"):
    frames = []
    for ft in targets:
        tg = {"r": RC.to_skel_cm(*ft.right) if ft.right else None,
              "l": RC.to_skel_cm(*ft.left) if ft.left else None}
        look = RC.to_skel_cm(*ft.look) if ft.look else None
        ov, info = solver.solve(tg, look=look)
        a = solver.angles(ov, side)
        res = R.rula_from_angles(a.upper_arm_deg, a.elbow_deg, a.wrist_deg, a.neck_deg, a.trunk_deg,
                                 mass_kg=ft.mass_kg, is_repeated=ft.is_repeated, is_static=ft.is_static,
                                 trunk_twisted=a.trunk_twisted, trunk_side_bent=a.trunk_side_bent,
                                 upper_arm_abducted=a.upper_arm_abducted)
        frames.append(FrameResult(ft.t, ft.phase, ov, info["lean_deg"], bool(info["lean_ok"]),
                                  max(info["hand_error_cm"].values()) if info["hand_error_cm"] else 0.0,
                                  ft.mass_kg, a.trunk_deg, a.neck_deg, a.upper_arm_deg, a.elbow_deg, a.wrist_deg, res))
    return frames


def summarize(frames):
    n = len(frames)
    scores = [f.rula.grand_score for f in frames]
    levels = {"1-2": 0, "3-4": 0, "5-6": 0, "7": 0}
    for s in scores:
        levels["1-2" if s <= 2 else "3-4" if s <= 4 else "5-6" if s <= 6 else "7"] += 1
    return {
        "max_score": max(scores), "mean_score": sum(scores) / n,
        "pct_1_2": 100.0 * levels["1-2"] / n, "pct_3_4": 100.0 * levels["3-4"] / n,
        "pct_5_6": 100.0 * levels["5-6"] / n, "pct_7": 100.0 * levels["7"] / n,
        "max_trunk_deg": max(f.trunk for f in frames), "mean_trunk_deg": sum(f.trunk for f in frames) / n,
        "max_upper_arm_deg": max(f.upper_arm for f in frames), "max_neck_deg": max(f.neck for f in frames),
        "max_lean_deg": max(f.lean_deg for f in frames),
        "all_reachable": all(f.reachable for f in frames),
        "worst_hand_error_cm": max(f.hand_error_cm for f in frames),
    }


def sweep_bench_height(solver, kind="weld", heights=(1.10, 1.05, 1.00, 0.95, 0.90, 0.85, 0.80), fps=6, **task_kw):
    """Run the same task at several bench heights. Returns a list of {bench_height, ...summary...}."""
    rows = []
    for h in heights:
        spec = TaskSpec(kind=kind, bench_height=h, **task_kw)
        s = summarize(analyse(solver, build_targets(spec, fps)))
        s["bench_height"] = h
        rows.append(s)
    return rows


def write_summary_csv(rows, path):
    keys = ["bench_height"] + [k for k in rows[0] if k != "bench_height"]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})


def write_frames_csv(frames, path):
    cols = ["t_s", "phase", "mass_kg", "lean_deg", "trunk_deg", "neck_deg", "upper_arm_deg", "elbow_deg", "wrist_deg",
            "upper_arm_score", "lower_arm_score", "wrist_score", "neck_score", "trunk_score", "force", "muscle_use",
            "posture_a", "posture_b", "rula", "reachable"]
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for f in frames:
            r = f.rula
            w.writerow([round(f.t, 3), f.phase, f.mass_kg, round(f.lean_deg, 1), round(f.trunk, 1), round(f.neck, 1),
                        round(f.upper_arm, 1), round(f.elbow, 1), round(f.wrist, 1), r.upper_arm, r.lower_arm, r.wrist,
                        r.neck, r.trunk, r.force, r.muscle_use, r.posture_a, r.posture_b, r.grand_score, int(f.reachable)])


def frames_to_animation(stage, anim_path, rig, frames, timecodes_per_second=24.0):
    """Write the solved poses as a SkelAnimation (timecodes follow the frame times)."""
    return RC.write_animation(stage, anim_path, rig, [(f.t * timecodes_per_second, f.overrides) for f in frames])
