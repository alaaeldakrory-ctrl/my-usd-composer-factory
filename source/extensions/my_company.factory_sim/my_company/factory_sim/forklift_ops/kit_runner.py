"""Kit side of forklift operations: run scan / plan / bake on the open stage without freezing the UI for long,
and make the renderer pick up the new keyframes."""
import asyncio
import time
import traceback

import omni.kit.app
import omni.timeline
import omni.usd

from . import bake as B
from . import driver as D
from . import planner as P
from . import scene as S
from . import stats as T


def _stage():
    return omni.usd.get_context().get_stage()


async def _frame(n=1):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def tag_selection(role):
    stage = _stage()
    paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
    for p in paths:
        S.set_role(stage.GetPrimAtPath(p), role)
    return len(paths)


def scan_text():
    return S.summary(S.scan(_stage()))


def _report_path(stage):
    """<scene>_forklift_report.csv next to the scene file (or in TEMP for an unsaved stage)."""
    import os
    real = stage.GetRootLayer().realPath
    if real:
        return os.path.splitext(real)[0] + "_forklift_report.csv"
    return os.path.join(os.environ.get("TEMP", "."), "forklift_report.csv")


async def refresh(paths):
    """Fabric only tracks prims that were animated when the stage loaded; re-populating these makes
    the viewport follow keyframes written just now (deactivate, wait, reactivate)."""
    stage = _stage()
    prims = [stage.GetPrimAtPath(p) for p in paths]
    prims = [p for p in prims if p and p.IsValid()]
    for p in prims:
        p.SetActive(False)
    await _frame(2)
    for p in prims:
        stage.GetPrimAtPath(p.GetPath()).ClearActive()
    await _frame(1)


async def run(settings, drivers=True, character=D.DEFAULT_CHARACTER, status=print):
    stage = _stage()
    try:
        status("Scanning the stage...")
        await _frame()
        sc = S.scan(stage)
        if not sc.forklifts:
            status("No forklifts found. Name them Forklift... or tag them with the Forklift button.")
            return
        lines = [S.summary(sc), ""]
        status("\n".join(lines + ["Planning..."]))
        await _frame()
        t0 = time.time()
        plan = P.Planner(sc, settings, log=lines.append).plan()
        t_plan = time.time() - t0
        lines += [f"Clip: {plan.end_t:.1f} s, {len(plan.jobs)} pallet(s) moved."]
        lines += ["! " + w for w in plan.warnings]
        t0 = time.time()
        result = None
        for kind, val in B.bake_steps(stage, sc, plan):
            if kind == "progress":
                status("\n".join(lines + [val]))
                await _frame()
            else:
                result = val
        end_f, per_agent = result
        t_bake = time.time() - t0
        t0 = time.time()
        if drivers:
            for kind, val in D.add_drivers_steps(stage, sc, plan, per_agent, character):
                if kind == "progress":
                    status("\n".join(lines + [val]))
                    await _frame()
                else:
                    lines += val
        t_drv = time.time() - t0
        print(f"[forklift ops] plan {t_plan:.1f}s, keyframes {t_bake:.1f}s, drivers {t_drv:.1f}s, "
              f"{len(sc.forklifts)} forklifts, {len(plan.jobs)} jobs, clip {plan.end_t:.1f}s")
        if plan.jobs:
            st = T.compute(plan)
            lines += ["", T.summary_text(st)]
            try:
                lines.append("Event log + numbers: " + T.write_csv(plan, st, _report_path(stage)))
            except OSError as e:
                lines.append(f"(could not write the report: {e})")
        fps = stage.GetTimeCodesPerSecond() or 60.0
        tl = omni.timeline.get_timeline_interface()
        tl.set_start_time(0)
        tl.set_end_time(end_f / fps)
        tl.set_current_time(0)
        status("\n".join(lines + ["Refreshing the viewport..."]))
        await refresh([str(f.prim.GetPath()) for f in sc.forklifts] + [str(p.prim.GetPath()) for p in sc.pallets])
        status("\n".join(lines + ["Done. Press Play."]))
    except Exception as e:   # show it in the panel, full trace in the console
        traceback.print_exc()
        status(f"Failed: {e}")


async def clear(status=print):
    stage = _stage()
    try:
        sc = S.scan(stage)
        n = B.clear(stage, sc)
        await refresh([str(f.prim.GetPath()) for f in sc.forklifts] + [str(p.prim.GetPath()) for p in sc.pallets])
        status(f"Removed {n} keyframes. Everything is back where you placed it.")
    except Exception as e:
        traceback.print_exc()
        status(f"Failed: {e}")


OVERVIEW_CAM = "/FactoryOpsOverview"


def overview_camera(on=True):
    """Switch the viewport to a camera that frames every forklift, rack and pallet (kept in the session layer,
    so it is never saved), or back to Perspective."""
    from omni.kit.viewport.utility import get_active_viewport
    from pxr import Gf, Usd, UsdGeom
    vp = get_active_viewport()
    stage = _stage()
    if not on:
        vp.camera_path = "/OmniverseKit_Persp"
        return "Viewport back on Perspective."
    sc = S.scan(stage)
    pts = [f.origin for f in sc.forklifts]
    for r in sc.racks:
        pts += r.box.corners()
    for p in sc.pallets:
        pts += p.box.corners()
    if not pts:
        return "Nothing to frame."
    lo = (min(p[0] for p in pts), min(p[1] for p in pts))
    hi = (max(p[0] for p in pts), max(p[1] for p in pts))
    c = Gf.Vec3d((lo[0] + hi[0]) / 2, 80.0, (lo[1] + hi[1]) / 2)
    span = max(hi[0] - lo[0], hi[1] - lo[1], 600.0)
    eye = c + Gf.Vec3d(0.6, 0.45, 0.6) * span * 1.6
    eye = Gf.Vec3d(eye[0], min(eye[1], 900.0), eye[2])          # stay under typical warehouse roofs
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        cam = UsdGeom.Camera.Define(stage, OVERVIEW_CAM)
        cam.CreateFocalLengthAttr(24.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(10.0, 1.0e6))
        xf = UsdGeom.Xformable(cam)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, c, Gf.Vec3d(0, 1, 0)).GetInverse())
    vp.camera_path = OVERVIEW_CAM
    return "Viewport on the overview camera (not saved with the file)."


def launch(coro):
    return asyncio.ensure_future(coro)
