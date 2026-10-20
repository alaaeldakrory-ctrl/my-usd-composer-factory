# Kit job (run by Factory Sim's job runner, or from the console): renders the planned frames of the rack movie for
# each method through a session-layer camera (nothing is saved into the scenes). Settings at the top; copy this file
# into C:/OmniverseData/kit_jobs to run it.
import asyncio
import json
import os
import time

import omni.kit.app
import omni.timeline
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, Sdf, Usd, UsdGeom

PLAN = r"C:\OmniverseData\RackMovie\plan.json"
OUT = r"C:\OmniverseData\RackMovie"
METHODS = {"current": r"C:\OmniverseData\Rack_Assembly.usd", "improved": r"C:\OmniverseData\Rack_Assembly_Improved.usd"}
FRAMES = None            # None = every planned frame, else a list of output frame numbers
SPEED_TEST = 0           # also time this many consecutive frames
SETTLE = 5               # app updates per frame before capturing (RTX real-time accumulation)
RES = (1280, 720)


async def run(log):
    app = omni.kit.app.get_app()
    ctx = omni.usd.get_context()
    plan = json.load(open(PLAN))
    frames = list(range(len(plan["times"]))) if FRAMES is None else list(FRAMES)
    if SPEED_TEST:
        frames += list(range(1000, 1000 + SPEED_TEST))
    if ctx.has_pending_edit() and ctx.get_stage() and ctx.get_stage().GetRootLayer().realPath:
        layer = ctx.get_stage().GetRootLayer()
        backup = os.path.splitext(layer.realPath)[0] + "_unsaved_backup.usda"
        layer.Export(backup)
        log(f"unsaved changes exported to {backup}")
    for method, path in METHODS.items():
        for p in (path,):
            lay = Sdf.Layer.Find(p)
            if lay:
                lay.Reload(force=True)
        await ctx.open_stage_async(path)
        for _ in range(400):
            await app.next_update_async()
        stage = ctx.get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            cam = UsdGeom.Camera.Define(stage, "/MovieCam")
            cam.CreateFocalLengthAttr(float(plan["focal"]))
            cam.CreateClippingRangeAttr(Gf.Vec2f(5.0, 100000.0))
            op = UsdGeom.Xformable(cam).AddTransformOp()
        vp = get_active_viewport()
        vp.camera_path = "/MovieCam"
        try:
            vp.resolution = RES
        except Exception as e:   # noqa: BLE001
            log(f"could not set resolution: {e}")
        tl = omni.timeline.get_timeline_interface()
        tl.set_time_codes_per_second(plan["fps"])
        tl.pause()
        out = os.path.join(OUT, method)
        os.makedirs(out, exist_ok=True)
        t0 = time.time()
        for n, k in enumerate(frames):
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                op.Set(Gf.Matrix4d(*plan["cams"][k]))
            tl.set_current_time(plan["times"][k] / plan["fps"])
            for _ in range(SETTLE if n else 120):
                await app.next_update_async()
            await capture_viewport_to_file(vp, os.path.join(out, f"f{k:05d}.png")).wait_for_result()
            if n % 100 == 0 or n == len(frames) - 1:
                el = time.time() - t0
                log(f"{method}: {n + 1}/{len(frames)} frames, {el / (n + 1):.2f} s/frame, {el / 60:.1f} min")
        log(f"{method}: finished -> {out}")
    await asyncio.sleep(0)
