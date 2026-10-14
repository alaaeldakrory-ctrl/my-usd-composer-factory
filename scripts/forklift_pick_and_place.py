"""Forklift pick-and-place animation for Omniverse Kit.

Run in Window > Script Editor. Bakes per-frame transforms (time samples) so the
forklift drives to the pallet, lifts it, backs up, turns, carries it to the
rack, sets it down and backs away. Safe to re-run after moving things around.

Setup:
  1. Place the forklift FACING the pallet, forks at ground level.
  2. Create > Xform, name it e.g. PalletTarget, and move it to where the
     pallet's origin should end up on the rack shelf.
  3. Fill in the prim paths below (Stage panel: right-click > Copy Prim Path).
"""
import math

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

# ---- Configuration -----------------------------------------------------------
FORKLIFT_PATH = "/World/Forklift"
FORK_PATH = ""  # prim holding the forks/carriage (moves up/down); "" to skip
PALLET_PATH = "/World/Pallet"
TARGET_PATH = "/World/PalletTarget"

PICK_OFFSET_M = 1.2  # forklift origin -> pallet origin distance when picked up
CARRY_LIFT_M = 0.15  # how high to lift the pallet for driving
CLEARANCE_M = 0.10  # extra height above the shelf before lowering onto it
BACKUP_M = 2.0  # how far to reverse after pickup and after drop-off
FPS = 24

# Phase durations in seconds
T_DRIVE_IN, T_LIFT, T_REVERSE, T_TURN = 2.0, 1.0, 1.5, 1.5
T_CARRY, T_LOWER, T_EXIT, T_FORKS_DOWN = 3.0, 1.0, 1.5, 1.0
# -----------------------------------------------------------------------------


def main():
    stage = omni.usd.get_context().get_stage()
    t0 = Usd.TimeCode(0)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 0.01
    up = Gf.Vec3d(0, 0, 1) if UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z else Gf.Vec3d(0, 1, 0)

    def units(meters):
        return meters / mpu

    def flat(v):
        return v - up * Gf.Dot(v, up)

    def translate(v):
        return Gf.Matrix4d(1.0).SetTranslate(v)

    def rotate(deg):
        return Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(up, deg))

    def signed_angle(a, b):
        return math.degrees(math.atan2(Gf.Dot(up, Gf.Cross(a, b)), Gf.Dot(a, b)))

    def get(path):
        if not path:
            return None
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            keys = ("fork", "pallet", "rack", "target", "lift", "mast")
            hints = [str(p.GetPath()) for p in stage.Traverse() if any(k in p.GetName().lower() for k in keys)]
            raise RuntimeError(f"Prim not found: {path}\nPossible matches:\n  " + "\n  ".join(hints[:40]))
        return prim

    forklift, fork, pallet, target = get(FORKLIFT_PATH), get(FORK_PATH), get(PALLET_PATH), get(TARGET_PATH)

    # Everything is read at frame 0, which equals the original pose, so re-runs are stable.
    cache = UsdGeom.XformCache(t0)
    F0 = cache.GetLocalToWorldTransform(forklift)
    P0 = cache.GetLocalToWorldTransform(pallet)
    p0 = F0.ExtractTranslation()
    pallet_pos = P0.ExtractTranslation()
    target_pos = cache.GetLocalToWorldTransform(target).ExtractTranslation()

    # --- Plan the path (world space) ---
    to_pallet = flat(pallet_pos - p0)
    if to_pallet.GetLength() <= units(PICK_OFFSET_M):
        raise RuntimeError("Forklift is too close to the pallet; move it back or lower PICK_OFFSET_M.")
    d0 = to_pallet.GetNormalized()
    pick_pos = p0 + d0 * (to_pallet.GetLength() - units(PICK_OFFSET_M))
    back_pos = pick_pos - d0 * units(BACKUP_M)

    offset = flat(pallet_pos - pick_pos)  # pallet relative to forklift while carried
    d1 = flat(target_pos - back_pos).GetNormalized()
    yaw1 = signed_angle(d0, d1)
    final_pos = flat(target_pos) + up * Gf.Dot(p0, up) - rotate(yaw1).TransformDir(offset)
    exit_pos = final_pos - d1 * units(BACKUP_M)

    carry_lift = units(CARRY_LIFT_M)
    shelf_lift = Gf.Dot(target_pos - pallet_pos, up)
    high_lift = max(shelf_lift, carry_lift) + units(CLEARANCE_M)

    # Keyframes: (duration_s, position, yaw_deg, lift, pallet_carried_during_segment)
    segments = [
        (T_DRIVE_IN, pick_pos, 0.0, 0.0, False),
        (T_LIFT, pick_pos, 0.0, carry_lift, True),
        (T_REVERSE, back_pos, 0.0, carry_lift, True),
        (T_TURN, back_pos, yaw1, carry_lift, True),
        (T_CARRY, final_pos, yaw1, high_lift, True),
        (T_LOWER, final_pos, yaw1, shelf_lift, True),
        (T_EXIT, exit_pos, yaw1, shelf_lift, False),
        (T_FORKS_DOWN, exit_pos, yaw1, 0.0, False),
    ]

    def forklift_world(pos, yaw):
        return F0 * translate(-p0) * rotate(yaw) * translate(pos)

    pallet_rel = P0 * forklift_world(pick_pos, 0.0).GetInverse()

    # --- Bake every frame ---
    forklift_parent_inv = cache.GetParentToWorldTransform(forklift).GetInverse()
    pallet_parent_inv = cache.GetParentToWorldTransform(pallet).GetInverse()
    if fork:
        fork_local0 = cache.GetLocalTransformation(fork)[0]
        fork_parent_inv = cache.GetParentToWorldTransform(fork).GetInverse()

    forklift_keys, pallet_keys, fork_keys = [], [], []
    state = (p0, 0.0, 0.0)
    pallet_world = P0
    frame = 0
    for duration, pos, yaw, lift, carried in segments:
        n = max(1, round(duration * FPS))
        start_frame = 0 if frame == 0 else 1  # don't duplicate the shared boundary frame
        for i in range(start_frame, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)  # smoothstep easing
            cur_pos = state[0] + (pos - state[0]) * s
            cur_yaw = state[1] + (yaw - state[1]) * s
            cur_lift = state[2] + (lift - state[2]) * s

            fw = forklift_world(cur_pos, cur_yaw)
            if carried:
                pallet_world = pallet_rel * fw * translate(up * cur_lift)

            f = frame + i
            forklift_keys.append((f, fw * forklift_parent_inv))
            pallet_keys.append((f, pallet_world * pallet_parent_inv))
            if fork:
                fork_keys.append((f, fork_local0 * translate(fork_parent_inv.TransformDir(up * cur_lift))))
        frame += n
        state = (pos, yaw, lift)

    # --- Write time samples ---
    def write(prim, keys):
        op = UsdGeom.Xformable(prim).MakeMatrixXform()
        op.GetAttr().Clear()
        for f, mtx in keys:
            op.Set(mtx, Usd.TimeCode(f))

    write(forklift, forklift_keys)
    write(pallet, pallet_keys)
    if fork:
        write(fork, fork_keys)

    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(frame)
    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0)
        timeline.set_end_time(frame / FPS)
    except Exception:
        pass

    for prim in (forklift, pallet):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"WARNING: {prim.GetPath()} has physics; it may ignore the animation when you press Play. "
                  "Remove Rigid Body / Articulation, or enable Kinematic.")

    print(f"Forklift animation baked: {frame} frames ({frame / FPS:.1f}s). Press Play on the timeline.")


main()
