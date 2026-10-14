"""Turn joint positions into the body angles RULA needs. Pure Python (no Omniverse or USD imports).

Input: a dict of joint positions in meters, as (x, y, z) tuples, in any right-handed frame, plus the
frame's `up` direction. The person's facing direction is worked out from the line between the two
shoulder joints, so no extra "forward" input is needed.

Needed joint keys:
  hips, neck, head, l_up_leg, r_up_leg, l_arm, r_arm        (arm = the shoulder joint / start of the upper arm)
  and for the chosen side: arm, forearm, hand, hand_mid     (e.g. r_arm, r_forearm, r_hand, r_hand_mid)

Conventions: flexion (bending forward) is positive. The elbow angle is the angle INSIDE the elbow
(straight arm = 180, right angle = 90), which is what the RULA lower-arm ranges (60-100) use.
"""
import math
from dataclasses import dataclass

# thresholds for the yes/no flags (degrees)
TRUNK_TWIST_DEG = 20.0
TRUNK_SIDE_BEND_DEG = 10.0
NECK_TWIST_DEG = 20.0
NECK_SIDE_BEND_DEG = 15.0
ARM_ABDUCTED_DEG = 30.0


def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def scale(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def length(a): return math.sqrt(dot(a, a))


def unit(a):
    n = length(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-9 else (0.0, 0.0, 0.0)


def angle_between_deg(a, b):
    ua, ub = unit(a), unit(b)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot(ua, ub)))))


def _horizontal(v, up):
    return unit(sub(v, scale(up, dot(v, up))))


@dataclass
class PoseAngles:
    trunk_deg: float
    trunk_twisted: bool
    trunk_side_bent: bool
    neck_deg: float
    neck_twisted: bool
    neck_side_bent: bool
    upper_arm_deg: float
    upper_arm_abducted: bool
    elbow_deg: float
    wrist_deg: float
    side: str


def pose_angles(j, side="r", up=(0.0, 1.0, 0.0), offsets=None) -> PoseAngles:
    """Body angles from joint positions. `side` is "l" or "r" (which arm to evaluate).
    `offsets`: the PoseAngles of the character's REST pose. A rig's rest pose is not geometrically neutral
    (Mixamo's spine is slightly curved and its head sits forward), so trunk, neck and wrist are measured
    relative to the rest pose, which counts as neutral."""
    up = unit(up)
    lateral = _horizontal(sub(j["r_arm"], j["l_arm"]), up)         # left shoulder -> right shoulder
    forward = cross(up, lateral)                                    # the way the person faces
    hip_line = _horizontal(sub(j["r_up_leg"], j["l_up_leg"]), up)

    trunk = sub(j["neck"], j["hips"])
    trunk_flex = math.degrees(math.atan2(dot(trunk, forward), dot(trunk, up)))
    trunk_side = math.degrees(math.atan2(dot(trunk, lateral), dot(trunk, up)))
    twist = angle_between_deg(lateral, hip_line)

    neck = sub(j["head"], j["neck"])
    neck_world = math.degrees(math.atan2(dot(neck, forward), dot(neck, up)))
    if "spine2" in j:                                               # head bend relative to the upper trunk segment,
        upper = sub(j["neck"], j["spine2"])                         # so a curved spine does not read as a bent neck
        ref_flex = math.degrees(math.atan2(dot(upper, forward), dot(upper, up)))
        ref_side = math.degrees(math.atan2(dot(upper, lateral), dot(upper, up)))
    else:
        ref_flex, ref_side = trunk_flex, trunk_side
    neck_flex = neck_world - ref_flex
    neck_side = math.degrees(math.atan2(dot(neck, lateral), dot(neck, up))) - ref_side

    arm = sub(j[f"{side}_forearm"], j[f"{side}_arm"])               # shoulder -> elbow
    trunk_u = unit(trunk)
    trunk_down = scale(trunk_u, -1.0)
    trunk_fwd = unit(sub(forward, scale(trunk_u, dot(forward, trunk_u))))   # "forward" in the leaning trunk's own frame
    ua = angle_between_deg(arm, trunk_down)
    if dot(arm, trunk_fwd) < 0 and abs(dot(arm, trunk_fwd)) >= abs(dot(arm, lateral)):
        ua = -ua                      # extension only when the arm is more backward than sideways, relative to the trunk
    abducted = abs(dot(unit(arm), lateral)) > math.sin(math.radians(ARM_ABDUCTED_DEG))

    elbow = angle_between_deg(sub(j[f"{side}_arm"], j[f"{side}_forearm"]), sub(j[f"{side}_hand"], j[f"{side}_forearm"]))
    wrist = angle_between_deg(sub(j[f"{side}_hand"], j[f"{side}_forearm"]), sub(j[f"{side}_hand_mid"], j[f"{side}_hand"]))

    if offsets is not None:
        trunk_flex -= offsets.trunk_deg
        neck_flex -= offsets.neck_deg
        wrist = max(0.0, wrist - offsets.wrist_deg)
    return PoseAngles(trunk_flex, twist > TRUNK_TWIST_DEG, abs(trunk_side) > TRUNK_SIDE_BEND_DEG,
                      neck_flex, False, abs(neck_side) > NECK_SIDE_BEND_DEG,
                      ua, abducted, elbow, wrist, side)
