"""Read joint positions of an animated USD skeleton (e.g. a Mixamo character) at any timecode.

Needs only `pxr`, so it works in Omniverse and in plain Python with usd-core. It does NOT rely on the
skeleton's `animationSource`: Mixamo imports often point it at an empty "Take_001" animation while the real
motion is in "mixamo_com", so the animation prim with the most keyframes is used instead (or the one you name).
"""
from pxr import Gf, Usd, UsdGeom, UsdSkel

# joint-name endings, matched against the last part of each joint path (so any "mixamorig1_" / "mixamorig:" prefix works)
JOINT_ENDINGS = {
    "hips": "Hips", "neck": "Neck", "head": "Head", "spine": "Spine", "spine1": "Spine1", "spine2": "Spine2",
    "l_shoulder": "LeftShoulder", "r_shoulder": "RightShoulder",
    "l_up_leg": "LeftUpLeg", "r_up_leg": "RightUpLeg",
    "l_arm": "LeftArm", "r_arm": "RightArm",
    "l_forearm": "LeftForeArm", "r_forearm": "RightForeArm",
    "l_hand": "LeftHand", "r_hand": "RightHand",
    "l_hand_mid": "LeftHandMiddle1", "r_hand_mid": "RightHandMiddle1",
}


class SkeletonReader:
    def __init__(self, stage, skel_root_path=None, animation_path=None):
        self.stage = stage
        self.root = self._find_root(skel_root_path)
        self.skeleton = self._find_skeleton()
        self.animation = self._pick_animation(animation_path)
        skel = UsdSkel.Skeleton(self.skeleton)
        self.joints = list(skel.GetJointsAttr().Get())
        self.topology = UsdSkel.Topology(skel.GetJointsAttr().Get())
        self.rest = list(skel.GetRestTransformsAttr().Get())
        self._anim_query = UsdSkel.Cache().GetAnimQuery(self.animation)
        anim_joints = [str(j) for j in self._anim_query.GetJointOrder()]
        index = {str(j): i for i, j in enumerate(self.joints)}
        self._anim_to_skel = [index.get(j, -1) for j in anim_joints]
        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage) or 0.01
        self.up_axis = UsdGeom.GetStageUpAxis(stage)
        self.timecodes_per_second = stage.GetTimeCodesPerSecond() or 24.0
        self._by_key = self._map_joints()

    # ------------------------------------------------------------ discovery
    def _find_root(self, path):
        if path:
            return self.stage.GetPrimAtPath(path)
        for prim in self.stage.Traverse():
            if prim.IsA(UsdSkel.Root):
                return prim
        raise ValueError("no SkelRoot in the stage")

    def _find_skeleton(self):
        for prim in Usd.PrimRange(self.root):
            if prim.IsA(UsdSkel.Skeleton):
                return prim
        raise ValueError(f"no Skeleton under {self.root.GetPath()}")

    def _pick_animation(self, path):
        if path:
            return self.stage.GetPrimAtPath(path)
        best, best_keys = None, -1
        for prim in Usd.PrimRange(self.root):
            if prim.IsA(UsdSkel.Animation):
                keys = len(UsdSkel.Animation(prim).GetRotationsAttr().GetTimeSamples())
                if keys > best_keys:
                    best, best_keys = prim, keys
        if best is None or best_keys <= 0:
            raise ValueError("no SkelAnimation with keyframes under the SkelRoot")
        return best

    def _map_joints(self):
        found = {}
        for key, ending in JOINT_ENDINGS.items():
            for i, joint in enumerate(self.joints):
                if str(joint).split("/")[-1].lower().endswith(ending.lower()):
                    found.setdefault(key, i)
        return found

    # ------------------------------------------------------------ time
    def time_range(self):
        samples = UsdSkel.Animation(self.animation).GetRotationsAttr().GetTimeSamples()
        return (samples[0], samples[-1])

    def seconds_to_timecode(self, seconds):
        return seconds * self.timecodes_per_second

    # ------------------------------------------------------------ pose
    def joint_skel_transforms(self, timecode):
        """Joint transforms in skeleton space, in the skeleton's joint order."""
        local = list(self.rest)
        anim = self._anim_query.ComputeJointLocalTransforms(timecode)
        for a_index, s_index in enumerate(self._anim_to_skel):
            if s_index >= 0:
                local[s_index] = anim[a_index]
        skel = [None] * len(local)
        for i in range(len(local)):                      # parents always come before their children
            parent = self.topology.GetParent(i)
            skel[i] = local[i] if parent < 0 else local[i] * skel[parent]
        return skel

    def joint_world_positions(self, timecode):
        """Every joint's position in the stage's world frame, in meters: {joint path: (x, y, z)}."""
        to_world = UsdGeom.Xformable(self.skeleton).ComputeLocalToWorldTransform(Usd.TimeCode(timecode))
        skel = self.joint_skel_transforms(timecode)
        out = {}
        for joint, m in zip(self.joints, skel):
            p = to_world.Transform(Gf.Vec3d(*m.ExtractTranslation()))
            out[str(joint)] = (p[0] * self.meters_per_unit, p[1] * self.meters_per_unit, p[2] * self.meters_per_unit)
        return out

    def key_positions(self, timecode):
        """The joints RULA needs, by short key (see JOINT_ENDINGS), in meters."""
        every = self.joint_world_positions(timecode)
        names = [str(j) for j in self.joints]
        return {key: every[names[i]] for key, i in self._by_key.items()}

    def up_vector(self):
        return (0.0, 1.0, 0.0) if self.up_axis == "Y" else (0.0, 0.0, 1.0)
