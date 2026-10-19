"""Converts the ES2632 BIM (IFC, stowed) into a USD asset whose guardrails are separate, movable groups.

    python es2632_asset.py "<...>/ES2632_CLOSED POSITION.ifc" C:/OmniverseData/ES2632/ES2632_kit.usd

Layout (cm, Y up, the deck's long axis along X, origin under the machine center):
    /ES2632/Machine/...          chassis, scissor stack, deck, toe boards
    /ES2632/Kit/SideL|SideR      side guardrail panels (+Z / -Z), each with its two corner posts
    /ES2632/Kit/EndF             entry gate end (+X)
    /ES2632/Kit/EndB             back end with the control-box bracket (-X)
Each Kit group carries an identity transform op so a part can be animated off its installed spot.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ifc_to_usd   # noqa: E402
import pbr   # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdShade   # noqa: E402

RAIL_MIN_TOP = 150.0      # anything reaching above this (cm) belongs to the guardrail system

# The guardrail kit as it is really put together (B-rep indices in this IFC; see the exploded view):
COMPONENTS = {
    # main side rails: two corner posts, mid + top rail; the gate's latch (97) and hinge bar (121, 124, 125)
    # are mounted on the front posts
    "SideL": [18, 98, 112, 131, 97],
    "SideR": [19, 99, 113, 130, 121, 124, 125],
    # extension-deck rail: back end frame (posts, mid/top rail, mesh panel, labels) + its two short side rails
    "ExtFrame": [4, 5, 6, 27, 28, 29, 30, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51],
    "CrossBar": [88, 90, 91, 92, 132, 133, 134, 135],                 # front top crossbar + corner fittings
    "Gate": [114, 115, 117, 118, 119, 120, 123, 126, 127],            # entry swing gate + hinge knuckles
    "Controller": [54, 55, 56, 57, 58, 59, 60, 61, 62, 63],           # platform control box + hanger
}
_OWNER = {k: g for g, ks in COMPONENTS.items() for k in ks}


def group(k, rgb, lo, hi):
    if k in _OWNER:
        return "Kit/" + _OWNER[k]
    if hi[1] <= RAIL_MIN_TOP and lo[1] < 135.0:      # (small gate hinges sit wholly above 135)
        return "Machine"
    print("  unassigned rail part", k, [round(v, 1) for v in lo], [round(v, 1) for v in hi])
    return "Kit/SideL" if (lo[2] + hi[2]) > 0 else "Kit/SideR"


def main(ifc, out):
    info = ifc_to_usd.convert(ifc, out, group_fn=group)
    st = Usd.Stage.Open(out)
    for g in COMPONENTS:
        UsdGeom.Xformable(st.GetPrimAtPath(f"/ES2632/Kit/{g}")).AddTransformOp().Set(Gf.Matrix4d(1))
    # RTX materials: one OmniPBR look per IFC colour (painted metal, powder coat, rubber, steel, decals)
    looks = {}
    for m in st.GetPrimAtPath("/ES2632/Looks").GetChildren():
        sh = UsdShade.Shader(m.GetChild("PBR"))
        lin = sh.GetInput("diffuseColor").Get()
        srgb = tuple((1.055 * c ** (1 / 2.4) - 0.055) if c > 0.0031308 else 12.92 * c for c in lin)
        look = pbr.nearest_look(srgb)
        pbr.add_mdl(st, str(m.GetPath()), look, scale=0.05)
        looks[m.GetName()] = look
    print("looks", looks)
    st.GetRootLayer().Save()
    counts = {}
    for i in info:
        counts[i[5]] = counts.get(i[5], 0) + 1
    print(out, counts)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
