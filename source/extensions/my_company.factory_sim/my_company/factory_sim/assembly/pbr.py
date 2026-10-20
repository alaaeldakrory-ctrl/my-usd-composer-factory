"""OmniPBR (MDL) materials for RTX, with a UsdPreviewSurface fallback for other renderers.

Textures are projected in object space (project_uvw), so they also work on meshes without UVs (IFC / CAD exports).
"""
from pxr import Gf, Sdf, UsdShade

MATS = r"C:/Nvidia_Assets/Warehouse_NVD/Assets/DigitalTwin/Materials/Base/"
TEX = {
    "paint_n": MATS + "Paint/Painted_Metal/T_Painted_Metal_Green_A_Normal.png",
    "grunge_r": MATS + "Paint/Misc/T_Grunge_Roughness.png",
    "satin_n": MATS + "Paint/Paint_Satin/Paint_Satin_N.png",
    "powder_n": MATS + "Paint/Painted_Metal/T_Metal_Gray_PowderCoated_A_Normal.png",
    "rubber_n": MATS + "Rubber/Rough/T_Rubber_Black_Rough_A_Normal.png",
    "steel_n": MATS + "Metals/Steel/T_Steel_A1_Normal.png",
    "steel_orm": MATS + "Metals/Steel/T_Steel_A1_ORM.png",
}


def srgb_to_linear(c):
    return tuple(((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c)


# looks by role: linear base colour, roughness, metallic, normal map, normal strength, roughness map influence
LOOKS = {
    "jlg_orange": dict(color=srgb_to_linear((0.93, 0.33, 0.04)), rough=0.3, metal=0.0, normal="paint_n", bump=0.25, rmap=0.25),
    "black_powder": dict(color=(0.018, 0.018, 0.02), rough=0.55, metal=0.0, normal="powder_n", bump=0.6, rmap=0.2),
    "cream_paint": dict(color=srgb_to_linear((0.9, 0.84, 0.7)), rough=0.42, metal=0.0, normal="satin_n", bump=0.4, rmap=0.2),
    "tyre_rubber": dict(color=srgb_to_linear((0.52, 0.52, 0.5)), rough=0.85, metal=0.0, normal="rubber_n", bump=0.8, rmap=0.0),
    "bright_steel": dict(color=(0.62, 0.62, 0.64), rough=0.28, metal=1.0, normal="steel_n", bump=0.3, rmap=0.0),
    "zinc_bolt": dict(color=(0.7, 0.71, 0.72), rough=0.35, metal=1.0, normal="steel_n", bump=0.2, rmap=0.0),
    "decal_teal": dict(color=srgb_to_linear((0.0, 0.69, 0.78)), rough=0.25, metal=0.0, normal=None, bump=0.0, rmap=0.0),
    "decal_white": dict(color=srgb_to_linear((0.95, 0.95, 0.95)), rough=0.25, metal=0.0, normal=None, bump=0.0, rmap=0.0),
    "steel_tread": dict(color=(0.45, 0.46, 0.48), rough=0.45, metal=1.0, normal="steel_n", bump=0.5, rmap=0.0),
    "lift_blue": dict(color=srgb_to_linear((0.08, 0.22, 0.55)), rough=0.32, metal=0.0, normal="paint_n", bump=0.25, rmap=0.25),
    "galvanised": dict(color=(0.62, 0.63, 0.64), rough=0.38, metal=1.0, normal="steel_n", bump=0.15, rmap=0.0),
    "timber": dict(color=srgb_to_linear((0.62, 0.47, 0.3)), rough=0.75, metal=0.0, normal="satin_n", bump=0.6, rmap=0.3),
    "yellow_tape": dict(color=srgb_to_linear((0.95, 0.72, 0.05)), rough=0.5, metal=0.0, normal="satin_n", bump=0.2, rmap=0.0),
}


def _inp(sh, name, tp, val, cs=None):
    a = sh.CreateInput(name, tp)
    a.Set(val)
    if cs:
        a.GetAttr().SetColorSpace(cs)
    return a


def add_mdl(stage, mat_path, look, scale=0.05):
    """Attach an OmniPBR MDL surface (outputs:mdl:surface) to an existing or new UsdShade.Material."""
    L = LOOKS[look]
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, mat_path + "/OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    C3, F, B, A, F2 = Sdf.ValueTypeNames.Color3f, Sdf.ValueTypeNames.Float, Sdf.ValueTypeNames.Bool, \
        Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.Float2
    _inp(sh, "diffuse_color_constant", C3, Gf.Vec3f(*L["color"]))
    _inp(sh, "reflection_roughness_constant", F, float(L["rough"]))
    _inp(sh, "metallic_constant", F, float(L["metal"]))
    _inp(sh, "specular_level", F, 0.5)
    if L["normal"]:
        _inp(sh, "normalmap_texture", A, Sdf.AssetPath(TEX[L["normal"]]), "raw")
        _inp(sh, "bump_factor", F, float(L["bump"]))
    if L["rmap"] > 0:
        _inp(sh, "reflectionroughness_texture", A, Sdf.AssetPath(TEX["grunge_r"]), "raw")
        _inp(sh, "reflection_roughness_texture_influence", F, float(L["rmap"]))
    _inp(sh, "project_uvw", B, True)
    _inp(sh, "world_or_object", B, False)
    _inp(sh, "texture_scale", F2, Gf.Vec2f(scale, scale))
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    return mat


def material(stage, mat_path, look, scale=0.05):
    """A complete material: OmniPBR for RTX + UsdPreviewSurface fallback."""
    mat = add_mdl(stage, mat_path, look, scale)
    L = LOOKS[look]
    pv = UsdShade.Shader.Define(stage, mat_path + "/Preview")
    pv.CreateIdAttr("UsdPreviewSurface")
    pv.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*L["color"]))
    pv.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(L["rough"]))
    pv.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(L["metal"]))
    mat.CreateSurfaceOutput().ConnectToSource(pv.ConnectableAPI(), "surface")
    return mat


def nearest_look(srgb):
    """Map an IFC colour (sRGB) to a look for the ES2632 parts."""
    palette = {(0.94, 0.31, 0.0): "jlg_orange", (0.16, 0.16, 0.16): "black_powder", (0.99, 0.87, 0.70): "cream_paint",
               (0.59, 0.59, 0.59): "tyre_rubber", (0.69, 0.69, 0.69): "bright_steel", (0.0, 0.69, 0.78): "decal_teal",
               (1.0, 1.0, 1.0): "decal_white"}
    best = min(palette, key=lambda p: sum((a - b) ** 2 for a, b in zip(p, srgb)))
    return palette[best]
