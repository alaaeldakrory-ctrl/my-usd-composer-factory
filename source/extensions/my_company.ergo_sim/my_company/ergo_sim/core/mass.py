"""Read the mass of a prim in kilograms. Needs only `pxr` (works in Omniverse and in plain Python with usd-core).

Order of preference, first match wins:
  1. `ergo:mass_kg`      custom attribute on the prim: your own override, always kilograms
  2. `physics:mass`     UsdPhysics.MassAPI mass (greater than 0), converted with the stage's kilogramsPerUnit
  3. sum of 1 or 2 over the prim's descendants (e.g. a box whose contents carry the mass)
  4. unknown            returned as known=False so the UI can say so; the number is `default_kg`

Note: a `physics:mass` of 0 normally means "work it out from density and shape when the simulation
starts", so it is treated as not authored. Mass is not computed here from density.
"""
from dataclasses import dataclass

from pxr import Usd, UsdPhysics

ERGO_MASS_ATTR = "ergo:mass_kg"
PHYSICS_MASS_ATTR = "physics:mass"


@dataclass
class MassResult:
    kg: float
    source: str      # "ergo:mass_kg", "physics:mass", "descendants", or "unknown"
    known: bool


def _kg_per_unit(stage):
    try:
        value = UsdPhysics.GetStageKilogramsPerUnit(stage)
        return value if value and value > 0 else 1.0
    except Exception:
        return 1.0


def _own_mass(prim, kg_per_unit):
    """(kg, source) from this prim alone, or None."""
    attr = prim.GetAttribute(ERGO_MASS_ATTR)
    if attr and attr.HasValue():
        value = attr.Get()
        if value is not None and value > 0:
            return float(value), ERGO_MASS_ATTR
    attr = prim.GetAttribute(PHYSICS_MASS_ATTR)
    if attr and attr.HasValue():
        value = attr.Get()
        if value is not None and value > 0:
            return float(value) * kg_per_unit, PHYSICS_MASS_ATTR
    return None


def get_mass_kg(prim, default_kg=0.0) -> MassResult:
    """Mass of `prim` in kilograms; see the module notes for the lookup order and fallbacks."""
    if prim is None or not prim.IsValid():
        return MassResult(default_kg, "unknown", False)
    kg_per_unit = _kg_per_unit(prim.GetStage())
    own = _own_mass(prim, kg_per_unit)
    if own:
        return MassResult(own[0], own[1], True)
    total, found = 0.0, False
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if child == prim:
            continue
        part = _own_mass(child, kg_per_unit)
        if part:
            total += part[0]
            found = True
    if found:
        return MassResult(total, "descendants", True)
    return MassResult(default_kg, "unknown", False)


def get_mass_kg_at_path(stage, prim_path, default_kg=0.0) -> MassResult:
    return get_mass_kg(stage.GetPrimAtPath(prim_path), default_kg)
