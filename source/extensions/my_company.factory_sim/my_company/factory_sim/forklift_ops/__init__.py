"""Generic forklift operations: find forklifts, racks and pallets in a stage, plan who moves which
pallet to which rack slot, and bake it as keyframed animation.

Pure Python + pxr (no Omniverse imports), so it also runs offline with usd-core.
Stage convention: Y up, centimeters (what USD Composer creates by default).
"""
