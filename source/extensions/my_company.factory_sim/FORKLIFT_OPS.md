# Forklift operations (Factory Sim panel)

Drop forklifts, racks, pallets and (optionally) a dock into the stage, press **Plan & Bake**, press Play.

## What it finds
Names containing **Forklift / Rack / Pallet / Dock** (direct children of the default prim) are picked up
automatically. Anything else: select it and press the matching tag button (**Obstacle** = drive around it).
Once any prim is tagged, only tagged prims are used.

| Thing | What is measured |
|---|---|
| Forklift | forward direction, fork heel/tip/blade height, footprint, road wheels (front/rear), steering wheel + spinner knob, lifting parts |
| Rack | shelf heights, open faces, slots per shelf (from the pallet size), what is already stored |
| Pallet | footprint, height, fork-pocket height, whether it sits in a rack slot |
| Dock | a floor area (e.g. a plane named Dock) divided into pallet-sized spots |

Known models: **Forklift_C01** (counterbalance, seated driver, steering wheel + knob, telescoping mast) and
**Forklift_B01** (order picker: the operator platform + forks rise together; standing operator facing the
control panel; max lift 1.9 m because the mast does not extend). Other forklifts still drive and carry,
without a driver, lifting only parts whose name contains "fork".

## Tasks
* **Store floor pallets in racks** - every pallet on the floor goes to a free slot (bottom or top shelves
  first), checked against the forklift's lift height and the shelf clearance.
* **Retrieve rack pallets to the dock** - every stored pallet is taken out (from the rack face it is flush
  with) and set down on a free dock spot, far spots first so the near ones stay reachable.

Jobs go to whichever forklift is free first. Routes avoid racks, pallets and obstacles (grid A* with curved
corners or turns on the spot); every job is checked against the swept footprint of the forklift and its
load. Forklifts give way to each other, and an idle forklift that is in the way is sent home first.

## Results
* Keyframes (every 2nd frame) on the forklifts, their lifting parts, wheels, steering wheel and the pallets.
  Physics (SimReady `PhysicsVariant`) is switched off on everything animated; **Clear animation** restores it.
* Drivers: Snatch (or any Mixamo character set in the panel), posed with the ergo_sim reach solver.
* Throughput summary in the panel and `<scene>_forklift_report.csv` next to the scene file: event log
  (job start / pick / place / job end / give way / drive home) plus per-forklift time budget (driving,
  lifting, turning, giving way, idle), utilization, loaded/empty distance, pallets per hour, cycle times.

## Code (`my_company/factory_sim/forklift_ops/`)
`scene.py` find + measure, `planner.py` jobs/routes/traffic, `bake.py` keyframes + physics switch,
`driver.py` seated/standing drivers, `stats.py` numbers + CSV, `kit_runner.py` Kit glue (async, viewport refresh,
overview camera). Pure Python + pxr except `kit_runner.py`.

## Offline tests (`tests_offline/`, need usd-core + the local NVIDIA assets)
`python test_plan.py two|today`, `python test_bake.py two|today`, `python open_user_scene.py`
(the real warehouse with online assets swapped for local copies). `make_test_stage.py` builds the layouts
`today`, `two`, `retrieve`, `mixed`.
