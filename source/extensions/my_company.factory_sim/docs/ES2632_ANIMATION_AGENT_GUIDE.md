# Agent guide: put the ES2632 guardrail-assembly animation into another scene

You are placing an existing, finished animation into a bigger factory scene. The animation shows two workers
assembling the guardrail kit on an ES2632 scissor lift. **Do not re-create or re-animate anything.** The
animation is already baked to a USD file. Your job is to reference that file into the factory scene, put it in
the right spot, and check it.

## 1. What already exists

| What | Where |
|---|---|
| Baked animation (open this, don't edit it) | `C:\OmniverseData\RailAssembly_ES2632.usd` |
| Lift model it references | `C:\OmniverseData\ES2632\ES2632_kit.usd` |
| Warehouse it references (not needed in the factory) | `C:\OmniverseData\ES2632\Warehouse_Env.usd` |
| Code that builds it (only if a rebuild is needed) | `source/extensions/my_company.factory_sim/my_company/factory_sim/assembly/` |
| Build command | `python assembly/build_demo.py` (run from `my_company/factory_sim/`), about 1–2 min |
| Kit console scripts (movie, snapshots) | `C:\Users\alaae\OneDrive\Documents\Kit\apps\My USD Composer\scripts\` (`rail_movie.py`, `rail_snaps2.py`) |

Facts about the animation file:
- **Y-up, centimetres** (`metersPerUnit = 0.01`), **30 fps**, frames **0–4584** (about 153 s).
- Everything sits under `/World`. The lift is on the origin, long axis along X, and the entry gate is at +X.
- Children of `/World` that matter:
  - `ScissorLift`: the lift and its six guardrail parts. They are animated.
  - `WorkerA`, `WorkerB`: the two NVIDIA characters with their baked skeleton animation.
  - `Tools`: the ratchets, shown only while a bolt is being tightened.
  - `Stands`, `Props`, `PartsTable`, `Cart`, `Marking*`: storage stands, the step stand, the parts table, the cart and floor tape.
  - `Cameras/Director`: the animated movie camera. `Cameras/Overview` is a fixed view.
  - `Environment`: the demo warehouse. **Deactivate this in the factory.**
- Floor space used, measured from the lift centre: about **x −450…+450 cm, z −400…+400 cm**. That's roughly a
  9 × 8 m clear rectangle with nothing taller than ankle height inside it. The workers walk all over this area.

## 2. Steps

Do them in order, and stop and report if any step fails.

1. **Open the factory scene** and read its stage metadata: `upAxis`, `metersPerUnit`, `timeCodesPerSecond`, and
   start and end time codes. Write them down.
2. **Pick the spot.** Find a clear 9 × 8 m floor area. Get the floor height there (the Y or Z of the floor
   surface) and the direction the cell should face. Don't move or delete factory content to make room
   without asking the user.
3. **Add a wrapper prim** in the factory scene, e.g. `/World/AssemblyCell` (Xform), and give it a **reference**
   to `C:\OmniverseData\RailAssembly_ES2632.usd` with prim path `/World`. Use a reference, not a sublayer and not
   a copy.
4. **Fix the units and up axis on the wrapper only:**
   - If the factory uses metres (`metersPerUnit = 1`), add scale `0.01` on the wrapper.
   - If the factory is Z-up, add `rotateX = 90` on the wrapper. That turns the cell's Y-up into Z-up; check that
     the workers stand upright, not lying down.
   - Then add translate (the chosen spot, at floor height) and rotate about the up axis (the facing).
   - The op order on the wrapper should be translate, then rotate-up-axis, then rotateX(if needed), then scale.
5. **Deactivate the demo warehouse:** set `active = false` on `/World/AssemblyCell/Environment`. Its lights and
   dome would otherwise double up with the factory's. If the cell looks dark afterwards, use the factory's
   lighting and don't re-enable the warehouse.
6. **Time:** the factory stage must cover the animation.
   - Set the factory's end time to at least frame 4584 at 30 fps (153 s).
   - If the factory runs at a different fps (e.g. 24), leave it. USD rescales referenced time automatically,
     because the animation file declares 30 fps. Just make the end time long enough (153 s × factory fps).
   - Save the factory scene to a **new file name** the first time (e.g. `<factory>_with_assembly.usd`). Never
     overwrite the user's original.
7. **Check it**, and see section 3.

## 3. Checks you must pass before saying you are done

Take screenshots or render frames at these times: **5 s, 20 s, 55 s, 95 s, 105 s, 125 s, 150 s**. Use the
fixed views below, not the director camera. Then confirm all of the following:

- [ ] The workers' feet are on the factory floor. They are not floating and not sunk. Check at 5 s and 150 s.
- [ ] No factory machine, rack, column or wall passes through a worker or a part at any of those times. The
      workers walk the whole 9 × 8 m area, so check the corners of the area too.
- [ ] The lift is the right size: about 2.3 m long, and the deck is about 1.2 m above the floor. If it looks tiny
      or huge, the unit scale in step 4 is wrong.
- [ ] At 150 s all six guardrail parts are on the lift, with none floating.
- [ ] The demo warehouse walls are not visible.
- [ ] The factory file still opens with everything else unchanged. Only the wrapper prim and the time range were added.

**The director camera** (`/World/AssemblyCell/Cameras/Director`) was framed for an open warehouse. In a busy
factory it may look through walls or machines. If it's blocked at any of the times above, don't edit the
animation file. Instead, add a new camera under the wrapper, or add a new keyed camera next to it, and tell
the user which one to use.

## 4. Rules

- **Never edit `RailAssembly_ES2632.usd` or `ES2632_kit.usd` by hand.** If the animation itself has to change,
  edit the Python in `assembly/` and rebuild with `build_demo.py`.
- **Never rebuild or overwrite a .usd that Kit has open.** Kit crashes and the file can be corrupted. Close the
  stage in Kit first (open a new empty stage). The build writes `*.building.usd` and swaps it in at the end; if it
  says "could not replace", the file is still open somewhere.
- After a rebuild, run `python tests_offline/check_bake_v2.py` (from `source/extensions/my_company.factory_sim`).
  It must report:
  - grip error p95 under 1 cm for both workers;
  - planted-toe slide under 1 cm;
  - every part "seated".
- **Confidential:** the lift is a real JLG product with JLG colours and decals. The factory scene may be the
  user's work file. Don't upload, share or post either file or any render anywhere.
- When unsure about the spot, the facing, or moving anything in the factory, **ask the user** instead of guessing.

## 5. Making a movie of it in the factory (optional)

Use `rail_movie.py` as the template, from the Kit console scripts folder:
1. Set `SCENE` to the new factory file.
2. Set the camera path to the camera you chose in section 3.
3. Set `NAME` to a new movie name.

It captures PNG frames through `omni.kit.capture.viewport`. Kit's own MP4 writer produced 0-byte files on this
machine, so encode the PNGs with ffmpeg (the `imageio-ffmpeg` binary works). Rendering takes over Kit, so
check that the user isn't working in it first.
