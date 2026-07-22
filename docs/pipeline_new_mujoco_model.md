# Adapting the Pipeline to a Different MuJoCo XML Model

This file is the checklist for repeating it with a
**different** MJCF humanoid (a different body, a differently-named
skeleton, a modified-proportion variant, a robot, etc.).

## What's fully generic (no changes needed)

These steps only depend on MuJoCo's own body/geom arrays, not on specific names:

- **`export_skeleton_json.py`** — walks `model.nbody`/`model.ngeom`
  generically. Just point `--input` at the new `.xml`.
- **`build_armature_fbx.py`** — the BFS bone-building, zero-length-body
  merge (`resolve_bone_parent`/`has_own_bone`), leaf-tail extrapolation, and
  capsule/sphere/ellipsoid mesh construction are all name-agnostic; they
  operate purely on the JSON's `parent`/`world_pos` graph. The in-file
  comments mention CMU-specific body names (`lhipjoint`, `lowerback`, ...)
  only as *examples* of what gets merged — the logic itself doesn't
  hardcode them.

  ```bash
  /Applications/Blender.app/Contents/MacOS/Blender --background --python \
    scripts/build_armature_fbx.py -- \
    --input assets/{model}/{model}_skelton.json \
    --output assets/{model}/{model}.fbx
  ```

## Below is under repo BEDLEM2_RETARGETING

- **`create_cmu_ik_retargeter.py`** — takes source/target IK Rig paths as
  arguments; auto-maps chains by exact name and disables IK. No changes.
- **`retarget_batch.py` / `retarget.py` / `export_anim_dir_fbx.py` /
  `render_videos.py`** — all take asset paths / directories as arguments.
  No changes.
- **`import_batch.py` / `reimport_skeletal_mesh.py`** — generic FBX import.

### What must change per-model

### 1. The IK Rig's retarget chains (`create_cmu_ik_rig.py`)

This is the one part that's hardcoded to `humanoid_CMU`'s bone names
(`RETARGET_CHAINS`, `GOALS`, `RETARGET_ROOT_BONE`). For a new model you
need an analogous definition built from *that* model's own bone names.

**Steps:**

1. Dump the new model's skeleton and inspect its body hierarchy:
   ```bash
   uv run scripts/export_skeleton_json.py --input mjcf/<new_model>.xml \
   --output assets/<new_model>_skeleton.json
   python3 -c "
   import json
   d = json.load(open('assets/<new_model>_skeleton.json'))
   for b in d['bodies']:
       print(b['name'], '<-', b['parent'])
   "
   ```
   (Or, after importing into UE, use `dump_skeleton_bones.py` on the
   resulting SkeletalMesh — note zero-offset bodies will already be merged
   by then, so bone names may differ slightly from the raw body list.)

2. Identify the chain groups, matching `smplx_IKRig`'s chain *names* (so
   `auto_map_chains(EXACT)` can pair them up) but pointing at *this* model's
   bone names:
   - `root` — see the gotcha below before adding this.
   - `Spine` — start/end bones spanning the spine.
   - `neck`, `head`.
   - `LeftLeg` / `RightLeg` — hip-to-foot, with an IK goal on the foot bone
     (matches `smplx_IKRig`'s `LeftFootIK`/`RightFootIK` pattern, even
     though IK ends up disabled — the goal only needs to exist).
   - `LeftClavicle` / `RightClavicle` — usually a single bone.
   - `LeftArm` / `RightArm` — shoulder-to-wrist.
   - Optional: finger/thumb chains if you need hand detail (`smplx_IKRig`
     has `LeftThumb`/`LeftIndex`/etc. — CMU's rig skips these since
     `humanoid_CMU.xml` doesn't have individual finger joints).

3. Set `RETARGET_ROOT_BONE` to whichever bone represents the actual moving
   root/pelvis of the new model (usually the top-level body under
   `worldbody`, unless the model itself defines a separate static reference
   bone above it).

4. **Gotcha — don't double up the root bone.** `smplx_IKRig` has *two*
   bones at the top: a static `root` (mapped to its own `root` chain, used
   only as a stable reference) and a separate, actually-moving `pelvis`
   (set as `retarget_root_bone`, *not* given its own chain). If your new
   model — like `humanoid_CMU` — only has **one** top-level bone playing
   both roles, do **not** also map a `root` chain onto it. Doing so applies
   rotation twice to the same bone (once via the chain's own FK, once via
   Retarget Root Settings) and tips the whole body over. Either:
   - Skip the `root` chain entirely (works fine — this is what
     `create_cmu_ik_rig.py` does), or
   - If you want the two-tier structure to match `smplx_IKRig` exactly,
     add a genuinely separate static bone above the model's root in
     `build_armature_fbx.py` and point the new model's `root` chain at
     *that* bone instead of the retarget root.

5. Recreate the IK Rig and IK Retargeter (steps 4-5 of
   `pipeline_mujoco_cmu.md`), then check the IK Retargeter preview shows
   both source and target standing upright on the floor before batch
   retargeting (`auto_align_all_bones` if not — see that doc's step 5).

### 2. Capsule/mesh proportions (only if visually broken)

`build_armature_fbx.py`'s capsule builder now includes real hemispherical
caps, so this shouldn't need touching for a new model. If you do see a body
part visibly detached from its neighbor after building, it's almost always
a short/wide geom (small `half_len` relative to `radius`) — check that
`_make_mesh_data`'s capsule case is being hit (not silently falling through
for an unhandled `geom_type`) rather than assuming a new bug.

### 3. CSV pairing file

Just a new `csv/<name>_retarget.csv` with `target_body,source_anim` rows
pointing at the new model's body name(s) and whichever source AMASS clips
you want to retarget onto it — no code changes.

## Quick checklist for a new model

1. `export_skeleton_json.py --input mjcf/<new>.xml` → JSON.
2. `build_armature_fbx.py` → FBX. Render a quick T-pose check locally in
   Blender before touching UE at all (see `pipeline_mujoco_cmu.md`'s
   gotchas — cheaper to catch a mesh/proportion issue here).
3. `import_batch.py` into a fresh `/Game/BodyModels/<Name>` pool dir.
4. Write a `create_<name>_ik_rig.py` (copy `create_cmu_ik_rig.py`, swap in
   the new bone names per the chain checklist above).
5. `create_cmu_ik_retargeter.py <source_rig> <new_rig> <output_path>` (no
   changes needed, just new arguments).
6. Confirm upright alignment in the IK Retargeter preview.
7. New `csv/<name>_retarget.csv`, then `retarget_batch.py` with the new
   rig/retargeter/CSV paths.
8. `export_anim_dir_fbx.py` + `render_videos.py`, same as before.
