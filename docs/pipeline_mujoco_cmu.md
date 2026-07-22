# End-to-End Pipeline: MuJoCo Humanoid (`.xml`) → BEDLAM2_Retargeting -> MuJoCo qpos

This document records the full command sequence used to take a MuJoCo/
dm_control humanoid defined in an MJCF XML file,
build a matching Unreal Engine skeletal mesh for it, retarget AMASS/SMPL-X
motion onto it, and save it into MuJuCo qpos.

Two repos are involved:
- `mujoco_humanoid_retargeting` — MJCF, MuJoCo→JSON dump,
  Blender armature/FBX builder (no UE dependency).
- `bedlam2_retargeting` (this repo) — UE project, IK Rig/IK
  Retargeter setup, batch retargeting, FBX export, local video rendering.

See `pipeline_new_mujoco_model.md` for what to change if you switch to a
**different** MJCF model.

## 0. Environment

| Component | Path / version |
|---|---|
| Unreal Engine | 5.4, `/Users/Shared/Epic Games/UE_5.4/Engine/Binaries/Mac/UnrealEditor-Cmd` |
| Blender | 4.2.22 LTS, `/Applications/Blender.app/Contents/MacOS/Blender` |

## 1. Dump the MuJoCo rest-pose skeleton to JSON

## 2. Build the Blender armature + mesh, export FBX

## 3. Import into Unreal Engine

**First-time import** (no existing asset yet) — uses the repo's batch
importer, which skips any destination directory that already exists:

```bash
cd bedlam2_retargeting/retargeting/Content/Python
  import_batch.py \
  --input_dir /Users/coconut/mujoco_humanoid_retargeting/assets \
  --output_dir /Game/BodyModels/CMU \
  --num_batches 1 --processes 1
```

This creates `/Game/BodyModels/CMU/bodies/humanoid_CMU/humanoid_CMU`
(SkeletalMesh) and `..._Skeleton` (Skeleton).

**Reimporting after mesh/armature changes** (same bone names/hierarchy,
just updated geometry or bone roll) — updates the SkeletalMesh in place and
reuses the existing Skeleton, so `CMU_IKRig` / `CMU_IKRetargeter` / already
baked AnimSequences don't need to be rebuilt:

```bash
"/Users/Shared/Epic Games/UE_5.4/Engine/Binaries/Mac/UnrealEditor-Cmd" \
  /Users/coconut/bedlam2_retargeting/retargeting/retargeting.uproject \
  -run=pythonscript -script="/Users/coconut/bedlam2_retargeting/retargeting/Content/Python/reimport_skeletal_mesh.py \
  /Users/coconut/mujoco_humanoid_retargeting/assets/humanoid_CMU.fbx \
  /Game/BodyModels/CMU/bodies/humanoid_CMU humanoid_CMU"
```

## 4. Build the custom IK Rig

Defines retarget chains on the CMU skeleton, named to match `smplx_IKRig`'s
chain names 1:1 (`root`/`Spine`/`neck`/`head`/`LeftLeg`/`RightLeg`/
`LeftClavicle`/`RightClavicle`/`LeftArm`/`RightArm`) so the IK Retargeter
can auto-map by exact name.

```bash
"/Users/Shared/Epic Games/UE_5.4/Engine/Binaries/Mac/UnrealEditor-Cmd" \
  /Users/coconut/bedlam2_retargeting/retargeting/retargeting.uproject \
  -run=pythonscript -script="/Users/coconut/bedlam2_retargeting/retargeting/Content/Python/create_cmu_ik_rig.py \
  /Game/BodyModels/CMU/bodies/humanoid_CMU/humanoid_CMU /Game/BodyModels/CMU/CMU_IKRig"
```

> **Gotcha**: don't add a `"root"` chain on top of the bone already used as
> `retarget_root_bone` unless your skeleton has a *separate* static bone for
> it (like `smplx_IKRig`'s `root` vs `pelvis`). Doing so double-applies
> rotation to that bone (FK chain rotation + Retarget Root Settings) and
> makes the whole body tip over.

## 5. Build the IK Retargeter

Source = `smplx_IKRig`, target = `CMU_IKRig`, chains auto-mapped by exact
name, IK disabled per chain (FK-only retargeting — this repo's convention,
since goals are never initialized from a live preview pose here).

```bash
"/Users/Shared/Epic Games/UE_5.4/Engine/Binaries/Mac/UnrealEditor-Cmd" \
  /Users/coconut/bedlam2_retargeting/retargeting/retargeting.uproject \
  -run=pythonscript -script="/Users/coconut/bedlam2_retargeting/retargeting/Content/Python/create_cmu_ik_retargeter.py \
  /Game/BodyModels/Smplx/smplx_IKRig /Game/BodyModels/CMU/CMU_IKRig /Game/BodyModels/CMU/CMU_IKRetargeter"
```

If, after this, the target doesn't stand upright relative to the source in
the IK Retargeter preview, align bones (equivalent to the "Auto Align All
Bones" toolbar button):

```python
# via inspect_retargeter.py-style script, or the GUI toolbar button
controller.auto_align_all_bones(unreal.RetargetSourceOrTarget.TARGET)
```

## 6. Batch retarget

CSV format: `target_body,source_anim` header + one row per pair (see
`csv/cmu_retarget.csv`). **Must** run with the interactive Editor
(`GUI_OFF = False` in `retarget_batch.py`) — the commandlet crashes inside
`IKRetargetBatchOperation.DuplicateAndRetarget`. This is slow (opens a full
Editor window) but correct; don't flip `GUI_OFF` to fix that.

```bash
cd /Users/coconut/bedlam2_retargeting/retargeting/Content/Python
python3 retarget_batch.py \
  --pool_dir /Game/BodyModels/CMU \
  --csv_path_retargeting /Users/coconut/bedlam2_retargeting/csv/cmu_retarget.csv \
  --ik_retargeter_path /Game/BodyModels/CMU/CMU_IKRetargeter \
  --source_ik_rig_path /Game/BodyModels/Smplx/smplx_IKRig \
  --target_ik_rig_path /Game/BodyModels/CMU/CMU_IKRig \
  --num_batches 1 --processes 1
```

Output: `/Game/BodyModels/CMU/retargeting/cmu_retarget/<body>+<source_anim>_Anim`.

## 7. Export retargeted clips to FBX (with mesh)

```bash
"/Users/Shared/Epic Games/UE_5.4/Engine/Binaries/Mac/UnrealEditor-Cmd" \
  /Users/coconut/bedlam2_retargeting/retargeting/retargeting.uproject \
  -run=pythonscript -script="/Users/coconut/bedlam2_retargeting/retargeting/Content/Python/export_anim_dir_fbx.py \
  /Game/BodyModels/CMU/retargeting/cmu_retarget /Users/coconut/bedlam2_retargeting/output/cmu_fbx"
```

## 8. Render videos locally

```bash
cd /Users/coconut/bedlam2_retargeting
python3 render_videos.py \
  --input_dir output/cmu_fbx \
  --output_dir output/cmu_videos \
  --fps 30
```

## 9. Convert the retargeted FBX to MuJoCo `qpos` (optional)

If you want to play the retargeted motion back *inside MuJoCo itself*
(rather than just as a rendered video), convert the exported FBX from step 7
into a `(nframes, nq)` qpos array. This is a separate, two-stage tool in the
`mujoco_humanoid_retargeting` repo — see that repo's `scripts/` directory.

**Stage 1 (Blender): extract per-frame world pose.** Run once for the
retargeted clip, and once (ever, reusable across all clips) for the static
T-pose armature — the T-pose extraction measures each bone's fixed
Blender-vs-MuJoCo orientation offset, needed to correct stage 2's output.

```bash
/Applications/Blender.app/Contents/MacOS/Blender --background --python /Users/coconut/mujoco_humanoid_retargeting/scripts/extract_fbx_pose.py -- \
  --fbx /Users/coconut/mujoco_humanoid_retargeting/fbx/humanoid_CMU+Female_LiftBox_subj_Anim.fbx \
  --output /Users/coconut/mujoco_humanoid_retargeting/fbx/output/pose.json

/Applications/Blender.app/Contents/MacOS/Blender --background --python /Users/coconut/mujoco_humanoid_retargeting/scripts/extract_fbx_pose.py -- \
  --fbx /Users/coconut/mujoco_humanoid_retargeting/assets/humanoid_CMU/humanoid_CMU.fbx \
  --output /Users/coconut/mujoco_humanoid_retargeting/fbx/output/tpose.json
```

**Stage 2 (MuJoCo venv): assemble qpos.**

```bash
uv run scripts/fbx_pose_to_qpos.py \                                         
  --mjcf mjcf/humanoid_CMU.xml \
  --skeleton-json assets/humanoid_CMU/humanoid_CMU_skeltion.json \
  --pose-json fbx/output/pose.json \ 
  --tpose-json fbx/output/tpose.json \ 
  --output fbx/output/humanoid_CMU+Female_LiftBox_subj_qpos.npz 
```

**Verify** (kinematic playback, no physics — just confirms the conversion
looks right before attempting real simulation):

```bash
python3 scripts/render_qpos.py \
  --mjcf mjcf/humanoid_CMU.xml \
  --qpos-npz output/qpos/humanoid_CMU+Female_LiftBox_subj_qpos.npz \
  --output fbx/output/humanoid_CMU+Female_LiftBox_subj_qpos_playback.mp4
```

This is pure `mj_forward` per frame — not a physics rollout. To actually
drive `mj_step` (gravity, contacts, actuator torque) toward this qpos
trajectory, use it as the PD-controller target the way
`test_qpos_physics_playback.py` already does (root has no actuator, so it's
a target the physics tries to track, not a guaranteed outcome).

Known gotchas specific to this conversion:
- **Don't mix orientation sources.** `rest_local` and `current_local` must
  both come from Blender's own extraction (T-pose vs. animated), not one
  from the MuJoCo JSON and one from Blender — they use different per-bone
  orientation conventions that don't cancel if mixed.
- **Quaternion double-cover**: single-axis joints (1-DOF hinges like
  tibia/wrist) need the delta quaternion canonicalized to `w >= 0` before
  extracting the twist angle, or the result can jump by a spurious ±360°.
- **Bodies merged away in the armature** (zero offset from parent, e.g.
  `lowerback`) have no bone of their own to read a pose from — their joint
  angles are left at 0, with their rotation folded into their resolved
  child's joint instead (printed as a warning).

## Known gotchas (fixed this session, worth remembering)

- **Scale**: Blender's meter→FBX-centimeter unit conversion leaves a
  residual `scale=100` on bones unless coordinates are pre-scaled in
  `build_armature_fbx.py` (`WORLD_SCALE`) *and* `scene.unit_settings.scale_length`
  is declared to match. UE's bind pose tolerates the un-fixed version; the
  IK Retargeter's FK bake does not (collapses every non-root bone).
- **IK must be disabled per chain** in the IK Retargeter — it defaults to
  enabled, and un-initialized goals collapse the pose.
- **`auto_map_chains` must use `EXACT`, not `FUZZY`** — fuzzy name matching
  can silently map a chain to a differently-cased, nonexistent name.
- **Don't double up the retarget-root bone with its own FK chain** (see
  step 4's gotcha) — causes the whole body to tip over.
- **Capsule geoms need real hemispherical caps**, not a flat-capped
  cylinder (`build_armature_fbx.py`'s `_make_mesh_data`) — otherwise short,
  wide segments (head) visibly detach from their neighbor.
