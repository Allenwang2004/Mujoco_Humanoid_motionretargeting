# Retargeting `robot.xml` motion (qpos) onto other robot variants

This documents two new ways to take a `qpos` trajectory recorded for
`mjcf/robot.xml` (e.g. the 540 clips in `bedlam2_retargeting/robotmotion/`,
each `(nframes, 76)`, no FBX/AMASS involved at all) and produce the
equivalent motion for a same-topology variant like `robot_child.xml` or
`robot_elderly.xml`. It complements `pipeline_mujoco_cmu.md` (which goes the
other way: AMASS/SMPL-X motion -> UE IK Retargeter -> a MuJoCo skeleton) --
this is robot-to-robot, and the source is already MuJoCo-native qpos, not an
FBX.

## Bug fixed first: `build_armature_fbx.py` silently mis-placed bones at branch points

While validating the new qpos->FBX conversion below, `robot.fbx`'s rest pose
turned out to be wrong for any body with **more than one child bone**
(`Pelvis` -> `L_Hip`/`R_Hip`/`Torso`, and `Chest` -> `Neck`/`L_Thorax`/`R_Thorax`).
Root cause: `create_bone()` set `bone.use_connect = True` unconditionally for
every child, but Blender's `use_connect` is a *live* constraint (the child's
head is forced to always equal the parent's tail, not just snapped once) --
and a parent's tail can only point at one location (`children_of[parent][0]`,
per the tail-assignment pass). Every other sibling still had
`use_connect=True`, so it silently collapsed onto that same point, discarding
its own correct head position from the skeleton JSON.

Confirmed directly: importing the old `robot.fbx` fresh (no posing at all)
and reading each bone's rest-pose world position showed `R_Hip`/`Torso` both
sitting exactly at `L_Hip`'s position, and `L_Thorax`/`R_Thorax` both sitting
at `Neck`'s position. This is a pre-existing defect in the asset that
`bedlam2_retargeting`'s existing `Robot` skeletal mesh and its 61 completed
AMASS->robot retargeted clips were built from -- not something introduced by
the new code below.

**Fix** (`scripts/build_armature_fbx.py`): only the body that's actually
`children_of[parent][0]` (the one the tail-assignment pass points at) gets
`use_connect = True`; every other sibling gets `use_connect = False`, which
keeps its explicit `bone.head` position instead of snapping it away.

`robot.fbx`, `robot_child.fbx`, and `robot_elderly.fbx` have all been
regenerated with the fix (`assets/robot/*.fbx`, same command as always --
see `pipeline_new_mujoco_model.md`'s quick checklist). **Not yet done**:
reimporting the corrected `robot.fbx` into `bedlam2_retargeting`'s existing
`/Game/BodyModels/Robot/bodies/robot` SkeletalMesh, which would also mean the
61 already-completed retargeted clips there should arguably be redone (their
baked animation reflects retargeting against the old, slightly wrong rest
pose). Flagged for a separate decision -- not touched yet.

## Option A -- direct qpos-space retargeting (no Blender/UE at all)

`scripts/qpos_retarget_simple.py`. Valid specifically because `robot.xml`,
`robot_child.xml`, and `robot_elderly.xml` share the *exact* same body/joint
topology (confirmed via direct MuJoCo inspection: identical body names,
joint types, axes, and declaration order across all three; only `body_pos`,
i.e. segment lengths/offsets, differs, non-uniformly -- e.g. `robot_child`'s
Pelvis rest height is 0.611x `robot`'s). Since every non-root qpos value is
already a joint-local hinge angle, it carries over unchanged; only the free
joint root (qpos[0:3], the pelvis's world position) needs rescaling, by the
ratio of each model's own rest-pose (qpos=0) pelvis height, so a smaller/
larger skeleton's feet stay grounded instead of floating or clipping.

```bash
# single clip
python3 scripts/qpos_retarget_simple.py \
  --input_npz <robot_clip.npz> --output_npz <robot_child_clip.npz> \
  --source_skeleton_json assets/robot/robot.json \
  --target_skeleton_json assets/robot/robot_child.json

# batch (used for all 540 robotmotion clips)
python3 scripts/qpos_retarget_simple.py \
  --input_dir /path/to/robotmotion --output_dir output/robot_child_qpos_simple
```

Verified by rendering the same frame of a source clip on `robot.xml` and the
retargeted clip on `robot_child.xml` side by side (manual camera framing,
`render_qpos.py`'s auto camera only special-cases `robot.xml`'s stem) --
identical pose, hands/feet grounded correctly, just proportionally smaller.

This does **not** do proper IK (no per-limb foot/hand placement correction --
only the root height is corrected as a single global scale, not each leg's
actual reach). For that, use Option B.

## Option B -- qpos -> animated FBX -> Unreal Engine IK Retargeter

For when an actual FBX/UE animation asset is needed (rendering, game engine,
or true cross-topology IK retargeting), not just a qpos array. Two new
scripts, split the same way `extract_fbx_pose.py` (Blender) /
`fbx_pose_to_qpos.py` (venv) are split for the reverse direction -- Blender's
bundled Python has no `mujoco` package, so all MuJoCo work has to happen
outside it:

- **`scripts/qpos_to_world_pose.py`** (venv, stage 1): runs `mj_forward` per
  frame and records every body's world position/orientation directly from
  MuJoCo -- the same quantities `extract_fbx_pose.py` records when reading an
  *existing* FBX, just computed instead of extracted. Writes the same
  `{fps, frame_start, frame_end, num_frames, frames: [...]}` JSON shape as
  `extract_fbx_pose.py`'s `pose.json`.
- **`scripts/world_pose_to_fbx.py`** (Blender, stage 2): poses and keyframes
  `robot.fbx`'s armature from that JSON, one frame at a time, and exports an
  animated FBX. Per body, the only conversion needed is the same fixed
  per-bone conjugation offset `C(body)` that `fbx_pose_to_qpos.py`'s
  `bone_frame_offsets()` computes (Blender's bone-local frame != MuJoCo's
  `body_quat` convention), applied in the opposite direction -- no need to
  decompose/recompose per-joint hinge angles the way `fbx_pose_to_qpos.py`
  does, since `mj_forward` already gives fully-composed world transforms.
- **`scripts/batch_qpos_to_fbx.py`**: drives both stages over a directory of
  `.npz` files with a `multiprocessing.Pool` of Blender subprocesses, mirroring
  `bedlam2_retargeting/make_fbx_files.py`'s pattern.

```bash
# single clip
python3 scripts/qpos_to_world_pose.py --qpos_npz <clip.npz> --mjcf mjcf/robot.xml \
  --output /tmp/pose.json --fps 30
blender --background --python scripts/world_pose_to_fbx.py -- \
  --world_pose /tmp/pose.json --skeleton_json assets/robot/robot.json \
  --armature_fbx assets/robot/robot.fbx --out_fbx <clip.fbx>

# batch
python3 scripts/batch_qpos_to_fbx.py \
  --input_dir /path/to/robotmotion --output_dir output/robot_fbx \
  --blender_app_path /Applications/Blender.app/Contents/MacOS/Blender \
  --processes 8
```

Once `robot_child`'s FBX (already built, `assets/robot/robot_child.fbx`) is
imported into `bedlam2_retargeting` as a new body and given its own IK Rig
(reusing `create_robot_ik_rig.py` verbatim -- bone names are identical to
`robot`'s), these per-clip FBX files become the source-animation pool for a
normal `retarget_batch.py` run with `Robot_IKRig` as source and the new
`RobotChild_IKRig` as target. Not done yet -- **paused at 368/540 clips
converted** (`output/robot_fbx/`); rerun `batch_qpos_to_fbx.py` with the same
args to pick up where it left off (existing output files just get
overwritten, not skipped, so it's safe but redundant for the 368 already
done).

### Gotchas found while building this (worth remembering)

- **Never convert a world-space target into armature space via
  `matrix_world.inverted() @ world_matrix`** when the armature object's own
  `matrix_world` is a pure scale (which it is here -- importing `robot.fbx`
  fresh leaves a compensating ~0.01 object-level scale rather than baking
  `build_armature_fbx.py`'s `WORLD_SCALE=100` into the bind-pose numbers
  themselves). Multiplying a rotation+translation matrix by a scale-only
  matrix also scales the rotation submatrix, which Blender's pose solver
  strips back out cleanly for a single unchained bone (e.g. root) but
  compounds into real, growing position error for every bone deeper in a
  chain. Rescale the translation with a plain scalar instead (measured
  directly in armature-space units, see `derive_position_scale()`) and leave
  rotation untouched (already scale-invariant).
- **`bpy.ops.export_scene.fbx`'s `bake_anim_simplify_factor` defaults to
  `1.0`**, not `0.0` -- lossy keyframe simplification, silently drops/alters
  per-frame data. Pass `bake_anim_simplify_factor=0.0` explicitly whenever
  every frame matters (which is always, for this kind of conversion).
- **Validate with world-space bone transforms, not qpos-decomposed joint
  angles.** `fbx_pose_to_qpos.py`'s Euler-angle decomposition
  (`decompose_xyz`) has real branch-selection edge cases near certain
  rotation configurations (observed: a joint's recovered angle jumping by
  ~180 degrees for several frames in the middle of an otherwise-correct
  clip) that are a pre-existing property of that tool, not a sign of a
  conversion bug. Comparing extracted world quaternions/positions directly
  (as `extract_fbx_pose.py` produces them) is the reliable ground truth --
  it matched to sub-micron/sub-0.1-degree precision across every body and
  frame once the two points above were fixed, while the qpos-decomposed
  comparison still showed large spurious diffs from this unrelated
  decomposition issue.
