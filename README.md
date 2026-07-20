# mujoco_humanoid_retargeting

## Structure

- `mjcf/` — MuJoCo humanoid models (`humanoid_CMU.xml` + `stocky`/`tall` variants) and their shared `common/` includes.
- `assets/` — derived armature assets (`humanoid_CMU.fbx`, `humanoid_skeleton.json`).
- `scripts/` — pipeline scripts (skeleton export, FBX armature build, scaling, load/stability/render checks).
- `renders/` — snapshot images produced by the render/check scripts.
- `SMPL_X/` — SMPL-X body shapes (`body/`), motion capture (`animations/`), and cross-body retargeted motions (`retargeting/`).

Scripts assume the repo root as the working directory, e.g. `.venv/bin/python scripts/check_load.py`.
