"""Stage 1 (this repo's own venv, NOT Blender) of converting a MuJoCo `qpos`
array into an animated FBX -- see `world_pose_to_fbx.py` for stage 2, the
Blender half. Split the same way `extract_fbx_pose.py` (Blender) /
`fbx_pose_to_qpos.py` (venv) are split for the reverse direction: Blender's
bundled Python has no `mujoco` package, so all MuJoCo work has to happen
outside it.

For every frame in a qpos `.npz` (key "qpos", shape (nframes, nq)), runs
`mj_forward` and records every body's world position/orientation -- exactly
the per-frame quantities `extract_fbx_pose.py` records when reading an
*existing* animated FBX, just computed directly from qpos via MuJoCo's own
forward kinematics instead of extracting them from Blender. Writes the same
`{fps, frame_start, frame_end, num_frames, frames: [...]}` JSON shape as
`extract_fbx_pose.py`'s `pose.json`, so stage 2 (and the existing
`extract_fbx_pose.py`/`fbx_pose_to_qpos.py` round-trip tooling, used to
validate this new direction) can all consume/produce a compatible format.

Run with (this repo's own venv, NOT inside Blender):
    python3 scripts/qpos_to_world_pose.py --qpos_npz <path.npz> \
        --mjcf mjcf/robot.xml --output <path.json> [--fps 30]
"""
import argparse
import json

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--qpos_npz", required=True)
    parser.add_argument("--mjcf", default="mjcf/robot.xml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    qpos = np.load(args.qpos_npz)["qpos"]
    num_frames = qpos.shape[0]

    model = mujoco.MjModel.from_xml_path(args.mjcf)
    data = mujoco.MjData(model)

    body_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(model.nbody)
    ]

    frames = []
    for frame_idx in range(num_frames):
        data.qpos[:] = qpos[frame_idx]
        mujoco.mj_forward(model, data)
        entry = {}
        for body_id, name in enumerate(body_names):
            if name == "world":
                continue
            entry[name] = {
                "pos": data.xpos[body_id].tolist(),
                "quat_wxyz": data.xquat[body_id].tolist(),
            }
        frames.append(entry)

    out = {
        "fps": args.fps,
        "frame_start": 0,
        "frame_end": num_frames - 1,
        "num_frames": num_frames,
        "frames": frames,
    }
    with open(args.output, "w") as f:
        json.dump(out, f)
    print(f"Wrote {num_frames} frames -> {args.output}")


if __name__ == "__main__":
    main()
