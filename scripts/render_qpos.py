"""Render a qpos array (from `fbx_pose_to_qpos.py`) as a kinematic MuJoCo
playback video -- pure forward kinematics (`mj_forward` per frame), no
physics (gravity/contacts/actuators are not simulated). Useful to visually
verify a conversion before attempting a real `mj_step` physics rollout (see
`test_qpos_physics_playback.py` for that, which needs a PD controller since
this is only a kinematic target, not a dynamically consistent trajectory).

Run with (this repo's own venv):
    python3 scripts/render_qpos.py \
        --mjcf mjcf/humanoid_CMU.xml \
        --qpos-npz output/qpos/humanoid_CMU+Female_LiftBox_subj_qpos.npz \
        --output renders/humanoid_CMU+Female_LiftBox_subj_qpos_playback.mp4
"""
import argparse

import imageio.v2 as imageio
import mujoco
import numpy as np


def execute(mjcf_path, qpos_npz_path, output_path, width, height, azimuth):
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, camera)
    camera.azimuth = azimuth

    npz = np.load(qpos_npz_path)
    qpos = npz["qpos"]
    fps = float(npz["fps"])

    with imageio.get_writer(output_path, fps=fps) as writer:
        for frame in qpos:
            data.qpos[:] = frame
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            writer.append_data(renderer.render())

    print(f"Wrote {len(qpos)} frames @ {fps}fps -> {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", required=True)
    parser.add_argument("--qpos-npz", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--azimuth",
        type=float,
        default=-90,
        help="Camera azimuth in degrees (MuJoCo default free camera is 90; "
        "this defaults to -90, i.e. rotated 180 deg, to face the model front-on).",
    )
    args = parser.parse_args()
    execute(args.mjcf, args.qpos_npz, args.output, args.width, args.height, args.azimuth)
