"""Drive humanoid.xml with REAL physics (mj_step: gravity, contacts,
actuator forces) instead of the pure-kinematic mj_forward playback in
test_qpos_playback.py.

A qpos array by itself doesn't do this -- the model isn't a `position`
actuator rig, its 21 actuators are `motor` (torque) actuators with
ctrlrange [-1,1]. So a target qpos trajectory can only drive real physics
indirectly, via a PD controller computing torque-like ctrl signals each
step:

    ctrl = kp * (target_qpos - qpos) - kd * qvel   (clipped to [-1, 1])

The free joint (root, 7 qpos / 6 qvel) has no actuator at all -- it isn't
controlled directly, its motion is whatever gravity/contacts/reaction
forces from the actuated legs produce. That's the difference from kinematic
playback: the reference trajectory is only a *target* the body tries (and
may fail) to track, not the ground truth state.

Run with:
    .venv/bin/python scripts/test_qpos_physics_playback.py
"""
import imageio.v2 as imageio
import mujoco
import numpy as np

from humanoid_poses import POSES


def slerp(a, b, alpha):
    dot = np.dot(a, b)
    if dot < 0:
        b = -b
        dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    theta = np.arccos(dot) * alpha
    rel = b - a * dot
    norm = np.linalg.norm(rel)
    if norm > 1e-9:
        rel /= norm
    return a * np.cos(theta) + rel * np.sin(theta)


def reference_qpos(t, seconds_per_segment, waypoints):
    """Piecewise slerp/lerp through the waypoints, indexed by elapsed sim time."""
    seg = min(int(t // seconds_per_segment), len(waypoints) - 2)
    alpha = np.clip((t - seg * seconds_per_segment) / seconds_per_segment, 0.0, 1.0)
    a, b = waypoints[seg], waypoints[seg + 1]
    out = (1 - alpha) * a + alpha * b
    out[3:7] = slerp(a[3:7], b[3:7], alpha)
    return out


def actuator_addresses(model):
    """Map each actuator to the qpos/qvel index of the single hinge joint it drives."""
    qpos_adr = np.array([model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(model.nu)])
    dof_adr = np.array([model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)])
    return qpos_adr, dof_adr


def main():
    xml_path = "mjcf/humanoid.xml"
    output_path = "renders/humanoid_qpos_physics_playback.mp4"
    fps = 25.0  # divides the model's 0.005s timestep evenly (8 substeps/frame)
    seconds_per_segment = 1.5
    pose_names = ["squat", "stand_on_left_leg", "prone", "supine", "squat"]
    kp, kd = 20.0, 2.0

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=480, width=640)

    waypoints = [POSES[name] for name in pose_names]
    total_duration = seconds_per_segment * (len(waypoints) - 1)
    substeps_per_frame = round(1.0 / (fps * model.opt.timestep))
    qpos_adr, dof_adr = actuator_addresses(model)

    data.qpos[:] = waypoints[0]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    with imageio.get_writer(output_path, fps=fps) as writer:
        while data.time < total_duration:
            for _ in range(substeps_per_frame):
                target = reference_qpos(data.time, seconds_per_segment, waypoints)
                error = target[qpos_adr] - data.qpos[qpos_adr]
                velocity = data.qvel[dof_adr]
                data.ctrl[:] = np.clip(kp * error - kd * velocity, -1.0, 1.0)
                mujoco.mj_step(model, data)  # real physics: gravity + contacts + actuator torque
                if not np.all(np.isfinite(data.qpos)):
                    raise RuntimeError(f"unstable (NaN) at t={data.time:.3f}s")

            renderer.update_scene(data)
            writer.append_data(renderer.render())

    print(f"wrote {output_path}: {total_duration}s @ {fps}fps, PD-tracked physics "
          f"(kp={kp}, kd={kd}), final root height = {data.qpos[2]:.3f}")


if __name__ == "__main__":
    main()
