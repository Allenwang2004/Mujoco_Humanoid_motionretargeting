"""Simulate a MuJoCo humanoid model for a fixed duration and save the render
as an mp4, to sanity-check that a given MJCF can actually be stepped and
rendered end-to-end before wiring in real motion data.

Run with:
    .venv/bin/python scripts/render_video.py mjcf/humanoid_CMU.xml renders/humanoid_CMU.mp4
"""
import argparse

import imageio.v2 as imageio
import mujoco
import numpy as np


def render_video(xml_path, output_path, duration, fps, width, height):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    renderer = mujoco.Renderer(model, height=height, width=width)
    frame_interval = 1.0 / fps
    next_frame_time = 0.0

    with imageio.get_writer(output_path, fps=fps) as writer:
        while data.time < duration:
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)):
                raise RuntimeError(f"unstable (NaN) at t={data.time:.3f}s")
            if data.time >= next_frame_time:
                renderer.update_scene(data)
                writer.append_data(renderer.render())
                next_frame_time += frame_interval

    print(f"{xml_path}: wrote {output_path} ({duration}s @ {fps}fps, "
          f"final root height = {data.qpos[2]:.3f})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml_path")
    parser.add_argument("output_path")
    parser.add_argument("--duration", type=float, default=3.0, help="seconds")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()
    render_video(args.xml_path, args.output_path, args.duration, args.fps,
                 args.width, args.height)


if __name__ == "__main__":
    main()
