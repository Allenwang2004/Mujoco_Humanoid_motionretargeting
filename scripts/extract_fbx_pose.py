"""Blender script: dump the per-frame WORLD position + orientation of every
bone in a retargeted animation FBX (e.g. one exported from Unreal Engine via
`export_anim_dir_fbx.py`) to a plain JSON file.

This is stage 1 of converting a retargeted UE animation into MuJoCo `qpos`
(see `fbx_pose_to_qpos.py` for stage 2). Done in Blender rather than in UE
Python because Blender's imported armature is Z-up/right-handed, matching
MuJoCo's own convention -- reading world transforms here avoids UE's
left-handed coordinate convention entirely (no manual axis-flip needed).

Run with:
    blender --background --python scripts/extract_fbx_pose.py -- \
        --fbx <retargeted.fbx> --output <out.json>
"""
import argparse
import json
import sys

import bpy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fbx", required=True)
    parser.add_argument("--output", required=True)
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=args.fbx)

    armature_obj = next(o for o in bpy.data.objects if o.type == "ARMATURE")

    scene = bpy.context.scene
    frame_starts, frame_ends = [], []
    for action in bpy.data.actions:
        start, end = action.frame_range
        frame_starts.append(start)
        frame_ends.append(end)
    frame_start = int(min(frame_starts)) if frame_starts else scene.frame_start
    frame_end = int(max(frame_ends)) if frame_ends else scene.frame_end
    fps = scene.render.fps / scene.render.fps_base

    frames = []
    for f in range(frame_start, frame_end + 1):
        scene.frame_set(f)
        bpy.context.view_layer.update()
        entry = {}
        for pbone in armature_obj.pose.bones:
            # pbone.matrix is the bone's pose matrix in armature space; the
            # armature object itself is always constructed/imported at the
            # world origin with identity rotation, so armature space *is*
            # world space here.
            mat = armature_obj.matrix_world @ pbone.matrix
            loc = mat.to_translation()
            quat = mat.to_quaternion()
            entry[pbone.name] = {
                "pos": [loc.x, loc.y, loc.z],
                "quat_wxyz": [quat.w, quat.x, quat.y, quat.z],
            }
        frames.append(entry)

    with open(args.output, "w") as fh:
        json.dump({
            "fps": fps,
            "frame_start": frame_start,
            "frame_end": frame_end,
            "num_frames": len(frames),
            "frames": frames,
        }, fh)
    print(f"Wrote {len(frames)} frames ({frame_start}-{frame_end} @ {fps:.2f}fps) -> {args.output}")


if __name__ == "__main__":
    main()
