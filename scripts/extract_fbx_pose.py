"""Blender script: batch-dump the per-frame WORLD position + orientation of
every bone in each retargeted animation FBX (e.g. exported from Unreal Engine
via `export_anim_dir_fbx.py`) under `<folder>/fbx/`, plus a shared T-pose
reference, to `<folder>/fbx_pose/<clip>/{pose.json,tpose.json}`.

This is stage 1 of converting a retargeted UE animation into MuJoCo `qpos`
(see `fbx_pose_to_qpos.py` for stage 2). Done in Blender rather than in UE
Python because Blender's imported armature is Z-up/right-handed, matching
MuJoCo's own convention -- reading world transforms here avoids UE's
left-handed coordinate convention entirely (no manual axis-flip needed).

For every `<clip>.fbx` in `<folder>/fbx/`, writes:
    <folder>/fbx_pose/<clip>/pose.json   -- per-frame world bone poses for that clip
    <folder>/fbx_pose/<clip>/tpose.json  -- world bone poses of the static T-pose FBX
                                             (identical in every clip folder -- stage 2
                                             expects one alongside each pose.json)

Run with:
    blender --background --python scripts/extract_fbx_pose.py -- \
        --folder <folder> --tpose-fbx <tpose.fbx>
"""
import argparse
import json
import sys
from pathlib import Path

import bpy


def extract_world_bone_poses(fbx_path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(fbx_path))

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

    return {
        "fps": fps,
        "frame_start": frame_start,
        "frame_end": frame_end,
        "num_frames": len(frames),
        "frames": frames,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", required=True,
                         help="Directory containing an fbx/ subfolder of animation "
                              "clips; output is written to <folder>/fbx_pose/<clip>/")
    parser.add_argument("--tpose-fbx", required=True,
                         help="Static T-pose armature FBX, extracted once and copied "
                              "into every clip's output folder as tpose.json")
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    folder = Path(args.folder)
    fbx_dir = folder / "fbx"
    out_dir = folder / "fbx_pose"
    fbx_paths = sorted(fbx_dir.glob("*.fbx"))
    if not fbx_paths:
        print(f"No .fbx files found in {fbx_dir}")
        return

    tpose_data = extract_world_bone_poses(args.tpose_fbx)

    for fbx_path in fbx_paths:
        clip_dir = out_dir / fbx_path.stem
        clip_dir.mkdir(parents=True, exist_ok=True)

        pose_data = extract_world_bone_poses(fbx_path)
        with open(clip_dir / "pose.json", "w") as fh:
            json.dump(pose_data, fh)
        with open(clip_dir / "tpose.json", "w") as fh:
            json.dump(tpose_data, fh)

        print(f"Wrote {pose_data['num_frames']} frames "
              f"({pose_data['frame_start']}-{pose_data['frame_end']} @ {pose_data['fps']:.2f}fps) "
              f"-> {clip_dir}")


if __name__ == "__main__":
    main()
