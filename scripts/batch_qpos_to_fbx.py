"""Batch-convert every qpos .npz under --input_dir into an animated FBX, via
qpos_to_world_pose.py (stage 1, venv) + world_pose_to_fbx.py (stage 2,
Blender), mirroring bedlam2_retargeting/make_fbx_files.py's
Blender-subprocess-pool pattern (one subprocess pair per file, parallelized
with a multiprocessing.Pool).

Run with (this repo's own venv, NOT inside Blender):
    python3 scripts/batch_qpos_to_fbx.py --input_dir <robotmotion_dir> \
        --output_dir <fbx_output_dir> --blender_app_path <path/to/Blender> \
        [--mjcf mjcf/robot.xml] [--skeleton_json assets/robot/robot.json] \
        [--armature_fbx assets/robot/robot.fbx] [--fps 30] [--processes N]
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time
from multiprocessing import Pool
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def worker(npz_path, out_fbx_path, mjcf, skeleton_json, armature_fbx, blender_app_path, fps):
    with tempfile.TemporaryDirectory() as tmpdir:
        world_pose_path = Path(tmpdir) / "world_pose.json"

        stage1 = subprocess.run(
            [sys.executable, str(REPO_ROOT / "scripts" / "qpos_to_world_pose.py"),
             "--qpos_npz", str(npz_path), "--mjcf", mjcf,
             "--output", str(world_pose_path), "--fps", str(fps)],
            cwd=REPO_ROOT, capture_output=True, text=True,
        )
        if stage1.returncode != 0:
            print(f"[stage1 FAILED] {npz_path}\n{stage1.stdout}\n{stage1.stderr}", file=sys.stderr)
            return False

        Path(out_fbx_path).parent.mkdir(parents=True, exist_ok=True)
        stage2 = subprocess.run(
            [blender_app_path, "--background", "--python",
             str(REPO_ROOT / "scripts" / "world_pose_to_fbx.py"), "--",
             "--world_pose", str(world_pose_path), "--skeleton_json", skeleton_json,
             "--armature_fbx", armature_fbx, "--out_fbx", str(out_fbx_path)],
            cwd=REPO_ROOT, capture_output=True, text=True,
        )
        if stage2.returncode != 0:
            print(f"[stage2 FAILED] {npz_path}\n{stage2.stdout}\n{stage2.stderr}", file=sys.stderr)
            return False

    return True


def worker_args(args):
    return worker(*args)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=Path, required=True,
                         help="Directory to search recursively for qpos .npz files")
    parser.add_argument("--output_dir", type=Path, required=True,
                         help="Destination directory for output .fbx files")
    parser.add_argument("--blender_app_path", required=True)
    parser.add_argument("--mjcf", default="mjcf/robot.xml")
    parser.add_argument("--skeleton_json", default="assets/robot/robot.json")
    parser.add_argument("--armature_fbx", default="assets/robot/robot.fbx")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--processes", type=int, default=os.cpu_count())
    args = parser.parse_args()

    npz_paths = sorted(args.input_dir.rglob("*.npz"))
    print(f"Found {len(npz_paths)} .npz files under {args.input_dir}")

    tasklist = []
    seen_stems = set()
    for npz_path in npz_paths:
        stem = npz_path.stem
        if stem in seen_stems:
            print(f"WARNING: duplicate clip name '{stem}' ({npz_path}) -- skipping, "
                  f"output naming assumes stems are globally unique.", file=sys.stderr)
            continue
        seen_stems.add(stem)
        out_fbx_path = args.output_dir / f"{stem}.fbx"
        tasklist.append((
            str(npz_path), str(out_fbx_path), args.mjcf, args.skeleton_json,
            args.armature_fbx, args.blender_app_path, args.fps,
        ))

    print(f"Starting pool with {args.processes} processes\n", file=sys.stderr)
    start_time = time.perf_counter()
    with Pool(args.processes) as pool:
        results = pool.map(worker_args, tasklist)

    num_ok = sum(results)
    num_failed = len(results) - num_ok
    elapsed = time.perf_counter() - start_time
    print(f"\nConverted {num_ok}/{len(tasklist)} files ({num_failed} failed) in {elapsed:.2f}s")


if __name__ == "__main__":
    main()
