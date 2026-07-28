"""Stage 2 of converting a retargeted UE animation into MuJoCo `qpos`.

Takes:
  - stage-1 output from `extract_fbx_pose.py` (per-frame world pos/quat of
    every bone in the retargeted animation, in Blender/MuJoCo's shared
    Z-up right-handed convention),
  - the ORIGINAL rest-pose skeleton JSON (from `export_skeleton_json.py`,
    i.e. the qpos=0 world pos/quat of every body -- `assets/robot/robot.json`
    by default),
  - the MJCF the skeleton came from (to query each body's actual joints:
    how many DOFs, which axes, in which order, and where in `qpos` they go).

For each non-root body B with effective parent P (walking up through
zero-offset bodies merged away by `build_armature_fbx.py`, exactly like its
own `resolve_bone_parent`/`has_own_bone`), the joint rotation is:

    rest_local(B)    = inverse(world_quat(P, rest)) * world_quat(B, rest)
    current_local(B) = inverse(world_quat(P, frame)) * world_quat(B, frame)
    delta(B, frame)  = inverse(rest_local(B)) * current_local(B, frame)

`delta` is exactly the composed hinge rotation MuJoCo's own joints for that
body would produce from qpos=0, expressed in the body's own declared local
frame (its `axis="..."` attributes) -- decompose it into that body's actual
joint axes/order to get each hinge's angle.

The root uses a free joint: qpos[0:3]/[3:7] are just its world position
(already in meters) and world quaternion directly, no delta needed.

Known approximation (doesn't currently trigger for `robot.xml`, whose 23
posed bodies all have nonzero offsets from their parent, but kept in case a
future skeleton does): any body with zero position offset from its parent in
the MJCF gets merged away in `build_armature_fbx.py` and has no bone of its
own to read a pose from. Its own joint angles are left at 0; whatever
rotation it contributed in the source motion ends up folded into its
resolved child's joint instead (this is the same approximation the
armature-building step already makes, just made visible here).

Batch-processes every clip folder produced by stage 1 (`extract_fbx_pose.py`):
for each `<folder>/fbx_pose/<clip>/{pose.json,tpose.json}`, writes
`<folder>/qpos/<clip>.npz`.

Targets the MetaMotivo `robot.xml` pipeline: defaults to `mjcf/robot.xml` and
`assets/robot/robot.json` (from `export_skeleton_json.py --input mjcf/robot.xml
--output assets/robot/robot.json`), so 7 free-joint values + 23 bodies x 3
hinge DOFs (x,y,z order) = nq 76.

Run with (this repo's own venv, NOT inside Blender):
    python3 scripts/fbx_pose_to_qpos.py --folder <folder>
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

ZERO_LEN_EPS = 1e-4  # meters; matches build_armature_fbx.py's merge threshold


# ---- quaternion helpers (all wxyz order) -----------------------------------

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_inv(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def decompose_zyx(R):
    """R = Rz(a) @ Ry(b) @ Rx(c); returns (a, b, c)."""
    b = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    a = np.arctan2(R[1, 0], R[0, 0])
    c = np.arctan2(R[2, 1], R[2, 2])
    return a, b, c


def decompose_zx(R):
    """R = Rz(a) @ Rx(c); returns (a, c). (R[2, 0] should be ~0.)"""
    a = np.arctan2(R[1, 0], R[0, 0])
    c = np.arctan2(R[2, 1], R[2, 2])
    return a, c


def decompose_xyz(R):
    """R = Rx(a) @ Ry(b) @ Rz(c); returns (a, b, c)."""
    b = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    a = np.arctan2(-R[1, 2], R[2, 2])
    c = np.arctan2(-R[0, 1], R[0, 0])
    return a, b, c


def single_axis_twist(q, axis):
    """Angle of rotation about `axis` embedded in quaternion q (discards any
    perpendicular swing component).

    q and -q represent the identical rotation (quaternion double cover), but
    this formula is linear in q's components (not quadratic like the matrix
    forms above), so it isn't sign-invariant: picking up "-q" from an
    upstream multiply chain flips w and xyz together and shifts the result by
    a spurious +-360 deg. Canonicalize to w >= 0 first (the physically
    expected joint ranges here are well under 180 deg either way).
    """
    if q[0] < 0:
        q = -q
    w, xyz = q[0], q[1:]
    return 2.0 * np.arctan2(np.dot(xyz, axis), w)


AXIS_LETTER = {(0.0, 0.0, 1.0): "z", (0.0, 1.0, 0.0): "y", (1.0, 0.0, 0.0): "x"}


def decompose_for_axes(delta_quat, axes):
    """axes: list of axis tuples in declaration order. Returns angles in the
    same order, for the patterns that actually occur in humanoid_CMU.xml /
    robot.xml."""
    letters = tuple(AXIS_LETTER[tuple(a)] for a in axes)
    R = quat_to_matrix(delta_quat)
    if letters == ("z", "y", "x"):
        return list(decompose_zyx(R))
    if letters == ("z", "y"):
        a, b, _c = decompose_zyx(R)
        return [a, b]
    if letters == ("z", "x"):
        return list(decompose_zx(R))
    if letters == ("x", "y", "z"):
        return list(decompose_xyz(R))
    if letters == ("x",):
        return [single_axis_twist(delta_quat, np.array([1.0, 0.0, 0.0]))]
    if letters == ("y",):
        return [single_axis_twist(delta_quat, np.array([0.0, 1.0, 0.0]))]
    raise NotImplementedError(
        f"No decomposition implemented for joint axis sequence {letters}. "
        "Add one in decompose_for_axes -- this model was assumed to only "
        "ever use zyx/zy/zx/x/y hinge sequences."
    )


# ---- skeleton merge-resolution (mirrors build_armature_fbx.py, no bpy) -----

def load_bodies(skeleton_json_path):
    with open(skeleton_json_path) as f:
        data = json.load(f)
    bodies = {b["name"]: b for b in data["bodies"]}
    for b in bodies.values():
        if b["parent"] == "world":
            b["parent"] = None
    return bodies


def has_own_bone(bodies, name):
    node = bodies[name]
    if node["parent"] is None:
        return True
    parent = bodies[node["parent"]]
    length = float(np.linalg.norm(np.array(node["world_pos"]) - np.array(parent["world_pos"])))
    return length >= ZERO_LEN_EPS


def resolve_bone_parent(bodies, name):
    parent_name = bodies[name]["parent"]
    while parent_name is not None and not has_own_bone(bodies, parent_name):
        parent_name = bodies[parent_name]["parent"]
    return parent_name


# ---- main conversion --------------------------------------------------------

def bone_frame_offsets(bodies, tpose_frame):
    """Per-bone fixed conjugation offset between Blender's own bone-rest
    orientation (bone direction + Blender's auto-computed roll) and MuJoCo's
    actual body_quat convention (data.xquat, i.e. the JSON's world_quat_wxyz).

    These aren't the same frame: Blender's bone-local axes come from bone
    head/tail direction + an arbitrary roll, unrelated to the body's own
    `quat=` attribute in the MJCF. Comparing a "rest" world quat (from JSON,
    MuJoCo convention) against a "current" world quat (from Blender, its own
    convention) doesn't cancel cleanly in the delta -- it leaves the delta
    conjugated by this per-bone offset C, i.e.
    delta_blender = inverse(C) * true_delta * C. Solving: true_delta =
    C * delta_blender * inverse(C). C itself is fixed (purely structural, not
    time-varying) since it only depends on how the bone was built, so it's
    measured once here from Blender's own T-pose extraction of the
    (unanimated) armature FBX, compared against the same body's JSON rest
    quat -- both "at rest", so any real per-frame delta is zero and only the
    fixed conjugation offset remains: C(body) = inverse(mujoco_rest_world) *
    blender_tpose_world.
    """
    offsets = {}
    for name, body in bodies.items():
        if name not in tpose_frame:
            continue  # merged-away body, no bone to compare
        mujoco_rest_world = np.array(body["world_quat_wxyz"])
        blender_tpose_world = np.array(tpose_frame[name]["quat_wxyz"])
        offsets[name] = quat_mul(quat_inv(mujoco_rest_world), blender_tpose_world)
    return offsets


def body_joints(model, name):
    """[(axis_tuple, qpos_adr), ...] in declaration order for this body,
    or None if it's the free joint, or [] if it has no joints."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    jadr, jnum = model.body_jntadr[body_id], model.body_jntnum[body_id]
    joints = []
    for i in range(jadr, jadr + jnum):
        if mujoco.mjtJoint(model.jnt_type[i]) == mujoco.mjtJoint.mjJNT_FREE:
            return None
        joints.append((tuple(model.jnt_axis[i].tolist()), model.jnt_qposadr[i]))
    return joints


def convert_clip(model, bodies, pose_json_path, tpose_json_path, output_path):
    with open(pose_json_path) as f:
        pose_data = json.load(f)
    frames = pose_data["frames"]
    num_frames = len(frames)

    with open(tpose_json_path) as f:
        tpose_data = json.load(f)
    offsets = bone_frame_offsets(bodies, tpose_data["frames"][0])

    qpos = np.zeros((num_frames, model.nq))

    root_name = next(n for n, b in bodies.items() if b["parent"] is None)

    warned_unresolvable = set()
    for name in bodies:
        joints = body_joints(model, name)
        if joints is None:
            continue  # free joint (root), handled separately below
        if not joints:
            continue  # no DOFs on this body

        if not has_own_bone(bodies, name):
            if name not in warned_unresolvable:
                print(f"WARNING: '{name}' has joints but was merged away in the "
                      f"armature (zero offset from its parent) -- leaving its "
                      f"{len(joints)} DOF(s) at 0. Its rotation should already be "
                      f"folded into its resolved child's joint instead.")
                warned_unresolvable.add(name)
            continue

        parent_name = resolve_bone_parent(bodies, name)
        # rest_local MUST come from the same source (Blender) as current_local
        # below, not from the MuJoCo JSON -- mixing sources leaves the
        # per-bone conjugation offset (C) uncancelled even at matching poses.
        tpose_frame = tpose_data["frames"][0]
        rest_parent_q = np.array(tpose_frame[parent_name]["quat_wxyz"])
        rest_body_q = np.array(tpose_frame[name]["quat_wxyz"])
        rest_local = quat_mul(quat_inv(rest_parent_q), rest_body_q)
        c_body = offsets[name]
        c_body_inv = quat_inv(c_body)

        for frame_idx, frame in enumerate(frames):
            cur_parent_q = np.array(frame[parent_name]["quat_wxyz"])
            cur_body_q = np.array(frame[name]["quat_wxyz"])
            current_local = quat_mul(quat_inv(cur_parent_q), cur_body_q)
            delta_blender = quat_mul(quat_inv(rest_local), current_local)
            delta = quat_mul(quat_mul(c_body, delta_blender), c_body_inv)

            axes = [a for a, _adr in joints]
            angles = decompose_for_axes(delta, axes)
            for (_axis, adr), angle in zip(joints, angles):
                qpos[frame_idx, adr] = angle

    # Root (free joint): direct world position + orientation -- no *local*
    # delta (a free joint's qpos quat *is* the world quat), but it still
    # needs the same per-bone conjugation correction applied to translate
    # Blender's world quat back into MuJoCo's own world-quat convention:
    # true_world = blender_world * inverse(C_root).
    root_joints = body_joints(model, root_name)
    assert root_joints is None, f"expected '{root_name}' to be a free joint"
    root_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, root_name)
    root_jnt_adr = model.body_jntadr[root_body_id]
    root_qpos_adr = model.jnt_qposadr[root_jnt_adr]
    c_root_inv = quat_inv(offsets[root_name])
    for frame_idx, frame in enumerate(frames):
        pos = frame[root_name]["pos"]
        blender_quat = np.array(frame[root_name]["quat_wxyz"])
        true_quat = quat_mul(blender_quat, c_root_inv)
        qpos[frame_idx, root_qpos_adr:root_qpos_adr + 3] = pos
        qpos[frame_idx, root_qpos_adr + 3:root_qpos_adr + 7] = true_quat

    np.savez(output_path, qpos=qpos, fps=pose_data["fps"])
    print(f"Wrote qpos array {qpos.shape} (nq={model.nq}) -> {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mjcf", default="mjcf/robot.xml")
    parser.add_argument("--skeleton-json", default="assets/robot/robot.json",
                         help="Original rest-pose JSON from export_skeleton_json.py")
    parser.add_argument("--folder", required=True,
                         help="Directory containing an fbx_pose/ subfolder (stage-1 output "
                              "from extract_fbx_pose.py), one <clip>/{pose.json,tpose.json} "
                              "per clip. Output is written to <folder>/qpos/<clip>.npz")
    args = parser.parse_args()

    pose_dir = Path(args.folder) / "fbx_pose"
    out_dir = Path(args.folder) / "qpos"
    clip_dirs = sorted(p for p in pose_dir.iterdir() if p.is_dir())
    if not clip_dirs:
        print(f"No clip folders found in {pose_dir}")
        return
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(args.mjcf)
    bodies = load_bodies(args.skeleton_json)

    for clip_dir in clip_dirs:
        convert_clip(model, bodies,
                     clip_dir / "pose.json", clip_dir / "tpose.json",
                     out_dir / f"{clip_dir.name}.npz")


if __name__ == "__main__":
    main()
