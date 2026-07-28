"""Stage 2 (Blender) of converting a MuJoCo `qpos` array into an animated
FBX -- see `qpos_to_world_pose.py` for stage 1 (the venv half, which runs
`mj_forward` per frame since Blender's bundled Python has no `mujoco`
package). Consumes stage 1's world-pose JSON plus the skeleton's existing
armature FBX (e.g. `assets/robot/robot.fbx`), poses and keyframes it one
frame at a time, and exports an animated FBX for import into Unreal Engine.

Per body B with its own bone (`has_own_bone`), stage 1 already gives the
fully-composed world orientation/position for every frame directly (MuJoCo's
own `mj_forward`) -- no need to decompose/recompose per-joint hinge angles
the way `fbx_pose_to_qpos.py` does (that script needs angles because it
extracts scalar qpos values FROM a pose; here we already start with qpos).
The only conversion needed is the same fixed per-bone conjugation offset
`C(body)` that `fbx_pose_to_qpos.py`'s `bone_frame_offsets()` computes
(Blender's bone-local frame != MuJoCo's body_quat convention), applied in the
opposite direction:

    blender_world_quat(B, frame) = mujoco_world_quat(B, frame) * C(B)

(inverting `fbx_pose_to_qpos.py`'s `true_quat = blender_quat * inverse(C)`;
quaternion associativity makes this hold for every body, not just the root
-- see that script's `bone_frame_offsets()` docstring for why C is
frame-independent, and this script's own `bone_frame_offsets()` below for the
derivation.)

Run with:
    blender --background --python scripts/world_pose_to_fbx.py -- \
        --world_pose <path.json> --skeleton_json assets/robot/robot.json \
        --armature_fbx assets/robot/robot.fbx --out_fbx <path.fbx>
"""
import argparse
import json
import sys

import bpy  # type: ignore
import mathutils  # type: ignore

ZERO_LEN_EPS = 1e-4  # meters; matches fbx_pose_to_qpos.py's merge threshold


# ---- quaternion helpers (wxyz order, plain tuples -- no numpy in Blender) --

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_inv(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


# ---- skeleton merge-resolution -- ported from fbx_pose_to_qpos.py ---------

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
    length = (mathutils.Vector(node["world_pos"]) - mathutils.Vector(parent["world_pos"])).length
    return length >= ZERO_LEN_EPS


def resolve_bone_parent(bodies, name):
    parent_name = bodies[name]["parent"]
    while parent_name is not None and not has_own_bone(bodies, parent_name):
        parent_name = bodies[parent_name]["parent"]
    return parent_name


def bfs_order(bodies):
    """Parent-before-child order over bodies with their own bone. Required
    because posing a child bone (setting pbone.matrix to a world-space
    target) needs the parent's *already-posed* matrix to compute the child's
    local matrix_basis correctly -- posing out of order silently uses a
    stale (rest or previous-frame) parent transform."""
    children_of = {}
    for name in bodies:
        if not has_own_bone(bodies, name):
            continue
        children_of.setdefault(resolve_bone_parent(bodies, name), []).append(name)

    order = []
    queue = list(children_of.get(None, []))
    while queue:
        current = queue.pop(0)
        order.append(current)
        queue.extend(children_of.get(current, []))
    return order


# ---- Blender-vs-MuJoCo per-bone conjugation offset -------------------------

def bone_frame_offsets(bodies, armature_obj):
    """C(body) = inverse(mujoco_rest_world_quat) * blender_rest_world_quat,
    read directly off the just-imported (unposed) armature -- identical
    definition to fbx_pose_to_qpos.py's bone_frame_offsets(), just sourced
    from a live bpy armature instead of a pre-extracted tpose.json."""
    offsets = {}
    for pbone in armature_obj.pose.bones:
        name = pbone.name
        if name not in bodies:
            continue
        mat = armature_obj.matrix_world @ pbone.matrix
        q = mat.to_quaternion()
        blender_rest = (q.w, q.x, q.y, q.z)
        mujoco_rest = tuple(bodies[name]["world_quat_wxyz"])
        offsets[name] = quat_mul(quat_inv(mujoco_rest), blender_rest)
    return offsets


def derive_position_scale(bodies, armature_obj, order):
    """Empirically measure the ratio between this armature's own ARMATURE-
    space units (i.e. pose_bone.matrix, with no matrix_world applied) and
    MuJoCo's raw meters, from the head-to-head distance of the first bone
    pair (root -> its first child), rather than assuming
    build_armature_fbx.py's WORLD_SCALE=100 survives an FBX export/import
    round-trip unchanged -- Blender's importer may apply its own unit
    conversion on top of that pre-scale (confirmed: it leaves a
    compensating ~0.01 object-level scale on the armature instead, so
    ARMATURE space is ~100x meters while WORLD space -- matrix_world @
    pose_bone.matrix -- is back in real meters).

    Deliberately measured in armature space, and used to build pose targets
    directly in armature space (see main()) without ever multiplying a
    world-space matrix by matrix_world.inverted(): matrix_world here is a
    pure uniform scale with no rotation, and multiplying a rotation+
    translation matrix by a scale-only matrix scales the rotation submatrix
    too (turning it into a non-orthonormal "rotation*100" block). Blender's
    pose solver strips that back out cleanly for a single, unchained bone
    (e.g. the root) but compounds it into real, growing position error for
    every deeper bone in a chain -- so translation must be rescaled by a
    plain scalar (this function's result) instead, leaving rotation
    (already scale-invariant) untouched.
    """
    child_name = order[1]
    parent_name = resolve_bone_parent(bodies, child_name)

    def armature_pos(name):
        return armature_obj.pose.bones[name].matrix.to_translation()

    armature_dist = (armature_pos(child_name) - armature_pos(parent_name)).length
    json_dist = (
        mathutils.Vector(bodies[child_name]["world_pos"])
        - mathutils.Vector(bodies[parent_name]["world_pos"])
    ).length
    return armature_dist / json_dist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world_pose", required=True, help="Stage 1 output JSON")
    parser.add_argument("--skeleton_json", default="assets/robot/robot.json")
    parser.add_argument("--armature_fbx", default="assets/robot/robot.fbx")
    parser.add_argument("--out_fbx", required=True)
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    with open(args.world_pose) as f:
        pose_data = json.load(f)
    frames = pose_data["frames"]
    num_frames = pose_data["num_frames"]

    bodies = load_bodies(args.skeleton_json)
    order = bfs_order(bodies)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=args.armature_fbx)
    armature_obj = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    bpy.context.view_layer.update()

    scale_factor = derive_position_scale(bodies, armature_obj, order)
    print(f"Derived position scale factor: {scale_factor:.4f}")

    offsets = bone_frame_offsets(bodies, armature_obj)

    for name in order:
        armature_obj.pose.bones[name].rotation_mode = "QUATERNION"

    scene = bpy.context.scene
    scene.render.fps = int(round(pose_data["fps"]))
    scene.frame_start = 0
    scene.frame_end = max(num_frames - 1, 0)

    for frame_idx, frame in enumerate(frames):
        scene.frame_set(frame_idx)

        for name in order:
            pbone = armature_obj.pose.bones[name]
            mujoco_pos = frame[name]["pos"]
            mujoco_quat = tuple(frame[name]["quat_wxyz"])
            blender_quat = quat_mul(mujoco_quat, offsets[name])

            # pos_vec is already in ARMATURE-space units (scale_factor
            # converts meters directly -- see derive_position_scale's
            # docstring for why this must be a plain scalar multiply, not a
            # matrix_world.inverted() @ ... conversion). Rotation is
            # scale-invariant and needs no conversion either way.
            pos_vec = mathutils.Vector(mujoco_pos) * scale_factor
            quat = mathutils.Quaternion(blender_quat)
            armature_target = mathutils.Matrix.Translation(pos_vec) @ quat.to_matrix().to_4x4()

            # Bones are posed in `order` (parent before child) so this
            # assignment always sees the parent's already-updated matrix.
            pbone.matrix = armature_target
            bpy.context.view_layer.update()
            pbone.keyframe_insert(data_path="location", frame=frame_idx)
            pbone.keyframe_insert(data_path="rotation_quaternion", frame=frame_idx)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.fbx(
        filepath=args.out_fbx, use_selection=True, add_leaf_bones=False,
        global_scale=1.0, apply_unit_scale=True, apply_scale_options='FBX_SCALE_ALL',
        bake_anim=True, bake_anim_use_all_bones=True,
        bake_anim_use_nla_strips=False, bake_anim_use_all_actions=False,
        bake_anim_force_startend_keying=True, bake_anim_step=1.0,
        bake_anim_simplify_factor=0.0,
    )
    print(f"Wrote {num_frames} frames -> {args.out_fbx}")


if __name__ == "__main__":
    main()
