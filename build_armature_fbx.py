"""Blender script: build an Armature (+ simple capsule/sphere reference meshes)
from a humanoid_skeleton.json (produced by export_skeleton_json.py) and export
it as FBX for import into Unreal Engine.

Run with:
    blender --background --python build_armature_fbx.py -- \
        --input humanoid_skeleton.json --output humanoid_CMU.fbx

Design notes:
- MuJoCo and Blender are both Z-up, right-handed, meters -- world positions
  from the JSON are used directly with no axis conversion.
- A body gets its own bone only if it is offset from its parent (some MJCF
  bodies, e.g. "lhipjoint"/"rhipjoint"/"lowerback" here, have no <pos> of
  their own and sit exactly on top of their parent -- Blender bones can't
  have zero length, and there's nothing to rig there anyway, so these are
  merged into their parent bone; their children attach to that parent bone
  instead).
- Leaf bodies (no children in the kinematic tree, e.g. fingers/toes/head)
  have no "next joint" to define a bone tail, so the tail is extrapolated by
  continuing the incoming bone's direction for a fraction of its length.
- Geom placement uses each geom's own local pos/quat relative to its body
  (not derived from bone direction -- MJCF capsule geoms are cosmetically
  rotated to align visually and are not aligned to the body's joint frame).
"""
import argparse
import json
import sys

import bmesh
import bpy
import mathutils

ZERO_LEN_EPS = 1e-4
LEAF_TAIL_FRACTION = 0.5
MIN_BONE_LEN = 0.01


def load_skeleton(path):
    with open(path) as f:
        data = json.load(f)
    bodies = {b["name"]: b for b in data["bodies"]}
    for b in bodies.values():
        if b["parent"] == "world":
            b["parent"] = None
    return bodies, data["geoms"]


def resolve_bone_parent(bodies, name):
    """Walk up through zero-length (pos-less) bodies to find the body whose
    bone this one should actually attach to."""
    parent_name = bodies[name]["parent"]
    while parent_name is not None and not has_own_bone(bodies, parent_name):
        parent_name = bodies[parent_name]["parent"]
    return parent_name


def has_own_bone(bodies, name):
    """False for pos-less bodies that got merged into their parent's bone."""
    node = bodies[name]
    if node["parent"] is None:
        return True
    parent = bodies[node["parent"]]
    length = (mathutils.Vector(node["world_pos"]) - mathutils.Vector(parent["world_pos"])).length
    return length >= ZERO_LEN_EPS


def build_armature(bodies):
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))
    armature_obj = bpy.context.object
    armature_obj.name = "humanoid_CMU_armature"
    armature = armature_obj.data
    armature.name = "humanoid_CMU_armature"
    edit_bones = armature.edit_bones
    edit_bones.remove(edit_bones[0])  # remove Blender's default starter bone

    children_of = {}
    for name in bodies:
        eff_parent = resolve_bone_parent(bodies, name)
        if has_own_bone(bodies, name):
            children_of.setdefault(eff_parent, []).append(name)

    bone_name_for_body = {}

    def create_bone(name, parent_bone_name):
        node = bodies[name]
        head = mathutils.Vector(node["world_pos"])
        bone = edit_bones.new(name)
        bone.head = head
        if parent_bone_name is not None:
            bone.parent = edit_bones[parent_bone_name]
            bone.use_connect = True
        bone_name_for_body[name] = name
        return bone

    # Root first (no parent bone).
    root_bodies = children_of.get(None, [])
    for root_name in root_bodies:
        create_bone(root_name, None)

    # BFS the rest so parents always exist before children.
    queue = list(root_bodies)
    while queue:
        current = queue.pop(0)
        for child_name in children_of.get(current, []):
            create_bone(child_name, current)
            queue.append(child_name)

    # Second pass: set tails now that all heads exist.
    for name, bone in list(edit_bones.items()):
        node = bodies[name]
        child_names = children_of.get(name, [])
        if child_names:
            # Point the tail at the (first) child's head; a bone with several
            # children still needs one tail, the others attach at the same head.
            child_head = mathutils.Vector(bodies[child_names[0]]["world_pos"])
            bone.tail = child_head
        else:
            # Leaf: extrapolate the incoming bone's direction.
            if bone.parent is not None:
                direction = (bone.head - bone.parent.head)
            else:
                direction = mathutils.Vector((0, 0, 0.1))
            if direction.length < 1e-6:
                direction = mathutils.Vector((0, 0, 0.1))
            bone.tail = bone.head + direction * LEAF_TAIL_FRACTION
        if (bone.tail - bone.head).length < MIN_BONE_LEN:
            bone.tail = bone.head + mathutils.Vector((0, 0, MIN_BONE_LEN))

    bpy.ops.object.mode_set(mode="OBJECT")
    return armature_obj


def _make_mesh_data(name, gtype, size):
    # bpy.ops.mesh.primitive_*_add() operators poll on an interactive 3D
    # viewport and silently no-op in --background mode, so build geometry
    # directly with bmesh instead.
    bm = bmesh.new()
    if gtype == "mjGEOM_CAPSULE":
        radius, half_len = size[0], size[1]
        bmesh.ops.create_cone(bm, cap_ends=True, segments=12,
                               radius1=radius, radius2=radius, depth=half_len * 2)
    elif gtype == "mjGEOM_SPHERE":
        bmesh.ops.create_uvsphere(bm, u_segments=12, v_segments=8, radius=size[0])
    elif gtype == "mjGEOM_ELLIPSOID":
        bmesh.ops.create_uvsphere(bm, u_segments=12, v_segments=8, radius=1.0)
    else:
        bm.free()
        return None

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    return mesh


def add_geom_meshes(armature_obj, bodies, geoms):
    for geom in geoms:
        body_name = geom["body"]
        if body_name not in bodies:
            continue
        gtype = geom["type"]
        size = geom["size"]
        if gtype not in ("mjGEOM_CAPSULE", "mjGEOM_SPHERE", "mjGEOM_ELLIPSOID"):
            continue

        mesh = _make_mesh_data(f"geom_{geom['name']}", gtype, size)
        if mesh is None:
            continue
        mesh_obj = bpy.data.objects.new(f"geom_{geom['name']}", mesh)
        bpy.context.scene.collection.objects.link(mesh_obj)
        if gtype == "mjGEOM_ELLIPSOID":
            mesh_obj.scale = (size[0], size[1], size[2])

        # Use MuJoCo's own world-space geom transform (already correctly
        # composed through the body's orientation) rather than recombining
        # body_world_pos + local_pos, which are expressed in different frames.
        world_pos = mathutils.Vector(geom["world_pos"])
        w, x, y, z = geom["world_quat_wxyz"]
        world_quat = mathutils.Quaternion((w, x, y, z))

        mesh_obj.location = world_pos
        mesh_obj.rotation_mode = "QUATERNION"
        mesh_obj.rotation_quaternion = world_quat

        mesh_obj.parent = armature_obj
        mod = mesh_obj.modifiers.new(name="Armature", type="ARMATURE")
        mod.object = armature_obj
        bone_name = body_name if has_own_bone(bodies, body_name) else resolve_bone_parent(bodies, body_name)
        vg = mesh_obj.vertex_groups.new(name=bone_name)
        vg.add(range(len(mesh_obj.data.vertices)), 1.0, "REPLACE")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--with_mesh", action=argparse.BooleanOptionalAction, default=True)
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    args = parser.parse_args(argv)

    bpy.ops.wm.read_factory_settings(use_empty=True)

    bodies, geoms = load_skeleton(args.input)
    armature_obj = build_armature(bodies)
    if args.with_mesh:
        add_geom_meshes(armature_obj, bodies, geoms)

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.fbx(filepath=args.output, use_selection=True, add_leaf_bones=False)
    print(f"Exported {args.output}")


if __name__ == "__main__":
    main()
