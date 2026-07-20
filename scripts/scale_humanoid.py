"""Scale body-segment lengths and girth of humanoid_CMU.xml to produce
different body proportions/morphologies.

Each <body> stores its offset from its parent in its own `pos` attribute,
which represents the parent bone's length/direction. Scaling that vector
(and the pos/size of the geoms directly under it) lengthens or shortens just
that one segment, without needing to touch any other body in the chain.

Since the file has no explicit <inertial> elements and no <compiler
inertiafromgeom="false">, MuJoCo auto-computes mass/inertia from geom size,
so no manual inertia bookkeeping is needed after scaling.
"""
import argparse
from lxml import etree

LEG_BODIES = [
    "lfemur", "ltibia", "lfoot", "ltoes",
    "rfemur", "rtibia", "rfoot", "rtoes",
]
ARM_BODIES = [
    "lclavicle", "lhumerus", "lradius", "lwrist", "lhand", "lfingers", "lthumb",
    "rclavicle", "rhumerus", "rradius", "rwrist", "rhand", "rfingers", "rthumb",
]
TORSO_BODIES = ["lowerback", "upperback", "thorax"]
HEAD_BODIES = ["lowerneck", "upperneck", "head"]


def scale_vec_attr(elem, attr, factor):
    vals = [float(x) for x in elem.get(attr).split()]
    scaled = [v * factor for v in vals]
    elem.set(attr, " ".join(f"{v:.6g}" for v in scaled))


def scale_body(body_elem, length_factor, girth_factor):
    if body_elem.get("pos") is not None:
        scale_vec_attr(body_elem, "pos", length_factor)

    for geom in body_elem.findall("geom"):
        if geom.get("pos") is not None:
            scale_vec_attr(geom, "pos", length_factor)
        if geom.get("size") is not None:
            sizes = [float(x) for x in geom.get("size").split()]
            gtype = geom.get("type", "capsule")
            if gtype == "capsule" and len(sizes) >= 2:
                sizes[0] *= girth_factor
                sizes[1] *= length_factor
            elif gtype == "sphere":
                sizes = [s * girth_factor for s in sizes]
            else:
                sizes = [s * girth_factor for s in sizes]
            geom.set("size", " ".join(f"{s:.6g}" for s in sizes))


def scale_humanoid(input_path, output_path, leg_scale, arm_scale, torso_scale,
                    head_scale, girth_scale):
    tree = etree.parse(input_path)
    root = tree.getroot()

    body_by_name = {b.get("name"): b for b in root.iter("body")}

    groups = [
        (LEG_BODIES, leg_scale),
        (ARM_BODIES, arm_scale),
        (TORSO_BODIES, torso_scale),
        (HEAD_BODIES, head_scale),
    ]
    for names, length_factor in groups:
        for name in names:
            body = body_by_name.get(name)
            if body is None:
                continue
            scale_body(body, length_factor, girth_scale)

    tree.write(output_path, pretty_print=True, xml_declaration=False)
    print(f"Wrote {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Scale humanoid_CMU.xml body proportions.")
    parser.add_argument("--input", default="mjcf/humanoid_CMU.xml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--leg_scale", type=float, default=1.0)
    parser.add_argument("--arm_scale", type=float, default=1.0)
    parser.add_argument("--torso_scale", type=float, default=1.0)
    parser.add_argument("--head_scale", type=float, default=1.0)
    parser.add_argument("--girth_scale", type=float, default=1.0)
    args = parser.parse_args()

    scale_humanoid(
        args.input, args.output,
        leg_scale=args.leg_scale,
        arm_scale=args.arm_scale,
        torso_scale=args.torso_scale,
        head_scale=args.head_scale,
        girth_scale=args.girth_scale,
    )


if __name__ == "__main__":
    main()
