"""Walk a local AMASS download and list every motion/sequence name found.

AMASS ships as a collection of sub-datasets (CMU, ACCAD, BMLmovi, KIT, ...),
each a tree of `<subject>/<sequence>_poses.npz` files. There is no separate
"action" field in the raw .npz -- the sequence name embedded in the filename
(e.g. "01_01", "A1 - Stand") is the closest thing to an action label. This
script recursively finds every `*_poses.npz` under a root directory and
records, per file: dataset (top-level folder), subject (immediate parent
folder), and action (filename with the `_poses.npz` suffix stripped).

Usage:
    python scripts/extract_amass_action_names.py --amass-dir /path/to/AMASS \
        --out amass_action_names.txt
"""
import argparse
import csv
from pathlib import Path


def collect_actions(amass_dir: Path):
    entries = []
    for npz_path in sorted(amass_dir.rglob("*_poses.npz")):
        rel = npz_path.relative_to(amass_dir)
        dataset = rel.parts[0]
        subject = npz_path.parent.name
        action = npz_path.name[: -len("_poses.npz")]
        entries.append((dataset, subject, action))
    return entries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--amass-dir", required=True,
                         help="Root directory of a local AMASS download "
                              "(contains per-dataset subfolders like CMU/, ACCAD/, ...)")
    parser.add_argument("--out", default="amass_action_names.txt",
                         help="Output file. Written as plain text (one action name per "
                              "line) unless the extension is .csv, which also includes "
                              "dataset/subject columns.")
    parser.add_argument("--unique", action="store_true",
                         help="Deduplicate action names before writing (plain-text output only)")
    args = parser.parse_args()

    amass_dir = Path(args.amass_dir)
    if not amass_dir.is_dir():
        raise SystemExit(f"Not a directory: {amass_dir}")

    entries = collect_actions(amass_dir)
    if not entries:
        raise SystemExit(f"No *_poses.npz files found under {amass_dir}")

    out_path = Path(args.out)
    if out_path.suffix.lower() == ".csv":
        with open(out_path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["dataset", "subject", "action"])
            writer.writerows(entries)
    else:
        names = [action for _, _, action in entries]
        if args.unique:
            names = sorted(set(names))
        with open(out_path, "w") as fh:
            fh.write("\n".join(names) + "\n")

    print(f"Found {len(entries)} sequences ({len(set(a for _, _, a in entries))} unique action names)")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
