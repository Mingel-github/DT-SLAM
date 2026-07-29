#!/usr/bin/env python3
"""Create a one-to-one RGB/depth association without overwriting inputs."""

import argparse
import pathlib
import sys


def read_timestamp_file(path):
    entries = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 2:
                raise ValueError(
                    "{}:{}: expected timestamp and path".format(path, line_number)
                )
            entries.append((float(fields[0]), fields[0], fields[1]))
    if not entries:
        raise ValueError("{} contains no timestamped files".format(path))
    return entries


def keep_existing_files(entries, dataset_root):
    kept = []
    missing = []
    for entry in entries:
        if (dataset_root / entry[2]).is_file():
            kept.append(entry)
        else:
            missing.append(entry)
    return kept, missing


def associate_one_to_one(rgb_entries, depth_entries, max_difference_seconds):
    candidates = []
    for rgb_index, rgb in enumerate(rgb_entries):
        for depth_index, depth in enumerate(depth_entries):
            difference = abs(rgb[0] - depth[0])
            if difference < max_difference_seconds:
                candidates.append((difference, rgb_index, depth_index))

    candidates.sort()
    used_rgb = set()
    used_depth = set()
    matches = []
    for difference, rgb_index, depth_index in candidates:
        if rgb_index in used_rgb or depth_index in used_depth:
            continue
        used_rgb.add(rgb_index)
        used_depth.add(depth_index)
        matches.append((rgb_index, depth_index, difference))

    matches.sort(key=lambda match: rgb_entries[match[0]][0])
    return matches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("rgb_list", type=pathlib.Path)
    parser.add_argument("depth_list", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--max-difference-ms", type=float, default=20.0)
    parser.add_argument(
        "--dataset-root",
        type=pathlib.Path,
        help=(
            "filter timestamp entries whose referenced files do not exist "
            "under this dataset directory"
        ),
    )
    args = parser.parse_args()

    if args.max_difference_ms <= 0.0:
        raise ValueError("--max-difference-ms must be positive")
    if args.output.exists():
        raise ValueError(
            "{} already exists; refusing to overwrite it".format(args.output)
        )

    rgb_entries = read_timestamp_file(args.rgb_list)
    depth_entries = read_timestamp_file(args.depth_list)
    missing_rgb = []
    missing_depth = []
    if args.dataset_root is not None:
        rgb_entries, missing_rgb = keep_existing_files(
            rgb_entries, args.dataset_root
        )
        depth_entries, missing_depth = keep_existing_files(
            depth_entries, args.dataset_root
        )
        if not rgb_entries or not depth_entries:
            raise ValueError(
                "no existing RGB/depth files remain under {}".format(
                    args.dataset_root
                )
            )
    matches = associate_one_to_one(
        rgb_entries, depth_entries, args.max_difference_ms / 1000.0
    )
    if not matches:
        raise ValueError("no RGB/depth pairs satisfy the requested time threshold")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(
            "# one-to-one RGB/depth association; max_difference_ms={:.6f}\n".format(
                args.max_difference_ms
            )
        )
        if args.dataset_root is not None:
            stream.write(
                "# existing-file filter root={}; missing_rgb={}; "
                "missing_depth={}\n".format(
                    args.dataset_root, len(missing_rgb), len(missing_depth)
                )
            )
        for rgb_index, depth_index, _ in matches:
            rgb = rgb_entries[rgb_index]
            depth = depth_entries[depth_index]
            stream.write(
                "{} {} {} {}\n".format(rgb[1], rgb[2], depth[1], depth[2])
            )

    max_difference_ms = max(match[2] for match in matches) * 1000.0
    print(
        "wrote {} pairs to {}; unmatched_rgb={}; unmatched_depth={}; "
        "missing_rgb={}; missing_depth={}; max_abs_delta_ms={:.6f}".format(
            len(matches),
            args.output,
            len(rgb_entries) - len(matches),
            len(depth_entries) - len(matches),
            len(missing_rgb),
            len(missing_depth),
            max_difference_ms,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("association failed: {}".format(error), file=sys.stderr)
        sys.exit(2)
