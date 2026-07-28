#!/usr/bin/env python3
"""Audit RGB-D geometry input domains and timestamp associations.

This tool is read-only. It does not run SLAM or modify the dataset.
"""

import argparse
import bisect
import json
import math
import pathlib
import statistics
import struct
import sys
from collections import Counter


def parse_numeric_settings(path):
    settings = {}
    with path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            try:
                settings[key.strip()] = float(value.strip())
            except ValueError:
                continue
    return settings


def parse_associations(path):
    records = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 4:
                raise ValueError(
                    "{}:{}: expected rgb_ts rgb_path depth_ts depth_path".format(
                        path, line_number
                    )
                )
            records.append(
                {
                    "rgb_timestamp": float(fields[0]),
                    "rgb_path": fields[1],
                    "depth_timestamp": float(fields[2]),
                    "depth_path": fields[3],
                }
            )
    if not records:
        raise ValueError("{} contains no associations".format(path))
    return records


def parse_groundtruth_timestamps(path):
    timestamps = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) < 8:
                raise ValueError(
                    "{}:{}: expected timestamp tx ty tz qx qy qz qw".format(
                        path, line_number
                    )
                )
            timestamps.append(float(fields[0]))
    if not timestamps:
        raise ValueError("{} contains no ground-truth poses".format(path))
    timestamps.sort()
    return timestamps


def nearest_delta(timestamp, sorted_timestamps):
    index = bisect.bisect_left(sorted_timestamps, timestamp)
    candidates = []
    if index < len(sorted_timestamps):
        candidates.append(sorted_timestamps[index] - timestamp)
    if index > 0:
        candidates.append(sorted_timestamps[index - 1] - timestamp)
    return min(candidates, key=abs)


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    index = max(0, math.ceil(fraction * len(sorted_values)) - 1)
    return sorted_values[index]


def summarize_seconds(values):
    if not values:
        return None
    sorted_values = sorted(values)
    absolute = sorted(abs(value) for value in values)
    return {
        "count": len(values),
        "signed_mean_ms": 1000.0 * statistics.fmean(values),
        "abs_mean_ms": 1000.0 * statistics.fmean(absolute),
        "abs_median_ms": 1000.0 * statistics.median(absolute),
        "abs_p95_ms": 1000.0 * percentile(absolute, 0.95),
        "abs_max_ms": 1000.0 * absolute[-1],
        "within_5ms": sum(abs(value) <= 0.005 for value in values),
        "within_10ms": sum(abs(value) <= 0.010 for value in values),
        "within_20ms": sum(abs(value) <= 0.020 for value in values),
        "within_50ms": sum(abs(value) <= 0.050 for value in values),
    }


def read_png_header(path):
    with path.open("rb") as stream:
        signature = stream.read(8)
        if signature != b"\x89PNG\r\n\x1a\n":
            return {"format": "not_png"}
        length_bytes = stream.read(4)
        chunk_type = stream.read(4)
        if len(length_bytes) != 4 or chunk_type != b"IHDR":
            return {"format": "invalid_png"}
        length = struct.unpack(">I", length_bytes)[0]
        payload = stream.read(length)
        if len(payload) < 13:
            return {"format": "invalid_png"}
        width, height, bit_depth, color_type = struct.unpack(">IIBB", payload[:10])
        return {
            "format": "png",
            "width": width,
            "height": height,
            "bit_depth": bit_depth,
            "color_type": color_type,
        }


def inspect_paths(sequence, records):
    missing_rgb = []
    missing_depth = []
    for index, record in enumerate(records):
        if not (sequence / record["rgb_path"]).is_file():
            missing_rgb.append(index)
        if not (sequence / record["depth_path"]).is_file():
            missing_depth.append(index)

    sample_indices = sorted({0, len(records) // 2, len(records) - 1})
    samples = []
    for index in sample_indices:
        record = records[index]
        rgb_path = sequence / record["rgb_path"]
        depth_path = sequence / record["depth_path"]
        samples.append(
            {
                "index": index,
                "rgb": read_png_header(rgb_path) if rgb_path.is_file() else None,
                "depth": (
                    read_png_header(depth_path) if depth_path.is_file() else None
                ),
            }
        )
    return {
        "missing_rgb_count": len(missing_rgb),
        "missing_depth_count": len(missing_depth),
        "sample_headers": samples,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("settings", type=pathlib.Path)
    parser.add_argument("sequence", type=pathlib.Path)
    parser.add_argument("association", type=pathlib.Path)
    parser.add_argument(
        "--groundtruth",
        type=pathlib.Path,
        help="TUM-format groundtruth.txt; defaults to SEQUENCE/groundtruth.txt",
    )
    parser.add_argument("--json-output", type=pathlib.Path)
    args = parser.parse_args()

    settings = parse_numeric_settings(args.settings)
    records = parse_associations(args.association)
    groundtruth_path = args.groundtruth or args.sequence / "groundtruth.txt"

    distortion_names = [
        "Camera.k1",
        "Camera.k2",
        "Camera.p1",
        "Camera.p2",
        "Camera.k3",
    ]
    distortion = {name: settings.get(name, 0.0) for name in distortion_names}
    has_nonzero_distortion = any(
        abs(value) > 1e-8 for value in distortion.values()
    )

    rgb_depth_deltas = [
        record["depth_timestamp"] - record["rgb_timestamp"] for record in records
    ]
    depth_path_counts = Counter(record["depth_path"] for record in records)

    report = {
        "settings": str(args.settings.resolve()),
        "sequence": str(args.sequence.resolve()),
        "association": str(args.association.resolve()),
        "frames": len(records),
        "camera": {
            "fx": settings.get("Camera.fx"),
            "fy": settings.get("Camera.fy"),
            "cx": settings.get("Camera.cx"),
            "cy": settings.get("Camera.cy"),
            "width": settings.get("Camera.width"),
            "height": settings.get("Camera.height"),
            "depth_map_factor": settings.get("DepthMapFactor"),
            "distortion": distortion,
        },
        "geometry_domain": {
            "current_implementation": "raw RGB/depth pixels + pinhole K",
            "compatible": not has_nonzero_distortion,
            "decision": (
                "PASS: zero-distortion pinhole domain"
                if not has_nonzero_distortion
                else "BLOCK: rectify all modalities or implement distortion-aware warp"
            ),
        },
        "rgb_depth_sync": summarize_seconds(rgb_depth_deltas),
        "depth_reuse": {
            "unique_depth_paths": len(depth_path_counts),
            "reused_associations": len(records) - len(depth_path_counts),
            "max_rgb_frames_per_depth": max(depth_path_counts.values()),
        },
        "files": inspect_paths(args.sequence, records),
        "groundtruth": {
            "path": str(groundtruth_path.resolve()),
            "available": groundtruth_path.is_file(),
            "text_pose_convention": "Twc; invert before supplying Tcw to DT-SLAM",
        },
    }

    if groundtruth_path.is_file():
        gt_timestamps = parse_groundtruth_timestamps(groundtruth_path)
        rgb_gt_deltas = [
            nearest_delta(record["rgb_timestamp"], gt_timestamps)
            for record in records
        ]
        depth_gt_deltas = [
            nearest_delta(record["depth_timestamp"], gt_timestamps)
            for record in records
        ]
        report["groundtruth"].update(
            {
                "poses": len(gt_timestamps),
                "rgb_nearest_sync": summarize_seconds(rgb_gt_deltas),
                "depth_nearest_sync": summarize_seconds(depth_gt_deltas),
                "warning": (
                    "Nearest-pose statistics are diagnostic only; G0-2P must "
                    "record its interpolation and maximum time-gap policy."
                ),
            }
        )

    output = json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True)
    print(output)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("audit failed: {}".format(error), file=sys.stderr)
        sys.exit(2)
