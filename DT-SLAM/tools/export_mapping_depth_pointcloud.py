#!/usr/bin/env python3
"""Export paired RGB-D point clouds before and after the S3 depth mask.

The two clouds use exactly the same successful DT-SLAM poses, input frames,
pixel stride, and depth limits. Their only difference is whether pixels marked
non-zero by the S3 mapping mask are omitted. This is an offline evaluation
tool; it does not add a dense mapper to DT-SLAM.
"""

import argparse
import json
import math
from pathlib import Path
import struct

import cv2
import numpy as np


POINT_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("red", "u1"), ("green", "u1"), ("blue", "u1"),
])


def read_associations(path):
    rows = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.strip().split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) < 4:
                raise ValueError("association row must contain four fields")
            rows.append({
                "timestamp": float(fields[0]),
                "rgb": fields[1],
                "depth": fields[3],
            })
    return rows


def quaternion_to_rotation(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0.0:
        raise ValueError("zero-norm trajectory quaternion")
    x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def read_trajectory(path):
    poses = []
    with open(path, "r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.strip().split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                raise ValueError("TUM trajectory row must contain eight fields")
            values = [float(value) for value in fields]
            poses.append((
                values[0],
                quaternion_to_rotation(*values[4:8]),
                np.asarray(values[1:4], dtype=np.float64),
            ))
    return poses


def nearest_pose(poses, timestamp, maximum_difference):
    if not poses:
        return None
    # The trajectory and associations are ordered; this small offline tool uses
    # a direct search to keep timestamp matching explicit and deterministic.
    candidate = min(poses, key=lambda item: abs(item[0]-timestamp))
    if abs(candidate[0]-timestamp) > maximum_difference:
        return None
    return candidate


def load_frame(root, row, mask_path, depth_factor, maximum_depth,
               stride, apply_mask, dynamic_threshold):
    color = cv2.imread(str(root / row["rgb"]), cv2.IMREAD_COLOR)
    depth_raw = cv2.imread(str(root / row["depth"]), cv2.IMREAD_UNCHANGED)
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if color is None or depth_raw is None or mask is None:
        raise FileNotFoundError("failed to load RGB, depth, or S3 mask")
    if color.shape[:2] != depth_raw.shape[:2] or mask.shape != depth_raw.shape:
        raise ValueError("RGB, depth, and S3 mask dimensions must match")

    depth = depth_raw.astype(np.float32) / depth_factor
    sampled_depth = depth[::stride, ::stride]
    valid = np.isfinite(sampled_depth) & (sampled_depth > 0.0)
    if maximum_depth > 0.0:
        valid &= sampled_depth <= maximum_depth
    if apply_mask:
        valid &= mask[::stride, ::stride] < dynamic_threshold
    return color[::stride, ::stride], sampled_depth, valid


def count_points(frames, root, depth_factor, maximum_depth, stride,
                 dynamic_threshold):
    unfiltered = 0
    filtered = 0
    for frame in frames:
        _, _, valid_all = load_frame(
            root, frame["association"], frame["mask"], depth_factor,
            maximum_depth, stride, False, dynamic_threshold)
        _, _, valid_filtered = load_frame(
            root, frame["association"], frame["mask"], depth_factor,
            maximum_depth, stride, True, dynamic_threshold)
        unfiltered += int(np.count_nonzero(valid_all))
        filtered += int(np.count_nonzero(valid_filtered))
    return unfiltered, filtered


def write_header(stream, point_count):
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {point_count}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    stream.write(header.encode("ascii"))


def append_frame(stream, root, frame, fx, fy, cx, cy, depth_factor,
                 maximum_depth, stride, apply_mask, dynamic_threshold):
    color, depth, valid = load_frame(
        root, frame["association"], frame["mask"], depth_factor,
        maximum_depth, stride, apply_mask, dynamic_threshold)
    rows, cols = np.indices(depth.shape, dtype=np.float32)
    u = cols * stride
    v = rows * stride
    z = depth[valid].astype(np.float64)
    camera = np.stack([
        (u[valid]-cx) * z / fx,
        (v[valid]-cy) * z / fy,
        z,
    ], axis=1)
    rotation = frame["pose"][1]
    translation = frame["pose"][2]
    world = camera @ rotation.T + translation
    bgr = color[valid]
    points = np.empty(world.shape[0], dtype=POINT_DTYPE)
    points["x"] = world[:, 0]
    points["y"] = world[:, 1]
    points["z"] = world[:, 2]
    points["red"] = bgr[:, 2]
    points["green"] = bgr[:, 1]
    points["blue"] = bgr[:, 0]
    stream.write(points.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--associations", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--mask-directory", required=True)
    parser.add_argument(
        "--mask-pattern",
        default="frame_{index:06d}_dynamic_depth_mask.png",
        help="Python format pattern relative to mask-directory")
    parser.add_argument(
        "--dynamic-threshold", type=int, default=1,
        help="mask values greater than or equal to this are omitted")
    parser.add_argument("--unfiltered-output", required=True)
    parser.add_argument("--filtered-output", required=True)
    parser.add_argument("--summary-output", required=True)
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--depth-factor", type=float, default=5000.0)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--pixel-stride", type=int, default=4)
    parser.add_argument("--frame-step", type=int, default=5)
    parser.add_argument("--maximum-input-frames", type=int, default=0,
                        help="0 keeps every association row")
    parser.add_argument("--maximum-time-difference", type=float, default=0.02)
    args = parser.parse_args()
    if args.pixel_stride < 1 or args.frame_step < 1:
        raise ValueError("pixel stride and frame step must be positive")
    if not 0 <= args.dynamic_threshold <= 255:
        raise ValueError("dynamic threshold must be in [0,255]")

    root = Path(args.dataset_root)
    mask_directory = Path(args.mask_directory)
    associations = read_associations(args.associations)
    poses = read_trajectory(args.trajectory)
    frames = []
    unmatched_pose = 0
    missing_mask = 0
    selected_associations = associations
    if args.maximum_input_frames > 0:
        selected_associations = associations[:args.maximum_input_frames]
    for index, row in enumerate(selected_associations):
        if index % args.frame_step != 0:
            continue
        mask = mask_directory / args.mask_pattern.format(index=index)
        if not mask.is_file():
            missing_mask += 1
            continue
        pose = nearest_pose(poses, row["timestamp"],
                            args.maximum_time_difference)
        if pose is None:
            unmatched_pose += 1
            continue
        frames.append({"index": index, "association": row,
                       "mask": mask, "pose": pose})
    if not frames:
        raise RuntimeError("no frame has both a mask and a matched pose")

    unfiltered_count, filtered_count = count_points(
        frames, root, args.depth_factor, args.maximum_depth,
        args.pixel_stride, args.dynamic_threshold)
    output_pairs = [
        (Path(args.unfiltered_output), unfiltered_count, False),
        (Path(args.filtered_output), filtered_count, True),
    ]
    for output, count, apply_mask in output_pairs:
        output.parent.mkdir(parents=True, exist_ok=True)
        with open(output, "wb") as stream:
            write_header(stream, count)
            for frame in frames:
                append_frame(
                    stream, root, frame, args.fx, args.fy, args.cx,
                    args.cy, args.depth_factor, args.maximum_depth,
                    args.pixel_stride, apply_mask, args.dynamic_threshold)

    summary = {
        "method": "same-pose paired offline RGB-D point-cloud export",
        "frames": len(frames),
        "frame_step": args.frame_step,
        "pixel_stride": args.pixel_stride,
        "maximum_depth_m": args.maximum_depth,
        "mask_pattern": args.mask_pattern,
        "dynamic_threshold": args.dynamic_threshold,
        "unmatched_pose_frames": unmatched_pose,
        "missing_mask_frames": missing_mask,
        "unfiltered_points": unfiltered_count,
        "filtered_points": filtered_count,
        "removed_points": unfiltered_count-filtered_count,
        "removed_fraction": (
            (unfiltered_count-filtered_count)/unfiltered_count
            if unfiltered_count else 0.0),
        "unfiltered_output": str(Path(args.unfiltered_output)),
        "filtered_output": str(Path(args.filtered_output)),
    }
    summary_path = Path(args.summary_output)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with open(summary_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
