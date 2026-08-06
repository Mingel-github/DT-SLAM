#!/usr/bin/env python3
"""Audit a dynamic mask against the visible Gazebo box volume.

The reference is generated from synchronized camera and box poses, measured
RGB-D depth, and a known box half extent.  It is a simulation proxy for the
visible box surface, not a hand-labelled instance-segmentation ground truth.
The tool is read-only with respect to the detector and accepts an explicit
mask filename pattern and dynamic threshold so that tri-state masks are not
misinterpreted as binary masks.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def read_associations(path):
    rows = []
    with Path(path).open("r", encoding="utf-8") as stream:
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
        raise ValueError("zero-norm quaternion")
    x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
    return np.asarray([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def read_poses(path):
    poses = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.strip().split()
            if not fields or fields[0].startswith("#"):
                continue
            if len(fields) != 8:
                raise ValueError("TUM pose row must contain eight fields")
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
    pose = min(poses, key=lambda item: abs(item[0]-timestamp))
    difference = abs(pose[0]-timestamp)
    return (pose, difference) if difference <= maximum_difference else None


def visible_box_mask(depth_raw, camera_pose, box_pose, args):
    depth = depth_raw.astype(np.float64) / args.depth_factor
    sampled = depth[::args.sample_stride, ::args.sample_stride]
    rows, cols = np.indices(sampled.shape, dtype=np.float64)
    u = cols * args.sample_stride
    v = rows * args.sample_stride
    valid = np.isfinite(sampled) & (sampled > 0.0)
    if args.maximum_depth > 0.0:
        valid &= sampled <= args.maximum_depth

    z = sampled[valid]
    camera_points = np.stack([
        (u[valid]-args.cx) * z / args.fx,
        (v[valid]-args.cy) * z / args.fy,
        z,
    ], axis=1)
    world_points = camera_points @ camera_pose[1].T + camera_pose[2]
    box_points = (world_points-box_pose[2]) @ box_pose[1]
    inside = np.all(np.abs(box_points) <= args.box_half_extent, axis=1)
    result = np.zeros(sampled.shape, dtype=bool)
    result[valid] = inside
    return result


def quantile(values, probability):
    return float(np.quantile(np.asarray(values, dtype=np.float64), probability))


def make_contact_sheet(records, root, output_path, columns=4):
    visible = [record for record in records if record["box_pixels"] >= 25]
    invisible = [record for record in records if record["box_pixels"] == 0]
    selected = []
    if visible:
        ordered = sorted(visible, key=lambda item: item["box_pixels"])
        sample_indices = np.linspace(0, len(ordered)-1, 11).astype(int)
        selected.extend(ordered[index] for index in sample_indices)
    if invisible:
        selected.append(invisible[len(invisible)//2])
    if not selected:
        return

    tiles = []
    for record in selected:
        color = cv2.imread(str(root / record["rgb"]), cv2.IMREAD_COLOR)
        if color is None:
            continue
        full_box = cv2.resize(
            record["box_mask"].astype(np.uint8),
            (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)
        full_dynamic = cv2.resize(
            record["dynamic_mask"].astype(np.uint8),
            (color.shape[1], color.shape[0]), interpolation=cv2.INTER_NEAREST)
        overlay = color.copy()
        overlay[full_dynamic > 0] = (
            0.45*overlay[full_dynamic > 0] + 0.55*np.asarray([0, 0, 255])
        ).astype(np.uint8)
        contours, _ = cv2.findContours(
            (full_box*255).astype(np.uint8), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
        intersection = (full_box > 0) & (full_dynamic > 0)
        overlay[intersection] = (0, 255, 255)
        text = (f"f{record['frame']} box={record['box_pixels']} "
                f"rec={record['box_recall']:.2f} "
                f"box/mask={record['mask_precision']:.2f}")
        cv2.putText(overlay, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (0, 0, 0), 1, cv2.LINE_AA)
        tiles.append(cv2.resize(overlay, (480, 360)))
    if not tiles:
        return
    rows = int(math.ceil(len(tiles)/columns))
    blank = np.zeros_like(tiles[0])
    while len(tiles) < rows*columns:
        tiles.append(blank.copy())
    sheet = np.vstack([
        np.hstack(tiles[row*columns:(row+1)*columns])
        for row in range(rows)
    ])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), sheet)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--associations", required=True, type=Path)
    parser.add_argument("--camera-trajectory", required=True, type=Path)
    parser.add_argument("--box-trajectory", required=True, type=Path)
    parser.add_argument("--mask-directory", required=True, type=Path)
    parser.add_argument("--mask-pattern", default="frame_{index:06d}_dynamic_depth_mask.png")
    parser.add_argument("--dynamic-threshold", type=int, default=1)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--fx", required=True, type=float)
    parser.add_argument("--fy", required=True, type=float)
    parser.add_argument("--cx", required=True, type=float)
    parser.add_argument("--cy", required=True, type=float)
    parser.add_argument("--depth-factor", type=float, default=5000.0)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--sample-stride", type=int, default=2)
    parser.add_argument("--box-half-extent", type=float, default=0.32)
    parser.add_argument("--maximum-pose-time-difference", type=float, default=0.02)
    args = parser.parse_args()
    if args.sample_stride < 1:
        raise ValueError("sample stride must be positive")
    if not 0 <= args.dynamic_threshold <= 255:
        raise ValueError("dynamic threshold must be in [0,255]")

    root = args.dataset_root
    rows = read_associations(args.associations)
    camera_poses = read_poses(args.camera_trajectory)
    box_poses = read_poses(args.box_trajectory)
    records = []
    missing_masks = 0
    missing_poses = 0
    for index, row in enumerate(rows):
        mask_path = args.mask_directory / args.mask_pattern.format(index=index)
        if not mask_path.is_file():
            missing_masks += 1
            continue
        camera_match = nearest_pose(
            camera_poses, row["timestamp"], args.maximum_pose_time_difference)
        box_match = nearest_pose(
            box_poses, row["timestamp"], args.maximum_pose_time_difference)
        if camera_match is None or box_match is None:
            missing_poses += 1
            continue
        depth_raw = cv2.imread(str(root / row["depth"]), cv2.IMREAD_UNCHANGED)
        mask_raw = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if depth_raw is None or mask_raw is None:
            raise FileNotFoundError("failed to load depth or dynamic mask")
        if depth_raw.shape != mask_raw.shape:
            raise ValueError("depth and dynamic mask dimensions differ")
        box_mask = visible_box_mask(
            depth_raw, camera_match[0], box_match[0], args)
        dynamic_mask = mask_raw[::args.sample_stride,
                                ::args.sample_stride] >= args.dynamic_threshold
        intersection = box_mask & dynamic_mask
        box_pixels = int(np.count_nonzero(box_mask))
        dynamic_pixels = int(np.count_nonzero(dynamic_mask))
        intersection_pixels = int(np.count_nonzero(intersection))
        records.append({
            "frame": index,
            "timestamp": row["timestamp"],
            "rgb": row["rgb"],
            "box_pixels": box_pixels,
            "dynamic_pixels": dynamic_pixels,
            "intersection_pixels": intersection_pixels,
            "box_recall": intersection_pixels/box_pixels if box_pixels else math.nan,
            "mask_precision": intersection_pixels/dynamic_pixels if dynamic_pixels else math.nan,
            "global_mask_fraction": dynamic_pixels/dynamic_mask.size,
            "camera_pose_dt": camera_match[1],
            "box_pose_dt": box_match[1],
            "box_mask": box_mask,
            "dynamic_mask": dynamic_mask,
        })

    if not records:
        raise RuntimeError("no frame has mask, depth, and synchronized poses")
    args.output_directory.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_directory / "per_frame.csv"
    fields = [
        "frame", "timestamp", "box_pixels_stride", "dynamic_mask_pixels_stride",
        "intersection_pixels_stride", "box_recall", "mask_precision_to_box",
        "global_mask_fraction", "camera_pose_dt", "box_pose_dt",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({
                "frame": record["frame"],
                "timestamp": record["timestamp"],
                "box_pixels_stride": record["box_pixels"],
                "dynamic_mask_pixels_stride": record["dynamic_pixels"],
                "intersection_pixels_stride": record["intersection_pixels"],
                "box_recall": record["box_recall"],
                "mask_precision_to_box": record["mask_precision"],
                "global_mask_fraction": record["global_mask_fraction"],
                "camera_pose_dt": record["camera_pose_dt"],
                "box_pose_dt": record["box_pose_dt"],
            })

    visible = [record for record in records if record["box_pixels"] >= 25]
    total_box = sum(record["box_pixels"] for record in visible)
    total_intersection = sum(record["intersection_pixels"] for record in visible)
    all_dynamic = sum(record["dynamic_pixels"] for record in records)
    visible_dynamic = sum(record["dynamic_pixels"] for record in visible)
    visible_intersection = sum(record["intersection_pixels"] for record in visible)
    recalls = [record["box_recall"] for record in visible]
    precisions = [record["mask_precision"] for record in visible
                  if math.isfinite(record["mask_precision"])]
    size_quartiles = []
    if visible:
        ordered_visible = sorted(visible, key=lambda item: item["box_pixels"])
        for quartile, indices in enumerate(np.array_split(
                np.arange(len(ordered_visible)), 4), start=1):
            group = [ordered_visible[int(index)] for index in indices]
            group_recalls = [record["box_recall"] for record in group]
            size_quartiles.append({
                "quartile": quartile,
                "frames": len(group),
                "minimum_box_pixels": min(record["box_pixels"] for record in group),
                "maximum_box_pixels": max(record["box_pixels"] for record in group),
                "mean_box_recall": float(np.mean(group_recalls)),
                "median_box_recall": float(np.median(group_recalls)),
            })
    summary = {
        "method": "Gazebo camera/box poses and measured RGB-D visible-volume proxy",
        "interpretation_limit": (
            "The reference is a depth-supported 0.6 m box-volume proxy, not "
            "instance-segmentation ground truth; mask-to-box ratios are diagnostics, "
            "not standard semantic precision."),
        "mask_pattern": args.mask_pattern,
        "dynamic_threshold": args.dynamic_threshold,
        "sample_stride": args.sample_stride,
        "box_half_extent_with_tolerance_m": args.box_half_extent,
        "frames_with_mask": len(records),
        "missing_mask_frames": missing_masks,
        "missing_pose_frames": missing_poses,
        "visible_frames_min_25_sampled_box_pixels": len(visible),
        "box_pixel_weighted_recall": total_intersection/total_box if total_box else math.nan,
        "per_visible_frame_box_recall_mean": float(np.mean(recalls)) if recalls else math.nan,
        "per_visible_frame_box_recall_median": float(np.median(recalls)) if recalls else math.nan,
        "per_visible_frame_box_recall_p10": quantile(recalls, 0.1) if recalls else math.nan,
        "per_visible_frame_box_recall_p90": quantile(recalls, 0.9) if recalls else math.nan,
        "visible_frames_recall_below_0_25": sum(value < 0.25 for value in recalls),
        "visible_frames_recall_above_0_75": sum(value >= 0.75 for value in recalls),
        "all_frame_mask_pixels_belonging_to_box": (
            sum(record["intersection_pixels"] for record in records)/all_dynamic
            if all_dynamic else math.nan),
        "visible_frame_mask_pixels_belonging_to_box": (
            visible_intersection/visible_dynamic if visible_dynamic else math.nan),
        "per_visible_frame_mask_to_box_mean": (
            float(np.mean(precisions)) if precisions else math.nan),
        "nonzero_mask_frames": sum(record["dynamic_pixels"] > 0 for record in records),
        "nonzero_mask_frames_when_box_not_visible": sum(
            record["dynamic_pixels"] > 0 and record["box_pixels"] == 0
            for record in records),
        "box_not_visible_frames": sum(record["box_pixels"] == 0 for record in records),
        "total_dynamic_mask_pixels_at_sample_stride": all_dynamic,
        "mean_global_dynamic_mask_fraction": float(np.mean([
            record["global_mask_fraction"] for record in records])),
        "box_size_quartiles": size_quartiles,
        "maximum_camera_pose_time_difference_s": max(record["camera_pose_dt"] for record in records),
        "maximum_box_pose_time_difference_s": max(record["box_pose_dt"] for record in records),
    }
    (args.output_directory / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    make_contact_sheet(records, root, args.output_directory / "contact_sheet.png")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
