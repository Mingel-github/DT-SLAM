#!/usr/bin/env python3
"""Replay SInDSLAM Eq. (17)-style long-interval mapping refinement."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

import audit_r1_gazebo_failure_layers as r1
import audit_r2_ego_compensation as r2


def parse_args():
    parser = argparse.ArgumentParser(
        description="R4 mapping-only long-interval depth refinement")
    parser.add_argument("--dataset", required=True, type=r1.existing_path)
    parser.add_argument("--associations", required=True,
                        type=r1.existing_path)
    parser.add_argument("--audit-directory", required=True,
                        type=r1.existing_path)
    parser.add_argument("--slam-trajectory", required=True,
                        type=r1.existing_path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--frame-step", type=int, default=5)
    parser.add_argument("--relative-depth-threshold", type=float, default=0.13)
    parser.add_argument("--region-seed-fraction", type=float, default=0.40)
    parser.add_argument("--maximum-depth-m", type=float, default=6.0)
    parser.add_argument("--fx", type=float, default=554.3827128226441)
    parser.add_argument("--fy", type=float, default=554.3827128226441)
    parser.add_argument("--cx", type=float, default=320.5)
    parser.add_argument("--cy", type=float, default=240.5)
    parser.add_argument("--depth-map-factor", type=float, default=5000.0)
    parser.add_argument("--box-half-extent", type=float, default=0.32)
    parser.add_argument("--minimum-visible-box-pixels", type=int, default=100)
    parser.add_argument("--contact-sheet-count", type=int, default=10)
    return parser.parse_args()


def ratio(numerator, denominator):
    return (float(numerator) / float(denominator)
            if denominator else float("nan"))


def finite_stats(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"count": 0, "mean": None, "median": None,
                "p10": None, "p90": None}
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p10": float(np.percentile(values, 10)),
        "p90": float(np.percentile(values, 90)),
    }


def load_depth(dataset, association, factor):
    raw = cv2.imread(str(dataset / association["depth"]),
                     cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise ValueError("cannot read depth: {}".format(association["depth"]))
    return raw.astype(np.float64) / factor


def project_current_to_reference(depth_current, transform_reference_current,
                                 rays, fx, fy, cx, cy, maximum_depth):
    height, width = depth_current.shape
    current_valid = (np.isfinite(depth_current)
                     & (depth_current > 0.0)
                     & (depth_current <= maximum_depth))
    points_current = rays * depth_current[:, :, None]
    rotation = transform_reference_current[:3, :3]
    translation = transform_reference_current[:3, 3]
    points_reference = points_current @ rotation.T + translation
    z = points_reference[:, :, 2]
    valid = current_valid & np.isfinite(z) & (z > 1.0e-6)
    reference_x = np.zeros(depth_current.shape, dtype=np.float64)
    reference_y = np.zeros(depth_current.shape, dtype=np.float64)
    reference_x[valid] = (
        fx * points_reference[:, :, 0][valid] / z[valid] + cx)
    reference_y[valid] = (
        fy * points_reference[:, :, 1][valid] / z[valid] + cy)
    valid &= (reference_x >= 0.0) & (reference_x <= width - 1)
    valid &= (reference_y >= 0.0) & (reference_y <= height - 1)
    reference_col = np.clip(np.rint(reference_x), 0, width - 1).astype(np.int32)
    reference_row = np.clip(np.rint(reference_y), 0, height - 1).astype(np.int32)
    return valid, reference_row, reference_col


def refine_regions(seed, labels, valid, fraction_threshold):
    added = seed.copy()
    expanded_regions = 0
    region_records = []
    for label in np.unique(labels[valid]):
        label = int(label)
        if label <= 0:
            continue
        region = valid & (labels == label)
        region_pixels = int(np.count_nonzero(region))
        seed_pixels = int(np.count_nonzero(seed & region))
        seed_fraction = ratio(seed_pixels, region_pixels)
        expanded = bool(seed_fraction > fraction_threshold)
        if expanded:
            added |= region
            expanded_regions += 1
        region_records.append({
            "region_label": label,
            "region_pixels": region_pixels,
            "seed_pixels": seed_pixels,
            "seed_fraction": seed_fraction,
            "expanded_whole_region": int(expanded),
        })
    return added, expanded_regions, region_records


def overlay(rgb, box, current_mask, seed, refined):
    output = rgb.copy()
    output[current_mask] = (
        0.55 * output[current_mask] + 0.45 * np.array([0, 0, 255])
    ).astype(np.uint8)
    output[seed] = (0, 255, 255)
    new_refined = refined & ~current_mask & ~seed
    output[new_refined] = (255, 0, 255)
    contours, _ = cv2.findContours(
        box.astype(np.uint8) * 255, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(output, contours, -1, (0, 255, 0), 2)
    return output


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    if args.frame_step < 1:
        raise SystemExit("frame-step must be positive")
    output = args.output_directory.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit("output directory is not empty: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)
    seed_directory = output / "long_interval_seeds"
    refined_directory = output / "refined_mapping_masks"
    seed_directory.mkdir()
    refined_directory.mkdir()

    associations = r1.load_associations(args.associations)
    slam_trajectory = r1.load_trajectory(args.slam_trajectory)
    camera_gt = r1.load_trajectory(args.dataset / "groundtruth.txt")
    box_gt = r1.load_trajectory(args.dataset / "box_groundtruth.txt")
    height, width = 480, 640
    grid_y, grid_x = np.mgrid[0:height, 0:width]
    rays = np.stack([
        (grid_x - args.cx) / args.fx,
        (grid_y - args.cy) / args.fy,
        np.ones((height, width), dtype=np.float64),
    ], axis=-1)

    # Identity must project every valid pixel back to itself.
    synthetic = np.full((5, 7), 2.0, dtype=np.float64)
    sy, sx = np.mgrid[0:5, 0:7]
    synthetic_rays = np.stack([
        (sx - 3.0) / args.fx,
        (sy - 2.0) / args.fy,
        np.ones((5, 7), dtype=np.float64),
    ], axis=-1)
    identity_valid, identity_row, identity_col = project_current_to_reference(
        synthetic, np.eye(4), synthetic_rays,
        args.fx, args.fy, 3.0, 2.0, args.maximum_depth_m)
    if (not np.all(identity_valid)
            or not np.array_equal(identity_row, sy)
            or not np.array_equal(identity_col, sx)):
        raise RuntimeError("identity reprojection invariant failed")

    slam_poses = []
    for association in associations:
        pose, _, _ = r2.pose_at(slam_trajectory, association["timestamp"])
        slam_poses.append(pose)

    frame_records = []
    region_records = []
    contact_candidates = []
    sparse_indices = list(range(0, len(associations), args.frame_step))
    for sparse_position in range(1, len(sparse_indices)):
        current_index = sparse_indices[sparse_position]
        reference_index = sparse_indices[sparse_position - 1]
        current_association = associations[current_index]
        reference_association = associations[reference_index]
        current_depth = load_depth(
            args.dataset, current_association, args.depth_map_factor)
        reference_depth = load_depth(
            args.dataset, reference_association, args.depth_map_factor)
        current_dynamic = r1.load_mask(r1.frame_path(
            args.audit_directory, "classifier", current_index,
            "_dynamic.png"), (height, width)) != 0
        labels = r1.decoded_labels(r1.frame_path(
            args.audit_directory, "labels", current_index,
            "_rag_plus1.png"), (height, width))

        transform_reference_current = (
            np.linalg.inv(slam_poses[reference_index])
            @ slam_poses[current_index])
        projection_valid, reference_row, reference_col = (
            project_current_to_reference(
                current_depth, transform_reference_current, rays,
                args.fx, args.fy, args.cx, args.cy,
                args.maximum_depth_m))
        sampled_reference = reference_depth[reference_row, reference_col]
        valid = (projection_valid
                 & np.isfinite(sampled_reference)
                 & (sampled_reference > 0.0)
                 & (sampled_reference <= args.maximum_depth_m))
        difference = np.full(current_depth.shape, np.nan, dtype=np.float32)
        difference[valid] = np.abs(
            current_depth[valid] - sampled_reference[valid]).astype(np.float32)
        seed = (valid
                & ~current_dynamic
                & (difference > args.relative_depth_threshold * current_depth))
        long_added, expanded_regions, per_region = refine_regions(
            seed, labels, valid, args.region_seed_fraction)
        refined = current_dynamic | long_added

        timestamp = current_association["timestamp"]
        camera_pose_row, camera_dt = r1.nearest_pose(camera_gt, timestamp)
        box_pose_row, box_dt = r1.nearest_pose(box_gt, timestamp)
        box_mask = r1.visible_box_mask(
            current_depth,
            (camera_pose_row[1], camera_pose_row[2]),
            (box_pose_row[1], box_pose_row[2]),
            rays, args.box_half_extent) != 0
        box_pixels = int(np.count_nonzero(box_mask))
        valid_depth = (np.isfinite(current_depth)
                       & (current_depth > 0.0)
                       & (current_depth <= args.maximum_depth_m))
        non_box = valid_depth & ~box_mask
        new_pixels = refined & ~current_dynamic
        record = {
            "current_index": current_index,
            "reference_index": reference_index,
            "frame_lag": current_index - reference_index,
            "timestamp": timestamp,
            "valid_depth_pixels": int(np.count_nonzero(valid_depth)),
            "comparison_pixels": int(np.count_nonzero(valid)),
            "comparison_coverage": ratio(
                np.count_nonzero(valid), np.count_nonzero(valid_depth)),
            "current_dynamic_pixels": int(np.count_nonzero(current_dynamic)),
            "long_seed_pixels": int(np.count_nonzero(seed)),
            "long_added_pixels": int(np.count_nonzero(new_pixels)),
            "refined_dynamic_pixels": int(np.count_nonzero(refined)),
            "expanded_regions": expanded_regions,
            "box_pixels": box_pixels,
            "box_depth_median": (float(np.median(current_depth[box_mask]))
                                   if box_pixels else float("nan")),
            "current_box_coverage": ratio(
                np.count_nonzero(current_dynamic & box_mask), box_pixels),
            "seed_box_coverage": ratio(
                np.count_nonzero(seed & box_mask), box_pixels),
            "new_box_coverage": ratio(
                np.count_nonzero(new_pixels & box_mask), box_pixels),
            "refined_box_coverage": ratio(
                np.count_nonzero(refined & box_mask), box_pixels),
            "new_non_box_fraction": ratio(
                np.count_nonzero(new_pixels & non_box),
                np.count_nonzero(non_box)),
            "camera_gt_dt_s": camera_dt,
            "box_gt_dt_s": box_dt,
        }
        frame_records.append(record)
        for region in per_region:
            region_records.append({
                "current_index": current_index,
                "reference_index": reference_index,
                **region,
            })

        cv2.imwrite(str(seed_directory /
                        "frame_{:06d}_long_seed.png".format(current_index)),
                    seed.astype(np.uint8) * 255)
        cv2.imwrite(str(refined_directory /
                        "frame_{:06d}_refined_mapping_mask.png".format(
                            current_index)), refined.astype(np.uint8) * 255)
        if box_pixels >= args.minimum_visible_box_pixels:
            contact_candidates.append((record, box_mask, current_dynamic,
                                       seed, refined, current_association))

    write_csv(output / "r4_frame_metrics.csv", frame_records)
    write_csv(output / "r4_region_metrics.csv", region_records)

    visible = [row for row in frame_records
               if row["box_pixels"] >= args.minimum_visible_box_pixels]
    missed = [row for row in visible if row["current_box_coverage"] < 0.10]
    summary = {
        "identity_reprojection_invariant": "passed",
        "algorithm_identity": "paper-text-guided clean-room S4 replay",
        "frame_step": args.frame_step,
        "relative_depth_threshold": args.relative_depth_threshold,
        "region_seed_fraction": args.region_seed_fraction,
        "maximum_depth_m": args.maximum_depth_m,
        "sparse_comparisons": len(frame_records),
        "visible_box_sparse_frames": len(visible),
        "missed_by_current_s3_frames": len(missed),
        "missed_with_any_new_box_evidence": sum(
            row["new_box_coverage"] > 0.0 for row in missed),
        "missed_recovered_to_25_percent": sum(
            row["refined_box_coverage"] >= 0.25 for row in missed),
        "comparison_coverage": finite_stats(
            [row["comparison_coverage"] for row in frame_records]),
        "long_added_fraction_of_valid_depth": finite_stats([
            ratio(row["long_added_pixels"], row["valid_depth_pixels"])
            for row in frame_records]),
        "new_non_box_fraction": finite_stats(
            [row["new_non_box_fraction"] for row in frame_records]),
        "visible_current_box_coverage": finite_stats(
            [row["current_box_coverage"] for row in visible]),
        "visible_new_box_coverage": finite_stats(
            [row["new_box_coverage"] for row in visible]),
        "visible_refined_box_coverage": finite_stats(
            [row["refined_box_coverage"] for row in visible]),
        "non_box_limit": "Non-box can include the moving person.",
        "unknown_semantics": (
            "Invalid depth, out-of-image projection, and missing reference "
            "depth are excluded rather than interpreted as static."),
    }
    (output / "r4_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8")

    if contact_candidates:
        ordered = sorted(contact_candidates,
                         key=lambda item: item[0]["box_pixels"])
        selected = []
        for value in np.linspace(
                0, len(ordered) - 1,
                min(args.contact_sheet_count, len(ordered))):
            selected.append(ordered[int(round(value))])
        panels = []
        for record, box, current, seed, refined, association in selected:
            rgb = cv2.imread(str(args.dataset / association["rgb"]),
                             cv2.IMREAD_COLOR)
            panel = overlay(rgb, box, current, seed, refined)
            label = "frame {} depth {:.2f}m S3 {:.1f}% refined {:.1f}%".format(
                record["current_index"], record["box_depth_median"],
                100.0 * record["current_box_coverage"],
                100.0 * record["refined_box_coverage"])
            cv2.putText(panel, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(panel, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 0), 1, cv2.LINE_AA)
            panels.append(panel)
        cv2.imwrite(str(output / "r4_contact_sheet.png"), np.vstack(panels))

    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=True))


if __name__ == "__main__":
    main()
