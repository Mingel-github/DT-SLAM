#!/usr/bin/env python3
"""Compare homography and RGB-D/SE(3) ego-motion residuals offline."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

import audit_r1_gazebo_failure_layers as r1


MODELS = (
    "current_homography",
    "oracle_static_homography",
    "gazebo_reference_se3",
    "slam_posterior_se3",
    "slam_constant_velocity_se3",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="R2 read-only ego-motion compensation comparison")
    parser.add_argument("--dataset", required=True, type=r1.existing_path)
    parser.add_argument("--associations", required=True, type=r1.existing_path)
    parser.add_argument("--audit-directory", required=True,
                        type=r1.existing_path)
    parser.add_argument("--slam-trajectory", required=True,
                        type=r1.existing_path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--fx", type=float, default=554.3827128226441)
    parser.add_argument("--fy", type=float, default=554.3827128226441)
    parser.add_argument("--cx", type=float, default=320.5)
    parser.add_argument("--cy", type=float, default=240.5)
    parser.add_argument("--depth-map-factor", type=float, default=5000.0)
    parser.add_argument("--box-half-extent", type=float, default=0.32)
    parser.add_argument("--minimum-visible-box-pixels", type=int, default=100)
    parser.add_argument("--oracle-static-residual-px", type=float, default=0.75)
    parser.add_argument("--oracle-sample-stride", type=int, default=10)
    parser.add_argument("--contact-sheet-count", type=int, default=8)
    return parser.parse_args()


def pose_matrix(translation, quaternion):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = r1.quaternion_rotation(quaternion)
    matrix[:3, 3] = translation
    return matrix


def pose_at(trajectory, timestamp):
    (pose_timestamp, translation, quaternion), delta = r1.nearest_pose(
        trajectory, timestamp)
    return pose_matrix(translation, quaternion), delta, pose_timestamp


def induced_from_homography(homography, shape):
    height, width = shape
    grid_y, grid_x = np.mgrid[0:height, 0:width]
    x = grid_x.astype(np.float64)
    y = grid_y.astype(np.float64)
    denominator = (homography[2, 0] * x + homography[2, 1] * y
                   + homography[2, 2])
    valid = np.isfinite(denominator) & (np.abs(denominator) > 1.0e-12)
    reference_x = np.zeros_like(x)
    reference_y = np.zeros_like(y)
    reference_x[valid] = (
        homography[0, 0] * x[valid] + homography[0, 1] * y[valid]
        + homography[0, 2]) / denominator[valid]
    reference_y[valid] = (
        homography[1, 0] * x[valid] + homography[1, 1] * y[valid]
        + homography[1, 2]) / denominator[valid]
    induced = np.zeros((height, width, 2), dtype=np.float32)
    induced[:, :, 0] = (x - reference_x).astype(np.float32)
    induced[:, :, 1] = (y - reference_y).astype(np.float32)
    valid &= np.isfinite(reference_x) & np.isfinite(reference_y)
    return induced, valid


def induced_from_se3(depth_m, transform_reference_current, rays,
                     fx, fy, cx, cy):
    height, width = depth_m.shape
    depth_valid = np.isfinite(depth_m) & (depth_m > 0.0)
    points_current = rays * depth_m[:, :, None]
    rotation = transform_reference_current[:3, :3]
    translation = transform_reference_current[:3, 3]
    points_reference = points_current @ rotation.T + translation
    z = points_reference[:, :, 2]
    valid = depth_valid & np.isfinite(z) & (z > 1.0e-6)
    reference_x = np.zeros((height, width), dtype=np.float64)
    reference_y = np.zeros((height, width), dtype=np.float64)
    reference_x[valid] = (
        fx * points_reference[:, :, 0][valid] / z[valid] + cx)
    reference_y[valid] = (
        fy * points_reference[:, :, 1][valid] / z[valid] + cy)
    valid &= (reference_x >= 0.0) & (reference_x < width)
    valid &= (reference_y >= 0.0) & (reference_y < height)
    grid_y, grid_x = np.mgrid[0:height, 0:width]
    induced = np.zeros((height, width, 2), dtype=np.float32)
    induced[:, :, 0] = (grid_x - reference_x).astype(np.float32)
    induced[:, :, 1] = (grid_y - reference_y).astype(np.float32)
    return induced, valid


def residual_magnitude(observed_flow, induced_flow, valid):
    difference = observed_flow - induced_flow
    magnitude = np.linalg.norm(difference, axis=2).astype(np.float32)
    magnitude[~valid] = np.nan
    return magnitude


def source_thresholds(residual, valid):
    finite = valid & np.isfinite(residual)
    if not np.any(finite):
        return None
    maximum = float(np.max(residual[finite]))
    if not math.isfinite(maximum) or maximum <= 0.0:
        return None
    normalized = np.zeros(residual.shape, dtype=np.uint8)
    normalized[finite] = np.clip(
        np.rint(residual[finite] * 255.0 / maximum), 0, 255).astype(np.uint8)
    otsu, _ = cv2.threshold(normalized, 80, 255, cv2.THRESH_OTSU)
    triangle, _ = cv2.threshold(normalized, 80, 255, cv2.THRESH_TRIANGLE)
    low = min(float(otsu), float(triangle))
    high = max(float(otsu), float(triangle))
    minimum_low = 1.7 * 255.0 / maximum
    maximum_low = 3.0 * 255.0 / maximum
    low = min(max(low, minimum_low), maximum_low)
    low_mask = finite & (normalized > low)
    if np.count_nonzero(low_mask) > 0.5 * residual.size:
        low += 0.2 * 255.0 / maximum
        low_mask = finite & (normalized > low)
    minimum_high = max(3.0 * 255.0 / maximum, low * 1.2)
    maximum_high = 10.0 * 255.0 / maximum
    high = min(max(high, minimum_high), maximum_high)
    high_mask = finite & (normalized > high)
    return {
        "maximum_residual_px": maximum,
        "low_threshold_px": low * maximum / 255.0,
        "high_threshold_px": high * maximum / 255.0,
        "low_mask": low_mask,
        "high_mask": high_mask,
    }


def homography_from_row(row):
    values = []
    for r_index in range(3):
        for c_index in range(3):
            value = row["h{}{}".format(r_index, c_index)]
            if value == "":
                return None
            values.append(float(value))
    return np.asarray(values, dtype=np.float64).reshape(3, 3)


def fit_oracle_static_homography(observed_flow, gt_residual, gt_valid,
                                 box_mask, stride, threshold):
    height, width = box_mask.shape
    current_points = []
    reference_points = []
    for row in range(stride, height, stride):
        for col in range(stride, width, stride):
            if (not gt_valid[row, col] or box_mask[row, col] != 0
                    or gt_residual[row, col] > threshold):
                continue
            flow = observed_flow[row, col]
            reference_x = col - float(flow[0])
            reference_y = row - float(flow[1])
            if not (0.0 <= reference_x < width and 0.0 <= reference_y < height):
                continue
            current_points.append((float(col), float(row)))
            reference_points.append((reference_x, reference_y))
    if len(current_points) < 4:
        return None, len(current_points)
    homography, _ = cv2.findHomography(
        np.asarray(current_points, dtype=np.float32),
        np.asarray(reference_points, dtype=np.float32), 0)
    if homography is None or homography.shape != (3, 3):
        return None, len(current_points)
    return homography.astype(np.float64), len(current_points)


def constant_velocity_pose(slam_matrices, current_index):
    if current_index < 2:
        return None
    world_camera_previous_previous = slam_matrices[current_index - 2]
    world_camera_previous = slam_matrices[current_index - 1]
    camera_world_previous_previous = np.linalg.inv(
        world_camera_previous_previous)
    camera_world_previous = np.linalg.inv(world_camera_previous)
    velocity = camera_world_previous @ world_camera_previous_previous
    camera_world_prediction = velocity @ camera_world_previous
    return np.linalg.inv(camera_world_prediction)


def masked_stats(values, mask):
    selected = values[mask & np.isfinite(values)]
    if selected.size == 0:
        return {"count": 0, "median": float("nan"), "p90": float("nan")}
    return {
        "count": int(selected.size),
        "median": float(np.median(selected)),
        "p90": float(np.percentile(selected, 90)),
    }


def ratio(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else float("nan")


def model_record(model_name, input_index, timestamp, reference_index,
                 residual, valid, common_valid, box, ring, non_box,
                 oracle_static, thresholds):
    box_stats = masked_stats(residual, valid & box)
    ring_stats = masked_stats(residual, valid & ring)
    non_box_stats = masked_stats(residual, valid & non_box)
    oracle_stats = masked_stats(residual, valid & oracle_static)
    common_stats = masked_stats(residual, common_valid)
    valid_depth = box | non_box
    high = thresholds["high_mask"] if thresholds else np.zeros_like(valid)
    low = thresholds["low_mask"] if thresholds else np.zeros_like(valid)
    return {
        "input_index": input_index,
        "timestamp": timestamp,
        "reference_index": reference_index,
        "model": model_name,
        "valid_pixels": int(np.count_nonzero(valid)),
        "valid_fraction": ratio(np.count_nonzero(valid), valid.size),
        "valid_depth_pixels": int(np.count_nonzero(valid_depth)),
        "valid_over_depth_fraction": ratio(
            np.count_nonzero(valid & valid_depth),
            np.count_nonzero(valid_depth)),
        "common_valid_pixels": int(np.count_nonzero(common_valid)),
        "common_residual_median_px": common_stats["median"],
        "common_residual_p90_px": common_stats["p90"],
        "box_valid_pixels": box_stats["count"],
        "box_residual_median_px": box_stats["median"],
        "box_residual_p90_px": box_stats["p90"],
        "local_ring_residual_median_px": ring_stats["median"],
        "local_ring_residual_p90_px": ring_stats["p90"],
        "non_box_residual_median_px": non_box_stats["median"],
        "non_box_residual_p90_px": non_box_stats["p90"],
        "oracle_static_residual_median_px": oracle_stats["median"],
        "oracle_static_residual_p90_px": oracle_stats["p90"],
        "fixed3_box_fraction": ratio(
            np.count_nonzero(valid & box & (residual > 3.0)),
            np.count_nonzero(valid & box)),
        "fixed3_non_box_fraction": ratio(
            np.count_nonzero(valid & non_box & (residual > 3.0)),
            np.count_nonzero(valid & non_box)),
        "adaptive_low_threshold_px": (
            thresholds["low_threshold_px"] if thresholds else float("nan")),
        "adaptive_high_threshold_px": (
            thresholds["high_threshold_px"] if thresholds else float("nan")),
        "adaptive_low_box_fraction": ratio(
            np.count_nonzero(low & box), np.count_nonzero(valid & box)),
        "adaptive_high_box_fraction": ratio(
            np.count_nonzero(high & box), np.count_nonzero(valid & box)),
        "adaptive_high_non_box_fraction": ratio(
            np.count_nonzero(high & non_box), np.count_nonzero(valid & non_box)),
    }


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def aggregate(records, frame_metadata):
    visible_indices = {
        row["input_index"] for row in frame_metadata
        if row["box_pixels"] >= 100
    }
    summary = {}
    for model in MODELS:
        rows = [row for row in records if row["model"] == model]
        visible = [row for row in rows if row["input_index"] in visible_indices]
        summary[model] = {
            "frames": len(rows),
            "valid_fraction": r1.finite_stats(
                [row["valid_fraction"] for row in rows]),
            "valid_over_depth_fraction": r1.finite_stats(
                [row["valid_over_depth_fraction"] for row in rows]),
            "common_residual_median_px": r1.finite_stats(
                [row["common_residual_median_px"] for row in rows]),
            "oracle_static_residual_median_px": r1.finite_stats(
                [row["oracle_static_residual_median_px"] for row in rows]),
            "visible_box_residual_median_px": r1.finite_stats(
                [row["box_residual_median_px"] for row in visible]),
            "visible_local_ring_residual_median_px": r1.finite_stats(
                [row["local_ring_residual_median_px"] for row in visible]),
            "visible_fixed3_box_fraction": r1.finite_stats(
                [row["fixed3_box_fraction"] for row in visible]),
            "visible_adaptive_high_box_fraction": r1.finite_stats(
                [row["adaptive_high_box_fraction"] for row in visible]),
            "visible_adaptive_high_non_box_fraction": r1.finite_stats(
                [row["adaptive_high_non_box_fraction"] for row in visible]),
        }
    return summary


def residual_color(residual, valid, maximum=8.0):
    normalized = np.zeros(residual.shape, dtype=np.uint8)
    finite = valid & np.isfinite(residual)
    normalized[finite] = np.clip(
        np.rint(residual[finite] * 255.0 / maximum), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


def main():
    args = parse_args()
    output = args.output_directory.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit("output directory is not empty: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)

    associations = r1.load_associations(args.associations)
    frame_rows = r1.read_csv_rows(args.audit_directory / "r1_frame_index.csv")
    camera_gt = r1.load_trajectory(args.dataset / "groundtruth.txt")
    box_gt = r1.load_trajectory(args.dataset / "box_groundtruth.txt")
    slam_trajectory = r1.load_trajectory(args.slam_trajectory)
    height, width = 480, 640
    grid_y, grid_x = np.mgrid[0:height, 0:width]
    rays = np.stack([
        (grid_x - args.cx) / args.fx,
        (grid_y - args.cy) / args.fy,
        np.ones((height, width), dtype=np.float64),
    ], axis=-1)

    # Direction sanity check: identity has zero induced flow; translating a
    # current-frame point +x in the reference camera moves its reference pixel
    # right, so current-minus-reference is negative x.
    synthetic_depth = np.full((3, 3), 2.0, dtype=np.float64)
    synthetic_rays = np.stack([
        (np.mgrid[0:3, 0:3][1] - 1.0) / args.fx,
        (np.mgrid[0:3, 0:3][0] - 1.0) / args.fy,
        np.ones((3, 3), dtype=np.float64),
    ], axis=-1)
    identity_flow, identity_valid = induced_from_se3(
        synthetic_depth, np.eye(4), synthetic_rays,
        args.fx, args.fy, 1.0, 1.0)
    if not np.all(identity_valid) or np.max(np.abs(identity_flow)) > 1.0e-6:
        raise RuntimeError("identity SE3 direction invariant failed")
    translation = np.eye(4)
    translation[0, 3] = 0.1
    translation_flow, _ = induced_from_se3(
        synthetic_depth, translation, synthetic_rays,
        args.fx, args.fy, 1.0, 1.0)
    if not translation_flow[1, 1, 0] < 0.0:
        raise RuntimeError("SE3 translation direction invariant failed")

    slam_matrices = []
    slam_pose_deltas = []
    for association in associations:
        matrix, delta, _ = pose_at(slam_trajectory, association["timestamp"])
        slam_matrices.append(matrix)
        slam_pose_deltas.append(delta)

    records = []
    frame_metadata = []
    residual_cache = {}
    current_h_recompute_errors = []
    oracle_sample_counts = []
    for frame_row in frame_rows:
        if int(frame_row["flow_available"]) == 0:
            continue
        input_index = int(frame_row["input_index"])
        reference_index = int(frame_row["reference_index"])
        association = associations[input_index]
        reference_association = associations[reference_index]
        timestamp = association["timestamp"]
        depth_raw = cv2.imread(
            str(args.dataset / association["depth"]), cv2.IMREAD_UNCHANGED)
        depth_m = depth_raw.astype(np.float64) / args.depth_map_factor
        depth_valid = np.isfinite(depth_m) & (depth_m > 0.0)
        observed = r1.read_flo(r1.frame_path(
            args.audit_directory, "flow", input_index, "_observed.flo"))

        camera_current_pose_row, camera_current_dt = r1.nearest_pose(
            camera_gt, timestamp)
        camera_current = pose_matrix(
            camera_current_pose_row[1], camera_current_pose_row[2])
        camera_reference, camera_reference_dt, _ = pose_at(
            camera_gt, reference_association["timestamp"])
        box_current_row, box_current_dt = r1.nearest_pose(box_gt, timestamp)
        box_mask = r1.visible_box_mask(
            depth_m, (camera_current_pose_row[1], camera_current_pose_row[2]),
            (box_current_row[1], box_current_row[2]), rays,
            args.box_half_extent)
        box = box_mask != 0
        non_box = depth_valid & ~box
        ring = cv2.dilate(
            box_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)))
        ring = (ring != 0) & ~box & depth_valid

        transform_gt = np.linalg.inv(camera_reference) @ camera_current
        induced_gt, valid_gt = induced_from_se3(
            depth_m, transform_gt, rays,
            args.fx, args.fy, args.cx, args.cy)
        residual_gt = residual_magnitude(observed, induced_gt, valid_gt)
        oracle_static = (valid_gt & non_box & np.isfinite(residual_gt)
                         & (residual_gt <= args.oracle_static_residual_px))

        current_h = homography_from_row(frame_row)
        if current_h is None:
            raise RuntimeError("missing current homography at {}".format(input_index))
        induced_current_h, valid_current_h = induced_from_homography(
            current_h, depth_m.shape)
        residual_current_h = residual_magnitude(
            observed, induced_current_h, valid_current_h)
        saved_q64 = r1.load_mask(r1.frame_path(
            args.audit_directory, "residual", input_index,
            "_magnitude_q64.png"), depth_m.shape).astype(np.float32) / 64.0
        current_h_recompute_errors.append(float(np.nanmax(
            np.abs(residual_current_h - saved_q64))))

        oracle_h, sample_count = fit_oracle_static_homography(
            observed, residual_gt, valid_gt, box_mask,
            args.oracle_sample_stride, args.oracle_static_residual_px)
        oracle_sample_counts.append(sample_count)

        slam_current = slam_matrices[input_index]
        slam_reference = slam_matrices[reference_index]
        transform_slam = np.linalg.inv(slam_reference) @ slam_current
        induced_slam, valid_slam = induced_from_se3(
            depth_m, transform_slam, rays,
            args.fx, args.fy, args.cx, args.cy)

        predicted_current = constant_velocity_pose(slam_matrices, input_index)
        if predicted_current is not None:
            transform_prediction = np.linalg.inv(slam_reference) @ predicted_current
            induced_prediction, valid_prediction = induced_from_se3(
                depth_m, transform_prediction, rays,
                args.fx, args.fy, args.cx, args.cy)
        else:
            induced_prediction = np.zeros_like(observed)
            valid_prediction = np.zeros(depth_m.shape, dtype=bool)

        model_data = {
            "current_homography": (induced_current_h, valid_current_h),
            "gazebo_reference_se3": (induced_gt, valid_gt),
            "slam_posterior_se3": (induced_slam, valid_slam),
            "slam_constant_velocity_se3": (
                induced_prediction, valid_prediction),
        }
        if oracle_h is not None:
            model_data["oracle_static_homography"] = induced_from_homography(
                oracle_h, depth_m.shape)
        else:
            model_data["oracle_static_homography"] = (
                np.zeros_like(observed), np.zeros(depth_m.shape, dtype=bool))

        residuals = {
            name: residual_magnitude(observed, induced, valid)
            for name, (induced, valid) in model_data.items()
        }
        common_valid = depth_valid.copy()
        for _, valid in model_data.values():
            common_valid &= valid

        frame_metadata.append({
            "input_index": input_index,
            "timestamp": timestamp,
            "reference_index": reference_index,
            "reference_lag": input_index - reference_index,
            "box_pixels": int(np.count_nonzero(box)),
            "box_depth_median": (float(np.median(depth_m[box]))
                                   if np.any(box) else float("nan")),
            "oracle_static_pixels": int(np.count_nonzero(oracle_static)),
            "oracle_homography_samples": sample_count,
            "camera_current_pose_dt_s": camera_current_dt,
            "camera_reference_pose_dt_s": camera_reference_dt,
            "box_pose_dt_s": box_current_dt,
            "slam_current_pose_dt_s": slam_pose_deltas[input_index],
            "slam_reference_pose_dt_s": slam_pose_deltas[reference_index],
            "common_valid_pixels": int(np.count_nonzero(common_valid)),
        })
        for model_name in MODELS:
            induced, valid = model_data[model_name]
            residual = residuals[model_name]
            thresholds = source_thresholds(residual, valid)
            records.append(model_record(
                model_name, input_index, timestamp, reference_index,
                residual, valid, common_valid, box, ring, non_box,
                oracle_static, thresholds))
        residual_cache[input_index] = (box_mask, residuals, model_data)

    if not records:
        raise RuntimeError("no R2 records")
    write_csv(output / "r2_ego_compensation_per_model.csv", records)
    write_csv(output / "r2_frame_metadata.csv", frame_metadata)
    summary = {
        "models": aggregate(records, frame_metadata),
        "frames_with_flow": len(frame_metadata),
        "current_homography_recompute_max_abs_px": (
            max(current_h_recompute_errors)),
        "current_homography_recompute_median_abs_px": (
            float(np.median(current_h_recompute_errors))),
        "oracle_homography_sample_count": r1.finite_stats(oracle_sample_counts),
        "oracle_static_definition": (
            "non-box pixels whose observed flow agrees with Gazebo reference "
            "SE3 within {:.3f}px; oracle-only and not deployable".format(
                args.oracle_static_residual_px)),
        "slam_posterior_limit": (
            "The current SIn detector runs before Track(); posterior SLAM pose "
            "is diagnostic only."),
        "constant_velocity_limit": (
            "Offline proxy built only from the preceding two posterior poses; "
            "it does not claim exact equality with every ORB-SLAM2 runtime state."),
        "non_box_limit": "Non-box can include the moving person.",
        "flow_direction": "current_minus_reference",
        "se3_transform_direction": "reference_camera_from_current_camera",
    }
    (output / "r2_ego_compensation_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8")

    visible = [row for row in frame_metadata if row["box_pixels"] >= 100]
    selected = []
    if visible:
        ordered = sorted(visible, key=lambda row: row["box_pixels"])
        for value in np.linspace(0, len(ordered) - 1,
                                 min(args.contact_sheet_count, len(ordered))):
            selected.append(ordered[int(round(value))])
    panels = []
    for metadata in selected:
        input_index = metadata["input_index"]
        box_mask, residuals, model_data = residual_cache[input_index]
        row_panels = []
        for model_name in MODELS:
            residual = residuals[model_name]
            valid = model_data[model_name][1]
            panel = residual_color(residual, valid)
            contours, _ = cv2.findContours(
                box_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(panel, contours, -1, (0, 255, 0), 2)
            cv2.putText(panel, model_name, (6, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(panel, model_name, (6, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 0, 0), 1, cv2.LINE_AA)
            row_panels.append(panel)
        panels.append(np.hstack(row_panels))
    if panels:
        cv2.imwrite(str(output / "r2_ego_compensation_contact_sheet.png"),
                    np.vstack(panels))

    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=True))


if __name__ == "__main__":
    main()
