#!/usr/bin/env python3
"""Select Bonn moving-box development/review candidates from risk proxies.

The selected strata are review proxies, not box-motion labels or dynamic GT.
Because selection is conditioned on geometry evidence, the selected subset is
not an unbiased hold-out evaluation set.
"""

import argparse
import bisect
import csv
import json
import math
import pathlib
import sys

import cv2
import numpy as np


BONN_K = np.array(
    [
        [542.822841, 0.0, 315.593520],
        [0.0, 542.576870, 237.756098],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
BONN_D = np.array(
    [0.039903, -0.099343, -0.000730, -0.000144, 0.0],
    dtype=np.float64,
)
DEPTH_FACTOR = 5000.0
RESIDUAL_THRESHOLD_M = 0.10
RELATIVE_BOUNDARY_THRESHOLD = 0.025
ABSOLUTE_BOUNDARY_THRESHOLD_M = 0.08


def read_association(path, dataset_root):
    records = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(
                    "{}:{}: expected four association fields".format(
                        path, line_number
                    )
                )
            record = {
                "frame": len(records),
                "rgb_timestamp": float(fields[0]),
                "rgb_relative": fields[1],
                "depth_timestamp": float(fields[2]),
                "depth_relative": fields[3],
            }
            if not (dataset_root / record["rgb_relative"]).is_file():
                raise FileNotFoundError(dataset_root / record["rgb_relative"])
            if not (dataset_root / record["depth_relative"]).is_file():
                raise FileNotFoundError(dataset_root / record["depth_relative"])
            records.append(record)
    if not records:
        raise ValueError("{} contains no association pairs".format(path))
    return records


def read_ground_truth(path):
    samples = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 8:
                raise ValueError(
                    "{}:{}: expected timestamp, t and quaternion".format(
                        path, line_number
                    )
                )
            values = [float(value) for value in fields]
            quaternion = np.asarray(values[4:8], dtype=np.float64)
            norm = np.linalg.norm(quaternion)
            if not np.isfinite(norm) or norm <= 1e-12:
                raise ValueError(
                    "{}:{}: invalid quaternion".format(path, line_number)
                )
            samples.append(
                {
                    "timestamp": values[0],
                    "translation": np.asarray(values[1:4], dtype=np.float64),
                    "quaternion_xyzw": quaternion / norm,
                }
            )
    if not samples:
        raise ValueError("{} contains no poses".format(path))
    samples.sort(key=lambda sample: sample["timestamp"])
    return samples


def quaternion_slerp(left, right, alpha):
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = left + alpha * (right - left)
        return result / np.linalg.norm(result)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    return (
        math.sin((1.0 - alpha) * theta) / sin_theta * left
        + math.sin(alpha * theta) / sin_theta * right
    )


def quaternion_to_rotation(quaternion_xyzw):
    x, y, z, w = quaternion_xyzw
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def make_pose_twc(translation, quaternion_xyzw):
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = quaternion_to_rotation(quaternion_xyzw)
    pose[:3, 3] = translation
    return pose


def interpolate_twc(samples, timestamps, timestamp, max_delta_seconds):
    upper = bisect.bisect_left(timestamps, timestamp)
    if upper < len(samples) and abs(timestamps[upper] - timestamp) <= 1e-9:
        sample = samples[upper]
        return make_pose_twc(
            sample["translation"], sample["quaternion_xyzw"]
        )
    if upper == 0 or upper == len(samples):
        return None
    before = samples[upper - 1]
    after = samples[upper]
    before_delta = timestamp - before["timestamp"]
    after_delta = after["timestamp"] - timestamp
    interval = after["timestamp"] - before["timestamp"]
    if (
        before_delta < 0.0
        or after_delta < 0.0
        or interval <= 0.0
        or before_delta > max_delta_seconds
        or after_delta > max_delta_seconds
    ):
        return None
    alpha = before_delta / interval
    translation = (
        (1.0 - alpha) * before["translation"] + alpha * after["translation"]
    )
    quaternion = quaternion_slerp(
        before["quaternion_xyzw"], after["quaternion_xyzw"], alpha
    )
    return make_pose_twc(translation, quaternion)


def compute_warp_proxy(
    reference_depth_m,
    current_depth_m,
    twc_reference,
    twc_current,
    camera_matrix,
    scale,
):
    rows = (reference_depth_m.shape[0] + scale - 1) // scale
    cols = (reference_depth_m.shape[1] + scale - 1) // scale
    sample_v = np.arange(rows, dtype=np.int32) * scale
    sample_u = np.arange(cols, dtype=np.int32) * scale
    grid_u, grid_v = np.meshgrid(sample_u, sample_v)
    reference = reference_depth_m[grid_v, grid_u]
    valid_reference = np.isfinite(reference) & (reference > 0.0)

    predicted = np.full((rows, cols), np.inf, dtype=np.float64)
    if np.any(valid_reference):
        z = reference[valid_reference].astype(np.float64)
        u = grid_u[valid_reference].astype(np.float64)
        v = grid_v[valid_reference].astype(np.float64)
        points = np.vstack(
            (
                (u - camera_matrix[0, 2]) * z / camera_matrix[0, 0],
                (v - camera_matrix[1, 2]) * z / camera_matrix[1, 1],
                z,
                np.ones_like(z),
            )
        )
        current_from_reference = np.linalg.inv(twc_current) @ twc_reference
        current_points = current_from_reference @ points
        current_z = current_points[2]
        projectable = np.isfinite(current_z) & (current_z > 0.0)
        projected_u = np.full_like(current_z, np.nan)
        projected_v = np.full_like(current_z, np.nan)
        projected_u[projectable] = (
            camera_matrix[0, 0]
            * current_points[0, projectable]
            / current_z[projectable]
            + camera_matrix[0, 2]
        )
        projected_v[projectable] = (
            camera_matrix[1, 1]
            * current_points[1, projectable]
            / current_z[projectable]
            + camera_matrix[1, 2]
        )
        target_u = np.rint(projected_u / scale).astype(
            np.int64, casting="unsafe"
        )
        target_v = np.rint(projected_v / scale).astype(
            np.int64, casting="unsafe"
        )
        in_bounds = (
            projectable
            & (target_u >= 0)
            & (target_u < cols)
            & (target_v >= 0)
            & (target_v < rows)
        )
        flat_indices = target_v[in_bounds] * cols + target_u[in_bounds]
        np.minimum.at(
            predicted.reshape(-1), flat_indices, current_z[in_bounds]
        )

    current = current_depth_m[grid_v, grid_u].astype(np.float64)
    valid_current = np.isfinite(current) & (current > 0.0)
    valid_comparison = np.isfinite(predicted) & valid_current
    residual = np.full((rows, cols), np.nan, dtype=np.float64)
    residual[valid_comparison] = (
        predicted[valid_comparison] - current[valid_comparison]
    )
    return residual, valid_comparison


def depth_boundary_ratio(depth_m, scale):
    sampled = depth_m[::scale, ::scale]
    valid = np.isfinite(sampled) & (sampled > 0.0)
    boundary = np.zeros(sampled.shape, dtype=bool)

    left = sampled[:, :-1]
    right = sampled[:, 1:]
    pair_valid = valid[:, :-1] & valid[:, 1:]
    threshold = np.maximum(
        ABSOLUTE_BOUNDARY_THRESHOLD_M,
        RELATIVE_BOUNDARY_THRESHOLD * np.minimum(left, right),
    )
    edge = pair_valid & (np.abs(left - right) > threshold)
    boundary[:, :-1] |= edge
    boundary[:, 1:] |= edge

    top = sampled[:-1, :]
    bottom = sampled[1:, :]
    pair_valid = valid[:-1, :] & valid[1:, :]
    threshold = np.maximum(
        ABSOLUTE_BOUNDARY_THRESHOLD_M,
        RELATIVE_BOUNDARY_THRESHOLD * np.minimum(top, bottom),
    )
    edge = pair_valid & (np.abs(top - bottom) > threshold)
    boundary[:-1, :] |= edge
    boundary[1:, :] |= edge
    return float(np.count_nonzero(boundary) / boundary.size)


def pose_motion(twc_reference, twc_current):
    current_from_reference = np.linalg.inv(twc_current) @ twc_reference
    translation = float(np.linalg.norm(current_from_reference[:3, 3]))
    cosine = (np.trace(current_from_reference[:3, :3]) - 1.0) / 2.0
    cosine = min(1.0, max(-1.0, float(cosine)))
    rotation_degrees = math.degrees(math.acos(cosine))
    return translation, rotation_degrees


def finite_or_none(value):
    if value is None or not math.isfinite(float(value)):
        return None
    return float(value)


def select_ranked(
    metrics,
    key,
    reverse,
    count,
    globally_selected,
    appearance_by_frame,
):
    eligible = [
        row
        for row in metrics
        if row["eligible"]
        and row["frame"] not in globally_selected
        and finite_or_none(row[key]) is not None
    ]
    eligible.sort(
        key=lambda row: (row[key], -row["frame"])
        if reverse
        else (row[key], row["frame"]),
        reverse=reverse,
    )

    selected = []
    diversity_rounds = (
        (20, 5, 0.050),
        (10, 3, 0.025),
        (5, 1, 0.0),
        (0, 0, 0.0),
    )
    for same_role_gap, global_gap, minimum_appearance_distance in diversity_rounds:
        for row in eligible:
            if len(selected) >= count:
                break
            if row["frame"] in globally_selected:
                continue
            if any(
                abs(row["frame"] - other["frame"]) < same_role_gap
                for other in selected
            ):
                continue
            if any(
                abs(row["frame"] - frame) < global_gap
                for frame in globally_selected
            ):
                continue
            if selected and minimum_appearance_distance > 0.0:
                appearance = appearance_by_frame[row["frame"]]
                nearest_selected_distance = min(
                    float(
                        np.mean(
                            np.abs(
                                appearance
                                - appearance_by_frame[other["frame"]]
                            )
                        )
                    )
                    for other in selected
                )
                if nearest_selected_distance < minimum_appearance_distance:
                    continue
            selected.append(row)
            globally_selected.add(row["frame"])
        if len(selected) >= count:
            break
    return selected


def percentile_summary(values):
    array = np.asarray(values, dtype=np.float64)
    return {
        "min": float(np.min(array)),
        "p10": float(np.percentile(array, 10)),
        "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)),
        "max": float(np.max(array)),
    }


def selected_role_diversity(selected, appearance_by_frame):
    result = {}
    roles = sorted({row["selection_role"] for row in selected})
    for role in roles:
        rows = [
            row for row in selected if row["selection_role"] == role
        ]
        temporal_gaps = []
        appearance_distances = []
        for left_index, left in enumerate(rows):
            for right in rows[left_index + 1 :]:
                temporal_gaps.append(abs(left["frame"] - right["frame"]))
                appearance_distances.append(
                    float(
                        np.mean(
                            np.abs(
                                appearance_by_frame[left["frame"]]
                                - appearance_by_frame[right["frame"]]
                            )
                        )
                    )
                )
        result[role] = {
            "minimum_pairwise_frame_gap": (
                min(temporal_gaps) if temporal_gaps else None
            ),
            "minimum_pairwise_thumbnail_mad": (
                min(appearance_distances)
                if appearance_distances
                else None
            ),
        }
    return result


def save_contact_sheet(
    dataset_root, selected, map_x, map_y, output_path
):
    tile_width = 240
    tile_height = 180
    label_height = 52
    roles = [
        "proxy_high_inconsistency",
        "proxy_transition",
        "proxy_geometry_difficult",
        "proxy_low_inconsistency",
    ]
    canvas = np.zeros(
        (len(roles) * (tile_height + label_height), 6 * tile_width, 3),
        dtype=np.uint8,
    )
    role_rows = {role: index for index, role in enumerate(roles)}
    for row in selected:
        image = cv2.imread(
            str(dataset_root / row["rgb_relative"]), cv2.IMREAD_COLOR
        )
        rectified = cv2.remap(
            image,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        thumbnail = cv2.resize(
            rectified, (tile_width, tile_height), interpolation=cv2.INTER_AREA
        )
        role_row = role_rows[row["selection_role"]]
        role_col = row["selection_rank"] - 1
        top = role_row * (tile_height + label_height)
        left = role_col * tile_width
        canvas[top : top + tile_height, left : left + tile_width] = thumbnail
        cv2.putText(
            canvas,
            "{} #{}".format(row["selection_role"].replace("proxy_", ""), row["frame"]),
            (left + 4, top + tile_height + 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "inc={:.3f} trans={:.3f} diff={:.3f}".format(
                row["inconsistent_residual_ratio"],
                row["transition_score"],
                row["difficulty_score"],
            ),
            (left + 4, top + tile_height + 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(str(output_path), canvas):
        raise OSError("failed to write {}".format(output_path))


def self_test():
    camera_matrix = np.array(
        [[100.0, 0.0, 2.0], [0.0, 100.0, 2.0], [0.0, 0.0, 1.0]]
    )
    identity = np.eye(4, dtype=np.float64)
    plane = np.full((5, 5), 3.0, dtype=np.float32)
    residual, valid = compute_warp_proxy(
        plane, plane, identity, identity, camera_matrix, 1
    )
    if np.count_nonzero(valid) != plane.size:
        raise RuntimeError("identity plane did not compare every pixel")
    if float(np.nanmax(np.abs(residual))) > 1e-9:
        raise RuntimeError("identity plane residual is non-zero")

    twc_current = np.eye(4, dtype=np.float64)
    twc_current[2, 3] = 1.0
    current = np.full((5, 5), 2.0, dtype=np.float32)
    residual, valid = compute_warp_proxy(
        plane, current, identity, twc_current, camera_matrix, 1
    )
    if not valid[2, 2] or abs(float(residual[2, 2])) > 1e-9:
        raise RuntimeError("known camera translation has wrong warp direction")
    print("[G2-4C Selection Self-Test] PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root", nargs="?", type=pathlib.Path)
    parser.add_argument("association", nargs="?", type=pathlib.Path)
    parser.add_argument("ground_truth", nargs="?", type=pathlib.Path)
    parser.add_argument("output_dir", nargs="?", type=pathlib.Path)
    parser.add_argument("--sequence-name")
    parser.add_argument("--scale", type=int, default=4)
    parser.add_argument("--per-stratum", type=int, default=6)
    parser.add_argument("--max-gt-delta-ms", type=float, default=40.0)
    parser.add_argument(
        "--pose-timestamp-source",
        choices=("rgb", "depth"),
        default="depth",
        help=(
            "timestamp used to interpolate the camera pose for depth warp; "
            "'depth' is the physical depth-capture-time proxy, while 'rgb' "
            "matches the current SLAM frame timestamp convention"
        ),
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if any(
        value is None
        for value in (
            args.dataset_root,
            args.association,
            args.ground_truth,
            args.output_dir,
            args.sequence_name,
        )
    ):
        parser.error(
            "dataset_root, association, ground_truth, output_dir and "
            "--sequence-name are required unless --self-test is used"
        )
    if args.scale < 1 or args.per_stratum < 1:
        raise ValueError("--scale and --per-stratum must be positive")
    if args.max_gt_delta_ms <= 0.0:
        raise ValueError("--max-gt-delta-ms must be positive")

    expected_outputs = [
        args.output_dir / "all_frame_metrics.csv",
        args.output_dir / "selected_frames.csv",
        args.output_dir / "selection_summary.json",
        args.output_dir / "selected_contact_sheet.png",
    ]
    if any(path.exists() for path in expected_outputs):
        raise FileExistsError(
            "selection output already exists; refusing to overwrite"
        )

    associations = read_association(args.association, args.dataset_root)
    ground_truth = read_ground_truth(args.ground_truth)
    gt_timestamps = [sample["timestamp"] for sample in ground_truth]
    max_gt_delta_seconds = args.max_gt_delta_ms / 1000.0
    map_x, map_y = cv2.initUndistortRectifyMap(
        BONN_K,
        BONN_D,
        None,
        BONN_K,
        (640, 480),
        cv2.CV_32FC1,
    )

    metrics = []
    previous_depth_m = None
    previous_gray = None
    previous_twc = None
    previous_inconsistency = None
    appearance_by_frame = {}
    for record in associations:
        rgb = cv2.imread(
            str(args.dataset_root / record["rgb_relative"]),
            cv2.IMREAD_COLOR,
        )
        depth = cv2.imread(
            str(args.dataset_root / record["depth_relative"]),
            cv2.IMREAD_UNCHANGED,
        )
        if rgb is None or depth is None:
            raise ValueError("failed to load association frame")
        if rgb.shape[:2] != (480, 640) or depth.shape != (480, 640):
            raise ValueError("Bonn selector expects 640x480 registered RGB-D")
        if depth.dtype != np.uint16:
            raise ValueError("Bonn selector expects uint16 depth")

        rectified_rgb = cv2.remap(
            rgb,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        rectified_depth = cv2.remap(
            depth,
            map_x,
            map_y,
            cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        depth_m = rectified_depth.astype(np.float32) / DEPTH_FACTOR
        gray = cv2.cvtColor(rectified_rgb, cv2.COLOR_BGR2GRAY)
        appearance_by_frame[record["frame"]] = (
            cv2.resize(gray, (32, 24), interpolation=cv2.INTER_AREA).astype(
                np.float32
            )
            / 255.0
        )
        pose_timestamp = (
            record["depth_timestamp"]
            if args.pose_timestamp_source == "depth"
            else record["rgb_timestamp"]
        )
        twc = interpolate_twc(
            ground_truth,
            gt_timestamps,
            pose_timestamp,
            max_gt_delta_seconds,
        )

        row = dict(record)
        row.update(
            {
                "pose_timestamp_source": args.pose_timestamp_source,
                "pose_timestamp": pose_timestamp,
                "gt_available": int(twc is not None),
                "eligible": False,
                "valid_comparison_ratio": None,
                "positive_residual_ratio": None,
                "negative_residual_ratio": None,
                "inconsistent_residual_ratio": None,
                "mean_abs_residual_m": None,
                "rgb_temporal_difference": None,
                "invalid_depth_ratio": float(
                    np.count_nonzero(rectified_depth == 0)
                    / rectified_depth.size
                ),
                "depth_boundary_ratio": depth_boundary_ratio(
                    depth_m, args.scale
                ),
                "camera_translation_m": None,
                "camera_rotation_deg": None,
                "transition_score": None,
                "difficulty_score": None,
            }
        )

        if (
            previous_depth_m is not None
            and previous_twc is not None
            and twc is not None
        ):
            residual, valid = compute_warp_proxy(
                previous_depth_m,
                depth_m,
                previous_twc,
                twc,
                BONN_K,
                args.scale,
            )
            valid_count = int(np.count_nonzero(valid))
            total_count = int(valid.size)
            valid_ratio = valid_count / total_count
            if valid_count:
                values = residual[valid]
                positive = values > RESIDUAL_THRESHOLD_M
                negative = values < -RESIDUAL_THRESHOLD_M
                inconsistent = positive | negative
                inconsistency_ratio = float(
                    np.count_nonzero(inconsistent) / valid_count
                )
                translation, rotation = pose_motion(previous_twc, twc)
                rgb_difference = float(
                    np.mean(
                        np.abs(
                            gray[:: args.scale, :: args.scale].astype(
                                np.float32
                            )
                            - previous_gray[
                                :: args.scale, :: args.scale
                            ].astype(np.float32)
                        )
                    )
                    / 255.0
                )
                transition = (
                    0.0
                    if previous_inconsistency is None
                    else abs(
                        inconsistency_ratio - previous_inconsistency
                    )
                )
                negative_ratio = float(
                    np.count_nonzero(negative) / valid_count
                )
                difficulty = (
                    0.5 * row["invalid_depth_ratio"]
                    + row["depth_boundary_ratio"]
                    + negative_ratio
                )
                row.update(
                    {
                        "eligible": valid_ratio >= 0.01,
                        "valid_comparison_ratio": valid_ratio,
                        "positive_residual_ratio": float(
                            np.count_nonzero(positive) / valid_count
                        ),
                        "negative_residual_ratio": negative_ratio,
                        "inconsistent_residual_ratio": inconsistency_ratio,
                        "mean_abs_residual_m": float(
                            np.mean(np.abs(values))
                        ),
                        "rgb_temporal_difference": rgb_difference,
                        "camera_translation_m": translation,
                        "camera_rotation_deg": rotation,
                        "transition_score": transition,
                        "difficulty_score": difficulty,
                    }
                )
                previous_inconsistency = inconsistency_ratio

        metrics.append(row)
        previous_depth_m = depth_m
        previous_gray = gray
        previous_twc = twc
        if record["frame"] == 0 or (record["frame"] + 1) % 100 == 0:
            print(
                "[G2-4C Selection] processed {}/{}".format(
                    record["frame"] + 1, len(associations)
                ),
                flush=True,
            )

    globally_selected = set()
    selection_plan = [
        (
            "proxy_high_inconsistency",
            "inconsistent_residual_ratio",
            True,
        ),
        ("proxy_transition", "transition_score", True),
        ("proxy_geometry_difficult", "difficulty_score", True),
        (
            "proxy_low_inconsistency",
            "inconsistent_residual_ratio",
            False,
        ),
    ]
    selected = []
    for role, key, reverse in selection_plan:
        rows = select_ranked(
            metrics,
            key,
            reverse,
            args.per_stratum,
            globally_selected,
            appearance_by_frame,
        )
        for rank, row in enumerate(rows, 1):
            selected_row = dict(row)
            selected_row["selection_role"] = role
            selected_row["selection_rank"] = rank
            selected.append(selected_row)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = list(metrics[0].keys())
    with expected_outputs[0].open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(metrics)

    selected_fieldnames = [
        "selection_role",
        "selection_rank",
    ] + fieldnames
    with expected_outputs[1].open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=selected_fieldnames)
        writer.writeheader()
        writer.writerows(selected)

    eligible = [row for row in metrics if row["eligible"]]
    role_counts = {
        role: sum(row["selection_role"] == role for row in selected)
        for role, _, _ in selection_plan
    }
    summary = {
        "stage": "G2-4C",
        "sequence": args.sequence_name,
        "selection_is_motion_gt": False,
        "selection_is_dynamic_decision": False,
        "selection_is_holdout_evaluation": False,
        "selection_conditioned_on_geometry_proxy": True,
        "suitable_for_unbiased_sequence_metrics": False,
        "coordinate_domain": "undistorted_pinhole_P_equals_K",
        "pose_timestamp_source": args.pose_timestamp_source,
        "maximum_rgb_depth_timestamp_delta_ms": max(
            abs(
                row["rgb_timestamp"]
                - row["depth_timestamp"]
            )
            for row in metrics
        )
        * 1000.0,
        "sampling_scale": args.scale,
        "residual_threshold_m_for_ranking_proxy": RESIDUAL_THRESHOLD_M,
        "frames": len(metrics),
        "eligible_frames": len(eligible),
        "selected_frames": len(selected),
        "selected_unique_frames": len(
            {row["frame"] for row in selected}
        ),
        "role_counts": role_counts,
        "eligible_inconsistency_distribution": percentile_summary(
            [row["inconsistent_residual_ratio"] for row in eligible]
        ),
        "selected_frame_indices_by_role": {
            role: [
                row["frame"]
                for row in selected
                if row["selection_role"] == role
            ]
            for role, _, _ in selection_plan
        },
        "selection_diversity_policy": [
            {
                "same_role_gap_frames": 20,
                "global_gap_frames": 5,
                "same_role_thumbnail_mad": 0.050,
            },
            {
                "same_role_gap_frames": 10,
                "global_gap_frames": 3,
                "same_role_thumbnail_mad": 0.025,
            },
            {
                "same_role_gap_frames": 5,
                "global_gap_frames": 1,
                "same_role_thumbnail_mad": 0.0,
            },
            {
                "same_role_gap_frames": 0,
                "global_gap_frames": 0,
                "same_role_thumbnail_mad": 0.0,
            },
        ],
        "selected_role_diversity": selected_role_diversity(
            selected, appearance_by_frame
        ),
    }
    with expected_outputs[2].open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    save_contact_sheet(
        args.dataset_root, selected, map_x, map_y, expected_outputs[3]
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print("Bonn frame selection failed: {}".format(error), file=sys.stderr)
        sys.exit(2)
