#!/usr/bin/env python3
"""Audit Bonn static-model projection against a true-static RGB-D sequence.

The output is evaluation-only.  This utility deliberately emits no dynamic
decision and has no connection to the SLAM runtime state.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import tempfile
import time
import zipfile
from pathlib import Path

import cv2
import numpy as np


BONN_K = np.asarray(
    [
        [542.822841, 0.0, 315.593520],
        [0.0, 542.576870, 237.756098],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
BONN_D = np.asarray(
    [0.039903, -0.099343, -0.000730, -0.000144, 0.0],
    dtype=np.float64,
)
T_ROS = np.asarray(
    [
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
T_M = np.asarray(
    [
        [1.0157, 0.1828, -0.2389, 0.0113],
        [0.0009, -0.8431, -0.6413, -0.0098],
        [-0.3009, 0.6147, -0.8085, 0.0111],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def read_associations(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(f"{path}:{line_number}: expected four fields")
            rows.append(
                {
                    "rgb_timestamp": float(fields[0]),
                    "rgb_relative": fields[1],
                    "depth_timestamp": float(fields[2]),
                    "depth_relative": fields[3],
                }
            )
    if not rows:
        raise ValueError(f"{path} contains no associations")
    return rows


def archive_root(archive: zipfile.ZipFile) -> str:
    roots = {name.split("/", 1)[0] for name in archive.namelist() if "/" in name}
    if len(roots) != 1:
        raise ValueError(f"archive has ambiguous roots: {sorted(roots)!r}")
    return next(iter(roots))


def parse_groundtruth(text: str) -> np.ndarray:
    rows = []
    for line_number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"groundtruth:{line_number}: expected eight fields")
        rows.append([float(value) for value in fields])
    result = np.asarray(rows, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 8:
        raise ValueError("groundtruth has invalid shape")
    if np.any(np.diff(result[:, 0]) <= 0.0):
        raise ValueError("groundtruth timestamps are not strictly increasing")
    return result


def normalized_quaternion(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid zero/non-finite quaternion")
    return quaternion / norm


def slerp_xyzw(first: np.ndarray, second: np.ndarray, alpha: float) -> np.ndarray:
    first = normalized_quaternion(first)
    second = normalized_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    if dot > 0.9995:
        return normalized_quaternion(first + alpha * (second - first))
    angle = math.acos(dot)
    return (
        math.sin((1.0 - alpha) * angle) / math.sin(angle) * first
        + math.sin(alpha * angle) / math.sin(angle) * second
    )


def rotation_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = normalized_quaternion(quaternion)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def interpolate_pose(groundtruth: np.ndarray, timestamp: float) -> tuple[np.ndarray, dict]:
    right = int(np.searchsorted(groundtruth[:, 0], timestamp, side="left"))
    if right == 0 or right == len(groundtruth):
        raise ValueError("timestamp lies outside the groundtruth interpolation range")
    left = right - 1
    interval = groundtruth[right, 0] - groundtruth[left, 0]
    alpha = (timestamp - groundtruth[left, 0]) / interval
    translation = (
        (1.0 - alpha) * groundtruth[left, 1:4]
        + alpha * groundtruth[right, 1:4]
    )
    quaternion = slerp_xyzw(
        groundtruth[left, 4:8], groundtruth[right, 4:8], float(alpha)
    )
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = rotation_from_xyzw(quaternion)
    pose[:3, 3] = translation
    return pose, {
        "left_timestamp": float(groundtruth[left, 0]),
        "right_timestamp": float(groundtruth[right, 0]),
        "interval_seconds": float(interval),
        "alpha": float(alpha),
        "nearest_seconds": float(
            min(
                timestamp - groundtruth[left, 0],
                groundtruth[right, 0] - timestamp,
            )
        ),
    }


def model_from_camera(marker_pose: np.ndarray) -> np.ndarray:
    # This is the official formula with T_i replacing T_0 for a per-frame
    # sensor pose.  T_M intentionally retains the official uniform scale.
    return T_ROS @ marker_pose @ T_ROS @ T_M


def render_depth(
    points_model: np.ndarray,
    camera_from_model: np.ndarray,
    image_size: tuple[int, int],
    splat_radius: int,
) -> tuple[np.ndarray, dict]:
    width, height = image_size
    start = time.perf_counter()
    points_camera = (
        points_model @ camera_from_model[:3, :3].T
        + camera_from_model[:3, 3]
    )
    positive = points_camera[:, 2] > 0.05
    points_camera = points_camera[positive]
    u = np.rint(
        BONN_K[0, 0] * points_camera[:, 0] / points_camera[:, 2]
        + BONN_K[0, 2]
    ).astype(np.int32)
    v = np.rint(
        BONN_K[1, 1] * points_camera[:, 1] / points_camera[:, 2]
        + BONN_K[1, 2]
    ).astype(np.int32)
    in_image = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    pixel_indices = v[in_image] * width + u[in_image]
    depth = np.full(width * height, np.inf, dtype=np.float32)
    np.minimum.at(
        depth,
        pixel_indices,
        points_camera[in_image, 2].astype(np.float32),
    )
    depth = depth.reshape(height, width)
    raw_valid_pixels = int(np.count_nonzero(np.isfinite(depth)))
    if splat_radius:
        kernel_size = 2 * splat_radius + 1
        depth = cv2.erode(
            depth, np.ones((kernel_size, kernel_size), dtype=np.uint8)
        )
    # cv::erode may replace an all-INF neighborhood with FLT_MAX.  FLT_MAX is
    # technically finite, but it is a missing rendered sample, not a depth.
    valid = np.isfinite(depth) & (
        depth < np.finfo(np.float32).max * np.float32(0.5)
    )
    depth[~valid] = 0.0
    return depth, {
        "model_points": int(points_model.shape[0]),
        "positive_z_points": int(np.count_nonzero(positive)),
        "in_image_points": int(np.count_nonzero(in_image)),
        "raw_zbuffer_valid_pixels": raw_valid_pixels,
        "splat_valid_pixels": int(np.count_nonzero(valid)),
        "render_ms": float((time.perf_counter() - start) * 1000.0),
    }


def decode_image(archive: zipfile.ZipFile, member: str, mode: int) -> np.ndarray:
    try:
        encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    except KeyError as error:
        raise ValueError(f"missing archive member: {member}") from error
    image = cv2.imdecode(encoded, mode)
    if image is None:
        raise ValueError(f"failed to decode archive member: {member}")
    return image


def depth_edges(depth: np.ndarray) -> np.ndarray:
    valid = depth > 0.0
    edges = np.zeros(depth.shape, dtype=np.uint8)
    left, right = depth[:, :-1], depth[:, 1:]
    pair_valid = valid[:, :-1] & valid[:, 1:]
    threshold = np.maximum(0.08, 0.025 * np.minimum(left, right))
    selected = pair_valid & (np.abs(left - right) > threshold)
    edges[:, :-1][selected] = 255
    edges[:, 1:][selected] = 255
    top, bottom = depth[:-1, :], depth[1:, :]
    pair_valid = valid[:-1, :] & valid[1:, :]
    threshold = np.maximum(0.08, 0.025 * np.minimum(top, bottom))
    selected = pair_valid & (np.abs(top - bottom) > threshold)
    edges[:-1, :][selected] = 255
    edges[1:, :][selected] = 255
    return edges


def statistics(values: np.ndarray) -> dict:
    if values.size == 0:
        return {"count": 0}
    median = float(np.median(values))
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "median": median,
        "mad": float(np.median(np.abs(values - median))),
        "q01": float(np.quantile(values, 0.01)),
        "q05": float(np.quantile(values, 0.05)),
        "q10": float(np.quantile(values, 0.10)),
        "q90": float(np.quantile(values, 0.90)),
        "q95": float(np.quantile(values, 0.95)),
        "q99": float(np.quantile(values, 0.99)),
        "abs_lt_0p05_ratio": float(np.mean(np.abs(values) < 0.05)),
        "abs_lt_0p10_ratio": float(np.mean(np.abs(values) < 0.10)),
        "positive_gt_0p10_ratio": float(np.mean(values > 0.10)),
        "negative_lt_m0p10_ratio": float(np.mean(values < -0.10)),
    }


def write_debug_images(
    output_dir: Path,
    frame_index: int,
    rgb: np.ndarray,
    current_depth: np.ndarray,
    model_depth: np.ndarray,
    joint_valid: np.ndarray,
    residual: np.ndarray,
    risk: np.ndarray,
) -> None:
    prefix = output_dir / f"frame_{frame_index:06d}"
    model_mm = np.clip(np.rint(model_depth * 5000.0), 0, 65535).astype(np.uint16)
    cv2.imwrite(str(prefix) + "_rgb.png", rgb)
    cv2.imwrite(str(prefix) + "_model_depth.png", model_mm)
    cv2.imwrite(
        str(prefix) + "_joint_valid.png", joint_valid.astype(np.uint8) * 255
    )
    cv2.imwrite(str(prefix) + "_risk.png", risk.astype(np.uint8) * 255)
    signed = np.zeros(residual.shape, dtype=np.uint8)
    signed[joint_valid] = np.clip(
        np.rint((residual[joint_valid] + 0.30) / 0.60 * 255.0), 0, 255
    ).astype(np.uint8)
    colored = cv2.applyColorMap(signed, cv2.COLORMAP_TURBO)
    colored[~joint_valid] = 0
    cv2.imwrite(str(prefix) + "_signed_residual_pm0p30m.png", colored)


def aggregate_frame_rows(rows: list[dict]) -> dict:
    numeric_keys = [
        "current_valid_pixels",
        "model_valid_pixels",
        "joint_valid_pixels",
        "joint_valid_ratio_of_current",
        "nonrisk_joint_pixels",
        "residual_median",
        "residual_mad",
        "residual_abs_lt_0p10_ratio",
        "residual_positive_gt_0p10_ratio",
        "nonrisk_residual_median",
        "nonrisk_residual_mad",
        "nonrisk_abs_lt_0p10_ratio",
        "nonrisk_positive_gt_0p10_ratio",
        "render_ms",
    ]
    result = {}
    for key in numeric_keys:
        values = np.asarray(
            [float(row[key]) for row in rows if row[key] != ""], dtype=np.float64
        )
        if values.size:
            result[key] = {
                "mean": float(np.mean(values)),
                "median": float(np.median(values)),
                "min": float(np.min(values)),
                "max": float(np.max(values)),
            }
    return result


def audit(args: argparse.Namespace) -> dict:
    if args.splat_radius < 0:
        raise ValueError("--splat-radius must be non-negative")
    if args.frame_step <= 0 or args.max_frames <= 0:
        raise ValueError("--frame-step and --max-frames must be positive")
    if args.output_dir.exists():
        raise ValueError(f"refusing to overwrite existing {args.output_dir}")

    points = np.load(args.model_npy, allow_pickle=False)
    if points.dtype != np.float32 or points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("model NPY must be an Nx3 float32 array")
    if not np.isfinite(points).all():
        raise ValueError("model points contain non-finite values")

    associations = read_associations(args.association)
    args.output_dir.mkdir(parents=True)
    debug_dir = args.output_dir / "debug"
    debug_dir.mkdir()
    map_x, map_y = cv2.initUndistortRectifyMap(
        BONN_K, BONN_D, None, BONN_K, (640, 480), cv2.CV_32FC1
    )

    rows = []
    skipped_outside_gt = 0
    all_residual_parts = []
    all_nonrisk_parts = []
    with zipfile.ZipFile(args.dataset_zip) as archive:
        root = archive_root(archive)
        groundtruth = parse_groundtruth(
            archive.read(root + "/groundtruth.txt").decode("utf-8")
        )
        candidate_indices = list(range(0, len(associations), args.frame_step))
        for frame_index in candidate_indices:
            if len(rows) >= args.max_frames:
                break
            association = associations[frame_index]
            try:
                marker_pose, interpolation = interpolate_pose(
                    groundtruth, association["depth_timestamp"]
                )
            except ValueError:
                skipped_outside_gt += 1
                continue

            rgb = decode_image(
                archive,
                root + "/" + association["rgb_relative"],
                cv2.IMREAD_COLOR,
            )
            raw_depth = decode_image(
                archive,
                root + "/" + association["depth_relative"],
                cv2.IMREAD_UNCHANGED,
            )
            if rgb.shape[:2] != (480, 640) or raw_depth.shape != (480, 640):
                raise ValueError("Bonn audit expects 640x480 RGB-D")
            if raw_depth.dtype != np.uint16:
                raise ValueError("Bonn depth must be uint16")
            rectified_rgb = cv2.remap(
                rgb,
                map_x,
                map_y,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            current_depth = (
                cv2.remap(
                    raw_depth,
                    map_x,
                    map_y,
                    cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                ).astype(np.float32)
                / 5000.0
            )

            global_from_camera = model_from_camera(marker_pose)
            camera_from_global = np.linalg.inv(global_from_camera)
            model_depth, render = render_depth(
                points,
                camera_from_global,
                (640, 480),
                args.splat_radius,
            )
            joint_valid = (current_depth > 0.0) & (model_depth > 0.0)
            residual = model_depth - current_depth

            current_edge = depth_edges(current_depth)
            model_edge = depth_edges(model_depth)
            risk = cv2.dilate(
                ((current_edge > 0) | (model_edge > 0)).astype(np.uint8),
                np.ones((5, 5), dtype=np.uint8),
            ) > 0
            nonrisk = joint_valid & ~risk
            residual_stats = statistics(residual[joint_valid])
            nonrisk_stats = statistics(residual[nonrisk])
            all_residual_parts.append(residual[joint_valid])
            all_nonrisk_parts.append(residual[nonrisk])

            row = {
                "frame": frame_index,
                "rgb_timestamp": association["rgb_timestamp"],
                "depth_timestamp": association["depth_timestamp"],
                "gt_nearest_seconds": interpolation["nearest_seconds"],
                "gt_interval_seconds": interpolation["interval_seconds"],
                "current_valid_pixels": int(np.count_nonzero(current_depth)),
                "model_valid_pixels": int(np.count_nonzero(model_depth)),
                "joint_valid_pixels": int(np.count_nonzero(joint_valid)),
                "joint_valid_ratio_of_current": float(
                    np.count_nonzero(joint_valid)
                    / max(np.count_nonzero(current_depth), 1)
                ),
                "nonrisk_joint_pixels": int(np.count_nonzero(nonrisk)),
                "residual_median": residual_stats.get("median", ""),
                "residual_mad": residual_stats.get("mad", ""),
                "residual_abs_lt_0p10_ratio": residual_stats.get(
                    "abs_lt_0p10_ratio", ""
                ),
                "residual_positive_gt_0p10_ratio": residual_stats.get(
                    "positive_gt_0p10_ratio", ""
                ),
                "nonrisk_residual_median": nonrisk_stats.get("median", ""),
                "nonrisk_residual_mad": nonrisk_stats.get("mad", ""),
                "nonrisk_abs_lt_0p10_ratio": nonrisk_stats.get(
                    "abs_lt_0p10_ratio", ""
                ),
                "nonrisk_positive_gt_0p10_ratio": nonrisk_stats.get(
                    "positive_gt_0p10_ratio", ""
                ),
                "render_ms": render["render_ms"],
            }
            rows.append(row)
            if len(rows) <= args.debug_limit:
                write_debug_images(
                    debug_dir,
                    frame_index,
                    rectified_rgb,
                    current_depth,
                    model_depth,
                    joint_valid,
                    residual,
                    risk,
                )

    if not rows:
        raise ValueError("no frames remained after GT interpolation checks")
    all_residual = np.concatenate(all_residual_parts)
    all_nonrisk = np.concatenate(all_nonrisk_parts)
    summary = {
        "schema": "dtslam_g2_6e1_static_model_alignment_v1",
        "stage": "G2-6E1",
        "role": "evaluation-only static-model alignment audit",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "dataset_zip": str(args.dataset_zip.resolve()),
        "dataset_zip_sha256": sha256_file(args.dataset_zip),
        "association": str(args.association.resolve()),
        "association_sha256": sha256_file(args.association),
        "model_npy": str(args.model_npy.resolve()),
        "model_npy_sha256": sha256_file(args.model_npy),
        "model_points": int(points.shape[0]),
        "coordinate_domain": "joint rectified P=official K",
        "pose_timestamp": "depth timestamp",
        "pose_interpolation": "linear translation plus shortest-path quaternion SLERP",
        "transform": (
            "camera_from_model = inverse(T_ROS * T_i * T_ROS * T_m); "
            "official T_m scale retained"
        ),
        "rasterization": {
            "zbuffer": "nearest integer pixel, minimum positive camera z",
            "splat_radius_pixels": args.splat_radius,
            "splat_operation": "local minimum over z-buffer",
        },
        "frames": len(rows),
        "skipped_outside_gt": skipped_outside_gt,
        "pooled_joint_residual": statistics(all_residual),
        "pooled_nonrisk_residual": statistics(all_nonrisk),
        "per_frame": aggregate_frame_rows(rows),
        "risk_definition": (
            "Chebyshev distance <=2 from current-depth or rendered-model "
            "depth discontinuity; diagnostic only"
        ),
    }
    csv_path = args.output_dir / "per_frame.csv"
    with csv_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def self_test() -> None:
    identity_xyzw = np.asarray([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(rotation_from_xyzw(identity_xyzw), np.eye(3))
    np.testing.assert_allclose(
        slerp_xyzw(identity_xyzw, -identity_xyzw, 0.5), identity_xyzw
    )
    assert np.array_equal(T_ROS @ T_ROS, np.eye(4))
    singular_values = np.linalg.svd(T_M[:3, :3], compute_uv=False)
    np.testing.assert_allclose(
        singular_values,
        np.full(3, np.mean(singular_values)),
        rtol=2e-4,
        atol=2e-4,
    )
    points = np.asarray([[0.0, 0.0, 1.0], [0.0, 0.0, 2.0]], np.float32)
    depth, metadata = render_depth(points, np.eye(4), (640, 480), 0)
    assert depth[238, 316] == 1.0
    assert metadata["raw_zbuffer_valid_pixels"] == 1
    print("audit_bonn_static_model_alignment self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-zip", type=Path)
    parser.add_argument("--association", type=Path)
    parser.add_argument("--model-npy", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frame-step", type=int, default=10)
    parser.add_argument("--max-frames", type=int, default=30)
    parser.add_argument("--splat-radius", type=int, default=1)
    parser.add_argument("--debug-limit", type=int, default=6)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    required = (
        args.dataset_zip,
        args.association,
        args.model_npy,
        args.output_dir,
    )
    if not args.self_test and any(value is None for value in required):
        parser.error(
            "--dataset-zip, --association, --model-npy and --output-dir "
            "are required"
        )
    if args.debug_limit < 0:
        parser.error("--debug-limit must be non-negative")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        self_test()
        return
    summary = audit(args)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
