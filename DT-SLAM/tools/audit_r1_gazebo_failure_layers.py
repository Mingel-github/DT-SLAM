#!/usr/bin/env python3
"""Audit the frozen R1 Gazebo detector chain without changing SLAM.

The simulator-derived visible-box reference is constructed by testing current
RGB-D points against the known box volume in world coordinates.  It is a
diagnostic reference, not a claim of pixel-perfect segmentation ground truth.
"""

import argparse
import csv
import json
import math
import struct
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


def existing_path(value):
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise argparse.ArgumentTypeError("path does not exist: {}".format(path))
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read-only Gazebo flow/residual/region/classifier audit")
    parser.add_argument("--dataset", required=True, type=existing_path)
    parser.add_argument("--associations", required=True, type=existing_path)
    parser.add_argument("--audit-directory", required=True, type=existing_path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--box-half-extent", type=float, default=0.32)
    parser.add_argument("--minimum-visible-box-pixels", type=int, default=100)
    parser.add_argument("--fx", type=float, default=554.3827128226441)
    parser.add_argument("--fy", type=float, default=554.3827128226441)
    parser.add_argument("--cx", type=float, default=320.5)
    parser.add_argument("--cy", type=float, default=240.5)
    parser.add_argument("--depth-map-factor", type=float, default=5000.0)
    parser.add_argument("--region-maximum-depth", type=float, default=6.0)
    parser.add_argument("--contact-sheet-count", type=int, default=12)
    return parser.parse_args()


def load_associations(path):
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = line.split()
            if len(values) < 4:
                raise ValueError("invalid association row: {}".format(line))
            rows.append({
                "timestamp": float(values[0]),
                "rgb": values[1],
                "depth_timestamp": float(values[2]),
                "depth": values[3],
            })
    return rows


def load_trajectory(path):
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 8:
                raise ValueError("invalid trajectory row: {}".format(line))
            rows.append((values[0], np.asarray(values[1:4], dtype=np.float64),
                         np.asarray(values[4:8], dtype=np.float64)))
    if not rows:
        raise ValueError("empty trajectory: {}".format(path))
    return rows


def nearest_pose(rows, timestamp):
    # Exported Gazebo trajectories contain one pose per associated timestamp.
    index = min(range(len(rows)), key=lambda i: abs(rows[i][0] - timestamp))
    row = rows[index]
    return row, abs(row[0] - timestamp)


def quaternion_rotation(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def read_flo(path):
    with path.open("rb") as stream:
        magic = struct.unpack("<f", stream.read(4))[0]
        if abs(magic - 202021.25) > 1.0e-3:
            raise ValueError("invalid .flo magic: {}".format(path))
        width = struct.unpack("<i", stream.read(4))[0]
        height = struct.unpack("<i", stream.read(4))[0]
        values = np.frombuffer(stream.read(), dtype="<f4")
    expected = height * width * 2
    if values.size != expected:
        raise ValueError("truncated .flo file: {}".format(path))
    return values.reshape(height, width, 2)


def read_csv_rows(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def load_mask(path, shape=None):
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("cannot read image: {}".format(path))
    if image.ndim == 3:
        image = image[:, :, 0]
    if shape is not None and image.shape != shape:
        raise ValueError("shape mismatch for {}: {} vs {}".format(
            path, image.shape, shape))
    return image


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


def scalar(value):
    if value is None:
        return float("nan")
    return float(value)


def ratio(numerator, denominator):
    return float(numerator) / float(denominator) if denominator else float("nan")


def frame_path(root, child, input_index, suffix):
    return root / child / "frame_{:06d}{}".format(input_index, suffix)


def visible_box_mask(depth_m, camera_pose, box_pose, rays, half_extent):
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    mask = np.zeros(depth_m.shape, dtype=np.uint8)
    if not np.any(valid):
        return mask
    camera_translation, camera_quaternion = camera_pose
    box_translation, box_quaternion = box_pose
    rotation_world_camera = quaternion_rotation(camera_quaternion)
    rotation_world_box = quaternion_rotation(box_quaternion)
    points_camera = rays[valid] * depth_m[valid, None]
    points_world = points_camera @ rotation_world_camera.T + camera_translation
    points_box = (points_world - box_translation) @ rotation_world_box
    inside = np.all(np.abs(points_box) <= half_extent, axis=1)
    mask[valid] = inside.astype(np.uint8) * 255
    return mask


def decoded_labels(path, shape):
    encoded = load_mask(path, shape).astype(np.int32)
    labels = encoded - 1
    labels[encoded == 0] = -1
    return labels


def region_quality(labels, box_mask):
    box = box_mask != 0
    box_pixels = int(np.count_nonzero(box))
    overlapping = labels[box]
    overlapping = overlapping[overlapping > 0]
    if box_pixels == 0 or overlapping.size == 0:
        return {
            "box_region_assigned_coverage": 0.0 if box_pixels else float("nan"),
            "box_region_count": 0,
            "dominant_region_label": -1,
            "dominant_region_box_pixels": 0,
            "dominant_region_box_coverage": float("nan"),
            "dominant_region_purity": float("nan"),
        }
    unique, counts = np.unique(overlapping, return_counts=True)
    best = int(np.argmax(counts))
    label = int(unique[best])
    intersection = int(counts[best])
    region_pixels = int(np.count_nonzero(labels == label))
    return {
        "box_region_assigned_coverage": ratio(overlapping.size, box_pixels),
        "box_region_count": int(unique.size),
        "dominant_region_label": label,
        "dominant_region_box_pixels": intersection,
        "dominant_region_box_coverage": ratio(intersection, box_pixels),
        "dominant_region_purity": ratio(intersection, region_pixels),
    }


def classifier_reason(region_rows, input_index, label):
    if label <= 0:
        return "no_box_region", "none"
    row = region_rows.get((input_index, label))
    if row is None:
        return "missing_region_audit", "none"
    return row["decision_reason"], row["output_state"]


def oracle_box_classifier(high_mask, support_mask, box_mask):
    """Replay the existing contour gates on the visible-box region only.

    This is a diagnostic upper-bound intervention.  It is not a deployable
    detector and it does not enter SLAM.
    """
    box = box_mask != 0
    high = (high_mask != 0) & box
    high_pixels = int(np.count_nonzero(high))
    result = {
        "oracle_box_high_pixels": high_pixels,
        "oracle_box_high_fraction": ratio(high_pixels, np.count_nonzero(box)),
        "oracle_box_passed_minimum_high_pixels": int(high_pixels > 100),
        "oracle_box_high_contours": 0,
        "oracle_box_eligible_contours": 0,
        "oracle_box_seeded_contours": 0,
        "oracle_box_decision_reason": "insufficient_high_pixels",
    }
    if high_pixels <= 100:
        return result
    contours, _ = cv2.findContours(
        high.astype(np.uint8) * 255, cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_NONE)
    result["oracle_box_high_contours"] = len(contours)
    if not contours:
        result["oracle_box_decision_reason"] = "no_high_contours"
        return result
    eligible = 0
    seeded = 0
    support = (support_mask != 0) & box
    for contour in contours:
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        roundness = (4.0 * math.pi * area / (perimeter * perimeter)
                     if perimeter > 0.0 else 0.0)
        if not ((area > 100.0 and roundness > 0.2) or area > 2000.0):
            continue
        eligible += 1
        points = contour.reshape(-1, 2)
        if np.any(support[points[:, 1], points[:, 0]]):
            seeded += 1
    result["oracle_box_eligible_contours"] = eligible
    result["oracle_box_seeded_contours"] = seeded
    if eligible == 0:
        result["oracle_box_decision_reason"] = "contour_geometry_rejected"
    elif seeded == 0:
        result["oracle_box_decision_reason"] = "no_low_support_seed"
    else:
        result["oracle_box_decision_reason"] = "oracle_region_has_valid_seed"
    return result


def visual_panel(rgb, box_mask, high_mask, dynamic_mask, title):
    image = rgb.copy()
    dynamic = dynamic_mask != 0
    high = high_mask != 0
    box = box_mask != 0
    image[dynamic] = np.asarray([180, 0, 180], dtype=np.uint8)
    image[high] = np.asarray([0, 255, 255], dtype=np.uint8)
    contours, _ = cv2.findContours(
        box.astype(np.uint8) * 255, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
    cv2.putText(image, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 0, 0), 1, cv2.LINE_AA)
    return image


def select_contact_frames(rows, count):
    visible = [row for row in rows if row["box_pixels"] >= 100]
    invisible = [row for row in rows if row["box_pixels"] == 0]
    selected = []
    if visible:
        visible = sorted(visible, key=lambda row: row["box_pixels"])
        quantiles = np.linspace(0, len(visible) - 1, max(1, count - 2))
        selected.extend(visible[int(round(value))] for value in quantiles)
    if invisible:
        selected.extend([invisible[0], invisible[len(invisible) // 2]])
    unique = {}
    for row in selected:
        unique[row["input_index"]] = row
    return list(unique.values())[:count]


def aggregate_rows(rows):
    visible = [row for row in rows if row["box_pixels"] >= 100]
    invisible = [row for row in rows if row["box_pixels"] == 0]
    summary = {
        "frames_total": len(rows),
        "frames_with_flow": sum(row["flow_available"] for row in rows),
        "visible_box_frames_minimum_100_pixels": len(visible),
        "box_absent_frames": len(invisible),
        "box_pixels": finite_stats([row["box_pixels"] for row in visible]),
        "box_depth_m": finite_stats([row["box_depth_median"] for row in visible]),
        "observed_flow_box_px": finite_stats(
            [row["observed_flow_box_median_px"] for row in visible]),
        "observed_flow_non_box_px": finite_stats(
            [row["observed_flow_non_box_median_px"] for row in visible]),
        "homography_residual_box_px": finite_stats(
            [row["residual_box_median_px"] for row in visible]),
        "homography_residual_non_box_px": finite_stats(
            [row["residual_non_box_median_px"] for row in visible]),
        "box_high_residual_coverage": finite_stats(
            [row["high_box_coverage"] for row in visible]),
        "final_box_coverage": finite_stats(
            [row["final_box_coverage"] for row in visible]),
        "final_non_box_fraction_visible_frames": finite_stats(
            [row["final_non_box_fraction"] for row in visible]),
        "final_dynamic_fraction_box_absent_frames": finite_stats(
            [row["final_dynamic_fraction"] for row in invisible]),
        "dominant_region_box_coverage": finite_stats(
            [row["dominant_region_box_coverage"] for row in visible]),
        "dominant_region_purity": finite_stats(
            [row["dominant_region_purity"] for row in visible]),
        "initial_region_assigned_box_coverage": finite_stats(
            [row["initial_box_region_assigned_coverage"] for row in visible]),
        "gradient_region_assigned_box_coverage": finite_stats(
            [row["gradient_box_region_assigned_coverage"] for row in visible]),
        "rag_region_assigned_box_coverage": finite_stats(
            [row["box_region_assigned_coverage"] for row in visible]),
        "temporal_added_box_coverage": finite_stats(
            [row["temporal_added_box_coverage"] for row in visible]),
        "temporal_added_fraction_box_absent_frames": finite_stats(
            [row["temporal_added_fraction"] for row in invisible]),
        "dominant_region_decision_reasons": dict(Counter(
            row["dominant_region_decision_reason"] for row in visible)),
        "oracle_box_decision_reasons": dict(Counter(
            row["oracle_box_decision_reason"] for row in visible)),
    }
    if visible:
        ordered = sorted(visible, key=lambda row: row["box_pixels"])
        groups = np.array_split(np.asarray(ordered, dtype=object), 4)
        summary["box_area_quartiles"] = []
        for index, group in enumerate(groups, 1):
            values = list(group)
            summary["box_area_quartiles"].append({
                "quartile": index,
                "frames": len(values),
                "box_pixels": finite_stats([row["box_pixels"] for row in values]),
                "flow_box_px": finite_stats(
                    [row["observed_flow_box_median_px"] for row in values]),
                "residual_box_px": finite_stats(
                    [row["residual_box_median_px"] for row in values]),
                "high_box_coverage": finite_stats(
                    [row["high_box_coverage"] for row in values]),
                "initial_region_assigned_box_coverage": finite_stats(
                    [row["initial_box_region_assigned_coverage"]
                     for row in values]),
                "rag_region_assigned_box_coverage": finite_stats(
                    [row["box_region_assigned_coverage"] for row in values]),
                "dominant_region_purity": finite_stats(
                    [row["dominant_region_purity"] for row in values]),
                "final_box_coverage": finite_stats(
                    [row["final_box_coverage"] for row in values]),
            })
    return summary


def write_csv(path, rows):
    if not rows:
        raise ValueError("no rows to write")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    output = args.output_directory.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit("output directory is not empty: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)

    associations = load_associations(args.associations)
    frame_rows = read_csv_rows(args.audit_directory / "r1_frame_index.csv")
    region_source = read_csv_rows(
        args.audit_directory / "r1_region_decisions.csv")
    region_rows = {
        (int(row["input_index"]), int(row["region_label"])): row
        for row in region_source
    }
    camera_gt = load_trajectory(args.dataset / "groundtruth.txt")
    box_gt = load_trajectory(args.dataset / "box_groundtruth.txt")
    height, width = 480, 640
    grid_y, grid_x = np.mgrid[0:height, 0:width]
    rays = np.stack([
        (grid_x - args.cx) / args.fx,
        (grid_y - args.cy) / args.fy,
        np.ones((height, width), dtype=np.float64),
    ], axis=-1)

    results = []
    visual_cache = {}
    for frame_row in frame_rows:
        input_index = int(frame_row["input_index"])
        if input_index >= len(associations):
            raise ValueError("R1 index exceeds associations: {}".format(input_index))
        association = associations[input_index]
        timestamp = association["timestamp"]
        depth_raw = cv2.imread(
            str(args.dataset / association["depth"]), cv2.IMREAD_UNCHANGED)
        if depth_raw is None or depth_raw.shape != (height, width):
            raise ValueError("invalid depth for index {}".format(input_index))
        depth_m = depth_raw.astype(np.float64) / args.depth_map_factor
        (camera_time, camera_translation, camera_quaternion), camera_dt = (
            nearest_pose(camera_gt, timestamp))
        (box_time, box_translation, box_quaternion), box_dt = nearest_pose(
            box_gt, timestamp)
        box_mask = visible_box_mask(
            depth_m, (camera_translation, camera_quaternion),
            (box_translation, box_quaternion), rays, args.box_half_extent)
        box = box_mask != 0
        valid_depth = depth_m > 0.0
        non_box = valid_depth & ~box
        box_pixels = int(np.count_nonzero(box))
        valid_pixels = int(np.count_nonzero(valid_depth))
        ring = cv2.dilate(
            box_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)))
        ring = (ring != 0) & ~box & valid_depth

        flow_available = int(frame_row["flow_available"]) != 0
        observed_flow = np.full((height, width), np.nan, dtype=np.float32)
        residual = np.full((height, width), np.nan, dtype=np.float32)
        above_low = np.zeros((height, width), dtype=np.uint8)
        high = np.zeros((height, width), dtype=np.uint8)
        temporal_added = np.zeros((height, width), dtype=np.uint8)
        final_dynamic = np.zeros((height, width), dtype=np.uint8)
        support = np.zeros((height, width), dtype=np.uint8)
        empty_quality = region_quality(
            np.full((height, width), -1, dtype=np.int32), box_mask)
        initial_quality = empty_quality
        gradient_quality = empty_quality
        rag_quality = empty_quality
        oracle = {
            "oracle_box_high_pixels": 0,
            "oracle_box_high_fraction": float("nan"),
            "oracle_box_passed_minimum_high_pixels": 0,
            "oracle_box_high_contours": 0,
            "oracle_box_eligible_contours": 0,
            "oracle_box_seeded_contours": 0,
            "oracle_box_decision_reason": "flow_unavailable",
        }
        dominant_reason, dominant_state = "flow_unavailable", "none"

        if flow_available:
            flow = read_flo(frame_path(
                args.audit_directory, "flow", input_index, "_observed.flo"))
            observed_flow = np.linalg.norm(flow, axis=2)
            residual_q64 = load_mask(frame_path(
                args.audit_directory, "residual", input_index,
                "_magnitude_q64.png"), (height, width))
            residual = residual_q64.astype(np.float32) / 64.0
            above_low = load_mask(frame_path(
                args.audit_directory, "residual", input_index,
                "_above_low.png"), (height, width))
            high = load_mask(frame_path(
                args.audit_directory, "residual", input_index,
                "_high.png"), (height, width))
            temporal_added = load_mask(frame_path(
                args.audit_directory, "classifier", input_index,
                "_temporal_added.png"), (height, width))
            final_dynamic = load_mask(frame_path(
                args.audit_directory, "classifier", input_index,
                "_dynamic.png"), (height, width))
            support = load_mask(frame_path(
                args.audit_directory, "classifier", input_index,
                "_support_after_dilate.png"), (height, width))
            rag_labels = decoded_labels(frame_path(
                args.audit_directory, "labels", input_index,
                "_rag_plus1.png"), (height, width))
            initial_labels = decoded_labels(frame_path(
                args.audit_directory, "labels", input_index,
                "_initial_plus1.png"), (height, width))
            gradient_labels = decoded_labels(frame_path(
                args.audit_directory, "labels", input_index,
                "_gradient_plus1.png"), (height, width))
            initial_quality = region_quality(initial_labels, box_mask)
            gradient_quality = region_quality(gradient_labels, box_mask)
            rag_quality = region_quality(rag_labels, box_mask)
            dominant_reason, dominant_state = classifier_reason(
                region_rows, input_index,
                rag_quality["dominant_region_label"])
            oracle = oracle_box_classifier(high, support, box_mask)

        dynamic = final_dynamic != 0
        low = above_low != 0
        high_bool = high != 0
        temporal = temporal_added != 0
        result = {
            "input_index": input_index,
            "frame": int(frame_row["frame"]),
            "timestamp": timestamp,
            "flow_available": int(flow_available),
            "camera_pose_dt_s": camera_dt,
            "box_pose_dt_s": box_dt,
            "valid_depth_pixels": valid_pixels,
            "box_pixels": box_pixels,
            "box_visible": int(box_pixels >= args.minimum_visible_box_pixels),
            "box_depth_median": (float(np.median(depth_m[box]))
                                   if box_pixels else float("nan")),
            "box_within_region_maximum_depth_fraction": ratio(
                np.count_nonzero(box & (depth_m <= args.region_maximum_depth)),
                box_pixels),
            "low_threshold_px": float(frame_row["low_threshold_px"] or 0.0),
            "high_threshold_px": float(frame_row["high_threshold_px"] or 0.0),
            "observed_flow_box_median_px": (float(np.median(observed_flow[box]))
                                             if box_pixels and flow_available
                                             else float("nan")),
            "observed_flow_non_box_median_px": (
                float(np.median(observed_flow[non_box]))
                if np.any(non_box) and flow_available else float("nan")),
            "observed_flow_local_ring_median_px": (
                float(np.median(observed_flow[ring]))
                if np.any(ring) and flow_available else float("nan")),
            "residual_box_median_px": (float(np.median(residual[box]))
                                        if box_pixels and flow_available
                                        else float("nan")),
            "residual_non_box_median_px": (
                float(np.median(residual[non_box]))
                if np.any(non_box) and flow_available else float("nan")),
            "residual_local_ring_median_px": (
                float(np.median(residual[ring]))
                if np.any(ring) and flow_available else float("nan")),
            "low_box_coverage": ratio(np.count_nonzero(low & box), box_pixels),
            "high_box_coverage": ratio(np.count_nonzero(high_bool & box), box_pixels),
            "low_non_box_fraction": ratio(np.count_nonzero(low & non_box),
                                            np.count_nonzero(non_box)),
            "high_non_box_fraction": ratio(np.count_nonzero(high_bool & non_box),
                                             np.count_nonzero(non_box)),
            "high_local_ring_fraction": ratio(
                np.count_nonzero(high_bool & ring), np.count_nonzero(ring)),
            **{"initial_" + key: value
               for key, value in initial_quality.items()},
            **{"gradient_" + key: value
               for key, value in gradient_quality.items()},
            **rag_quality,
            "dominant_region_decision_reason": dominant_reason,
            "dominant_region_output_state": dominant_state,
            **oracle,
            "temporal_added_box_coverage": ratio(
                np.count_nonzero(temporal & box), box_pixels),
            "temporal_added_non_box_fraction": ratio(
                np.count_nonzero(temporal & non_box), np.count_nonzero(non_box)),
            "temporal_added_fraction": ratio(
                np.count_nonzero(temporal), valid_pixels),
            "final_box_coverage": ratio(np.count_nonzero(dynamic & box), box_pixels),
            "final_non_box_fraction": ratio(
                np.count_nonzero(dynamic & non_box), np.count_nonzero(non_box)),
            "final_dynamic_fraction": ratio(np.count_nonzero(dynamic), valid_pixels),
        }
        results.append(result)
        visual_cache[input_index] = (box_mask, high, final_dynamic)

    write_csv(output / "r1_failure_layer_per_frame.csv", results)
    summary = aggregate_rows(results)
    summary.update({
        "method": "simulator-derived visible-box reference from RGB-D and poses",
        "interpretation_limit": (
            "The known box volume is intersected with measured RGB-D points; "
            "it is a diagnostic visible-surface reference, not pixel-perfect GT. "
            "Non-box pixels can include the moving person."),
        "box_half_extent_m": args.box_half_extent,
        "minimum_visible_box_pixels": args.minimum_visible_box_pixels,
        "region_maximum_depth_m": args.region_maximum_depth,
        "flow_direction": "current_to_reference",
        "residual_model": "current SIn-style homography compensation",
        "oracle_scope": "offline visible-box region only; never enters SLAM",
    })
    (output / "r1_failure_layer_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8")

    panels = []
    selected_rows = select_contact_frames(results, args.contact_sheet_count)
    for row in selected_rows:
        association = associations[row["input_index"]]
        rgb = cv2.imread(str(args.dataset / association["rgb"]),
                         cv2.IMREAD_COLOR)
        if rgb is None:
            continue
        box_mask, high_mask, dynamic_mask = visual_cache[row["input_index"]]
        title = "i={} box={} high={:.2f} final={:.2f}".format(
            row["input_index"], row["box_pixels"],
            scalar(row["high_box_coverage"]),
            scalar(row["final_box_coverage"]))
        panels.append(visual_panel(
            rgb, box_mask, high_mask, dynamic_mask, title))
    if panels:
        columns = 3
        rows_of_panels = []
        for start in range(0, len(panels), columns):
            row = panels[start:start + columns]
            while len(row) < columns:
                row.append(np.zeros_like(panels[0]))
            rows_of_panels.append(np.hstack(row))
        cv2.imwrite(str(output / "r1_failure_layer_contact_sheet.png"),
                    np.vstack(rows_of_panels))

    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=True))


if __name__ == "__main__":
    main()
