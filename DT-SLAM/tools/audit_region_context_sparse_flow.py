#!/usr/bin/env python3
"""Offline G2-4F4 region-context audit for exact sparse ego-flow residuals.

The tool deliberately emits no motion class. It assigns already-exported
shadow-only C++ feature measurements to the existing G2-3R0
depth-discontinuity partition and evaluates a frozen RGB-only coarse-box
proxy. It never recomputes optical flow, semantic masks, or SLAM poses.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np


BONN_K = np.array(
    [[542.822841, 0.0, 315.593520],
     [0.0, 542.576870, 237.756098],
     [0.0, 0.0, 1.0]],
    dtype=np.float32)
BONN_D = np.array(
    [0.039903, -0.099343, -0.000730, -0.000144, 0.0],
    dtype=np.float32)
DEPTH_SCALE = 5000.0
RELATIVE_BOUNDARY_THRESHOLD = 0.025
ABSOLUTE_BOUNDARY_THRESHOLD_METERS = 0.08


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_float(row, key):
    value = row.get(key, "")
    if value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def median_absolute_deviation(values):
    if not values:
        return None
    array = np.asarray(values, dtype=np.float64)
    median = np.median(array)
    return float(np.median(np.abs(array-median)))


def safe_ratio(numerator, denominator):
    if numerator is None or denominator is None or denominator == 0:
        return None
    return float(numerator/denominator)


def load_rectified_depth(archive, archive_root, relative_path, map_x, map_y):
    member = f"{archive_root.rstrip('/')}/{relative_path}"
    encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    raw = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if raw is None or raw.dtype != np.uint16:
        raise RuntimeError(f"failed to decode uint16 depth: {member}")
    rectified = cv2.remap(
        raw, map_x, map_y, cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return rectified.astype(np.float32)/DEPTH_SCALE


def partition_depth(depth):
    """Mirror G2-3R0 boundary definition and 4-connected components."""
    if depth.dtype != np.float32 or depth.ndim != 2:
        raise ValueError("depth must be CV_32F-equivalent single-channel")
    valid = np.isfinite(depth) & (depth > 0.0)
    boundary = np.zeros(depth.shape, dtype=bool)

    # Each valid center pixel uses its own depth-dependent threshold, exactly
    # as the C++ implementation does.
    directions = ((0, -1), (0, 1), (-1, 0), (1, 0))
    height, width = depth.shape
    for delta_v, delta_u in directions:
        current_v0 = max(0, -delta_v)
        current_v1 = min(height, height-delta_v)
        current_u0 = max(0, -delta_u)
        current_u1 = min(width, width-delta_u)
        neighbor_v0 = current_v0+delta_v
        neighbor_v1 = current_v1+delta_v
        neighbor_u0 = current_u0+delta_u
        neighbor_u1 = current_u1+delta_u

        center = depth[current_v0:current_v1, current_u0:current_u1]
        neighbor = depth[
            neighbor_v0:neighbor_v1, neighbor_u0:neighbor_u1]
        center_valid = valid[
            current_v0:current_v1, current_u0:current_u1]
        neighbor_valid = valid[
            neighbor_v0:neighbor_v1, neighbor_u0:neighbor_u1]
        threshold = np.maximum(
            RELATIVE_BOUNDARY_THRESHOLD*center,
            ABSOLUTE_BOUNDARY_THRESHOLD_METERS)
        boundary[current_v0:current_v1, current_u0:current_u1] |= (
            center_valid & neighbor_valid &
            (np.abs(neighbor-center) > threshold))

    assigned = valid & ~boundary
    component_count, components = cv2.connectedComponents(
        assigned.astype(np.uint8), connectivity=4, ltype=cv2.CV_32S)
    labels = np.full(depth.shape, -1, dtype=np.int32)
    labels[boundary] = -2
    labels[assigned] = components[assigned]-1
    region_sizes = np.bincount(
        components[assigned], minlength=component_count)[1:]
    return labels, boundary, region_sizes.astype(np.int64)


def run_self_test():
    plane = np.ones((4, 6), dtype=np.float32)
    labels, boundary, sizes = partition_depth(plane)
    assert not np.any(boundary)
    assert len(sizes) == 1 and int(sizes[0]) == plane.size
    assert np.all(labels == 0)

    step = np.ones((4, 6), dtype=np.float32)
    step[:, 3:] = 2.0
    labels, boundary, sizes = partition_depth(step)
    assert np.all(boundary[:, 2:4])
    assert len(sizes) == 2
    assert sorted(int(value) for value in sizes) == [8, 8]
    assert np.all(labels[:, 2:4] == -2)

    invalid_barrier = np.ones((4, 5), dtype=np.float32)
    invalid_barrier[:, 2] = 0.0
    labels, boundary, sizes = partition_depth(invalid_barrier)
    assert not np.any(boundary)
    assert len(sizes) == 2
    assert sorted(int(value) for value in sizes) == [8, 8]
    assert np.all(labels[:, 2] == -1)
    print("[G2-4F4 region-context self-test] PASS")


def validate_frame_invariants(frame_rows):
    violations = [
        row for row in frame_rows
        if row.get("dynamic_decision") != "none" or
        row.get("direct_slam_state_mutation") != "none"]
    if violations:
        raise RuntimeError(
            f"SLAM mutation invariant violated in {len(violations)} rows")


def bbox_contains(box, u, v):
    return (
        box["x"] <= u < box["x"]+box["width"] and
        box["y"] <= v < box["y"]+box["height"])


def measured_nodes_for_frame(rows, labels):
    nodes = []
    state_counts = defaultdict(int)
    for row in rows:
        state = row.get("evidence_state", "")
        state_counts[state] += 1
        if state != "measured":
            continue
        if int(row.get("semantic_nonzero", "0")) != 0:
            raise RuntimeError(
                "measured rigidity node unexpectedly has semantic_nonzero")
        u = finite_float(row, "u_current")
        v = finite_float(row, "v_current")
        residual = finite_float(row, "flow_residual_magnitude_px")
        if u is None or v is None or residual is None:
            raise RuntimeError("measured node has missing finite value")
        u_round = int(round(u))
        v_round = int(round(v))
        if not (
                0 <= u_round < labels.shape[1] and
                0 <= v_round < labels.shape[0]):
            region = -3
        else:
            region = int(labels[v_round, u_round])
        nodes.append({
            "feature_index": int(row["feature_index"]),
            "u": u,
            "v": v,
            "residual": residual,
            "region": region,
        })
    return nodes, dict(state_counts)


def select_proxy_region(nodes, box):
    grouped = defaultdict(list)
    for node in nodes:
        if node["region"] >= 0:
            grouped[node["region"]].append(node)
    candidates = []
    for label, region_nodes in grouped.items():
        inside = sum(
            bbox_contains(box, node["u"], node["v"])
            for node in region_nodes)
        if inside == 0:
            continue
        purity = inside/len(region_nodes)
        candidates.append((inside, purity, -label, label))
    return max(candidates)[3] if candidates else None


def summarize_values(prefix, values):
    return {
        f"{prefix}_count": len(values),
        f"{prefix}_median_px": percentile(values, 50),
        f"{prefix}_p90_px": percentile(values, 90),
        f"{prefix}_mad_px": median_absolute_deviation(values),
    }


def audit_frame(frame, depth, node_rows, box):
    labels, boundary, region_sizes = partition_depth(depth)
    nodes, state_counts = measured_nodes_for_frame(node_rows, labels)
    selected_label = select_proxy_region(nodes, box)

    grouped = defaultdict(list)
    for node in nodes:
        if node["region"] >= 0:
            grouped[node["region"]].append(node)

    region_rows = []
    bbox_y0 = max(0, box["y"])
    bbox_y1 = min(labels.shape[0], box["y"]+box["height"])
    bbox_x0 = max(0, box["x"])
    bbox_x1 = min(labels.shape[1], box["x"]+box["width"])
    for label in sorted(grouped):
        region_nodes = grouped[label]
        inside_nodes = [
            node for node in region_nodes
            if bbox_contains(box, node["u"], node["v"])]
        region_area = int(region_sizes[label])
        bbox_region_pixels = int(np.count_nonzero(
            labels[bbox_y0:bbox_y1, bbox_x0:bbox_x1] == label))
        residuals = [node["residual"] for node in region_nodes]
        region_rows.append({
            "frame": frame,
            "region_label": label,
            "selected_by_bbox_only": int(label == selected_label),
            "region_pixels": region_area,
            "bbox_region_pixels": bbox_region_pixels,
            "region_pixel_bbox_purity":
                safe_ratio(bbox_region_pixels, region_area),
            "bbox_coverage_by_region":
                safe_ratio(
                    bbox_region_pixels,
                    max(0, bbox_y1-bbox_y0)*max(0, bbox_x1-bbox_x0)),
            "feature_count": len(region_nodes),
            "bbox_feature_count": len(inside_nodes),
            "bbox_feature_purity":
                safe_ratio(len(inside_nodes), len(region_nodes)),
            "residual_median_px": percentile(residuals, 50),
            "residual_p90_px": percentile(residuals, 90),
            "residual_mad_px": median_absolute_deviation(residuals),
            "dynamic_decision": "none",
            "direct_slam_state_mutation": "none",
        })

    selected_nodes = (
        grouped[selected_label] if selected_label is not None else [])
    selected_inside_nodes = [
        node for node in selected_nodes
        if bbox_contains(box, node["u"], node["v"])]
    point_inside_nodes = [
        node for node in nodes
        if bbox_contains(box, node["u"], node["v"])]
    background_nodes = [
        node for node in nodes
        if node["region"] >= 0 and
        node["region"] != selected_label and
        not bbox_contains(box, node["u"], node["v"])]

    selected_values = [node["residual"] for node in selected_nodes]
    point_inside_values = [node["residual"] for node in point_inside_nodes]
    background_values = [node["residual"] for node in background_nodes]
    summary = {
        "frame": frame,
        "selected_region": selected_label,
        "partition_regions": len(region_sizes),
        "valid_depth_pixels": int(np.count_nonzero(depth > 0.0)),
        "boundary_pixels": int(np.count_nonzero(boundary)),
        "measured_nodes": len(nodes),
        "assigned_nodes": sum(node["region"] >= 0 for node in nodes),
        "boundary_nodes": sum(node["region"] == -2 for node in nodes),
        "invalid_nodes": sum(node["region"] == -1 for node in nodes),
        "outside_nodes": sum(node["region"] == -3 for node in nodes),
        "selected_bbox_feature_count": len(selected_inside_nodes),
        "selected_feature_purity":
            safe_ratio(len(selected_inside_nodes), len(selected_nodes)),
        "state_counts_json": json.dumps(state_counts, sort_keys=True),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    summary.update(summarize_values("selected", selected_values))
    summary.update(summarize_values("point_inside", point_inside_values))
    summary.update(summarize_values("background", background_values))
    summary["selected_minus_background_median_px"] = (
        None if summary["selected_median_px"] is None or
        summary["background_median_px"] is None
        else summary["selected_median_px"]-
        summary["background_median_px"])
    summary["selected_over_background_median"] = safe_ratio(
        summary["selected_median_px"], summary["background_median_px"])
    summary["point_inside_minus_background_median_px"] = (
        None if summary["point_inside_median_px"] is None or
        summary["background_median_px"] is None
        else summary["point_inside_median_px"]-
        summary["background_median_px"])
    summary["point_inside_over_background_median"] = safe_ratio(
        summary["point_inside_median_px"], summary["background_median_px"])

    selected_region_row = next(
        (row for row in region_rows
         if row["selected_by_bbox_only"] == 1), None)
    summary["selected_region_pixels"] = (
        selected_region_row["region_pixels"]
        if selected_region_row else None)
    summary["selected_region_bbox_pixels"] = (
        selected_region_row["bbox_region_pixels"]
        if selected_region_row else None)
    summary["selected_region_pixel_bbox_purity"] = (
        selected_region_row["region_pixel_bbox_purity"]
        if selected_region_row else None)
    summary["selected_region_bbox_coverage"] = (
        selected_region_row["bbox_coverage_by_region"]
        if selected_region_row else None)
    return summary, region_rows


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"cannot write empty CSV: {path}")
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def aggregate_summary(sequence, frame_rows):
    support_rows = [
        row for row in frame_rows
        if row["selected_bbox_feature_count"] >= 3]
    comparable = [
        row for row in support_rows
        if row["selected_median_px"] is not None and
        row["background_median_px"] is not None]
    selected_positive = sum(
        row["selected_median_px"] > row["background_median_px"]
        for row in comparable)
    point_positive = sum(
        row["point_inside_median_px"] is not None and
        row["point_inside_median_px"] > row["background_median_px"]
        for row in comparable)
    p90_comparable = [
        row for row in support_rows
        if row["selected_p90_px"] is not None and
        row["background_p90_px"] is not None]
    selected_p90_positive = sum(
        row["selected_p90_px"] > row["background_p90_px"]
        for row in p90_comparable)
    return {
        "sequence": sequence,
        "method_identity":
            "[A/S/H] region-constrained aggregation of exact sparse "
            "ego-flow residual using G2-3R0 partition",
        "proxy_identity":
            "frozen RGB-only coarse boxes; not motion/pixel/object GT",
        "frame_count": len(frame_rows),
        "selected_region_frame_count": sum(
            row["selected_region"] is not None for row in frame_rows),
        "support_ge_3_frame_count": len(support_rows),
        "comparable_frame_count": len(comparable),
        "selected_median_gt_background_count": selected_positive,
        "point_inside_median_gt_background_count": point_positive,
        "selected_p90_gt_background_count": selected_p90_positive,
        "p90_comparable_frame_count": len(p90_comparable),
        "selected_feature_purity_median": percentile(
            [row["selected_feature_purity"] for row in support_rows
             if row["selected_feature_purity"] is not None], 50),
        "selected_pixel_bbox_purity_median": percentile(
            [row["selected_region_pixel_bbox_purity"] for row in support_rows
             if row["selected_region_pixel_bbox_purity"] is not None], 50),
        "selected_region_bbox_coverage_median": percentile(
            [row["selected_region_bbox_coverage"] for row in support_rows
             if row["selected_region_bbox_coverage"] is not None], 50),
        "selected_over_background_ratio_median": percentile(
            [row["selected_over_background_median"] for row in comparable
             if row["selected_over_background_median"] is not None], 50),
        "point_inside_over_background_ratio_median": percentile(
            [row["point_inside_over_background_median"] for row in comparable
             if row["point_inside_over_background_median"] is not None], 50),
        "representation_gate": {
            "minimum_support_frames_required": 10,
            "minimum_direction_fraction_required": 0.8,
            "support_frames_pass":
                len(support_rows) >= 10,
            "selected_direction_fraction":
                safe_ratio(selected_positive, len(comparable)),
            "selected_direction_pass":
                len(comparable) > 0 and
                selected_positive/len(comparable) >= 0.8,
        },
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }


def read_saved_frame_rows(paths):
    integer_fields = {
        "frame", "selected_bbox_feature_count"}
    optional_integer_fields = {"selected_region"}
    optional_float_fields = {
        "selected_median_px", "selected_p90_px",
        "point_inside_median_px", "background_median_px",
        "background_p90_px", "selected_feature_purity",
        "selected_region_pixel_bbox_purity",
        "selected_region_bbox_coverage",
        "selected_over_background_median",
        "point_inside_over_background_median"}
    rows = []
    for path in paths:
        for raw in read_csv(path):
            if raw.get("dynamic_decision") != "none" or \
                    raw.get("direct_slam_state_mutation") != "none":
                raise RuntimeError(f"invalid shadow invariant in {path}")
            row = dict(raw)
            for field in integer_fields:
                row[field] = int(row[field])
            for field in optional_integer_fields:
                row[field] = (
                    None if row.get(field, "") == ""
                    else int(row[field]))
            for field in optional_float_fields:
                row[field] = (
                    None if row.get(field, "") == ""
                    else float(row[field]))
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--combine-frame-csv", nargs="+", type=Path,
        help="combine existing per-frame audit CSV files and exit")
    parser.add_argument("--combined-name", default="combined_development")
    parser.add_argument("--combined-output", type=Path)
    parser.add_argument("--sequence")
    parser.add_argument("--node-csv", type=Path)
    parser.add_argument("--frame-csv", type=Path)
    parser.add_argument("--candidate-csv", type=Path)
    parser.add_argument("--bbox-csv", type=Path)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--archive-root")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return
    if args.combine_frame_csv:
        if args.combined_output is None:
            parser.error("--combined-output is required with --combine-frame-csv")
        combined = aggregate_summary(
            args.combined_name,
            read_saved_frame_rows(args.combine_frame_csv))
        args.combined_output.parent.mkdir(parents=True, exist_ok=True)
        with open(
                args.combined_output, "w", encoding="utf-8") as stream:
            json.dump(combined, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(combined, indent=2, sort_keys=True))
        return
    required = (
        "sequence", "node_csv", "frame_csv", "candidate_csv", "bbox_csv",
        "archive", "archive_root", "output_dir")
    missing = [name for name in required if getattr(args, name) is None]
    if missing:
        parser.error("missing required arguments: "+", ".join(missing))

    frame_contract = read_csv(args.frame_csv)
    validate_frame_invariants(frame_contract)
    frame_contract_ids = {int(row["frame"]) for row in frame_contract}

    by_frame = defaultdict(list)
    for row in read_csv(args.node_csv):
        by_frame[int(row["frame"])].append(row)
    candidates = {
        int(row["frame"]): row for row in read_csv(args.candidate_csv)}
    boxes = {
        int(row["source_frame"]): {
            key: int(row[key]) for key in ("x", "y", "width", "height")}
        for row in read_csv(args.bbox_csv)
        if row.get("visibility") != "absent" and
        all(row.get(key, "") for key in ("x", "y", "width", "height"))}
    frames = sorted(
        set(by_frame) & set(candidates) & set(boxes) & frame_contract_ids)
    if not frames:
        raise RuntimeError("no common node/candidate/bbox/frame-contract rows")

    map_x, map_y = cv2.initUndistortRectifyMap(
        BONN_K, BONN_D, np.eye(3, dtype=np.float32), BONN_K,
        (640, 480), cv2.CV_32FC1)
    frame_rows = []
    region_rows = []
    with ZipFile(args.archive) as archive:
        for frame in frames:
            depth = load_rectified_depth(
                archive, args.archive_root,
                candidates[frame]["depth_relative"], map_x, map_y)
            frame_summary, frame_regions = audit_frame(
                frame, depth, by_frame[frame], boxes[frame])
            frame_rows.append(frame_summary)
            region_rows.extend(frame_regions)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(
        args.output_dir/f"{args.sequence}_per_frame.csv", frame_rows)
    write_csv(
        args.output_dir/f"{args.sequence}_per_region.csv", region_rows)
    summary = aggregate_summary(args.sequence, frame_rows)
    with open(
            args.output_dir/f"{args.sequence}_summary.json",
            "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
