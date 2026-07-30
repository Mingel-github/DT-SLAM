#!/usr/bin/env python3
"""Audit exact C++ G2-4F3U continuous edge measurements.

The tool only groups already-exported C++ nodes/edges by frozen RGB-only
coarse boxes. It does not recompute depth uncertainty, select a threshold,
or produce a dynamic/static decision.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


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
    finite = [value for value in values if value is not None and
              math.isfinite(value)]
    if not finite:
        return None
    return float(np.percentile(np.asarray(finite, dtype=np.float64), q))


def pearson(first, second):
    pairs = [(a, b) for a, b in zip(first, second)
             if a is not None and b is not None and
             math.isfinite(a) and math.isfinite(b)]
    if len(pairs) < 3:
        return None
    a = np.asarray([pair[0] for pair in pairs], dtype=np.float64)
    b = np.asarray([pair[1] for pair in pairs], dtype=np.float64)
    if float(np.std(a)) == 0.0 or float(np.std(b)) == 0.0:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def inside_box(node, box):
    u = float(node["u_current"])
    v = float(node["v_current"])
    return (box["x"] <= u < box["x"]+box["width"] and
            box["y"] <= v < box["y"]+box["height"])


def group_summary(edges):
    return {
        "count": len(edges),
        "absolute_median_m": percentile(
            [edge["absolute"] for edge in edges], 50),
        "relative_median": percentile(
            [edge["relative"] for edge in edges], 50),
        "normalized_median": percentile(
            [edge["normalized"] for edge in edges], 50),
        "uncertainty_std_median_m": percentile(
            [edge["uncertainty"] for edge in edges], 50),
        "edge_distance_median_m": percentile(
            [edge["distance"] for edge in edges], 50),
        "mean_depth_median_m": percentile(
            [edge["mean_depth"] for edge in edges], 50),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--node-csv", required=True, type=Path)
    parser.add_argument("--edge-csv", required=True, type=Path)
    parser.add_argument("--bbox-csv", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    required_node_fields = {
        "frame", "feature_index", "u_current", "v_current",
        "z_current_m", "current_depth_uncertainty_std_m",
        "evidence_state"}
    required_edge_fields = {
        "frame", "feature_index_a", "feature_index_b",
        "reference_distance_m", "current_distance_m",
        "absolute_strain_m", "relative_strain",
        "delta_length_uncertainty_std_m",
        "uncertainty_normalized_strain", "dynamic_decision",
        "direct_slam_state_mutation"}

    node_rows = read_csv(args.node_csv)
    edge_rows = read_csv(args.edge_csv)
    if not node_rows or not required_node_fields.issubset(node_rows[0]):
        raise RuntimeError("node CSV does not contain the F3U schema")
    if not edge_rows or not required_edge_fields.issubset(edge_rows[0]):
        raise RuntimeError("edge CSV does not contain the F3U schema")
    if any(row["dynamic_decision"] != "none" or
           row["direct_slam_state_mutation"] != "none"
           for row in edge_rows):
        raise RuntimeError("F3U invariant violation in edge CSV")

    boxes = {
        int(row["source_frame"]): {
            key: int(row[key]) for key in ("x", "y", "width", "height")}
        for row in read_csv(args.bbox_csv)
        if row.get("export_name", "").startswith(args.sequence+"_") and
        row.get("visibility") != "absent" and
        all(row.get(key, "") for key in ("x", "y", "width", "height"))}

    nodes = {}
    for row in node_rows:
        key = (int(row["frame"]), int(row["feature_index"]))
        nodes[key] = row

    grouped_by_frame = defaultdict(lambda: defaultdict(list))
    all_edges = []
    missing_endpoint = 0
    for row in edge_rows:
        frame = int(row["frame"])
        if frame not in boxes:
            continue
        key_a = (frame, int(row["feature_index_a"]))
        key_b = (frame, int(row["feature_index_b"]))
        if key_a not in nodes or key_b not in nodes:
            missing_endpoint += 1
            continue
        node_a = nodes[key_a]
        node_b = nodes[key_b]
        inside_a = inside_box(node_a, boxes[frame])
        inside_b = inside_box(node_b, boxes[frame])
        relation = ("inside_inside" if inside_a and inside_b else
                    "outside_outside" if not inside_a and not inside_b else
                    "crossing")
        reference_distance = float(row["reference_distance_m"])
        current_distance = float(row["current_distance_m"])
        edge = {
            "frame": frame,
            "relation": relation,
            "absolute": float(row["absolute_strain_m"]),
            "relative": float(row["relative_strain"]),
            "uncertainty": float(row["delta_length_uncertainty_std_m"]),
            "normalized": float(row["uncertainty_normalized_strain"]),
            "distance": 0.5*(reference_distance+current_distance),
            "mean_depth": 0.5*(float(node_a["z_current_m"])+
                               float(node_b["z_current_m"])),
        }
        grouped_by_frame[frame][relation].append(edge)
        all_edges.append(edge)

    frame_rows = []
    for frame in sorted(grouped_by_frame):
        groups = grouped_by_frame[frame]
        summary = {"frame": frame}
        for relation in ("inside_inside", "outside_outside", "crossing"):
            values = group_summary(groups[relation])
            for key, value in values.items():
                summary[f"{relation}_{key}"] = value
        summary["dynamic_decision"] = "none"
        summary["direct_slam_state_mutation"] = "none"
        frame_rows.append(summary)

    if not frame_rows:
        raise RuntimeError("no common C++ edge and frozen-box frames")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with open(args.output_dir/f"{args.sequence}_per_frame.csv", "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(frame_rows[0]))
        writer.writeheader()
        writer.writerows(frame_rows)

    comparable = [
        row for row in frame_rows
        if row["inside_inside_normalized_median"] is not None and
        row["outside_outside_normalized_median"] is not None]
    crossing_comparable = [
        row for row in comparable
        if row["crossing_normalized_median"] is not None]
    inside_not_greater = sum(
        row["inside_inside_normalized_median"] <=
        row["outside_outside_normalized_median"]
        for row in comparable)
    crossing_greater = sum(
        row["crossing_normalized_median"] >
        row["outside_outside_normalized_median"]
        for row in crossing_comparable)

    summary = {
        "sequence": args.sequence,
        "frame_count": len(frame_rows),
        "edge_count": len(all_edges),
        "missing_endpoint_count": missing_endpoint,
        "proxy_identity":
            "frozen RGB-only coarse boxes; not motion ground truth",
        "inside_normalized_le_outside_count": inside_not_greater,
        "normalized_comparable_frame_count": len(comparable),
        "crossing_normalized_gt_outside_count": crossing_greater,
        "crossing_comparable_frame_count": len(crossing_comparable),
        "absolute_vs_edge_distance_pearson": pearson(
            [edge["absolute"] for edge in all_edges],
            [edge["distance"] for edge in all_edges]),
        "relative_vs_edge_distance_pearson": pearson(
            [edge["relative"] for edge in all_edges],
            [edge["distance"] for edge in all_edges]),
        "normalized_vs_edge_distance_pearson": pearson(
            [edge["normalized"] for edge in all_edges],
            [edge["distance"] for edge in all_edges]),
        "absolute_vs_mean_depth_pearson": pearson(
            [edge["absolute"] for edge in all_edges],
            [edge["mean_depth"] for edge in all_edges]),
        "normalized_vs_mean_depth_pearson": pearson(
            [edge["normalized"] for edge in all_edges],
            [edge["mean_depth"] for edge in all_edges]),
        "all_group_summary": group_summary(all_edges),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "method_identity": {
            "point_edge_covariance": "[A] Dai et al.",
            "depth_square_noise": "[L/A] Khoshelham and Elberink",
            "axial_edge_length_propagation": "[S]",
            "continuous_score": "[S/H]",
        },
    }
    with open(args.output_dir/f"{args.sequence}_summary.json", "w",
              encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
