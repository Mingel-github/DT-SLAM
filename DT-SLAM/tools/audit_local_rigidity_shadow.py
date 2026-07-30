#!/usr/bin/env python3
"""Audit G2-4F3 continuous local-rigidity evidence on frozen RGB proxies.

This tool intentionally makes no dynamic decision. It reuses exact feature
correspondences and semantic flags exported by G2-4F1, reads current depth
from a Bonn archive, applies the same joint RGB-D rectification model, and
compares continuous edge-strain distributions inside/outside frozen RGB-only
coarse object boxes.
"""

import argparse
import csv
import io
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
FB_MAX_PIXELS = 0.25
RELATIVE_DENOMINATOR_FLOOR_METERS = 1e-4
NUMERICAL_DUPLICATE_TOLERANCE_PIXELS = 1e-3


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def finite_float(row, key):
    value = row.get(key, "")
    if value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def back_project(u, v, depth):
    return np.array(
        [(u-BONN_K[0, 2])*depth/BONN_K[0, 0],
         (v-BONN_K[1, 2])*depth/BONN_K[1, 1],
         depth],
        dtype=np.float32)


def load_rectified_depth(archive, archive_root, relative_path, map_x, map_y):
    member = f"{archive_root.rstrip('/')}/{relative_path}"
    raw = archive.read(member)
    encoded = np.frombuffer(raw, dtype=np.uint8)
    depth_raw = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if depth_raw is None or depth_raw.dtype != np.uint16:
        raise RuntimeError(f"failed to decode uint16 depth: {member}")
    depth_rectified = cv2.remap(
        depth_raw, map_x, map_y, cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return depth_rectified.astype(np.float32)/DEPTH_SCALE


def build_graph(rows, current_depth):
    nodes = {}
    rejected = defaultdict(int)
    for row in rows:
        feature = int(row["feature_index"])
        if row["evidence_state"] != "measured":
            rejected["sparse_flow_invalid"] += 1
            continue
        fb = finite_float(row, "forward_backward_error_px")
        if fb is None or fb > FB_MAX_PIXELS:
            rejected["forward_backward_rejected"] += 1
            continue
        if int(row["semantic_nonzero"]) != 0:
            rejected["semantic_excluded"] += 1
            continue
        reference_depth = finite_float(row, "reference_depth_m")
        if reference_depth is None or reference_depth <= 0.0:
            rejected["reference_depth_invalid"] += 1
            continue
        u_current = float(row["u_current"])
        v_current = float(row["v_current"])
        u_reference = finite_float(row, "u_reference")
        v_reference = finite_float(row, "v_reference")
        if u_reference is None or v_reference is None:
            rejected["reference_pixel_invalid"] += 1
            continue
        u_round = int(round(u_current))
        v_round = int(round(v_current))
        if not (0 <= u_round < current_depth.shape[1] and
                0 <= v_round < current_depth.shape[0]):
            rejected["outside_image"] += 1
            continue
        current_z = float(current_depth[v_round, u_round])
        if not math.isfinite(current_z) or current_z <= 0.0:
            rejected["current_depth_invalid"] += 1
            continue
        nodes[feature] = {
            "feature": feature,
            "u": u_current,
            "v": v_current,
            "reference_point": back_project(
                u_reference, v_reference, reference_depth),
            "current_point": back_project(
                u_current, v_current, current_z),
            "flow_residual": float(row["slam_residual_magnitude_px"]),
        }

    subdiv = cv2.Subdiv2D(
        (0, 0, current_depth.shape[1], current_depth.shape[0]))
    feature_by_vertex = {}
    accepted_cells = defaultdict(list)
    for feature in sorted(nodes):
        node = nodes[feature]
        cell = (
            math.floor(
                node["u"]/NUMERICAL_DUPLICATE_TOLERANCE_PIXELS),
            math.floor(
                node["v"]/NUMERICAL_DUPLICATE_TOLERANCE_PIXELS))
        near_duplicate = False
        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                for nearby_feature in accepted_cells[
                        (cell[0]+offset_x, cell[1]+offset_y)]:
                    nearby = nodes[nearby_feature]
                    if math.hypot(
                            node["u"]-nearby["u"],
                            node["v"]-nearby["v"]) <= \
                            NUMERICAL_DUPLICATE_TOLERANCE_PIXELS:
                        near_duplicate = True
                        break
                if near_duplicate:
                    break
            if near_duplicate:
                break
        if near_duplicate:
            rejected["duplicate_image_point"] += 1
            del nodes[feature]
            continue
        try:
            vertex = int(subdiv.insert((node["u"], node["v"])))
        except cv2.error:
            rejected["outside_image"] += 1
            del nodes[feature]
            continue
        if vertex in feature_by_vertex:
            rejected["duplicate_image_point"] += 1
            del nodes[feature]
            continue
        feature_by_vertex[vertex] = feature
        accepted_cells[cell].append(feature)

    unique_edges = set()
    for leading in subdiv.getLeadingEdgeList():
        edge = int(leading)
        for _ in range(3):
            origin = int(subdiv.edgeOrg(edge)[0])
            destination = int(subdiv.edgeDst(edge)[0])
            if (origin in feature_by_vertex and
                    destination in feature_by_vertex):
                a = feature_by_vertex[origin]
                b = feature_by_vertex[destination]
                if a != b:
                    unique_edges.add((min(a, b), max(a, b)))
            edge = int(cv2.Subdiv2D.getEdge(
                subdiv, edge, cv2.Subdiv2D_NEXT_AROUND_LEFT))

    edges = []
    incident = defaultdict(list)
    for a, b in sorted(unique_edges):
        first = nodes[a]
        second = nodes[b]
        d_reference = float(np.linalg.norm(
            first["reference_point"]-second["reference_point"]))
        d_current = float(np.linalg.norm(
            first["current_point"]-second["current_point"]))
        absolute = abs(d_current-d_reference)
        relative = absolute/max(
            RELATIVE_DENOMINATOR_FLOOR_METERS,
            0.5*(d_current+d_reference))
        edge = {
            "feature_index_a": a,
            "feature_index_b": b,
            "reference_distance_m": d_reference,
            "current_distance_m": d_current,
            "absolute_strain_m": absolute,
            "relative_strain": relative,
        }
        edges.append(edge)
        incident[a].append(edge)
        incident[b].append(edge)
    return nodes, edges, incident, dict(rejected)


def point_inside(node, box):
    return (
        box["x"] <= node["u"] < box["x"]+box["width"] and
        box["y"] <= node["v"] < box["y"]+box["height"])


def summarize_frame(frame, nodes, edges, incident, rejected, box):
    node_inside = {
        feature: point_inside(node, box)
        for feature, node in nodes.items()}
    edge_groups = defaultdict(list)
    for edge in edges:
        inside_a = node_inside[edge["feature_index_a"]]
        inside_b = node_inside[edge["feature_index_b"]]
        if inside_a and inside_b:
            group = "inside_inside"
        elif not inside_a and not inside_b:
            group = "outside_outside"
        else:
            group = "crossing"
        edge_groups[group].append(edge)

    inside_flow = [
        node["flow_residual"] for feature, node in nodes.items()
        if node_inside[feature]]
    outside_flow = [
        node["flow_residual"] for feature, node in nodes.items()
        if not node_inside[feature]]
    summary = {
        "frame": frame,
        "eligible_nodes": len(nodes),
        "inside_nodes": len(inside_flow),
        "outside_nodes": len(outside_flow),
        "valid_edges": len(edges),
        "inside_inside_edges": len(edge_groups["inside_inside"]),
        "outside_outside_edges": len(edge_groups["outside_outside"]),
        "crossing_edges": len(edge_groups["crossing"]),
        "inside_flow_median_px": percentile(inside_flow, 50),
        "outside_flow_median_px": percentile(outside_flow, 50),
        "inside_flow_p90_px": percentile(inside_flow, 90),
        "outside_flow_p90_px": percentile(outside_flow, 90),
        "inside_inside_absolute_strain_median_m": percentile(
            [edge["absolute_strain_m"]
             for edge in edge_groups["inside_inside"]], 50),
        "outside_outside_absolute_strain_median_m": percentile(
            [edge["absolute_strain_m"]
             for edge in edge_groups["outside_outside"]], 50),
        "crossing_absolute_strain_median_m": percentile(
            [edge["absolute_strain_m"]
             for edge in edge_groups["crossing"]], 50),
        "inside_inside_relative_strain_median": percentile(
            [edge["relative_strain"]
             for edge in edge_groups["inside_inside"]], 50),
        "outside_outside_relative_strain_median": percentile(
            [edge["relative_strain"]
             for edge in edge_groups["outside_outside"]], 50),
        "crossing_relative_strain_median": percentile(
            [edge["relative_strain"]
             for edge in edge_groups["crossing"]], 50),
        "rejected": rejected,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    for edge in edges:
        edge["frame"] = frame
        inside_a = node_inside[edge["feature_index_a"]]
        inside_b = node_inside[edge["feature_index_b"]]
        edge["bbox_relation"] = (
            "inside_inside" if inside_a and inside_b else
            "outside_outside" if not inside_a and not inside_b else
            "crossing")
    node_rows = []
    for feature, node in sorted(nodes.items()):
        same_group_edges = [
            edge for edge in incident[feature]
            if node_inside[edge["feature_index_a"]] ==
               node_inside[edge["feature_index_b"]]]
        node_rows.append({
            "frame": frame,
            "feature_index": feature,
            "u_current": node["u"],
            "v_current": node["v"],
            "inside_box": int(node_inside[feature]),
            "flow_residual_magnitude_px": node["flow_residual"],
            "valid_neighbors": len(incident[feature]),
            "incident_absolute_strain_median_m": percentile(
                [edge["absolute_strain_m"]
                 for edge in incident[feature]], 50),
            "incident_relative_strain_median": percentile(
                [edge["relative_strain"]
                 for edge in incident[feature]], 50),
            "same_group_neighbors": len(same_group_edges),
            "same_group_absolute_strain_median_m": percentile(
                [edge["absolute_strain_m"]
                 for edge in same_group_edges], 50),
            "same_group_relative_strain_median": percentile(
                [edge["relative_strain"]
                 for edge in same_group_edges], 50),
            "dynamic_decision": "none",
            "direct_slam_state_mutation": "none",
        })
    return summary, node_rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--feature-csv", required=True, type=Path)
    parser.add_argument("--candidate-csv", required=True, type=Path)
    parser.add_argument("--bbox-csv", required=True, type=Path)
    parser.add_argument("--archive", required=True, type=Path)
    parser.add_argument("--archive-root", required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    feature_rows = read_csv(args.feature_csv)
    candidates = {
        int(row["frame"]): row for row in read_csv(args.candidate_csv)}
    boxes = {
        int(row["source_frame"]): {
            key: int(row[key]) for key in ("x", "y", "width", "height")}
        for row in read_csv(args.bbox_csv)
        if row["export_name"].startswith(args.sequence+"_") and
        row.get("visibility") != "absent" and
        all(row.get(key, "") for key in ("x", "y", "width", "height"))}
    by_frame = defaultdict(list)
    for row in feature_rows:
        by_frame[int(row["frame"])].append(row)
    frames = sorted(set(by_frame) & set(candidates) & set(boxes))
    if not frames:
        raise RuntimeError("no common feature/candidate/bbox frames")

    map_x, map_y = cv2.initUndistortRectifyMap(
        BONN_K, BONN_D, np.eye(3, dtype=np.float32), BONN_K,
        (640, 480), cv2.CV_32FC1)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frame_summaries = []
    all_nodes = []
    all_edges = []
    with ZipFile(args.archive) as archive:
        for frame in frames:
            current_depth = load_rectified_depth(
                archive, args.archive_root,
                candidates[frame]["depth_relative"], map_x, map_y)
            nodes, edges, incident, rejected = build_graph(
                by_frame[frame], current_depth)
            summary, node_rows = summarize_frame(
                frame, nodes, edges, incident, rejected, boxes[frame])
            frame_summaries.append(summary)
            all_nodes.extend(node_rows)
            all_edges.extend(edges)

    frame_csv = args.output_dir/f"{args.sequence}_per_frame.csv"
    flat_rows = []
    for row in frame_summaries:
        flat = {
            key: value for key, value in row.items()
            if key != "rejected"}
        flat["rejected_json"] = json.dumps(
            row["rejected"], sort_keys=True)
        flat_rows.append(flat)
    with open(frame_csv, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat_rows[0]))
        writer.writeheader()
        writer.writerows(flat_rows)

    edge_csv = args.output_dir/f"{args.sequence}_edges.csv"
    with open(edge_csv, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_edges[0]))
        writer.writeheader()
        writer.writerows(all_edges)

    node_csv = args.output_dir/f"{args.sequence}_nodes.csv"
    with open(node_csv, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_nodes[0]))
        writer.writeheader()
        writer.writerows(all_nodes)

    valid_inside = [
        row for row in frame_summaries
        if row["inside_nodes"] > 0]
    flow_greater = sum(
        row["inside_flow_median_px"] >
        row["outside_flow_median_px"]
        for row in valid_inside
        if row["inside_flow_median_px"] is not None and
        row["outside_flow_median_px"] is not None)
    strain_available = [
        row for row in frame_summaries
        if row["inside_inside_absolute_strain_median_m"] is not None and
        row["outside_outside_absolute_strain_median_m"] is not None]
    inside_strain_not_greater = sum(
        row["inside_inside_absolute_strain_median_m"] <=
        row["outside_outside_absolute_strain_median_m"]
        for row in strain_available)
    crossing_strain_greater = sum(
        row["crossing_absolute_strain_median_m"] is not None and
        row["crossing_absolute_strain_median_m"] >
        row["outside_outside_absolute_strain_median_m"]
        for row in strain_available)
    summary = {
        "sequence": args.sequence,
        "frame_count": len(frame_summaries),
        "proxy_identity":
            "frozen RGB-only coarse boxes; not motion ground truth",
        "exact_semantic_source":
            "saved G2-4F1 per-feature semantic_nonzero",
        "flow_inside_median_gt_outside_count": flow_greater,
        "flow_comparable_frame_count": len(valid_inside),
        "inside_strain_le_outside_count": inside_strain_not_greater,
        "strain_comparable_frame_count": len(strain_available),
        "crossing_strain_gt_outside_count": crossing_strain_greater,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "method_identity": {
            "delaunay_local_graph": "[A] Dai et al. point correlation",
            "two_frame_edge_length_change": "[A/H]",
            "flow_residual": "[A] FlowFusion-inspired sparse residual",
            "joint_interpretation": "[S/H]",
        },
    }
    with open(
            args.output_dir/f"{args.sequence}_summary.json",
            "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
