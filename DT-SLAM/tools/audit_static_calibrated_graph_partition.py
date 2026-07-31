#!/usr/bin/env python3
"""Audit scalar edge-culling topology without making a dynamic decision.

Thresholds come only from the first half of a true-static sequence.  Frozen
RGB-only boxes are used later as development proxies, never as calibration or
pixel/motion ground truth.  This is an adaptation inspired by Dai et al.; it
does not implement their point-correlation optimization.
"""

import argparse
import csv
import json
import math
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np


QUANTILES = (90.0, 95.0, 97.5, 99.0, 99.5)
VARIANTS = ("all_transient", "mappoint_only")
DUPLICATE_TOLERANCE_PX = 1e-3
EPSILON = 1e-12
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite(row, key):
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def load_measured_nodes(path):
    by_frame = defaultdict(dict)
    all_frames = set()
    for row in read_csv(path):
        frame = int(row["frame"])
        all_frames.add(frame)
        if row.get("evidence_state") != "measured":
            continue
        values = [
            finite(row, key) for key in (
                "u_current", "v_current",
                "x_reference_m", "y_reference_m", "z_reference_m",
                "x_current_m", "y_current_m", "z_current_m")]
        if any(value is None for value in values):
            continue
        u, v, xr, yr, zr, xc, yc, zc = values
        if zr <= 0.0 or zc <= 0.0:
            continue
        feature = int(row["feature_index"])
        by_frame[frame][feature] = {
            "feature": feature,
            "u": u,
            "v": v,
            "reference": np.asarray((xr, yr, zr), dtype=np.float64),
            "current": np.asarray((xc, yc, zc), dtype=np.float64),
            "has_mappoint": int(row.get("has_mappoint", "0")) != 0,
        }
    return by_frame, sorted(all_frames)


def deduplicate_nodes(nodes):
    accepted = {}
    cells = defaultdict(list)
    rejected = 0
    for feature in sorted(nodes):
        node = nodes[feature]
        cell = (
            math.floor(node["u"]/DUPLICATE_TOLERANCE_PX),
            math.floor(node["v"]/DUPLICATE_TOLERANCE_PX))
        duplicate = False
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                for other_feature in cells[(cell[0]+dx, cell[1]+dy)]:
                    other = accepted[other_feature]
                    if math.hypot(
                            node["u"]-other["u"],
                            node["v"]-other["v"]) <= \
                            DUPLICATE_TOLERANCE_PX:
                        duplicate = True
                        break
                if duplicate:
                    break
            if duplicate:
                break
        if duplicate:
            rejected += 1
            continue
        accepted[feature] = node
        cells[cell].append(feature)
    return accepted, rejected


def delaunay_edges(nodes):
    nodes, duplicate_count = deduplicate_nodes(nodes)
    if len(nodes) < 3:
        return nodes, [], duplicate_count

    subdiv = cv2.Subdiv2D((0, 0, IMAGE_WIDTH, IMAGE_HEIGHT))
    feature_by_vertex = {}
    accepted = {}
    for feature in sorted(nodes):
        node = nodes[feature]
        try:
            vertex = int(subdiv.insert((node["u"], node["v"])))
        except cv2.error:
            continue
        if vertex in feature_by_vertex:
            continue
        feature_by_vertex[vertex] = feature
        accepted[feature] = node
    nodes = accepted

    pairs = set()
    for leading in subdiv.getLeadingEdgeList():
        edge = int(leading)
        for _ in range(3):
            origin = int(subdiv.edgeOrg(edge)[0])
            destination = int(subdiv.edgeDst(edge)[0])
            if origin in feature_by_vertex and destination in feature_by_vertex:
                a = feature_by_vertex[origin]
                b = feature_by_vertex[destination]
                if a != b:
                    pairs.add((min(a, b), max(a, b)))
            edge = int(cv2.Subdiv2D.getEdge(
                subdiv, edge, cv2.Subdiv2D_NEXT_AROUND_LEFT))

    edges = []
    for a, b in sorted(pairs):
        first = nodes[a]
        second = nodes[b]
        reference_distance = float(np.linalg.norm(
            first["reference"]-second["reference"]))
        current_distance = float(np.linalg.norm(
            first["current"]-second["current"]))
        edges.append({
            "a": a,
            "b": b,
            "absolute_strain_m": abs(
                current_distance-reference_distance),
        })
    return nodes, edges, duplicate_count


def build_graphs(nodes_by_frame, frames):
    graphs = {variant: {} for variant in VARIANTS}
    for frame in frames:
        frame_nodes = nodes_by_frame.get(frame, {})
        for variant in VARIANTS:
            selected = {
                feature: node for feature, node in frame_nodes.items()
                if variant == "all_transient" or node["has_mappoint"]}
            nodes, edges, duplicates = delaunay_edges(selected)
            graphs[variant][frame] = {
                "nodes": nodes,
                "edges": edges,
                "duplicate_count": duplicates,
            }
    return graphs


def load_edge_reference(path):
    by_frame = defaultdict(dict)
    for row in read_csv(path):
        frame = int(row["frame"])
        a = int(row["feature_index_a"])
        b = int(row["feature_index_b"])
        pair = (min(a, b), max(a, b))
        by_frame[frame][pair] = float(row["absolute_strain_m"])
    return by_frame


def edge_parity(graphs, edge_reference):
    missing = 0
    extra = 0
    comparable = 0
    maximum_difference = 0.0
    frame_count = 0
    for frame, graph in graphs["all_transient"].items():
        rebuilt = {
            (edge["a"], edge["b"]): edge["absolute_strain_m"]
            for edge in graph["edges"]}
        expected = edge_reference.get(frame, {})
        if rebuilt or expected:
            frame_count += 1
        missing += len(set(expected)-set(rebuilt))
        extra += len(set(rebuilt)-set(expected))
        for pair in set(rebuilt) & set(expected):
            comparable += 1
            maximum_difference = max(
                maximum_difference, abs(rebuilt[pair]-expected[pair]))
    return {
        "frames_with_edges": frame_count,
        "comparable_edges": comparable,
        "missing_edges": missing,
        "extra_edges": extra,
        "max_absolute_strain_difference_m": maximum_difference,
        "pass": missing == 0 and extra == 0 and
        maximum_difference <= 1e-6,
    }


def partition(graph, threshold):
    nodes = graph["nodes"]
    if len(nodes) < 3:
        return {
            "state": "insufficient_nodes",
            "components": [],
            "primary": set(),
            "outside": set(nodes),
            "isolated": set(nodes),
            "retained_edges": 0,
        }
    adjacency = {feature: set() for feature in nodes}
    retained_edges = 0
    for edge in graph["edges"]:
        if edge["absolute_strain_m"] <= threshold:
            adjacency[edge["a"]].add(edge["b"])
            adjacency[edge["b"]].add(edge["a"])
            retained_edges += 1
    if not graph["edges"]:
        state = "insufficient_edges"
    else:
        state = "measured"

    components = []
    unseen = set(nodes)
    while unseen:
        seed = min(unseen)
        component = set()
        queue = deque((seed,))
        unseen.remove(seed)
        while queue:
            current = queue.popleft()
            component.add(current)
            for neighbor in sorted(adjacency[current]):
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        components.append(component)
    components.sort(key=lambda group: (-len(group), min(group)))
    primary = components[0] if components else set()
    if state == "measured":
        state = "single_component" if len(components) == 1 else "partitioned"
    return {
        "state": state,
        "components": components,
        "primary": primary,
        "outside": set(nodes)-primary,
        "isolated": {
            feature for feature, neighbors in adjacency.items()
            if not neighbors},
        "retained_edges": retained_edges,
    }


def load_boxes(path):
    boxes = {}
    for row in read_csv(path):
        if row.get("visibility") == "absent":
            continue
        keys = ("x", "y", "width", "height")
        if not all(row.get(key, "") for key in keys):
            continue
        export = row["export_name"]
        sequence = "balloon2" if export.startswith("balloon2_") else \
            "balloon" if export.startswith("balloon_") else None
        if sequence is None:
            continue
        boxes[(sequence, int(row["source_frame"]))] = {
            key: int(row[key]) for key in keys}
    return boxes


def is_inside(node, box):
    return (
        box["x"] <= node["u"] < box["x"]+box["width"] and
        box["y"] <= node["v"] < box["y"]+box["height"])


def frame_row(role, sequence, frame, variant, quantile, threshold,
              graph, result, box=None):
    nodes = graph["nodes"]
    outside = result["outside"]
    node_count = len(nodes)
    row = {
        "dataset_role": role,
        "sequence": sequence,
        "frame": frame,
        "variant": variant,
        "quantile": quantile,
        "threshold_m": threshold,
        "state": result["state"],
        "node_count": node_count,
        "edge_count": len(graph["edges"]),
        "retained_edge_count": result["retained_edges"],
        "component_count": len(result["components"]),
        "primary_node_count": len(result["primary"]),
        "outside_primary_node_count": len(outside),
        "outside_primary_fraction": (
            len(outside)/node_count if node_count else ""),
        "isolated_node_count": len(result["isolated"]),
        "isolated_node_fraction": (
            len(result["isolated"])/node_count if node_count else ""),
        "bbox_node_count": "",
        "bbox_outside_primary_count": "",
        "bbox_recall_proxy": "",
        "outside_primary_precision_proxy": "",
        "background_outside_rate": "",
        "enrichment_proxy": "",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    if box is None:
        return row
    bbox_nodes = {
        feature for feature, node in nodes.items()
        if is_inside(node, box)}
    bbox_outside = bbox_nodes & outside
    nonbbox_nodes = set(nodes)-bbox_nodes
    background_outside = nonbbox_nodes & outside
    bbox_rate = len(bbox_outside)/(len(bbox_nodes)+EPSILON)
    background_rate = len(background_outside)/(len(nonbbox_nodes)+EPSILON)
    row.update({
        "bbox_node_count": len(bbox_nodes),
        "bbox_outside_primary_count": len(bbox_outside),
        "bbox_recall_proxy": bbox_rate,
        "outside_primary_precision_proxy": (
            len(bbox_outside)/(len(outside)+EPSILON)),
        "background_outside_rate": background_rate,
        "enrichment_proxy": bbox_rate/(background_rate+EPSILON),
    })
    return row


def aggregate_static(rows):
    fractions = [
        float(row["outside_primary_fraction"]) for row in rows
        if row["outside_primary_fraction"] != "" and
        not row["state"].startswith("insufficient")]
    return {
        "frame_count": len(rows),
        "measured_frame_count": len(fractions),
        "insufficient_frame_count": sum(
            row["state"].startswith("insufficient") for row in rows),
        "outside_primary_median": percentile(fractions, 50),
        "outside_primary_p90": percentile(fractions, 90),
        "outside_primary_p95": percentile(fractions, 95),
        "outside_primary_gt_1pct_frame_fraction": (
            sum(value > 0.01 for value in fractions)/len(fractions)
            if fractions else None),
        "outside_primary_gt_5pct_frame_fraction": (
            sum(value > 0.05 for value in fractions)/len(fractions)
            if fractions else None),
        "outside_primary_gt_10pct_frame_fraction": (
            sum(value > 0.10 for value in fractions)/len(fractions)
            if fractions else None),
    }


def aggregate_dynamic(rows):
    supported = [
        row for row in rows if row["bbox_node_count"] != "" and
        int(row["bbox_node_count"]) >= 3]
    recalls = [float(row["bbox_recall_proxy"]) for row in supported]
    enrichments = [float(row["enrichment_proxy"]) for row in supported]
    return {
        "candidate_frame_count": len(rows),
        "supported_frame_count": len(supported),
        "supported_frame_fraction": (
            len(supported)/len(rows) if rows else None),
        "bbox_recall_ge_50pct_frame_fraction": (
            sum(value >= 0.5 for value in recalls)/len(recalls)
            if recalls else None),
        "bbox_recall_proxy_median": percentile(recalls, 50),
        "enrichment_proxy_median": percentile(enrichments, 50),
    }


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--static-nodes", type=Path, required=True)
    parser.add_argument("--static-edges", type=Path, required=True)
    parser.add_argument(
        "--dynamic", action="append", nargs=3, required=True,
        metavar=("SEQUENCE", "NODE_CSV", "EDGE_CSV"))
    parser.add_argument("--bbox-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    static_nodes, all_static_frames = load_measured_nodes(args.static_nodes)
    static_graphs = build_graphs(static_nodes, all_static_frames)
    static_frames = [
        frame for frame in all_static_frames
        if len(static_graphs["all_transient"][frame]["nodes"]) >= 3 and
        static_graphs["all_transient"][frame]["edges"]]
    split = len(static_frames)//2
    calibration_frames = static_frames[:split]
    validation_frames = static_frames[split:]
    if not calibration_frames or not validation_frames:
        raise RuntimeError("static chronological split is empty")

    thresholds = {}
    for variant in VARIANTS:
        strains = [
            edge["absolute_strain_m"]
            for frame in calibration_frames
            for edge in static_graphs[variant][frame]["edges"]]
        if not strains:
            raise RuntimeError(
                f"no static calibration edges for {variant}")
        thresholds[variant] = {
            str(q): percentile(strains, q) for q in QUANTILES}

    parity = {
        "static": edge_parity(
            static_graphs, load_edge_reference(args.static_edges))}
    dynamic_graphs = {}
    dynamic_frames = {}
    for sequence, node_path, edge_path in args.dynamic:
        nodes, frames = load_measured_nodes(Path(node_path))
        graphs = build_graphs(nodes, frames)
        dynamic_graphs[sequence] = graphs
        dynamic_frames[sequence] = frames
        parity[sequence] = edge_parity(
            graphs, load_edge_reference(Path(edge_path)))

    boxes = load_boxes(args.bbox_csv)
    rows = []
    aggregate = {}
    for variant in VARIANTS:
        aggregate[variant] = {}
        for quantile in QUANTILES:
            threshold = thresholds[variant][str(quantile)]
            static_rows = []
            for frame in validation_frames:
                graph = static_graphs[variant][frame]
                result = partition(graph, threshold)
                row = frame_row(
                    "static_validation", "bonn_static_close_far", frame,
                    variant, quantile, threshold, graph, result)
                rows.append(row)
                static_rows.append(row)

            dynamic_rows = []
            for sequence in sorted(dynamic_graphs):
                candidate_frames = sorted(
                    frame for frame in dynamic_frames[sequence]
                    if (sequence, frame) in boxes)
                for frame in candidate_frames:
                    graph = dynamic_graphs[sequence][variant][frame]
                    result = partition(graph, threshold)
                    row = frame_row(
                        "dynamic_development_proxy", sequence, frame,
                        variant, quantile, threshold, graph, result,
                        boxes[(sequence, frame)])
                    rows.append(row)
                    dynamic_rows.append(row)

            static_summary = aggregate_static(static_rows)
            dynamic_summary = aggregate_dynamic(dynamic_rows)
            gate = {
                "static_p95_le_5pct":
                    static_summary["outside_primary_p95"] is not None and
                    static_summary["outside_primary_p95"] <= 0.05,
                "support_ge_half":
                    dynamic_summary["supported_frame_fraction"] is not None and
                    dynamic_summary["supported_frame_fraction"] >= 0.5,
                "recall_ge_half_on_half_supported":
                    dynamic_summary[
                        "bbox_recall_ge_50pct_frame_fraction"] is not None and
                    dynamic_summary[
                        "bbox_recall_ge_50pct_frame_fraction"] >= 0.5,
                "median_enrichment_ge_3":
                    dynamic_summary["enrichment_proxy_median"] is not None and
                    dynamic_summary["enrichment_proxy_median"] >= 3.0,
            }
            gate["pass"] = all(gate.values())
            aggregate[variant][str(quantile)] = {
                "threshold_m": threshold,
                "static_validation": static_summary,
                "dynamic_development_proxy": dynamic_summary,
                "preregistered_gate": gate,
            }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir/"per_frame_partition.csv", rows)
    threshold_output = {
        "method_identity":
            "[A/H] Dai-inspired scalar-strain topology feasibility audit",
        "calibration_source": "Bonn static_close_far chronological first half",
        "calibration_frames": calibration_frames,
        "validation_frames": validation_frames,
        "quantiles": list(QUANTILES),
        "thresholds_m": thresholds,
        "edge_rebuild_parity": parity,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    with open(
            args.output_dir/"static_thresholds.json",
            "w", encoding="utf-8") as stream:
        json.dump(threshold_output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    summary = {
        "proxy_identity":
            "frozen RGB-only coarse bbox; not motion ground truth",
        "sealed_holdout_used": False,
        "variants": aggregate,
        "any_all_transient_gate_pass": any(
            item["preregistered_gate"]["pass"]
            for item in aggregate["all_transient"].values()),
        "any_mappoint_only_gate_pass": any(
            item["preregistered_gate"]["pass"]
            for item in aggregate["mappoint_only"].values()),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    with open(
            args.output_dir/"aggregate_summary.json",
            "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
