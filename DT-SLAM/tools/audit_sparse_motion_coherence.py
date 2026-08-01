#!/usr/bin/env python3
"""Audit continuous sparse residual-vector coherence in coarse target boxes.

This is a read-only input-feasibility audit. It does not cluster features,
select a dynamic threshold, infer an object, or mutate SLAM state.
"""

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


EPSILON = 1e-9
REQUIRED_FEATURE_COLUMNS = {
    "frame",
    "feature_index",
    "u_current",
    "v_current",
    "semantic_nonzero",
    "quality_eligible",
    "q_candidate",
    "slam_residual_x_px",
    "slam_residual_y_px",
    "slam_residual_magnitude_px",
    "evidence_state",
}
REQUIRED_BOX_COLUMNS = {
    "export_name",
    "source_frame",
    "x",
    "y",
    "width",
    "height",
}
REQUIRED_MOTION_COLUMNS = {
    "source_frame",
    "motion_label",
    "confidence",
}


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--features", type=Path, required=True)
    parser.add_argument("--boxes", type=Path, required=True)
    parser.add_argument("--export-name", required=True)
    parser.add_argument("--sequence-name", required=True)
    parser.add_argument("--motion-proxy", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def read_csv(path, required):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "{} missing columns: {}".format(
                    path, ",".join(sorted(missing))))
        return list(reader)


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def vector_stats(vectors, q_candidates):
    if not vectors:
        return {
            "count": 0,
            "q10_count": 0,
            "magnitude_median_px": None,
            "magnitude_p90_px": None,
            "median_x_px": None,
            "median_y_px": None,
            "median_vector_magnitude_px": None,
            "dispersion_median_px": None,
            "direction_concentration": None,
        }
    xs = [item[0] for item in vectors]
    ys = [item[1] for item in vectors]
    magnitudes = [math.hypot(x, y) for x, y in vectors]
    median_x = statistics.median(xs)
    median_y = statistics.median(ys)
    dispersion = statistics.median(
        math.hypot(x - median_x, y - median_y) for x, y in vectors)
    sum_x = sum(xs)
    sum_y = sum(ys)
    magnitude_sum = sum(magnitudes)
    concentration = (
        math.hypot(sum_x, sum_y) / magnitude_sum
        if magnitude_sum > EPSILON else 0.0)
    return {
        "count": len(vectors),
        "q10_count": q_candidates,
        "magnitude_median_px": statistics.median(magnitudes),
        "magnitude_p90_px": percentile(magnitudes, 0.90),
        "median_x_px": median_x,
        "median_y_px": median_y,
        "median_vector_magnitude_px": math.hypot(median_x, median_y),
        "dispersion_median_px": dispersion,
        "direction_concentration": concentration,
    }


def inside(row, box):
    u = float(row["u_current"])
    v = float(row["v_current"])
    x, y, width, height = box
    return x <= u < x + width and y <= v < y + height


def eligible_vector(row):
    if (row["evidence_state"] != "measured" or
            row["quality_eligible"] != "1" or
            row["semantic_nonzero"] != "0"):
        return None
    values = (
        float(row["slam_residual_x_px"]),
        float(row["slam_residual_y_px"]),
    )
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def flatten(prefix, values, output):
    for name, value in values.items():
        output[prefix + "_" + name] = value


def tied_rank_auc(positive, negative):
    if not positive or not negative:
        return None
    wins = 0.0
    for positive_value in positive:
        for negative_value in negative:
            if positive_value > negative_value:
                wins += 1.0
            elif positive_value == negative_value:
                wins += 0.5
    return wins / (len(positive) * len(negative))


def labeled_group_summary(rows, label):
    selected = [row for row in rows if row.get("motion_label") == label]
    comparable = [row for row in selected if row["comparable"]]
    numeric_fields = (
        "inside_count",
        "inside_magnitude_median_px",
        "inside_direction_concentration",
        "inside_dispersion_median_px",
        "centroid_separation_px",
        "normalized_separation",
    )
    summary = {
        "frame_count": len(selected),
        "comparable_frames": len(comparable),
        "coherent_proxy_frames": sum(
            row["coherent_proxy_frame"] == 1 for row in comparable),
    }
    for name in numeric_fields:
        values = [
            float(row[name]) for row in comparable
            if row[name] is not None]
        summary[name + "_median"] = (
            statistics.median(values) if values else None)
        summary[name + "_minimum"] = min(values) if values else None
        summary[name + "_maximum"] = max(values) if values else None
    return summary


def audit(feature_rows, box_rows, export_name, sequence_name,
          motion_rows=None):
    by_frame = defaultdict(list)
    duplicate_keys = set()
    seen = set()
    for row in feature_rows:
        item_key = (int(row["frame"]), int(row["feature_index"]))
        if item_key in seen:
            duplicate_keys.add(item_key)
        seen.add(item_key)
        by_frame[item_key[0]].append(row)

    selected = [row for row in box_rows if row["export_name"] == export_name]
    if not selected:
        raise ValueError("no boxes match export-name " + export_name)
    selected_frames = [int(row["source_frame"]) for row in selected]
    if len(selected_frames) != len(set(selected_frames)):
        raise ValueError("box CSV contains duplicate selected frames")

    motion_by_frame = {}
    if motion_rows is not None:
        for row in motion_rows:
            frame = int(row["source_frame"])
            if frame in motion_by_frame:
                raise ValueError("duplicate motion-proxy frame")
            motion_by_frame[frame] = row
        missing_motion = set(selected_frames) - set(motion_by_frame)
        if missing_motion:
            raise ValueError(
                "motion proxy missing selected frames: {}".format(
                    sorted(missing_motion)))

    output_rows = []
    violations = []
    if duplicate_keys:
        violations.append("duplicate feature frame/index keys")
    for box_row in selected:
        frame = int(box_row["source_frame"])
        rows = by_frame.get(frame, [])
        if not rows:
            violations.append("missing feature rows frame {}".format(frame))
            continue
        box = tuple(
            int(round(float(box_row[name])))
            for name in ("x", "y", "width", "height"))
        inside_vectors = []
        outside_vectors = []
        inside_q = 0
        outside_q = 0
        for row in rows:
            vector = eligible_vector(row)
            if vector is None:
                continue
            if inside(row, box):
                inside_vectors.append(vector)
                inside_q += row["q_candidate"] == "1"
            else:
                outside_vectors.append(vector)
                outside_q += row["q_candidate"] == "1"
        in_stats = vector_stats(inside_vectors, inside_q)
        out_stats = vector_stats(outside_vectors, outside_q)
        output = {
            "sequence": sequence_name,
            "frame": frame,
            "motion_label": (
                motion_by_frame[frame]["motion_label"]
                if motion_rows is not None else "unlabeled"),
            "motion_confidence": (
                motion_by_frame[frame]["confidence"]
                if motion_rows is not None else "unlabeled"),
            "inside_support_ge_3": int(in_stats["count"] >= 3),
            "inside_support_ge_7": int(in_stats["count"] >= 7),
            "inside_support_ge_10": int(in_stats["count"] >= 10),
        }
        flatten("inside", in_stats, output)
        flatten("outside", out_stats, output)
        comparable = in_stats["count"] >= 3 and out_stats["count"] >= 3
        output["comparable"] = int(comparable)
        if comparable:
            separation = math.hypot(
                in_stats["median_x_px"] - out_stats["median_x_px"],
                in_stats["median_y_px"] - out_stats["median_y_px"])
            pooled_dispersion = 0.5 * (
                in_stats["dispersion_median_px"] +
                out_stats["dispersion_median_px"])
            output["centroid_separation_px"] = separation
            output["normalized_separation"] = (
                separation / max(EPSILON, pooled_dispersion))
            output["inside_magnitude_gt_outside"] = int(
                in_stats["magnitude_median_px"] >
                out_stats["magnitude_median_px"])
            output["inside_concentration_gt_outside"] = int(
                in_stats["direction_concentration"] >
                out_stats["direction_concentration"])
            output["separation_gt_inside_dispersion"] = int(
                separation > in_stats["dispersion_median_px"])
            output["coherent_proxy_frame"] = int(
                output["inside_concentration_gt_outside"] and
                output["separation_gt_inside_dispersion"])
        else:
            for name in (
                    "centroid_separation_px",
                    "normalized_separation",
                    "inside_magnitude_gt_outside",
                    "inside_concentration_gt_outside",
                    "separation_gt_inside_dispersion",
                    "coherent_proxy_frame"):
                output[name] = None
        output_rows.append(output)

    review_count = len(selected)
    comparable_rows = [row for row in output_rows if row["comparable"]]
    coherent_count = sum(
        row["coherent_proxy_frame"] == 1 for row in comparable_rows)
    # Without an independently assigned motion state, coherence alone cannot
    # distinguish a moving target from a static surface or pose bias.
    interpretation = "unlabeled_descriptive_only"

    def count(name, rows=output_rows):
        return sum(row[name] == 1 for row in rows)

    summary = {
        "identity": (
            "two-frame continuous sparse residual-vector input audit; "
            "not motion grouping or dynamic detection"),
        "sequence": sequence_name,
        "export_name": export_name,
        "review_frames": review_count,
        "audited_frames": len(output_rows),
        "comparable_frames": len(comparable_rows),
        "frames_inside_support_ge_3": count("inside_support_ge_3"),
        "frames_inside_support_ge_7": count("inside_support_ge_7"),
        "frames_inside_support_ge_10": count("inside_support_ge_10"),
        "frames_inside_magnitude_gt_outside": count(
            "inside_magnitude_gt_outside", comparable_rows),
        "frames_inside_concentration_gt_outside": count(
            "inside_concentration_gt_outside", comparable_rows),
        "frames_separation_gt_inside_dispersion": count(
            "separation_gt_inside_dispersion", comparable_rows),
        "coherent_proxy_frames": coherent_count,
        "interpretation": interpretation,
        "invariant_violations": violations,
        "passed_invariants": not violations,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "notes": [
            "coarse RGB-only boxes are not object or motion ground truth",
            "q10 is counted only as a frozen failed baseline",
            "the coherence statistics are project diagnostics, not Lee 2019 rigid hypotheses",
        ],
    }
    if motion_rows is not None:
        group_summaries = {
            label: labeled_group_summary(output_rows, label)
            for label in sorted({row["motion_label"] for row in output_rows})
        }
        moving = [
            row for row in output_rows
            if row["motion_label"] == "moving" and row["comparable"]]
        stationary = [
            row for row in output_rows
            if row["motion_label"] == "stationary" and row["comparable"]]
        auc_fields = (
            "inside_magnitude_median_px",
            "inside_direction_concentration",
            "inside_dispersion_median_px",
            "centroid_separation_px",
            "normalized_separation",
        )
        auc = {}
        for name in auc_fields:
            auc[name] = tied_rank_auc(
                [float(row[name]) for row in moving if row[name] is not None],
                [float(row[name]) for row in stationary
                 if row[name] is not None])
        summary["motion_proxy_identity"] = (
            "pre-existing agent RGB-temporal proxy; not ground truth")
        summary["motion_label_groups"] = group_summaries
        summary["moving_vs_stationary_proxy_auc"] = auc
        summary["interpretation"] = (
            "descriptive_only_after_mixed_motion_correction")
    return output_rows, summary


def self_test():
    vectors = [(2.0, 0.0), (2.0, 0.0), (2.0, 0.0)]
    stats = vector_stats(vectors, 1)
    assert stats["count"] == 3
    assert abs(stats["median_x_px"] - 2.0) < EPSILON
    assert abs(stats["dispersion_median_px"]) < EPSILON
    assert abs(stats["direction_concentration"] - 1.0) < EPSILON
    cancel = vector_stats([(1.0, 0.0), (-1.0, 0.0)], 0)
    assert abs(cancel["direction_concentration"]) < EPSILON
    print("audit_sparse_motion_coherence self-test: PASS")


def main():
    args = arguments()
    if args.self_test:
        self_test()
    features = read_csv(args.features, REQUIRED_FEATURE_COLUMNS)
    boxes = read_csv(args.boxes, REQUIRED_BOX_COLUMNS)
    motion = (
        read_csv(args.motion_proxy, REQUIRED_MOTION_COLUMNS)
        if args.motion_proxy is not None else None)
    rows, summary = audit(
        features, boxes, args.export_name, args.sequence_name, motion)
    args.output_directory.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError("audit produced no per-frame rows")
    with (args.output_directory / "per_frame.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.output_directory / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not summary["passed_invariants"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
