#!/usr/bin/env python3
"""Audit gradient-only RAG labels as region evidence, never dynamics."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

from audit_sin_style_native_initial_regions import (
    adjusted_rand_index,
    internal_boundaries,
    normalized_mutual_information,
)


def mean(values):
    return float(np.mean(values)) if values else None


def median(values):
    return float(np.median(values)) if values else None


def boundary_precision_recall(
    candidate_labels, retained_gradient_edge, reference_labels, tolerance
):
    # RAG labels intentionally keep gradient-edge pixels at label zero, so an
    # internal-boundary operator alone would miss most retained boundaries.
    candidate = cv2.bitwise_or(
        internal_boundaries(candidate_labels),
        np.where(retained_gradient_edge > 0, 255, 0).astype(np.uint8),
    )
    reference = internal_boundaries(reference_labels)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * tolerance + 1, 2 * tolerance + 1)
    )
    candidate_dilated = cv2.dilate(candidate, kernel)
    reference_dilated = cv2.dilate(reference, kernel)
    candidate_count = int(np.count_nonzero(candidate))
    reference_count = int(np.count_nonzero(reference))
    precision = (
        float(np.count_nonzero((candidate > 0) & (reference_dilated > 0)))
        / candidate_count
        if candidate_count
        else None
    )
    recall = (
        float(np.count_nonzero((reference > 0) & (candidate_dilated > 0)))
        / reference_count
        if reference_count
        else None
    )
    return precision, recall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--gradient-dir", required=True, type=Path)
    parser.add_argument("--initial-dir", required=True, type=Path)
    parser.add_argument("--rag-dir", required=True, type=Path)
    parser.add_argument("--repeat-rag-dir", type=Path)
    parser.add_argument("--reference-dir", type=Path)
    parser.add_argument(
        "--allow-missing-reference-index", action="append", default=[], type=int
    )
    parser.add_argument("--expected-rows", required=True, type=int)
    parser.add_argument("--boundary-tolerance", default=2, type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    errors = []
    if len(rows) != args.expected_rows:
        errors.append(f"row count {len(rows)} != expected {args.expected_rows}")
    if [int(row["input_index"]) for row in rows] != list(range(len(rows))):
        errors.append("input indices are not unique and continuous from zero")

    input_counts = []
    output_counts = []
    region_reduction_ratios = []
    high_merges = []
    low_merges = []
    unmerged_low = []
    adjacent_pairs = []
    fake_pairs = []
    eligible_pairs = []
    rejected_pairs = []
    total_times = []
    graph_times = []
    merge_times = []
    ari_values = []
    nmi_values = []
    boundary_precisions = []
    boundary_recalls = []
    deterministic_frames = 0
    reference_frames = 0
    allowed_missing = set(args.allow_missing_reference_index)

    for row in rows:
        index = int(row["input_index"])
        if int(row["native_rag_enabled"]) != 1:
            errors.append(f"input {index}: RAG disabled")
        if int(row["native_rag_available"]) != 1:
            errors.append(f"input {index}: RAG unavailable")
        if int(row["native_rag_dynamic_state_available"]) != 0:
            errors.append(f"input {index}: RAG exposes dynamic state")
        if int(row["native_rag_plane_rejection_available"]) != 0:
            errors.append(f"input {index}: missing plane rejection reported available")
        if row["native_rag_dynamic_decision"] != "none":
            errors.append(f"input {index}: RAG dynamic decision is not none")
        if row["direct_slam_state_mutation"] != "none" or int(
            row["actual_slam_removed"]
        ) != 0:
            errors.append(f"input {index}: shadow-only invariant failed")
        if int(row["native_rag_labels_written"]) != 1:
            errors.append(f"input {index}: RAG label write not confirmed")

        input_count = int(row["native_rag_input_component_count"])
        output_count = int(row["native_rag_output_region_count"])
        high_merge = int(row["native_rag_high_middle_merges"])
        low_merge = int(row["native_rag_low_merges"])
        if output_count != input_count - high_merge - low_merge:
            errors.append(f"input {index}: region/merge accounting failed")
        if output_count <= 0 or output_count > input_count:
            errors.append(f"input {index}: invalid output region count")
        if int(row["native_rag_cross_gradient_merge_violations"]) != 0:
            errors.append(f"input {index}: cross-gradient merge reported")
        if int(row["native_rag_input_core_pixels"]) != int(
            row["native_rag_output_core_pixels"]
        ):
            errors.append(f"input {index}: RAG changed core pixel count")

        split_path = args.gradient_dir / (
            f"frame_{index:06d}_native_gradient_split_labels.png"
        )
        edge_path = args.gradient_dir / f"frame_{index:06d}_native_gradient_edge.png"
        initial_path = args.initial_dir / (
            f"frame_{index:06d}_native_initial_labels.png"
        )
        rag_path = args.rag_dir / f"frame_{index:06d}_native_rag_merged_labels.png"
        split = cv2.imread(str(split_path), cv2.IMREAD_UNCHANGED)
        edge = cv2.imread(str(edge_path), cv2.IMREAD_UNCHANGED)
        initial = cv2.imread(str(initial_path), cv2.IMREAD_UNCHANGED)
        rag = cv2.imread(str(rag_path), cv2.IMREAD_UNCHANGED)
        if split is None or edge is None or initial is None or rag is None:
            errors.append(f"input {index}: diagnostic label image missing")
            continue
        if not (split.ndim == edge.ndim == initial.ndim == rag.ndim == 2):
            errors.append(f"input {index}: diagnostic labels are not single-channel")
            continue
        if not (split.shape == edge.shape == initial.shape == rag.shape):
            errors.append(f"input {index}: diagnostic label shapes differ")
            continue

        split_positive = split > 0
        rag_positive = rag > 0
        if not np.array_equal(split_positive, rag_positive):
            errors.append(f"input {index}: RAG changed core support")
        rag_ids = np.unique(rag[rag_positive]).astype(np.int64)
        if not np.array_equal(rag_ids, np.arange(1, output_count + 1)):
            errors.append(f"input {index}: RAG IDs are not contiguous")

        # Every merged group may contain at most one split component from any
        # one initial region. Otherwise a true gradient split was undone.
        for rag_id in rag_ids:
            group = rag == rag_id
            for initial_id in np.unique(initial[group]):
                if initial_id <= 0:
                    continue
                split_ids = np.unique(split[group & (initial == initial_id)])
                split_ids = split_ids[split_ids > 0]
                if len(split_ids) > 1:
                    errors.append(
                        f"input {index}: output {rag_id} rejoined gradient split "
                        f"inside initial region {initial_id}"
                    )
                    break

        if args.repeat_rag_dir:
            repeat = cv2.imread(
                str(args.repeat_rag_dir / rag_path.name), cv2.IMREAD_UNCHANGED
            )
            if repeat is None or not np.array_equal(rag, repeat):
                errors.append(f"input {index}: repeated RAG output differs")
            else:
                deterministic_frames += 1

        if args.reference_dir and index not in allowed_missing:
            reference = cv2.imread(
                str(args.reference_dir / f"frame_{index:06d}_labels.png"),
                cv2.IMREAD_UNCHANGED,
            )
            if reference is None or reference.shape != rag.shape:
                errors.append(f"input {index}: author reference label missing/misaligned")
            else:
                overlap = rag_positive & (reference > 0)
                if np.any(overlap):
                    ari_values.append(adjusted_rand_index(rag[overlap], reference[overlap]))
                    nmi_values.append(
                        normalized_mutual_information(rag[overlap], reference[overlap])
                    )
                precision, recall = boundary_precision_recall(
                    rag, edge, reference, args.boundary_tolerance
                )
                if precision is not None:
                    boundary_precisions.append(precision)
                if recall is not None:
                    boundary_recalls.append(recall)
                reference_frames += 1

        input_counts.append(input_count)
        output_counts.append(output_count)
        region_reduction_ratios.append(
            1.0 - output_count / input_count if input_count else 0.0
        )
        high_merges.append(high_merge)
        low_merges.append(low_merge)
        unmerged_low.append(int(row["native_rag_unmerged_low_regions"]))
        adjacent_pairs.append(int(row["native_rag_spatial_adjacent_pairs"]))
        fake_pairs.append(int(row["native_rag_shared_fake_edge_pairs"]))
        eligible_pairs.append(int(row["native_rag_eligible_pairs"]))
        rejected_pairs.append(int(row["native_rag_depth_rejected_pairs"]))
        for column, values in (
            ("native_rag_total_ms", total_times),
            ("native_rag_graph_ms", graph_times),
            ("native_rag_merge_ms", merge_times),
        ):
            value = float(row[column])
            if not math.isfinite(value) or value < 0:
                errors.append(f"input {index}: invalid runtime in {column}")
            values.append(value)

    summary = {
        "identity": "S1 gradient-only RAG region evidence; no dynamic decision",
        "frame_rows": len(rows),
        "reference_compared_frames": reference_frames,
        "deterministic_frames": deterministic_frames if args.repeat_rag_dir else None,
        "input_component_count_median": median(input_counts),
        "output_region_count_median": median(output_counts),
        "region_reduction_ratio_mean": mean(region_reduction_ratios),
        "high_middle_merges_mean": mean(high_merges),
        "low_merges_mean": mean(low_merges),
        "unmerged_low_regions_mean": mean(unmerged_low),
        "initial_spatial_adjacent_pairs_mean": mean(adjacent_pairs),
        "initial_shared_fake_edge_pairs_mean": mean(fake_pairs),
        "initial_depth_rejected_pairs_mean": mean(rejected_pairs),
        "initial_eligible_pairs_mean": mean(eligible_pairs),
        "rag_total_ms_mean": mean(total_times),
        "rag_graph_ms_mean": mean(graph_times),
        "rag_merge_ms_mean": mean(merge_times),
        "rag_vs_author_final_ari_mean": mean(ari_values),
        "rag_vs_author_final_nmi_mean": mean(nmi_values),
        "rag_boundary_precision_at_tolerance_mean": mean(boundary_precisions),
        "author_boundary_recall_at_tolerance_mean": mean(boundary_recalls),
        "boundary_tolerance_pixels": args.boundary_tolerance,
        "invariant_errors": errors,
        "pass": not errors,
        "interpretation_limit": (
            "Author final labels are descriptive only. This adaptation has no plane "
            "edge, dense flow, temporal dynamic state, dynamic mask, or SLAM filtering."
        ),
    }
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
