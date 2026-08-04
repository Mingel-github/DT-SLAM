#!/usr/bin/env python3
"""Audit the S1 gradient-depth split without treating it as dynamics.

Author final labels are an optional descriptive partition reference, not
ground truth for this clean-room increment.
"""

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


def edge_to_reference_boundary_precision_recall(edge, reference, tolerance):
    reference_boundary = internal_boundaries(reference)
    kernel_size = 2 * tolerance + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    edge_binary = np.where(edge > 0, 255, 0).astype(np.uint8)
    edge_dilated = cv2.dilate(edge_binary, kernel)
    reference_dilated = cv2.dilate(reference_boundary, kernel)
    edge_count = int(np.count_nonzero(edge_binary))
    reference_count = int(np.count_nonzero(reference_boundary))
    precision = (
        float(np.count_nonzero((edge_binary > 0) & (reference_dilated > 0)))
        / edge_count
        if edge_count
        else None
    )
    recall = (
        float(np.count_nonzero((reference_boundary > 0) & (edge_dilated > 0)))
        / reference_count
        if reference_count
        else None
    )
    return precision, recall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--gradient-dir", required=True, type=Path)
    parser.add_argument("--native-initial-dir", required=True, type=Path)
    parser.add_argument("--reference-dir", type=Path)
    parser.add_argument("--repeat-gradient-dir", type=Path)
    parser.add_argument("--expected-rows", required=True, type=int)
    parser.add_argument("--allow-missing-reference-index", action="append", default=[], type=int)
    parser.add_argument("--small-component-pixels", default=80, type=int)
    parser.add_argument("--boundary-tolerance", default=2, type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    errors = []
    if len(rows) != args.expected_rows:
        errors.append(f"row count {len(rows)} != expected {args.expected_rows}")
    indices = [int(row["input_index"]) for row in rows]
    if indices != list(range(len(rows))):
        errors.append("input indices are not unique and continuous from zero")

    allowed_missing = set(args.allow_missing_reference_index)
    edge_ratios = []
    core_coverages = []
    split_counts = []
    split_initial_counts = []
    fragmentation_medians = []
    fragmentation_maxima = []
    total_times = []
    median_times = []
    edge_times = []
    component_times = []
    ari_values = []
    nmi_values = []
    boundary_precisions = []
    boundary_recalls = []
    deterministic_frames = 0
    reference_frames = 0

    for row in rows:
        index = int(row["input_index"])
        if int(row["native_gradient_enabled"]) != 1:
            errors.append(f"input {index}: gradient split disabled")
        if int(row["native_gradient_available"]) != 1:
            errors.append(f"input {index}: gradient split unavailable")
        if int(row["native_gradient_dynamic_state_available"]) != 0:
            errors.append(f"input {index}: gradient split exposes dynamic state")
        if row["native_gradient_dynamic_decision"] != "none":
            errors.append(f"input {index}: gradient dynamic decision is not none")
        if row["direct_slam_state_mutation"] != "none" or int(row["actual_slam_removed"]) != 0:
            errors.append(f"input {index}: shadow-only invariant failed")
        if int(row["native_gradient_edge_written"]) != 1:
            errors.append(f"input {index}: gradient edge write not confirmed")
        if int(row["native_gradient_split_labels_written"]) != 1:
            errors.append(f"input {index}: gradient labels write not confirmed")

        image_pixels = int(row["native_gradient_image_pixels"])
        initial_pixels = int(row["native_gradient_initial_region_pixels"])
        median_valid = int(row["native_gradient_median_valid_pixels"])
        insufficient = int(row["native_gradient_insufficient_support_pixels"])
        raw_edge = int(row["native_gradient_raw_edge_pixels"])
        boundary = int(row["native_gradient_split_boundary_pixels"])
        core = int(row["native_gradient_split_core_pixels"])
        split_count = int(row["native_gradient_split_component_count"])
        if median_valid + insufficient != initial_pixels:
            errors.append(f"input {index}: median support accounting failed")
        if raw_edge != boundary:
            errors.append(f"input {index}: raw edge/boundary accounting failed")
        if boundary + core != median_valid:
            errors.append(f"input {index}: boundary/core accounting failed")
        if initial_pixels > image_pixels or median_valid > image_pixels:
            errors.append(f"input {index}: gradient pixel count exceeds image")

        edge_path = args.gradient_dir / f"frame_{index:06d}_native_gradient_edge.png"
        split_path = args.gradient_dir / f"frame_{index:06d}_native_gradient_split_labels.png"
        initial_path = args.native_initial_dir / f"frame_{index:06d}_native_initial_labels.png"
        edge_image = cv2.imread(str(edge_path), cv2.IMREAD_UNCHANGED)
        split_image = cv2.imread(str(split_path), cv2.IMREAD_UNCHANGED)
        initial_image = cv2.imread(str(initial_path), cv2.IMREAD_UNCHANGED)
        if edge_image is None or split_image is None or initial_image is None:
            errors.append(f"input {index}: one or more diagnostic images are missing")
            continue
        if not (edge_image.ndim == split_image.ndim == initial_image.ndim == 2):
            errors.append(f"input {index}: diagnostics are not single-channel")
            continue
        if not (edge_image.shape == split_image.shape == initial_image.shape):
            errors.append(f"input {index}: diagnostic shapes differ")
            continue
        if edge_image.size != image_pixels:
            errors.append(f"input {index}: image-pixel count mismatch")

        initial_positive = initial_image > 0
        split_positive = split_image > 0
        if int(np.count_nonzero(initial_positive)) != initial_pixels:
            errors.append(f"input {index}: initial positive-pixel mismatch")
        if len(np.unique(initial_image[initial_positive])) != int(
            row["native_gradient_initial_region_count"]
        ):
            errors.append(f"input {index}: initial region-count mismatch")
        if int(np.count_nonzero(edge_image)) != raw_edge:
            errors.append(f"input {index}: edge-image count mismatch")
        if int(np.count_nonzero(split_positive)) != core:
            errors.append(f"input {index}: split-image core count mismatch")
        edge_positive = edge_image > 0
        if np.any(edge_positive & split_positive):
            errors.append(f"input {index}: edge and split core overlap")
        if np.any(edge_positive & ~initial_positive):
            errors.append(f"input {index}: edge escapes initial positive domain")
        if np.any(split_positive & ~initial_positive):
            errors.append(f"input {index}: split core escapes initial positive domain")
        if int(np.count_nonzero(edge_positive | split_positive)) != median_valid:
            errors.append(f"input {index}: edge/core pixel union mismatch")
        split_ids = np.unique(split_image[split_positive]).astype(np.int64)
        if not np.array_equal(split_ids, np.arange(1, split_count + 1)):
            errors.append(f"input {index}: split label IDs are not contiguous")

        component_areas = np.bincount(
            split_image[split_positive].astype(np.int64), minlength=split_count + 1
        )[1:]
        small_areas = component_areas[component_areas < args.small_component_pixels]
        if len(small_areas) != int(row["native_gradient_small_component_count"]):
            errors.append(f"input {index}: small-component count mismatch")
        if int(small_areas.sum()) != int(row["native_gradient_small_component_pixels"]):
            errors.append(f"input {index}: small-component pixels mismatch")

        fragment_counts = []
        fully_consumed = 0
        split_initial = 0
        for initial_id in np.unique(initial_image[initial_positive]):
            member_split_ids = np.unique(split_image[initial_image == initial_id])
            member_split_ids = member_split_ids[member_split_ids > 0]
            count = len(member_split_ids)
            if count == 0:
                fully_consumed += 1
            else:
                fragment_counts.append(count)
            if count > 1:
                split_initial += 1
        for split_id in split_ids:
            parent_ids = np.unique(initial_image[split_image == split_id])
            parent_ids = parent_ids[parent_ids > 0]
            if len(parent_ids) != 1:
                errors.append(f"input {index}: split component crosses initial regions")
                break
        expected_fragmentation_median = float(np.median(fragment_counts)) if fragment_counts else 0.0
        expected_fragmentation_maximum = max(fragment_counts) if fragment_counts else 0
        if split_initial != int(row["native_gradient_split_initial_region_count"]):
            errors.append(f"input {index}: split-initial-region count mismatch")
        if fully_consumed != int(row["native_gradient_fully_consumed_initial_region_count"]):
            errors.append(f"input {index}: fully-consumed region count mismatch")
        if abs(expected_fragmentation_median - float(row["native_gradient_median_fragmentation"])) > 1e-9:
            errors.append(f"input {index}: median fragmentation mismatch")
        if expected_fragmentation_maximum != int(row["native_gradient_maximum_fragmentation"]):
            errors.append(f"input {index}: maximum fragmentation mismatch")

        if args.repeat_gradient_dir:
            repeat_edge = cv2.imread(
                str(args.repeat_gradient_dir / edge_path.name), cv2.IMREAD_UNCHANGED
            )
            repeat_split = cv2.imread(
                str(args.repeat_gradient_dir / split_path.name), cv2.IMREAD_UNCHANGED
            )
            if repeat_edge is None or repeat_split is None or not (
                np.array_equal(edge_image, repeat_edge)
                and np.array_equal(split_image, repeat_split)
            ):
                errors.append(f"input {index}: repeated gradient output differs")
            else:
                deterministic_frames += 1

        if args.reference_dir and index not in allowed_missing:
            reference = cv2.imread(
                str(args.reference_dir / f"frame_{index:06d}_labels.png"),
                cv2.IMREAD_UNCHANGED,
            )
            if reference is None:
                errors.append(f"input {index}: reference labels missing")
            elif reference.shape != split_image.shape:
                errors.append(f"input {index}: reference label shape mismatch")
            else:
                overlap = split_positive & (reference > 0)
                if np.any(overlap):
                    ari_values.append(adjusted_rand_index(split_image[overlap], reference[overlap]))
                    nmi_values.append(normalized_mutual_information(split_image[overlap], reference[overlap]))
                precision, recall = edge_to_reference_boundary_precision_recall(
                    edge_image, reference, args.boundary_tolerance
                )
                if precision is not None:
                    boundary_precisions.append(precision)
                if recall is not None:
                    boundary_recalls.append(recall)
                reference_frames += 1

        edge_ratios.append(raw_edge / median_valid if median_valid else 0.0)
        core_coverages.append(core / image_pixels if image_pixels else 0.0)
        split_counts.append(split_count)
        split_initial_counts.append(split_initial)
        fragmentation_medians.append(expected_fragmentation_median)
        fragmentation_maxima.append(expected_fragmentation_maximum)
        for column, values in (
            ("native_gradient_total_ms", total_times),
            ("native_gradient_median_filter_ms", median_times),
            ("native_gradient_edge_ms", edge_times),
            ("native_gradient_components_ms", component_times),
        ):
            value = float(row[column])
            if not math.isfinite(value) or value < 0.0:
                errors.append(f"input {index}: invalid runtime in {column}")
            values.append(value)

    summary = {
        "identity": "S1 gradient-depth split shadow evidence; no dynamic decision",
        "frame_rows": len(rows),
        "reference_compared_frames": reference_frames,
        "deterministic_frames": deterministic_frames if args.repeat_gradient_dir else None,
        "edge_ratio_on_median_valid_mean": mean(edge_ratios),
        "split_core_image_coverage_mean": mean(core_coverages),
        "split_component_count_median": median(split_counts),
        "split_initial_region_count_mean": mean(split_initial_counts),
        "fragmentation_median_mean": mean(fragmentation_medians),
        "fragmentation_maximum_max": max(fragmentation_maxima) if fragmentation_maxima else None,
        "gradient_total_ms_mean": mean(total_times),
        "median_filter_ms_mean": mean(median_times),
        "gradient_edge_ms_mean": mean(edge_times),
        "connected_components_ms_mean": mean(component_times),
        "split_vs_reference_ari_mean": mean(ari_values),
        "split_vs_reference_nmi_mean": mean(nmi_values),
        "raw_edge_precision_at_tolerance_mean": mean(boundary_precisions),
        "reference_boundary_recall_at_tolerance_mean": mean(boundary_recalls),
        "boundary_tolerance_pixels": args.boundary_tolerance,
        "invariant_errors": errors,
        "pass": not errors,
        "interpretation_limit": (
            "Author final labels are descriptive only. This increment has no "
            "plane edge, RAG merge, dense flow, temporal dynamic state, or SLAM filtering."
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
