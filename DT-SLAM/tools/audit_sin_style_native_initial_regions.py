#!/usr/bin/env python3
"""Audit the S1 clean-room initial 3D partition against SIn final labels.

The comparison is descriptive and label-permutation invariant. The native
partition is only an initial K-means representation; it is not expected to
equal the author's post split/merge regions and has no dynamic state.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def choose2(values):
    values = values.astype(np.float64)
    return values * (values - 1.0) / 2.0


def adjusted_rand_index(first, second):
    first_ids, first_inverse = np.unique(first, return_inverse=True)
    second_ids, second_inverse = np.unique(second, return_inverse=True)
    contingency = np.zeros((len(first_ids), len(second_ids)), dtype=np.int64)
    np.add.at(contingency, (first_inverse, second_inverse), 1)
    total = int(contingency.sum())
    if total < 2:
        return None
    pair_total = total * (total - 1.0) / 2.0
    sum_cells = float(choose2(contingency).sum())
    sum_rows = float(choose2(contingency.sum(axis=1)).sum())
    sum_cols = float(choose2(contingency.sum(axis=0)).sum())
    expected = sum_rows * sum_cols / pair_total
    maximum = 0.5 * (sum_rows + sum_cols)
    denominator = maximum - expected
    if abs(denominator) < 1e-12:
        return 1.0
    return (sum_cells - expected) / denominator


def normalized_mutual_information(first, second):
    first_ids, first_inverse = np.unique(first, return_inverse=True)
    second_ids, second_inverse = np.unique(second, return_inverse=True)
    contingency = np.zeros((len(first_ids), len(second_ids)), dtype=np.float64)
    np.add.at(contingency, (first_inverse, second_inverse), 1.0)
    total = contingency.sum()
    if total <= 0.0:
        return None
    joint = contingency / total
    first_probability = joint.sum(axis=1)
    second_probability = joint.sum(axis=0)
    nonzero = joint > 0.0
    expected = first_probability[:, None] * second_probability[None, :]
    mutual_information = float(
        np.sum(joint[nonzero] * np.log(joint[nonzero] / expected[nonzero]))
    )
    first_entropy = float(-np.sum(first_probability * np.log(first_probability)))
    second_entropy = float(-np.sum(second_probability * np.log(second_probability)))
    denominator = math.sqrt(first_entropy * second_entropy)
    if denominator <= 1e-12:
        return 1.0
    return mutual_information / denominator


def internal_boundaries(labels):
    positive = labels > 0
    boundary = np.zeros(labels.shape, dtype=np.uint8)
    horizontal = (
        positive[:, :-1]
        & positive[:, 1:]
        & (labels[:, :-1] != labels[:, 1:])
    )
    vertical = (
        positive[:-1, :]
        & positive[1:, :]
        & (labels[:-1, :] != labels[1:, :])
    )
    boundary[:, :-1][horizontal] = 255
    boundary[:, 1:][horizontal] = 255
    boundary[:-1, :][vertical] = 255
    boundary[1:, :][vertical] = 255
    return boundary


def boundary_precision_recall(first, second, tolerance):
    first_boundary = internal_boundaries(first)
    second_boundary = internal_boundaries(second)
    kernel_size = 2 * tolerance + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    first_dilated = cv2.dilate(first_boundary, kernel)
    second_dilated = cv2.dilate(second_boundary, kernel)
    first_count = int(np.count_nonzero(first_boundary))
    second_count = int(np.count_nonzero(second_boundary))
    precision = (
        float(np.count_nonzero((first_boundary > 0) & (second_dilated > 0)))
        / first_count
        if first_count
        else None
    )
    recall = (
        float(np.count_nonzero((second_boundary > 0) & (first_dilated > 0)))
        / second_count
        if second_count
        else None
    )
    return first_count, second_count, precision, recall


def finite_mean(values):
    return float(np.mean(values)) if values else None


def finite_median(values):
    return float(np.median(values)) if values else None


def split_ints(value):
    return [int(item) for item in value.split(";") if item]


def split_floats(value):
    return [float(item) for item in value.split(";") if item]


def split_level_shapes(value):
    shapes = []
    for item in value.split(";"):
        if not item:
            continue
        level_text, size_text = item.split(":", 1)
        width_text, height_text = size_text.split("x", 1)
        shapes.append((int(level_text), int(width_text), int(height_text)))
    return shapes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--native-dir", required=True, type=Path)
    parser.add_argument("--reference-dir", required=True, type=Path)
    parser.add_argument("--expected-rows", required=True, type=int)
    parser.add_argument("--allow-missing-reference-index", action="append", default=[], type=int)
    parser.add_argument("--boundary-tolerance", default=2, type=int)
    parser.add_argument("--require-coarse-to-fine", action="store_true")
    parser.add_argument("--temporal-commit-start-index", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    errors = []
    if len(rows) != args.expected_rows:
        errors.append(f"row count {len(rows)} != expected {args.expected_rows}")
    allowed_missing_reference = set(args.allow_missing_reference_index)

    input_indices = [int(row["input_index"]) for row in rows]
    if input_indices != list(range(len(rows))):
        errors.append("input indices are not unique and continuous from zero")
    reset_epochs = [int(row["reset_epoch"]) for row in rows]
    if any(current < previous for previous, current in zip(reset_epochs, reset_epochs[1:])):
        errors.append("reset epochs are not monotonically non-decreasing")

    native_coverages = []
    native_region_counts = []
    reference_region_counts = []
    ari_values = []
    nmi_values = []
    boundary_precisions = []
    boundary_recalls = []
    native_total_ms = []
    native_kmeans_ms = []
    native_orb_coverages = []
    initialization_sources = {}
    previous_prior_coverages = []
    compared_frames = 0

    previous_reset_epoch = None
    for row in rows:
        input_index = int(row["input_index"])
        reset_epoch = int(row["reset_epoch"])
        first_in_reset_epoch = (
            previous_reset_epoch is None or reset_epoch != previous_reset_epoch
        )
        previous_reset_epoch = reset_epoch
        if int(row["native_initial_enabled"]) != 1:
            errors.append(f"input {input_index}: native initial path is disabled")
        if int(row["native_initial_available"]) != 1:
            errors.append(f"input {input_index}: native initial partition unavailable")
        if int(row["native_dynamic_state_available"]) != 0:
            errors.append(f"input {input_index}: initial regions expose dynamic state")
        if row["native_dynamic_decision"] != "none":
            errors.append(f"input {input_index}: native dynamic decision is not none")
        if row["direct_slam_state_mutation"] != "none" or int(row["actual_slam_removed"]) != 0:
            errors.append(f"input {input_index}: shadow-only invariant failed")
        if int(row["native_initial_labels_written"]) != 1:
            errors.append(f"input {input_index}: native label write not confirmed")

        image_pixels = int(row["native_image_pixels"])
        clustering_valid = int(row["native_clustering_depth_valid_pixels"])
        input_valid = int(row["native_input_depth_valid_pixels"])
        excluded_far = int(row["native_excluded_far_depth_pixels"])
        if clustering_valid + excluded_far != input_valid:
            errors.append(f"input {input_index}: native depth accounting failed")
        if clustering_valid > image_pixels or input_valid > image_pixels:
            errors.append(f"input {input_index}: native depth count exceeds image")
        requested = int(row["native_requested_clusters"])
        produced = int(row["native_produced_clusters"])
        if requested != produced or produced <= 0:
            errors.append(f"input {input_index}: native cluster-count invariant failed")

        coarse_to_fine = int(row.get("native_coarse_to_fine", "0")) != 0
        if args.require_coarse_to_fine and not coarse_to_fine:
            errors.append(f"input {input_index}: coarse-to-fine path is disabled")
        initialization_source = row.get(
            "native_initialization_source", "from_scratch"
        )
        initialization_sources[initialization_source] = (
            initialization_sources.get(initialization_source, 0) + 1
        )
        if coarse_to_fine:
            level_count = int(row["native_pyramid_levels"])
            level_valid_samples = split_ints(row["native_level_valid_samples"])
            level_prior_samples = split_ints(row["native_level_prior_samples"])
            level_fallback_samples = split_ints(
                row["native_level_grid_fallback_samples"]
            )
            level_shapes = split_level_shapes(row["native_level_shapes"])
            level_compactness = split_floats(row["native_level_compactness"])
            level_prepare_ms = split_floats(row["native_level_prepare_ms"])
            level_kmeans_ms = split_floats(row["native_level_kmeans_ms"])
            level_label_ms = split_floats(row["native_level_label_ms"])
            if not (
                len(level_shapes)
                == len(level_valid_samples)
                == len(level_prior_samples)
                == len(level_fallback_samples)
                == len(level_compactness)
                == len(level_prepare_ms)
                == len(level_kmeans_ms)
                == len(level_label_ms)
                == level_count
            ):
                errors.append(f"input {input_index}: pyramid-array length mismatch")
            for level_position, (valid, prior, fallback) in enumerate(
                zip(level_valid_samples, level_prior_samples, level_fallback_samples)
            ):
                if prior + fallback != valid:
                    errors.append(
                        f"input {input_index}: level {level_position} "
                        "initialization accounting failed"
                    )
            expected_levels = list(range(level_count - 1, -1, -1))
            observed_levels = [level for level, _, _ in level_shapes]
            if observed_levels != expected_levels:
                errors.append(f"input {input_index}: pyramid level order mismatch")
            for (_, previous_width, previous_height), (_, width, height) in zip(
                level_shapes, level_shapes[1:]
            ):
                if width < previous_width or height < previous_height:
                    errors.append(
                        f"input {input_index}: pyramid shapes are not coarse-to-fine"
                    )
            if level_valid_samples and level_valid_samples[-1] != clustering_valid:
                errors.append(
                    f"input {input_index}: finest-level valid count mismatch"
                )
            for name, values in (
                ("compactness", level_compactness),
                ("prepare runtime", level_prepare_ms),
                ("kmeans runtime", level_kmeans_ms),
                ("label runtime", level_label_ms),
            ):
                if any(not math.isfinite(value) or value < 0.0 for value in values):
                    errors.append(
                        f"input {input_index}: invalid per-level {name}"
                    )
            previous_samples = int(row["native_previous_prior_samples"])
            fallback_samples = int(row["native_grid_fallback_samples"])
            if level_valid_samples and (
                previous_samples + fallback_samples != level_valid_samples[0]
            ):
                errors.append(
                    f"input {input_index}: coarsest initialization accounting failed"
                )
            prior_coverage = float(row["native_previous_prior_coverage"])
            expected_coverage = (
                previous_samples / (previous_samples + fallback_samples)
                if previous_samples + fallback_samples
                else 0.0
            )
            if abs(prior_coverage - expected_coverage) > 1e-9:
                errors.append(f"input {input_index}: prior coverage formula failed")
            previous_prior_coverages.append(prior_coverage)
            if args.temporal_commit_start_index is not None:
                start = args.temporal_commit_start_index
                committed = int(row["native_temporal_prior_committed"]) != 0
                if committed != (input_index >= start):
                    errors.append(
                        f"input {input_index}: temporal commit state mismatch"
                    )
                if (input_index <= start or first_in_reset_epoch) and initialization_source != "grid":
                    errors.append(
                        f"input {input_index}: author-aligned initial source is not grid"
                    )
                if input_index > start and not first_in_reset_epoch and initialization_source not in (
                    "previous",
                    "mixed",
                ):
                    errors.append(
                        f"input {input_index}: continuous temporal prior was not used"
                    )

        native_path = args.native_dir / (
            f"frame_{input_index:06d}_native_initial_labels.png"
        )
        native = cv2.imread(str(native_path), cv2.IMREAD_UNCHANGED)
        if native is None:
            errors.append(f"input {input_index}: missing native labels")
            continue
        if native.ndim != 2:
            errors.append(f"input {input_index}: native labels are not single-channel")
            continue
        if coarse_to_fine and level_shapes:
            _, finest_width, finest_height = level_shapes[-1]
            if (finest_height, finest_width) != native.shape:
                errors.append(f"input {input_index}: finest-level shape mismatch")
        native_positive = native > 0
        if int(np.count_nonzero(native_positive)) != clustering_valid:
            errors.append(f"input {input_index}: native valid-pixel mismatch")
        native_ids = np.unique(native[native_positive])
        if len(native_ids) != produced:
            errors.append(f"input {input_index}: native image region-count mismatch")
        if not np.array_equal(
            native_ids.astype(np.int64), np.arange(1, produced + 1, dtype=np.int64)
        ):
            errors.append(f"input {input_index}: native label IDs are not contiguous")
        native_areas = np.bincount(
            native[native_positive].astype(np.int64), minlength=produced + 1
        )[1:]
        if native_areas.size:
            if int(native_areas.sum()) != clustering_valid:
                errors.append(f"input {input_index}: native area sum mismatch")
            if int(native_areas.min()) != int(row["native_smallest_region_pixels"]):
                errors.append(f"input {input_index}: smallest native area mismatch")
            if int(native_areas.max()) != int(row["native_largest_region_pixels"]):
                errors.append(f"input {input_index}: largest native area mismatch")

        native_coverages.append(clustering_valid / image_pixels)
        native_region_counts.append(produced)
        native_total_ms.append(float(row["native_total_ms"]))
        native_kmeans_ms.append(float(row["native_kmeans_ms"]))
        raw_orb = int(row["raw_orb_count"])
        assigned_orb = int(row["native_initial_orb_assigned_count"])
        if assigned_orb > raw_orb:
            errors.append(f"input {input_index}: native ORB assignment exceeds raw ORBs")
        native_orb_coverages.append(assigned_orb / raw_orb if raw_orb else 0.0)

        if input_index in allowed_missing_reference:
            continue
        reference_path = args.reference_dir / f"frame_{input_index:06d}_labels.png"
        reference = cv2.imread(str(reference_path), cv2.IMREAD_UNCHANGED)
        if reference is None:
            errors.append(f"input {input_index}: missing reference labels")
            continue
        if reference.shape != native.shape:
            errors.append(f"input {input_index}: native/reference shape mismatch")
            continue
        reference_positive = reference > 0
        overlap = native_positive & reference_positive
        if not np.any(overlap):
            errors.append(f"input {input_index}: no positive-label overlap")
            continue
        reference_region_counts.append(len(np.unique(reference[reference_positive])))
        ari_values.append(adjusted_rand_index(native[overlap], reference[overlap]))
        nmi_values.append(normalized_mutual_information(native[overlap], reference[overlap]))
        _, _, precision, recall = boundary_precision_recall(
            native, reference, args.boundary_tolerance
        )
        if precision is not None:
            boundary_precisions.append(precision)
        if recall is not None:
            boundary_recalls.append(recall)
        compared_frames += 1

    for key, values in (
        ("native_total_ms", native_total_ms),
        ("native_kmeans_ms", native_kmeans_ms),
    ):
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            errors.append(f"invalid runtime values in {key}")

    summary = {
        "identity": "S1 clean-room initial 3D partition; no dynamic decision",
        "frame_rows": len(rows),
        "reference_compared_frames": compared_frames,
        "native_region_count_median": finite_median(native_region_counts),
        "reference_final_region_count_median": finite_median(reference_region_counts),
        "native_valid_coverage_mean": finite_mean(native_coverages),
        "native_orb_assignment_coverage_mean": finite_mean(native_orb_coverages),
        "initialization_source_frames": initialization_sources,
        "previous_prior_coverage_mean": finite_mean(previous_prior_coverages),
        "label_permutation_invariant_ari_mean": finite_mean(ari_values),
        "label_permutation_invariant_nmi_mean": finite_mean(nmi_values),
        "native_boundary_precision_at_tolerance_mean": finite_mean(boundary_precisions),
        "reference_boundary_recall_at_tolerance_mean": finite_mean(boundary_recalls),
        "boundary_tolerance_pixels": args.boundary_tolerance,
        "native_kmeans_ms_mean": finite_mean(native_kmeans_ms),
        "native_total_ms_mean": finite_mean(native_total_ms),
        "invariant_errors": errors,
        "pass": not errors,
        "interpretation_limit": (
            "Similarity to reference final labels is descriptive only; the native "
            "path has not implemented edge split, RAG merge, flow, or dynamics."
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
