#!/usr/bin/env python3
"""Audit S1 reference-replay CSV against independently exported SIn masks."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np


def as_int(row, key):
    return int(row[key])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--reference-dir", required=True, type=Path)
    parser.add_argument("--expected-rows", required=True, type=int)
    parser.add_argument(
        "--expected-backend",
        required=True,
        choices=("deepflow_cpu", "brox_cuda"),
    )
    parser.add_argument(
        "--allow-missing-input-index",
        action="append",
        default=[],
        type=int,
    )
    parser.add_argument("--require-labels", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("shadow CSV is empty")

    errors = []
    input_indices = [as_int(row, "input_index") for row in rows]
    if len(rows) != args.expected_rows:
        errors.append(
            f"row count {len(rows)} does not match expected {args.expected_rows}"
        )
    if len(input_indices) != len(set(input_indices)):
        errors.append("duplicate input-index rows")
    if input_indices != list(range(len(rows))):
        errors.append("input-index rows are not contiguous from zero")
    reset_epochs = [as_int(row, "reset_epoch") for row in rows]
    if any(current < previous for previous, current in zip(reset_epochs, reset_epochs[1:])):
        errors.append("reset epochs are not monotonic")

    allowed_missing = set(args.allow_missing_input_index)
    unexpected_allowed = allowed_missing.difference(input_indices)
    if unexpected_allowed:
        errors.append(f"allowed missing indices outside CSV: {sorted(unexpected_allowed)}")

    reference_frames = 0
    fallback_frames = 0
    total_author_hits = 0
    total_depth_supported_hits = 0
    total_counterfactual_removed = 0
    total_actual_removed = 0
    raw_count_mismatches = 0

    for row in rows:
        input_index = as_int(row, "input_index")
        frame = as_int(row, "frame")
        pixels = as_int(row, "pixels")
        raw_unknown = as_int(row, "raw_unknown_pixels")
        raw_static = as_int(row, "raw_static_pixels")
        raw_dynamic = as_int(row, "raw_dynamic_pixels")
        valid = as_int(row, "valid_pixels")
        static = as_int(row, "static_pixels")
        dynamic = as_int(row, "dynamic_pixels")
        unknown = as_int(row, "unknown_pixels")

        if raw_unknown + raw_static + raw_dynamic != pixels:
            errors.append(f"input {input_index}: raw tri-state conservation failed")
        if static + dynamic != valid or valid + unknown != pixels:
            errors.append(f"input {input_index}: project tri-state failed")
        if dynamic > valid:
            errors.append(f"input {input_index}: dynamic is not a valid subset")
        if as_int(row, "depth_valid_pixels") > pixels:
            errors.append(f"input {input_index}: depth-valid count exceeds image")
        if row["dynamic_decision"] != "shadow_only":
            errors.append(f"input {input_index}: dynamic_decision is not shadow_only")
        if row["direct_slam_state_mutation"] != "none":
            errors.append(f"input {input_index}: SLAM mutation is not none")
        if as_int(row, "actual_slam_removed") != 0:
            errors.append(f"input {input_index}: shadow removed SLAM observations")

        if row["backend"] != args.expected_backend:
            errors.append(
                f"input {input_index}: backend {row['backend']} != "
                f"{args.expected_backend}"
            )
        for key in ("load_ms", "state_conversion_ms", "region_statistics_ms", "total_ms"):
            value = float(row[key])
            if not math.isfinite(value) or value < 0.0:
                errors.append(f"input {input_index}: invalid {key}={value}")

        available = as_int(row, "reference_available") != 0
        should_be_available = input_index not in allowed_missing
        if available != should_be_available:
            errors.append(
                f"input {input_index}: reference availability {int(available)} "
                f"does not match expected {int(should_be_available)}"
            )
        labels_available = as_int(row, "labels_available") != 0
        if args.require_labels and labels_available != available:
            errors.append(
                f"input {input_index}: labels availability does not match reference"
            )
        if available:
            reference_frames += 1
            mask_path = args.reference_dir / f"frame_{input_index:06d}_mask_final.png"
            mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
            if mask is None:
                errors.append(f"input {input_index}: reference marked available but missing")
            elif mask.ndim != 2:
                errors.append(f"input {input_index}: reference is not single channel")
            else:
                values = set(int(value) for value in np.unique(mask))
                if not values.issubset({0, 125, 255}):
                    errors.append(f"input {input_index}: unsupported reference values {values}")
                expected = (
                    int(np.count_nonzero(mask == 0)),
                    int(np.count_nonzero(mask == 125)),
                    int(np.count_nonzero(mask == 255)),
                )
                observed = (raw_unknown, raw_static, raw_dynamic)
                if expected != observed:
                    raw_count_mismatches += 1
                    errors.append(
                        f"input {input_index}: raw count mismatch {observed} != {expected}"
                    )
            if args.require_labels:
                label_path = args.reference_dir / f"frame_{input_index:06d}_labels.png"
                labels = cv2.imread(str(label_path), cv2.IMREAD_UNCHANGED)
                if labels is None:
                    errors.append(f"input {input_index}: required labels are missing")
                else:
                    positive = labels > 0
                    if int(np.count_nonzero(positive)) != as_int(
                        row, "positive_label_pixels"
                    ):
                        errors.append(
                            f"input {input_index}: positive-label pixel mismatch"
                        )
                    positive_ids = np.unique(labels[positive])
                    if len(positive_ids) != as_int(row, "positive_label_count"):
                        errors.append(
                            f"input {input_index}: positive-label count mismatch"
                        )
                    dynamic_on_positive = int(np.count_nonzero((mask == 255) & positive))
                    dynamic_on_zero = int(np.count_nonzero((mask == 255) & (labels == 0)))
                    if dynamic_on_positive != as_int(
                        row, "author_dynamic_pixels_on_positive_labels"
                    ):
                        errors.append(
                            f"input {input_index}: dynamic-on-positive mismatch"
                        )
                    if dynamic_on_zero != as_int(
                        row, "author_dynamic_pixels_on_label_zero"
                    ):
                        errors.append(
                            f"input {input_index}: dynamic-on-label-zero mismatch"
                        )
                    if dynamic_on_positive + dynamic_on_zero != raw_dynamic:
                        errors.append(
                            f"input {input_index}: label dynamic conservation failed"
                        )
        elif raw_unknown != pixels or raw_static != 0 or raw_dynamic != 0:
            errors.append(f"input {input_index}: unavailable reference is not all unknown")

        raw_orb = as_int(row, "raw_orb_count")
        author_hits = as_int(row, "author_dynamic_mask_hit_on_dt_orb_set")
        depth_hits = as_int(row, "depth_supported_dynamic_orb_count")
        would_keep = as_int(row, "would_keep_orb_count")
        fallback = as_int(row, "counterfactual_fallback_on_dt_orb_set")
        counterfactual_removed = as_int(row, "counterfactual_removed_on_dt_orb_set")
        if author_hits > raw_orb or depth_hits > author_hits:
            errors.append(f"input {input_index}: ORB mask subset invariant failed")
        if would_keep != raw_orb - author_hits:
            errors.append(f"input {input_index}: would-keep ORB formula failed")
        expected_fallback = int(would_keep < 250)
        if fallback != expected_fallback:
            errors.append(f"input {input_index}: fallback formula failed")
        expected_removed = 0 if fallback else author_hits
        if counterfactual_removed != expected_removed:
            errors.append(f"input {input_index}: counterfactual removal formula failed")
        if as_int(row, "valid_orb_count") + as_int(row, "unknown_orb_count") != raw_orb:
            errors.append(f"input {input_index}: ORB validity conservation failed")

        fallback_frames += fallback
        total_author_hits += author_hits
        total_depth_supported_hits += depth_hits
        total_counterfactual_removed += counterfactual_removed
        total_actual_removed += as_int(row, "actual_slam_removed")

    summary = {
        "csv": str(args.csv),
        "reference_directory": str(args.reference_dir),
        "frame_rows": len(rows),
        "first_input_index": input_indices[0],
        "last_input_index": input_indices[-1],
        "reference_available_frames": reference_frames,
        "expected_reference_available_frames": len(rows) - len(allowed_missing),
        "counterfactual_fallback_on_dt_orb_set_frames": fallback_frames,
        "author_dynamic_mask_hits_on_dt_orb_set": total_author_hits,
        "depth_supported_dynamic_orb_hits": total_depth_supported_hits,
        "counterfactual_removed_on_dt_orb_set": total_counterfactual_removed,
        "actual_slam_removed": total_actual_removed,
        "raw_count_mismatches": raw_count_mismatches,
        "invariant_errors": errors,
        "pass": not errors,
    }

    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
