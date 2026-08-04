#!/usr/bin/env python3
"""Audit DT-SLAM S1 dense-flow replay rows against the author export manifest."""

import argparse
import csv
import json
import math
from pathlib import Path


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_int(row, key):
    return int(row[key])


def as_float(row, key):
    return float(row[key])


def close(left, right, tolerance=1e-9):
    scale = max(1.0, abs(left), abs(right))
    return (math.isfinite(left) and math.isfinite(right) and
            abs(left-right) <= tolerance*scale)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tracking_csv")
    parser.add_argument("export_manifest")
    parser.add_argument("output_json")
    args = parser.parse_args()

    tracking = read_rows(args.tracking_csv)
    exported = read_rows(args.export_manifest)
    errors = []
    if not tracking:
        errors.append("tracking CSV has no rows")
    export_by_frame = {as_int(row, "frame_index"): row for row in exported}

    if tracking:
        first = tracking[0]
        if as_int(first, "input_index") != 0:
            errors.append("first tracking input_index is not zero")
        if as_int(first, "dense_flow_available") != 0:
            errors.append("frame zero unexpectedly has dense-flow evidence")
        if first["dense_flow_failure_reason"] != "history_unavailable":
            errors.append("frame zero lacks history_unavailable state")

    matched = 0
    for row in tracking:
        frame = as_int(row, "input_index")
        if as_int(row, "dense_flow_enabled") != 1:
            errors.append(f"frame {frame}: replay disabled")
        if as_int(row, "dense_flow_dynamic_state_available") != 0:
            errors.append(f"frame {frame}: dynamic state unexpectedly available")
        if row["dense_flow_dynamic_decision"] != "none":
            errors.append(f"frame {frame}: dynamic decision is not none")
        if row["dense_flow_direct_slam_state_mutation"] != "none":
            errors.append(f"frame {frame}: dense replay mutated SLAM")
        if row["direct_slam_state_mutation"] != "none" or as_int(
                row, "actual_slam_removed") != 0:
            errors.append(f"frame {frame}: shadow invariant failed")
        if as_int(row, "dense_flow_high_pixels") > as_int(
                row, "dense_flow_low_pixels"):
            errors.append(f"frame {frame}: high support exceeds low support")
        if frame == 0:
            continue
        source = export_by_frame.get(frame)
        if source is None:
            errors.append(f"frame {frame}: missing export manifest row")
            continue
        matched += 1
        integer_pairs = (
            ("dense_flow_intended_reference_lag", "intended_reference_lag"),
            ("dense_flow_reference_index", "reference_index"),
            ("dense_flow_actual_reference_lag", "actual_reference_lag"),
            ("dense_flow_large_motion", "large_motion"),
            ("dense_flow_homography_samples", "homography_sample_count"),
            ("dense_flow_low_pixels", "low_pixels"),
            ("dense_flow_high_pixels", "high_pixels"),
        )
        for replay_key, export_key in integer_pairs:
            if as_int(row, replay_key) != as_int(source, export_key):
                errors.append(
                    f"frame {frame}: {replay_key} differs from export")
        float_pairs = (
            ("dense_flow_image_scale", "image_scale"),
            ("dense_flow_max_flow_px", "max_flow_px"),
            ("dense_flow_max_residual_px", "max_residual_px"),
            ("dense_flow_low_threshold_u8", "low_threshold_u8"),
            ("dense_flow_high_threshold_u8", "high_threshold_u8"),
            ("dense_flow_low_threshold_px", "low_threshold_px"),
            ("dense_flow_high_threshold_px", "high_threshold_px"),
        )
        for replay_key, export_key in float_pairs:
            # The author manifest uses the stream default precision (about six
            # significant digits); YAML and DT-SLAM CSV retain more digits.
            if not close(as_float(row, replay_key), as_float(source, export_key), 5e-6):
                errors.append(
                    f"frame {frame}: {replay_key} differs from export")

    expected_matches = max(0, len(tracking)-1)
    if matched != expected_matches:
        errors.append(
            f"matched {matched} export rows, expected {expected_matches}")
    if len(exported) != expected_matches:
        errors.append(
            f"export manifest has {len(exported)} rows, expected {expected_matches}")

    summary = {
        "tracking_csv": str(Path(args.tracking_csv).resolve()),
        "export_manifest": str(Path(args.export_manifest).resolve()),
        "tracking_rows": len(tracking),
        "export_rows": len(exported),
        "matched_evidence_rows": matched,
        "invariant_error_count": len(errors),
        "errors": errors,
        "passed": not errors,
    }
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
