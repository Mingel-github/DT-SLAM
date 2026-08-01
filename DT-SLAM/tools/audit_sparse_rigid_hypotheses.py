#!/usr/bin/env python3
"""Audit G2-MH1 shadow CSV invariants and continuous distributions.

This tool deliberately does not choose a dynamic threshold or class.  Its
output describes measurement availability, SE(3) sanity, fit distributions,
and runtime only.
"""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


MEASURED = "measured"
VALIDATION_MEASURED = "measured"


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values):
    finite = [value for value in values if math.isfinite(value)]
    return {
        "count": len(finite),
        "median": median(finite) if finite else None,
        "p90": percentile(finite, 0.90),
        "p95": percentile(finite, 0.95),
        "maximum": max(finite) if finite else None,
    }


def determinant3(matrix):
    return (
        matrix[0][0] *
        (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] *
        (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] *
        (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def rotation_orthogonality_error(matrix):
    maximum = 0.0
    for row_a in range(3):
        for row_b in range(3):
            dot = sum(
                matrix[row_a][column] * matrix[row_b][column]
                for column in range(3)
            )
            target = 1.0 if row_a == row_b else 0.0
            maximum = max(maximum, abs(dot - target))
    return maximum


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("hypothesis_csv", type=Path)
    parser.add_argument(
        "--frame-csv",
        type=Path,
        help="Defaults to HYPOTHESIS_CSV.frames.csv",
    )
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    frame_path = args.frame_csv or Path(str(args.hypothesis_csv) + ".frames.csv")
    rows = load_csv(args.hypothesis_csv)
    frames = load_csv(frame_path)
    by_frame = defaultdict(list)
    for row in rows:
        by_frame[int(row["frame"])].append(row)

    violations = []
    state_counts = Counter(row["evidence_state"] for row in rows)
    measured = [row for row in rows if row["evidence_state"] == MEASURED]
    validation_state_counts = Counter(
        row["validation_state"] for row in rows
    )
    validated = [
        row for row in measured
        if row["validation_state"] == VALIDATION_MEASURED
    ]

    for row in rows:
        if row["dynamic_decision"] != "none":
            violations.append("hypothesis dynamic_decision is not none")
        if row["direct_slam_state_mutation"] != "none":
            violations.append("hypothesis direct_slam_state_mutation is not none")

    determinant_errors = []
    orthogonality_errors = []
    for row in measured:
        member_count = int(row["member_count"])
        members = [
            value for value in row["member_feature_indices"].split(";")
            if value
        ]
        if member_count != len(members):
            violations.append("member_count does not match member list")
        if member_count != 7:
            violations.append("measured hypothesis does not contain 7 members")
        matrix = [
            [float(row[f"h{r}{c}"]) for c in range(3)]
            for r in range(3)
        ]
        determinant_errors.append(abs(determinant3(matrix) - 1.0))
        orthogonality_errors.append(rotation_orthogonality_error(matrix))
        for field in (
            "local_fit_median_m",
            "local_fit_rms_m",
            "local_fit_p90_m",
            "background_fit_median_m",
            "background_fit_rms_m",
            "background_fit_p90_m",
            "median_improvement_m",
            "background_to_local_rms_ratio",
            "relative_translation_m",
            "relative_rotation_rad",
            "maximum_image_radius_px",
            "reference_depth_span_m",
            "current_depth_span_m",
            "reference_second_to_first_singular_ratio",
        ):
            if not math.isfinite(float(row[field])):
                violations.append(f"non-finite measured value: {field}")

        validation_members = [
            value for value in row["validation_feature_indices"].split(";")
            if value
        ]
        validation_count = int(row["validation_count"])
        if validation_count != len(validation_members):
            violations.append(
                "validation_count does not match validation member list"
            )
        if row["validation_state"] == VALIDATION_MEASURED:
            if validation_count != 7:
                violations.append(
                    "measured validation does not contain 7 points"
                )
            for field in (
                "validation_local_fit_median_m",
                "validation_local_fit_rms_m",
                "validation_local_fit_p90_m",
                "validation_background_fit_median_m",
                "validation_background_fit_rms_m",
                "validation_background_fit_p90_m",
                "validation_median_improvement_m",
                "validation_background_to_local_rms_ratio",
                "validation_local_better_fraction",
                "global_local_better_fraction",
                "global_median_improvement_m",
            ):
                if not math.isfinite(float(row[field])):
                    violations.append(
                        f"non-finite validated value: {field}"
                    )
            global_count = int(row["global_validation_count"])
            global_better = int(row["global_local_better_count"])
            if global_count < validation_count:
                violations.append(
                    "global validation count is smaller than local holdout"
                )
            if not 0 <= global_better <= global_count:
                violations.append("invalid global local-better count")

    for frame in frames:
        if frame["dynamic_decision"] != "none":
            violations.append("frame dynamic_decision is not none")
        if frame["direct_slam_state_mutation"] != "none":
            violations.append("frame direct_slam_state_mutation is not none")
        frame_id = int(frame["frame"])
        # The C++ frame-id filter intentionally records per-hypothesis rows
        # for selected review frames while retaining timing rows for every
        # computed frame.  Unselected frames therefore have no row-level
        # data to cross-check.
        if frame_id not in by_frame:
            continue
        frame_rows = by_frame.get(frame_id, [])
        if int(frame["input_nodes"]) != len(frame_rows):
            violations.append(f"frame {frame_id}: input_nodes does not match rows")
        if int(frame["valid_hypotheses"]) != sum(
            row["evidence_state"] == MEASURED for row in frame_rows
        ):
            violations.append(
                f"frame {frame_id}: valid_hypotheses does not match rows"
            )
        if int(frame["valid_validations"]) != sum(
            row["validation_state"] == VALIDATION_MEASURED
            for row in frame_rows
        ):
            violations.append(
                f"frame {frame_id}: valid_validations does not match rows"
            )

    metric_fields = (
        "local_fit_median_m",
        "local_fit_rms_m",
        "background_fit_median_m",
        "background_fit_rms_m",
        "median_improvement_m",
        "background_to_local_rms_ratio",
        "relative_translation_m",
        "relative_rotation_rad",
        "maximum_image_radius_px",
        "reference_depth_span_m",
        "current_depth_span_m",
        "reference_second_to_first_singular_ratio",
    )
    validation_metric_fields = (
        "validation_local_fit_median_m",
        "validation_local_fit_rms_m",
        "validation_local_fit_p90_m",
        "validation_background_fit_median_m",
        "validation_background_fit_rms_m",
        "validation_background_fit_p90_m",
        "validation_median_improvement_m",
        "validation_background_to_local_rms_ratio",
        "validation_local_better_fraction",
        "global_validation_count",
        "global_local_better_count",
        "global_local_better_fraction",
        "global_median_improvement_m",
    )
    report = {
        "method": "G2-MH1 sparse local 3-D rigid hypotheses",
        "scope": "shadow-only continuous diagnostics; no dynamic class",
        "hypothesis_csv": str(args.hypothesis_csv),
        "frame_csv": str(frame_path),
        "row_count": len(rows),
        "frame_count": len(frames),
        "state_counts": dict(sorted(state_counts.items())),
        "validation_state_counts": dict(
            sorted(validation_state_counts.items())
        ),
        "measured_fraction": len(measured) / len(rows) if rows else 0.0,
        "validated_fraction_of_measured": (
            len(validated) / len(measured) if measured else 0.0
        ),
        "se3_sanity": {
            "determinant_absolute_error": distribution(determinant_errors),
            "orthogonality_max_absolute_error": distribution(
                orthogonality_errors
            ),
        },
        "measured_distributions": {
            field: distribution([float(row[field]) for row in measured])
            for field in metric_fields
        },
        "independent_validation_distributions": {
            field: distribution([float(row[field]) for row in validated])
            for field in validation_metric_fields
        },
        "frame_timing_ms": {
            field: distribution([float(row[field]) for row in frames])
            for field in (
                "neighbor_search_ms",
                "fit_ms",
                "support_evaluation_ms",
                "total_ms",
            )
        },
        "invariant_violation_count": len(violations),
        "invariant_violations": violations[:100],
        "dynamic_threshold_selected": False,
        "direct_slam_state_mutation": False,
    }

    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 1 if violations else 0


if __name__ == "__main__":
    raise SystemExit(main())
