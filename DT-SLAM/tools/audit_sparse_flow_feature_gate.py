#!/usr/bin/env python3
"""Audit G2-4F2 sparse-flow quality and static-risk curves.

The tool consumes existing G2-4F1 feature CSV files. It does not emit a
dynamic/static decision and does not modify SLAM state. Forward-backward error
is used only as correspondence quality; motion inconsistency remains the
ego-flow residual.
"""

import argparse
import csv
import json
import math
import pathlib
import statistics
import tempfile


REQUIRED_COLUMNS = {
    "frame",
    "feature_index",
    "has_mappoint",
    "semantic_nonzero",
    "backward_lk_status",
    "forward_lk_status",
    "forward_backward_error_px",
    "reference_depth_valid",
    "slam_ego_projection_valid",
    "slam_residual_magnitude_px",
    "evidence_state",
}

OPTIONAL_GT_COLUMNS = {
    "gt_pose_available",
    "gt_ego_projection_valid",
    "gt_residual_magnitude_px",
}

OPTIONAL_DEPTH_RISK_COLUMNS = {
    "reference_depth_boundary_d1",
    "reference_depth_boundary_d2",
    "reference_invalid_depth_d1",
    "reference_invalid_depth_d2",
}

DEFAULT_FB_THRESHOLDS = (0.25, 0.5, 1.0, 2.0)
DEFAULT_Q_THRESHOLDS = (2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)
DEFAULT_RAW_THRESHOLDS = (0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0)


def parse_named_path(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected NAME=PATH")
    name, path = value.split("=", 1)
    if not name:
        raise argparse.ArgumentTypeError("dataset NAME cannot be empty")
    return name, pathlib.Path(path)


def finite_float(value, field_name):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "invalid float for {}: {!r}".format(field_name, value)
        ) from error
    if not math.isfinite(number):
        raise ValueError(
            "non-finite float for {}: {!r}".format(field_name, value)
        )
    return number


def binary_flag(value, field_name):
    if value not in ("0", "1"):
        raise ValueError(
            "{} must be 0 or 1, got {!r}".format(field_name, value)
        )
    return value == "1"


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return (
        ordered[lower] * (1.0 - fraction)
        + ordered[upper] * fraction
    )


def distribution(values):
    materialized = [float(value) for value in values]
    return {
        "n": len(materialized),
        "median": percentile(materialized, 0.5),
        "p90": percentile(materialized, 0.9),
        "p95": percentile(materialized, 0.95),
        "p99": percentile(materialized, 0.99),
        "min": min(materialized) if materialized else None,
        "max": max(materialized) if materialized else None,
        "mean": (
            statistics.fmean(materialized) if materialized else None
        ),
    }


def read_feature_csv(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("{} has no CSV header".format(path))
        missing = sorted(REQUIRED_COLUMNS - set(reader.fieldnames))
        if missing:
            raise ValueError(
                "{} missing columns: {}".format(
                    path, ", ".join(missing)
                )
            )
        has_gt_columns = OPTIONAL_GT_COLUMNS.issubset(reader.fieldnames)
        has_depth_risk_columns = OPTIONAL_DEPTH_RISK_COLUMNS.issubset(
            reader.fieldnames
        )
        raw_rows = list(reader)
        if any(None in row for row in raw_rows):
            raise ValueError("{} has rows wider than header".format(path))

    frames = {}
    seen_feature_keys = set()
    for row_number, row in enumerate(raw_rows, start=2):
        try:
            frame = int(row["frame"])
            feature_index = int(row["feature_index"])
        except ValueError as error:
            raise ValueError(
                "{}:{} invalid frame/feature index".format(
                    path, row_number
                )
            ) from error
        key = (frame, feature_index)
        if key in seen_feature_keys:
            raise ValueError(
                "{}:{} duplicate frame/feature {}".format(
                    path, row_number, key
                )
            )
        seen_feature_keys.add(key)

        parsed = {
            "frame": frame,
            "feature_index": feature_index,
            "has_mappoint": binary_flag(
                row["has_mappoint"], "has_mappoint"
            ),
            "semantic_nonzero": binary_flag(
                row["semantic_nonzero"], "semantic_nonzero"
            ),
            "backward_lk_status": binary_flag(
                row["backward_lk_status"], "backward_lk_status"
            ),
            "forward_lk_status": binary_flag(
                row["forward_lk_status"], "forward_lk_status"
            ),
            "reference_depth_valid": binary_flag(
                row["reference_depth_valid"], "reference_depth_valid"
            ),
            "slam_ego_projection_valid": binary_flag(
                row["slam_ego_projection_valid"],
                "slam_ego_projection_valid",
            ),
            "evidence_state": row["evidence_state"],
            "depth_risk_available": has_depth_risk_columns,
        }
        if has_depth_risk_columns:
            for field_name in sorted(OPTIONAL_DEPTH_RISK_COLUMNS):
                parsed[field_name] = binary_flag(
                    row[field_name], field_name
                )
        if parsed["evidence_state"] == "measured":
            parsed["fb_error_px"] = finite_float(
                row["forward_backward_error_px"],
                "forward_backward_error_px",
            )
            parsed["residual_px"] = finite_float(
                row["slam_residual_magnitude_px"],
                "slam_residual_magnitude_px",
            )
            if parsed["fb_error_px"] < 0.0 or parsed["residual_px"] < 0.0:
                raise ValueError(
                    "{}:{} errors must be non-negative".format(
                        path, row_number
                    )
                )
            required_validity = (
                parsed["backward_lk_status"]
                and parsed["forward_lk_status"]
                and parsed["reference_depth_valid"]
                and parsed["slam_ego_projection_valid"]
            )
            if not required_validity:
                raise ValueError(
                    "{}:{} measured row violates validity contract".format(
                        path, row_number
                    )
                )
            if has_gt_columns:
                parsed["gt_pose_available"] = binary_flag(
                    row["gt_pose_available"], "gt_pose_available"
                )
                parsed["gt_ego_projection_valid"] = binary_flag(
                    row["gt_ego_projection_valid"],
                    "gt_ego_projection_valid",
                )
                parsed["gt_residual_px"] = (
                    finite_float(
                        row["gt_residual_magnitude_px"],
                        "gt_residual_magnitude_px",
                    )
                    if parsed["gt_pose_available"]
                    and parsed["gt_ego_projection_valid"]
                    else None
                )
                if (
                    parsed["gt_residual_px"] is not None
                    and parsed["gt_residual_px"] < 0.0
                ):
                    raise ValueError(
                        "{}:{} GT residual must be non-negative".format(
                            path, row_number
                        )
                    )
            else:
                parsed["gt_pose_available"] = False
                parsed["gt_ego_projection_valid"] = False
                parsed["gt_residual_px"] = None
        else:
            parsed["fb_error_px"] = None
            parsed["residual_px"] = None
            parsed["gt_pose_available"] = False
            parsed["gt_ego_projection_valid"] = False
            parsed["gt_residual_px"] = None
        frames.setdefault(frame, []).append(parsed)
    return frames


def quality_eligible(
    row, fb_threshold, residual_field="residual_px"
):
    return (
        row["evidence_state"] == "measured"
        and not row["semantic_nonzero"]
        and row["fb_error_px"] <= fb_threshold
        and row.get(residual_field) is not None
    )


def summarize_working_point(
    frames,
    fb_threshold,
    q_threshold,
    scale_floor,
    minimum_scale_support,
    residual_field="residual_px",
):
    total_nonsemantic = 0
    total_eligible = 0
    total_candidates = 0
    total_mappoint_eligible = 0
    total_mappoint_candidates = 0
    valid_scale_frames = 0
    candidate_frames = 0
    frame_scales = []
    candidate_counts = []
    retained_counts = []
    mappoint_candidate_counts = []
    mappoint_retained_counts = []

    for rows in frames.values():
        nonsemantic = [
            row for row in rows if not row["semantic_nonzero"]
        ]
        eligible = [
            row
            for row in rows
            if quality_eligible(
                row, fb_threshold, residual_field
            )
        ]
        total_nonsemantic += len(nonsemantic)
        total_eligible += len(eligible)
        total_mappoint_eligible += sum(
            row["has_mappoint"] for row in eligible
        )

        candidate_count = 0
        mappoint_candidate_count = 0
        if len(eligible) >= minimum_scale_support:
            valid_scale_frames += 1
            scale = max(
                scale_floor,
                1.4826
                * statistics.median(
                    row[residual_field] for row in eligible
                ),
            )
            frame_scales.append(scale)
            for row in eligible:
                normalized_residual = row[residual_field] / scale
                if normalized_residual >= q_threshold:
                    candidate_count += 1
                    if row["has_mappoint"]:
                        mappoint_candidate_count += 1

        total_candidates += candidate_count
        total_mappoint_candidates += mappoint_candidate_count
        if candidate_count:
            candidate_frames += 1
        candidate_counts.append(candidate_count)
        retained_counts.append(len(eligible) - candidate_count)
        mappoint_candidate_counts.append(mappoint_candidate_count)
        mappoint_retained_counts.append(
            sum(row["has_mappoint"] for row in eligible)
            - mappoint_candidate_count
        )

    return {
        "fb_threshold_px": fb_threshold,
        "normalized_residual_threshold": q_threshold,
        "frames": len(frames),
        "valid_scale_frames": valid_scale_frames,
        "candidate_frames": candidate_frames,
        "candidate_frame_fraction": (
            candidate_frames / len(frames) if frames else None
        ),
        "nonsemantic_features": total_nonsemantic,
        "quality_eligible_features": total_eligible,
        "quality_eligible_fraction_of_nonsemantic": (
            total_eligible / total_nonsemantic
            if total_nonsemantic
            else None
        ),
        "candidate_features": total_candidates,
        "candidate_fraction_of_quality_eligible": (
            total_candidates / total_eligible
            if total_eligible
            else None
        ),
        "mappoint_quality_eligible": total_mappoint_eligible,
        "mappoint_candidates": total_mappoint_candidates,
        "mappoint_candidate_fraction": (
            total_mappoint_candidates / total_mappoint_eligible
            if total_mappoint_eligible
            else None
        ),
        "frame_scale_px": distribution(frame_scales),
        "candidate_count_per_frame": distribution(candidate_counts),
        "retained_quality_features_per_frame": distribution(
            retained_counts
        ),
        "mappoint_candidate_count_per_frame": distribution(
            mappoint_candidate_counts
        ),
        "mappoint_retained_per_frame": distribution(
            mappoint_retained_counts
        ),
    }


def summarize_depth_risk_strata(
    frames,
    fb_threshold,
    q_threshold,
    scale_floor,
    minimum_scale_support,
    residual_field="residual_px",
):
    band_predicates = {
        "boundary_d1": (
            lambda row: row["reference_depth_boundary_d1"]
        ),
        "boundary_d2": (
            lambda row: row["reference_depth_boundary_d2"]
        ),
        "invalid_depth_d1": (
            lambda row: row["reference_invalid_depth_d1"]
        ),
        "invalid_depth_d2": (
            lambda row: row["reference_invalid_depth_d2"]
        ),
        "clean_d2": (
            lambda row:
            not row["reference_depth_boundary_d2"]
            and not row["reference_invalid_depth_d2"]
        ),
    }
    totals = {
        name: {
            "quality_eligible_features": 0,
            "candidate_features": 0,
            "mappoint_quality_eligible": 0,
            "mappoint_candidates": 0,
        }
        for name in band_predicates
    }
    all_candidates = 0
    all_mappoint_candidates = 0
    valid_scale_frames = 0
    for rows in frames.values():
        eligible = [
            row
            for row in rows
            if quality_eligible(row, fb_threshold, residual_field)
        ]
        if len(eligible) < minimum_scale_support:
            continue
        valid_scale_frames += 1
        scale = max(
            scale_floor,
            1.4826
            * statistics.median(
                row[residual_field] for row in eligible
            ),
        )
        for row in eligible:
            is_candidate = row[residual_field] / scale >= q_threshold
            if is_candidate:
                all_candidates += 1
                if row["has_mappoint"]:
                    all_mappoint_candidates += 1
            for name, predicate in band_predicates.items():
                if not predicate(row):
                    continue
                totals[name]["quality_eligible_features"] += 1
                if row["has_mappoint"]:
                    totals[name]["mappoint_quality_eligible"] += 1
                if is_candidate:
                    totals[name]["candidate_features"] += 1
                    if row["has_mappoint"]:
                        totals[name]["mappoint_candidates"] += 1

    for values in totals.values():
        eligible = values["quality_eligible_features"]
        candidates = values["candidate_features"]
        mappoint_eligible = values["mappoint_quality_eligible"]
        mappoint_candidates = values["mappoint_candidates"]
        values["candidate_fraction_of_band"] = (
            candidates / eligible if eligible else None
        )
        values["candidate_share_of_all_candidates"] = (
            candidates / all_candidates if all_candidates else None
        )
        values["mappoint_candidate_fraction_of_band"] = (
            mappoint_candidates / mappoint_eligible
            if mappoint_eligible
            else None
        )
        values["mappoint_candidate_share_of_all_candidates"] = (
            mappoint_candidates / all_mappoint_candidates
            if all_mappoint_candidates
            else None
        )
    return {
        "fb_threshold_px": fb_threshold,
        "normalized_residual_threshold": q_threshold,
        "valid_scale_frames": valid_scale_frames,
        "all_candidates": all_candidates,
        "all_mappoint_candidates": all_mappoint_candidates,
        "bands": totals,
        "dynamic_decision": "none",
    }


def summarize_raw_threshold(
    frames,
    fb_threshold,
    residual_threshold,
    residual_field="residual_px",
):
    eligible_rows = []
    for rows in frames.values():
        eligible_rows.extend(
            row
            for row in rows
            if quality_eligible(
                row, fb_threshold, residual_field
            )
        )
    candidates = [
        row
        for row in eligible_rows
        if row[residual_field] >= residual_threshold
    ]
    return {
        "fb_threshold_px": fb_threshold,
        "raw_residual_threshold_px": residual_threshold,
        "quality_eligible_features": len(eligible_rows),
        "candidate_features": len(candidates),
        "candidate_fraction_of_quality_eligible": (
            len(candidates) / len(eligible_rows)
            if eligible_rows
            else None
        ),
    }


def summarize_continuous_static_weight(
    frames,
    fb_threshold,
    scale_floor,
    minimum_scale_support,
    student_t_nu=10.0,
    residual_field="residual_px",
):
    scales = []
    normalized_residuals = []
    static_weights = []
    valid_scale_frames = 0
    for rows in frames.values():
        eligible = [
            row
            for row in rows
            if quality_eligible(
                row, fb_threshold, residual_field
            )
        ]
        if len(eligible) < minimum_scale_support:
            continue
        valid_scale_frames += 1
        scale = max(
            scale_floor,
            1.4826
            * statistics.median(
                row[residual_field] for row in eligible
            ),
        )
        scales.append(scale)
        for row in eligible:
            normalized = row[residual_field] / scale
            weight = min(
                1.0,
                (student_t_nu + 1.0)
                / (student_t_nu + normalized * normalized),
            )
            normalized_residuals.append(normalized)
            static_weights.append(weight)
    return {
        "fb_threshold_px": fb_threshold,
        "student_t_nu_reference": student_t_nu,
        "valid_scale_frames": valid_scale_frames,
        "frame_scale_px": distribution(scales),
        "normalized_residual": distribution(normalized_residuals),
        "continuous_static_weight": distribution(static_weights),
        "dynamic_decision": "none",
    }


def summarize_dataset(
    frames,
    fb_thresholds,
    q_thresholds,
    raw_thresholds,
    scale_floor,
    minimum_scale_support,
):
    measured = [
        row
        for rows in frames.values()
        for row in rows
        if row["evidence_state"] == "measured"
        and not row["semantic_nonzero"]
    ]
    result = {
        "frames": len(frames),
        "nonsemantic_measured_features": len(measured),
        "measured_fb_error_px": distribution(
            row["fb_error_px"] for row in measured
        ),
        "measured_residual_px": distribution(
            row["residual_px"] for row in measured
        ),
        "continuous_static_weight_audit": [
            summarize_continuous_static_weight(
                frames,
                fb_threshold,
                scale_floor,
                minimum_scale_support,
            )
            for fb_threshold in fb_thresholds
        ],
        "normalized_working_points": [
            summarize_working_point(
                frames,
                fb_threshold,
                q_threshold,
                scale_floor,
                minimum_scale_support,
            )
            for fb_threshold in fb_thresholds
            for q_threshold in q_thresholds
        ],
        "raw_residual_audit": [
            summarize_raw_threshold(
                frames, fb_threshold, residual_threshold
            )
            for fb_threshold in fb_thresholds
            for residual_threshold in raw_thresholds
        ],
        "depth_risk_diagnostics_available": bool(
            measured and measured[0]["depth_risk_available"]
        ),
    }
    if result["depth_risk_diagnostics_available"]:
        if not all(row["depth_risk_available"] for row in measured):
            raise ValueError(
                "depth-risk columns must be available for every measured row"
            )
        result["depth_risk_stratified_working_points"] = [
            summarize_depth_risk_strata(
                frames,
                fb_threshold,
                q_threshold,
                scale_floor,
                minimum_scale_support,
            )
            for fb_threshold in fb_thresholds
            for q_threshold in q_thresholds
        ]
    gt_measured = [
        row
        for rows in frames.values()
        for row in rows
        if row.get("gt_residual_px") is not None
        and not row["semantic_nonzero"]
    ]
    if gt_measured:
        result["ground_truth_pose_diagnostic"] = {
            "nonsemantic_measured_features": len(gt_measured),
            "measured_residual_px": distribution(
                row["gt_residual_px"] for row in gt_measured
            ),
            "continuous_static_weight_audit": [
                summarize_continuous_static_weight(
                    frames,
                    fb_threshold,
                    scale_floor,
                    minimum_scale_support,
                    residual_field="gt_residual_px",
                )
                for fb_threshold in fb_thresholds
            ],
            "normalized_working_points": [
                summarize_working_point(
                    frames,
                    fb_threshold,
                    q_threshold,
                    scale_floor,
                    minimum_scale_support,
                    residual_field="gt_residual_px",
                )
                for fb_threshold in fb_thresholds
                for q_threshold in q_thresholds
            ],
            "raw_residual_audit": [
                summarize_raw_threshold(
                    frames,
                    fb_threshold,
                    residual_threshold,
                    residual_field="gt_residual_px",
                )
                for fb_threshold in fb_thresholds
                for residual_threshold in raw_thresholds
            ],
            "deployment_use": "forbidden",
        }
        if result["depth_risk_diagnostics_available"]:
            result["ground_truth_pose_diagnostic"][
                "depth_risk_stratified_working_points"
            ] = [
                summarize_depth_risk_strata(
                    frames,
                    fb_threshold,
                    q_threshold,
                    scale_floor,
                    minimum_scale_support,
                    residual_field="gt_residual_px",
                )
                for fb_threshold in fb_thresholds
                for q_threshold in q_thresholds
            ]
    return result


def audit(
    named_paths,
    fb_thresholds=DEFAULT_FB_THRESHOLDS,
    q_thresholds=DEFAULT_Q_THRESHOLDS,
    raw_thresholds=DEFAULT_RAW_THRESHOLDS,
    scale_floor=1.0e-3,
    minimum_scale_support=20,
):
    if not named_paths:
        raise ValueError("at least one input dataset is required")
    if scale_floor <= 0.0:
        raise ValueError("scale_floor must be positive")
    if minimum_scale_support <= 0:
        raise ValueError("minimum_scale_support must be positive")
    names = [name for name, _ in named_paths]
    if len(set(names)) != len(names):
        raise ValueError("dataset names must be unique")

    datasets = {}
    for name, path in named_paths:
        frames = read_feature_csv(path)
        datasets[name] = {
            "path": str(path),
            "summary": summarize_dataset(
                frames,
                fb_thresholds,
                q_thresholds,
                raw_thresholds,
                scale_floor,
                minimum_scale_support,
            ),
        }
    return {
        "identity": "G2-4F2 quality and static-risk curve audit",
        "method_identity": (
            "[A/S/H] FlowFusion residual + Kalal FB quality + "
            "Li-Lee-inspired robust scale + "
            "SInDSLAM/DynaSLAM-inspired depth-boundary risk audit"
        ),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "depth_flow_fusion": "none",
        "depth_boundary_use": "risk_stratification_only",
        "g1_release": False,
        "scale_definition": (
            "max(scale_floor, 1.4826 * median(nonsemantic "
            "quality-eligible residual magnitude))"
        ),
        "student_t_nu_reference": 10,
        "student_t_note": (
            "Continuous weight is specified by the stage document; "
            "this audit evaluates threshold curves and does not classify."
        ),
        "parameters": {
            "fb_thresholds_px": list(fb_thresholds),
            "normalized_residual_thresholds": list(q_thresholds),
            "raw_residual_thresholds_px": list(raw_thresholds),
            "scale_floor_px": scale_floor,
            "minimum_scale_support": minimum_scale_support,
        },
        "datasets": datasets,
    }


def write_fixture(path):
    fieldnames = [
        "frame",
        "feature_index",
        "has_mappoint",
        "semantic_nonzero",
        "backward_lk_status",
        "forward_lk_status",
        "forward_backward_error_px",
        "reference_depth_valid",
        "slam_ego_projection_valid",
        "slam_residual_magnitude_px",
        "evidence_state",
        "reference_depth_boundary_d1",
        "reference_depth_boundary_d2",
        "reference_invalid_depth_d1",
        "reference_invalid_depth_d2",
    ]
    rows = []
    for frame in (1, 2):
        for index in range(25):
            residual = 0.0 if index < 20 else float(index - 19)
            rows.append(
                {
                    "frame": frame,
                    "feature_index": index,
                    "has_mappoint": "1" if index % 2 == 0 else "0",
                    "semantic_nonzero": "1" if index == 24 else "0",
                    "backward_lk_status": "1",
                    "forward_lk_status": "1",
                    "forward_backward_error_px": (
                        "0.1" if index != 23 else "3.0"
                    ),
                    "reference_depth_valid": "1",
                    "slam_ego_projection_valid": "1",
                    "slam_residual_magnitude_px": str(residual),
                    "evidence_state": "measured",
                    "reference_depth_boundary_d1": (
                        "1" if index == 20 else "0"
                    ),
                    "reference_depth_boundary_d2": (
                        "1" if index == 20 else "0"
                    ),
                    "reference_invalid_depth_d1": "0",
                    "reference_invalid_depth_d2": (
                        "1" if index == 21 else "0"
                    ),
                }
            )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def self_test():
    with tempfile.TemporaryDirectory(
        prefix="dtslam_g2_4f2_test_"
    ) as directory:
        fixture = pathlib.Path(directory) / "fixture.csv"
        write_fixture(fixture)
        result = audit(
            [("fixture", fixture)],
            fb_thresholds=(0.5,),
            q_thresholds=(2.0,),
            raw_thresholds=(1.0,),
            scale_floor=0.1,
            minimum_scale_support=20,
        )
        summary = result["datasets"]["fixture"]["summary"]
        working = summary["normalized_working_points"][0]
        assert working["valid_scale_frames"] == 2
        assert working["quality_eligible_features"] == 46
        assert working["candidate_features"] == 6
        assert working["mappoint_candidates"] == 4
        continuous = summary["continuous_static_weight_audit"][0]
        assert continuous["valid_scale_frames"] == 2
        assert (
            continuous["continuous_static_weight"]["n"]
            == working["quality_eligible_features"]
        )
        assert continuous["continuous_static_weight"]["min"] >= 0.0
        assert continuous["continuous_static_weight"]["max"] <= 1.0
        assert result["dynamic_decision"] == "none"
        assert result["direct_slam_state_mutation"] == "none"
        assert summary["depth_risk_diagnostics_available"]
        risk = summary["depth_risk_stratified_working_points"][0]
        assert risk["all_candidates"] == 6
        assert risk["bands"]["boundary_d1"]["candidate_features"] == 2
        assert risk["bands"]["invalid_depth_d2"]["candidate_features"] == 2
        assert risk["bands"]["clean_d2"]["candidate_features"] == 2
    print("audit_sparse_flow_feature_gate self-test: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        action="append",
        type=parse_named_path,
        default=[],
        help="repeatable NAME=G2-4F1_FEATURE_CSV",
    )
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument(
        "--scale-floor-px", type=float, default=1.0e-3
    )
    parser.add_argument(
        "--minimum-scale-support", type=int, default=20
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if not args.output:
        parser.error("--output is required unless --self-test is used")

    result = audit(
        args.input,
        scale_floor=args.scale_floor_px,
        minimum_scale_support=args.minimum_scale_support,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(
        "wrote {} datasets to {}; dynamic_decision=none "
        "direct_slam_state_mutation=none".format(
            len(result["datasets"]), args.output
        )
    )


if __name__ == "__main__":
    main()
