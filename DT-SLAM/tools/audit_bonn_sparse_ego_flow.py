#!/usr/bin/env python3
"""Audit continuous G2-4F1 sparse ego-flow evidence on Bonn review frames.

This tool never chooses a motion threshold and never emits a dynamic/static
feature decision. RGB-temporal labels are development proxies, not ground
truth. Exact exported person masks are used only for stratification.
"""

import argparse
import csv
import json
import math
import pathlib
import statistics
import tempfile

import cv2


SEQUENCE_EXPORTS = {
    "nonobstructing": "nonobstructing_cpp_person_export",
    "obstructing": "obstructing_cpp_person_export",
    "balloon": "balloon_f1d_exact_semantic_export",
    "balloon2": "balloon2_f1d_exact_semantic_export",
}


def parse_named_path(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected NAME=PATH")
    name, path = value.split("=", 1)
    if name not in SEQUENCE_EXPORTS:
        raise argparse.ArgumentTypeError(
            "NAME must be one of {}".format(",".join(SEQUENCE_EXPORTS))
        )
    return name, pathlib.Path(path)


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if not reader.fieldnames:
            raise ValueError("{} has no CSV header".format(path))
        if any(None in row for row in rows):
            raise ValueError("{} has rows wider than its header".format(path))
    return rows


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
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def finite_float(value):
    if value in (None, ""):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def distribution(values):
    valid = [float(value) for value in values if value is not None]
    return {
        "n": len(valid),
        "median": percentile(valid, 0.5),
        "p90": percentile(valid, 0.9),
        "p95": percentile(valid, 0.95),
        "mean": statistics.fmean(valid) if valid else None,
    }


def rank_auc(positive_values, negative_values):
    positive = [
        float(value) for value in positive_values if value is not None
    ]
    negative = [
        float(value) for value in negative_values if value is not None
    ]
    if not positive or not negative:
        return None
    wins = 0.0
    for left in positive:
        for right in negative:
            if left > right:
                wins += 1.0
            elif left == right:
                wins += 0.5
    return wins / (len(positive) * len(negative))


def proxy_comparison(frame_results, positive_stratum, negative_stratum):
    positive = [
        frame
        for frame in frame_results
        if frame["stratum"] == positive_stratum
    ]
    negative = [
        frame
        for frame in frame_results
        if frame["stratum"] == negative_stratum
    ]

    def metric(frame, name):
        if name == "slam_inside_median_px":
            return frame["inside_bbox_person_excluded"][
                "slam_residual_px"
            ]["median"]
        if name == "gt_inside_median_px":
            return frame["inside_bbox_person_excluded"][
                "gt_residual_px"
            ]["median"]
        if name == "slam_inside_over_outside":
            return frame["inside_over_outside_slam_median"]
        if name == "slam_inside_minus_outside_px":
            return frame["inside_minus_outside_slam_median_px"]
        raise KeyError(name)

    metrics = {}
    for name in (
        "slam_inside_median_px",
        "gt_inside_median_px",
        "slam_inside_over_outside",
        "slam_inside_minus_outside_px",
    ):
        positive_values = [
            metric(frame, name) for frame in positive
        ]
        negative_values = [
            metric(frame, name) for frame in negative
        ]
        metrics[name] = {
            "positive": distribution(positive_values),
            "negative": distribution(negative_values),
            "rank_auc": rank_auc(positive_values, negative_values),
        }
    return {
        "positive_stratum": positive_stratum,
        "negative_stratum": negative_stratum,
        "positive_frames": len(positive),
        "negative_frames": len(negative),
        "proxy_only_not_ground_truth": True,
        "metrics": metrics,
    }


def paired_inside_outside_summary(frames):
    return {
        "frames": len(frames),
        "sequences": sorted(set(frame["sequence"] for frame in frames)),
        "inside_measured_frames": sum(
            frame["inside_bbox_person_excluded"]["slam_residual_px"][
                "median"
            ] is not None
            for frame in frames
        ),
        "frame_balanced_inside_slam_median_px": distribution(
            frame["inside_bbox_person_excluded"]["slam_residual_px"][
                "median"
            ]
            for frame in frames
        ),
        "frame_balanced_outside_slam_median_px": distribution(
            frame["outside_bbox_person_excluded"]["slam_residual_px"][
                "median"
            ]
            for frame in frames
        ),
        "frame_balanced_inside_minus_outside_slam_median_px":
            distribution(
                frame["inside_minus_outside_slam_median_px"]
                for frame in frames
            ),
        "frame_balanced_inside_over_outside_slam_median":
            distribution(
                frame["inside_over_outside_slam_median"]
                for frame in frames
            ),
        "frame_balanced_inside_gt_median_px": distribution(
            frame["inside_bbox_person_excluded"]["gt_residual_px"][
                "median"
            ]
            for frame in frames
        ),
    }


def summarize_subset(rows):
    measured = [
        row for row in rows if row["evidence_state"] == "measured"
    ]
    paired_ground_truth = [
        row
        for row in measured
        if row["gt_ego_projection_valid"] == "1"
    ]
    return {
        "features": len(rows),
        "measured": len(measured),
        "measured_coverage": (
            len(measured) / len(rows) if rows else None
        ),
        "slam_residual_px": distribution(
            finite_float(row["slam_residual_magnitude_px"])
            for row in measured
        ),
        "gt_residual_px": distribution(
            finite_float(row["gt_residual_magnitude_px"])
            for row in paired_ground_truth
        ),
        "forward_backward_error_px": distribution(
            finite_float(row["forward_backward_error_px"])
            for row in measured
        ),
        "mappoint_fraction": (
            sum(row["has_mappoint"] == "1" for row in rows) / len(rows)
            if rows
            else None
        ),
    }


def frame_stratum(motion_label, person_present):
    if motion_label == "not_visible":
        return "target_not_visible"
    return "{}_person_{}".format(
        motion_label, "present" if person_present else "absent"
    )


def bbox_contains(metadata, u, v):
    values = [
        metadata.get("bbox_x", ""),
        metadata.get("bbox_y", ""),
        metadata.get("bbox_width", ""),
        metadata.get("bbox_height", ""),
    ]
    if any(value == "" for value in values):
        return False
    x, y, width, height = (int(value) for value in values)
    return x <= u < x + width and y <= v < y + height


def audit_sequence(
    sequence,
    feature_path,
    motion_rows,
    coverage_by_key,
    person_mask_dir,
    online_semantic,
):
    feature_rows = read_csv(feature_path)
    frame_diagnostic_path = pathlib.Path(
        str(feature_path) + ".frames.csv"
    )
    frame_diagnostics = read_csv(frame_diagnostic_path)
    if any(
        row["dynamic_decision"] != "none"
        or row["direct_slam_state_mutation"] != "none"
        for row in frame_diagnostics
    ):
        raise ValueError(
            "{} frame diagnostics contain a decision or mutation".format(
                sequence
            )
        )
    for row in frame_diagnostics:
        if row["reference_available"] == "1" and (
            int(row["reference_frame"]) != int(row["frame"]) - 1
            or float(row["dt_seconds"]) <= 0.0
        ):
            raise ValueError(
                "{} has a non-adjacent or non-forward reference".format(
                    sequence
                )
            )
    runtime = {
        "computed_frames": len(frame_diagnostics),
        "active_total_ms": distribution(
            finite_float(row["active_total_ms"])
            for row in frame_diagnostics
        ),
        "compute_total_ms": distribution(
            finite_float(row["compute_total_ms"])
            for row in frame_diagnostics
        ),
        "backward_lk_ms": distribution(
            finite_float(row["backward_lk_ms"])
            for row in frame_diagnostics
        ),
        "forward_lk_ms": distribution(
            finite_float(row["forward_lk_ms"])
            for row in frame_diagnostics
        ),
        "depth_projection_ms": distribution(
            finite_float(row["depth_projection_ms"])
            for row in frame_diagnostics
        ),
        "feature_record_ms": distribution(
            finite_float(row["record_ms"])
            for row in frame_diagnostics
        ),
    }
    features_by_frame = {}
    for row in feature_rows:
        frame = int(row["frame"])
        features_by_frame.setdefault(frame, []).append(row)

    motion_by_frame = {
        int(row["source_frame"]): row for row in motion_rows
    }
    if set(features_by_frame) != set(motion_by_frame):
        raise ValueError(
            "{} feature/motion frame sets differ: {} vs {}".format(
                sequence,
                sorted(features_by_frame),
                sorted(motion_by_frame),
            )
        )

    export_name = SEQUENCE_EXPORTS[sequence]
    frame_results = []
    annotated_rows = []
    for frame in sorted(features_by_frame):
        key = (export_name, frame)
        if key not in coverage_by_key:
            raise KeyError("missing coverage metadata for {}".format(key))
        metadata = coverage_by_key[key]
        motion = motion_by_frame[frame]
        if motion.get("is_ground_truth", "").lower() != "false":
            raise ValueError("motion proxy must explicitly be non-GT")
        if motion.get("geometry_or_flow_seen", "").lower() != "false":
            raise ValueError("motion proxy reviewer saw geometry/flow")

        mask_path = person_mask_dir / "frame_{:06d}.png".format(frame)
        person_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if person_mask is None:
            raise FileNotFoundError(mask_path)
        if person_mask.shape != (480, 640):
            raise ValueError(
                "{} has unexpected shape {}".format(
                    mask_path, person_mask.shape
                )
            )
        person_present = int(metadata["person_mask_pixels"]) > 0
        if bool(cv2.countNonZero(person_mask)) != person_present:
            raise ValueError(
                "{} person-mask presence disagrees with coverage CSV".format(
                    key
                )
            )

        rows = features_by_frame[frame]
        for row in rows:
            u = float(row["u_current"])
            v = float(row["v_current"])
            iu = min(639, max(0, int(u)))
            iv = min(479, max(0, int(v)))
            row = dict(row)
            row["_inside_bbox"] = bbox_contains(metadata, u, v)
            row["_inside_person_mask"] = person_mask[iv, iu] != 0
            if online_semantic and (
                (row["semantic_nonzero"] == "1")
                != row["_inside_person_mask"]
            ):
                raise ValueError(
                    "{} frame {} feature {} online semantic flag "
                    "disagrees with the frozen exact C++ mask".format(
                        sequence, frame, row["feature_index"]
                    )
                )
            annotated_rows.append(
                (
                    frame_stratum(
                        motion["motion_label"], person_present
                    ),
                    row,
                )
            )

        inside = [row for row in rows if bbox_contains(
            metadata, float(row["u_current"]), float(row["v_current"])
        )]
        outside = [row for row in rows if row not in inside]
        inside_person_excluded = []
        for row in inside:
            iu = min(639, max(0, int(float(row["u_current"]))))
            iv = min(479, max(0, int(float(row["v_current"]))))
            if person_mask[iv, iu] == 0:
                inside_person_excluded.append(row)
        outside_person_excluded = []
        for row in outside:
            iu = min(639, max(0, int(float(row["u_current"]))))
            iv = min(479, max(0, int(float(row["v_current"]))))
            if person_mask[iv, iu] == 0:
                outside_person_excluded.append(row)

        inside_summary = summarize_subset(inside)
        outside_summary = summarize_subset(outside)
        inside_clean_summary = summarize_subset(inside_person_excluded)
        outside_clean_summary = summarize_subset(outside_person_excluded)
        inside_median = inside_clean_summary["slam_residual_px"]["median"]
        outside_median = outside_clean_summary["slam_residual_px"]["median"]
        frame_results.append(
            {
                "sequence": sequence,
                "frame": frame,
                "motion_label": motion["motion_label"],
                "motion_confidence": motion["confidence"],
                "motion_label_source": motion["label_source"],
                "is_ground_truth": motion["is_ground_truth"],
                "person_present": int(person_present),
                "target_person_mask_overlap_pixels": int(
                    metadata["person_mask_pixels_inside_bbox"]
                ),
                "target_semantic_exact_zero_overlap": int(
                    int(metadata["person_mask_pixels_inside_bbox"]) == 0
                ),
                "visibility": metadata["visibility"],
                "stratum": frame_stratum(
                    motion["motion_label"], person_present
                ),
                "all": summarize_subset(rows),
                "inside_bbox": inside_summary,
                "inside_bbox_person_excluded": inside_clean_summary,
                "outside_bbox": outside_summary,
                "outside_bbox_person_excluded": outside_clean_summary,
                "inside_minus_outside_slam_median_px": (
                    inside_median - outside_median
                    if inside_median is not None
                    and outside_median is not None
                    else None
                ),
                "inside_over_outside_slam_median": (
                    inside_median / outside_median
                    if inside_median is not None
                    and outside_median not in (None, 0.0)
                    else None
                ),
            }
        )
    return frame_results, annotated_rows, runtime


def aggregate_strata(frame_results, annotated_rows):
    frame_by_stratum = {}
    for frame in frame_results:
        frame_by_stratum.setdefault(frame["stratum"], []).append(frame)
    feature_by_stratum = {}
    for stratum, row in annotated_rows:
        feature_by_stratum.setdefault(stratum, []).append(row)

    result = {}
    for stratum in sorted(frame_by_stratum):
        frames = frame_by_stratum[stratum]
        features = feature_by_stratum[stratum]
        inside_clean = [
            row
            for row in features
            if row["_inside_bbox"] and not row["_inside_person_mask"]
        ]
        outside = [row for row in features if not row["_inside_bbox"]]
        outside_clean = [
            row
            for row in outside
            if not row["_inside_person_mask"]
        ]
        result[stratum] = {
            "frames": len(frames),
            "sequences": sorted(set(frame["sequence"] for frame in frames)),
            "all_features": summarize_subset(features),
            "inside_bbox_person_excluded": summarize_subset(inside_clean),
            "outside_bbox": summarize_subset(outside),
            "outside_bbox_person_excluded":
                summarize_subset(outside_clean),
            "frame_balanced_inside_slam_median_px": distribution(
                frame["inside_bbox_person_excluded"][
                    "slam_residual_px"
                ]["median"]
                for frame in frames
            ),
            "frame_balanced_inside_gt_median_px": distribution(
                frame["inside_bbox_person_excluded"][
                    "gt_residual_px"
                ]["median"]
                for frame in frames
            ),
            "frame_balanced_inside_minus_outside_slam_median_px":
                distribution(
                    frame["inside_minus_outside_slam_median_px"]
                    for frame in frames
                ),
            "frame_balanced_inside_over_outside_slam_median":
                distribution(
                    frame["inside_over_outside_slam_median"]
                    for frame in frames
                ),
        }
    return result


def flatten_frame_result(frame):
    result = {
        "sequence": frame["sequence"],
        "frame": frame["frame"],
        "motion_label": frame["motion_label"],
        "motion_confidence": frame["motion_confidence"],
        "motion_label_source": frame["motion_label_source"],
        "is_ground_truth": frame["is_ground_truth"],
        "person_present": frame["person_present"],
        "target_person_mask_overlap_pixels":
            frame["target_person_mask_overlap_pixels"],
        "target_semantic_exact_zero_overlap":
            frame["target_semantic_exact_zero_overlap"],
        "visibility": frame["visibility"],
        "stratum": frame["stratum"],
    }
    for subset in (
        "all",
        "inside_bbox",
        "inside_bbox_person_excluded",
        "outside_bbox",
        "outside_bbox_person_excluded",
    ):
        values = frame[subset]
        prefix = subset + "_"
        result[prefix + "features"] = values["features"]
        result[prefix + "measured"] = values["measured"]
        result[prefix + "measured_coverage"] = values[
            "measured_coverage"
        ]
        result[prefix + "slam_median_px"] = values[
            "slam_residual_px"
        ]["median"]
        result[prefix + "slam_p90_px"] = values[
            "slam_residual_px"
        ]["p90"]
        result[prefix + "gt_median_px"] = values[
            "gt_residual_px"
        ]["median"]
        result[prefix + "fb_median_px"] = values[
            "forward_backward_error_px"
        ]["median"]
    result["inside_minus_outside_slam_median_px"] = frame[
        "inside_minus_outside_slam_median_px"
    ]
    result["inside_over_outside_slam_median"] = frame[
        "inside_over_outside_slam_median"
    ]
    return result


def self_test():
    values = [1.0, 2.0, 3.0, 4.0]
    assert percentile(values, 0.5) == 2.5
    assert percentile(values, 0.9) == 3.7
    rows = [
        {
            "evidence_state": "measured",
            "slam_residual_magnitude_px": "2",
            "gt_ego_projection_valid": "1",
            "gt_residual_magnitude_px": "3",
            "forward_backward_error_px": "0.1",
            "has_mappoint": "1",
        },
        {
            "evidence_state": "depth_invalid",
            "slam_residual_magnitude_px": "",
            "gt_ego_projection_valid": "0",
            "gt_residual_magnitude_px": "",
            "forward_backward_error_px": "",
            "has_mappoint": "0",
        },
    ]
    summary = summarize_subset(rows)
    assert summary["measured"] == 1
    assert summary["measured_coverage"] == 0.5
    assert summary["slam_residual_px"]["median"] == 2.0
    assert rank_auc([2.0, 3.0], [1.0, 2.0]) == 0.875
    print("[G2-4F1 Sparse Flow Audit Self-Test] PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--feature", action="append", type=parse_named_path, default=[]
    )
    parser.add_argument(
        "--motion-proxy",
        action="append",
        type=parse_named_path,
        default=[],
    )
    parser.add_argument("--coverage", type=pathlib.Path)
    parser.add_argument(
        "--person-mask-dir",
        action="append",
        type=parse_named_path,
        default=[],
    )
    parser.add_argument("--output-dir", type=pathlib.Path)
    parser.add_argument("--online-semantic", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0

    feature_paths = dict(args.feature)
    motion_paths = dict(args.motion_proxy)
    mask_dirs = dict(args.person_mask_dir)
    required = set(feature_paths)
    if (
        not required
        or not required.issubset(SEQUENCE_EXPORTS)
        or set(motion_paths) != required
        or set(mask_dirs) != required
        or args.coverage is None
        or args.output_dir is None
    ):
        parser.error(
            "provide matching supported sequence names for feature, "
            "motion-proxy, and person-mask-dir, plus coverage and output-dir"
        )

    coverage_by_key = {}
    for row in read_csv(args.coverage):
        key = (row["export_name"], int(row["source_frame"]))
        if key in coverage_by_key:
            raise ValueError("duplicate coverage key {}".format(key))
        coverage_by_key[key] = row

    all_frame_results = []
    all_annotated_rows = []
    runtime_by_sequence = {}
    for sequence in sorted(required):
        frame_results, annotated_rows, runtime = audit_sequence(
            sequence,
            feature_paths[sequence],
            read_csv(motion_paths[sequence]),
            coverage_by_key,
            mask_dirs[sequence],
            args.online_semantic,
        )
        all_frame_results.extend(frame_results)
        all_annotated_rows.extend(annotated_rows)
        runtime_by_sequence[sequence] = runtime

    strata = aggregate_strata(
        all_frame_results, all_annotated_rows
    )
    moving_absent_frames = strata.get(
        "moving_person_absent", {}
    ).get("frames", 0)
    moving_observable_zero_overlap_frames = sum(
        frame["motion_label"] == "moving_observable"
        and frame["visibility"] in ("visible", "partial", "occluded")
        and frame["target_semantic_exact_zero_overlap"] == 1
        for frame in all_frame_results
    )
    semantic_uncovered_moving_frames = [
        frame
        for frame in all_frame_results
        if frame["motion_label"] == "moving_observable"
        and frame["visibility"] in ("visible", "partial", "occluded")
        and frame["target_semantic_exact_zero_overlap"] == 1
    ]
    primary_gate_evaluable = (
        moving_absent_frames >= 3
        or moving_observable_zero_overlap_frames >= 5
    )
    summary = {
        "identity": (
            "G2-4F1 continuous sparse observed-flow minus ego-flow "
            "development audit"
        ),
        "motion_labels_are_ground_truth": False,
        "dynamic_threshold": None,
        "dynamic_decision": "none",
        "depth_flow_fusion": "none",
        "direct_slam_state_mutation": "none",
        "g1_release": False,
        "strict_holdout_opened": False,
        "online_semantic": args.online_semantic,
        "input_note": (
            "Synchronous online semantic masks have age zero; exact saved "
            "C++ candidate masks match every current-feature semantic flag, "
            "and successful-frame reference depth is semantic-cleaned."
            if args.online_semantic
            else
            "These runs omit online semantic inference; exact saved C++ "
            "person masks stratify current-frame features offline, but "
            "reference depth was not semantic-cleaned."
        ),
        "gt_pose_note": (
            "Bonn GT residual uses the official configured right-frame "
            "chain, but remains a diagnostic pending synchronization/frame "
            "validation and must not define a motion gate."
        ),
        "frames": len(all_frame_results),
        "moving_person_absent_frames": moving_absent_frames,
        "moving_observable_exact_zero_target_overlap_frames":
            moving_observable_zero_overlap_frames,
        "semantic_uncovered_moving_paired_inside_outside":
            paired_inside_outside_summary(
                semantic_uncovered_moving_frames
            ),
        "primary_gate_evaluable": primary_gate_evaluable,
        "primary_gate_reason": (
            "development proxy has at least three moving+person-absent "
            "frames"
            if moving_absent_frames >= 3
            else (
                "development proxy has at least five moving-object frames "
                "with exact zero person-mask pixels in the object bbox"
                if moving_observable_zero_overlap_frames >= 5
                else "insufficient independently reviewed semantic-uncovered "
                "moving-object frames"
            )
        ),
        "strata": strata,
        "runtime_by_sequence": runtime_by_sequence,
        "development_proxy_comparisons": {
            "moving_person_absent_vs_stationary_person_absent":
                proxy_comparison(
                    all_frame_results,
                    "moving_person_absent",
                    "stationary_person_absent",
                ),
            "moving_person_present_vs_stationary_person_absent":
                proxy_comparison(
                    all_frame_results,
                    "moving_person_present",
                    "stationary_person_absent",
                ),
        },
        "inputs": {
            "feature": {
                key: str(path) for key, path in feature_paths.items()
            },
            "motion_proxy": {
                key: str(path) for key, path in motion_paths.items()
            },
            "coverage": str(args.coverage),
            "person_mask_dir": {
                key: str(path) for key, path in mask_dirs.items()
            },
        },
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    flattened = [
        flatten_frame_result(frame)
        for frame in sorted(
            all_frame_results,
            key=lambda row: (row["sequence"], row["frame"]),
        )
    ]
    frame_path = args.output_dir / "per_frame.csv"
    with frame_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flattened[0]))
        writer.writeheader()
        writer.writerows(flattened)
    print(
        "[G2-4F1 Sparse Flow Audit] frames={} moving_person_absent={} "
        "primary_gate_evaluable={} dynamic_decision=none "
        "direct_slam_state_mutation=none".format(
            len(flattened),
            moving_absent_frames,
            str(summary["primary_gate_evaluable"]).lower(),
        )
    )
    print("[G2-4F1 Sparse Flow Audit] wrote {}".format(summary_path))
    print("[G2-4F1 Sparse Flow Audit] wrote {}".format(frame_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
