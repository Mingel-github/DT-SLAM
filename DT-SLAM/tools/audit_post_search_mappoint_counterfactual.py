#!/usr/bin/env python3
"""Join F1 residuals to G1-F0B raw association snapshots."""

import argparse
import csv
import json
import math
from pathlib import Path

from audit_initial_mappoint_counterfactual import (
    Q_THRESHOLDS,
    SCALE_MODES,
    STATIC_MAPPOINT_BUDGET,
    SUPPORT_LINES,
    frame_scale,
)
from audit_sparse_flow_feature_gate import (
    binary_flag,
    percentile,
    read_feature_csv,
)


STAGES = ("post_search_pre_pose", "post_existing_pose")
SNAPSHOT_REQUIRED_COLUMNS = {
    "frame",
    "timestamp",
    "stage",
    "feature_index",
    "has_mappoint",
    "mappoint_bad",
    "mappoint_observations",
    "current_frame_outlier",
    "semantic_nonzero",
    "only_tracking",
    "within_relocalization_window",
    "counted_tracking_inlier",
    "tracking_inliers",
    "counterfactual_only",
    "dynamic_decision",
    "direct_slam_state_mutation",
    "pose_reoptimization",
}


def distribution(values):
    return {
        "n": len(values),
        "median": percentile(values, 0.5),
        "p10": percentile(values, 0.1),
        "p95": percentile(values, 0.95),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def read_snapshot_csv(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("{} has no CSV header".format(path))
        missing = SNAPSHOT_REQUIRED_COLUMNS - set(reader.fieldnames)
        if missing:
            raise ValueError(
                "{} missing columns: {}".format(
                    path, ", ".join(sorted(missing))))
        raw_rows = list(reader)
        if any(None in row for row in raw_rows):
            raise ValueError("{} has rows wider than header".format(path))

    snapshots = {}
    seen = set()
    for row_number, row in enumerate(raw_rows, start=2):
        try:
            frame = int(row["frame"])
            feature_index = int(row["feature_index"])
            observations = int(row["mappoint_observations"])
            tracking_inliers = int(row["tracking_inliers"])
            timestamp = float(row["timestamp"])
        except ValueError as error:
            raise ValueError(
                "{}:{} invalid numeric field".format(
                    path, row_number)) from error
        if not math.isfinite(timestamp):
            raise ValueError(
                "{}:{} timestamp is non-finite".format(
                    path, row_number))
        stage = row["stage"]
        if stage not in STAGES:
            raise ValueError(
                "{}:{} invalid stage {!r}".format(
                    path, row_number, stage))
        key = (frame, stage, feature_index)
        if key in seen:
            raise ValueError(
                "{}:{} duplicate key {}".format(
                    path, row_number, key))
        seen.add(key)
        if row["counterfactual_only"] != "true":
            raise ValueError(
                "{}:{} is not counterfactual-only".format(
                    path, row_number))
        for field in (
            "dynamic_decision",
            "direct_slam_state_mutation",
            "pose_reoptimization",
        ):
            if row[field] != "none":
                raise ValueError(
                    "{}:{} {} must be none".format(
                        path, row_number, field))
        parsed = {
            "frame": frame,
            "timestamp": timestamp,
            "stage": stage,
            "feature_index": feature_index,
            "has_mappoint": binary_flag(
                row["has_mappoint"], "has_mappoint"),
            "mappoint_bad": binary_flag(
                row["mappoint_bad"], "mappoint_bad"),
            "mappoint_observations": observations,
            "current_frame_outlier": binary_flag(
                row["current_frame_outlier"],
                "current_frame_outlier"),
            "semantic_nonzero": binary_flag(
                row["semantic_nonzero"], "semantic_nonzero"),
            "only_tracking": binary_flag(
                row["only_tracking"], "only_tracking"),
            "within_relocalization_window": binary_flag(
                row["within_relocalization_window"],
                "within_relocalization_window"),
            "counted_tracking_inlier": binary_flag(
                row["counted_tracking_inlier"],
                "counted_tracking_inlier"),
            "tracking_inliers": tracking_inliers,
        }
        if observations < 0:
            raise ValueError(
                "{}:{} negative observations".format(
                    path, row_number))
        expected_inlier_state = (
            tracking_inliers == -1
            if stage == "post_search_pre_pose"
            else tracking_inliers >= 0
        )
        if not expected_inlier_state:
            raise ValueError(
                "{}:{} invalid tracking_inliers for stage".format(
                    path, row_number))
        snapshots.setdefault((frame, stage), {})[
            feature_index] = parsed
    return snapshots


def association_is_supported(snapshot):
    if snapshot["stage"] == "post_existing_pose":
        return snapshot["counted_tracking_inlier"]
    if not snapshot["has_mappoint"] or snapshot["mappoint_bad"]:
        return False
    return True


def frame_audit(
    sequence,
    role,
    frame,
    stage,
    feature_rows,
    snapshot_by_index,
    scale_mode,
    threshold,
):
    feature_by_index = {
        row["feature_index"]: row for row in feature_rows}
    if set(feature_by_index) != set(snapshot_by_index):
        raise ValueError(
            "{} frame {} stage {} feature-index set mismatch".format(
                sequence, frame, stage))
    for feature_index, snapshot in snapshot_by_index.items():
        if (
            snapshot["semantic_nonzero"]
            != feature_by_index[feature_index]["semantic_nonzero"]
        ):
            raise ValueError(
                "{} frame {} feature {} semantic mismatch".format(
                    sequence, frame, feature_index))

    scale, eligible_rows, scale_support = frame_scale(
        feature_rows, scale_mode)
    eligible_indices = {
        row["feature_index"] for row in eligible_rows}
    candidate_indices = set()
    if scale is not None:
        candidate_indices = {
            row["feature_index"] for row in eligible_rows
            if row["residual_px"] / scale >= threshold
        }

    supported = {
        feature_index
        for feature_index, snapshot in snapshot_by_index.items()
        if association_is_supported(snapshot)
    }
    eligible_supported = supported & eligible_indices
    candidate_supported = supported & candidate_indices
    baseline = len(supported)
    candidate = len(candidate_supported)
    remaining = baseline - candidate
    first_snapshot = next(iter(snapshot_by_index.values()))
    tracking_inliers = first_snapshot["tracking_inliers"]
    if any(
        row["tracking_inliers"] != tracking_inliers
        for row in snapshot_by_index.values()
    ):
        raise ValueError(
            "{} frame {} stage {} mixed tracking inliers".format(
                sequence, frame, stage))
    if stage == "post_existing_pose" and baseline != tracking_inliers:
        raise ValueError(
            "{} frame {} post-pose proxy {} != tracking inliers {}".format(
                sequence, frame, baseline, tracking_inliers))

    output = {
        "sequence": sequence,
        "role": role,
        "frame": frame,
        "stage": stage,
        "scale_mode": scale_mode,
        "q_threshold": threshold,
        "state": (
            "evaluable" if scale is not None
            else "insufficient_scale_fail_open"),
        "frame_scale_px": scale if scale is not None else "",
        "scale_support": scale_support,
        "baseline_supported_mappoint": baseline,
        "eligible_nonsemantic_mappoint": len(eligible_supported),
        "candidate_mappoint": candidate,
        "remaining_mappoint": remaining,
        "candidate_over_eligible_mappoint": (
            candidate / len(eligible_supported)
            if eligible_supported else ""),
        "candidate_over_baseline_mappoint": (
            candidate / baseline if baseline else ""),
        "tracking_inliers": tracking_inliers,
        "only_tracking": int(first_snapshot["only_tracking"]),
        "within_relocalization_window": int(
            first_snapshot["within_relocalization_window"]),
        "counterfactual_only": "true",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "pose_reoptimization": "none",
    }
    for line in SUPPORT_LINES:
        output["baseline_below_{}".format(line)] = int(
            baseline < line)
        output["crossed_below_{}".format(line)] = int(
            baseline >= line and remaining < line)
    return output


def aggregate(sequence, role, stage, scale_mode, threshold, rows):
    evaluable = [row for row in rows if row["state"] == "evaluable"]
    baseline = [
        int(row["baseline_supported_mappoint"]) for row in evaluable]
    candidates = [
        int(row["candidate_mappoint"]) for row in evaluable]
    remaining = [
        int(row["remaining_mappoint"]) for row in evaluable]
    eligible = sum(
        int(row["eligible_nonsemantic_mappoint"]) for row in evaluable)
    candidate = sum(candidates)
    result = {
        "sequence": sequence,
        "role": role,
        "stage": stage,
        "scale_mode": scale_mode,
        "q_threshold": threshold,
        "frame_count": len(rows),
        "evaluable_frame_count": len(evaluable),
        "fail_open_frame_count": len(rows) - len(evaluable),
        "baseline_supported_mappoint": distribution(baseline),
        "candidate_mappoint": distribution(candidates),
        "remaining_mappoint": distribution(remaining),
        "total_eligible_nonsemantic_mappoint": eligible,
        "total_candidate_mappoint": candidate,
        "candidate_over_eligible_mappoint_rate": (
            candidate / eligible if eligible else None),
        "static_budget_le_0_20pct": (
            candidate / eligible <= STATIC_MAPPOINT_BUDGET
            if eligible else None),
    }
    for line in SUPPORT_LINES:
        result["crossed_below_{}_frames".format(line)] = sum(
            int(row["crossed_below_{}".format(line)])
            for row in evaluable)
    result["strict_relocalization_cross_below_50_frames"] = sum(
        int(row["crossed_below_50"])
        for row in evaluable
        if int(row["within_relocalization_window"]) == 1)
    return result


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def self_test():
    feature_rows = []
    snapshots = {}
    for index in range(20):
        feature_rows.append({
            "feature_index": index,
            "semantic_nonzero": False,
            "evidence_state": "measured",
            "fb_error_px": 0.0,
            "residual_px": 20.0 if index == 19 else 1.0,
        })
        snapshots[index] = {
            "feature_index": index,
            "stage": "post_existing_pose",
            "has_mappoint": index < 12 or index == 19,
            "mappoint_bad": False,
            "mappoint_observations": 2,
            "current_frame_outlier": False,
            "semantic_nonzero": False,
            "only_tracking": False,
            "within_relocalization_window": False,
            "counted_tracking_inlier": (
                index < 12 or index == 19),
            "tracking_inliers": 13,
        }
    row = frame_audit(
        "test", "test", 1, "post_existing_pose",
        feature_rows, snapshots,
        "semantic_blind_all_eligible", 6.0)
    assert row["baseline_supported_mappoint"] == 13
    assert row["candidate_mappoint"] == 1
    assert row["remaining_mappoint"] == 12
    assert row["crossed_below_10"] == 0
    print("audit_post_search_mappoint_counterfactual self-test PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input", nargs=4, action="append",
        metavar=("NAME", "ROLE", "F1_CSV", "SNAPSHOT_CSV"))
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.input or args.output_dir is None:
        parser.error("--input and --output-dir are required")

    per_frame = []
    summaries = {}
    input_integrity = {}
    for sequence, role, feature_path, snapshot_path in args.input:
        features = read_feature_csv(Path(feature_path))
        snapshots = read_snapshot_csv(Path(snapshot_path))
        snapshot_frames = sorted(
            {frame for frame, _ in snapshots})
        missing_feature_frames = [
            frame for frame in snapshot_frames if frame not in features]
        if missing_feature_frames:
            raise ValueError(
                "{} snapshot frames missing F1 rows: {}".format(
                    sequence, missing_feature_frames))
        stage_pairs = {
            frame: {
                stage for candidate_frame, stage in snapshots
                if candidate_frame == frame
            }
            for frame in snapshot_frames
        }
        incomplete = [
            frame for frame, stages in stage_pairs.items()
            if stages != set(STAGES)]
        if incomplete:
            raise ValueError(
                "{} frames missing a snapshot stage: {}".format(
                    sequence, incomplete))
        input_integrity[sequence] = {
            "feature_frame_count": len(features),
            "snapshot_frame_count": len(snapshot_frames),
            "f1_frames_without_snapshot": len(
                set(features) - set(snapshot_frames)),
            "feature_index_exact_match": True,
            "semantic_flag_exact_match": True,
            "post_pose_inlier_exact_match": True,
        }
        summaries[sequence] = {}
        for stage in STAGES:
            summaries[sequence][stage] = {}
            for scale_mode in SCALE_MODES:
                summaries[sequence][stage][scale_mode] = {}
                for threshold in Q_THRESHOLDS:
                    rows = [
                        frame_audit(
                            sequence, role, frame, stage,
                            features[frame],
                            snapshots[(frame, stage)],
                            scale_mode, threshold)
                        for frame in snapshot_frames]
                    per_frame.extend(rows)
                    summaries[sequence][stage][scale_mode][
                        str(threshold)
                    ] = aggregate(
                        sequence, role, stage, scale_mode,
                        threshold, rows)

    pass_by_scale_mode = {}
    for scale_mode in SCALE_MODES:
        pass_by_scale_mode[scale_mode] = {}
        for threshold in Q_THRESHOLDS:
            key = str(threshold)
            static_items = [
                stages["post_existing_pose"][scale_mode][key]
                for stages in summaries.values()
                if stages["post_existing_pose"][scale_mode][key][
                    "role"] == "true_static_negative"
            ]
            post_pose_items = [
                stages["post_existing_pose"][scale_mode][key]
                for stages in summaries.values()
            ]
            conditions = {
                "both_static_domains_available":
                    len(static_items) >= 2,
                "static_budget_pass": (
                    len(static_items) >= 2
                    and all(
                        item["static_budget_le_0_20pct"]
                        for item in static_items)),
                "no_cross_below_30_all_sequences": all(
                    item["crossed_below_30_frames"] == 0
                    for item in post_pose_items),
                "no_strict_relocalization_cross_below_50": all(
                    item[
                        "strict_relocalization_cross_below_50_frames"
                    ] == 0
                    for item in post_pose_items),
            }
            conditions["pass"] = all(conditions.values())
            pass_by_scale_mode[scale_mode][key] = conditions

    output = {
        "parameters": {
            "q_thresholds": list(Q_THRESHOLDS),
            "scale_modes": list(SCALE_MODES),
            "static_mappoint_budget": STATIC_MAPPOINT_BUDGET,
            "support_lines": list(SUPPORT_LINES),
        },
        "input_integrity": input_integrity,
        "sequences": summaries,
        "pass_by_scale_mode": pass_by_scale_mode,
        "both_scale_modes_have_post_search_support": all(
            any(item["pass"] for item in by_threshold.values())
            for by_threshold in pass_by_scale_mode.values()),
        "counterfactual_only": True,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "pose_reoptimization": "none",
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "per_frame.csv", per_frame)
    with (args.output_dir / "summary.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
