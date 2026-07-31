#!/usr/bin/env python3
"""Count hypothetical sparse-flow MapPoint removals without mutating SLAM."""

import argparse
import csv
import json
from pathlib import Path

from audit_sparse_flow_feature_gate import percentile, read_feature_csv


FB_THRESHOLD_PX = 0.25
SCALE_FACTOR = 1.4826
SCALE_FLOOR_PX = 0.001
MINIMUM_SCALE_SUPPORT = 20
Q_THRESHOLDS = (6.0, 8.0, 10.0)
SCALE_MODES = (
    "semantic_blind_all_eligible",
    "combined_semantic_excluded",
)
SUPPORT_LINES = (10, 15, 20, 30, 50)
STATIC_MAPPOINT_BUDGET = 0.002


def measurement_eligible(row):
    return (
        row["evidence_state"] == "measured"
        and row["fb_error_px"] is not None
        and row["residual_px"] is not None
        and row["fb_error_px"] <= FB_THRESHOLD_PX
    )


def distribution(values):
    return {
        "n": len(values),
        "median": percentile(values, 0.5),
        "p10": percentile(values, 0.1),
        "p95": percentile(values, 0.95),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def frame_scale(rows, scale_mode):
    measured = [row for row in rows if measurement_eligible(row)]
    candidates = [
        row for row in measured if not row["semantic_nonzero"]]
    scale_rows = (
        measured if scale_mode == "semantic_blind_all_eligible"
        else candidates
    )
    if len(scale_rows) < MINIMUM_SCALE_SUPPORT:
        return None, candidates, len(scale_rows)
    return max(
        SCALE_FLOOR_PX,
        SCALE_FACTOR * percentile(
            [row["residual_px"] for row in scale_rows], 0.5),
    ), candidates, len(scale_rows)


def frame_row(sequence, role, frame, rows, scale_mode, threshold):
    scale, eligible, scale_support = frame_scale(rows, scale_mode)
    baseline_mappoint = sum(row["has_mappoint"] for row in rows)
    eligible_mappoint = sum(row["has_mappoint"] for row in eligible)
    if scale is None:
        candidates = []
        state = "insufficient_scale_fail_open"
    else:
        candidates = [
            row for row in eligible
            if row["residual_px"] / scale >= threshold
        ]
        state = "evaluable"
    candidate_mappoint = sum(
        row["has_mappoint"] for row in candidates)
    remaining_mappoint = baseline_mappoint - candidate_mappoint
    output = {
        "sequence": sequence,
        "role": role,
        "frame": frame,
        "scale_mode": scale_mode,
        "q_threshold": threshold,
        "state": state,
        "frame_scale_px": scale if scale is not None else "",
        "scale_support": scale_support,
        "baseline_associated_mappoint_proxy": baseline_mappoint,
        "eligible_nonsemantic_mappoint": eligible_mappoint,
        "candidate_mappoint": candidate_mappoint,
        "remaining_mappoint_proxy": remaining_mappoint,
        "candidate_over_eligible_mappoint": (
            candidate_mappoint / eligible_mappoint
            if eligible_mappoint else ""),
        "candidate_over_baseline_mappoint": (
            candidate_mappoint / baseline_mappoint
            if baseline_mappoint else ""),
        "eligible_nonsemantic_features": len(eligible),
        "candidate_features": len(candidates),
        "candidate_feature_rate": (
            len(candidates) / len(eligible) if eligible else ""),
        "counterfactual_only": "true",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "pose_reoptimization": "none",
    }
    for line in SUPPORT_LINES:
        output["baseline_below_{}".format(line)] = int(
            baseline_mappoint < line)
        output["crossed_below_{}".format(line)] = int(
            baseline_mappoint >= line and remaining_mappoint < line)
    return output


def aggregate(sequence, role, scale_mode, threshold, rows):
    evaluable = [row for row in rows if row["state"] == "evaluable"]
    baseline = [
        int(row["baseline_associated_mappoint_proxy"])
        for row in evaluable]
    candidates = [
        int(row["candidate_mappoint"]) for row in evaluable]
    remaining = [
        int(row["remaining_mappoint_proxy"]) for row in evaluable]
    eligible_mappoint = sum(
        int(row["eligible_nonsemantic_mappoint"]) for row in evaluable)
    candidate_mappoint = sum(candidates)
    baseline_mappoint = sum(baseline)
    output = {
        "sequence": sequence,
        "role": role,
        "scale_mode": scale_mode,
        "q_threshold": threshold,
        "frame_count": len(rows),
        "evaluable_frame_count": len(evaluable),
        "fail_open_frame_count": len(rows)-len(evaluable),
        "baseline_associated_mappoint_proxy": distribution(baseline),
        "candidate_mappoint": distribution(candidates),
        "remaining_mappoint_proxy": distribution(remaining),
        "total_eligible_nonsemantic_mappoint": eligible_mappoint,
        "total_candidate_mappoint": candidate_mappoint,
        "candidate_over_eligible_mappoint_rate": (
            candidate_mappoint / eligible_mappoint
            if eligible_mappoint else None),
        "candidate_over_baseline_mappoint_rate": (
            candidate_mappoint / baseline_mappoint
            if baseline_mappoint else None),
        "static_budget_le_0_20pct": (
            candidate_mappoint / eligible_mappoint
            <= STATIC_MAPPOINT_BUDGET
            if eligible_mappoint else None),
    }
    for line in SUPPORT_LINES:
        output["baseline_below_{}_frames".format(line)] = sum(
            int(row["baseline_below_{}".format(line)])
            for row in evaluable)
        output["crossed_below_{}_frames".format(line)] = sum(
            int(row["crossed_below_{}".format(line)])
            for row in evaluable)
    return output


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def self_test():
    rows = []
    for index in range(20):
        rows.append({
            "has_mappoint": index < 12,
            "semantic_nonzero": False,
            "evidence_state": "measured",
            "fb_error_px": 0.0,
            "residual_px": 1.0 if index < 19 else 20.0,
        })
    result = frame_row(
        "test", "test", 1,
        rows, "semantic_blind_all_eligible", 6.0)
    assert result["baseline_associated_mappoint_proxy"] == 12
    assert result["candidate_mappoint"] == 0
    assert result["remaining_mappoint_proxy"] == 12
    rows[-1]["has_mappoint"] = True
    result = frame_row(
        "test", "test", 1,
        rows, "semantic_blind_all_eligible", 6.0)
    assert result["candidate_mappoint"] == 1
    assert result["remaining_mappoint_proxy"] == 12
    assert result["crossed_below_10"] == 0
    print("audit_initial_mappoint_counterfactual self-test PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input", nargs=3, action="append",
        metavar=("NAME", "ROLE", "FEATURE_CSV"))
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
    for sequence, role, path_string in args.input:
        frames = read_feature_csv(Path(path_string))
        summaries[sequence] = {}
        for scale_mode in SCALE_MODES:
            summaries[sequence][scale_mode] = {}
            for threshold in Q_THRESHOLDS:
                threshold_rows = [
                    frame_row(
                        sequence, role, frame, frames[frame],
                        scale_mode, threshold)
                    for frame in sorted(frames)]
                per_frame.extend(threshold_rows)
                summaries[sequence][scale_mode][str(threshold)] = aggregate(
                    sequence, role, scale_mode, threshold, threshold_rows)

    pass_by_scale_mode = {}
    for scale_mode in SCALE_MODES:
        pass_by_threshold = {}
        for threshold in Q_THRESHOLDS:
            key = str(threshold)
            static_items = [
                values[scale_mode][key] for values in summaries.values()
                if values[scale_mode][key]["role"]
                == "true_static_negative"]
            all_items = [
                values[scale_mode][key] for values in summaries.values()]
            conditions = {
                "both_static_domains_available": len(static_items) >= 2,
                "static_budget_pass": (
                    len(static_items) >= 2 and all(
                        item["static_budget_le_0_20pct"]
                        for item in static_items)),
                "no_cross_below_10_all_sequences": all(
                    item["crossed_below_10_frames"] == 0
                    for item in all_items),
            }
            conditions["pass"] = all(conditions.values())
            pass_by_threshold[key] = conditions
        pass_by_scale_mode[scale_mode] = pass_by_threshold

    output = {
        "parameters": {
            "fb_threshold_px": FB_THRESHOLD_PX,
            "scale_factor": SCALE_FACTOR,
            "scale_floor_px": SCALE_FLOOR_PX,
            "minimum_scale_support": MINIMUM_SCALE_SUPPORT,
            "q_thresholds": list(Q_THRESHOLDS),
            "scale_modes": list(SCALE_MODES),
            "static_mappoint_budget": STATIC_MAPPOINT_BUDGET,
            "support_lines": list(SUPPORT_LINES),
        },
        "sequences": summaries,
        "pass_by_scale_mode": pass_by_scale_mode,
        "any_threshold_initial_support_pass": any(
            item["pass"]
            for by_threshold in pass_by_scale_mode.values()
            for item in by_threshold.values()),
        "both_scale_modes_have_initial_support": all(
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
            "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
