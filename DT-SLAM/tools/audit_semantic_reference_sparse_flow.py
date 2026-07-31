#!/usr/bin/env python3
"""Audit semantic-blind sparse-flow scores against semantic region proxies.

Semantic labels are read only after F1 measurement and are never used to
compute LK, ego flow, frame scale, or a dynamic decision.
"""

import argparse
import csv
import json
import math
from pathlib import Path

from audit_sparse_flow_feature_gate import percentile, read_feature_csv


FB_THRESHOLD_PX = 0.25
SCALE_FACTOR = 1.4826
SCALE_FLOOR_PX = 0.001
MINIMUM_SCALE_SUPPORT = 20
MINIMUM_SEMANTIC_SUPPORT = 3
Q_THRESHOLDS = (2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)


def parse_input(values):
    name, role, path = values
    return name, role, Path(path)


def base_eligible(row):
    return (
        row["evidence_state"] == "measured"
        and row["fb_error_px"] is not None
        and row["residual_px"] is not None
        and row["fb_error_px"] <= FB_THRESHOLD_PX
    )


def tied_rank_auc(positive, negative):
    if not positive or not negative:
        return None
    ordered = sorted(
        [(float(value), 1) for value in positive]
        + [(float(value), 0) for value in negative],
        key=lambda item: item[0],
    )
    positive_rank_sum = 0.0
    index = 0
    while index < len(ordered):
        end = index + 1
        while end < len(ordered) and ordered[end][0] == ordered[index][0]:
            end += 1
        average_rank = 0.5 * ((index + 1) + end)
        positive_rank_sum += average_rank * sum(
            item[1] for item in ordered[index:end]
        )
        index = end
    n_positive = len(positive)
    n_negative = len(negative)
    return (
        positive_rank_sum
        - n_positive * (n_positive + 1) / 2.0
    ) / (n_positive * n_negative)


def safe_ratio(numerator, denominator):
    if denominator is None or denominator == 0.0:
        return None
    return numerator / denominator


def metric_summary(values):
    return {
        "n": len(values),
        "median": percentile(values, 0.5),
        "p10": percentile(values, 0.1),
        "p90": percentile(values, 0.9),
    }


def summarize_frame(sequence, role, frame, rows):
    eligible = [row for row in rows if base_eligible(row)]
    nonsemantic_scale_rows = [
        row for row in eligible if not row["semantic_nonzero"]
    ]
    blind_scale_valid = len(eligible) >= MINIMUM_SCALE_SUPPORT
    combined_scale_valid = (
        len(nonsemantic_scale_rows) >= MINIMUM_SCALE_SUPPORT)
    blind_scale = (
        max(
            SCALE_FLOOR_PX,
            SCALE_FACTOR * percentile(
                [row["residual_px"] for row in eligible], 0.5),
        )
        if blind_scale_valid
        else None
    )
    combined_scale = (
        max(
            SCALE_FLOOR_PX,
            SCALE_FACTOR * percentile(
                [row["residual_px"]
                 for row in nonsemantic_scale_rows], 0.5),
        )
        if combined_scale_valid
        else None
    )
    semantic = [
        row for row in eligible if row["semantic_nonzero"]
    ]
    nonsemantic = [
        row for row in eligible if not row["semantic_nonzero"]
    ]
    comparable = (
        blind_scale_valid
        and combined_scale_valid
        and len(semantic) >= MINIMUM_SEMANTIC_SUPPORT
        and len(nonsemantic) >= MINIMUM_SCALE_SUPPORT
    )
    semantic_raw = [row["residual_px"] for row in semantic]
    nonsemantic_raw = [row["residual_px"] for row in nonsemantic]
    semantic_q_blind = (
        [value / blind_scale for value in semantic_raw]
        if blind_scale_valid else []
    )
    nonsemantic_q_blind = (
        [value / blind_scale for value in nonsemantic_raw]
        if blind_scale_valid else []
    )
    semantic_q_combined = (
        [value / combined_scale for value in semantic_raw]
        if combined_scale_valid else []
    )
    nonsemantic_q_combined = (
        [value / combined_scale for value in nonsemantic_raw]
        if combined_scale_valid else []
    )
    semantic_mappoint = [
        row["residual_px"] for row in semantic if row["has_mappoint"]
    ]
    nonsemantic_mappoint = [
        row["residual_px"] for row in nonsemantic if row["has_mappoint"]
    ]
    semantic_median = percentile(semantic_raw, 0.5)
    nonsemantic_median = percentile(nonsemantic_raw, 0.5)
    semantic_q_median = percentile(semantic_q_blind, 0.5)
    nonsemantic_q_median = percentile(nonsemantic_q_blind, 0.5)
    return {
        "sequence": sequence,
        "role": role,
        "frame": frame,
        "quality_eligible_count": len(eligible),
        "semantic_count": len(semantic),
        "nonsemantic_count": len(nonsemantic),
        "semantic_mappoint_count": len(semantic_mappoint),
        "nonsemantic_mappoint_count": len(nonsemantic_mappoint),
        "blind_scale_support": len(eligible),
        "combined_scale_support": len(nonsemantic_scale_rows),
        "blind_scale_valid": int(blind_scale_valid),
        "combined_scale_valid": int(combined_scale_valid),
        "blind_frame_scale_px":
            blind_scale if blind_scale is not None else "",
        "combined_frame_scale_px":
            combined_scale if combined_scale is not None else "",
        "comparable": int(comparable),
        "semantic_raw_median_px":
            semantic_median if semantic_median is not None else "",
        "nonsemantic_raw_median_px":
            nonsemantic_median if nonsemantic_median is not None else "",
        "semantic_raw_p90_px":
            percentile(semantic_raw, 0.9) if semantic_raw else "",
        "nonsemantic_raw_p90_px":
            percentile(nonsemantic_raw, 0.9) if nonsemantic_raw else "",
        "semantic_q_median":
            semantic_q_median if semantic_q_median is not None else "",
        "nonsemantic_q_median":
            nonsemantic_q_median if nonsemantic_q_median is not None else "",
        "semantic_q_p90":
            percentile(semantic_q_blind, 0.9)
            if semantic_q_blind else "",
        "nonsemantic_q_p90":
            percentile(nonsemantic_q_blind, 0.9)
            if nonsemantic_q_blind else "",
        "semantic_combined_q_median": (
            percentile(semantic_q_combined, 0.5)
            if semantic_q_combined else ""),
        "nonsemantic_combined_q_median": (
            percentile(nonsemantic_q_combined, 0.5)
            if nonsemantic_q_combined else ""),
        "raw_median_difference_px": (
            semantic_median - nonsemantic_median
            if comparable else ""),
        "raw_median_ratio": (
            safe_ratio(semantic_median, nonsemantic_median)
            if comparable else ""),
        "q_median_difference": (
            semantic_q_median - nonsemantic_q_median
            if comparable else ""),
        "q_median_ratio": (
            safe_ratio(semantic_q_median, nonsemantic_q_median)
            if comparable else ""),
        "raw_auc": (
            tied_rank_auc(semantic_raw, nonsemantic_raw)
            if comparable else ""),
        "q_auc": (
            tied_rank_auc(semantic_q_blind, nonsemantic_q_blind)
            if comparable else ""),
        "mappoint_raw_auc": (
            tied_rank_auc(semantic_mappoint, nonsemantic_mappoint)
            if len(semantic_mappoint) >= MINIMUM_SEMANTIC_SUPPORT
            and len(nonsemantic_mappoint) >= MINIMUM_SCALE_SUPPORT
            else ""),
        "reference_identity":
            "semantic person-region proxy; not motion ground truth",
        "raw_residual_uses_semantic": "false",
        "blind_scale_uses_semantic": "false",
        "combined_scale_excludes_semantic": "true",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }, eligible, {
        "semantic_blind_all_eligible": blind_scale,
        "combined_semantic_excluded": combined_scale,
    }


def aggregate_sequence(name, role, frame_rows):
    comparable = [row for row in frame_rows if row["comparable"] == 1]
    raw_auc = [float(row["raw_auc"]) for row in comparable]
    q_auc = [float(row["q_auc"]) for row in comparable]
    return {
        "sequence": name,
        "role": role,
        "frame_count": len(frame_rows),
        "comparable_frame_count": len(comparable),
        "semantic_raw_median_gt_nonsemantic_fraction": (
            sum(
                float(row["semantic_raw_median_px"])
                > float(row["nonsemantic_raw_median_px"])
                for row in comparable
            ) / len(comparable)
            if comparable else None
        ),
        "semantic_q_median_gt_nonsemantic_fraction": (
            sum(
                float(row["semantic_q_median"])
                > float(row["nonsemantic_q_median"])
                for row in comparable
            ) / len(comparable)
            if comparable else None
        ),
        "raw_auc": metric_summary(raw_auc),
        "q_auc": metric_summary(q_auc),
        "continuous_support_conditions": {
            "comparable_frames_ge_20": len(comparable) >= 20,
            "raw_direction_ge_80pct": (
                bool(comparable)
                and sum(
                    float(row["semantic_raw_median_px"])
                    > float(row["nonsemantic_raw_median_px"])
                    for row in comparable
                ) / len(comparable) >= 0.8
            ),
            "q_direction_ge_80pct": (
                bool(comparable)
                and sum(
                    float(row["semantic_q_median"])
                    > float(row["nonsemantic_q_median"])
                    for row in comparable
                ) / len(comparable) >= 0.8
            ),
            "auc_median_ge_075": (
                percentile(raw_auc, 0.5) is not None
                and percentile(raw_auc, 0.5) >= 0.75
            ),
        },
    }


def candidate_curve_rows(sequence, role, frames, scales):
    output = []
    for scale_mode in (
            "semantic_blind_all_eligible",
            "combined_semantic_excluded"):
        for threshold in Q_THRESHOLDS:
            counts = {
            "semantic_eligible": 0,
            "semantic_candidate": 0,
            "nonsemantic_eligible": 0,
            "nonsemantic_candidate": 0,
            "semantic_mappoint_eligible": 0,
            "semantic_mappoint_candidate": 0,
            "nonsemantic_mappoint_eligible": 0,
            "nonsemantic_mappoint_candidate": 0,
            }
            valid_frames = 0
            for frame in sorted(frames):
                scale = scales.get(frame, {}).get(scale_mode)
                if scale is None:
                    continue
                valid_frames += 1
                for row in frames[frame]:
                    if not base_eligible(row):
                        continue
                    group = (
                        "semantic" if row["semantic_nonzero"]
                        else "nonsemantic"
                    )
                    q_value = row["residual_px"] / scale
                    counts[group + "_eligible"] += 1
                    if row["has_mappoint"]:
                        counts[group + "_mappoint_eligible"] += 1
                    if q_value >= threshold:
                        counts[group + "_candidate"] += 1
                        if row["has_mappoint"]:
                            counts[group + "_mappoint_candidate"] += 1
            row = {
                "sequence": sequence,
                "role": role,
                "scale_mode": scale_mode,
                "q_threshold": threshold,
                "valid_scale_frames": valid_frames,
                **counts,
            }
            for group in ("semantic", "nonsemantic"):
                eligible_count = counts[group + "_eligible"]
                candidate_count = counts[group + "_candidate"]
                mappoint_eligible = counts[
                    group + "_mappoint_eligible"]
                mappoint_candidate = counts[
                    group + "_mappoint_candidate"]
                row[group + "_candidate_rate"] = (
                    candidate_count / eligible_count
                    if eligible_count else "")
                row[group + "_mappoint_candidate_rate"] = (
                    mappoint_candidate / mappoint_eligible
                    if mappoint_eligible else "")
            row.update({
                "reference_identity":
                    "semantic person-region proxy; not motion ground truth",
                "raw_residual_uses_semantic": "false",
                "scale_uses_semantic": str(
                    scale_mode ==
                    "combined_semantic_excluded").lower(),
                "dynamic_decision": "none",
                "direct_slam_state_mutation": "none",
            })
            output.append(row)
    return output


def write_csv(path, rows):
    if not rows:
        raise ValueError("cannot write empty CSV")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def self_test():
    assert tied_rank_auc([2.0, 3.0], [0.0, 1.0]) == 1.0
    assert tied_rank_auc([0.0, 1.0], [2.0, 3.0]) == 0.0
    assert math.isclose(tied_rank_auc([1.0], [1.0]), 0.5)
    rows = []
    for index, (semantic, residual) in enumerate(
            [(False, 1.0)] * 20 + [(True, 10.0)] * 21):
        rows.append({
            "feature_index": index,
            "has_mappoint": False,
            "semantic_nonzero": semantic,
            "evidence_state": "measured",
            "fb_error_px": 0.0,
            "residual_px": residual,
        })
    summary, _, scales = summarize_frame("test", "test", 1, rows)
    assert scales["semantic_blind_all_eligible"] > SCALE_FACTOR
    assert math.isclose(
        scales["combined_semantic_excluded"], SCALE_FACTOR)
    assert summary["comparable"] == 1
    assert summary["raw_auc"] == 1.0
    assert summary["semantic_raw_median_px"] == 10.0
    print("audit_semantic_reference_sparse_flow self-test PASS")


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

    all_frame_rows = []
    all_curve_rows = []
    summaries = {}
    for values in args.input:
        name, role, path = parse_input(values)
        frames = read_feature_csv(path)
        frame_rows = []
        scales = {}
        for frame in sorted(frames):
            summary, _, scale = summarize_frame(
                name, role, frame, frames[frame])
            frame_rows.append(summary)
            scales[frame] = scale
        all_frame_rows.extend(frame_rows)
        all_curve_rows.extend(
            candidate_curve_rows(name, role, frames, scales))
        summaries[name] = aggregate_sequence(name, role, frame_rows)

    walking = [
        summary for summary in summaries.values()
        if summary["role"] == "walking_semantic_development"]
    any_walking_support = any(
        all(item["continuous_support_conditions"].values())
        for item in walking
    )
    output = {
        "parameters": {
            "fb_threshold_px": FB_THRESHOLD_PX,
            "scale_factor": SCALE_FACTOR,
            "scale_floor_px": SCALE_FLOOR_PX,
            "minimum_scale_support": MINIMUM_SCALE_SUPPORT,
            "minimum_semantic_support": MINIMUM_SEMANTIC_SUPPORT,
            "q_thresholds": list(Q_THRESHOLDS),
        },
        "sequences": summaries,
        "any_walking_continuous_support_pass": any_walking_support,
        "unknown_cross_class_result":
            "reuse frozen balloon/balloon2 result; not recomputed here",
        "reference_identity":
            "semantic person-region proxy; not motion ground truth",
        "raw_residual_uses_semantic": False,
        "semantic_blind_scale_uses_semantic": False,
        "combined_scale_excludes_semantic": True,
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "per_frame.csv", all_frame_rows)
    write_csv(args.output_dir / "candidate_curve.csv", all_curve_rows)
    with (args.output_dir / "summary.json").open(
            "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
