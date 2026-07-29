#!/usr/bin/env python3
"""Audit geometry-evidence separability without making a motion decision."""

import argparse
import csv
import json
import math
import statistics


SCORE_THRESHOLDS = (0.0, 0.05, 0.10, 0.20, 0.30, 0.50, 0.75, 1.0)


def ratio(numerator, denominator):
    return numerator / denominator if denominator else 0.0


def quantile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def rank_auc(positives, negatives):
    if not positives or not negatives:
        return None
    values = [(value, 1) for value in positives]
    values.extend((value, 0) for value in negatives)
    values.sort(key=lambda item: item[0])
    rank_sum = 0.0
    rank = 1
    index = 0
    while index < len(values):
        end = index + 1
        while end < len(values) and values[end][0] == values[index][0]:
            end += 1
        average_rank = (rank + rank + end - index - 1) / 2.0
        rank_sum += average_rank * sum(
            label for _, label in values[index:end])
        rank += end - index
        index = end
    positive_count = len(positives)
    negative_count = len(negatives)
    return (
        rank_sum - positive_count * (positive_count + 1) / 2.0
    ) / (positive_count * negative_count)


def load_rows(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as stream:
        for raw in csv.DictReader(stream):
            row = {
                "frame": int(raw["frame"]),
                "region_pixels": int(raw["region_pixels"]),
                "semantic_pixels": int(raw["semantic_proxy_pixels"]),
                "comparison_pixels": int(raw["comparison_pixels"]),
                "positive_pixels": int(raw["positive_presence_pixels"]),
                "comparison_votes": int(raw["comparison_votes"]),
                "positive_votes": int(raw["positive_votes"]),
                "semantic_ratio": float(raw["semantic_proxy_region_ratio"]),
                "positive_vote_ratio": float(raw["positive_vote_ratio"]),
            }
            if row["positive_votes"] > row["comparison_votes"]:
                raise ValueError("positive votes exceed comparison votes")
            rows.append(row)
    return rows


def proxy_group(row):
    if row["semantic_ratio"] >= 0.5:
        return "semantic_dominant_proxy"
    if row["semantic_pixels"] > 0:
        return "mixed_proxy"
    return "nonsemantic_background_proxy"


def region_size_stratum(pixels):
    if pixels <= 64:
        return "1_64"
    if pixels <= 255:
        return "65_255"
    return "ge_256"


def comparison_stratum(pixels):
    if pixels <= 0:
        return "0"
    if pixels <= 4:
        return "1_4"
    if pixels <= 19:
        return "5_19"
    return "ge_20"


def summarize_group(rows):
    compared = [row for row in rows if row["comparison_votes"] > 0]
    scores = [row["positive_vote_ratio"] for row in compared]
    comparison_votes = sum(row["comparison_votes"] for row in rows)
    positive_votes = sum(row["positive_votes"] for row in rows)
    return {
        "regions": len(rows),
        "compared_regions": len(compared),
        "comparison_votes": comparison_votes,
        "positive_votes": positive_votes,
        "positive_vote_ratio_weighted": ratio(
            positive_votes, comparison_votes),
        "region_score_quantiles_unweighted": {
            "p10": quantile(scores, 0.10),
            "p25": quantile(scores, 0.25),
            "p50": quantile(scores, 0.50),
            "p75": quantile(scores, 0.75),
            "p90": quantile(scores, 0.90),
            "p95": quantile(scores, 0.95),
        },
    }


def within_frame_proxy_contrast(rows):
    frames = {}
    for row in rows:
        frames.setdefault(row["frame"], []).append(row)
    deltas = []
    frame_records = []
    for frame_id, frame_rows in sorted(frames.items()):
        semantic = [
            row for row in frame_rows
            if proxy_group(row) == "semantic_dominant_proxy"
        ]
        background = [
            row for row in frame_rows
            if proxy_group(row) == "nonsemantic_background_proxy"
        ]
        semantic_comparisons = sum(
            row["comparison_votes"] for row in semantic)
        background_comparisons = sum(
            row["comparison_votes"] for row in background)
        if semantic_comparisons == 0 or background_comparisons == 0:
            continue
        semantic_score = ratio(
            sum(row["positive_votes"] for row in semantic),
            semantic_comparisons)
        background_score = ratio(
            sum(row["positive_votes"] for row in background),
            background_comparisons)
        delta = semantic_score - background_score
        deltas.append(delta)
        frame_records.append({
            "frame": frame_id,
            "semantic_proxy_positive_vote_ratio": semantic_score,
            "background_proxy_positive_vote_ratio": background_score,
            "delta": delta,
        })
    return {
        "eligible_frames": len(frame_records),
        "delta_quantiles": {
            "p10": quantile(deltas, 0.10),
            "p50": quantile(deltas, 0.50),
            "p90": quantile(deltas, 0.90),
        },
        "positive_delta_frame_ratio": ratio(
            sum(delta > 0.0 for delta in deltas), len(deltas)),
        "frames": frame_records,
    }


def static_background_risk_curve(rows):
    background = [
        row for row in rows
        if proxy_group(row) == "nonsemantic_background_proxy"
        and row["comparison_votes"] > 0
    ]
    total_votes = sum(row["comparison_votes"] for row in background)
    return [
        {
            "descriptive_score_threshold_not_a_decision": threshold,
            "region_ratio": ratio(
                sum(row["positive_vote_ratio"] >= threshold
                    for row in background),
                len(background)),
            "comparison_vote_mass_ratio": ratio(
                sum(row["comparison_votes"] for row in background
                    if row["positive_vote_ratio"] >= threshold),
                total_votes),
        }
        for threshold in SCORE_THRESHOLDS
    ]


def summarize_sequence(rows, include_static_curve):
    groups = {}
    for name in (
        "semantic_dominant_proxy",
        "mixed_proxy",
        "nonsemantic_background_proxy",
    ):
        groups[name] = summarize_group(
            [row for row in rows if proxy_group(row) == name])

    strata = {}
    for size_name in ("1_64", "65_255", "ge_256"):
        for comparison_name in ("0", "1_4", "5_19", "ge_20"):
            key = f"region_{size_name}__comparison_{comparison_name}"
            strata[key] = summarize_group([
                row for row in rows
                if region_size_stratum(row["region_pixels"]) == size_name
                and comparison_stratum(row["comparison_pixels"])
                == comparison_name
            ])

    semantic_scores = [
        row["positive_vote_ratio"] for row in rows
        if proxy_group(row) == "semantic_dominant_proxy"
        and row["comparison_votes"] > 0
    ]
    background_scores = [
        row["positive_vote_ratio"] for row in rows
        if proxy_group(row) == "nonsemantic_background_proxy"
        and row["comparison_votes"] > 0
    ]
    output = {
        "frames": len({row["frame"] for row in rows}),
        "regions": len(rows),
        "proxy_groups": groups,
        "support_strata": strata,
        "semantic_vs_background_proxy_auc_not_motion_auc": rank_auc(
            semantic_scores, background_scores),
        "within_frame_proxy_contrast": within_frame_proxy_contrast(rows),
    }
    if include_static_curve:
        output["static_background_risk_curve_not_fpr"] = (
            static_background_risk_curve(rows)
        )
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--walking", required=True)
    parser.add_argument("--sitting", required=True)
    parser.add_argument("--static", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()

    output = {
        "classification_output": "none",
        "dynamic_threshold_selected": False,
        "warnings": [
            "semantic proxy is not motion ground truth",
            "static background curve is a risk proxy, not measured FPR",
            "region rows are correlated within frames",
        ],
        "walking": summarize_sequence(
            load_rows(args.walking), False),
        "sitting": summarize_sequence(
            load_rows(args.sitting), False),
        "fr1_xyz_static_risk_proxy": summarize_sequence(
            load_rows(args.static), True),
    }
    serialized = json.dumps(output, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.write("\n")
    print(serialized)


if __name__ == "__main__":
    main()
