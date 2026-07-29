#!/usr/bin/env python3
"""Audit G2-3R1 region evidence without selecting a dynamic threshold."""

import argparse
import csv
import json
import math
import statistics


def load_rows(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def ratio(numerator, denominator):
    return numerator / denominator if denominator else 0.0


def quantile(values, probability):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def sum_int(rows, key):
    return sum(int(row[key]) for row in rows)


def summarize(rows):
    frames = {}
    for row in rows:
        frames.setdefault(int(row["frame"]), row)

    region_pixels = sum_int(rows, "region_pixels")
    comparison_pixels = sum_int(rows, "comparison_pixels")
    positive_pixels = sum_int(rows, "positive_presence_pixels")
    semantic_pixels = sum_int(rows, "semantic_proxy_pixels")
    semantic_comparisons = sum_int(rows, "semantic_comparison_pixels")
    semantic_positives = sum_int(
        rows, "semantic_positive_presence_pixels")
    background_pixels = region_pixels - semantic_pixels
    background_comparisons = comparison_pixels - semantic_comparisons
    background_positives = positive_pixels - semantic_positives

    frame_rows = list(frames.values())
    partition_ms = [float(row["partition_ms"]) for row in frame_rows]
    aggregation_ms = [float(row["aggregation_ms"]) for row in frame_rows]
    return {
        "frames": len(frame_rows),
        "region_rows": len(rows),
        "comparison_coverage": ratio(
            comparison_pixels, region_pixels),
        "positive_given_comparison": ratio(
            positive_pixels, comparison_pixels),
        "semantic_pixels": semantic_pixels,
        "semantic_comparison_coverage": ratio(
            semantic_comparisons, semantic_pixels),
        "semantic_positive_given_comparison": ratio(
            semantic_positives, semantic_comparisons),
        "background_comparison_coverage": ratio(
            background_comparisons, background_pixels),
        "background_positive_given_comparison": ratio(
            background_positives, background_comparisons),
        "partition_ms_mean": statistics.mean(partition_ms),
        "partition_ms_p95": quantile(partition_ms, 0.95),
        "aggregation_ms_mean": statistics.mean(aggregation_ms),
        "aggregation_ms_p95": quantile(aggregation_ms, 0.95),
    }


def rank_auc(positives, negatives):
    if not positives or not negatives:
        return math.nan
    values = [(value, 1) for value in positives]
    values.extend((value, 0) for value in negatives)
    values.sort(key=lambda item: item[0])

    positive_rank_sum = 0.0
    rank = 1
    index = 0
    while index < len(values):
        end = index + 1
        while end < len(values) and values[end][0] == values[index][0]:
            end += 1
        average_rank = (rank + rank + end - index - 1) / 2.0
        positive_rank_sum += average_rank * sum(
            label for _, label in values[index:end])
        rank += end - index
        index = end

    positive_count = len(positives)
    negative_count = len(negatives)
    return (
        positive_rank_sum -
        positive_count * (positive_count + 1) / 2.0
    ) / (positive_count * negative_count)


def select_scores(rows, semantic, minimum_region_pixels,
                  minimum_comparison_pixels):
    scores = []
    for row in rows:
        if int(row["region_pixels"]) < minimum_region_pixels:
            continue
        if int(row["comparison_pixels"]) < minimum_comparison_pixels:
            continue
        semantic_ratio = float(row["semantic_proxy_region_ratio"])
        if semantic:
            if semantic_ratio < 0.5:
                continue
        elif int(row["semantic_proxy_pixels"]) != 0:
            continue
        scores.append(float(row["positive_vote_ratio"]))
    return scores


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--walking", required=True)
    parser.add_argument("--sitting", required=True)
    parser.add_argument("--static", required=True)
    args = parser.parse_args()

    rows = {
        "walking": load_rows(args.walking),
        "sitting": load_rows(args.sitting),
        "static": load_rows(args.static),
    }
    output = {
        name: summarize(sequence_rows)
        for name, sequence_rows in rows.items()
    }

    auc_diagnostics = []
    for minimum_region_pixels, minimum_comparison_pixels in (
        (1, 1), (65, 5), (65, 20), (256, 20)
    ):
        static_scores = select_scores(
            rows["static"], False,
            minimum_region_pixels, minimum_comparison_pixels)
        walking_scores = select_scores(
            rows["walking"], True,
            minimum_region_pixels, minimum_comparison_pixels)
        sitting_scores = select_scores(
            rows["sitting"], True,
            minimum_region_pixels, minimum_comparison_pixels)
        auc_diagnostics.append({
            "minimum_region_pixels": minimum_region_pixels,
            "minimum_comparison_pixels": minimum_comparison_pixels,
            "walking_regions": len(walking_scores),
            "sitting_regions": len(sitting_scores),
            "static_regions": len(static_scores),
            "walking_vs_static_auc": rank_auc(
                walking_scores, static_scores),
            "sitting_vs_static_auc": rank_auc(
                sitting_scores, static_scores),
        })
    output["diagnostic_auc_not_a_method_gate"] = auc_diagnostics
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
