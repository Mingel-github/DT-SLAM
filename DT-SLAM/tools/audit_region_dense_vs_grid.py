#!/usr/bin/env python3
"""Audit paired G2-3R2 dense and grid evidence on identical regions."""

import argparse
import csv
import json
import statistics


def load_policy_rows(path, policy):
    rows = {}
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["sampling_policy"] != policy:
                continue
            key = (int(row["frame"]), int(row["region_label"]))
            if key in rows:
                raise ValueError(
                    f"duplicate row for policy={policy}, key={key}")
            rows[key] = row
    return rows


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
    return (
        ordered[lower] * (1.0 - fraction) +
        ordered[upper] * fraction
    )


def unique_frame_values(rows, key):
    values = {}
    for row in rows.values():
        frame = int(row["frame"])
        value = float(row[key])
        if frame in values and values[frame] != value:
            raise ValueError(
                f"inconsistent {key} within frame {frame}")
        values[frame] = value
    return list(values.values())


def load_frame_timings(path, policies):
    timings = {policy: {} for policy in policies}
    preprocess_timings = {policy: {} for policy in policies}
    expand_timings = {policy: {} for policy in policies}
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            policy = row["sampling_policy"]
            if policy not in timings:
                continue
            frame = int(row["frame"])
            total_ms = float(row["total_ms"])
            if (
                frame in timings[policy]
                and timings[policy][frame] != total_ms
            ):
                raise ValueError(
                    f"inconsistent total_ms for {policy}, frame={frame}")
            timings[policy][frame] = total_ms
            if "preprocess_ms" in row:
                preprocess_timings[policy][frame] = float(
                    row["preprocess_ms"])
            if "expand_ms" in row:
                expand_timings[policy][frame] = float(
                    row["expand_ms"])
    output = {}
    for policy, frame_values in timings.items():
        values = list(frame_values.values())
        if not values:
            raise ValueError(
                f"no timing rows for policy={policy}")
        output[policy] = {
            "frames": len(values),
            "mean_ms": statistics.mean(values),
            "p95_ms": quantile(values, 0.95),
        }
        preprocess_values = list(
            preprocess_timings[policy].values())
        expand_values = list(expand_timings[policy].values())
        if preprocess_values:
            output[policy]["preprocess_mean_ms"] = (
                statistics.mean(preprocess_values))
            output[policy]["preprocess_p95_ms"] = quantile(
                preprocess_values, 0.95)
        if expand_values:
            output[policy]["expand_mean_ms"] = statistics.mean(
                expand_values)
            output[policy]["expand_p95_ms"] = quantile(
                expand_values, 0.95)
    return output


def load_same_pixel_audit(path, candidate_policy):
    totals = {
        "sampled_comparison_pixels": 0,
        "dense_comparison_on_sampled_pixels": 0,
        "sampled_positive_presence_pixels": 0,
        "dense_positive_on_sampled_pixels": 0,
        "both_positive_pixels": 0,
        "positive_presence_agreement_pixels": 0,
        "exact_vote_agreement_pixels": 0,
    }
    frames = 0
    with open(path, newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["sampling_policy"] != candidate_policy:
                continue
            if int(row["dense_audit_computed"]) != 1:
                continue
            frames += 1
            for field in totals:
                totals[field] += int(row[field])

    comparisons = totals["sampled_comparison_pixels"]
    candidate_positives = totals[
        "sampled_positive_presence_pixels"]
    dense_positives = totals[
        "dense_positive_on_sampled_pixels"]
    both_positives = totals["both_positive_pixels"]
    return {
        "frames": frames,
        **totals,
        "dense_comparison_on_candidate_ratio": ratio(
            totals["dense_comparison_on_sampled_pixels"],
            comparisons),
        "candidate_positive_supported_by_dense_ratio": ratio(
            both_positives, candidate_positives),
        "dense_positive_recovered_on_candidate_domain_ratio": ratio(
            both_positives, dense_positives),
        "positive_presence_agreement_ratio": ratio(
            totals["positive_presence_agreement_pixels"],
            comparisons),
        "exact_vote_agreement_ratio": ratio(
            totals["exact_vote_agreement_pixels"],
            comparisons),
    }


def summarize(path, grid_policy, dense_policy, multiref_csv=None,
              selection_csv=None):
    grid = load_policy_rows(path, grid_policy)
    dense = load_policy_rows(path, dense_policy)
    grid_keys = set(grid)
    dense_keys = set(dense)
    paired_keys = sorted(grid_keys & dense_keys)
    if not paired_keys:
        raise ValueError("no paired grid/dense region rows")

    mismatched_region_structure = 0
    for key in paired_keys:
        for field in (
            "region_pixels",
            "semantic_proxy_pixels",
            "valid_depth_pixels",
            "boundary_pixels",
            "partition_region_count",
        ):
            if grid[key][field] != dense[key][field]:
                mismatched_region_structure += 1
                break
    if mismatched_region_structure:
        raise ValueError(
            "grid and dense rows do not share identical region structure: "
            f"{mismatched_region_structure} mismatches")

    def sum_field(rows, field):
        return sum(int(rows[key][field]) for key in paired_keys)

    region_pixels = sum_field(grid, "region_pixels")
    semantic_pixels = sum_field(grid, "semantic_proxy_pixels")
    grid_comparison = sum_field(grid, "comparison_pixels")
    dense_comparison = sum_field(dense, "comparison_pixels")
    grid_positive = sum_field(grid, "positive_presence_pixels")
    dense_positive = sum_field(dense, "positive_presence_pixels")
    grid_semantic_comparison = sum_field(
        grid, "semantic_comparison_pixels")
    dense_semantic_comparison = sum_field(
        dense, "semantic_comparison_pixels")
    grid_semantic_positive = sum_field(
        grid, "semantic_positive_presence_pixels")
    dense_semantic_positive = sum_field(
        dense, "semantic_positive_presence_pixels")
    background_pixels = region_pixels - semantic_pixels
    grid_background_comparison = (
        grid_comparison - grid_semantic_comparison)
    dense_background_comparison = (
        dense_comparison - dense_semantic_comparison)
    grid_background_positive = (
        grid_positive - grid_semantic_positive)
    dense_background_positive = (
        dense_positive - dense_semantic_positive)

    dense_supported = [
        key for key in paired_keys
        if int(dense[key]["comparison_pixels"]) > 0
    ]
    dense_positive_supported = [
        key for key in paired_keys
        if int(dense[key]["positive_presence_pixels"]) > 0
    ]
    dense_supported_grid_missing = sum(
        int(grid[key]["comparison_pixels"]) == 0
        for key in dense_supported
    )
    dense_positive_grid_missing = sum(
        int(grid[key]["positive_presence_pixels"]) == 0
        for key in dense_positive_supported
    )

    coverage_gaps = [
        float(dense[key]["comparison_coverage"]) -
        float(grid[key]["comparison_coverage"])
        for key in paired_keys
    ]
    vote_ratio_differences = [
        abs(
            float(dense[key]["positive_vote_ratio"]) -
            float(grid[key]["positive_vote_ratio"])
        )
        for key in paired_keys
        if int(dense[key]["comparison_votes"]) > 0
        and int(grid[key]["comparison_votes"]) > 0
    ]

    grid_aggregation_ms = unique_frame_values(
        grid, "aggregation_ms")
    dense_aggregation_ms = unique_frame_values(
        dense, "aggregation_ms")
    partition_ms = unique_frame_values(grid, "partition_ms")

    support_thresholds = []
    for minimum_comparisons in (1, 5, 20, 50):
        dense_regions = [
            key for key in paired_keys
            if int(dense[key]["comparison_pixels"]) >=
            minimum_comparisons
        ]
        grid_retained = sum(
            int(grid[key]["comparison_pixels"]) >=
            minimum_comparisons
            for key in dense_regions
        )
        support_thresholds.append({
            "minimum_comparison_pixels": minimum_comparisons,
            "dense_regions": len(dense_regions),
            "grid_regions_at_same_minimum": grid_retained,
            "region_support_retention": ratio(
                grid_retained, len(dense_regions)),
        })

    frames = {key[0] for key in paired_keys}
    output = {
        "input_csv": path,
        "grid_policy": grid_policy,
        "dense_policy": dense_policy,
        "paired_frames": len(frames),
        "paired_region_rows": len(paired_keys),
        "grid_only_rows": len(grid_keys - dense_keys),
        "dense_only_rows": len(dense_keys - grid_keys),
        "region_structure_mismatches": 0,
        "region_pixels": region_pixels,
        "grid_comparison_coverage": ratio(
            grid_comparison, region_pixels),
        "dense_comparison_coverage": ratio(
            dense_comparison, region_pixels),
        "grid_dense_comparison_pixel_retention": ratio(
            grid_comparison, dense_comparison),
        "grid_positive_pixels": grid_positive,
        "dense_positive_pixels": dense_positive,
        "grid_dense_positive_pixel_count_retention": ratio(
            grid_positive, dense_positive),
        "grid_positive_given_comparison": ratio(
            grid_positive, grid_comparison),
        "dense_positive_given_comparison": ratio(
            dense_positive, dense_comparison),
        "semantic_proxy_pixels": semantic_pixels,
        "grid_semantic_comparison_coverage": ratio(
            grid_semantic_comparison, semantic_pixels),
        "dense_semantic_comparison_coverage": ratio(
            dense_semantic_comparison, semantic_pixels),
        "grid_dense_semantic_comparison_retention": ratio(
            grid_semantic_comparison, dense_semantic_comparison),
        "grid_semantic_positive_pixels": grid_semantic_positive,
        "dense_semantic_positive_pixels": dense_semantic_positive,
        "grid_dense_semantic_positive_count_retention": ratio(
            grid_semantic_positive, dense_semantic_positive),
        "grid_semantic_positive_given_comparison": ratio(
            grid_semantic_positive, grid_semantic_comparison),
        "dense_semantic_positive_given_comparison": ratio(
            dense_semantic_positive, dense_semantic_comparison),
        "grid_background_comparison_coverage": ratio(
            grid_background_comparison, background_pixels),
        "dense_background_comparison_coverage": ratio(
            dense_background_comparison, background_pixels),
        "grid_background_positive_given_comparison": ratio(
            grid_background_positive, grid_background_comparison),
        "dense_background_positive_given_comparison": ratio(
            dense_background_positive, dense_background_comparison),
        "dense_supported_regions": len(dense_supported),
        "dense_supported_but_grid_missing_regions":
            dense_supported_grid_missing,
        "dense_supported_but_grid_missing_ratio": ratio(
            dense_supported_grid_missing, len(dense_supported)),
        "dense_positive_regions": len(dense_positive_supported),
        "dense_positive_but_grid_missing_regions":
            dense_positive_grid_missing,
        "dense_positive_but_grid_missing_ratio": ratio(
            dense_positive_grid_missing,
            len(dense_positive_supported)),
        "dense_minus_grid_region_coverage_gap_mean":
            statistics.mean(coverage_gaps),
        "dense_minus_grid_region_coverage_gap_p50":
            quantile(coverage_gaps, 0.50),
        "dense_minus_grid_region_coverage_gap_p95":
            quantile(coverage_gaps, 0.95),
        "positive_vote_ratio_abs_difference_mean":
            statistics.mean(vote_ratio_differences)
            if vote_ratio_differences else 0.0,
        "positive_vote_ratio_abs_difference_p95":
            quantile(vote_ratio_differences, 0.95),
        "partition_ms_mean": statistics.mean(partition_ms),
        "grid_aggregation_ms_mean":
            statistics.mean(grid_aggregation_ms),
        "dense_aggregation_ms_mean":
            statistics.mean(dense_aggregation_ms),
        "support_thresholds": support_thresholds,
        "metric_warning":
            "count retention is not motion precision or recall",
    }
    if multiref_csv:
        output["multi_reference_timing"] = load_frame_timings(
            multiref_csv, (grid_policy, dense_policy))
    if selection_csv:
        output["same_pixel_dense_audit"] = load_same_pixel_audit(
            selection_csv, grid_policy)
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--region-csv", required=True)
    parser.add_argument("--grid-policy", default="grid_depth_s4")
    parser.add_argument(
        "--dense-policy", default="dense_same_reference_audit")
    parser.add_argument("--multiref-csv")
    parser.add_argument("--selection-csv")
    args = parser.parse_args()
    print(json.dumps(
        summarize(
            args.region_csv,
            args.grid_policy,
            args.dense_policy,
            args.multiref_csv,
            args.selection_csv),
        indent=2,
        sort_keys=True))


if __name__ == "__main__":
    main()
