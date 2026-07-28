#!/usr/bin/env python3
"""Summarize buffered G0-2P SLAM/GT residual diagnostics."""

import argparse
import csv
import json
import math
import pathlib
import statistics
import sys


def percentile(values, fraction):
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def describe(values):
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "min": min(values),
        "max": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_input", type=pathlib.Path)
    parser.add_argument("--json-output", type=pathlib.Path)
    args = parser.parse_args()

    with args.csv_input.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("{} contains no diagnostic rows".format(args.csv_input))

    paired_metrics = [
        "comparisons",
        "compare_coverage",
        "mean_abs_m",
        "positive_ratio",
        "negative_ratio",
        "total_ms",
    ]
    report = {
        "input": str(args.csv_input.resolve()),
        "records": len(rows),
        "first_frame": int(rows[0]["frame"]),
        "last_frame": int(rows[-1]["frame"]),
        "reference_dt_s": describe([float(row["dt_s"]) for row in rows]),
        "metrics": {},
    }
    for metric in paired_metrics:
        slam = [float(row["slam_" + metric]) for row in rows]
        ground_truth = [float(row["gt_" + metric]) for row in rows]
        difference = [
            slam_value - gt_value
            for slam_value, gt_value in zip(slam, ground_truth)
        ]
        report["metrics"][metric] = {
            "slam": describe(slam),
            "ground_truth": describe(ground_truth),
            "slam_minus_ground_truth": describe(difference),
            "ground_truth_lower_count": sum(
                gt_value < slam_value
                for slam_value, gt_value in zip(slam, ground_truth)
            ),
        }

    output = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(output)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("summary failed: {}".format(error), file=sys.stderr)
        sys.exit(2)
