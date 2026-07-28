#!/usr/bin/env python3
"""Audit G2-1 multi-reference evidence histograms without changing SLAM."""

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def read_histogram(path, sampling_policy=None):
    frames = defaultdict(list)
    observed_sampling_policies = set()
    with open(path, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "frame",
            "reference_count",
            "comparison_count",
            "positive_count",
            "pixel_count",
            "semantic_pixel_count",
            "total_ms",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns {sorted(missing)}")
        if sampling_policy and "sampling_policy" not in (reader.fieldnames or []):
            raise ValueError(
                f"{path}: --sampling-policy requires a sampling_policy column"
            )
        for row in reader:
            if "sampling_policy" in row:
                observed_sampling_policies.add(row["sampling_policy"])
            if sampling_policy and row["sampling_policy"] != sampling_policy:
                continue
            parsed = {
                "frame": int(row["frame"]),
                "reference_count": int(row["reference_count"]),
                "comparison_count": int(row["comparison_count"]),
                "positive_count": int(row["positive_count"]),
                "pixel_count": int(row["pixel_count"]),
                "semantic_pixel_count": int(row["semantic_pixel_count"]),
                "total_ms": float(row["total_ms"]),
            }
            if (
                parsed["comparison_count"] < 0
                or parsed["positive_count"] < 0
                or parsed["positive_count"] > parsed["comparison_count"]
                or parsed["comparison_count"] > parsed["reference_count"]
                or parsed["semantic_pixel_count"] > parsed["pixel_count"]
            ):
                raise ValueError(f"{path}: invalid histogram row {row}")
            frames[parsed["frame"]].append(parsed)
    if not sampling_policy and len(observed_sampling_policies) > 1:
        raise ValueError(
            f"{path}: mixed sampling policies "
            f"{sorted(observed_sampling_policies)}; use --sampling-policy"
        )
    if not frames:
        raise ValueError(f"{path}: no histogram rows")
    return frames


def safe_ratio(numerator, denominator):
    return numerator / denominator if denominator else 0.0


def summarize_rule(frames, min_comparisons, min_positive_votes):
    eligible = 0
    semantic_eligible = 0
    selected = 0
    selected_semantic = 0
    all_pixels = 0
    semantic_pixels = 0
    for rows in frames.values():
        for row in rows:
            pixels = row["pixel_count"]
            semantic = row["semantic_pixel_count"]
            all_pixels += pixels
            semantic_pixels += semantic
            if row["comparison_count"] >= min_comparisons:
                eligible += pixels
                semantic_eligible += semantic
                if row["positive_count"] >= min_positive_votes:
                    selected += pixels
                    selected_semantic += semantic

    static_eligible = eligible - semantic_eligible
    selected_static = selected - selected_semantic
    return {
        "min_comparisons": min_comparisons,
        "min_positive_votes": min_positive_votes,
        "all_pixels": all_pixels,
        "semantic_pixels": semantic_pixels,
        "eligible_pixels": eligible,
        "semantic_eligible_pixels": semantic_eligible,
        "selected_pixels": selected,
        "selected_semantic_pixels": selected_semantic,
        "coverage": safe_ratio(eligible, all_pixels),
        "selected_area_fraction": safe_ratio(selected, eligible),
        "proxy_precision": safe_ratio(selected_semantic, selected),
        "conditional_proxy_recall": safe_ratio(
            selected_semantic, semantic_eligible
        ),
        "proxy_background_rate": safe_ratio(
            selected_static, static_eligible
        ),
    }


def runtime_summary(frames):
    values = [rows[0]["total_ms"] for rows in frames.values()]
    return {
        "frames": len(values),
        "mean_ms": statistics.fmean(values),
        "median_ms": statistics.median(values),
        "p95_ms": sorted(values)[
            min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)
        ],
        "minimum_ms": min(values),
        "maximum_ms": max(values),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dynamic", required=True)
    parser.add_argument("--static", required=True)
    parser.add_argument("--low-dynamic")
    parser.add_argument(
        "--sampling-policy",
        help="optional sampling_policy value for mixed-policy histogram CSVs",
    )
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    datasets = {
        "dynamic": read_histogram(args.dynamic, args.sampling_policy),
        "static": read_histogram(args.static, args.sampling_policy),
    }
    if args.low_dynamic:
        datasets["low_dynamic"] = read_histogram(
            args.low_dynamic, args.sampling_policy
        )

    reference_counts = {
        row["reference_count"]
        for frames in datasets.values()
        for rows in frames.values()
        for row in rows
    }
    if len(reference_counts) != 1:
        raise ValueError(
            f"inconsistent reference counts: {sorted(reference_counts)}"
        )
    reference_count = next(iter(reference_counts))

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows_out = []
    for min_comparisons in range(1, reference_count + 1):
        for min_positive_votes in range(1, reference_count + 1):
            for dataset_name, frames in datasets.items():
                result = summarize_rule(
                    frames, min_comparisons, min_positive_votes
                )
                result["dataset"] = dataset_name
                rows_out.append(result)

    csv_path = output_dir / "g2_1_vote_grid.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows_out[0]))
        writer.writeheader()
        writer.writerows(rows_out)

    summary = {
        "scope": "G2-1 shadow-only; YOLO person masks are proxies, not motion ground truth",
        "sampling_policy": args.sampling_policy,
        "reference_count": reference_count,
        "inputs": {
            "dynamic": str(Path(args.dynamic).resolve()),
            "static": str(Path(args.static).resolve()),
            "low_dynamic": (
                str(Path(args.low_dynamic).resolve())
                if args.low_dynamic
                else None
            ),
        },
        "runtime": {
            name: runtime_summary(frames)
            for name, frames in datasets.items()
        },
        "rules": rows_out,
    }
    json_path = output_dir / "g2_1_vote_grid.json"
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(
        json.dumps(
            {
                "reference_count": reference_count,
                "runtime": summary["runtime"],
                "csv": str(csv_path),
                "json": str(json_path),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
