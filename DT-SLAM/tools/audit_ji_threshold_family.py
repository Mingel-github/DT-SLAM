#!/usr/bin/env python3
"""Shadow-only audit of relative Ji cluster threshold adaptations.

The threshold family is an explicit engineering interpretation of Ji et al.'s
text, not a reproduction of an undisclosed author implementation.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


NORMALIZATIONS = ("support_weighted", "cluster_unweighted")
LAMBDA_EXPONENTS = tuple(range(-2, 7))
LAMBDA_VALUES = tuple(2.0 ** (exponent / 2.0) for exponent in LAMBDA_EXPONENTS)


def parse_frame_range(text):
    pieces = text.split(":")
    if len(pieces) != 2:
        raise argparse.ArgumentTypeError("frame range must be START:END")
    start = int(pieces[0])
    end = int(pieces[1])
    if start < 0 or end < start:
        raise argparse.ArgumentTypeError("invalid inclusive frame range")
    return start, end


def finite_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"Non-finite numeric value: {value}")
    return parsed


def safe_ratio(numerator, denominator):
    if denominator <= 0:
        return None
    return float(numerator) / float(denominator)


def load_rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No cluster rows in {path}")
    grouped = defaultdict(list)
    for row in rows:
        grouped[int(row["frame"])].append(row)
    return grouped


def select_frames(grouped, frame_range):
    start, end = frame_range
    selected = {
        frame_id: grouped[frame_id]
        for frame_id in sorted(grouped)
        if start <= frame_id <= end
    }
    expected = set(range(start, end + 1))
    missing = sorted(expected.difference(selected))
    if missing:
        raise ValueError(
            f"Missing {len(missing)} frames in requested range "
            f"{start}:{end}; first missing={missing[0]}"
        )
    return selected


def measured_items(rows, score_field):
    items = []
    for row in rows:
        support = int(row["valid_reprojection_support"])
        if row["evidence_state"] != "measured" or support <= 0:
            continue
        score = finite_float(row[score_field])
        area = int(row["depth_pixels"])
        proxy_overlap = int(row["proxy_overlap_pixels"])
        if score < 0.0 or area < 0 or proxy_overlap < 0:
            raise ValueError("Negative score, area, or proxy overlap")
        if proxy_overlap > area:
            raise ValueError("Proxy overlap exceeds cluster area")
        items.append(
            {
                "score": score,
                "support": support,
                "area": area,
                "proxy_overlap": proxy_overlap,
            }
        )
    return items


def frame_normalizer(items, normalization):
    if not items:
        return None
    if normalization == "support_weighted":
        denominator = sum(item["support"] for item in items)
        if denominator <= 0:
            return None
        return (
            sum(item["score"] * item["support"] for item in items)
            / denominator
        )
    if normalization == "cluster_unweighted":
        return sum(item["score"] for item in items) / len(items)
    raise ValueError(f"Unknown normalization: {normalization}")


def evaluate(grouped, frame_range, score_field, normalization, multiplier):
    frames = select_frames(grouped, frame_range)
    totals = {
        "frames": 0,
        "frames_with_measured_clusters": 0,
        "measured_clusters": 0,
        "selected_clusters": 0,
        "measured_area_pixels": 0,
        "selected_area_pixels": 0,
        "measured_proxy_pixels": 0,
        "selected_proxy_pixels": 0,
    }
    for rows in frames.values():
        totals["frames"] += 1
        items = measured_items(rows, score_field)
        if not items:
            continue
        totals["frames_with_measured_clusters"] += 1
        normalizer = frame_normalizer(items, normalization)
        if normalizer is None:
            continue
        threshold = multiplier * normalizer
        selected = [
            item for item in items if item["score"] > threshold
        ]
        totals["measured_clusters"] += len(items)
        totals["selected_clusters"] += len(selected)
        totals["measured_area_pixels"] += sum(
            item["area"] for item in items
        )
        totals["selected_area_pixels"] += sum(
            item["area"] for item in selected
        )
        totals["measured_proxy_pixels"] += sum(
            item["proxy_overlap"] for item in items
        )
        totals["selected_proxy_pixels"] += sum(
            item["proxy_overlap"] for item in selected
        )

    precision = safe_ratio(
        totals["selected_proxy_pixels"],
        totals["selected_area_pixels"],
    )
    recall = safe_ratio(
        totals["selected_proxy_pixels"],
        totals["measured_proxy_pixels"],
    )
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if precision is not None
        and recall is not None
        and precision + recall > 0.0
        else None
    )
    proxy_prevalence = safe_ratio(
        totals["measured_proxy_pixels"],
        totals["measured_area_pixels"],
    )
    select_all_f1 = (
        2.0 * proxy_prevalence / (1.0 + proxy_prevalence)
        if proxy_prevalence is not None
        else None
    )
    result = dict(totals)
    result.update(
        {
            "frame_start": frame_range[0],
            "frame_end": frame_range[1],
            "score_field": score_field,
            "normalization": normalization,
            "lambda": multiplier,
            "selected_cluster_fraction": safe_ratio(
                totals["selected_clusters"],
                totals["measured_clusters"],
            ),
            "selected_area_fraction": safe_ratio(
                totals["selected_area_pixels"],
                totals["measured_area_pixels"],
            ),
            "proxy_precision": precision,
            "proxy_recall": recall,
            "proxy_f1": f1,
            "proxy_prevalence_in_measured_area": proxy_prevalence,
            "select_all_proxy_f1": select_all_f1,
        }
    )
    return result


def choose_calibration_result(results):
    valid = [result for result in results if result["proxy_f1"] is not None]
    if not valid:
        raise ValueError("Calibration has no measurable proxy F1")
    return max(
        valid,
        key=lambda result: (result["proxy_f1"], result["lambda"]),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dynamic_cluster_metrics_csv", type=Path)
    parser.add_argument("static_cluster_metrics_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--calibration-frames",
        type=parse_frame_range,
        default=(1, 99),
    )
    parser.add_argument(
        "--validation-frames",
        type=parse_frame_range,
        default=(100, 199),
    )
    parser.add_argument(
        "--static-frames",
        type=parse_frame_range,
        default=(1, 199),
    )
    parser.add_argument(
        "--score-field",
        default="mean_squared_error_px2",
        choices=(
            "mean_squared_error_px2",
            "mean_error_px",
            "median_error_px",
            "p90_error_px",
        ),
    )
    args = parser.parse_args()

    dynamic_grouped = load_rows(args.dynamic_cluster_metrics_csv)
    static_grouped = load_rows(args.static_cluster_metrics_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    grid_rows = []
    summary = {
        "dynamic_cluster_metrics_csv": str(
            args.dynamic_cluster_metrics_csv
        ),
        "static_cluster_metrics_csv": str(
            args.static_cluster_metrics_csv
        ),
        "score_field": args.score_field,
        "lambda_definition": "2^(k/2), k=-2..6",
        "lambda_values": LAMBDA_VALUES,
        "calibration_frames": args.calibration_frames,
        "validation_frames": args.validation_frames,
        "static_frames": args.static_frames,
        "selection_rule": (
            "maximum calibration proxy F1; ties choose larger lambda"
        ),
        "normalizations": {},
    }

    for normalization in NORMALIZATIONS:
        calibration_results = []
        for multiplier in LAMBDA_VALUES:
            calibration = evaluate(
                dynamic_grouped,
                args.calibration_frames,
                args.score_field,
                normalization,
                multiplier,
            )
            calibration["split"] = "dynamic_calibration"
            grid_rows.append(calibration)
            calibration_results.append(calibration)

            validation = evaluate(
                dynamic_grouped,
                args.validation_frames,
                args.score_field,
                normalization,
                multiplier,
            )
            validation["split"] = "dynamic_validation"
            grid_rows.append(validation)

            static = evaluate(
                static_grouped,
                args.static_frames,
                args.score_field,
                normalization,
                multiplier,
            )
            static["split"] = "static_validation"
            grid_rows.append(static)

        chosen_calibration = choose_calibration_result(
            calibration_results
        )
        chosen_lambda = chosen_calibration["lambda"]
        chosen_validation = evaluate(
            dynamic_grouped,
            args.validation_frames,
            args.score_field,
            normalization,
            chosen_lambda,
        )
        chosen_static = evaluate(
            static_grouped,
            args.static_frames,
            args.score_field,
            normalization,
            chosen_lambda,
        )
        summary["normalizations"][normalization] = {
            "chosen_lambda": chosen_lambda,
            "calibration": chosen_calibration,
            "dynamic_validation": chosen_validation,
            "static_validation": chosen_static,
        }

    grid_path = args.output_dir / "threshold_grid.csv"
    with grid_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(grid_rows[0].keys())
        )
        writer.writeheader()
        writer.writerows(grid_rows)

    summary_path = args.output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")

    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"[Ji GJ-3A] wrote {grid_path}")
    print(f"[Ji GJ-3A] wrote {summary_path}")


if __name__ == "__main__":
    main()
