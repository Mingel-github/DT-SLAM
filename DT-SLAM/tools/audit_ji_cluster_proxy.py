#!/usr/bin/env python3
"""Audit Ji cluster reprojection rankings against offline person proxy masks.

The proxy is diagnostic only. It is never fed back into SLAM and is not motion
ground truth.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


TOP_K_VALUES = (1, 3, 5)
ERROR_FIELDS = (
    "mean_squared_error_px2",
    "mean_error_px",
    "median_error_px",
    "p90_error_px",
)


def finite_float(value):
    parsed = float(value)
    if not np.isfinite(parsed):
        raise ValueError(f"Non-finite numeric value: {value}")
    return parsed


def average_ranks(values):
    values = np.asarray(values, dtype=np.float64)
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(values.size, dtype=np.float64)
    start = 0
    while start < values.size:
        end = start + 1
        while end < values.size and values[order[end]] == values[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + end - 1) + 1.0
        start = end
    return ranks


def spearman(values_a, values_b):
    if len(values_a) < 2:
        return None
    ranks_a = average_ranks(values_a)
    ranks_b = average_ranks(values_b)
    if np.std(ranks_a) == 0.0 or np.std(ranks_b) == 0.0:
        return None
    return float(np.corrcoef(ranks_a, ranks_b)[0, 1])


def safe_ratio(numerator, denominator):
    if denominator <= 0:
        return None
    return float(numerator) / float(denominator)


def nearest_rank(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, int(np.ceil(quantile * len(ordered))) - 1)
    return float(ordered[index])


def load_reprojection_rows(path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"No reprojection rows in {path}")
    grouped = defaultdict(list)
    for row in rows:
        grouped[int(row["frame"])].append(row)
    return rows, grouped


def load_mask(path, flags):
    image = cv2.imread(str(path), flags)
    if image is None:
        raise FileNotFoundError(path)
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("reprojection_csv", type=Path)
    parser.add_argument("label_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--proxy-dir", type=Path)
    parser.add_argument("--permutations", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=2021)
    parser.add_argument(
        "--error-field",
        choices=ERROR_FIELDS,
        default="mean_squared_error_px2",
        help=(
            "Cluster score used for ranking. The default matches the "
            "declared identity rho(s)=s baseline in Ji's rho(||e||^2) "
            "formula; the paper does not disclose rho."
        ),
    )
    args = parser.parse_args()
    if args.permutations <= 0:
        raise ValueError("--permutations must be positive")

    original_rows, grouped = load_reprojection_rows(args.reprojection_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    augmented_rows = []
    frame_metrics = []
    global_errors = []
    global_outlier_fractions = []
    global_proxy_fractions = []
    ranking_frames = []

    for frame_id in sorted(grouped):
        rows = grouped[frame_id]
        label_path = (
            args.label_dir / f"frame_{frame_id:06d}_ji_labels_u16.png"
        )
        labels = load_mask(label_path, cv2.IMREAD_UNCHANGED)
        if labels.dtype != np.uint16 or labels.ndim != 2:
            raise ValueError(
                f"{label_path} must be a single-channel uint16 image"
            )

        if args.proxy_dir is None:
            proxy = np.zeros(labels.shape, dtype=np.uint8)
            proxy_available = False
        else:
            proxy_path = args.proxy_dir / f"frame_{frame_id:06d}.png"
            proxy = load_mask(proxy_path, cv2.IMREAD_GRAYSCALE)
            if proxy.shape != labels.shape:
                raise ValueError(
                    f"Proxy/label shape mismatch for frame {frame_id}: "
                    f"{proxy.shape} vs {labels.shape}"
                )
            proxy_available = True
        proxy_binary = proxy != 0
        proxy_pixels = int(np.count_nonzero(proxy_binary))

        measured = []
        for row in rows:
            cluster_id = int(row["cluster_id"])
            cluster_mask = labels == cluster_id + 1
            cluster_pixels = int(np.count_nonzero(cluster_mask))
            csv_depth_pixels = int(row["depth_pixels"])
            if cluster_pixels != csv_depth_pixels:
                raise ValueError(
                    f"Frame {frame_id} cluster {cluster_id}: label pixels "
                    f"{cluster_pixels} != CSV depth_pixels {csv_depth_pixels}"
                )
            overlap_pixels = int(
                np.count_nonzero(cluster_mask & proxy_binary)
            )
            union_pixels = cluster_pixels + proxy_pixels - overlap_pixels
            augmented = dict(row)
            matched_map_support = int(row["matched_map_support"])
            optimizer_outlier_support = int(row["optimizer_outlier_support"])
            optimizer_outlier_fraction = (
                optimizer_outlier_support / matched_map_support
                if matched_map_support
                else 0.0
            )
            augmented.update(
                {
                    "proxy_available": int(proxy_available),
                    "proxy_pixels": proxy_pixels,
                    "proxy_overlap_pixels": overlap_pixels,
                    "proxy_fraction_of_cluster": (
                        overlap_pixels / cluster_pixels
                        if cluster_pixels
                        else 0.0
                    ),
                    "proxy_iou": (
                        overlap_pixels / union_pixels
                        if union_pixels
                        else 0.0
                    ),
                    "proxy_recall_contribution": (
                        overlap_pixels / proxy_pixels
                        if proxy_pixels
                        else 0.0
                    ),
                    "optimizer_outlier_fraction": (
                        optimizer_outlier_fraction
                    ),
                }
            )
            augmented_rows.append(augmented)

            if (
                row["evidence_state"] == "measured"
                and int(row["valid_reprojection_support"]) > 0
            ):
                error = finite_float(row[args.error_field])
                proxy_fraction = augmented["proxy_fraction_of_cluster"]
                item = {
                    "cluster_id": cluster_id,
                    "error": error,
                    "outlier_fraction": optimizer_outlier_fraction,
                    "overlap": overlap_pixels,
                    "pixels": cluster_pixels,
                    "proxy_fraction": proxy_fraction,
                }
                measured.append(item)
                global_errors.append(error)
                global_outlier_fractions.append(optimizer_outlier_fraction)
                global_proxy_fractions.append(proxy_fraction)

        measured.sort(key=lambda item: item["error"], reverse=True)
        measured_proxy_pixels = sum(item["overlap"] for item in measured)
        measured_cluster_pixels = sum(item["pixels"] for item in measured)
        frame_record = {
            "frame": frame_id,
            "initial_pose_available": int(rows[0]["initial_pose_available"]),
            "proxy_available": int(proxy_available),
            "proxy_pixels": proxy_pixels,
            "measured_clusters": len(measured),
            "measured_proxy_pixels": measured_proxy_pixels,
            "proxy_coverage_by_measured_clusters": (
                measured_proxy_pixels / proxy_pixels
                if proxy_pixels
                else 0.0
            ),
            "spearman_error_vs_proxy_fraction": spearman(
                [item["error"] for item in measured],
                [item["proxy_fraction"] for item in measured],
            ),
            "spearman_outlier_fraction_vs_proxy_fraction": spearman(
                [item["outlier_fraction"] for item in measured],
                [item["proxy_fraction"] for item in measured],
            ),
        }

        if measured and measured_proxy_pixels > 0:
            ranking_frames.append(
                {
                    "frame": frame_id,
                    "items": measured,
                    "proxy_denominator": measured_proxy_pixels,
                    "area_denominator": measured_cluster_pixels,
                }
            )
        for top_k in TOP_K_VALUES:
            selected = measured[: min(top_k, len(measured))]
            captured = sum(item["overlap"] for item in selected)
            selected_area = sum(item["pixels"] for item in selected)
            capture_fraction = safe_ratio(
                captured, measured_proxy_pixels
            )
            area_fraction = safe_ratio(
                selected_area, measured_cluster_pixels
            )
            enrichment = (
                capture_fraction / area_fraction
                if capture_fraction is not None
                and area_fraction is not None
                and area_fraction > 0.0
                else None
            )

            random_captures = []
            if measured_proxy_pixels > 0 and measured:
                overlap_values = np.asarray(
                    [item["overlap"] for item in measured],
                    dtype=np.float64,
                )
                selected_count = min(top_k, len(measured))
                for _ in range(args.permutations):
                    indices = rng.permutation(len(measured))[:selected_count]
                    random_captures.append(
                        float(np.sum(overlap_values[indices]))
                        / measured_proxy_pixels
                    )
            random_mean = (
                float(np.mean(random_captures))
                if random_captures
                else None
            )
            empirical_p = (
                (1.0 + sum(value >= capture_fraction for value in random_captures))
                / (1.0 + len(random_captures))
                if random_captures and capture_fraction is not None
                else None
            )
            prefix = f"top{top_k}"
            frame_record[f"{prefix}_proxy_capture"] = capture_fraction
            frame_record[f"{prefix}_area_fraction"] = area_fraction
            frame_record[f"{prefix}_area_enrichment"] = enrichment
            frame_record[f"{prefix}_random_capture_mean"] = random_mean
            frame_record[f"{prefix}_random_empirical_p"] = empirical_p

            outlier_ranked = sorted(
                measured,
                key=lambda item: item["outlier_fraction"],
                reverse=True,
            )
            outlier_selected = outlier_ranked[
                : min(top_k, len(outlier_ranked))
            ]
            outlier_captured = sum(
                item["overlap"] for item in outlier_selected
            )
            outlier_area = sum(
                item["pixels"] for item in outlier_selected
            )
            outlier_capture_fraction = safe_ratio(
                outlier_captured, measured_proxy_pixels
            )
            outlier_area_fraction = safe_ratio(
                outlier_area, measured_cluster_pixels
            )
            frame_record[
                f"{prefix}_outlier_proxy_capture"
            ] = outlier_capture_fraction
            frame_record[
                f"{prefix}_outlier_area_fraction"
            ] = outlier_area_fraction
            frame_record[
                f"{prefix}_outlier_area_enrichment"
            ] = (
                outlier_capture_fraction / outlier_area_fraction
                if outlier_capture_fraction is not None
                and outlier_area_fraction is not None
                and outlier_area_fraction > 0.0
                else None
            )
        frame_metrics.append(frame_record)

    cluster_output = args.output_dir / "cluster_proxy_metrics.csv"
    with cluster_output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(augmented_rows[0].keys())
        )
        writer.writeheader()
        writer.writerows(augmented_rows)

    frame_output = args.output_dir / "frame_ranking_metrics.csv"
    frame_fieldnames = list(frame_metrics[0].keys())
    with frame_output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=frame_fieldnames)
        writer.writeheader()
        writer.writerows(frame_metrics)

    summary = {
        "reprojection_csv": str(args.reprojection_csv),
        "error_field": args.error_field,
        "label_dir": str(args.label_dir),
        "proxy_dir": str(args.proxy_dir) if args.proxy_dir else None,
        "frames": len(grouped),
        "ranking_frames": len(ranking_frames),
        "measured_cluster_rows": len(global_errors),
        "spearman_error_vs_proxy_fraction": spearman(
            global_errors, global_proxy_fractions
        ),
        "spearman_outlier_fraction_vs_proxy_fraction": spearman(
            global_outlier_fractions, global_proxy_fractions
        ),
        "permutations": args.permutations,
        "seed": args.seed,
        "top_k": {},
        "outlier_fraction_baseline_top_k": {},
        "per_frame_stability": {},
    }

    ranked_frame_records = [
        record
        for record in frame_metrics
        if record["measured_proxy_pixels"] > 0
    ]
    frame_spearman_values = [
        record["spearman_error_vs_proxy_fraction"]
        for record in ranked_frame_records
        if record["spearman_error_vs_proxy_fraction"] is not None
    ]
    summary["per_frame_stability"][
        "spearman_error_vs_proxy_fraction"
    ] = {
        "frames": len(frame_spearman_values),
        "positive_frames": sum(
            value > 0.0 for value in frame_spearman_values
        ),
        "median": (
            float(np.median(frame_spearman_values))
            if frame_spearman_values
            else None
        ),
        "minimum": (
            float(np.min(frame_spearman_values))
            if frame_spearman_values
            else None
        ),
        "maximum": (
            float(np.max(frame_spearman_values))
            if frame_spearman_values
            else None
        ),
    }

    measured_reprojection_rows = [
        row
        for row in original_rows
        if row["evidence_state"] == "measured"
        and int(row["valid_reprojection_support"]) > 0
    ]
    measured_errors = [
        finite_float(row["mean_error_px"])
        for row in measured_reprojection_rows
    ]
    valid_support_total = sum(
        int(row["valid_reprojection_support"])
        for row in measured_reprojection_rows
    )
    weighted_error_total = sum(
        finite_float(row["mean_error_px"])
        * int(row["valid_reprojection_support"])
        for row in measured_reprojection_rows
    )
    first_row_by_frame = {
        int(row["frame"]): row for row in original_rows
    }
    initial_pose_frames = [
        row
        for row in first_row_by_frame.values()
        if int(row["initial_pose_available"]) != 0
    ]
    summary["reprojection_statistics"] = {
        "initial_pose_frames": len(initial_pose_frames),
        "valid_reprojection_support": valid_support_total,
        "optimizer_outlier_support": sum(
            int(row["optimizer_outlier_support"])
            for row in original_rows
            if int(row["initial_pose_available"]) != 0
        ),
        "support_weighted_mean_error_px": (
            weighted_error_total / valid_support_total
            if valid_support_total
            else None
        ),
        "cluster_mean_error_median_px": (
            float(np.median(measured_errors))
            if measured_errors
            else None
        ),
        "cluster_mean_error_p90_nearest_rank_px": nearest_rank(
            measured_errors, 0.90
        ),
        "cluster_mean_error_maximum_px": (
            float(np.max(measured_errors))
            if measured_errors
            else None
        ),
        "mean_unknown_clusters_per_initial_pose_frame": (
            float(np.mean([
                int(row["clusters_without_evidence"])
                for row in initial_pose_frames
            ]))
            if initial_pose_frames
            else None
        ),
        "mean_gj2_total_ms": (
            float(np.mean([
                finite_float(row["total_ms"])
                for row in initial_pose_frames
            ]))
            if initial_pose_frames
            else None
        ),
    }

    for top_k in TOP_K_VALUES:
        actual_numerator = 0
        actual_denominator = 0
        actual_area = 0
        area_denominator = 0
        for frame in ranking_frames:
            selected = frame["items"][: min(top_k, len(frame["items"]))]
            actual_numerator += sum(item["overlap"] for item in selected)
            actual_denominator += frame["proxy_denominator"]
            actual_area += sum(item["pixels"] for item in selected)
            area_denominator += frame["area_denominator"]
        actual_capture = safe_ratio(actual_numerator, actual_denominator)
        area_fraction = safe_ratio(actual_area, area_denominator)
        area_enrichment = (
            actual_capture / area_fraction
            if actual_capture is not None
            and area_fraction is not None
            and area_fraction > 0.0
            else None
        )

        random_values = []
        if actual_denominator > 0:
            for _ in range(args.permutations):
                random_numerator = 0
                for frame in ranking_frames:
                    selected_count = min(top_k, len(frame["items"]))
                    indices = rng.permutation(
                        len(frame["items"])
                    )[:selected_count]
                    random_numerator += sum(
                        frame["items"][index]["overlap"]
                        for index in indices
                    )
                random_values.append(
                    random_numerator / actual_denominator
                )
        random_mean = (
            float(np.mean(random_values)) if random_values else None
        )
        empirical_p = (
            (1.0 + sum(value >= actual_capture for value in random_values))
            / (1.0 + len(random_values))
            if random_values and actual_capture is not None
            else None
        )
        summary["top_k"][str(top_k)] = {
            "proxy_capture": actual_capture,
            "area_fraction": area_fraction,
            "area_enrichment": area_enrichment,
            "random_capture_mean": random_mean,
            "random_enrichment": (
                actual_capture / random_mean
                if actual_capture is not None
                and random_mean is not None
                and random_mean > 0.0
                else None
            ),
            "random_empirical_p": empirical_p,
        }

        outlier_numerator = 0
        outlier_area = 0
        for frame in ranking_frames:
            outlier_ranked = sorted(
                frame["items"],
                key=lambda item: item["outlier_fraction"],
                reverse=True,
            )
            selected = outlier_ranked[
                : min(top_k, len(outlier_ranked))
            ]
            outlier_numerator += sum(
                item["overlap"] for item in selected
            )
            outlier_area += sum(item["pixels"] for item in selected)
        outlier_capture = safe_ratio(
            outlier_numerator, actual_denominator
        )
        outlier_area_fraction = safe_ratio(
            outlier_area, area_denominator
        )
        summary["outlier_fraction_baseline_top_k"][str(top_k)] = {
            "proxy_capture": outlier_capture,
            "area_fraction": outlier_area_fraction,
            "area_enrichment": (
                outlier_capture / outlier_area_fraction
                if outlier_capture is not None
                and outlier_area_fraction is not None
                and outlier_area_fraction > 0.0
                else None
            ),
            "relative_capture_vs_error_ranking": (
                outlier_capture / actual_capture
                if outlier_capture is not None
                and actual_capture is not None
                and actual_capture > 0.0
                else None
            ),
        }

        prefix = f"top{top_k}"
        frame_enrichments = [
            record[f"{prefix}_area_enrichment"]
            for record in ranked_frame_records
            if record[f"{prefix}_area_enrichment"] is not None
        ]
        capture_contributions = [
            record[f"{prefix}_proxy_capture"]
            * record["measured_proxy_pixels"]
            for record in ranked_frame_records
            if record[f"{prefix}_proxy_capture"] is not None
        ]
        total_capture_contribution = sum(capture_contributions)
        summary["per_frame_stability"][prefix] = {
            "frames": len(frame_enrichments),
            "frames_enrichment_gt_one": sum(
                value > 1.0 for value in frame_enrichments
            ),
            "median_area_enrichment": (
                float(np.median(frame_enrichments))
                if frame_enrichments
                else None
            ),
            "minimum_area_enrichment": (
                float(np.min(frame_enrichments))
                if frame_enrichments
                else None
            ),
            "maximum_area_enrichment": (
                float(np.max(frame_enrichments))
                if frame_enrichments
                else None
            ),
            "largest_frame_share_of_captured_proxy": (
                max(capture_contributions)
                / total_capture_contribution
                if total_capture_contribution > 0.0
                else None
            ),
        }

    summary_path = args.output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"[Ji GJ-2A] wrote {cluster_output}")
    print(f"[Ji GJ-2A] wrote {frame_output}")
    print(f"[Ji GJ-2A] wrote {summary_path}")


if __name__ == "__main__":
    main()
