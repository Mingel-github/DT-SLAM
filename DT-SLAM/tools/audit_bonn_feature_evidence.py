#!/usr/bin/env python3
"""Audit direct multi-reference evidence at ORB features.

This tool is diagnostic-only. It does not create a dynamic decision and treats
the RGB-only coarse box and visibility fields as development proxies, not GT.
"""

import argparse
import csv
import json
import math
import statistics
import tempfile
from collections import defaultdict
from pathlib import Path


SEQUENCE_BY_EXPORT = {
    "nonobstructing_cpp_person_export": "moving_nonobstructing_box",
    "obstructing_cpp_person_export": "moving_obstructing_box",
}


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_int(row, key):
    return int(row[key])


def optional_int(row, key, default=0):
    value = row.get(key, "")
    return int(value) if value not in ("", None) else default


def load_review_rows(path):
    reviews = {}
    for row in read_csv(path):
        export_name = row["export_name"]
        if export_name not in SEQUENCE_BY_EXPORT:
            raise ValueError("unknown export_name: " + export_name)
        key = (SEQUENCE_BY_EXPORT[export_name], as_int(row, "source_frame"))
        if key in reviews:
            raise ValueError("duplicate review row: " + repr(key))
        reviews[key] = {
            "visibility": row["visibility"],
            "person_present": as_int(row, "person_mask_pixels") > 0,
            "bbox_x": optional_int(row, "bbox_x"),
            "bbox_y": optional_int(row, "bbox_y"),
            "bbox_width": optional_int(row, "bbox_width"),
            "bbox_height": optional_int(row, "bbox_height"),
        }
    return reviews


def inside_bbox(row, review):
    if review["visibility"] == "absent":
        return False
    u = as_int(row, "u_raw")
    v = as_int(row, "v_raw")
    return (
        review["bbox_x"] <= u < review["bbox_x"] + review["bbox_width"]
        and review["bbox_y"] <= v < review["bbox_y"] + review["bbox_height"]
    )


def ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def summarize_values(values):
    values = [value for value in values if value is not None]
    if not values:
        return {
            "count": 0,
            "minimum": None,
            "median": None,
            "mean": None,
            "maximum": None,
        }
    return {
        "count": len(values),
        "minimum": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "maximum": max(values),
    }


def summarize_group(rows):
    fields = [
        "inside_comparison_coverage",
        "outside_comparison_coverage",
        "inside_positive_presence_ratio",
        "outside_positive_presence_ratio",
        "inside_positive_vote_ratio",
        "outside_positive_vote_ratio",
        "positive_presence_enrichment",
        "positive_vote_enrichment",
        "inside_multi_reference_fraction",
        "inside_unanimous_positive_fraction",
        "inside_unique_comparison_cells",
        "inside_unique_positive_cells",
        "inside_max_features_per_native_cell",
        "inside_mappoint_fraction",
        "inside_current_outlier_fraction",
    ]
    result = {"frame_count": len(rows)}
    for field in fields:
        result[field] = summarize_values([row[field] for row in rows])
    return result


def audit(feature_inputs, review_path):
    reviews = load_review_rows(review_path)
    features = defaultdict(list)
    for sequence, path in feature_inputs:
        seen = set()
        for row in read_csv(path):
            frame = as_int(row, "frame")
            feature_index = as_int(row, "feature_index")
            key = (frame, feature_index)
            if key in seen:
                raise ValueError("duplicate feature row: " + repr((sequence,) + key))
            seen.add(key)
            comparisons = as_int(row, "comparison_count")
            positives = as_int(row, "positive_count")
            negatives = as_int(row, "negative_count")
            consistent = as_int(row, "consistent_count")
            if positives + negatives + consistent != comparisons:
                raise ValueError("feature vote conservation failed")
            features[(sequence, frame)].append(row)

    per_frame = []
    missing_feature_frames = []
    for key in sorted(reviews):
        sequence, frame = key
        review = reviews[key]
        frame_features = features.get(key)
        if not frame_features:
            missing_feature_frames.append(
                {"sequence": sequence, "frame": frame}
            )
            continue

        inside = [row for row in frame_features if inside_bbox(row, review)]
        outside = [row for row in frame_features if not inside_bbox(row, review)]

        def metrics(rows):
            comparisons = [row for row in rows if as_int(row, "comparison_count") > 0]
            positive = [row for row in comparisons if as_int(row, "positive_count") > 0]
            total_comparisons = sum(
                as_int(row, "comparison_count") for row in comparisons
            )
            total_positives = sum(
                as_int(row, "positive_count") for row in comparisons
            )
            cells = defaultdict(int)
            positive_cells = set()
            for row in comparisons:
                cell = (
                    as_int(row, "native_scale"),
                    as_int(row, "native_u"),
                    as_int(row, "native_v"),
                )
                cells[cell] += 1
                if as_int(row, "positive_count") > 0:
                    positive_cells.add(cell)
            map_points = [row for row in rows if as_int(row, "has_mappoint") != 0]
            outliers = [
                row
                for row in map_points
                if as_int(row, "current_frame_outlier_flag") != 0
            ]
            return {
                "features": len(rows),
                "compared_features": len(comparisons),
                "positive_features": len(positive),
                "comparison_coverage": ratio(len(comparisons), len(rows)),
                "positive_presence_ratio": ratio(len(positive), len(comparisons)),
                "positive_vote_ratio": ratio(total_positives, total_comparisons),
                "multi_reference_fraction": ratio(
                    sum(as_int(row, "comparison_count") >= 2 for row in comparisons),
                    len(comparisons),
                ),
                "unanimous_positive_fraction": ratio(
                    sum(
                        as_int(row, "comparison_count") > 0
                        and as_int(row, "positive_count")
                        == as_int(row, "comparison_count")
                        for row in comparisons
                    ),
                    len(comparisons),
                ),
                "unique_comparison_cells": len(cells),
                "unique_positive_cells": len(positive_cells),
                "max_features_per_native_cell": max(cells.values(), default=0),
                "mappoint_fraction": ratio(len(map_points), len(rows)),
                "current_outlier_fraction": ratio(len(outliers), len(map_points)),
            }

        inside_metrics = metrics(inside)
        outside_metrics = metrics(outside)
        presence_enrichment = None
        vote_enrichment = None
        if (
            inside_metrics["positive_presence_ratio"] is not None
            and outside_metrics["positive_presence_ratio"] not in (None, 0.0)
        ):
            presence_enrichment = (
                inside_metrics["positive_presence_ratio"]
                / outside_metrics["positive_presence_ratio"]
            )
        if (
            inside_metrics["positive_vote_ratio"] is not None
            and outside_metrics["positive_vote_ratio"] not in (None, 0.0)
        ):
            vote_enrichment = (
                inside_metrics["positive_vote_ratio"]
                / outside_metrics["positive_vote_ratio"]
            )

        record = {
            "sequence": sequence,
            "frame": frame,
            "visibility": review["visibility"],
            "person_present": int(review["person_present"]),
            "total_features": len(frame_features),
            "semantic_nonzero_features": sum(
                as_int(row, "semantic_nonzero") != 0 for row in frame_features
            ),
            "inside_features": inside_metrics["features"],
            "outside_features": outside_metrics["features"],
            "inside_compared_features": inside_metrics["compared_features"],
            "outside_compared_features": outside_metrics["compared_features"],
            "inside_positive_features": inside_metrics["positive_features"],
            "outside_positive_features": outside_metrics["positive_features"],
            "inside_comparison_coverage": inside_metrics["comparison_coverage"],
            "outside_comparison_coverage": outside_metrics["comparison_coverage"],
            "inside_positive_presence_ratio": inside_metrics[
                "positive_presence_ratio"
            ],
            "outside_positive_presence_ratio": outside_metrics[
                "positive_presence_ratio"
            ],
            "inside_positive_vote_ratio": inside_metrics["positive_vote_ratio"],
            "outside_positive_vote_ratio": outside_metrics["positive_vote_ratio"],
            "positive_presence_enrichment": presence_enrichment,
            "positive_vote_enrichment": vote_enrichment,
            "inside_multi_reference_fraction": inside_metrics[
                "multi_reference_fraction"
            ],
            "inside_unanimous_positive_fraction": inside_metrics[
                "unanimous_positive_fraction"
            ],
            "inside_unique_comparison_cells": inside_metrics[
                "unique_comparison_cells"
            ],
            "inside_unique_positive_cells": inside_metrics[
                "unique_positive_cells"
            ],
            "inside_max_features_per_native_cell": inside_metrics[
                "max_features_per_native_cell"
            ],
            "inside_mappoint_fraction": inside_metrics["mappoint_fraction"],
            "inside_current_outlier_fraction": inside_metrics[
                "current_outlier_fraction"
            ],
        }
        per_frame.append(record)

    strata = {}
    for sequence in sorted(set(row["sequence"] for row in per_frame)):
        sequence_rows = [row for row in per_frame if row["sequence"] == sequence]
        strata[sequence] = {
            "all_available": summarize_group(sequence_rows),
            "target_visible_person_absent": summarize_group(
                [
                    row
                    for row in sequence_rows
                    if row["visibility"] != "absent"
                    and not row["person_present"]
                ]
            ),
            "target_visible_person_present": summarize_group(
                [
                    row
                    for row in sequence_rows
                    if row["visibility"] != "absent"
                    and row["person_present"]
                ]
            ),
            "target_absent": summarize_group(
                [row for row in sequence_rows if row["visibility"] == "absent"]
            ),
        }

    summary = {
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "selection_is_holdout_evaluation": False,
        "coarse_bbox_is_pixel_ground_truth": False,
        "visibility_is_motion_ground_truth": False,
        "feature_frame_count": len(per_frame),
        "missing_feature_frames": missing_feature_frames,
        "sequences": strata,
    }
    return per_frame, summary


def write_outputs(per_frame, summary, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    per_frame_path = output_dir / "per_frame_feature_evidence.csv"
    if per_frame:
        with per_frame_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(per_frame[0]))
            writer.writeheader()
            writer.writerows(per_frame)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def self_test():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        review = root / "review.csv"
        review.write_text(
            "export_name,source_frame,visibility,bbox_x,bbox_y,bbox_width,"
            "bbox_height,person_mask_pixels\n"
            "nonobstructing_cpp_person_export,10,visible,0,0,2,2,0\n"
            "nonobstructing_cpp_person_export,11,absent,,,,,0\n",
            encoding="utf-8",
        )
        feature = root / "features.csv"
        header = (
            "frame,timestamp,sampling_policy,feature_index,u_raw,v_raw,octave,"
            "has_mappoint,current_frame_outlier_flag,semantic_nonzero,native_scale,"
            "native_u,native_v,comparison_count,positive_count,negative_count,"
            "consistent_count\n"
        )
        feature.write_text(
            header
            + "10,0,pyramid_dense_s2,0,0,0,0,1,0,0,2,0,0,2,1,0,1\n"
            + "10,0,pyramid_dense_s2,1,1,1,0,1,1,255,2,0,0,2,1,0,1\n"
            + "10,0,pyramid_dense_s2,2,3,3,0,0,0,0,2,1,1,1,0,0,1\n"
            + "11,0,pyramid_dense_s2,0,0,0,0,0,0,0,2,0,0,1,0,0,1\n",
            encoding="utf-8",
        )
        per_frame, summary = audit(
            [("moving_nonobstructing_box", feature)], review
        )
        if len(per_frame) != 2 or summary["missing_feature_frames"]:
            raise AssertionError("self-test frame join failed")
        first = per_frame[0]
        if (
            first["inside_features"] != 2
            or first["inside_unique_comparison_cells"] != 1
            or first["inside_max_features_per_native_cell"] != 2
            or not math.isclose(first["inside_positive_presence_ratio"], 1.0)
            or first["semantic_nonzero_features"] != 1
        ):
            raise AssertionError("self-test feature aggregation failed")
        second = per_frame[1]
        if second["inside_features"] != 0:
            raise AssertionError("absent target must have no inside features")
    print("audit_bonn_feature_evidence self-test: PASS")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--feature-input",
        action="append",
        nargs=2,
        metavar=("SEQUENCE", "CSV"),
        default=[],
    )
    parser.add_argument("--review-csv")
    parser.add_argument("--output-dir")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if not args.feature_input or not args.review_csv or not args.output_dir:
        raise SystemExit(
            "--feature-input, --review-csv and --output-dir are required"
        )
    per_frame, summary = audit(args.feature_input, args.review_csv)
    write_outputs(per_frame, summary, args.output_dir)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
