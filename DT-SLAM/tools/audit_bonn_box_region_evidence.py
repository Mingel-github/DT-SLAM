#!/usr/bin/env python3
"""Join exact C++ box/region partitions with online shadow evidence.

This is a development-only diagnostic. Coarse RGB-only bboxes are not pixel
ground truth or object-motion labels, and this tool chooses no threshold.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


INTEGER_PARTITION_FIELDS = (
    "valid_depth_pixels",
    "boundary_pixels",
    "partition_region_count",
)
FLOAT_PARTITION_FIELDS = (
    "largest_region_valid_ratio",
    "top_five_region_valid_ratio",
)
VISIBLE_STATES = {"visible", "partial", "occluded"}


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def integer(row, name):
    return int(row[name])


def finite_float(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}={row[name]!r}")
    return value


def safe_ratio(numerator, denominator):
    return numerator / denominator if denominator else math.nan


def summarize(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "mean": None, "median": None, "minimum": None, "maximum": None}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "minimum": min(finite),
        "maximum": max(finite),
    }


def validate_repeated_online_partition(rows):
    if not rows:
        return
    first = rows[0]
    repeated = INTEGER_PARTITION_FIELDS + FLOAT_PARTITION_FIELDS
    for row in rows[1:]:
        for field in repeated:
            if row[field] != first[field]:
                raise ValueError(
                    f"online partition field varies within frame: {field}"
                )


def validate_exact_partition(frame, online_rows):
    validate_repeated_online_partition(online_rows)
    first = online_rows[0]
    mismatches = []
    for field in INTEGER_PARTITION_FIELDS:
        if integer(frame, field) != integer(first, field):
            mismatches.append(
                (field, integer(frame, field), integer(first, field))
            )
    for field in FLOAT_PARTITION_FIELDS:
        offline = finite_float(frame, field)
        online = finite_float(first, field)
        if abs(offline - online) > 1e-12:
            mismatches.append((field, offline, online))
    if len(online_rows) != integer(frame, "partition_region_count"):
        mismatches.append(
            (
                "region_row_count",
                integer(frame, "partition_region_count"),
                len(online_rows),
            )
        )
    if mismatches:
        raise ValueError(
            f"partition mismatch frame={frame['source_frame']}: {mismatches}"
        )


def aggregate_frame_regions(rows):
    comparison_pixels = sum(integer(row, "comparison_pixels") for row in rows)
    positive_pixels = sum(
        integer(row, "positive_presence_pixels") for row in rows
    )
    comparison_votes = sum(integer(row, "comparison_votes") for row in rows)
    positive_votes = sum(integer(row, "positive_votes") for row in rows)
    return {
        "frame_comparison_pixels": comparison_pixels,
        "frame_positive_presence_pixels": positive_pixels,
        "frame_positive_compared_pixel_ratio": safe_ratio(
            positive_pixels, comparison_pixels
        ),
        "frame_comparison_votes": comparison_votes,
        "frame_positive_votes": positive_votes,
        "frame_positive_vote_ratio": safe_ratio(
            positive_votes, comparison_votes
        ),
    }


def dominant_region_fields(region, intersection):
    comparison_pixels = integer(region, "comparison_pixels")
    positive_pixels = integer(region, "positive_presence_pixels")
    comparison_votes = integer(region, "comparison_votes")
    positive_votes = integer(region, "positive_votes")
    multi_reference_pixels = integer(
        region, "multi_reference_comparison_pixels"
    )
    unanimous_positive_pixels = integer(region, "unanimous_positive_pixels")
    boundary_d2_positive = integer(
        region, "boundary_d2_positive_presence_pixels"
    )
    invalid_d2_positive = integer(
        region, "invalid_d2_positive_presence_pixels"
    )
    return {
        "dominant_region_label": integer(region, "region_label"),
        "dominant_bbox_intersection_pixels": integer(
            intersection, "bbox_intersection_pixels"
        ),
        "dominant_bbox_coverage": finite_float(
            intersection, "bbox_coverage"
        ),
        "dominant_region_fraction_inside_bbox": finite_float(
            intersection, "region_coverage"
        ),
        "dominant_region_pixels": integer(region, "region_pixels"),
        "dominant_comparison_coverage": finite_float(
            region, "comparison_coverage"
        ),
        "dominant_positive_compared_pixel_ratio": safe_ratio(
            positive_pixels, comparison_pixels
        ),
        "dominant_positive_vote_ratio": safe_ratio(
            positive_votes, comparison_votes
        ),
        "dominant_multi_reference_comparison_fraction": safe_ratio(
            multi_reference_pixels, comparison_pixels
        ),
        "dominant_unanimous_positive_fraction": safe_ratio(
            unanimous_positive_pixels, comparison_pixels
        ),
        "dominant_boundary_d2_positive_fraction": safe_ratio(
            boundary_d2_positive, positive_pixels
        ),
        "dominant_invalid_d2_positive_fraction": safe_ratio(
            invalid_d2_positive, positive_pixels
        ),
    }


def process_sequence(workspace, configuration, sampling_policy):
    def resolve(value):
        path = Path(value)
        return path if path.is_absolute() else workspace / path

    frames = read_csv(resolve(configuration["frame_partition_csv"]))
    intersections = read_csv(
        resolve(configuration["box_region_intersection_csv"])
    )
    online_all = read_csv(
        resolve(configuration["online_region_evidence_csv"])
    )
    semantic = read_csv(resolve(configuration["semantic_manifest_csv"]))

    frame_by_id = {}
    for row in frames:
        frame_id = integer(row, "source_frame")
        if frame_id in frame_by_id:
            raise ValueError(f"duplicate offline frame={frame_id}")
        frame_by_id[frame_id] = row

    semantic_by_id = {}
    for row in semantic:
        frame_id = integer(row, "source_frame")
        if frame_id in semantic_by_id:
            raise ValueError(f"duplicate semantic frame={frame_id}")
        semantic_by_id[frame_id] = row
    if set(frame_by_id) != set(semantic_by_id):
        raise ValueError("offline partition and semantic candidate sets differ")

    intersection_by_frame = {}
    for row in intersections:
        frame_id = integer(row, "source_frame")
        label = integer(row, "region_label")
        key = (frame_id, label)
        if any(
            integer(existing, "region_label") == label
            for existing in intersection_by_frame.get(frame_id, [])
        ):
            raise ValueError(f"duplicate box-region intersection {key}")
        intersection_by_frame.setdefault(frame_id, []).append(row)

    online_by_frame = {}
    for row in online_all:
        if row["sampling_policy"] != sampling_policy:
            continue
        frame_id = integer(row, "frame")
        label = integer(row, "region_label")
        if label in online_by_frame.setdefault(frame_id, {}):
            raise ValueError(f"duplicate online region {(frame_id, label)}")
        online_by_frame[frame_id][label] = row

    per_frame = []
    joined = []
    exact_match_count = 0
    missing_online = []
    for frame_id in sorted(frame_by_id):
        frame = frame_by_id[frame_id]
        semantic_row = semantic_by_id[frame_id]
        online_map = online_by_frame.get(frame_id)
        base = {
            "sequence": configuration["name"],
            "export_name": configuration["export_name"],
            "source_frame": frame_id,
            "visibility": frame["visibility"],
            "target_box_present": int(frame["has_box"]),
            "person_detection_count": integer(
                semantic_row, "detection_count"
            ),
            "person_mask_present": int(
                integer(semantic_row, "mask_nonzero_pixels") > 0
            ),
            "online_geometry_available": int(online_map is not None),
            "partition_exact_match": 0,
            "bbox_area": integer(frame, "bbox_area"),
            "bbox_invalid_pixels": integer(frame, "bbox_invalid_pixels"),
            "bbox_boundary_pixels": integer(frame, "bbox_boundary_pixels"),
            "bbox_assigned_region_pixels": integer(
                frame, "bbox_assigned_region_pixels"
            ),
            "bbox_intersecting_region_count": integer(
                frame, "bbox_intersecting_region_count"
            ),
            "bbox_invalid_ratio": safe_ratio(
                integer(frame, "bbox_invalid_pixels"),
                integer(frame, "bbox_area"),
            ),
            "bbox_boundary_ratio": safe_ratio(
                integer(frame, "bbox_boundary_pixels"),
                integer(frame, "bbox_area"),
            ),
            "bbox_assigned_region_ratio": safe_ratio(
                integer(frame, "bbox_assigned_region_pixels"),
                integer(frame, "bbox_area"),
            ),
        }
        nan_fields = {
            "frame_comparison_pixels": "",
            "frame_positive_presence_pixels": "",
            "frame_positive_compared_pixel_ratio": "",
            "frame_comparison_votes": "",
            "frame_positive_votes": "",
            "frame_positive_vote_ratio": "",
            "dominant_region_label": "",
            "dominant_bbox_intersection_pixels": "",
            "dominant_bbox_coverage": "",
            "dominant_region_fraction_inside_bbox": "",
            "dominant_region_pixels": "",
            "dominant_comparison_coverage": "",
            "dominant_positive_compared_pixel_ratio": "",
            "dominant_positive_vote_ratio": "",
            "dominant_multi_reference_comparison_fraction": "",
            "dominant_unanimous_positive_fraction": "",
            "dominant_boundary_d2_positive_fraction": "",
            "dominant_invalid_d2_positive_fraction": "",
            "dominant_minus_frame_positive_ratio": "",
        }
        base.update(nan_fields)

        if online_map is None:
            missing_online.append(frame_id)
            per_frame.append(base)
            continue

        online_rows = list(online_map.values())
        validate_exact_partition(frame, online_rows)
        base["partition_exact_match"] = 1
        exact_match_count += 1
        base.update(aggregate_frame_regions(online_rows))

        frame_intersections = intersection_by_frame.get(frame_id, [])
        for intersection in frame_intersections:
            label = integer(intersection, "region_label")
            if label not in online_map:
                raise ValueError(
                    f"intersecting label missing online {(frame_id, label)}"
                )
            region = online_map[label]
            joined_row = {
                "sequence": configuration["name"],
                "export_name": configuration["export_name"],
                "source_frame": frame_id,
                "visibility": frame["visibility"],
                "person_mask_present": base["person_mask_present"],
                "dominant_bbox_region": 0,
                **intersection,
                **region,
            }
            joined.append(joined_row)

        if frame_intersections:
            dominant = min(
                frame_intersections,
                key=lambda row: (
                    -integer(row, "bbox_intersection_pixels"),
                    integer(row, "region_label"),
                ),
            )
            dominant_region = online_map[
                integer(dominant, "region_label")
            ]
            fields = dominant_region_fields(dominant_region, dominant)
            base.update(fields)
            base["dominant_minus_frame_positive_ratio"] = (
                fields["dominant_positive_compared_pixel_ratio"]
                - base["frame_positive_compared_pixel_ratio"]
            )
            for row in joined:
                if (
                    integer(row, "source_frame") == frame_id
                    and integer(row, "region_label")
                    == fields["dominant_region_label"]
                ):
                    row["dominant_bbox_region"] = 1
                    break
        per_frame.append(base)

    strata = {
        "target_visible_person_absent": lambda row: (
            row["visibility"] in VISIBLE_STATES
            and row["person_mask_present"] == 0
            and row["online_geometry_available"] == 1
        ),
        "target_visible_person_present": lambda row: (
            row["visibility"] in VISIBLE_STATES
            and row["person_mask_present"] == 1
            and row["online_geometry_available"] == 1
        ),
        "target_absent": lambda row: (
            row["visibility"] == "absent"
            and row["online_geometry_available"] == 1
        ),
    }
    metric_names = (
        "frame_positive_compared_pixel_ratio",
        "frame_positive_vote_ratio",
        "dominant_positive_compared_pixel_ratio",
        "dominant_positive_vote_ratio",
        "dominant_comparison_coverage",
        "dominant_multi_reference_comparison_fraction",
        "dominant_unanimous_positive_fraction",
        "dominant_boundary_d2_positive_fraction",
        "dominant_invalid_d2_positive_fraction",
        "dominant_bbox_coverage",
        "dominant_region_fraction_inside_bbox",
        "dominant_minus_frame_positive_ratio",
        "bbox_intersecting_region_count",
        "bbox_assigned_region_ratio",
        "bbox_boundary_ratio",
        "bbox_invalid_ratio",
    )
    stratum_summary = {}
    for name, predicate in strata.items():
        selected = [row for row in per_frame if predicate(row)]
        entry = {"frame_count": len(selected)}
        for metric in metric_names:
            values = []
            for row in selected:
                value = row.get(metric, "")
                if value != "" and math.isfinite(float(value)):
                    values.append(float(value))
            entry[metric] = summarize(values)
        stratum_summary[name] = entry

    summary = {
        "candidate_count": len(per_frame),
        "online_geometry_candidate_count": sum(
            row["online_geometry_available"] for row in per_frame
        ),
        "missing_online_geometry_frames": missing_online,
        "partition_exact_match_count": exact_match_count,
        "partition_mismatch_count": 0,
        "joined_box_region_count": len(joined),
        "strata": stratum_summary,
    }
    return per_frame, joined, summary


def write_csv(path, rows):
    if not rows:
        raise ValueError(f"refusing to write empty CSV: {path}")
    fields = []
    seen = set()
    for row in rows:
        for field in row:
            if field not in seen:
                fields.append(field)
                seen.add(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fields, extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(rows)


def self_test():
    assert safe_ratio(2, 4) == 0.5
    assert math.isnan(safe_ratio(1, 0))
    values = summarize([1.0, 2.0, 3.0])
    assert values["count"] == 3
    assert values["mean"] == 2.0
    assert values["median"] == 2.0
    assert summarize([])["count"] == 0
    print("audit_bonn_box_region_evidence self-test: PASS")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--inputs", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if not args.workspace or not args.inputs or not args.output_dir:
        raise ValueError("--workspace, --inputs and --output-dir are required")
    if args.output_dir.exists():
        raise FileExistsError(
            f"refusing to overwrite output directory: {args.output_dir}"
        )
    configuration = json.loads(args.inputs.read_text(encoding="utf-8"))
    if configuration.get("selection_is_holdout_evaluation") is not False:
        raise ValueError("development audit must explicitly be non-holdout")
    if configuration.get("dynamic_decision") != "none":
        raise ValueError("G2-4E cannot consume a dynamic decision")

    all_frames = []
    all_joined = []
    summaries = {}
    for sequence in configuration["sequences"]:
        per_frame, joined, summary = process_sequence(
            args.workspace,
            sequence,
            configuration["sampling_policy"],
        )
        all_frames.extend(per_frame)
        all_joined.extend(joined)
        summaries[sequence["name"]] = summary

    args.output_dir.mkdir(parents=True, exist_ok=False)
    write_csv(args.output_dir / "per_frame_audit.csv", all_frames)
    write_csv(args.output_dir / "joined_box_regions.csv", all_joined)
    output_summary = {
        "selection_is_holdout_evaluation": False,
        "coarse_bbox_is_pixel_ground_truth": False,
        "visibility_is_motion_ground_truth": False,
        "sampling_policy": configuration["sampling_policy"],
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "sequences": summaries,
    }
    with (args.output_dir / "summary.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(output_summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output_summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
