#!/usr/bin/env python3
"""Describe G2-MH1 hypotheses inside coarse RGB-only review boxes.

Boxes and motion labels are evaluation proxies only.  They never enter the
hypothesis computation, and this script never selects a dynamic threshold.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median


METRICS = (
    "local_fit_median_m",
    "local_fit_rms_m",
    "background_fit_median_m",
    "background_fit_rms_m",
    "median_improvement_m",
    "background_to_local_rms_ratio",
    "relative_translation_m",
    "relative_rotation_rad",
    "maximum_image_radius_px",
    "reference_depth_span_m",
    "current_depth_span_m",
    "validation_local_fit_median_m",
    "validation_local_fit_rms_m",
    "validation_background_fit_median_m",
    "validation_background_fit_rms_m",
    "validation_median_improvement_m",
    "validation_background_to_local_rms_ratio",
    "validation_local_better_fraction",
    "global_local_better_fraction",
    "global_median_improvement_m",
)

VALIDATION_METRICS = frozenset(
    field for field in METRICS
    if field.startswith("validation_") or field.startswith("global_")
)


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def tied_auc(positive, negative):
    if not positive or not negative:
        return None
    score = 0.0
    for positive_value in positive:
        for negative_value in negative:
            if positive_value > negative_value:
                score += 1.0
            elif positive_value == negative_value:
                score += 0.5
    return score / (len(positive) * len(negative))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hypotheses", type=Path, required=True)
    parser.add_argument("--boxes", type=Path, required=True)
    parser.add_argument("--export-name", required=True)
    parser.add_argument("--motion-proxy", type=Path, required=True)
    parser.add_argument(
        "--semantic-mode",
        required=True,
        help="Provenance label such as none or exact_online",
    )
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()

    boxes = {}
    for row in read_rows(args.boxes):
        if row.get("export_name") != args.export_name or not row.get("x"):
            continue
        boxes[int(row["source_frame"])] = tuple(
            float(row[field]) for field in ("x", "y", "width", "height")
        )
    labels = {
        int(row["source_frame"]): row["motion_label"]
        for row in read_rows(args.motion_proxy)
    }

    values = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    state_counts = defaultdict(lambda: defaultdict(int))
    for row in read_rows(args.hypotheses):
        frame = int(row["frame"])
        if frame not in boxes or frame not in labels:
            continue
        state_counts[frame][row["evidence_state"]] += 1
        if row["evidence_state"] != "measured":
            continue
        u = float(row["u_current"])
        v = float(row["v_current"])
        x, y, width, height = boxes[frame]
        region = "inside" if x <= u < x + width and y <= v < y + height else "outside"
        for field in METRICS:
            if (field in VALIDATION_METRICS and
                    row["validation_state"] != "measured"):
                continue
            value = float(row[field])
            if math.isfinite(value):
                values[frame][region][field].append(value)

    per_frame = []
    for frame in sorted(set(boxes) & set(labels)):
        record = {
            "frame": frame,
            "motion_label": labels[frame],
            "state_counts": dict(sorted(state_counts[frame].items())),
        }
        for region in ("inside", "outside"):
            record[f"{region}_measured_count"] = len(
                values[frame][region][METRICS[0]]
            )
            for field in METRICS:
                samples = values[frame][region][field]
                record[f"{region}_{field}_median"] = (
                    median(samples) if samples else None
                )
        per_frame.append(record)

    metric_summary = {}
    for field in METRICS:
        inside_by_label = defaultdict(list)
        delta_by_label = defaultdict(list)
        for record in per_frame:
            label = record["motion_label"]
            inside = record[f"inside_{field}_median"]
            outside = record[f"outside_{field}_median"]
            if inside is not None:
                inside_by_label[label].append(inside)
            if inside is not None and outside is not None:
                delta_by_label[label].append(inside - outside)
        moving = inside_by_label.get("moving", [])
        stationary = inside_by_label.get("stationary", [])
        moving_delta = delta_by_label.get("moving", [])
        stationary_delta = delta_by_label.get("stationary", [])
        metric_summary[field] = {
            "moving_inside_frame_count": len(moving),
            "stationary_inside_frame_count": len(stationary),
            "moving_inside_median": median(moving) if moving else None,
            "stationary_inside_median": (
                median(stationary) if stationary else None
            ),
            "moving_gt_stationary_proxy_auc": tied_auc(moving, stationary),
            "moving_inside_minus_outside_median": (
                median(moving_delta) if moving_delta else None
            ),
            "stationary_inside_minus_outside_median": (
                median(stationary_delta) if stationary_delta else None
            ),
            "inside_minus_outside_proxy_auc": tied_auc(
                moving_delta, stationary_delta
            ),
        }

    report = {
        "identity": (
            "G2-MH1 coarse-box/motion-proxy descriptive audit; "
            "not object or motion ground truth"
        ),
        "hypotheses": str(args.hypotheses),
        "boxes": str(args.boxes),
        "motion_proxy": str(args.motion_proxy),
        "semantic_mode": args.semantic_mode,
        "review_frame_count": len(per_frame),
        "motion_label_counts": {
            label: sum(record["motion_label"] == label for record in per_frame)
            for label in sorted({record["motion_label"] for record in per_frame})
        },
        "metric_summary": metric_summary,
        "per_frame": per_frame,
        "dynamic_threshold_selected": False,
        "direct_slam_state_mutation": False,
        "limitations": [
            "coarse boxes are RGB-only unverified spatial proxies",
            "motion labels are RGB-temporal proxies, not ground truth",
            "AUC is development ranking on five moving frames and is not accuracy",
            "semantic_mode must be interpreted as recorded; none can contain person motion",
        ],
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report["metric_summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
