#!/usr/bin/env python3
"""Audit exact C++ person-filter masks against RGB-only target-box bbox proxies.

The target boxes are development preannotations, not ground truth. This tool
does not read geometry residuals, region labels, scores, or candidate roles.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

import cv2


VISIBLE_STATES = {"visible", "partial", "occluded"}


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def as_int(value, field):
    try:
        return int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"invalid integer {field}={value!r}") from error


def load_export(export_root, export_name):
    directory = export_root / export_name
    manifest = read_csv(directory / "manifest.csv")
    by_frame = {}
    for row in manifest:
        frame = as_int(row["source_frame"], "source_frame")
        if frame in by_frame:
            raise ValueError(f"duplicate manifest source_frame={frame}")
        by_frame[frame] = row
    return directory, by_frame


def validate_bbox(row, width, height):
    visibility = row["visibility"]
    if visibility == "absent":
        if any(row[field].strip() for field in ("x", "y", "width", "height")):
            raise ValueError("absent row must not contain bbox coordinates")
        return None
    if visibility not in VISIBLE_STATES:
        raise ValueError(f"unsupported visibility={visibility!r}")
    x = as_int(row["x"], "x")
    y = as_int(row["y"], "y")
    box_width = as_int(row["width"], "width")
    box_height = as_int(row["height"], "height")
    if (
        x < 0
        or y < 0
        or box_width <= 0
        or box_height <= 0
        or x + box_width > width
        or y + box_height > height
    ):
        raise ValueError(
            f"bbox {(x, y, box_width, box_height)} outside {width}x{height}"
        )
    return x, y, box_width, box_height


def evaluate(preannotations, export_root, output_dir, save_overlays=True):
    exports = {}
    for export_name in sorted({row["export_name"] for row in preannotations}):
        exports[export_name] = load_export(export_root, export_name)

    expected = {
        (export_name, frame)
        for export_name, (_, rows) in exports.items()
        for frame in rows
    }
    provided = set()
    per_frame = []
    overlay_dir = output_dir / "overlays"
    if save_overlays:
        overlay_dir.mkdir(parents=True, exist_ok=False)

    for row in preannotations:
        export_name = row["export_name"]
        frame = as_int(row["source_frame"], "source_frame")
        key = (export_name, frame)
        if key in provided:
            raise ValueError(f"duplicate preannotation {key}")
        provided.add(key)
        if export_name not in exports or frame not in exports[export_name][1]:
            raise ValueError(f"preannotation has no matching C++ export: {key}")

        export_dir, manifest_by_frame = exports[export_name]
        manifest = manifest_by_frame[frame]
        width = as_int(manifest["width"], "width")
        height = as_int(manifest["height"], "height")
        bbox = validate_bbox(row, width, height)
        mask = cv2.imread(
            str(export_dir / manifest["mask_output"]), cv2.IMREAD_UNCHANGED
        )
        rgb = cv2.imread(
            str(export_dir / manifest["rgb_output"]), cv2.IMREAD_COLOR
        )
        if mask is None or mask.shape != (height, width):
            raise ValueError(f"invalid exported mask for {key}")
        if rgb is None or rgb.shape[:2] != (height, width):
            raise ValueError(f"invalid exported RGB for {key}")

        mask_binary = mask != 0
        total_person_pixels = int(mask_binary.sum())
        colored = rgb.copy()
        colored[mask_binary] = (0, 0, 255)
        rgb = cv2.addWeighted(rgb, 0.65, colored, 0.35, 0.0)
        overlap_pixels = 0
        bbox_area = 0
        bbox_coverage = math.nan
        person_fraction_inside_bbox = math.nan
        if bbox is not None:
            x, y, box_width, box_height = bbox
            bbox_area = box_width * box_height
            overlap_pixels = int(
                mask_binary[y : y + box_height, x : x + box_width].sum()
            )
            bbox_coverage = overlap_pixels / bbox_area
            person_fraction_inside_bbox = (
                overlap_pixels / total_person_pixels
                if total_person_pixels
                else 0.0
            )
            if save_overlays:
                cv2.rectangle(
                    rgb,
                    (x, y),
                    (x + box_width - 1, y + box_height - 1),
                    (255, 255, 0),
                    2,
                )
        if save_overlays:
            label = (
                f"target-box proxy: {row['visibility']} "
                f"person/bbox={bbox_coverage:.3f}"
                if bbox is not None
                else "target-box proxy: absent"
            )
            cv2.putText(
                rgb,
                label,
                (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
            output_name = f"{export_name}_frame_{frame:06d}.png"
            if not cv2.imwrite(str(overlay_dir / output_name), rgb):
                raise RuntimeError(f"failed to write overlay {output_name}")

        per_frame.append(
            {
                "export_name": export_name,
                "source_frame": frame,
                "visibility": row["visibility"],
                "annotation_source": row["annotation_source"],
                "review_status": row["review_status"],
                "bbox_x": "" if bbox is None else bbox[0],
                "bbox_y": "" if bbox is None else bbox[1],
                "bbox_width": "" if bbox is None else bbox[2],
                "bbox_height": "" if bbox is None else bbox[3],
                "bbox_area": bbox_area,
                "person_detection_count": as_int(
                    manifest["detection_count"], "detection_count"
                ),
                "person_mask_pixels": total_person_pixels,
                "person_mask_pixels_inside_bbox": overlap_pixels,
                "proxy_bbox_person_coverage": (
                    "" if bbox is None else bbox_coverage
                ),
                "person_mask_fraction_inside_proxy_bbox": (
                    "" if bbox is None else person_fraction_inside_bbox
                ),
                "notes": row["notes"],
            }
        )

    if provided != expected:
        missing = sorted(expected - provided)
        extra = sorted(provided - expected)
        raise ValueError(
            f"preannotation coverage mismatch: missing={missing} extra={extra}"
        )

    summaries = {}
    for export_name in sorted(exports):
        rows = [row for row in per_frame if row["export_name"] == export_name]
        visible = [row for row in rows if row["visibility"] in VISIBLE_STATES]
        values = [float(row["proxy_bbox_person_coverage"]) for row in visible]
        summaries[export_name] = {
            "candidate_count": len(rows),
            "target_box_visible_or_partial_count": len(visible),
            "target_box_absent_count": sum(
                row["visibility"] == "absent" for row in rows
            ),
            "person_detected_candidate_count": sum(
                row["person_detection_count"] > 0 for row in rows
            ),
            "person_detected_when_target_visible_count": sum(
                row["person_detection_count"] > 0 for row in visible
            ),
            "proxy_bbox_person_coverage_mean": statistics.fmean(values),
            "proxy_bbox_person_coverage_median": statistics.median(values),
            "proxy_bbox_person_coverage_max": max(values),
            "visible_bbox_coverage_over_0_01_count": sum(v > 0.01 for v in values),
            "visible_bbox_coverage_over_0_10_count": sum(v > 0.10 for v in values),
            "visible_bbox_coverage_over_0_25_count": sum(v > 0.25 for v in values),
        }

    annotation_sources = sorted(
        {row["annotation_source"] for row in preannotations}
    )
    review_statuses = sorted({row["review_status"] for row in preannotations})
    summary = {
        "annotation_status": "mixed_agent_rgb_only_coarse_bbox_review",
        "annotation_sources": annotation_sources,
        "review_statuses": review_statuses,
        "temporally_corrected_bbox_count": sum(
            row["annotation_source"]
            == "agent_rgb_only_coarse_bbox_v2_temporal_correction"
            for row in preannotations
        ),
        "is_ground_truth": False,
        "selection_is_holdout_evaluation": False,
        "geometry_evidence_used_for_annotation": False,
        "semantic_mask_semantics": "current_cpp_final_person_filter_nonzero",
        "metric_scope": "development_semantic_coverage_review_only",
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "exports": summaries,
    }
    return per_frame, summary


def self_test():
    absent = {
        "visibility": "absent",
        "x": "",
        "y": "",
        "width": "",
        "height": "",
    }
    assert validate_bbox(absent, 640, 480) is None
    visible = {
        "visibility": "visible",
        "x": "10",
        "y": "20",
        "width": "30",
        "height": "40",
    }
    assert validate_bbox(visible, 640, 480) == (10, 20, 30, 40)
    try:
        validate_bbox({**visible, "width": "700"}, 640, 480)
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-bounds bbox was accepted")
    print("audit_bonn_semantic_box_coverage self-test: PASS")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--preannotations", type=Path)
    parser.add_argument("--export-root", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    if not args.preannotations or not args.export_root or not args.output_dir:
        raise ValueError(
            "--preannotations, --export-root and --output-dir are required"
        )
    if args.output_dir.exists():
        raise FileExistsError(
            f"refusing to overwrite output directory: {args.output_dir}"
        )
    preannotations = read_csv(args.preannotations)
    # The audit never modifies the source exports and refuses to overwrite a
    # prior review directory.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    try:
        per_frame, summary = evaluate(
            preannotations, args.export_root, args.output_dir, save_overlays=True
        )
        fields = list(per_frame[0])
        with (args.output_dir / "per_frame_coverage.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(per_frame)
        with (args.output_dir / "summary.json").open(
            "w", encoding="utf-8"
        ) as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except Exception:
        print(
            "output directory may contain partial audit artifacts: "
            f"{args.output_dir}",
            file=sys.stderr,
        )
        raise
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
