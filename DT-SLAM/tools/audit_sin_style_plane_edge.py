#!/usr/bin/env python3
"""Audit the OpenCV plane-edge substitute as S1 region evidence only."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

from audit_sin_style_native_initial_regions import (
    adjusted_rand_index,
    internal_boundaries,
    normalized_mutual_information,
)


def mean(values):
    return float(np.mean(values)) if values else None


def median(values):
    return float(np.median(values)) if values else None


def boundary_precision_recall(candidate, reference_labels, tolerance):
    reference = internal_boundaries(reference_labels)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * tolerance + 1, 2 * tolerance + 1)
    )
    candidate_dilated = cv2.dilate(candidate, kernel)
    reference_dilated = cv2.dilate(reference, kernel)
    candidate_count = int(np.count_nonzero(candidate))
    reference_count = int(np.count_nonzero(reference))
    precision = (
        float(np.count_nonzero((candidate > 0) & (reference_dilated > 0)))
        / candidate_count
        if candidate_count
        else None
    )
    recall = (
        float(np.count_nonzero((reference > 0) & (candidate_dilated > 0)))
        / reference_count
        if reference_count
        else None
    )
    return precision, recall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--plane-dir", required=True, type=Path)
    parser.add_argument("--rag-dir", required=True, type=Path)
    parser.add_argument("--repeat-plane-dir", type=Path)
    parser.add_argument("--repeat-rag-dir", type=Path)
    parser.add_argument("--reference-dir", type=Path)
    parser.add_argument(
        "--allow-missing-reference-index", action="append", default=[], type=int
    )
    parser.add_argument("--expected-rows", required=True, type=int)
    parser.add_argument("--boundary-tolerance", default=2, type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    errors = []
    if len(rows) != args.expected_rows:
        errors.append(f"row count {len(rows)} != expected {args.expected_rows}")
    if [int(row["input_index"]) for row in rows] != list(range(len(rows))):
        errors.append("input indices are not unique and continuous from zero")

    metric_columns = {
        "plane_coverage": [],
        "raw_plane_boundary_pixels": [],
        "retained_plane_boundary_pixels": [],
        "retained_segment_fraction": [],
        "gradient_components": [],
        "combined_components": [],
        "component_growth_ratio": [],
        "rag_regions": [],
        "plane_extraction_ms": [],
        "plane_total_ms": [],
        "rag_total_ms": [],
    }
    ari_values = []
    nmi_values = []
    boundary_precisions = []
    boundary_recalls = []
    reference_frames = 0
    deterministic_frames = 0
    allowed_missing = set(args.allow_missing_reference_index)

    for row in rows:
        index = int(row["input_index"])
        if int(row["native_plane_enabled"]) != 1:
            errors.append(f"input {index}: plane substitute disabled")
        if int(row["native_plane_available"]) != 1:
            errors.append(f"input {index}: plane substitute unavailable")
        if int(row["native_plane_dynamic_state_available"]) != 0:
            errors.append(f"input {index}: plane stage exposes dynamic state")
        if int(row["native_plane_opencv_substitute"]) != 1:
            errors.append(f"input {index}: substitute identity missing")
        if row["native_plane_dynamic_decision"] != "none":
            errors.append(f"input {index}: plane dynamic decision is not none")
        if row["direct_slam_state_mutation"] != "none" or int(
            row["actual_slam_removed"]
        ) != 0:
            errors.append(f"input {index}: shadow-only invariant failed")
        if int(row["native_rag_plane_rejection_available"]) != 0:
            errors.append(
                f"input {index}: unimplemented pair-level plane rejection reported"
            )
        for column in (
            "native_plane_raw_boundary_written",
            "native_plane_retained_boundary_written",
            "native_plane_combined_edge_written",
            "native_plane_combined_labels_written",
            "native_rag_labels_written",
        ):
            if int(row[column]) != 1:
                errors.append(f"input {index}: {column} is not one")

        raw_path = args.plane_dir / (
            f"frame_{index:06d}_native_plane_raw_boundary.png"
        )
        retained_path = args.plane_dir / (
            f"frame_{index:06d}_native_plane_retained_boundary.png"
        )
        combined_edge_path = args.plane_dir / (
            f"frame_{index:06d}_native_combined_edge.png"
        )
        combined_labels_path = args.plane_dir / (
            f"frame_{index:06d}_native_combined_split_labels.png"
        )
        rag_path = args.rag_dir / f"frame_{index:06d}_native_rag_merged_labels.png"
        raw = cv2.imread(str(raw_path), cv2.IMREAD_UNCHANGED)
        retained = cv2.imread(str(retained_path), cv2.IMREAD_UNCHANGED)
        combined_edge = cv2.imread(str(combined_edge_path), cv2.IMREAD_UNCHANGED)
        combined_labels = cv2.imread(
            str(combined_labels_path), cv2.IMREAD_UNCHANGED
        )
        rag = cv2.imread(str(rag_path), cv2.IMREAD_UNCHANGED)
        if any(image is None for image in (raw, retained, combined_edge, combined_labels, rag)):
            errors.append(f"input {index}: diagnostic image missing")
            continue
        if not (
            raw.shape
            == retained.shape
            == combined_edge.shape
            == combined_labels.shape
            == rag.shape
        ):
            errors.append(f"input {index}: diagnostic image shapes differ")
            continue
        if np.any((retained > 0) & (raw == 0)):
            errors.append(f"input {index}: retained boundary is not a raw subset")
        if np.any((retained > 0) & (combined_edge == 0)):
            errors.append(f"input {index}: retained boundary missing from combined edge")
        if not np.array_equal(combined_labels > 0, rag > 0):
            errors.append(f"input {index}: RAG changed combined core support")

        combined_ids = np.unique(combined_labels[combined_labels > 0])
        rag_ids = np.unique(rag[rag > 0])
        expected_components = int(row["native_plane_combined_component_count"])
        expected_regions = int(row["native_rag_output_region_count"])
        if not np.array_equal(
            combined_ids, np.arange(1, expected_components + 1)
        ):
            errors.append(f"input {index}: combined component IDs are not contiguous")
        if not np.array_equal(rag_ids, np.arange(1, expected_regions + 1)):
            errors.append(f"input {index}: RAG IDs are not contiguous")
        if int(np.count_nonzero(raw)) != int(
            row["native_plane_raw_boundary_pixels"]
        ):
            errors.append(f"input {index}: raw boundary count mismatch")
        if int(np.count_nonzero(retained)) != int(
            row["native_plane_retained_boundary_pixels"]
        ):
            errors.append(f"input {index}: retained boundary count mismatch")

        if args.repeat_plane_dir and args.repeat_rag_dir:
            repeat_images = [
                cv2.imread(str(args.repeat_plane_dir / path.name), cv2.IMREAD_UNCHANGED)
                for path in (raw_path, retained_path, combined_edge_path, combined_labels_path)
            ]
            repeat_rag = cv2.imread(
                str(args.repeat_rag_dir / rag_path.name), cv2.IMREAD_UNCHANGED
            )
            if any(image is None for image in repeat_images) or repeat_rag is None:
                errors.append(f"input {index}: repeated diagnostic image missing")
            elif not all(
                np.array_equal(first, second)
                for first, second in zip(
                    (raw, retained, combined_edge, combined_labels), repeat_images
                )
            ) or not np.array_equal(rag, repeat_rag):
                errors.append(f"input {index}: repeated plane/RAG output differs")
            else:
                deterministic_frames += 1

        if args.reference_dir and index not in allowed_missing:
            reference = cv2.imread(
                str(args.reference_dir / f"frame_{index:06d}_labels.png"),
                cv2.IMREAD_UNCHANGED,
            )
            if reference is None or reference.shape != rag.shape:
                errors.append(f"input {index}: author reference missing/misaligned")
            else:
                overlap = (rag > 0) & (reference > 0)
                if np.any(overlap):
                    ari_values.append(
                        adjusted_rand_index(rag[overlap], reference[overlap])
                    )
                    nmi_values.append(
                        normalized_mutual_information(
                            rag[overlap], reference[overlap]
                        )
                    )
                precision, recall = boundary_precision_recall(
                    combined_edge, reference, args.boundary_tolerance
                )
                if precision is not None:
                    boundary_precisions.append(precision)
                if recall is not None:
                    boundary_recalls.append(recall)
                reference_frames += 1

        depth_valid = int(row["native_plane_input_depth_valid_pixels"])
        plane_pixels = int(row["native_plane_pixels"])
        segment_count = int(row["native_plane_boundary_segment_count"])
        retained_segments = int(row["native_plane_retained_segment_count"])
        gradient_components = int(row["native_gradient_split_component_count"])
        combined_components = int(row["native_plane_combined_component_count"])
        metric_columns["plane_coverage"].append(
            plane_pixels / depth_valid if depth_valid else 0.0
        )
        metric_columns["raw_plane_boundary_pixels"].append(
            int(row["native_plane_raw_boundary_pixels"])
        )
        metric_columns["retained_plane_boundary_pixels"].append(
            int(row["native_plane_retained_boundary_pixels"])
        )
        metric_columns["retained_segment_fraction"].append(
            retained_segments / segment_count if segment_count else 0.0
        )
        metric_columns["gradient_components"].append(gradient_components)
        metric_columns["combined_components"].append(combined_components)
        metric_columns["component_growth_ratio"].append(
            combined_components / gradient_components if gradient_components else 0.0
        )
        metric_columns["rag_regions"].append(expected_regions)
        for source, target in (
            ("native_plane_extraction_ms", "plane_extraction_ms"),
            ("native_plane_total_ms", "plane_total_ms"),
            ("native_rag_total_ms", "rag_total_ms"),
        ):
            value = float(row[source])
            if not math.isfinite(value) or value < 0:
                errors.append(f"input {index}: invalid runtime in {source}")
            metric_columns[target].append(value)

    summary = {
        "identity": (
            "S1 OpenCV RgbdPlane substitute for SIn-style plane-edge region "
            "evidence; no dynamic decision"
        ),
        "frame_rows": len(rows),
        "reference_compared_frames": reference_frames,
        "deterministic_frames": (
            deterministic_frames
            if args.repeat_plane_dir and args.repeat_rag_dir
            else None
        ),
        **{
            f"{name}_mean": mean(values)
            for name, values in metric_columns.items()
        },
        **{
            f"{name}_median": median(values)
            for name, values in metric_columns.items()
        },
        "rag_vs_author_final_ari_mean": mean(ari_values),
        "rag_vs_author_final_nmi_mean": mean(nmi_values),
        "combined_boundary_precision_at_tolerance_mean": mean(
            boundary_precisions
        ),
        "author_boundary_recall_at_tolerance_mean": mean(boundary_recalls),
        "boundary_tolerance_pixels": args.boundary_tolerance,
        "invariant_errors": errors,
        "pass": not errors,
        "interpretation_limit": (
            "OpenCV RgbdPlane is a BSD substitute, not PEAC. Author final labels "
            "are descriptive only. PNG encodes both invalid and boundary as zero. "
            "This stage has no dense flow, dynamic state, mask, or SLAM filtering."
        ),
    }
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
