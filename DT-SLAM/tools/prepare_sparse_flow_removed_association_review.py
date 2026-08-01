#!/usr/bin/env python3
"""Prepare a spatial review of exact G1-F1 association removals.

The box rectangles consumed by this tool are coarse, unverified review
proxies. Consequently, the reported inside/outside counts are diagnostics,
not object-level precision or recall measurements.
"""

import argparse
import collections
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tracking-filter", type=Path, required=True)
    parser.add_argument("--removed-associations", type=Path, required=True)
    parser.add_argument("--box-bboxes", type=Path, required=True)
    parser.add_argument("--export-name", required=True)
    parser.add_argument("--rgb-directory", type=Path, required=True)
    parser.add_argument("--person-mask-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args()


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def point_in_box(x, y, box):
    return (box[0] <= x < box[0] + box[2] and
            box[1] <= y < box[1] + box[3])


def main():
    args = parse_arguments()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    overlay_directory = args.output_directory / "overlays"
    overlay_directory.mkdir(parents=True, exist_ok=True)

    tracking_rows = read_csv(args.tracking_filter)
    removed_rows = read_csv(args.removed_associations)
    box_rows = [
        row for row in read_csv(args.box_bboxes)
        if row["export_name"] == args.export_name
    ]

    tracking_by_frame = {int(row["frame"]): row for row in tracking_rows}
    removed_by_frame = collections.defaultdict(list)
    seen_keys = set()
    duplicate_keys = []
    for row in removed_rows:
        frame = int(row["frame"])
        key = (frame, int(row["feature_index"]))
        if key in seen_keys:
            duplicate_keys.append(key)
        seen_keys.add(key)
        removed_by_frame[frame].append(row)

    per_frame_count_mismatches = []
    for frame, row in tracking_by_frame.items():
        expected = int(row["removed_associations"])
        measured = len(removed_by_frame[frame])
        if expected != measured:
            per_frame_count_mismatches.append({
                "frame": frame,
                "aggregate_removed": expected,
                "feature_rows": measured,
            })
    semantic_overlap_rows = sum(
        int(row["semantic_dynamic"]) != 0 for row in removed_rows)

    review_rows = []
    contact_images = []
    totals = collections.Counter()
    for box_row in box_rows:
        frame = int(box_row["source_frame"])
        box = tuple(int(float(box_row[name]))
                    for name in ("x", "y", "width", "height"))
        rgb_path = args.rgb_directory / "frame_{:06d}.png".format(frame)
        mask_path = (
            args.person_mask_directory /
            "frame_{:06d}.png".format(frame))
        image = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        person_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if image is None or person_mask is None:
            raise FileNotFoundError(
                "missing review RGB or mask for frame {}".format(frame))
        if image.shape[:2] != person_mask.shape[:2]:
            raise ValueError("RGB/mask size mismatch at frame {}".format(frame))

        blue = np.zeros_like(image)
        blue[:, :, 0] = 255
        active = person_mask != 0
        image[active] = np.clip(
            image[active].astype(np.float32) * 0.55 +
            blue[active].astype(np.float32) * 0.45,
            0, 255).astype(np.uint8)
        x, y, width, height = box
        cv2.rectangle(image, (x, y), (x + width, y + height),
                      (0, 255, 0), 2)

        frame_counts = collections.Counter()
        points = removed_by_frame[frame]
        for point in points:
            u = float(point["u"])
            v = float(point["v"])
            px = min(max(int(round(u)), 0), image.shape[1] - 1)
            py = min(max(int(round(v)), 0), image.shape[0] - 1)
            inside_box = point_in_box(u, v, box)
            inside_person = bool(person_mask[py, px] != 0)
            if inside_person:
                category = "person_mask"
            elif inside_box:
                category = "coarse_box_bbox"
            else:
                category = "outside_both"
            frame_counts[category] += 1
            totals[category] += 1
            cv2.circle(image, (px, py), 5, (0, 0, 255), -1,
                       lineType=cv2.LINE_AA)
            cv2.circle(image, (px, py), 7, (255, 255, 255), 1,
                       lineType=cv2.LINE_AA)

        totals["reviewed_points"] += len(points)
        cv2.putText(
            image,
            "frame {} removed={} box={} person={} outside={}".format(
                frame, len(points), frame_counts["coarse_box_bbox"],
                frame_counts["person_mask"], frame_counts["outside_both"]),
            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3,
            cv2.LINE_AA)
        cv2.putText(
            image,
            "frame {} removed={} box={} person={} outside={}".format(
                frame, len(points), frame_counts["coarse_box_bbox"],
                frame_counts["person_mask"], frame_counts["outside_both"]),
            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1,
            cv2.LINE_AA)
        overlay_path = overlay_directory / "frame_{:06d}.png".format(frame)
        if not cv2.imwrite(str(overlay_path), image):
            raise RuntimeError("failed to write {}".format(overlay_path))
        contact_images.append(cv2.resize(image, (320, 240)))
        review_rows.append({
            "frame": frame,
            "removed_points": len(points),
            "inside_coarse_box_bbox": frame_counts["coarse_box_bbox"],
            "inside_person_mask": frame_counts["person_mask"],
            "outside_both": frame_counts["outside_both"],
            "bbox_review_status": box_row["review_status"],
            "bbox_annotation_source": box_row["annotation_source"],
        })

    columns = 4
    rows = []
    for start in range(0, len(contact_images), columns):
        group = contact_images[start:start + columns]
        while len(group) < columns:
            group.append(np.zeros_like(contact_images[0]))
        rows.append(cv2.hconcat(group))
    if rows:
        contact_sheet = cv2.vconcat(rows)
        contact_path = args.output_directory / "contact_sheet.png"
        if not cv2.imwrite(str(contact_path), contact_sheet):
            raise RuntimeError("failed to write {}".format(contact_path))

    review_csv = args.output_directory / "review_counts.csv"
    with review_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(review_rows[0]))
        writer.writeheader()
        writer.writerows(review_rows)

    summary = {
        "scope": "coarse unverified review proxy; not object ground truth",
        "tracking_frame_rows": len(tracking_rows),
        "all_removed_association_rows": len(removed_rows),
        "aggregate_removed_sum": sum(
            int(row["removed_associations"]) for row in tracking_rows),
        "duplicate_frame_feature_keys": len(duplicate_keys),
        "per_frame_count_mismatches": per_frame_count_mismatches,
        "semantic_dynamic_rows": semantic_overlap_rows,
        "review_bbox_frames": len(box_rows),
        "review_bbox_frames_with_removals": sum(
            row["removed_points"] > 0 for row in review_rows),
        "reviewed_removed_points": totals["reviewed_points"],
        "inside_coarse_box_bbox": totals["coarse_box_bbox"],
        "inside_person_mask": totals["person_mask"],
        "outside_both": totals["outside_both"],
    }
    summary_path = args.output_directory / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
