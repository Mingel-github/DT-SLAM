#!/usr/bin/env python3
"""Audit the G1 sparse-flow evidence funnel on coarse review boxes."""

import argparse
import collections
import csv
import json
from pathlib import Path

import cv2
import numpy as np


STAGES = (
    "orb_features",
    "measured",
    "quality_eligible",
    "q_candidate",
    "q_candidate_initial_association",
    "q_candidate_post_search_association",
    "removed_association",
)


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sparse-flow-features", type=Path, required=True)
    parser.add_argument("--tracking-filter", type=Path, required=True)
    parser.add_argument("--candidate-associations", type=Path, required=True)
    parser.add_argument("--removed-associations", type=Path, required=True)
    parser.add_argument("--box-bboxes", type=Path, required=True)
    parser.add_argument("--export-name", required=True)
    parser.add_argument("--rgb-directory", type=Path, required=True)
    parser.add_argument("--person-mask-directory", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    return parser.parse_args()


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def key(row):
    return int(row["frame"]), int(row["feature_index"])


def inside_box(row, box):
    u = float(row.get("u_current", row.get("u")))
    v = float(row.get("v_current", row.get("v")))
    x, y, width, height = box
    return x <= u < x + width and y <= v < y + height


def stage_counts(rows, box):
    counts = collections.Counter()
    for row in rows:
        counts["orb_features"] += 1
        if inside_box(row, box):
            counts["orb_features_inside"] += 1
        measured = row["evidence_state"] == "measured"
        eligible = row["quality_eligible"] == "1"
        candidate = row["q_candidate"] == "1"
        initial = candidate and row["has_mappoint"] == "1"
        for name, active in (
                ("measured", measured),
                ("quality_eligible", eligible),
                ("q_candidate", candidate),
                ("q_candidate_initial_association", initial)):
            if active:
                counts[name] += 1
                if inside_box(row, box):
                    counts[name + "_inside"] += 1
    return counts


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)

    def percentile(fraction):
        index = int(round((len(ordered) - 1) * fraction))
        return ordered[index]

    return {
        "count": len(ordered),
        "median": percentile(0.50),
        "p90": percentile(0.90),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "maximum": ordered[-1],
        "count_q_ge_3": sum(value >= 3.0 for value in ordered),
        "count_q_ge_5": sum(value >= 5.0 for value in ordered),
        "count_q_ge_10": sum(value >= 10.0 for value in ordered),
    }


def main():
    args = arguments()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    overlay_directory = args.output_directory / "overlays"
    overlay_directory.mkdir(parents=True, exist_ok=True)

    feature_rows = read_csv(args.sparse_flow_features)
    tracking_rows = read_csv(args.tracking_filter)
    candidate_rows = read_csv(args.candidate_associations)
    removed_rows = read_csv(args.removed_associations)
    boxes = [row for row in read_csv(args.box_bboxes)
             if row["export_name"] == args.export_name]

    tracking = {int(row["frame"]): row for row in tracking_rows}
    features = collections.defaultdict(list)
    candidates = collections.defaultdict(list)
    removed = collections.defaultdict(list)
    for row in feature_rows:
        features[int(row["frame"])].append(row)
    for row in candidate_rows:
        candidates[int(row["frame"])].append(row)
    for row in removed_rows:
        removed[int(row["frame"])].append(row)

    selected_frames = {int(row["source_frame"]) for row in boxes}
    violations = []
    for frame in selected_frames:
        aggregate = tracking.get(frame)
        if aggregate is None:
            violations.append("missing tracking row frame {}".format(frame))
            continue
        rows = features.get(frame, [])
        if not rows:
            violations.append("missing feature rows frame {}".format(frame))
        eligible_count = sum(row["quality_eligible"] == "1" for row in rows)
        q_count = sum(row["q_candidate"] == "1" for row in rows)
        post_count = len(candidates.get(frame, []))
        removed_count = len(removed.get(frame, []))
        checks = (
            ("eligible", eligible_count,
             int(aggregate["quality_eligible_features"])),
            ("q_candidate", q_count,
             int(aggregate["candidate_features"])),
            ("post_search_association", post_count,
             int(aggregate["candidate_associations"])),
            ("removed", removed_count,
             int(aggregate["removed_associations"])),
        )
        for name, measured, expected in checks:
            if measured != expected:
                violations.append(
                    "frame {} {} {} != {}".format(
                        frame, name, measured, expected))

    candidate_keys = [key(row) for row in candidate_rows]
    removed_keys = [key(row) for row in removed_rows]
    candidate_removed_keys = [
        key(row) for row in candidate_rows if row["removed"] == "1"]
    if len(candidate_keys) != len(set(candidate_keys)):
        violations.append("duplicate candidate association frame/feature key")
    if len(removed_keys) != len(set(removed_keys)):
        violations.append("duplicate removed association frame/feature key")
    if set(candidate_removed_keys) != set(removed_keys):
        violations.append("candidate removed flags do not match exact removals")

    totals = collections.Counter()
    q_inside = []
    q_outside = []
    per_frame = []
    contact_images = []
    for box_row in boxes:
        frame = int(box_row["source_frame"])
        box = tuple(int(float(box_row[name]))
                    for name in ("x", "y", "width", "height"))
        frame_features = features[frame]
        counts = stage_counts(frame_features, box)
        for row in frame_features:
            if (row["quality_eligible"] != "1" or
                    row["semantic_nonzero"] != "0" or
                    not row["normalized_residual_q"]):
                continue
            target = q_inside if inside_box(row, box) else q_outside
            target.append(float(row["normalized_residual_q"]))
        for row in candidates[frame]:
            counts["q_candidate_post_search_association"] += 1
            if inside_box(row, box):
                counts["q_candidate_post_search_association_inside"] += 1
        for row in removed[frame]:
            counts["removed_association"] += 1
            if inside_box(row, box):
                counts["removed_association_inside"] += 1
        for name in STAGES:
            totals[name] += counts[name]
            totals[name + "_inside"] += counts[name + "_inside"]

        output_row = {"frame": frame}
        for name in STAGES:
            output_row[name] = counts[name]
            output_row[name + "_inside_box"] = counts[name + "_inside"]
        output_row["filter_state"] = tracking[frame]["state"]
        per_frame.append(output_row)

        image = cv2.imread(
            str(args.rgb_directory / "frame_{:06d}.png".format(frame)),
            cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError("missing RGB frame {}".format(frame))
        if args.person_mask_directory is None:
            person = np.zeros(image.shape[:2], dtype=np.uint8)
        else:
            person = cv2.imread(
                str(args.person_mask_directory /
                    "frame_{:06d}.png".format(frame)),
                cv2.IMREAD_GRAYSCALE)
            if person is None:
                raise FileNotFoundError(
                    "missing person mask frame {}".format(frame))
        active = person != 0
        tint = np.zeros_like(image)
        tint[:, :, 0] = 255
        image[active] = np.clip(
            image[active].astype(np.float32) * 0.55 +
            tint[active].astype(np.float32) * 0.45,
            0, 255).astype(np.uint8)
        x, y, width, height = box
        cv2.rectangle(image, (x, y), (x + width, y + height),
                      (0, 255, 0), 2)

        feature_by_key = {key(row): row for row in frame_features}
        for row in frame_features:
            if row["q_candidate"] != "1":
                continue
            point = (int(round(float(row["u_current"]))),
                     int(round(float(row["v_current"]))))
            cv2.circle(image, point, 3, (255, 0, 255), -1,
                       lineType=cv2.LINE_AA)
        for row in candidates[frame]:
            source = feature_by_key.get(key(row))
            if source is None:
                continue
            point = (int(round(float(source["u_current"]))),
                     int(round(float(source["v_current"]))))
            cv2.circle(image, point, 6, (0, 165, 255), 2,
                       lineType=cv2.LINE_AA)
        for row in removed[frame]:
            point = (int(round(float(row["u"]))),
                     int(round(float(row["v"]))))
            cv2.circle(image, point, 5, (0, 0, 255), -1,
                       lineType=cv2.LINE_AA)
            cv2.circle(image, point, 7, (255, 255, 255), 1,
                       lineType=cv2.LINE_AA)
        label = "f{} box ORB={} M={} Q={} A={} R={}".format(
            frame, counts["orb_features_inside"],
            counts["measured_inside"], counts["q_candidate_inside"],
            counts["q_candidate_post_search_association_inside"],
            counts["removed_association_inside"])
        cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)
        overlay = overlay_directory / "frame_{:06d}.png".format(frame)
        if not cv2.imwrite(str(overlay), image):
            raise RuntimeError("failed to write {}".format(overlay))
        contact_images.append(cv2.resize(image, (320, 240)))

    per_frame_path = args.output_directory / "funnel_per_frame.csv"
    with per_frame_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(per_frame[0]))
        writer.writeheader()
        writer.writerows(per_frame)

    contact_rows = []
    for start in range(0, len(contact_images), 4):
        group = contact_images[start:start + 4]
        while len(group) < 4:
            group.append(np.zeros_like(contact_images[0]))
        contact_rows.append(cv2.hconcat(group))
    contact = cv2.vconcat(contact_rows)
    cv2.imwrite(str(args.output_directory / "funnel_contact_sheet.png"),
                contact)

    summary = {
        "scope": "coarse unverified RGB-only box review frames",
        "invariant_violations": violations,
        "passed": not violations,
        "all_candidate_associations": len(candidate_rows),
        "all_removed_associations": len(removed_rows),
        "review_frames": len(boxes),
        "q_distribution_semantic_static_quality_eligible": {
            "inside_box": distribution(q_inside),
            "outside_box": distribution(q_outside),
            "note": "q>=3/5 counts are descriptive only; frozen q=10 is unchanged",
        },
        "stages": {},
    }
    for name in STAGES:
        total = totals[name]
        inside = totals[name + "_inside"]
        summary["stages"][name] = {
            "all_review_frame": total,
            "inside_box": inside,
            "inside_fraction": inside / total if total else None,
        }
    (args.output_directory / "funnel_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if violations:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
