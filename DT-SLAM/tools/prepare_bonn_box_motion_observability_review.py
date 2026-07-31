#!/usr/bin/env python3
"""Prepare Bonn RGB-only box-motion observability review clips.

Candidate sampling combines uniform coverage with high/low coarse RGB temporal
change. The metric is a review-selection proxy, never a motion label. The tool
does not accept geometry, flow, depth-residual, or SLAM-result inputs.
"""

import argparse
import csv
import json
import tempfile
import zipfile
from pathlib import Path

import cv2
import numpy as np


SEALED_HOLDOUT_BASENAME = "rgbd_bonn_balloon_tracking.zip"
ROLES = ("uniform", "rgb_change_high", "rgb_change_low")


def read_associations(path):
    rows = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(
                    "%s:%d requires four fields" % (path, line_number)
                )
            rows.append(
                {
                    "frame": len(rows),
                    "rgb_timestamp": fields[0],
                    "rgb_relative": fields[1],
                    "depth_timestamp": fields[2],
                    "depth_relative": fields[3],
                }
            )
    if not rows:
        raise ValueError("association contains no rows")
    return rows


def read_candidate_rows(path, sequence):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("reuse candidate CSV contains no rows")
    parsed = []
    seen = set()
    for row in rows:
        if row["sequence"] != sequence:
            raise ValueError("reuse candidate sequence differs")
        frame = int(row["frame"])
        if frame in seen:
            raise ValueError("duplicate reused candidate frame")
        seen.add(frame)
        role = row["selection_role"]
        if role not in ROLES:
            raise ValueError("unknown reused candidate role: " + role)
        parsed.append(
            (
                frame,
                role,
                float(row["proxy_rgb_temporal_change"]),
            )
        )
    return parsed


def find_archive_root(archive):
    roots = {
        member.split("/", 1)[0]
        for member in archive.namelist()
        if "/" in member
    }
    if len(roots) != 1:
        raise ValueError("archive must contain exactly one root directory")
    return next(iter(roots))


def decode_rgb(archive, member):
    try:
        encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    except KeyError as error:
        raise ValueError("missing archive member: " + member) from error
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode archive member: " + member)
    return image


def load_rectification(config_path, image_size):
    storage = cv2.FileStorage(str(config_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("failed to open settings: " + str(config_path))

    def setting(name):
        node = storage.getNode(name)
        if node.empty():
            raise ValueError("missing setting: " + name)
        return float(node.real())

    if int(setting("RGBD.InputRectification.Enable")) != 1:
        raise ValueError("Bonn review requires enabled input rectification")
    camera = np.array(
        [
            [
                setting("RGBD.InputRectification.fx"),
                0.0,
                setting("RGBD.InputRectification.cx"),
            ],
            [
                0.0,
                setting("RGBD.InputRectification.fy"),
                setting("RGBD.InputRectification.cy"),
            ],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.array(
        [
            setting("RGBD.InputRectification.k1"),
            setting("RGBD.InputRectification.k2"),
            setting("RGBD.InputRectification.p1"),
            setting("RGBD.InputRectification.p2"),
            setting("RGBD.InputRectification.k3"),
        ],
        dtype=np.float64,
    )
    storage.release()
    return cv2.initUndistortRectifyMap(
        camera,
        distortion,
        np.eye(3, dtype=np.float64),
        camera,
        image_size,
        cv2.CV_32FC1,
    )


def temporal_metric(thumbnails, frame, radius):
    before = thumbnails[frame - radius]
    after = thumbnails[frame + radius]
    return float(np.mean(np.abs(after - before)))


def far_enough(frame, selected, minimum_separation):
    return all(
        abs(frame - other) >= minimum_separation for other in selected
    )


def take_ordered(order, count, selected, minimum_separation):
    chosen = []
    for frame in order:
        if far_enough(frame, selected + chosen, minimum_separation):
            chosen.append(frame)
            if len(chosen) == count:
                break
    return chosen


def uniform_order(eligible, count):
    if not eligible:
        return []
    targets = np.linspace(0, len(eligible) - 1, max(count, 1))
    preferred = []
    for target in targets:
        frame = eligible[int(round(float(target)))]
        if frame not in preferred:
            preferred.append(frame)
    return preferred + [frame for frame in eligible if frame not in preferred]


def annotate_tile(image, frame, center, offset, tile_size):
    tile = cv2.resize(image, tile_size, interpolation=cv2.INTER_AREA)
    color = (0, 255, 255) if frame == center else (255, 255, 255)
    text = "f=%d offset=%+d%s" % (
        frame,
        offset,
        " CENTER" if frame == center else "",
    )
    cv2.putText(
        tile,
        text,
        (5, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        color,
        1,
        cv2.LINE_AA,
    )
    return tile


def prepare(args):
    archive_path = Path(args.archive)
    if archive_path.name == SEALED_HOLDOUT_BASENAME:
        raise ValueError("refusing to open sealed strict hold-out archive")

    associations = read_associations(args.association)
    radius = args.metric_radius
    if len(associations) <= 2 * radius:
        raise ValueError("sequence is too short for requested metric radius")

    output = Path(args.output_dir)
    clips_dir = output / "clips"
    sheets_dir = output / "contact_sheets"
    clips_dir.mkdir(parents=True, exist_ok=True)
    sheets_dir.mkdir(parents=True, exist_ok=True)

    images = []
    thumbnails = []
    with zipfile.ZipFile(archive_path) as archive:
        root = find_archive_root(archive)
        maps = None
        for row in associations:
            raw = decode_rgb(
                archive, root + "/" + row["rgb_relative"]
            )
            if maps is None:
                maps = load_rectification(
                    args.config, (raw.shape[1], raw.shape[0])
                )
            rectified = cv2.remap(
                raw,
                maps[0],
                maps[1],
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            images.append(rectified)
            gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
            thumbnails.append(
                cv2.resize(
                    gray,
                    (args.metric_width, args.metric_height),
                    interpolation=cv2.INTER_AREA,
                ).astype(np.float32)
                / 255.0
            )

    eligible = list(range(radius, len(associations) - radius))
    metrics = {
        frame: temporal_metric(thumbnails, frame, radius)
        for frame in eligible
    }
    reused_candidates = bool(args.reuse_candidates)
    if reused_candidates:
        reused = read_candidate_rows(args.reuse_candidates, args.sequence)
        selected = sorted(frame for frame, _, _ in reused)
        role_by_frame = {frame: role for frame, role, _ in reused}
        reused_metric = {frame: metric for frame, _, metric in reused}
        if any(frame not in metrics for frame in selected):
            raise ValueError("reused candidate lacks temporal metric support")
        if any(
            not np.isclose(metrics[frame], reused_metric[frame], atol=5e-10)
            for frame in selected
        ):
            raise ValueError("reused candidate metric differs")
    else:
        selected = []
        role_by_frame = {}
        role_orders = {
            "uniform": uniform_order(eligible, args.uniform_count),
            "rgb_change_high": sorted(
                eligible, key=lambda frame: (-metrics[frame], frame)
            ),
            "rgb_change_low": sorted(
                eligible, key=lambda frame: (metrics[frame], frame)
            ),
        }
        role_counts = {
            "uniform": args.uniform_count,
            "rgb_change_high": args.high_count,
            "rgb_change_low": args.low_count,
        }
        for role in ROLES:
            chosen = take_ordered(
                role_orders[role],
                role_counts[role],
                selected,
                args.minimum_separation,
            )
            for frame in chosen:
                role_by_frame[frame] = role
            selected.extend(chosen)
        selected.sort()

    offsets = list(range(-args.clip_radius, args.clip_radius + 1, args.clip_step))
    if 0 not in offsets:
        offsets.append(0)
        offsets.sort()
    records = []
    clip_images = []
    for frame in selected:
        tiles = [
            annotate_tile(
                images[min(max(frame + offset, 0), len(images) - 1)],
                min(max(frame + offset, 0), len(images) - 1),
                frame,
                offset,
                (args.tile_width, args.tile_height),
            )
            for offset in offsets
        ]
        clip = np.hstack(tiles)
        cv2.putText(
            clip,
            "%s | %s | proxy_rgb_change=%.6f"
            % (args.sequence, role_by_frame[frame], metrics[frame]),
            (6, args.tile_height - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        clip_name = "%s_frame_%06d.png" % (args.sequence, frame)
        clip_path = clips_dir / clip_name
        if not cv2.imwrite(str(clip_path), clip):
            raise ValueError("failed to write " + str(clip_path))
        clip_images.append((frame, clip))
        records.append(
            {
                "sequence": args.sequence,
                "frame": frame,
                "rgb_timestamp": associations[frame]["rgb_timestamp"],
                "selection_role": role_by_frame[frame],
                "proxy_rgb_temporal_change": "%.9f" % metrics[frame],
                "box_visibility": "",
                "box_motion": "",
                "person_presence": "",
                "confidence": "",
                "reason": "",
                "label_source": "agent_rgb_temporal_review_v1",
                "is_ground_truth": "false",
                "geometry_flow_depth_score_used": "false",
                "clip_relative": str(clip_path.relative_to(output)),
            }
        )

    fields = list(records[0])
    candidates_path = output / (
        args.sequence + "_box_motion_observability_review.csv"
    )
    with candidates_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    for start in range(0, len(clip_images), args.clips_per_sheet):
        group = clip_images[start:start + args.clips_per_sheet]
        sheet = np.vstack([clip for _, clip in group])
        sheet_path = sheets_dir / (
            "%s_%02d_frames_%06d_%06d.png"
            % (
                args.sequence,
                start // args.clips_per_sheet,
                group[0][0],
                group[-1][0],
            )
        )
        if not cv2.imwrite(str(sheet_path), sheet):
            raise ValueError("failed to write " + str(sheet_path))

    summary = {
        "sequence": args.sequence,
        "archive": str(archive_path),
        "association": str(args.association),
        "frame_count": len(associations),
        "candidate_count": len(records),
        "candidate_role_counts": {
            role: sum(row["selection_role"] == role for row in records)
            for role in ROLES
        },
        "metric": "mean_abs_gray(frame-radius, frame+radius)",
        "metric_radius": radius,
        "clip_offsets": offsets,
        "minimum_separation": args.minimum_separation,
        "selection_is_motion_ground_truth": False,
        "candidate_selection_reused": reused_candidates,
        "candidate_selection_source": (
            str(args.reuse_candidates) if reused_candidates else "generated"
        ),
        "geometry_flow_depth_score_used": False,
        "sealed_holdout_accessed": False,
        "labels_complete": False,
    }
    summary_path = output / (args.sequence + "_summary.json")
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


def self_test():
    eligible = list(range(6, 94))
    order = uniform_order(eligible, 8)
    selected = take_ordered(order, 8, [], 10)
    if len(selected) != 8:
        raise AssertionError("uniform selection test failed")
    if any(
        abs(first - second) < 10
        for index, first in enumerate(selected)
        for second in selected[index + 1:]
    ):
        raise AssertionError("minimum separation test failed")
    thumbnails = [
        np.full((3, 4), frame / 100.0, dtype=np.float32)
        for frame in range(20)
    ]
    measured = temporal_metric(thumbnails, 10, 5)
    if not np.isclose(measured, 0.1):
        raise AssertionError("temporal metric test failed")
    with tempfile.TemporaryDirectory() as directory:
        archive_path = Path(directory) / "development.zip"
        image = np.full((4, 6, 3), 17, dtype=np.uint8)
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise AssertionError("image encode test failed")
        with zipfile.ZipFile(archive_path, "w") as archive:
            archive.writestr("development/rgb/0.png", encoded.tobytes())
        with zipfile.ZipFile(archive_path) as archive:
            if find_archive_root(archive) != "development":
                raise AssertionError("archive root test failed")
            decoded = decode_rgb(archive, "development/rgb/0.png")
            if not np.array_equal(decoded, image):
                raise AssertionError("archive decode test failed")
    print("prepare_bonn_box_motion_observability_review self-test: PASS")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive")
    parser.add_argument("--association")
    parser.add_argument("--sequence")
    parser.add_argument("--config")
    parser.add_argument("--output-dir")
    parser.add_argument("--uniform-count", type=int, default=8)
    parser.add_argument("--high-count", type=int, default=6)
    parser.add_argument("--low-count", type=int, default=4)
    parser.add_argument("--minimum-separation", type=int, default=12)
    parser.add_argument("--metric-radius", type=int, default=6)
    parser.add_argument("--metric-width", type=int, default=80)
    parser.add_argument("--metric-height", type=int, default=60)
    parser.add_argument("--clip-radius", type=int, default=6)
    parser.add_argument("--clip-step", type=int, default=2)
    parser.add_argument("--tile-width", type=int, default=160)
    parser.add_argument("--tile-height", type=int, default=120)
    parser.add_argument("--clips-per-sheet", type=int, default=4)
    parser.add_argument("--reuse-candidates")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    required = ("archive", "association", "sequence", "config", "output_dir")
    missing = [name for name in required if not getattr(args, name)]
    if missing:
        raise SystemExit("missing required arguments: " + ", ".join(missing))
    prepare(args)


if __name__ == "__main__":
    main()
