#!/usr/bin/env python3
"""Prepare Bonn RGB/semantic-only review candidates.

The tool deliberately does not accept depth, geometry, flow, or SLAM-result
inputs. Selection is a development proxy, not motion ground truth.
"""

import argparse
import csv
import json
import math
import pathlib

import cv2
import numpy as np


SEALED_HOLDOUT_NAME = "rgbd_bonn_balloon_tracking"


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if not reader.fieldnames or any(None in row for row in rows):
            raise ValueError("invalid CSV: {}".format(path))
    return rows


def read_association(path):
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(
                    "{}:{} requires four fields".format(
                        path, line_number
                    )
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
        raise ValueError("{} contains no associations".format(path))
    return rows


def write_semantic_input(association_path, output_path):
    rows = read_association(association_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "selection_role",
        "selection_rank",
        "frame",
        "rgb_timestamp",
        "rgb_relative",
        "depth_timestamp",
        "depth_relative",
        "pose_timestamp_source",
        "pose_timestamp",
    ]
    with output_path.open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "selection_role": "all_frame_semantic_manifest",
                    "selection_rank": row["frame"] + 1,
                    **row,
                    "pose_timestamp_source": "rgb",
                    "pose_timestamp": row["rgb_timestamp"],
                }
            )
    print(
        "[G2-4F1 Independent Review] wrote {} semantic input rows to {}".
        format(len(rows), output_path)
    )


def load_rectification(config_path, image_size):
    storage = cv2.FileStorage(str(config_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("failed to open settings: {}".format(config_path))

    def setting(name):
        node = storage.getNode(name)
        if node.empty():
            raise ValueError("missing setting {}".format(name))
        return float(node.real())

    if int(setting("RGBD.InputRectification.Enable")) != 1:
        raise ValueError("Bonn input rectification must be enabled")
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


def load_rectified_images(dataset_root, associations, config):
    if SEALED_HOLDOUT_NAME in str(dataset_root):
        raise ValueError("refusing to access sealed strict hold-out")
    images = []
    thumbnails = []
    maps = None
    for row in associations:
        path = dataset_root / row["rgb_relative"]
        raw = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if raw is None:
            raise FileNotFoundError(path)
        if maps is None:
            maps = load_rectification(
                config, (raw.shape[1], raw.shape[0])
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
                gray, (40, 30), interpolation=cv2.INTER_AREA
            ).astype(np.float32)
            / 255.0
        )
    return images, thumbnails


def temporal_difference(thumbnails, frame):
    before = thumbnails[max(0, frame - 2)]
    after = thumbnails[min(len(thumbnails) - 1, frame + 2)]
    return float(np.mean(np.abs(after - before)))


def far_enough(frame, selected, excluded, minimum_separation):
    return all(
        abs(frame - other) >= minimum_separation
        for other in selected
    ) and all(
        abs(frame - other) >= minimum_separation
        for other in excluded
    )


def take_ranked(
    candidates,
    count,
    selected,
    excluded,
    minimum_separation,
):
    chosen = []
    for frame in candidates:
        if far_enough(
            frame,
            selected + chosen,
            excluded,
            minimum_separation,
        ):
            chosen.append(frame)
            if len(chosen) == count:
                break
    return chosen


def uniform_order(candidates, count):
    if not candidates:
        return []
    if count == 1:
        return [candidates[len(candidates) // 2]]
    targets = np.linspace(0, len(candidates) - 1, count)
    ordered = []
    for target in targets:
        frame = candidates[int(round(float(target)))]
        if frame not in ordered:
            ordered.append(frame)
    for frame in candidates:
        if frame not in ordered:
            ordered.append(frame)
    return ordered


def make_clip(images, center, role, sequence):
    tiles = []
    for frame in range(center - 2, center + 3):
        tile = cv2.resize(
            images[frame], (160, 120), interpolation=cv2.INTER_AREA
        )
        cv2.putText(
            tile,
            "f={} {:+d}{}".format(
                frame, frame - center, " CENTER" if frame == center else ""
            ),
            (4, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 255) if frame == center else (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)
    clip = np.hstack(tiles)
    cv2.putText(
        clip,
        "{} {} {}".format(sequence, role, center),
        (4, 116),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )
    return clip


def select_candidates(args):
    associations = read_association(args.association)
    manifest = read_csv(args.semantic_manifest)
    if len(manifest) != len(associations):
        raise ValueError("semantic manifest and association lengths differ")
    mask_pixels = {}
    for row in manifest:
        frame = int(row["source_frame"])
        if frame in mask_pixels:
            raise ValueError("duplicate semantic source frame")
        mask_pixels[frame] = int(row["mask_nonzero_pixels"])
    if set(mask_pixels) != set(range(len(associations))):
        raise ValueError("semantic manifest does not cover every frame")

    images, thumbnails = load_rectified_images(
        args.dataset_root, associations, args.config
    )
    existing = set()
    if args.existing_candidates:
        existing = {
            int(row["frame"])
            for row in read_csv(args.existing_candidates)
        }

    eligible = list(range(2, len(associations) - 2))
    center_absent = [
        frame for frame in eligible if mask_pixels[frame] == 0
    ]
    full_window_absent = [
        frame
        for frame in center_absent
        if all(
            mask_pixels[index] == 0
            for index in range(frame - 2, frame + 3)
        )
    ]
    transition_absent = [
        frame
        for frame in center_absent
        if any(
            mask_pixels[index] > 0
            for index in range(frame - 2, frame + 3)
        )
    ]
    screening_pool = (
        eligible if args.all_frame_screening else center_absent
    )
    differences = {
        frame: temporal_difference(thumbnails, frame)
        for frame in screening_pool
    }

    selected = []
    roles = {}
    if args.all_frame_screening:
        high_change_pool = eligible
        high_change_role = "rgb_change_all_frame_screening"
    else:
        high_change_pool = full_window_absent
        high_change_role = "rgb_change_person_absent_window"
    high_change = take_ranked(
        sorted(
            high_change_pool,
            key=lambda frame: (-differences[frame], frame),
        ),
        args.per_stratum,
        selected,
        existing,
        args.minimum_separation,
    )
    for frame in high_change:
        roles[frame] = high_change_role
    selected.extend(high_change)

    transitions = []
    if not args.all_frame_screening:
        transitions = take_ranked(
            sorted(
                transition_absent,
                key=lambda frame: (-differences[frame], frame),
            ),
            args.per_stratum,
            selected,
            existing,
            args.minimum_separation,
        )
    for frame in transitions:
        roles[frame] = "semantic_transition_center_absent"
    selected.extend(transitions)

    if args.all_frame_screening:
        uniform_pool = eligible
        uniform_role = "uniform_all_frame_screening"
    else:
        uniform_pool = full_window_absent
        uniform_role = "uniform_person_absent_window"
    uniform = take_ranked(
        uniform_order(
            sorted(uniform_pool),
            max(args.per_stratum * 3, args.per_stratum),
        ),
        args.per_stratum,
        selected,
        existing,
        args.minimum_separation,
    )
    for frame in uniform:
        roles[frame] = uniform_role
    selected.extend(uniform)

    output = args.output_dir
    clips_directory = output / "clips"
    output.mkdir(parents=True, exist_ok=True)
    clips_directory.mkdir(parents=True, exist_ok=True)
    rows = []
    clip_rows = []
    for rank, frame in enumerate(selected, 1):
        association = associations[frame]
        role = roles[frame]
        clip = make_clip(images, frame, role, args.sequence)
        clip_name = "{}_frame_{:06d}.png".format(
            args.sequence, frame
        )
        cv2.imwrite(str(clips_directory / clip_name), clip)
        clip_rows.append(clip)
        rows.append(
            {
                "selection_role": role,
                "selection_rank": rank,
                **association,
                "pose_timestamp_source": "rgb",
                "pose_timestamp": association["rgb_timestamp"],
                "center_person_mask_pixels": mask_pixels[frame],
                "window_person_absent": int(
                    frame in full_window_absent
                ),
                "rgb_temporal_difference": differences[frame],
                "motion_label": "",
                "confidence": "",
                "reason": "",
                "label_source": "agent_rgb_temporal_only_v2",
                "is_ground_truth": "false",
                "geometry_or_flow_seen": "false",
                "clip_relative": "clips/" + clip_name,
            }
        )

    fields = list(rows[0])
    with (output / "selected_frames.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    sheets = []
    for start in range(0, len(clip_rows), args.clips_per_sheet):
        batch = clip_rows[start : start + args.clips_per_sheet]
        sheet = np.vstack(batch)
        sheet_path = output / "review_sheet_{:02d}.png".format(
            len(sheets)
        )
        cv2.imwrite(str(sheet_path), sheet)
        sheets.append(str(sheet_path))

    summary = {
        "sequence": args.sequence,
        "identity": "RGB/semantic-only development review candidates",
        "is_ground_truth": False,
        "geometry_or_flow_read": False,
        "strict_holdout_opened": False,
        "association_frames": len(associations),
        "person_absent_center_frames": len(center_absent),
        "person_absent_full_window_frames": len(full_window_absent),
        "semantic_transition_center_absent_frames": len(
            transition_absent
        ),
        "existing_candidates_excluded": len(existing),
        "minimum_separation_frames": args.minimum_separation,
        "per_stratum_requested": args.per_stratum,
        "all_frame_screening": args.all_frame_screening,
        "selected_by_role": {
            role: sum(row["selection_role"] == role for row in rows)
            for role in sorted(set(roles.values()))
        },
        "selected_frames": selected,
        "review_sheets": sheets,
        "dynamic_decision": None,
        "direct_slam_state_mutation": "none",
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        "[G2-4F1 Independent Review] selected {} frames into {}".
        format(len(rows), output)
    )


def self_test():
    assert temporal_difference(
        [
            np.zeros((2, 2), np.float32),
            np.zeros((2, 2), np.float32),
            np.zeros((2, 2), np.float32),
            np.ones((2, 2), np.float32),
            np.ones((2, 2), np.float32),
        ],
        2,
    ) == 1.0
    assert take_ranked([1, 2, 10], 2, [], [], 3) == [1, 10]
    print("[G2-4F1 Independent Review Self-Test] PASS")


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    semantic = subparsers.add_parser("semantic-input")
    semantic.add_argument("association", type=pathlib.Path)
    semantic.add_argument("output", type=pathlib.Path)

    select = subparsers.add_parser("select")
    select.add_argument("dataset_root", type=pathlib.Path)
    select.add_argument("association", type=pathlib.Path)
    select.add_argument("semantic_manifest", type=pathlib.Path)
    select.add_argument("config", type=pathlib.Path)
    select.add_argument("output_dir", type=pathlib.Path)
    select.add_argument("--sequence", required=True)
    select.add_argument(
        "--existing-candidates", type=pathlib.Path
    )
    select.add_argument("--per-stratum", type=int, default=12)
    select.add_argument("--minimum-separation", type=int, default=6)
    select.add_argument("--clips-per-sheet", type=int, default=8)
    select.add_argument(
        "--all-frame-screening",
        action="store_true",
        help=(
            "screen all eligible RGB frames, including person-present "
            "frames; selection still does not read geometry or flow"
        ),
    )

    subparsers.add_parser("self-test")
    args = parser.parse_args()
    if args.command == "semantic-input":
        write_semantic_input(args.association, args.output)
    elif args.command == "select":
        select_candidates(args)
    else:
        self_test()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
