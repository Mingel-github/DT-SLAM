#!/usr/bin/env python3
"""Prepare RGB-only Bonn temporal clips for blind motion-state review.

The tool reads only an explicitly supplied development archive. It refuses the
sealed balloon-tracking hold-out by basename. No depth, geometry score, flow,
or proxy ranking value is loaded.
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


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_associations(path):
    rows = []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) != 4:
                raise ValueError("association rows must contain four fields")
            rows.append(
                {
                    "rgb_timestamp": float(fields[0]),
                    "rgb_relative": fields[1],
                    "depth_timestamp": float(fields[2]),
                    "depth_relative": fields[3],
                }
            )
    return rows


def load_rectification(config_path, image_size):
    storage = cv2.FileStorage(str(config_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("failed to open OpenCV settings: " + str(config_path))

    def value(name):
        node = storage.getNode(name)
        if node.empty():
            raise ValueError("missing setting: " + name)
        return float(node.real())

    enabled = int(value("RGBD.InputRectification.Enable"))
    if enabled != 1:
        raise ValueError("temporal review requires enabled Bonn rectification")
    camera = np.array(
        [
            [value("RGBD.InputRectification.fx"), 0.0,
             value("RGBD.InputRectification.cx")],
            [0.0, value("RGBD.InputRectification.fy"),
             value("RGBD.InputRectification.cy")],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.array(
        [
            value("RGBD.InputRectification.k1"),
            value("RGBD.InputRectification.k2"),
            value("RGBD.InputRectification.p1"),
            value("RGBD.InputRectification.p2"),
            value("RGBD.InputRectification.k3"),
        ],
        dtype=np.float64,
    )
    storage.release()
    maps = cv2.initUndistortRectifyMap(
        camera,
        distortion,
        np.eye(3, dtype=np.float64),
        camera,
        image_size,
        cv2.CV_32FC1,
    )
    return maps


def find_archive_root(archive):
    roots = set()
    for member in archive.namelist():
        if "/" in member:
            roots.add(member.split("/", 1)[0])
    if len(roots) != 1:
        raise ValueError("archive must contain exactly one root directory")
    return next(iter(roots))


def decode_member(archive, member):
    try:
        encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    except KeyError as error:
        raise ValueError("missing archive member: " + member) from error
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode archive member: " + member)
    return image


def load_reviews(path, export_name):
    reviews = {}
    for row in read_rows(path):
        if row["export_name"] != export_name:
            continue
        frame = int(row["source_frame"])
        if frame in reviews:
            raise ValueError("duplicate review frame")
        reviews[frame] = row
    return reviews


def optional_int(row, key):
    value = row.get(key, "")
    return int(value) if value not in ("", None) else None


def make_tile(image, frame, offset, center, bbox, tile_size):
    tile_width, tile_height = tile_size
    resized = cv2.resize(image, (tile_width, tile_height),
                         interpolation=cv2.INTER_AREA)
    if center and bbox is not None:
        sx = tile_width / image.shape[1]
        sy = tile_height / image.shape[0]
        x, y, width, height = bbox
        top_left = (round(x * sx), round(y * sy))
        bottom_right = (round((x + width) * sx),
                        round((y + height) * sy))
        cv2.rectangle(resized, top_left, bottom_right, (0, 255, 0), 2)
    cv2.putText(
        resized,
        "frame=%d offset=%+d%s" % (
            frame, offset, " CENTER" if center else ""),
        (6, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255) if center else (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return resized


def prepare(args):
    archive_path = Path(args.archive)
    if archive_path.name == SEALED_HOLDOUT_BASENAME:
        raise ValueError("refusing to open sealed strict hold-out archive")

    associations = read_associations(args.association)
    candidates = sorted(
        {int(row["frame"]) for row in read_rows(args.candidates)}
    )
    reviews = load_reviews(args.review_csv, args.export_name)
    if set(candidates) != set(reviews):
        raise ValueError("candidate frames and bbox review frames differ")

    output = Path(args.output_dir)
    clips_dir = output / "clips"
    clips_dir.mkdir(parents=True, exist_ok=True)
    center_reference = (
        Path(args.center_reference_dir)
        if args.center_reference_dir
        else None
    )

    records = []
    clip_images = []
    exact_center_matches = 0
    boundary_shifted_windows = 0
    with zipfile.ZipFile(archive_path) as archive:
        root = find_archive_root(archive)
        maps = None
        for source_frame in candidates:
            if len(associations) < 5:
                raise ValueError("sequence must contain at least five frames")
            window_start = min(
                max(source_frame - 2, 0), len(associations) - 5)
            window_frames = list(range(window_start, window_start + 5))
            if window_frames != list(
                    range(source_frame - 2, source_frame + 3)):
                boundary_shifted_windows += 1
            review = reviews[source_frame]
            visibility = review["visibility"]
            bbox_values = tuple(
                optional_int(review, key)
                for key in ("bbox_x", "bbox_y", "bbox_width", "bbox_height")
            )
            bbox = (
                None
                if visibility == "absent" or any(
                    value is None for value in bbox_values)
                else bbox_values
            )
            tiles = []
            source_frames = []
            for frame in window_frames:
                offset = frame - source_frame
                association = associations[frame]
                member = root + "/" + association["rgb_relative"]
                raw = decode_member(archive, member)
                if maps is None:
                    maps = load_rectification(
                        args.config, (raw.shape[1], raw.shape[0]))
                rectified = cv2.remap(
                    raw, maps[0], maps[1], cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT)
                if offset == 0 and center_reference is not None:
                    reference_path = (
                        center_reference / ("frame_%06d.png" % source_frame)
                    )
                    reference = cv2.imread(
                        str(reference_path), cv2.IMREAD_COLOR)
                    if reference is None:
                        raise ValueError(
                            "missing center reference: " +
                            str(reference_path))
                    if not np.array_equal(rectified, reference):
                        raise ValueError(
                            "Python/C++ rectified center images differ: " +
                            str(reference_path))
                    exact_center_matches += 1
                tiles.append(
                    make_tile(
                        rectified,
                        frame,
                        offset,
                        offset == 0,
                        bbox,
                        (args.tile_width, args.tile_height),
                    )
                )
                source_frames.append(frame)
            clip = np.hstack(tiles)
            clip_path = clips_dir / (
                "%s_frame_%06d.png" % (args.sequence, source_frame))
            cv2.imwrite(str(clip_path), clip)
            clip_images.append((source_frame, clip))
            records.append(
                {
                    "sequence": args.sequence,
                    "source_frame": source_frame,
                    "review_frame_start": source_frames[0],
                    "review_frame_end": source_frames[-1],
                    "motion_label": "",
                    "confidence": "",
                    "reason": "",
                    "label_source": "agent_rgb_temporal_only_v1",
                    "is_ground_truth": "false",
                    "geometry_or_flow_seen": "false",
                    "center_visibility_proxy": visibility,
                    "clip_relative": str(clip_path.relative_to(output)),
                }
            )

    sheets_dir = output / "contact_sheets"
    sheets_dir.mkdir(parents=True, exist_ok=True)
    per_sheet = args.clips_per_sheet
    for start in range(0, len(clip_images), per_sheet):
        group = clip_images[start:start + per_sheet]
        sheet = np.vstack([image for _, image in group])
        sheet_path = sheets_dir / (
            "%s_%02d_frames_%06d_%06d.png"
            % (
                args.sequence,
                start // per_sheet,
                group[0][0],
                group[-1][0],
            )
        )
        cv2.imwrite(str(sheet_path), sheet)

    template_path = output / (
        args.sequence + "_agent_rgb_temporal_motion_proxy_v1.csv")
    with template_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    summary = {
        "sequence": args.sequence,
        "archive": str(archive_path),
        "sealed_holdout_accessed": False,
        "candidate_count": len(candidates),
        "nominal_window_offsets": [-2, -1, 0, 1, 2],
        "boundary_shifted_windows": boundary_shifted_windows,
        "center_reference_exact_matches": exact_center_matches,
        "label_source": "agent_rgb_temporal_only_v1",
        "is_ground_truth": False,
        "geometry_or_flow_used": False,
        "motion_labels_complete": False,
    }
    (output / (args.sequence + "_summary.json")).write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


def self_test():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        archive_path = root / "development.zip"
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        image[:, :, 1] = 100
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise AssertionError("failed to encode test image")
        with zipfile.ZipFile(archive_path, "w") as archive:
            for frame in range(5):
                archive.writestr(
                    "development/rgb/%d.png" % frame, encoded.tobytes())
        with zipfile.ZipFile(archive_path) as archive:
            if find_archive_root(archive) != "development":
                raise AssertionError("archive root test failed")
            decoded = decode_member(archive, "development/rgb/0.png")
            if not np.array_equal(decoded, image):
                raise AssertionError("archive image decode test failed")
        try:
            with zipfile.ZipFile(root / SEALED_HOLDOUT_BASENAME, "w"):
                pass
            args = argparse.Namespace(
                archive=str(root / SEALED_HOLDOUT_BASENAME))
            if Path(args.archive).name == SEALED_HOLDOUT_BASENAME:
                raise ValueError("refusing to open sealed strict hold-out archive")
        except ValueError:
            pass
        else:
            raise AssertionError("sealed hold-out guard failed")
    print("prepare_bonn_temporal_motion_review self-test: PASS")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive")
    parser.add_argument("--association")
    parser.add_argument("--candidates")
    parser.add_argument("--review-csv")
    parser.add_argument("--export-name")
    parser.add_argument("--sequence")
    parser.add_argument("--config")
    parser.add_argument("--center-reference-dir")
    parser.add_argument("--output-dir")
    parser.add_argument("--tile-width", type=int, default=256)
    parser.add_argument("--tile-height", type=int, default=192)
    parser.add_argument("--clips-per-sheet", type=int, default=4)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return
    required = (
        "archive",
        "association",
        "candidates",
        "review_csv",
        "export_name",
        "sequence",
        "config",
        "output_dir",
    )
    missing = [name for name in required if not getattr(args, name)]
    if missing:
        raise SystemExit("missing required arguments: " + ", ".join(missing))
    prepare(args)


if __name__ == "__main__":
    main()
