#!/usr/bin/env python3
"""Audit whether current sparse evidence can support short motion tracks.

This is an input-feasibility audit, not a motion-segmentation algorithm. It
starts from exact C++ F1/F3 eligible feature locations, follows them causally
through earlier rectified RGB-D frames with the same pyramidal LK settings,
and reports 1/2/3/5-frame survival. Frozen RGB-only coarse boxes are consumed
only through the precomputed ``inside_box`` flag.

No threshold is selected for dynamic motion, no object identity is inferred,
and no SLAM state is modified.
"""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np


LK_WINDOW = (21, 21)
LK_MAX_LEVEL = 3
LK_CRITERIA = (
    cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS,
    30,
    0.01,
)
LK_MIN_EIGENVALUE = 1e-4
FB_MAX_PIXELS = 0.25
DEPTH_SCALE = 5000.0
HORIZONS = (1, 2, 3, 5)


def read_csv(path):
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
                raise ValueError(
                    "association rows must contain four fields")
            rows.append({
                "rgb_timestamp": float(fields[0]),
                "rgb_relative": fields[1],
                "depth_timestamp": float(fields[2]),
                "depth_relative": fields[3],
            })
    return rows


def load_rectification(config_path, image_size):
    storage = cv2.FileStorage(str(config_path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("failed to open settings: " + str(config_path))

    def value(name):
        node = storage.getNode(name)
        if node.empty():
            raise ValueError("missing setting: " + name)
        return float(node.real())

    if int(value("RGBD.InputRectification.Enable")) != 1:
        raise ValueError("Bonn input rectification must be enabled")
    camera = np.array([
        [value("RGBD.InputRectification.fx"), 0.0,
         value("RGBD.InputRectification.cx")],
        [0.0, value("RGBD.InputRectification.fy"),
         value("RGBD.InputRectification.cy")],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    distortion = np.array([
        value("RGBD.InputRectification.k1"),
        value("RGBD.InputRectification.k2"),
        value("RGBD.InputRectification.p1"),
        value("RGBD.InputRectification.p2"),
        value("RGBD.InputRectification.k3"),
    ], dtype=np.float64)
    storage.release()
    return cv2.initUndistortRectifyMap(
        camera,
        distortion,
        np.eye(3, dtype=np.float64),
        camera,
        image_size,
        cv2.CV_32FC1,
    )


def find_archive_root(archive):
    roots = {
        member.split("/", 1)[0]
        for member in archive.namelist()
        if "/" in member
    }
    if len(roots) != 1:
        raise ValueError("archive must contain exactly one root directory")
    return next(iter(roots))


def decode_member(archive, member, flags):
    try:
        encoded = np.frombuffer(archive.read(member), dtype=np.uint8)
    except KeyError as error:
        raise ValueError("missing archive member: " + member) from error
    image = cv2.imdecode(encoded, flags)
    if image is None:
        raise ValueError("failed to decode archive member: " + member)
    return image


class RectifiedSequence:
    def __init__(self, archive, archive_root, associations, config):
        self.archive = archive
        self.archive_root = archive_root.rstrip("/")
        self.associations = associations
        self.config = config
        self.maps = None
        self.gray_cache = {}
        self.depth_cache = {}

    def _member(self, relative):
        return self.archive_root + "/" + relative

    def gray(self, frame):
        if frame in self.gray_cache:
            return self.gray_cache[frame]
        row = self.associations[frame]
        raw = decode_member(
            self.archive,
            self._member(row["rgb_relative"]),
            cv2.IMREAD_COLOR,
        )
        if self.maps is None:
            self.maps = load_rectification(
                self.config, (raw.shape[1], raw.shape[0]))
        rectified = cv2.remap(
            raw,
            self.maps[0],
            self.maps[1],
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        self.gray_cache[frame] = gray
        return gray

    def depth(self, frame):
        if frame in self.depth_cache:
            return self.depth_cache[frame]
        # Ensure the maps are initialized from RGB with the expected size.
        self.gray(frame)
        row = self.associations[frame]
        raw = decode_member(
            self.archive,
            self._member(row["depth_relative"]),
            cv2.IMREAD_UNCHANGED,
        )
        if raw.dtype != np.uint16:
            raise ValueError("Bonn depth image must be uint16")
        rectified = cv2.remap(
            raw,
            self.maps[0],
            self.maps[1],
            cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        depth = rectified.astype(np.float32) / DEPTH_SCALE
        self.depth_cache[frame] = depth
        return depth


def load_start_nodes(proxy_path, feature_path):
    proxy_rows = read_csv(proxy_path)
    feature_rows = read_csv(feature_path)
    feature_by_key = {
        (int(row["frame"]), int(row["feature_index"])): row
        for row in feature_rows
    }
    nodes_by_frame = defaultdict(list)
    missing = []
    for row in proxy_rows:
        key = (int(row["frame"]), int(row["feature_index"]))
        feature = feature_by_key.get(key)
        if feature is None:
            missing.append(key)
            continue
        u = float(row["u_current"])
        v = float(row["v_current"])
        residual = float(row["flow_residual_magnitude_px"])
        if not all(math.isfinite(value) for value in (u, v, residual)):
            raise ValueError("proxy node contains a non-finite value")
        nodes_by_frame[key[0]].append({
            "frame": key[0],
            "feature_index": key[1],
            "u": u,
            "v": v,
            "inside_box": int(row["inside_box"]),
            "flow_residual_magnitude_px": residual,
            "has_mappoint": int(feature["has_mappoint"]),
            "semantic_nonzero": int(feature["semantic_nonzero"]),
        })
    if missing:
        raise ValueError(
            f"{len(missing)} proxy nodes are absent from feature CSV")
    if not nodes_by_frame:
        raise ValueError("no start nodes were loaded")
    return nodes_by_frame


def valid_depth_at(depth, points):
    rounded = np.rint(points).astype(np.int32)
    inside = (
        (rounded[:, 0] >= 0) &
        (rounded[:, 0] < depth.shape[1]) &
        (rounded[:, 1] >= 0) &
        (rounded[:, 1] < depth.shape[0])
    )
    valid = np.zeros(len(points), dtype=bool)
    indices = np.flatnonzero(inside)
    if len(indices):
        values = depth[
            rounded[indices, 1],
            rounded[indices, 0],
        ]
        valid[indices] = np.isfinite(values) & (values > 0.0)
    return valid


def track_backward(sequence, frame, nodes, maximum_horizon):
    points = np.asarray(
        [[node["u"], node["v"]] for node in nodes],
        dtype=np.float32,
    )
    positions = points.copy()
    alive = np.ones(len(nodes), dtype=bool)
    survived = np.zeros(len(nodes), dtype=np.int32)
    maximum_fb = np.zeros(len(nodes), dtype=np.float32)
    failure_reason = np.full(len(nodes), "", dtype=object)
    source_gray = sequence.gray(frame)

    for horizon in range(1, maximum_horizon + 1):
        target_frame = frame - horizon
        active_indices = np.flatnonzero(alive)
        if target_frame < 0 or len(active_indices) == 0:
            if target_frame < 0:
                failure_reason[active_indices] = "sequence_boundary"
            break
        target_gray = sequence.gray(target_frame)
        source_points = positions[active_indices].reshape(-1, 1, 2)
        target_points, backward_status, _ = cv2.calcOpticalFlowPyrLK(
            source_gray,
            target_gray,
            source_points,
            None,
            winSize=LK_WINDOW,
            maxLevel=LK_MAX_LEVEL,
            criteria=LK_CRITERIA,
            flags=0,
            minEigThreshold=LK_MIN_EIGENVALUE,
        )
        return_points, forward_status, _ = cv2.calcOpticalFlowPyrLK(
            target_gray,
            source_gray,
            target_points,
            None,
            winSize=LK_WINDOW,
            maxLevel=LK_MAX_LEVEL,
            criteria=LK_CRITERIA,
            flags=0,
            minEigThreshold=LK_MIN_EIGENVALUE,
        )
        backward_ok = backward_status.reshape(-1) != 0
        forward_ok = forward_status.reshape(-1) != 0
        target_flat = target_points.reshape(-1, 2)
        return_flat = return_points.reshape(-1, 2)
        fb_error = np.linalg.norm(
            return_flat - source_points.reshape(-1, 2), axis=1)
        finite = (
            np.all(np.isfinite(target_flat), axis=1) &
            np.isfinite(fb_error)
        )
        depth_ok = valid_depth_at(
            sequence.depth(target_frame), target_flat)
        good = (
            backward_ok & forward_ok & finite &
            (fb_error <= FB_MAX_PIXELS) & depth_ok
        )

        for local_index, global_index in enumerate(active_indices):
            maximum_fb[global_index] = max(
                maximum_fb[global_index],
                float(fb_error[local_index])
                if math.isfinite(float(fb_error[local_index]))
                else float("inf"),
            )
            if good[local_index]:
                survived[global_index] = horizon
                positions[global_index] = target_flat[local_index]
            else:
                if not backward_ok[local_index]:
                    reason = "backward_lk_invalid"
                elif not forward_ok[local_index]:
                    reason = "forward_lk_invalid"
                elif not finite[local_index]:
                    reason = "nonfinite"
                elif fb_error[local_index] > FB_MAX_PIXELS:
                    reason = "forward_backward_rejected"
                else:
                    reason = "depth_invalid"
                failure_reason[global_index] = reason
                alive[global_index] = False

        source_gray = target_gray

    failure_reason[alive & (survived == maximum_horizon)] = (
        "maximum_horizon_reached")
    return survived, maximum_fb, failure_reason


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def summarize_group(per_frame_rows, group):
    rows = [row for row in per_frame_rows if row["group"] == group]
    summary = {
        "frame_count": len(rows),
        "start_track_count": sum(row["start_tracks"] for row in rows),
        "has_mappoint_count": sum(
            row["has_mappoint_tracks"] for row in rows),
    }
    start = summary["start_track_count"]
    summary["has_mappoint_ratio"] = (
        summary["has_mappoint_count"] / start if start else None)
    for horizon in HORIZONS:
        key = f"survive_{horizon}_frames"
        counts = [row[key] for row in rows]
        total = sum(counts)
        summary[key + "_total"] = total
        summary[key + "_ratio"] = total / start if start else None
        summary[key + "_per_frame_median"] = percentile(counts, 50)
        for support in (3, 6, 10):
            summary[
                f"frames_with_at_least_{support}_tracks_at_{horizon}"
            ] = int(sum(count >= support for count in counts))
    return summary


def audit(args):
    nodes_by_frame = load_start_nodes(
        args.proxy_nodes, args.feature_csv)
    associations = read_associations(args.association)
    maximum_horizon = max(HORIZONS)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    per_track_rows = []
    per_frame_rows = []
    with ZipFile(args.archive) as archive:
        sequence = RectifiedSequence(
            archive,
            find_archive_root(archive),
            associations,
            args.config,
        )
        for frame in sorted(nodes_by_frame):
            if frame >= len(associations):
                raise ValueError(
                    f"frame {frame} exceeds association length")
            nodes = nodes_by_frame[frame]
            survived, maximum_fb, failure_reason = track_backward(
                sequence, frame, nodes, maximum_horizon)
            for index, node in enumerate(nodes):
                row = dict(node)
                row["survived_frames"] = int(survived[index])
                row["maximum_step_fb_error_px"] = float(maximum_fb[index])
                row["terminal_state"] = str(failure_reason[index])
                row["dynamic_decision"] = "none"
                row["direct_slam_state_mutation"] = "none"
                per_track_rows.append(row)

            for group, inside_value in (
                    ("inside_proxy", 1), ("background", 0)):
                indices = [
                    index for index, node in enumerate(nodes)
                    if node["inside_box"] == inside_value
                ]
                frame_row = {
                    "frame": frame,
                    "group": group,
                    "start_tracks": len(indices),
                    "has_mappoint_tracks": sum(
                        nodes[index]["has_mappoint"]
                        for index in indices),
                    "residual_median_px": percentile(
                        [
                            nodes[index]["flow_residual_magnitude_px"]
                            for index in indices
                        ],
                        50,
                    ),
                    "dynamic_decision": "none",
                    "direct_slam_state_mutation": "none",
                }
                for horizon in HORIZONS:
                    frame_row[f"survive_{horizon}_frames"] = int(sum(
                        survived[index] >= horizon for index in indices))
                per_frame_rows.append(frame_row)

    track_fields = [
        "frame", "feature_index", "u", "v", "inside_box",
        "flow_residual_magnitude_px", "has_mappoint",
        "semantic_nonzero", "survived_frames",
        "maximum_step_fb_error_px", "terminal_state",
        "dynamic_decision", "direct_slam_state_mutation",
    ]
    with (output / "per_track.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=track_fields)
        writer.writeheader()
        writer.writerows(per_track_rows)

    frame_fields = [
        "frame", "group", "start_tracks", "has_mappoint_tracks",
        "residual_median_px",
        *[f"survive_{horizon}_frames" for horizon in HORIZONS],
        "dynamic_decision", "direct_slam_state_mutation",
    ]
    with (output / "per_frame.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=frame_fields)
        writer.writeheader()
        writer.writerows(per_frame_rows)

    summary = {
        "sequence": args.sequence,
        "audit_identity": (
            "input support only; not motion grouping or object GT"),
        "track_direction": "causal backward from selected current frame",
        "lk_settings": {
            "window": list(LK_WINDOW),
            "maximum_level": LK_MAX_LEVEL,
            "termination_count": LK_CRITERIA[1],
            "termination_epsilon": LK_CRITERIA[2],
            "minimum_eigenvalue": LK_MIN_EIGENVALUE,
            "per_step_forward_backward_max_pixels": FB_MAX_PIXELS,
        },
        "horizons": list(HORIZONS),
        "inside_proxy": summarize_group(
            per_frame_rows, "inside_proxy"),
        "background": summarize_group(
            per_frame_rows, "background"),
        "proxy_identity": (
            "frozen RGB-only coarse box; not pixel/object/motion GT"),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
    }
    with (output / "summary.json").open(
            "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--archive", required=True, type=Path)
    parser.add_argument("--association", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--proxy-nodes", required=True, type=Path)
    parser.add_argument("--feature-csv", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    audit(parser.parse_args())


if __name__ == "__main__":
    main()
