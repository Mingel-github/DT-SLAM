#!/usr/bin/env python3
"""Audit Bonn joint RGB/registered-depth rectification without SLAM mutation."""

import argparse
import json
import pathlib
import sys

import cv2
import numpy as np


BONN_K = np.array(
    [
        [542.822841, 0.0, 315.593520],
        [0.0, 542.576870, 237.756098],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
BONN_D = np.array(
    [0.039903, -0.099343, -0.000730, -0.000144, 0.0],
    dtype=np.float64,
)


def read_association(path, max_frames):
    pairs = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 4:
                raise ValueError(
                    "{}:{}: expected four fields".format(path, line_number)
                )
            pairs.append((fields[1], fields[3]))
            if len(pairs) == max_frames:
                break
    if not pairs:
        raise ValueError("{} contains no RGB-D pairs".format(path))
    return pairs


def depth_edges(depth):
    depth_m = depth.astype(np.float32) / 5000.0
    valid = depth_m > 0.0
    edges = np.zeros(depth.shape, dtype=np.uint8)

    left = depth_m[:, :-1]
    right = depth_m[:, 1:]
    horizontal_valid = valid[:, :-1] & valid[:, 1:]
    horizontal_threshold = np.maximum(
        0.08, 0.025 * np.minimum(left, right)
    )
    horizontal_edge = horizontal_valid & (
        np.abs(left - right) > horizontal_threshold
    )
    edges[:, :-1][horizontal_edge] = 255
    edges[:, 1:][horizontal_edge] = 255

    top = depth_m[:-1, :]
    bottom = depth_m[1:, :]
    vertical_valid = valid[:-1, :] & valid[1:, :]
    vertical_threshold = np.maximum(
        0.08, 0.025 * np.minimum(top, bottom)
    )
    vertical_edge = vertical_valid & (
        np.abs(top - bottom) > vertical_threshold
    )
    edges[:-1, :][vertical_edge] = 255
    edges[1:, :][vertical_edge] = 255
    return edges


def alignment_proxy(rgb, depth):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    gradient_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gradient_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    gradient = cv2.magnitude(gradient_x, gradient_y)
    valid_gradient = gradient[depth > 0]
    if valid_gradient.size == 0:
        return {"depth_edge_pixels": 0, "rgb_edge_near_depth_edge_ratio": None}

    threshold = float(np.percentile(valid_gradient, 90.0))
    rgb_edges = (gradient >= threshold).astype(np.uint8) * 255
    rgb_edges_nearby = cv2.dilate(
        rgb_edges, np.ones((5, 5), dtype=np.uint8)
    )
    geometry_edges = depth_edges(depth)
    edge_count = int(np.count_nonzero(geometry_edges))
    ratio = None
    if edge_count:
        ratio = float(
            np.count_nonzero(
                (geometry_edges > 0) & (rgb_edges_nearby > 0)
            )
            / edge_count
        )
    return {
        "depth_edge_pixels": edge_count,
        "rgb_edge_near_depth_edge_ratio": ratio,
    }


def summarize(values):
    array = np.asarray(values, dtype=np.float64)
    return {
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset_root", type=pathlib.Path)
    parser.add_argument("association", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--max-frames", type=int, default=30)
    args = parser.parse_args()

    if args.max_frames <= 0:
        raise ValueError("--max-frames must be positive")
    if args.output.exists():
        raise ValueError(
            "{} already exists; refusing to overwrite it".format(args.output)
        )

    pairs = read_association(args.association, args.max_frames)
    map_x, map_y = cv2.initUndistortRectifyMap(
        BONN_K,
        BONN_D,
        None,
        BONN_K,
        (640, 480),
        cv2.CV_32FC1,
    )
    in_bounds = (
        (map_x >= 0.0)
        & (map_x <= 639.0)
        & (map_y >= 0.0)
        & (map_y <= 479.0)
    )

    raw_valid_counts = []
    rectified_valid_counts = []
    retention_ratios = []
    nearest_value_violations = []
    raw_alignment = []
    rectified_alignment = []

    for rgb_relative, depth_relative in pairs:
        rgb_path = args.dataset_root / rgb_relative
        depth_path = args.dataset_root / depth_relative
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_UNCHANGED)
        depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            raise ValueError(
                "failed to load existing association pair: {} {}".format(
                    rgb_path, depth_path
                )
            )
        if rgb.shape[:2] != (480, 640) or depth.shape != (480, 640):
            raise ValueError("Bonn audit expects 640x480 registered RGB-D")
        if rgb.dtype != np.uint8 or depth.dtype != np.uint16:
            raise ValueError("unexpected Bonn RGB/depth type")

        rectified_rgb = cv2.remap(
            rgb,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        rectified_depth = cv2.remap(
            depth,
            map_x,
            map_y,
            cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        raw_valid = int(np.count_nonzero(depth))
        rectified_valid = int(np.count_nonzero(rectified_depth))
        raw_valid_counts.append(raw_valid)
        rectified_valid_counts.append(rectified_valid)
        retention_ratios.append(rectified_valid / raw_valid)

        allowed_values = np.unique(depth)
        violations = int(
            np.count_nonzero(~np.isin(rectified_depth, allowed_values))
        )
        nearest_value_violations.append(violations)

        raw_alignment.append(alignment_proxy(rgb, depth))
        rectified_alignment.append(
            alignment_proxy(rectified_rgb, rectified_depth)
        )

    raw_proxy_values = [
        item["rgb_edge_near_depth_edge_ratio"]
        for item in raw_alignment
        if item["rgb_edge_near_depth_edge_ratio"] is not None
    ]
    rectified_proxy_values = [
        item["rgb_edge_near_depth_edge_ratio"]
        for item in rectified_alignment
        if item["rgb_edge_near_depth_edge_ratio"] is not None
    ]

    result = {
        "stage": "G2-4B",
        "role": "coordinate-domain and risk-proxy audit only",
        "classification_output": "none",
        "direct_slam_state_mutation": "none",
        "calibration_source": "Bonn official RGB-D Dynamic Dataset page",
        "output_projection": "P=K",
        "rgb_interpolation": "linear",
        "depth_interpolation": "nearest",
        "frames": len(pairs),
        "image_size": [640, 480],
        "in_bounds_source_pixel_ratio": float(np.mean(in_bounds)),
        "out_of_bounds_source_pixel_count": int(
            in_bounds.size - np.count_nonzero(in_bounds)
        ),
        "raw_valid_depth_pixels": summarize(raw_valid_counts),
        "rectified_valid_depth_pixels": summarize(rectified_valid_counts),
        "valid_depth_retention_ratio": summarize(retention_ratios),
        "nearest_depth_value_violations": {
            "total": int(sum(nearest_value_violations)),
            "max_per_frame": int(max(nearest_value_violations)),
        },
        "alignment_proxy": {
            "definition": (
                "fraction of depth-discontinuity pixels within Chebyshev "
                "distance 2 of the top-10% RGB gradient; not alignment GT"
            ),
            "raw": summarize(raw_proxy_values),
            "rectified": summarize(rectified_proxy_values),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as error:
        print("Bonn rectification audit failed: {}".format(error), file=sys.stderr)
        sys.exit(2)
