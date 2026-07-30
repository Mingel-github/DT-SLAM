#!/usr/bin/env python3
"""G2-4R1 offline normal+distance geometric-segment audit.

The segmentation is generated only from a rectified metric depth image and
camera intrinsics. Semantic masks, motion residuals, coarse boxes, and feature
measurements are never used to create or merge segments. Optional boxes and
exact C++ sparse-flow nodes are consumed only after segmentation for a
development-only representation review.

This tool emits no dynamic decision and never mutates SLAM state.
"""

import argparse
import csv
import json
import math
import time
from collections import defaultdict
from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np


CAMERAS = {
    "bonn": {
        "K": np.array(
            [[542.822841, 0.0, 315.593520],
             [0.0, 542.576870, 237.756098],
             [0.0, 0.0, 1.0]],
            dtype=np.float32),
        "D": np.array(
            [0.039903, -0.099343, -0.000730, -0.000144, 0.0],
            dtype=np.float32),
        "depth_scale": 5000.0,
    },
    "tum1": {
        "K": np.array(
            [[517.306408, 0.0, 318.643040],
             [0.0, 516.469215, 255.313989],
             [0.0, 0.0, 1.0]],
            dtype=np.float32),
        "D": np.array(
            [0.262383, -0.953104, -0.005358, 0.002628, 1.163314],
            dtype=np.float32),
        "depth_scale": 5000.0,
    },
}

IMAGE_SIZE = (640, 480)
TAU_PHI = 0.94
NGUYEN_SIGMA_OFFSET_METERS = 0.0012
NGUYEN_SIGMA_QUADRATIC_PER_METER = 0.0019
NGUYEN_DEPTH_OFFSET_METERS = 0.4
BILATERAL_RADIUS_PIXELS = 2
BILATERAL_SIGMA_SPACE_PIXELS = 2.0
BILATERAL_SIGMA_DEPTH_METERS = 0.05
BASELINE_RELATIVE_THRESHOLD = 0.025
BASELINE_ABSOLUTE_THRESHOLD_METERS = 0.08

UNKNOWN_INVALID_DEPTH = -3
UNKNOWN_NORMAL_UNAVAILABLE = -2
GEOMETRIC_BOUNDARY = -1


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"cannot write empty CSV: {path}")
    fieldnames = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def finite_float(row, key):
    text = row.get(key, "")
    if text == "":
        return None
    value = float(text)
    return value if math.isfinite(value) else None


def percentile(values, q):
    values = [value for value in values if value is not None]
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def safe_ratio(numerator, denominator):
    if numerator is None or denominator is None or denominator == 0:
        return None
    return float(numerator/denominator)


def neighbor_view(array, delta_v, delta_u, fill_value):
    """Return an array where output[v,u] is input[v+dv,u+du]."""
    output = np.full_like(array, fill_value)
    height, width = array.shape[:2]
    destination_v0 = max(0, -delta_v)
    destination_v1 = min(height, height-delta_v)
    destination_u0 = max(0, -delta_u)
    destination_u1 = min(width, width-delta_u)
    source_v0 = destination_v0+delta_v
    source_v1 = destination_v1+delta_v
    source_u0 = destination_u0+delta_u
    source_u1 = destination_u1+delta_u
    output[
        destination_v0:destination_v1,
        destination_u0:destination_u1] = array[
            source_v0:source_v1, source_u0:source_u1]
    return output


def masked_bilateral_depth(
        depth,
        radius=BILATERAL_RADIUS_PIXELS,
        sigma_space=BILATERAL_SIGMA_SPACE_PIXELS,
        sigma_depth=BILATERAL_SIGMA_DEPTH_METERS):
    """Deterministic invalid-aware bilateral smoothing in metric depth."""
    if depth.dtype != np.float32 or depth.ndim != 2:
        raise ValueError("depth must be float32 single-channel")
    valid = np.isfinite(depth) & (depth > 0.0)
    numerator = np.zeros_like(depth, dtype=np.float64)
    denominator = np.zeros_like(depth, dtype=np.float64)
    center = depth.astype(np.float64)
    spatial_denominator = 2.0*sigma_space*sigma_space
    range_denominator = 2.0*sigma_depth*sigma_depth
    for delta_v in range(-radius, radius+1):
        for delta_u in range(-radius, radius+1):
            neighbor = neighbor_view(
                center, delta_v, delta_u, np.nan)
            neighbor_valid = neighbor_view(
                valid, delta_v, delta_u, False)
            use = valid & neighbor_valid
            spatial_weight = math.exp(
                -(delta_u*delta_u+delta_v*delta_v)/
                spatial_denominator)
            difference = np.zeros_like(center)
            difference[use] = neighbor[use]-center[use]
            weight = np.zeros_like(center)
            weight[use] = spatial_weight*np.exp(
                -(difference[use]*difference[use])/range_denominator)
            numerator[use] += weight[use]*neighbor[use]
            denominator[use] += weight[use]
    smoothed = np.zeros_like(depth)
    usable = valid & (denominator > 0.0)
    smoothed[usable] = (
        numerator[usable]/denominator[usable]).astype(np.float32)
    return smoothed, usable


def vertex_map(depth, camera_matrix):
    height, width = depth.shape
    u = np.arange(width, dtype=np.float32)[None, :]
    v = np.arange(height, dtype=np.float32)[:, None]
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    vertices = np.zeros((height, width, 3), dtype=np.float32)
    vertices[..., 0] = (u-cx)*depth/fx
    vertices[..., 1] = (v-cy)*depth/fy
    vertices[..., 2] = depth
    return vertices


def central_difference_normals(vertices, valid):
    left = neighbor_view(vertices, 0, -1, np.nan)
    right = neighbor_view(vertices, 0, 1, np.nan)
    up = neighbor_view(vertices, -1, 0, np.nan)
    down = neighbor_view(vertices, 1, 0, np.nan)
    support = (
        valid &
        neighbor_view(valid, 0, -1, False) &
        neighbor_view(valid, 0, 1, False) &
        neighbor_view(valid, -1, 0, False) &
        neighbor_view(valid, 1, 0, False))
    tangent_u = right-left
    tangent_v = down-up
    normals = np.cross(tangent_u, tangent_v)
    magnitude = np.linalg.norm(normals, axis=2)
    normal_valid = support & np.isfinite(magnitude) & (magnitude > 1e-8)
    normalized = np.zeros_like(normals, dtype=np.float32)
    normalized[normal_valid] = (
        normals[normal_valid]/
        magnitude[normal_valid, None]).astype(np.float32)
    # With u increasing right and v increasing down, cross(du,dv) points
    # toward +z for a fronto-parallel surface. This orientation is required
    # by Tateno's signed convexity branch.
    return normalized, normal_valid


def nguyen_axial_sigma(depth):
    return (
        NGUYEN_SIGMA_OFFSET_METERS +
        NGUYEN_SIGMA_QUADRATIC_PER_METER*
        np.square(depth-NGUYEN_DEPTH_OFFSET_METERS))


def normal_distance_segments(depth, camera_matrix):
    total_start = time.perf_counter()
    smooth_start = time.perf_counter()
    smoothed, smooth_valid = masked_bilateral_depth(depth)
    smooth_end = time.perf_counter()

    normal_start = time.perf_counter()
    vertices = vertex_map(smoothed, camera_matrix)
    normals, normal_valid = central_difference_normals(
        vertices, smooth_valid)
    neighbor_directions = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1))
    comparison_valid = normal_valid.copy()
    for delta_v, delta_u in neighbor_directions:
        comparison_valid &= neighbor_view(
            normal_valid, delta_v, delta_u, False)
    normal_end = time.perf_counter()

    edge_start = time.perf_counter()
    phi = np.ones(depth.shape, dtype=np.float32)
    gamma = np.zeros(depth.shape, dtype=np.float32)
    for delta_v, delta_u in neighbor_directions:
        neighbor_vertices = neighbor_view(
            vertices, delta_v, delta_u, np.nan)
        neighbor_normals = neighbor_view(
            normals, delta_v, delta_u, np.nan)
        delta = neighbor_vertices-vertices
        signed_point_plane = np.sum(delta*normals, axis=2)
        normal_dot = np.sum(neighbor_normals*normals, axis=2)
        phi_i = np.where(signed_point_plane > 0.0, 1.0, normal_dot)
        phi = np.minimum(phi, phi_i.astype(np.float32))
        gamma = np.maximum(
            gamma, np.abs(signed_point_plane).astype(np.float32))
    phi[~comparison_valid] = np.nan
    gamma[~comparison_valid] = np.nan
    normal_boundary = comparison_valid & (phi < TAU_PHI)
    sigma = nguyen_axial_sigma(smoothed)
    distance_boundary = comparison_valid & (gamma > sigma)
    combined_boundary = normal_boundary | distance_boundary
    edge_end = time.perf_counter()

    component_start = time.perf_counter()
    segmentable = comparison_valid & ~combined_boundary
    component_count, raw_components = cv2.connectedComponents(
        segmentable.astype(np.uint8),
        connectivity=4, ltype=cv2.CV_32S)
    labels = np.full(depth.shape, UNKNOWN_INVALID_DEPTH, dtype=np.int32)
    input_valid = np.isfinite(depth) & (depth > 0.0)
    labels[input_valid & ~comparison_valid] = UNKNOWN_NORMAL_UNAVAILABLE
    labels[combined_boundary] = GEOMETRIC_BOUNDARY
    labels[segmentable] = raw_components[segmentable]-1
    region_sizes = np.bincount(
        raw_components[segmentable],
        minlength=component_count)[1:].astype(np.int64)
    component_end = time.perf_counter()

    stats = {
        "valid_depth_pixels": int(np.count_nonzero(input_valid)),
        "normal_valid_pixels": int(np.count_nonzero(normal_valid)),
        "comparison_valid_pixels":
            int(np.count_nonzero(comparison_valid)),
        "normal_boundary_pixels":
            int(np.count_nonzero(normal_boundary)),
        "distance_boundary_pixels":
            int(np.count_nonzero(distance_boundary)),
        "combined_boundary_pixels":
            int(np.count_nonzero(combined_boundary)),
        "segmentable_pixels": int(np.count_nonzero(segmentable)),
        "region_count": int(len(region_sizes)),
        "singleton_region_count":
            int(np.count_nonzero(region_sizes == 1)),
        "small_region_count":
            int(np.count_nonzero(region_sizes <= 64)),
        "largest_region_pixels":
            int(np.max(region_sizes)) if len(region_sizes) else 0,
        "top_five_region_pixels":
            int(np.sum(np.sort(region_sizes)[-5:])),
        "smoothing_ms": (smooth_end-smooth_start)*1000.0,
        "vertex_normal_ms": (normal_end-normal_start)*1000.0,
        "edge_ms": (edge_end-edge_start)*1000.0,
        "connected_component_ms":
            (component_end-component_start)*1000.0,
        "total_ms": (component_end-total_start)*1000.0,
    }
    return {
        "smoothed_depth": smoothed,
        "normal_valid": normal_valid,
        "comparison_valid": comparison_valid,
        "phi": phi,
        "gamma": gamma,
        "normal_boundary": normal_boundary,
        "distance_boundary": distance_boundary,
        "combined_boundary": combined_boundary,
        "labels": labels,
        "region_sizes": region_sizes,
        "stats": stats,
    }


def baseline_depth_components(depth):
    """Mirror the frozen G2-3R0 depth-discontinuity partition."""
    start = time.perf_counter()
    valid = np.isfinite(depth) & (depth > 0.0)
    boundary = np.zeros(depth.shape, dtype=bool)
    directions = ((0, -1), (0, 1), (-1, 0), (1, 0))
    for delta_v, delta_u in directions:
        neighbor = neighbor_view(depth, delta_v, delta_u, np.nan)
        neighbor_valid = neighbor_view(valid, delta_v, delta_u, False)
        threshold = np.maximum(
            BASELINE_RELATIVE_THRESHOLD*depth,
            BASELINE_ABSOLUTE_THRESHOLD_METERS)
        boundary |= (
            valid & neighbor_valid &
            (np.abs(neighbor-depth) > threshold))
    segmentable = valid & ~boundary
    component_count, raw_components = cv2.connectedComponents(
        segmentable.astype(np.uint8),
        connectivity=4, ltype=cv2.CV_32S)
    labels = np.full(depth.shape, UNKNOWN_INVALID_DEPTH, dtype=np.int32)
    labels[boundary] = GEOMETRIC_BOUNDARY
    labels[segmentable] = raw_components[segmentable]-1
    sizes = np.bincount(
        raw_components[segmentable],
        minlength=component_count)[1:].astype(np.int64)
    end = time.perf_counter()
    return {
        "labels": labels,
        "boundary": boundary,
        "region_sizes": sizes,
        "stats": {
            "valid_depth_pixels": int(np.count_nonzero(valid)),
            "boundary_pixels": int(np.count_nonzero(boundary)),
            "segmentable_pixels": int(np.count_nonzero(segmentable)),
            "region_count": int(len(sizes)),
            "singleton_region_count":
                int(np.count_nonzero(sizes == 1)),
            "small_region_count":
                int(np.count_nonzero(sizes <= 64)),
            "largest_region_pixels":
                int(np.max(sizes)) if len(sizes) else 0,
            "top_five_region_pixels":
                int(np.sum(np.sort(sizes)[-5:])),
            "total_ms": (end-start)*1000.0,
        },
    }


def run_self_test():
    camera = np.array(
        [[525.0, 0.0, 3.5],
         [0.0, 525.0, 3.5],
         [0.0, 0.0, 1.0]],
        dtype=np.float32)

    plane = np.ones((12, 16), dtype=np.float32)
    first = normal_distance_segments(plane, camera)
    second = normal_distance_segments(plane, camera)
    assert np.array_equal(first["labels"], second["labels"])
    assert first["stats"]["region_count"] == 1
    assert first["stats"]["combined_boundary_pixels"] == 0
    assert first["stats"]["segmentable_pixels"] > 0

    step = np.ones((12, 16), dtype=np.float32)
    step[:, 8:] = 2.0
    segmented = normal_distance_segments(step, camera)
    assert segmented["stats"]["region_count"] >= 2
    assert segmented["stats"]["combined_boundary_pixels"] > 0

    invalid = np.ones((12, 16), dtype=np.float32)
    invalid[:, 7:9] = 0.0
    segmented = normal_distance_segments(invalid, camera)
    assert np.all(
        segmented["labels"][:, 7:9] == UNKNOWN_INVALID_DEPTH)
    assert segmented["stats"]["region_count"] >= 2

    baseline = baseline_depth_components(step)
    assert baseline["stats"]["region_count"] == 2
    color = labels_to_color(
        np.asarray(
            [[UNKNOWN_INVALID_DEPTH, UNKNOWN_NORMAL_UNAVAILABLE,
              GEOMETRIC_BOUNDARY, 0, 1]],
            dtype=np.int32))
    assert color.shape == (1, 5, 3)
    assert np.array_equal(color[0, 0], (0, 0, 0))
    assert np.array_equal(color[0, 1], (80, 80, 80))
    assert np.array_equal(color[0, 2], (255, 255, 255))
    print("[G2-4R1 normal+distance self-test] PASS")


class ImageSource:
    def __init__(self, archive_path=None, archive_root=None, dataset_root=None):
        self.archive = (
            ZipFile(archive_path) if archive_path is not None else None)
        self.archive_root = archive_root.rstrip("/") if archive_root else None
        self.dataset_root = dataset_root

    def close(self):
        if self.archive is not None:
            self.archive.close()

    def read_encoded(self, relative_path):
        if self.archive is not None:
            member = f"{self.archive_root}/{relative_path}"
            return np.frombuffer(self.archive.read(member), dtype=np.uint8)
        return np.fromfile(self.dataset_root/relative_path, dtype=np.uint8)

    def read_depth(self, relative_path, map_x, map_y, depth_scale):
        raw = cv2.imdecode(
            self.read_encoded(relative_path), cv2.IMREAD_UNCHANGED)
        if raw is None or raw.dtype != np.uint16:
            raise RuntimeError(f"invalid uint16 depth: {relative_path}")
        rectified = cv2.remap(
            raw, map_x, map_y, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return rectified.astype(np.float32)/depth_scale

    def read_rgb(self, relative_path, map_x, map_y):
        image = cv2.imdecode(
            self.read_encoded(relative_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"invalid RGB image: {relative_path}")
        return cv2.remap(
            image, map_x, map_y, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def read_associations(path, frame_limit):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 4:
                raise RuntimeError(f"invalid association line: {line}")
            rows.append({
                "frame": len(rows),
                "rgb_relative": fields[1],
                "depth_relative": fields[3],
            })
            if frame_limit and len(rows) >= frame_limit:
                break
    return rows


def read_candidates(path, frame_limit):
    rows = []
    for raw in read_csv(path):
        if not raw.get("rgb_relative") or not raw.get("depth_relative"):
            continue
        rows.append({
            "frame": int(raw["frame"]),
            "rgb_relative": raw["rgb_relative"],
            "depth_relative": raw["depth_relative"],
        })
        if frame_limit and len(rows) >= frame_limit:
            break
    return rows


def read_boxes(path, export_name=None):
    boxes = {}
    for row in read_csv(path):
        if export_name and row.get("export_name") != export_name:
            continue
        if row.get("visibility") == "absent":
            continue
        frame_text = row.get("source_frame", row.get("frame", ""))
        if frame_text == "":
            continue
        if row.get("bbox_x", "") != "":
            keys = (
                ("x", "bbox_x"), ("y", "bbox_y"),
                ("width", "bbox_width"), ("height", "bbox_height"))
        else:
            keys = (
                ("x", "x"), ("y", "y"),
                ("width", "width"), ("height", "height"))
        if any(row.get(source, "") == "" for _, source in keys):
            continue
        boxes[int(frame_text)] = {
            target: int(row[source]) for target, source in keys}
    return boxes


def read_measured_nodes(path):
    by_frame = defaultdict(list)
    for row in read_csv(path):
        if (
                "dynamic_decision" in row and
                row["dynamic_decision"] != "none") or (
                "direct_slam_state_mutation" in row and
                row["direct_slam_state_mutation"] != "none"):
            raise RuntimeError("input node CSV violates shadow invariant")
        if row.get("evidence_state") != "measured":
            continue
        u = finite_float(row, "u_current")
        v = finite_float(row, "v_current")
        residual = finite_float(row, "slam_residual_magnitude_px")
        if residual is None:
            residual = finite_float(row, "flow_residual_magnitude_px")
        if u is None or v is None or residual is None:
            raise RuntimeError("measured node lacks finite coordinate/residual")
        by_frame[int(row["frame"])].append({
            "feature_index": int(row["feature_index"]),
            "u": u,
            "v": v,
            "residual": residual,
        })
    return by_frame


def bbox_mask(shape, box):
    mask = np.zeros(shape, dtype=bool)
    x0 = max(0, box["x"])
    y0 = max(0, box["y"])
    x1 = min(shape[1], box["x"]+box["width"])
    y1 = min(shape[0], box["y"]+box["height"])
    if x1 > x0 and y1 > y0:
        mask[y0:y1, x0:x1] = True
    return mask


def oracle_segment(labels, region_sizes, depth, box):
    box_pixels = bbox_mask(labels.shape, box)
    valid_bbox = box_pixels & np.isfinite(depth) & (depth > 0.0)
    bbox_valid_count = int(np.count_nonzero(valid_bbox))
    segmentable_bbox = valid_bbox & (labels >= 0)
    present_labels, intersections = np.unique(
        labels[segmentable_bbox], return_counts=True)
    candidates = []
    for label, intersection in zip(present_labels, intersections):
        label = int(label)
        intersection = int(intersection)
        area = int(region_sizes[label])
        union = area+bbox_valid_count-intersection
        iou = safe_ratio(intersection, union)
        candidates.append((iou, intersection, -area, -label, label))
    selected = max(candidates)[-1] if candidates else None
    selected_intersection = (
        int(np.count_nonzero(valid_bbox & (labels == selected)))
        if selected is not None else 0)
    selected_area = (
        int(region_sizes[selected]) if selected is not None else 0)

    sorted_intersections = sorted(
        (int(value) for value in intersections), reverse=True)
    cumulative = 0
    count_50 = None
    count_80 = None
    for index, value in enumerate(sorted_intersections, start=1):
        cumulative += value
        coverage = safe_ratio(cumulative, bbox_valid_count)
        if count_50 is None and coverage is not None and coverage >= 0.5:
            count_50 = index
        if count_80 is None and coverage is not None and coverage >= 0.8:
            count_80 = index
            break
    return {
        "selected_label": selected,
        "bbox_valid_depth_pixels": bbox_valid_count,
        "bbox_segmentable_pixels": int(np.count_nonzero(segmentable_bbox)),
        "selected_region_pixels": selected_area,
        "selected_bbox_pixels": selected_intersection,
        "selected_bbox_iou": (
            safe_ratio(
                selected_intersection,
                selected_area+bbox_valid_count-selected_intersection)
            if selected is not None else None),
        "selected_bbox_coverage":
            safe_ratio(selected_intersection, bbox_valid_count),
        "selected_bbox_purity":
            safe_ratio(selected_intersection, selected_area),
        "segments_for_50pct_bbox_coverage": count_50,
        "segments_for_80pct_bbox_coverage": count_80,
    }


def audit_nodes(nodes, labels, selected_label, box):
    selected = []
    inside = []
    background = []
    selected_inside = []
    for node in nodes:
        u = int(round(node["u"]))
        v = int(round(node["v"]))
        label = (
            int(labels[v, u])
            if 0 <= u < labels.shape[1] and 0 <= v < labels.shape[0]
            else UNKNOWN_INVALID_DEPTH)
        is_inside = (
            box["x"] <= node["u"] < box["x"]+box["width"] and
            box["y"] <= node["v"] < box["y"]+box["height"])
        if is_inside:
            inside.append(node["residual"])
        if selected_label is not None and label == selected_label:
            selected.append(node["residual"])
            if is_inside:
                selected_inside.append(node["residual"])
        elif label >= 0 and not is_inside:
            background.append(node["residual"])
    selected_median = percentile(selected, 50)
    inside_median = percentile(inside, 50)
    background_median = percentile(background, 50)
    return {
        "feature_count": len(nodes),
        "inside_feature_count": len(inside),
        "selected_feature_count": len(selected),
        "selected_inside_feature_count": len(selected_inside),
        "selected_feature_purity":
            safe_ratio(len(selected_inside), len(selected)),
        "selected_background_feature_leakage":
            len(selected)-len(selected_inside),
        "selected_residual_median_px": selected_median,
        "inside_residual_median_px": inside_median,
        "background_residual_median_px": background_median,
        "selected_over_background_median":
            safe_ratio(selected_median, background_median),
        "inside_over_background_median":
            safe_ratio(inside_median, background_median),
        "selected_gt_background": (
            None if selected_median is None or background_median is None
            else int(selected_median > background_median)),
        "inside_gt_background": (
            None if inside_median is None or background_median is None
            else int(inside_median > background_median)),
    }


def method_frame_row(prefix, labels, region_sizes, depth, box, nodes):
    oracle = oracle_segment(labels, region_sizes, depth, box)
    node_audit = audit_nodes(
        nodes, labels, oracle["selected_label"], box)
    row = {}
    for key, value in oracle.items():
        row[f"{prefix}_{key}"] = value
    for key, value in node_audit.items():
        row[f"{prefix}_{key}"] = value
    return row


def segment_rows(frame, method, labels, region_sizes, depth, box):
    rows = []
    if box is None:
        box_pixels = np.zeros(labels.shape, dtype=bool)
        bbox_valid = box_pixels
    else:
        box_pixels = bbox_mask(labels.shape, box)
        bbox_valid = box_pixels & np.isfinite(depth) & (depth > 0.0)
    bbox_valid_count = int(np.count_nonzero(bbox_valid))
    for label in range(len(region_sizes)):
        label_mask = labels == label
        area = int(region_sizes[label])
        intersection = int(np.count_nonzero(label_mask & bbox_valid))
        rows.append({
            "frame": frame,
            "method": method,
            "region_label": label,
            "region_pixels": area,
            "bbox_valid_intersection_pixels": intersection,
            "bbox_coverage": safe_ratio(intersection, bbox_valid_count),
            "bbox_purity": safe_ratio(intersection, area),
            "dynamic_decision": "none",
            "direct_slam_state_mutation": "none",
            "proxy_is_not_gt": "true",
        })
    return rows


def labels_to_color(labels):
    image = np.zeros((*labels.shape, 3), dtype=np.uint8)
    image[labels == UNKNOWN_INVALID_DEPTH] = (0, 0, 0)
    image[labels == UNKNOWN_NORMAL_UNAVAILABLE] = (80, 80, 80)
    image[labels == GEOMETRIC_BOUNDARY] = (255, 255, 255)
    positive = labels >= 0
    values = labels[positive].astype(np.uint32)+1
    image[positive, 0] = ((values*37) % 223+16).astype(np.uint8)
    image[positive, 1] = ((values*73) % 223+16).astype(np.uint8)
    image[positive, 2] = ((values*109) % 223+16).astype(np.uint8)
    return image


def visualization(rgb, labels, box, title):
    colors = labels_to_color(labels)
    blended = cv2.addWeighted(rgb, 0.45, colors, 0.55, 0.0)
    if box is not None:
        cv2.rectangle(
            blended,
            (box["x"], box["y"]),
            (box["x"]+box["width"], box["y"]+box["height"]),
            (0, 255, 255), 2)
    cv2.putText(
        blended, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(
        blended, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return blended


def save_contact_sheet(path, images, columns=2):
    if not images:
        return
    thumb_size = (640, 240)
    thumbs = [cv2.resize(image, thumb_size) for image in images]
    rows = []
    for start in range(0, len(thumbs), columns):
        row = thumbs[start:start+columns]
        while len(row) < columns:
            row.append(np.zeros_like(thumbs[0]))
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows))


def pure_frame_row(frame, normal_result, baseline_result):
    normal_stats = normal_result["stats"]
    baseline_stats = baseline_result["stats"]
    return {
        "frame": frame,
        "normal_valid_depth_pixels": normal_stats["valid_depth_pixels"],
        "normal_comparison_valid_pixels":
            normal_stats["comparison_valid_pixels"],
        "normal_unknown_fraction_of_valid": safe_ratio(
            normal_stats["valid_depth_pixels"]-
            normal_stats["comparison_valid_pixels"],
            normal_stats["valid_depth_pixels"]),
        "normal_boundary_pixels":
            normal_stats["combined_boundary_pixels"],
        "normal_boundary_fraction_of_valid": safe_ratio(
            normal_stats["combined_boundary_pixels"],
            normal_stats["valid_depth_pixels"]),
        "normal_region_count": normal_stats["region_count"],
        "normal_small_region_fraction": safe_ratio(
            normal_stats["small_region_count"],
            normal_stats["region_count"]),
        "normal_largest_region_fraction_of_segmentable": safe_ratio(
            normal_stats["largest_region_pixels"],
            normal_stats["segmentable_pixels"]),
        "normal_top_five_fraction_of_segmentable": safe_ratio(
            normal_stats["top_five_region_pixels"],
            normal_stats["segmentable_pixels"]),
        "normal_smoothing_ms": normal_stats["smoothing_ms"],
        "normal_vertex_normal_ms": normal_stats["vertex_normal_ms"],
        "normal_edge_ms": normal_stats["edge_ms"],
        "normal_connected_component_ms":
            normal_stats["connected_component_ms"],
        "normal_total_ms": normal_stats["total_ms"],
        "baseline_boundary_pixels": baseline_stats["boundary_pixels"],
        "baseline_boundary_fraction_of_valid": safe_ratio(
            baseline_stats["boundary_pixels"],
            baseline_stats["valid_depth_pixels"]),
        "baseline_region_count": baseline_stats["region_count"],
        "baseline_small_region_fraction": safe_ratio(
            baseline_stats["small_region_count"],
            baseline_stats["region_count"]),
        "baseline_largest_region_fraction_of_segmentable": safe_ratio(
            baseline_stats["largest_region_pixels"],
            baseline_stats["segmentable_pixels"]),
        "baseline_top_five_fraction_of_segmentable": safe_ratio(
            baseline_stats["top_five_region_pixels"],
            baseline_stats["segmentable_pixels"]),
        "baseline_total_ms": baseline_stats["total_ms"],
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "proxy_is_not_gt": "true",
    }


def aggregate(sequence, frame_rows, has_proxy):
    summary = {
        "sequence": sequence,
        "method_identity":
            "[L/A/S/H] offline Tateno-style frame-wise normal+distance "
            "geometric segmentation representation audit",
        "frame_count": len(frame_rows),
        "tau_phi": TAU_PHI,
        "bilateral_radius_pixels": BILATERAL_RADIUS_PIXELS,
        "bilateral_sigma_space_pixels": BILATERAL_SIGMA_SPACE_PIXELS,
        "bilateral_sigma_depth_meters": BILATERAL_SIGMA_DEPTH_METERS,
        "nguyen_sigma_model":
            "sigma_z=0.0012+0.0019*(z-0.4)^2 meters",
        "normal_unknown_fraction_median": percentile(
            [row["normal_unknown_fraction_of_valid"] for row in frame_rows],
            50),
        "normal_boundary_fraction_median": percentile(
            [row["normal_boundary_fraction_of_valid"] for row in frame_rows],
            50),
        "normal_region_count_median": percentile(
            [row["normal_region_count"] for row in frame_rows], 50),
        "normal_small_region_fraction_median": percentile(
            [row["normal_small_region_fraction"] for row in frame_rows], 50),
        "normal_largest_region_fraction_median": percentile(
            [row["normal_largest_region_fraction_of_segmentable"]
             for row in frame_rows], 50),
        "normal_total_ms_median": percentile(
            [row["normal_total_ms"] for row in frame_rows], 50),
        "normal_total_ms_p90": percentile(
            [row["normal_total_ms"] for row in frame_rows], 90),
        "baseline_region_count_median": percentile(
            [row["baseline_region_count"] for row in frame_rows], 50),
        "baseline_largest_region_fraction_median": percentile(
            [row["baseline_largest_region_fraction_of_segmentable"]
             for row in frame_rows], 50),
        "baseline_total_ms_median": percentile(
            [row["baseline_total_ms"] for row in frame_rows], 50),
        "dynamic_decision": "none",
        "direct_slam_state_mutation": "none",
        "proxy_is_not_gt": True,
    }
    if has_proxy:
        comparable = [
            row for row in frame_rows
            if row.get("normal_selected_gt_background") is not None]
        baseline_comparable = [
            row for row in frame_rows
            if row.get("baseline_selected_gt_background") is not None]
        summary.update({
            "proxy_frame_count": len(frame_rows),
            "normal_comparable_frame_count": len(comparable),
            "normal_selected_gt_background_count": sum(
                row["normal_selected_gt_background"]
                for row in comparable),
            "baseline_comparable_frame_count": len(baseline_comparable),
            "baseline_selected_gt_background_count": sum(
                row["baseline_selected_gt_background"]
                for row in baseline_comparable),
            "normal_selected_background_feature_leakage_median": percentile(
                [row["normal_selected_background_feature_leakage"]
                 for row in frame_rows], 50),
            "baseline_selected_background_feature_leakage_median": percentile(
                [row["baseline_selected_background_feature_leakage"]
                 for row in frame_rows], 50),
            "normal_selected_bbox_iou_median": percentile(
                [row["normal_selected_bbox_iou"] for row in frame_rows], 50),
            "baseline_selected_bbox_iou_median": percentile(
                [row["baseline_selected_bbox_iou"] for row in frame_rows],
                50),
            "normal_segments_for_80pct_bbox_coverage_median": percentile(
                [row["normal_segments_for_80pct_bbox_coverage"]
                 for row in frame_rows], 50),
            "baseline_segments_for_80pct_bbox_coverage_median": percentile(
                [row["baseline_segments_for_80pct_bbox_coverage"]
                 for row in frame_rows], 50),
        })
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--sequence")
    parser.add_argument("--camera", choices=sorted(CAMERAS))
    parser.add_argument("--candidate-csv", type=Path)
    parser.add_argument("--association-file", type=Path)
    parser.add_argument("--frame-limit", type=int, default=0)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--archive-root")
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--node-csv", type=Path)
    parser.add_argument("--bbox-csv", type=Path)
    parser.add_argument("--bbox-export-name")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--save-images", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return
    for name in ("sequence", "camera", "output_dir"):
        if getattr(args, name) is None:
            parser.error(f"--{name.replace('_', '-')} is required")
    if (args.candidate_csv is None) == (args.association_file is None):
        parser.error(
            "provide exactly one of --candidate-csv or --association-file")
    if (args.archive is None) == (args.dataset_root is None):
        parser.error(
            "provide exactly one archive source or --dataset-root")
    if args.archive is not None and not args.archive_root:
        parser.error("--archive-root is required with --archive")
    if (args.node_csv is None) != (args.bbox_csv is None):
        parser.error("--node-csv and --bbox-csv must be provided together")

    camera = CAMERAS[args.camera]
    map_x, map_y = cv2.initUndistortRectifyMap(
        camera["K"], camera["D"], np.eye(3, dtype=np.float32),
        camera["K"], IMAGE_SIZE, cv2.CV_32FC1)
    frames = (
        read_candidates(args.candidate_csv, args.frame_limit)
        if args.candidate_csv is not None
        else read_associations(args.association_file, args.frame_limit))
    nodes = (
        read_measured_nodes(args.node_csv)
        if args.node_csv is not None else {})
    boxes = (
        read_boxes(args.bbox_csv, args.bbox_export_name)
        if args.bbox_csv is not None else {})
    if args.node_csv is not None:
        frames = [
            row for row in frames
            if row["frame"] in nodes and row["frame"] in boxes]
    if not frames:
        raise RuntimeError("no frames remain after input join")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    image_dir = args.output_dir/"images"
    if args.save_images:
        image_dir.mkdir(parents=True, exist_ok=True)
    source = ImageSource(
        archive_path=args.archive,
        archive_root=args.archive_root,
        dataset_root=args.dataset_root)
    frame_rows = []
    per_segment_rows = []
    contact_images = []
    try:
        for row in frames:
            frame = row["frame"]
            depth = source.read_depth(
                row["depth_relative"], map_x, map_y,
                camera["depth_scale"])
            normal_result = normal_distance_segments(depth, camera["K"])
            baseline_result = baseline_depth_components(depth)
            frame_row = pure_frame_row(
                frame, normal_result, baseline_result)
            frame_row.update({
                "camera_domain": args.camera,
                "noise_model_transfer_status": "unvalidated_transfer",
                "noise_model_out_of_domain":
                    "true" if args.camera == "bonn" else "false",
            })
            box = boxes.get(frame)
            if box is not None:
                frame_row.update(method_frame_row(
                    "normal",
                    normal_result["labels"],
                    normal_result["region_sizes"],
                    depth, box, nodes[frame]))
                frame_row.update(method_frame_row(
                    "baseline",
                    baseline_result["labels"],
                    baseline_result["region_sizes"],
                    depth, box, nodes[frame]))
            frame_rows.append(frame_row)
            per_segment_rows.extend(segment_rows(
                frame, "normal_distance",
                normal_result["labels"],
                normal_result["region_sizes"], depth, box))
            per_segment_rows.extend(segment_rows(
                frame, "g2_3r0_baseline",
                baseline_result["labels"],
                baseline_result["region_sizes"], depth, box))

            if args.save_images:
                rgb = source.read_rgb(
                    row["rgb_relative"], map_x, map_y)
                normal_image = visualization(
                    rgb, normal_result["labels"], box,
                    f"{args.sequence} f{frame} normal+distance")
                baseline_image = visualization(
                    rgb, baseline_result["labels"], box,
                    f"{args.sequence} f{frame} G2-3R0")
                combined = np.hstack((normal_image, baseline_image))
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_paired.png"),
                    combined)
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_valid_depth.png"),
                    (np.isfinite(depth) & (depth > 0.0)).astype(
                        np.uint8)*255)
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_normal_valid.png"),
                    normal_result["normal_valid"].astype(np.uint8)*255)
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_normal_boundary.png"),
                    normal_result["normal_boundary"].astype(np.uint8)*255)
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_distance_boundary.png"),
                    normal_result["distance_boundary"].astype(np.uint8)*255)
                cv2.imwrite(
                    str(image_dir/f"frame_{frame:06d}_combined_boundary.png"),
                    normal_result["combined_boundary"].astype(np.uint8)*255)
                contact_images.append(combined)
    finally:
        source.close()

    write_csv(args.output_dir/"per_frame.csv", frame_rows)
    write_csv(args.output_dir/"paired_comparison.csv", frame_rows)
    write_csv(args.output_dir/"per_segment.csv", per_segment_rows)
    summary = aggregate(
        args.sequence, frame_rows, args.node_csv is not None)
    summary.update({
        "camera_domain": args.camera,
        "implementation_language": "Python/NumPy/OpenCV offline audit",
        "noise_model_transfer_status": "unvalidated_transfer",
        "noise_model_out_of_domain": args.camera == "bonn",
    })
    with open(
            args.output_dir/"summary.json", "w",
            encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    if args.save_images:
        save_contact_sheet(
            args.output_dir/"paired_contact_sheet.png",
            contact_images)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
