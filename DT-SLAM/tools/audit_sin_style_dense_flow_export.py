#!/usr/bin/env python3
"""Audit raw SInDSLAM dense-flow residual evidence; never label dynamics."""

import argparse
import csv
import json
import struct
from pathlib import Path

import cv2
import numpy as np


def read_flo(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        magic = struct.unpack("<f", stream.read(4))[0]
        if magic != 202021.25:
            raise ValueError(f"{path}: invalid .flo magic {magic}")
        width, height = struct.unpack("<ii", stream.read(8))
        values = np.frombuffer(stream.read(), dtype="<f4")
    expected = width * height * 2
    if values.size != expected:
        raise ValueError(f"{path}: {values.size} floats != {expected}")
    return values.reshape(height, width, 2)


def scalar(storage, name, integer=False):
    value = storage.getNode(name).real()
    return int(round(value)) if integer else float(value)


def audit_frame(root: Path, frame_index: int):
    errors = []
    stem = root / f"frame_{frame_index:06d}"
    raw = read_flo(Path(str(stem) + "_brox_internal_s06.flo"))
    refined = read_flo(Path(str(stem) + "_flow_refined_full.flo"))
    residual = read_flo(Path(str(stem) + "_residual_full.flo"))
    normalized = cv2.imread(
        str(stem) + "_residual_normalized.png", cv2.IMREAD_UNCHANGED
    )
    low_mask = cv2.imread(
        str(stem) + "_threshold_low.png", cv2.IMREAD_UNCHANGED
    )
    high_mask = cv2.imread(
        str(stem) + "_threshold_high.png", cv2.IMREAD_UNCHANGED
    )
    storage = cv2.FileStorage(
        str(stem) + "_meta.yml", cv2.FileStorage_READ
    )
    if not storage.isOpened():
        raise ValueError(f"{stem}: metadata cannot be opened")
    homography = storage.getNode("homography").mat()
    metadata = {
        "frame_index": scalar(storage, "frame_index", True),
        "available": scalar(storage, "available", True),
        "image_scale": scalar(storage, "image_scale"),
        "intended_reference_lag": scalar(
            storage, "intended_reference_lag", True
        ),
        "reference_index": scalar(storage, "reference_index", True),
        "actual_reference_lag": scalar(storage, "actual_reference_lag", True),
        "large_motion": scalar(storage, "large_motion", True),
        "max_flow_px": scalar(storage, "max_flow_px"),
        "max_residual_px": scalar(storage, "max_residual_px"),
        "low_threshold_u8": scalar(storage, "low_threshold_u8"),
        "high_threshold_u8": scalar(storage, "high_threshold_u8"),
        "low_threshold_px": scalar(storage, "low_threshold_px"),
        "high_threshold_px": scalar(storage, "high_threshold_px"),
        "low_pixels": scalar(storage, "low_pixels", True),
        "high_pixels": scalar(storage, "high_pixels", True),
    }
    storage.release()

    if raw.shape != (288, 384, 2):
        errors.append(f"raw shape {raw.shape} != (288,384,2)")
    if refined.shape != (480, 640, 2) or residual.shape != refined.shape:
        errors.append("full-resolution flow/residual shape mismatch")
    for name, image in (
        ("normalized", normalized),
        ("low_mask", low_mask),
        ("high_mask", high_mask),
    ):
        if image is None or image.shape != (480, 640) or image.dtype != np.uint8:
            errors.append(f"{name} is not aligned CV_8U")
    if homography is None or homography.shape != (3, 3) or not np.isfinite(homography).all():
        errors.append("homography invalid")
    if not np.isfinite(raw).all() or not np.isfinite(refined).all():
        errors.append("flow contains non-finite values")
    if not np.isfinite(residual).all():
        errors.append("residual contains non-finite values")

    residual_recompute_error = None
    magnitude_recompute_error_u8 = None
    if not errors:
        rows, cols = np.indices((480, 640), dtype=np.float64)
        denominator = (
            homography[2, 0] * cols
            + homography[2, 1] * rows
            + homography[2, 2]
        )
        if np.any(np.abs(denominator) < 1e-12):
            errors.append("homography has near-zero projection denominator")
        else:
            past_x = (
                homography[0, 0] * cols
                + homography[0, 1] * rows
                + homography[0, 2]
            ) / denominator
            past_y = (
                homography[1, 0] * cols
                + homography[1, 1] * rows
                + homography[1, 2]
            ) / denominator
            induced = np.stack((cols - past_x, rows - past_y), axis=-1)
            recomputed = refined.astype(np.float64) - induced
            residual_recompute_error = float(
                np.max(np.abs(recomputed - residual.astype(np.float64)))
            )
            if residual_recompute_error > 1e-4:
                errors.append(
                    f"residual reconstruction error {residual_recompute_error}"
                )
        magnitude = np.linalg.norm(residual.astype(np.float64), axis=2)
        max_residual = float(np.max(magnitude))
        if abs(max_residual - metadata["max_residual_px"]) > 1e-4:
            errors.append("max residual metadata mismatch")
        max_flow = float(np.max(np.linalg.norm(refined.astype(np.float64), axis=2)))
        if abs(max_flow - metadata["max_flow_px"]) > 1e-4:
            errors.append("max flow metadata mismatch")
        if max_residual > 0:
            expected_normalized = np.clip(
                np.rint(magnitude * 255.0 / max_residual), 0, 255
            ).astype(np.uint8)
            magnitude_recompute_error_u8 = int(
                np.max(
                    np.abs(
                        expected_normalized.astype(np.int16)
                        - normalized.astype(np.int16)
                    )
                )
            )
            if magnitude_recompute_error_u8 > 1:
                errors.append(
                    f"normalized residual error {magnitude_recompute_error_u8}"
                )
        expected_low = normalized > metadata["low_threshold_u8"]
        expected_high = normalized > metadata["high_threshold_u8"]
        if not np.array_equal(low_mask > 0, expected_low):
            errors.append("low threshold mask mismatch")
        if not np.array_equal(high_mask > 0, expected_high):
            errors.append("high threshold mask mismatch")
        if np.any((high_mask > 0) & ~(low_mask > 0)):
            errors.append("high mask is not a subset of low mask")
        if int(np.count_nonzero(low_mask)) != metadata["low_pixels"]:
            errors.append("low pixel count mismatch")
        if int(np.count_nonzero(high_mask)) != metadata["high_pixels"]:
            errors.append("high pixel count mismatch")
        if metadata["intended_reference_lag"] not in (1, 2):
            errors.append("reference lag is not 1 or 2")
        if frame_index - metadata["reference_index"] != metadata["actual_reference_lag"]:
            errors.append("actual reference lag mismatch")

    return {
        "frame_index": frame_index,
        "metadata": metadata,
        "residual_recompute_max_abs": residual_recompute_error,
        "normalized_recompute_max_abs_u8": magnitude_recompute_error_u8,
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--expected-first", required=True, type=int)
    parser.add_argument("--expected-last", required=True, type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    manifest_path = args.root / "manifest.csv"
    with manifest_path.open(newline="", encoding="utf-8") as stream:
        manifest = list(csv.DictReader(stream))
    expected_indices = list(range(args.expected_first, args.expected_last + 1))
    manifest_indices = [int(row["frame_index"]) for row in manifest]
    global_errors = []
    if manifest_indices != expected_indices:
        global_errors.append(
            f"manifest indices {manifest_indices} != {expected_indices}"
        )
    frames = []
    for index in expected_indices:
        try:
            frames.append(audit_frame(args.root, index))
        except Exception as error:  # report all frames in one pass
            frames.append({"frame_index": index, "errors": [str(error)]})
    all_errors = global_errors + [
        f"frame {frame['frame_index']}: {error}"
        for frame in frames
        for error in frame["errors"]
    ]
    summary = {
        "identity": "raw SIn dense-flow residual evidence; not dynamic truth",
        "root": str(args.root),
        "manifest_rows": len(manifest),
        "expected_frames": expected_indices,
        "frames": frames,
        "invariant_errors": all_errors,
        "pass": not all_errors,
        "interpretation_limit": (
            "This validates serialization, units, direction and threshold "
            "arithmetic only. It does not validate dynamic-object accuracy."
        ),
    }
    rendered = json.dumps(summary, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if all_errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
