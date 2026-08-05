#!/usr/bin/env python3
"""Summarize the frozen SIn-style representative-sequence evaluation."""

import argparse
import csv
import json
import re
import statistics
import subprocess
from pathlib import Path


SEQUENCES = {
    "tum_fr3_walking_xyz": {
        "groundtruth": "/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_walking_xyz/groundtruth.txt",
        "frames": 827,
    },
    "tum_fr3_sitting_static": {
        "groundtruth": "/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg3_sitting_static/groundtruth.txt",
        "frames": 680,
    },
    "tum_fr1_xyz": {
        "groundtruth": "/home/zhu/dynaslam_ws/TUM/rgbd_dataset_freiburg1_xyz/groundtruth.txt",
        "frames": 792,
    },
    "bonn_moving_nonobstructing_box": {
        "groundtruth": "/data/dynaslam/datasets/rgbd_bonn_moving_nonobstructing_box/groundtruth.txt",
        "frames": 778,
    },
    "bonn_moving_obstructing_box": {
        "groundtruth": "/data/dynaslam/datasets/rgbd_bonn_moving_obstructing_box/groundtruth.txt",
        "frames": 589,
    },
    "bonn_static_close_far": {
        "groundtruth": "/home/zhu/dynaslam_ws/BONN/rgbd_bonn_static_close_far/groundtruth.txt",
        "frames": 1750,
    },
}

MODES = ("orb_baseline", "semantic_only", "geometry_only", "semantic_geometry")


def numeric(row, key):
    try:
        return float(row.get(key, "") or 0)
    except ValueError:
        return 0.0


def load_csv(path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def parse_log(path):
    text = path.read_text(encoding="utf-8", errors="replace")

    def match(pattern, default=None, cast=float):
        found = re.search(pattern, text)
        return cast(found.group(1)) if found else default

    return {
        "actual_fps": match(r"actual_fps=([0-9.eE+-]+)"),
        "tracking_mean_ms": match(r"tracking\(ms\): mean=([0-9.eE+-]+)"),
        "tracking_median_ms": match(
            r"tracking\(ms\): mean=[0-9.eE+-]+ median=([0-9.eE+-]+)"
        ),
        "tracking_p95_ms": match(
            r"tracking\(ms\): mean=[0-9.eE+-]+ median=[0-9.eE+-]+ p95=([0-9.eE+-]+)"
        ),
        "deadline_missed": match(r"deadline_missed=([0-9]+)/", 0, int),
        "semantic_masks_ready": match(r"mask就绪: ([0-9]+)/", 0, int),
        "semantic_mask_age_median": match(r"mask年龄\(帧\): median=([0-9.eE+-]+)"),
        "semantic_mask_age_max": match(
            r"mask年龄\(帧\): median=[0-9.eE+-]+ max=([0-9.eE+-]+)"
        ),
        "semantic_total_median_ms": match(
            r"semantic_total\(ms\): mean=[0-9.eE+-]+ median=([0-9.eE+-]+)"
        ),
    }


def evo_rmse(tool, groundtruth, trajectory):
    command = [
        tool,
        "tum",
        str(groundtruth),
        str(trajectory),
        "--align",
        "--t_max_diff",
        "0.02",
    ]
    if tool == "evo_rpe":
        command.extend(["--delta", "1", "--delta_unit", "f"])
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    found = re.search(r"^\s*rmse\s+([0-9.eE+-]+)\s*$", result.stdout, re.MULTILINE)
    if not found:
        raise RuntimeError(f"Could not parse RMSE from {' '.join(command)}")
    return float(found.group(1))


def summarize_geometry(rows):
    if not rows:
        return {
            "geometry_rows": 0,
            "geometry_candidate_frames": 0,
            "geometry_candidate_orb_total": 0,
            "geometry_new_dynamic_orb_total": 0,
            "geometry_removed_associations_total": 0,
            "geometry_fail_open_frames": 0,
        }
    return {
        "geometry_rows": len(rows),
        "geometry_candidate_frames": sum(
            numeric(row, "region_dynamic_valid_dynamic_pixels") > 0 for row in rows
        ),
        "geometry_candidate_orb_total": int(
            sum(numeric(row, "region_feature_filter_candidate_features") for row in rows)
        ),
        "geometry_new_dynamic_orb_total": int(
            sum(numeric(row, "region_feature_filter_new_dynamic_features") for row in rows)
        ),
        "geometry_removed_associations_total": int(
            sum(
                numeric(row, "region_feature_filter_actual_removed_associations")
                for row in rows
            )
        ),
        "geometry_fail_open_frames": sum(
            numeric(row, "region_feature_filter_tracking_fail_open") > 0 for row in rows
        ),
    }


def summarize_depth(rows):
    if not rows:
        return {
            "depth_filter_rows": 0,
            "depth_reject_frames": 0,
            "depth_rejected_valid_total": 0,
            "depth_input_valid_total": 0,
            "depth_reject_ratio": None,
            "depth_filter_median_ms": None,
        }
    input_total = sum(numeric(row, "input_valid_depth_pixels") for row in rows)
    rejected_total = sum(numeric(row, "rejected_valid_depth_pixels") for row in rows)
    times = [numeric(row, "filter_ms") for row in rows]
    return {
        "depth_filter_rows": len(rows),
        "depth_reject_frames": sum(
            numeric(row, "rejected_valid_depth_pixels") > 0 for row in rows
        ),
        "depth_rejected_valid_total": int(rejected_total),
        "depth_input_valid_total": int(input_total),
        "depth_reject_ratio": rejected_total / input_total if input_total else None,
        "depth_filter_median_ms": statistics.median(times) if times else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()

    run_root = args.root / "runs"
    records = []
    for sequence, metadata in SEQUENCES.items():
        for mode in MODES:
            directory = run_root / sequence / mode
            trajectory = directory / "CameraTrajectory.txt"
            log = directory / "run.log"
            if not trajectory.is_file() or not log.is_file():
                raise FileNotFoundError(f"Incomplete run: {directory}")

            record = {
                "sequence": sequence,
                "mode": mode,
                "input_frames": metadata["frames"],
                "trajectory_poses": sum(
                    1
                    for line in trajectory.read_text(encoding="utf-8").splitlines()
                    if line.strip() and not line.lstrip().startswith("#")
                ),
                "ate_rmse_m": evo_rmse(
                    "evo_ape", metadata["groundtruth"], trajectory
                ),
                "rpe_rmse_m": evo_rmse(
                    "evo_rpe", metadata["groundtruth"], trajectory
                ),
            }
            record.update(parse_log(log))
            record.update(summarize_geometry(load_csv(directory / "sin_frame.csv")))
            record.update(summarize_depth(load_csv(directory / "depth_filter.csv")))
            records.append(record)

    args.root.mkdir(parents=True, exist_ok=True)
    json_path = args.root / "summary.json"
    csv_path = args.root / "summary.csv"
    json_path.write_text(
        json.dumps(records, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)

    print(json_path)
    print(csv_path)


if __name__ == "__main__":
    main()
