#!/usr/bin/env python3
"""Summarize paired R0 Gazebo baseline/zero-semantic repeat runs.

This tool is intentionally read-only with respect to DT-SLAM.  It evaluates
already frozen trajectories and verifies that the semantic-only path received
an all-zero dynamic mask before comparing it with the pure ORB-SLAM2 path.
"""

import argparse
import csv
import json
import re
import statistics
import subprocess
from pathlib import Path


MODES = ("orb_baseline", "semantic_only")


def existing_file(value):
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise argparse.ArgumentTypeError(f"file does not exist: {path}")
    return path


def existing_directory(value):
    path = Path(value).expanduser().resolve()
    if not path.is_dir():
        raise argparse.ArgumentTypeError(f"directory does not exist: {path}")
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize paired R0 Gazebo equivalence runs")
    parser.add_argument("--groundtruth", required=True, type=existing_file)
    parser.add_argument(
        "--pair",
        action="append",
        nargs=3,
        metavar=("LABEL", "ORB_RUN", "SEMANTIC_RUN"),
        required=True,
        help="repeat label and the two frozen run directories",
    )
    parser.add_argument("--output-directory", required=True, type=Path)
    return parser.parse_args()


def count_trajectory_poses(path):
    return sum(
        bool(line.strip()) and not line.lstrip().startswith("#")
        for line in path.read_text(encoding="utf-8").splitlines()
    )


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
    match = re.search(
        r"^\s*rmse\s+([0-9.eE+-]+)\s*$", result.stdout, re.MULTILINE)
    if not match:
        raise RuntimeError(f"cannot parse RMSE from {' '.join(command)}")
    return float(match.group(1))


def parse_timing(log_path):
    text = log_path.read_text(encoding="utf-8", errors="replace")

    def value(pattern, cast=float):
        match = re.search(pattern, text)
        return cast(match.group(1)) if match else None

    return {
        "actual_fps": value(r"actual_fps=([0-9.eE+-]+)"),
        "deadline_missed": value(r"deadline_missed=([0-9]+?)/", int),
        "semantic_masks_ready": value(r"mask就绪: ([0-9]+?)/", int),
        "semantic_provider_cuda": (
            "Semantic provider: CUDAExecutionProvider" in text),
    }


def load_semantic_mask_audit(run_directory):
    path = run_directory / "depth_filter.csv"
    if not path.is_file():
        return {
            "depth_filter_rows": 0,
            "semantic_dynamic_pixels_total": None,
            "rejected_valid_depth_pixels_total": None,
            "all_zero_semantic_mask": None,
        }
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    semantic_total = sum(int(float(row["semantic_dynamic_pixels"])) for row in rows)
    rejected_total = sum(
        int(float(row["rejected_valid_depth_pixels"])) for row in rows)
    return {
        "depth_filter_rows": len(rows),
        "semantic_dynamic_pixels_total": semantic_total,
        "rejected_valid_depth_pixels_total": rejected_total,
        "all_zero_semantic_mask": semantic_total == 0,
    }


def summarize_run(label, mode, directory, groundtruth):
    directory = existing_directory(directory)
    manifest_path = directory / "run_manifest.json"
    trajectory_path = directory / "CameraTrajectory.txt"
    keyframe_path = directory / "KeyFrameTrajectory.txt"
    log_path = directory / "run.log"
    for path in (manifest_path, trajectory_path, keyframe_path, log_path):
        if not path.is_file():
            raise FileNotFoundError(f"incomplete frozen run: {path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("mode") != mode:
        raise ValueError(
            f"mode mismatch for {directory}: {manifest.get('mode')} != {mode}")
    record = {
        "repeat": label,
        "mode": mode,
        "directory": str(directory),
        "commit": manifest.get("git", {}).get("commit"),
        "tracked_dirty": manifest.get("git", {}).get("tracked_dirty"),
        "status": manifest.get("status"),
        "return_code": manifest.get("return_code"),
        "trajectory_poses": count_trajectory_poses(trajectory_path),
        "keyframes": count_trajectory_poses(keyframe_path),
        "ate_rmse_m": evo_rmse("evo_ape", groundtruth, trajectory_path),
        "rpe_rmse_m": evo_rmse("evo_rpe", groundtruth, trajectory_path),
    }
    record.update(parse_timing(log_path))
    record.update(load_semantic_mask_audit(directory))
    return record


def median_and_range(values):
    return {
        "median": statistics.median(values),
        "minimum": min(values),
        "maximum": max(values),
        "range": max(values) - min(values),
    }


def main():
    args = parse_args()
    records = []
    pair_records = []
    for label, orb_directory, semantic_directory in args.pair:
        orb = summarize_run(
            label, "orb_baseline", Path(orb_directory), args.groundtruth)
        semantic = summarize_run(
            label, "semantic_only", Path(semantic_directory), args.groundtruth)
        records.extend((orb, semantic))
        pair_records.append({
            "repeat": label,
            "semantic_minus_orb_ate_rmse_m": (
                semantic["ate_rmse_m"] - orb["ate_rmse_m"]),
            "semantic_minus_orb_rpe_rmse_m": (
                semantic["rpe_rmse_m"] - orb["rpe_rmse_m"]),
            "semantic_minus_orb_keyframes": (
                semantic["keyframes"] - orb["keyframes"]),
        })

    mode_summary = {}
    for mode in MODES:
        selected = [record for record in records if record["mode"] == mode]
        mode_summary[mode] = {
            "runs": len(selected),
            "ate_rmse_m": median_and_range(
                [record["ate_rmse_m"] for record in selected]),
            "rpe_rmse_m": median_and_range(
                [record["rpe_rmse_m"] for record in selected]),
            "keyframes": median_and_range(
                [record["keyframes"] for record in selected]),
            "trajectory_complete_all": all(
                record["trajectory_poses"] == 600 for record in selected),
            "deadline_missed_total": sum(
                record["deadline_missed"] or 0 for record in selected),
        }

    semantic_records = [
        record for record in records if record["mode"] == "semantic_only"]
    invariants = {
        "same_commit_all": len({record["commit"] for record in records}) == 1,
        "tracked_clean_all": all(
            record["tracked_dirty"] is False for record in records),
        "completed_all": all(
            record["status"] == "completed" and record["return_code"] == 0
            for record in records),
        "trajectory_complete_all": all(
            record["trajectory_poses"] == 600 for record in records),
        "semantic_cuda_all": all(
            record["semantic_provider_cuda"] for record in semantic_records),
        "semantic_masks_ready_all": all(
            record["semantic_masks_ready"] == 600 for record in semantic_records),
        "semantic_dynamic_pixels_zero_all": all(
            record["all_zero_semantic_mask"] is True
            and record["rejected_valid_depth_pixels_total"] == 0
            for record in semantic_records),
    }

    output = {
        "protocol": {
            "groundtruth": str(args.groundtruth),
            "input_frames": 600,
            "viewer": "off",
            "evo_alignment": "SE(3) Umeyama",
            "evo_max_timestamp_difference_seconds": 0.02,
            "rpe_delta": "1 output frame",
        },
        "records": records,
        "paired_differences": pair_records,
        "mode_summary": mode_summary,
        "invariants": invariants,
    }

    output_directory = args.output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    json_path = output_directory / "r0_equivalence_summary.json"
    csv_path = output_directory / "r0_equivalence_records.csv"
    json_path.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    print(json_path)
    print(csv_path)
    print(json.dumps(invariants, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
