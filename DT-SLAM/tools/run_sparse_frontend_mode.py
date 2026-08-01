#!/usr/bin/env python3
"""Run one frozen DT-SLAM sparse-frontend ablation mode."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


MODES = (
    "orb_baseline",
    "semantic_only",
    "geometry_only",
    "semantic_geometry",
)


def existing_path(value):
    path = Path(value).expanduser().resolve()
    if not path.exists():
        raise argparse.ArgumentTypeError("path does not exist: {}".format(path))
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run one frozen G1 sparse-frontend mode")
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--working-directory", required=True,
                        type=existing_path)
    parser.add_argument("--binary", required=True, type=existing_path)
    parser.add_argument("--vocabulary", required=True, type=existing_path)
    parser.add_argument("--base-settings", required=True, type=existing_path)
    parser.add_argument("--geometry-settings", required=True,
                        type=existing_path)
    parser.add_argument("--dataset", required=True, type=existing_path)
    parser.add_argument("--associations", required=True, type=existing_path)
    parser.add_argument("--model", type=existing_path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--viewer", choices=("on", "off"), default="off")
    return parser.parse_args()


def git_snapshot(working_directory):
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(working_directory), "rev-parse", "HEAD"],
            text=True).strip()
        status = subprocess.check_output(
            ["git", "-C", str(working_directory), "status", "--porcelain"],
            text=True)
        return {"commit": commit, "dirty": bool(status.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def main():
    args = parse_args()
    semantic_enabled = args.mode in ("semantic_only", "semantic_geometry")
    geometry_enabled = args.mode in ("geometry_only", "semantic_geometry")
    if semantic_enabled and args.model is None:
        raise SystemExit("{} requires --model".format(args.mode))

    output_directory = args.output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    occupied = [
        output_directory / "run.log",
        output_directory / "run_manifest.json",
        output_directory / "CameraTrajectory.txt",
        output_directory / "KeyFrameTrajectory.txt",
    ]
    if any(path.exists() for path in occupied):
        raise SystemExit(
            "output directory already contains a frozen run: {}".format(
                output_directory))

    settings = args.geometry_settings if geometry_enabled else args.base_settings
    command = [
        str(args.binary),
        str(args.vocabulary),
        str(settings),
        str(args.dataset),
        str(args.associations),
    ]
    if semantic_enabled:
        command.append(str(args.model))

    environment = os.environ.copy()
    environment["DT_SLAM_GEOMETRY_TRACKING_FILTER"] = (
        "1" if geometry_enabled else "0")
    environment["DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL"] = "0"
    environment["DT_SLAM_GEOMETRY_MAPPING_FILTER"] = (
        "1" if geometry_enabled else "0")
    environment["DT_SLAM_GEOMETRY_MAP_QUALITY_AUDIT"] = "0"
    environment["DT_SLAM_DISABLE_VIEWER"] = (
        "0" if args.viewer == "on" else "1")

    geometry_outputs = {}
    if geometry_enabled:
        geometry_outputs = {
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_CSV":
                str(output_directory / "tracking_filter.csv"),
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_FEATURE_CSV":
                str(output_directory / "removed_associations.csv"),
            "DT_SLAM_GEOMETRY_TRACKING_FILTER_CANDIDATE_CSV":
                str(output_directory / "candidate_associations.csv"),
            "DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV":
                str(output_directory / "mapping_filter.csv"),
        }
        environment.update(geometry_outputs)
    else:
        for name in (
                "DT_SLAM_GEOMETRY_TRACKING_FILTER_CSV",
                "DT_SLAM_GEOMETRY_TRACKING_FILTER_FEATURE_CSV",
                "DT_SLAM_GEOMETRY_TRACKING_FILTER_CANDIDATE_CSV",
                "DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL_CSV",
                "DT_SLAM_GEOMETRY_MAPPING_FILTER_CSV",
                "DT_SLAM_GEOMETRY_MAP_QUALITY_PREFIX"):
            environment.pop(name, None)

    manifest = {
        "mode": args.mode,
        "semantic_enabled": semantic_enabled,
        "geometry_enabled": geometry_enabled,
        "viewer": args.viewer,
        "working_directory": str(args.working_directory),
        "settings": str(settings),
        "dataset": str(args.dataset),
        "associations": str(args.associations),
        "model": str(args.model) if semantic_enabled else None,
        "git": git_snapshot(args.working_directory),
        "command": command,
        "geometry_environment": {
            "DT_SLAM_GEOMETRY_TRACKING_FILTER":
                environment["DT_SLAM_GEOMETRY_TRACKING_FILTER"],
            "DT_SLAM_GEOMETRY_MAPPING_COUNTERFACTUAL": "0",
            "DT_SLAM_GEOMETRY_MAPPING_FILTER":
                environment["DT_SLAM_GEOMETRY_MAPPING_FILTER"],
            "DT_SLAM_GEOMETRY_MAP_QUALITY_AUDIT": "0",
            **geometry_outputs,
        },
        "status": "running",
    }
    manifest_path = output_directory / "run_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    start_ns = time.time_ns()
    log_path = output_directory / "run.log"
    return_code = 1
    with log_path.open("w", encoding="utf-8") as log_stream:
        process = subprocess.Popen(
            command,
            cwd=str(args.working_directory),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1)
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            log_stream.write(line)
        return_code = process.wait()

    manifest["return_code"] = return_code
    manifest["status"] = "completed" if return_code == 0 else "failed"
    if return_code == 0:
        for name in ("CameraTrajectory.txt", "KeyFrameTrajectory.txt"):
            source = args.working_directory / name
            if not source.exists() or source.stat().st_mtime_ns < start_ns:
                manifest["status"] = "failed_missing_fresh_trajectory"
                manifest_path.write_text(
                    json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
                raise SystemExit("missing fresh trajectory: {}".format(source))
            shutil.copy2(str(source), str(output_directory / name))

    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
