#!/usr/bin/env python3
"""Run one frozen SIn-style DT-SLAM system mode and preserve its outputs."""

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
        description="Run one frozen SIn-style DT-SLAM mode")
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--working-directory", required=True,
                        type=existing_path)
    parser.add_argument("--binary", required=True, type=existing_path)
    parser.add_argument("--vocabulary", required=True, type=existing_path)
    parser.add_argument("--base-settings", required=True, type=existing_path)
    parser.add_argument("--sin-settings", required=True, type=existing_path)
    parser.add_argument("--dataset", required=True, type=existing_path)
    parser.add_argument("--associations", required=True, type=existing_path)
    parser.add_argument("--model", type=existing_path)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--depth-mask-output-directory", type=Path)
    parser.add_argument("--viewer", choices=("on", "off"), default="off")
    return parser.parse_args()


def git_snapshot(working_directory):
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(working_directory), "rev-parse", "HEAD"],
            text=True).strip()
        tracked_status = subprocess.check_output(
            ["git", "-C", str(working_directory), "status", "--porcelain",
             "--untracked-files=no"], text=True)
        return {"commit": commit, "tracked_dirty": bool(tracked_status.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "tracked_dirty": None}


def clear_legacy_geometry(environment):
    for name in tuple(environment):
        if name.startswith("DT_SLAM_GEOMETRY_"):
            environment.pop(name, None)


def clear_sin_outputs(environment):
    for name in (
            "DT_SLAM_SIN_REFERENCE_DIR",
            "DT_SLAM_SIN_REGION_DYNAMIC_DIR",
            "DT_SLAM_SIN_NATIVE_INITIAL_DIR",
            "DT_SLAM_SIN_NATIVE_GRADIENT_DIR",
            "DT_SLAM_SIN_NATIVE_PLANE_DIR",
            "DT_SLAM_SIN_NATIVE_RAG_DIR",
            "DT_SLAM_SIN_DENSE_FLOW_REFERENCE_DIR",
            "DT_SLAM_SIN_DEPTH_FILTER_OUTPUT_DIR"):
        environment.pop(name, None)


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
        raise SystemExit("output already contains a frozen run: {}".format(
            output_directory))

    settings = args.sin_settings if geometry_enabled else args.base_settings
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
    clear_legacy_geometry(environment)
    clear_sin_outputs(environment)
    environment["DT_SLAM_DISABLE_VIEWER"] = (
        "0" if args.viewer == "on" else "1")
    environment["DT_SLAM_SIN_REGION_FEATURE_FILTER"] = (
        "1" if geometry_enabled else "0")

    depth_filter_enabled = semantic_enabled or geometry_enabled
    depth_filter_mode = {
        "semantic_only": "semantic_only",
        "geometry_only": "geometry_only",
        "semantic_geometry": "semantic_or_geometry",
    }.get(args.mode, "semantic_or_geometry")
    environment["DT_SLAM_SIN_DEPTH_FILTER"] = (
        "1" if depth_filter_enabled else "0")
    environment["DT_SLAM_SIN_DEPTH_FILTER_MODE"] = depth_filter_mode
    if depth_filter_enabled:
        environment["DT_SLAM_SIN_DEPTH_FILTER_CSV"] = str(
            output_directory / "depth_filter.csv")
        if args.depth_mask_output_directory is not None:
            depth_mask_output_directory = (
                args.depth_mask_output_directory.expanduser().resolve())
            depth_mask_output_directory.mkdir(parents=True, exist_ok=True)
            environment["DT_SLAM_SIN_DEPTH_FILTER_OUTPUT_DIR"] = str(
                depth_mask_output_directory)
        else:
            depth_mask_output_directory = None
    else:
        environment.pop("DT_SLAM_SIN_DEPTH_FILTER_CSV", None)
        depth_mask_output_directory = None
    if geometry_enabled:
        environment["DT_SLAM_SIN_SHADOW_FRAME_CSV"] = str(
            output_directory / "sin_frame.csv")
    else:
        environment.pop("DT_SLAM_SIN_SHADOW_FRAME_CSV", None)

    manifest = {
        "mode": args.mode,
        "semantic_enabled": semantic_enabled,
        "geometry_enabled": geometry_enabled,
        "depth_filter_enabled": depth_filter_enabled,
        "depth_filter_mode": depth_filter_mode if depth_filter_enabled else None,
        "depth_mask_output_directory": (
            str(depth_mask_output_directory)
            if depth_mask_output_directory is not None else None),
        "viewer": args.viewer,
        "working_directory": str(args.working_directory),
        "settings": str(settings),
        "dataset": str(args.dataset),
        "associations": str(args.associations),
        "model": str(args.model) if semantic_enabled else None,
        "git": git_snapshot(args.working_directory),
        "command": command,
        "status": "running",
    }
    manifest_path = output_directory / "run_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    start_ns = time.time_ns()
    start_wall = time.monotonic()
    log_path = output_directory / "run.log"
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
    manifest["wall_seconds"] = time.monotonic() - start_wall
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
