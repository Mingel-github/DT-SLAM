#!/usr/bin/env python3
"""Audit G1-M1 MapPoint-admission filter CSV invariants."""

import argparse
import csv
import json
from pathlib import Path


def as_int(row, name):
    return int(row[name])


def as_float(row, name):
    return float(row[name])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    summary = {
        "files": 0,
        "rows": 0,
        "initialization_rows": 0,
        "keyframe_rows": 0,
        "applied_rows": 0,
        "fail_open_rows": 0,
        "candidate_features": 0,
        "new_dynamic_flags": 0,
        "vetoed_depth_features": 0,
        "created_mappoints": 0,
        "candidate_created_mappoints": 0,
        "violations": [],
        "states": {},
    }

    for path in args.csv:
        summary["files"] += 1
        with path.open(newline="", encoding="utf-8") as stream:
            for line_number, row in enumerate(
                    csv.DictReader(stream), start=2):
                summary["rows"] += 1
                stage = row["stage"]
                state = row["state"]
                applied = as_int(row, "applied") != 0
                candidate_features = as_int(
                    row, "candidate_features")
                candidate_depth = as_int(
                    row, "candidate_valid_depth_features")
                new_flags = as_int(row, "new_dynamic_flags")
                vetoed_depth = as_int(
                    row, "vetoed_depth_features")
                valid_depth = as_int(row, "valid_depth_features")
                remaining_depth = as_int(
                    row, "remaining_valid_depth_features")
                created = as_int(row, "created_mappoints")
                candidate_created = as_int(
                    row, "candidate_created_mappoints")

                summary["states"][state] = (
                    summary["states"].get(state, 0) + 1)
                summary["candidate_features"] += candidate_features
                summary["new_dynamic_flags"] += new_flags
                summary["vetoed_depth_features"] += vetoed_depth
                summary["created_mappoints"] += created
                summary["candidate_created_mappoints"] += (
                    candidate_created)
                if stage == "stereo_initialization":
                    summary["initialization_rows"] += 1
                elif stage == "create_new_keyframe":
                    summary["keyframe_rows"] += 1
                if applied:
                    summary["applied_rows"] += 1
                elif state.endswith("fail_open"):
                    summary["fail_open_rows"] += 1

                violations = []
                if row["mapping_filter"] != "mvbDynamic":
                    violations.append("unexpected_mapping_filter")
                if row["pose_reoptimization"] != "none":
                    violations.append("unexpected_pose_reoptimization")
                if new_flags > candidate_features:
                    violations.append(
                        "new_flags_exceed_candidates")
                if vetoed_depth > candidate_depth:
                    violations.append(
                        "vetoed_depth_exceeds_candidate_depth")
                if candidate_created > created:
                    violations.append(
                        "candidate_created_exceeds_created")
                if applied:
                    if state != "applied":
                        violations.append(
                            "applied_flag_state_mismatch")
                    if new_flags == 0:
                        violations.append(
                            "applied_without_new_flags")
                    if candidate_created != 0:
                        violations.append(
                            "applied_but_candidate_created")
                    if remaining_depth != (
                            valid_depth - vetoed_depth):
                        violations.append(
                            "remaining_depth_mismatch")
                    if as_float(
                            row,
                            "candidate_feature_fraction") > (
                            as_float(
                                row,
                                "maximum_feature_fraction") + 1e-12):
                        violations.append(
                            "applied_above_feature_fraction")
                    if as_float(
                            row,
                            "candidate_depth_fraction") > (
                            as_float(
                                row,
                                "maximum_depth_fraction") + 1e-12):
                        violations.append(
                            "applied_above_depth_fraction")
                    if remaining_depth < as_int(
                            row,
                            "minimum_remaining_depth_features"):
                        violations.append(
                            "applied_below_minimum_depth")
                    if as_int(
                            row,
                            "tracking_safeguards_passed") == 0:
                        violations.append(
                            "applied_without_tracking_safeguards")
                else:
                    if new_flags != 0 or vetoed_depth != 0:
                        violations.append(
                            "mutation_while_not_applied")
                    if remaining_depth != valid_depth:
                        violations.append(
                            "fail_open_remaining_depth_mismatch")
                if stage == "stereo_initialization":
                    if applied:
                        violations.append(
                            "initialization_filter_applied")
                    if state != "reference_unavailable_fail_open":
                        violations.append(
                            "unexpected_initialization_state")

                for violation in violations:
                    summary["violations"].append({
                        "file": str(path),
                        "line": line_number,
                        "frame": as_int(row, "frame"),
                        "violation": violation,
                    })

    summary["status"] = (
        "PASS" if not summary["violations"] else "FAIL")
    rendered = json.dumps(summary, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    raise SystemExit(0 if summary["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
