#!/usr/bin/env python3
"""Validate and summarize G1-M0 MapPoint-admission counterfactual CSVs."""

import argparse
import csv
import json
import math
from pathlib import Path


REQUIRED_COLUMNS = {
    "frame",
    "timestamp",
    "stage",
    "q_threshold",
    "scale_valid",
    "candidate_vector_valid",
    "candidate_state",
    "feature_count",
    "candidate_features",
    "candidate_associations_before_mapping",
    "candidate_tracking_removals",
    "valid_depth_features",
    "candidate_valid_depth_features",
    "depth_admission_features",
    "candidate_depth_admission_features",
    "created_mappoints",
    "candidate_created_mappoints",
    "recreated_after_tracking_removal",
    "counterfactual_only",
    "direct_mapping_state_mutation",
    "mapping_veto",
}
STAGES = {"stereo_initialization", "create_new_keyframe"}


def parse_int(row, field, path, line):
    try:
        return int(row[field])
    except ValueError as error:
        raise ValueError(
            "{}:{} invalid integer {}".format(path, line, field)
        ) from error


def parse_float(row, field, path, line):
    try:
        value = float(row[field])
    except ValueError as error:
        raise ValueError(
            "{}:{} invalid number {}".format(path, line, field)
        ) from error
    if not math.isfinite(value):
        raise ValueError(
            "{}:{} non-finite {}".format(path, line, field)
        )
    return value


def audit(path):
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            raise ValueError("{} has no CSV header".format(path))
        missing = REQUIRED_COLUMNS - set(reader.fieldnames)
        if missing:
            raise ValueError(
                "{} missing columns: {}".format(
                    path, ", ".join(sorted(missing))
                )
            )
        rows = list(reader)

    violations = []
    seen = set()
    stages = {}
    totals = {
        "candidate_features": 0,
        "candidate_associations_before_mapping": 0,
        "candidate_tracking_removals": 0,
        "created_mappoints": 0,
        "candidate_created_mappoints": 0,
        "recreated_after_tracking_removal": 0,
    }

    for line, row in enumerate(rows, start=2):
        frame = parse_int(row, "frame", path, line)
        timestamp = parse_float(row, "timestamp", path, line)
        stage = row["stage"]
        key = (frame, stage)
        if key in seen:
            violations.append(
                "{}:{} duplicate event {}".format(path, line, key)
            )
        seen.add(key)
        if stage not in STAGES:
            violations.append(
                "{}:{} invalid stage {}".format(path, line, stage)
            )
        stages[stage] = stages.get(stage, 0) + 1
        if timestamp <= 0.0:
            violations.append(
                "{}:{} non-positive timestamp".format(path, line)
            )
        if abs(parse_float(row, "q_threshold", path, line) - 10.0) > 1e-9:
            violations.append(
                "{}:{} q threshold is not 10".format(path, line)
            )

        scale_valid = parse_int(row, "scale_valid", path, line)
        vector_valid = parse_int(
            row, "candidate_vector_valid", path, line
        )
        if scale_valid not in (0, 1) or vector_valid not in (0, 1):
            violations.append(
                "{}:{} non-binary validity flag".format(path, line)
            )

        values = {
            field: parse_int(row, field, path, line)
            for field in (
                "feature_count",
                "candidate_features",
                "candidate_associations_before_mapping",
                "candidate_tracking_removals",
                "valid_depth_features",
                "candidate_valid_depth_features",
                "depth_admission_features",
                "candidate_depth_admission_features",
                "created_mappoints",
                "candidate_created_mappoints",
                "recreated_after_tracking_removal",
            )
        }
        if any(value < 0 for value in values.values()):
            violations.append(
                "{}:{} negative count".format(path, line)
            )
        if values["candidate_features"] > values["feature_count"]:
            violations.append(
                "{}:{} candidates exceed features".format(path, line)
            )
        for field in (
            "candidate_associations_before_mapping",
            "candidate_tracking_removals",
            "candidate_valid_depth_features",
        ):
            if values[field] > values["candidate_features"]:
                violations.append(
                    "{}:{} {} exceeds candidates".format(
                        path, line, field
                    )
                )
        if values["candidate_valid_depth_features"] > values[
            "valid_depth_features"
        ]:
            violations.append(
                "{}:{} candidate valid depth exceeds total".format(
                    path, line
                )
            )
        if values["depth_admission_features"] > values[
            "valid_depth_features"
        ]:
            violations.append(
                "{}:{} depth admission exceeds valid depth".format(
                    path, line
                )
            )
        if values["candidate_depth_admission_features"] > min(
            values["candidate_valid_depth_features"],
            values["depth_admission_features"],
        ):
            violations.append(
                "{}:{} candidate depth admission mismatch".format(
                    path, line
                )
            )
        if values["created_mappoints"] > values[
            "depth_admission_features"
        ]:
            violations.append(
                "{}:{} creations exceed admission".format(path, line)
            )
        if values["candidate_created_mappoints"] > min(
            values["created_mappoints"],
            values["candidate_depth_admission_features"],
        ):
            violations.append(
                "{}:{} candidate creations mismatch".format(path, line)
            )
        if values["recreated_after_tracking_removal"] > min(
            values["candidate_created_mappoints"],
            values["candidate_tracking_removals"],
        ):
            violations.append(
                "{}:{} recreation mismatch".format(path, line)
            )

        if stage == "stereo_initialization":
            if (
                scale_valid
                or vector_valid
                or row["candidate_state"] != "reference_unavailable"
                or values["candidate_features"] != 0
            ):
                violations.append(
                    "{}:{} invalid initialization evidence state".format(
                        path, line
                    )
                )
        elif not scale_valid or not vector_valid:
            if values["candidate_features"] != 0:
                violations.append(
                    "{}:{} invalid evidence has candidates".format(
                        path, line
                    )
                )

        if row["counterfactual_only"] != "true":
            violations.append(
                "{}:{} not counterfactual-only".format(path, line)
            )
        if row["direct_mapping_state_mutation"] != "none":
            violations.append(
                "{}:{} unexpected mapping mutation".format(path, line)
            )
        if row["mapping_veto"] != "none":
            violations.append(
                "{}:{} unexpected mapping veto".format(path, line)
            )

        for field in totals:
            totals[field] += values[field]

    created = totals["created_mappoints"]
    totals["candidate_created_fraction"] = (
        float(totals["candidate_created_mappoints"]) / float(created)
        if created
        else 0.0
    )
    candidate_created = totals["candidate_created_mappoints"]
    totals["recreated_fraction_of_candidate_created"] = (
        float(totals["recreated_after_tracking_removal"])
        / float(candidate_created)
        if candidate_created
        else 0.0
    )
    return {
        "path": str(path),
        "rows": len(rows),
        "stages": dict(sorted(stages.items())),
        "totals": totals,
        "invariant_violations": violations,
        "passed": not violations,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    report = {"files": [audit(path) for path in args.csv]}
    report["passed"] = all(item["passed"] for item in report["files"])
    payload = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
