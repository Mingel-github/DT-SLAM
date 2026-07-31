#!/usr/bin/env python3
"""Validate and summarize G1-F1 tracking-filter frame CSV files."""

import argparse
import csv
import json
import math
from pathlib import Path


REQUIRED_COLUMNS = {
    "frame",
    "timestamp",
    "q_threshold",
    "scale_valid",
    "frame_scale_px",
    "scale_support",
    "quality_eligible_features",
    "candidate_features",
    "baseline_associations",
    "candidate_associations",
    "removed_associations",
    "remaining_associations",
    "candidate_association_fraction",
    "within_relocalization_window",
    "applied",
    "state",
    "pose_reoptimization",
    "mapping_veto",
}


def integer(row, field, path, line):
    try:
        return int(row[field])
    except ValueError as error:
        raise ValueError(
            "{}:{} invalid integer {}".format(path, line, field)
        ) from error


def number(row, field, path, line):
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

    states = {}
    seen_frames = set()
    removed_total = 0
    candidate_total = 0
    baseline_total = 0
    applied_frames = 0
    violations = []
    q_values = set()

    for line, row in enumerate(rows, start=2):
        frame = integer(row, "frame", path, line)
        if frame in seen_frames:
            violations.append(
                "{}:{} duplicate frame {}".format(path, line, frame)
            )
        seen_frames.add(frame)
        q_values.add(number(row, "q_threshold", path, line))
        scale_valid = integer(row, "scale_valid", path, line)
        within_relocalization = integer(
            row, "within_relocalization_window", path, line
        )
        applied = integer(row, "applied", path, line)
        if scale_valid not in (0, 1):
            violations.append(
                "{}:{} scale_valid is not binary".format(path, line)
            )
        if within_relocalization not in (0, 1):
            violations.append(
                "{}:{} relocalization flag is not binary".format(
                    path, line
                )
            )
        if applied not in (0, 1):
            violations.append(
                "{}:{} applied is not binary".format(path, line)
            )

        baseline = integer(row, "baseline_associations", path, line)
        candidate = integer(row, "candidate_associations", path, line)
        removed = integer(row, "removed_associations", path, line)
        remaining = integer(row, "remaining_associations", path, line)
        fraction = number(
            row, "candidate_association_fraction", path, line
        )
        state = row["state"]
        states[state] = states.get(state, 0) + 1

        if min(baseline, candidate, removed, remaining) < 0:
            violations.append(
                "{}:{} negative association count".format(path, line)
            )
        if candidate > baseline:
            violations.append(
                "{}:{} candidates exceed baseline".format(path, line)
            )
        expected_fraction = (
            float(candidate) / float(baseline) if baseline else 0.0
        )
        if abs(fraction - expected_fraction) > 1e-9:
            violations.append(
                "{}:{} candidate fraction mismatch".format(path, line)
            )
        if applied:
            applied_frames += 1
            if removed <= 0 or removed != candidate:
                violations.append(
                    "{}:{} applied removal mismatch".format(path, line)
                )
            if remaining != baseline - removed:
                violations.append(
                    "{}:{} applied remaining mismatch".format(path, line)
                )
            if fraction > 0.05 + 1e-12:
                violations.append(
                    "{}:{} applied above 5% cap".format(path, line)
                )
            if within_relocalization:
                violations.append(
                    "{}:{} applied in relocalization window".format(
                        path, line
                    )
                )
        else:
            if removed != 0 or remaining != baseline:
                violations.append(
                    "{}:{} fail-open row changed associations".format(
                        path, line
                    )
                )
        if row["pose_reoptimization"] != "none":
            violations.append(
                "{}:{} unexpected pose reoptimization".format(path, line)
            )
        if row["mapping_veto"] != "none":
            violations.append(
                "{}:{} unexpected mapping veto".format(path, line)
            )

        baseline_total += baseline
        candidate_total += candidate
        removed_total += removed

    return {
        "path": str(path),
        "rows": len(rows),
        "q_values": sorted(q_values),
        "states": dict(sorted(states.items())),
        "baseline_associations": baseline_total,
        "candidate_associations": candidate_total,
        "removed_associations": removed_total,
        "applied_frames": applied_frames,
        "removed_fraction_of_baseline": (
            float(removed_total) / float(baseline_total)
            if baseline_total
            else 0.0
        ),
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
