#!/usr/bin/env python3
"""Audit read-only sparse MapPoint lifecycle and summary CSV pairs."""

import argparse
import csv
import json
from pathlib import Path


def integer(row, name):
    return int(row[name])


def audit(prefix):
    lifecycle_path = Path(str(prefix) + "_candidate_lifecycle.csv")
    summary_path = Path(str(prefix) + "_summary.csv")
    with lifecycle_path.open(newline="", encoding="utf-8") as stream:
        lifecycle = list(csv.DictReader(stream))
    with summary_path.open(newline="", encoding="utf-8") as stream:
        summaries = list(csv.DictReader(stream))
    violations = []
    if len(summaries) != 1:
        violations.append("summary_row_count")
        summary = {}
    else:
        summary = summaries[0]

    ids = set()
    original_survivors = 0
    resolved_survivors = 0
    replacement_survivors = 0
    for line, row in enumerate(lifecycle, start=2):
        original_id = integer(row, "original_mappoint_id")
        if original_id in ids:
            violations.append(
                "{}:{} duplicate_original_id".format(
                    lifecycle_path, line))
        ids.add(original_id)
        original_in_final = integer(row, "original_in_final_map")
        original_bad = integer(row, "original_bad")
        replacement_depth = integer(row, "replacement_depth")
        resolved_in_final = integer(row, "resolved_in_final_map")
        resolved_bad = integer(row, "resolved_bad")
        survived = integer(row, "proxy_survived")
        if original_in_final and original_bad:
            violations.append(
                "{}:{} bad_original_in_final".format(
                    lifecycle_path, line))
        expected_survival = int(
            bool(resolved_in_final) and not bool(resolved_bad))
        if survived != expected_survival:
            violations.append(
                "{}:{} survival_mismatch".format(
                    lifecycle_path, line))
        if original_in_final and not original_bad:
            original_survivors += 1
        if survived:
            resolved_survivors += 1
            if replacement_depth > 0:
                replacement_survivors += 1

    if summary:
        candidate_created = integer(summary, "candidate_created")
        if candidate_created != len(lifecycle):
            violations.append("candidate_created_mismatch")
        if integer(
                summary,
                "candidate_original_survivors") != original_survivors:
            violations.append("original_survivor_mismatch")
        if integer(
                summary,
                "candidate_resolved_survivors") != resolved_survivors:
            violations.append("resolved_survivor_mismatch")
        if integer(
                summary,
                "candidate_replacement_survivors") != (
                replacement_survivors):
            violations.append("replacement_survivor_mismatch")
        if integer(
                summary,
                "candidate_culled_or_not_surviving") != (
                len(lifecycle) - resolved_survivors):
            violations.append("candidate_culled_mismatch")
        if summary["read_only"] != "true":
            violations.append("not_read_only")
        if summary["direct_map_mutation"] != "none":
            violations.append("unexpected_map_mutation")

    return {
        "prefix": str(prefix),
        "mode": summary.get("mode", "") if summary else "",
        "final_mappoints": (
            integer(summary, "final_mappoints") if summary else 0),
        "final_keyframes": (
            integer(summary, "final_keyframes") if summary else 0),
        "candidate_created": len(lifecycle),
        "candidate_original_survivors": original_survivors,
        "candidate_resolved_survivors": resolved_survivors,
        "candidate_replacement_survivors": replacement_survivors,
        "candidate_proxy_survival_ratio": (
            float(resolved_survivors) / float(len(lifecycle))
            if lifecycle else 0.0),
        "violations": violations,
        "passed": not violations,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"runs": [audit(prefix) for prefix in args.prefix]}
    report["passed"] = all(run["passed"] for run in report["runs"])
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
