#!/usr/bin/env python3
"""Attach existing per-frame proxy masks to G2-1 raw count histograms."""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--histogram", required=True)
    parser.add_argument("--count-dir", required=True)
    parser.add_argument("--proxy-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.histogram, newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames
        rows = list(reader)
    if not fieldnames:
        raise ValueError("histogram CSV has no header")

    rows_by_frame = defaultdict(dict)
    for row in rows:
        frame = int(row["frame"])
        key = (int(row["comparison_count"]), int(row["positive_count"]))
        if key in rows_by_frame[frame]:
            raise ValueError(f"duplicate frame/bin row: frame={frame}, bin={key}")
        rows_by_frame[frame][key] = row

    count_dir = Path(args.count_dir)
    proxy_dir = Path(args.proxy_dir)
    validated_frames = 0
    for frame, frame_rows in sorted(rows_by_frame.items()):
        comparison_path = count_dir / f"frame_{frame:06d}_comparisons.png"
        positive_path = count_dir / f"frame_{frame:06d}_positives.png"
        proxy_path = proxy_dir / f"frame_{frame:06d}.png"
        comparison = cv2.imread(str(comparison_path), cv2.IMREAD_UNCHANGED)
        positive = cv2.imread(str(positive_path), cv2.IMREAD_UNCHANGED)
        proxy = cv2.imread(str(proxy_path), cv2.IMREAD_UNCHANGED)
        if comparison is None or positive is None or proxy is None:
            raise FileNotFoundError(
                f"missing count/proxy image for frame {frame}: "
                f"{comparison_path}, {positive_path}, {proxy_path}"
            )
        if (
            comparison.shape != positive.shape
            or comparison.shape != proxy.shape
            or comparison.dtype != np.uint8
            or positive.dtype != np.uint8
        ):
            raise ValueError(f"image domain/type mismatch for frame {frame}")
        proxy = proxy != 0

        observed_pixel_total = 0
        for key, row in frame_rows.items():
            comparisons, positives = key
            bin_mask = (comparison == comparisons) & (positive == positives)
            pixel_count = int(np.count_nonzero(bin_mask))
            expected = int(row["pixel_count"])
            if pixel_count != expected:
                raise ValueError(
                    f"frame {frame} bin {key}: raw count pixels "
                    f"{pixel_count} != CSV {expected}"
                )
            row["semantic_pixel_count"] = str(
                int(np.count_nonzero(bin_mask & proxy))
            )
            observed_pixel_total += pixel_count
        if observed_pixel_total != comparison.size:
            raise ValueError(
                f"frame {frame}: histogram covers {observed_pixel_total} "
                f"pixels, expected {comparison.size}"
            )
        validated_frames += 1

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(
        f"validated_frames={validated_frames} output={output.resolve()}"
    )


if __name__ == "__main__":
    main()
