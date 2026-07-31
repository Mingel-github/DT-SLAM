#!/usr/bin/env python3
"""Prepare a compact, deterministic Bonn static-model point array.

This is an evaluation-data preparation utility.  It does not implement a
runtime dynamic detector and must not be used as an input to SLAM.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import tempfile
import zipfile
from pathlib import Path

import numpy as np


EXPECTED_PROPERTIES = (
    "property float x",
    "property float y",
    "property float z",
    "property uchar red",
    "property uchar green",
    "property uchar blue",
    "property float scalar_Scalar_field",
)


def sha256_file(path: Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def read_header(stream) -> tuple[list[str], int]:
    header: list[str] = []
    vertex_count = -1
    while True:
        raw = stream.readline()
        if not raw:
            raise ValueError("unexpected EOF before end_header")
        line = raw.decode("ascii").strip()
        header.append(line)
        if line.startswith("element vertex "):
            vertex_count = int(line.split()[2])
        if line == "end_header":
            break

    if not header or header[0] != "ply":
        raise ValueError("input member is not a PLY file")
    if "format ascii 1.0" not in header:
        raise ValueError("only the official ASCII PLY is supported")
    if vertex_count < 0:
        raise ValueError("PLY header has no vertex count")

    properties = tuple(line for line in header if line.startswith("property "))
    if properties != EXPECTED_PROPERTIES:
        raise ValueError(f"unexpected PLY properties: {properties!r}")
    return header, vertex_count


def prepare(source_zip: Path, output_npy: Path, stride: int) -> dict:
    if stride <= 0:
        raise ValueError("stride must be positive")

    with zipfile.ZipFile(source_zip) as archive:
        members = [name for name in archive.namelist() if name.lower().endswith(".ply")]
        if len(members) != 1:
            raise ValueError(f"expected exactly one PLY member, found {members!r}")
        member = members[0]

        chunks: list[np.ndarray] = []
        chunk_rows: list[tuple[float, float, float]] = []
        selected_count = 0

        with archive.open(member, "r") as stream:
            header, vertex_count = read_header(stream)
            for vertex_index in range(vertex_count):
                raw = stream.readline()
                if not raw:
                    raise ValueError(
                        f"unexpected EOF at vertex {vertex_index}/{vertex_count}"
                    )
                if vertex_index % stride:
                    continue
                fields = raw.split()
                if len(fields) != 7:
                    raise ValueError(
                        f"vertex {vertex_index} has {len(fields)} fields, expected 7"
                    )
                chunk_rows.append(
                    (float(fields[0]), float(fields[1]), float(fields[2]))
                )
                selected_count += 1
                if len(chunk_rows) >= 250_000:
                    chunks.append(np.asarray(chunk_rows, dtype=np.float32))
                    chunk_rows.clear()

            if stream.readline():
                raise ValueError("PLY contains data after declared vertex rows")

        if chunk_rows:
            chunks.append(np.asarray(chunk_rows, dtype=np.float32))
        points = np.concatenate(chunks, axis=0)
        expected_count = (vertex_count + stride - 1) // stride
        if selected_count != expected_count or points.shape != (expected_count, 3):
            raise AssertionError(
                f"selection invariant failed: {selected_count}, {points.shape}, "
                f"expected {expected_count}"
            )
        if not np.isfinite(points).all():
            raise ValueError("selected point array contains non-finite values")

        output_npy.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            dir=output_npy.parent, suffix=".npy", delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            np.save(temporary, points, allow_pickle=False)
        temporary_path.replace(output_npy)

    metadata = {
        "schema": "dtslam_bonn_static_model_stride_sample_v1",
        "evaluation_only": True,
        "source_zip": str(source_zip.resolve()),
        "source_zip_sha256": sha256_file(source_zip),
        "source_member": member,
        "source_header": header,
        "source_vertex_count": vertex_count,
        "selection": {
            "method": "zero_based_vertex_index_modulo_stride",
            "stride": stride,
            "selected_vertex_count": int(points.shape[0]),
        },
        "point_dtype": str(points.dtype),
        "point_shape": list(points.shape),
        "bounds_min_xyz": points.min(axis=0).astype(float).tolist(),
        "bounds_max_xyz": points.max(axis=0).astype(float).tolist(),
        "output_npy": str(output_npy.resolve()),
        "output_npy_sha256": sha256_file(output_npy),
    }
    metadata_path = output_npy.with_suffix(output_npy.suffix + ".json")
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def self_test() -> None:
    with tempfile.TemporaryDirectory(prefix="dtslam_bonn_model_test_") as directory:
        root = Path(directory)
        source = root / "model.zip"
        output = root / "points.npy"
        rows = [
            "ply",
            "format ascii 1.0",
            "element vertex 5",
            *EXPECTED_PROPERTIES,
            "end_header",
            "0 1 2 1 2 3 4",
            "1 2 3 1 2 3 4",
            "2 3 4 1 2 3 4",
            "3 4 5 1 2 3 4",
            "4 5 6 1 2 3 4",
            "",
        ]
        with zipfile.ZipFile(source, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("model.ply", "\n".join(rows))
        metadata = prepare(source, output, stride=2)
        points = np.load(output, allow_pickle=False)
        expected = np.asarray([[0, 1, 2], [2, 3, 4], [4, 5, 6]], np.float32)
        np.testing.assert_array_equal(points, expected)
        assert metadata["selection"]["selected_vertex_count"] == 3
    print("prepare_bonn_static_model self-test: PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-zip", type=Path)
    parser.add_argument("--output-npy", type=Path)
    parser.add_argument("--stride", type=int, default=16)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if not args.self_test and (args.source_zip is None or args.output_npy is None):
        parser.error("--source-zip and --output-npy are required")
    return args


def main() -> None:
    args = parse_args()
    if args.self_test:
        self_test()
        return
    metadata = prepare(args.source_zip, args.output_npy, args.stride)
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
