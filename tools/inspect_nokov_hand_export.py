#!/usr/bin/env python3
"""Inspect a NOKOV/XINGYING TRC, C3D or SDK marker CSV export.

The script is intentionally read-only unless --json-out or
--write-marker-names is supplied. TRC parsing uses only the Python standard
library. C3D parsing requires the optional ``ezc3d`` package.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="XINGYING .trc/.c3d or SDK nokov_markers.csv")
    parser.add_argument(
        "--markerset", help="for SDK CSV, inspect only this exact MarkerSet name"
    )
    parser.add_argument(
        "--expected-markers", type=int, default=24,
        help="expected marker count; use 0 to disable the count warning",
    )
    parser.add_argument(
        "--zero-is-missing", action=argparse.BooleanOptionalAction, default=True,
        help="treat an XYZ triplet of all zeros as a missing observation",
    )
    parser.add_argument("--json-out", type=Path, help="write the inspection report")
    parser.add_argument(
        "--write-marker-names", type=Path,
        help="write a marker-name template for manual semantic annotation",
    )
    return parser.parse_args()


def read_text(path: Path) -> tuple[str, str]:
    raw = path.read_bytes()
    for encoding in ("utf-8-sig", "gb18030", "utf-16", "latin-1"):
        try:
            return raw.decode(encoding), encoding
        except UnicodeDecodeError:
            continue
    raise RuntimeError(f"unable to decode text file: {path}")


def split_row(line: str) -> list[str]:
    if "\t" in line:
        return [cell.strip() for cell in line.rstrip("\r\n").split("\t")]
    if "," in line:
        return [cell.strip() for cell in line.rstrip("\r\n").split(",")]
    return line.strip().split()


def finite_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def marker_names_from_header(cells: list[str]) -> list[str]:
    payload = cells[2:]
    if not payload:
        return []
    # Standard TRC stores a name at the first column of every XYZ triplet and
    # leaves the following two cells empty. Some exporters repeat each name.
    grouped = []
    for start in range(0, len(payload), 3):
        group = [value for value in payload[start : start + 3] if value]
        grouped.append(group[0] if group else "")
    if any(grouped):
        return [name or f"unnamed_{index:02d}" for index, name in enumerate(grouped)]
    return []


def inspect_trc(path: Path, zero_is_missing: bool) -> dict[str, Any]:
    text, encoding = read_text(path)
    lines = text.splitlines()
    header_index = next(
        (index for index, line in enumerate(lines) if split_row(line)[:2] == ["Frame#", "Time"]),
        None,
    )
    if header_index is None:
        raise RuntimeError("TRC header row 'Frame# Time' was not found")

    header_cells = split_row(lines[header_index])
    marker_names = marker_names_from_header(header_cells)
    if not marker_names:
        raise RuntimeError("TRC marker names could not be read")

    metadata: dict[str, Any] = {}
    if header_index >= 2:
        keys = split_row(lines[header_index - 2])
        values = split_row(lines[header_index - 1])
        for key, value in zip(keys, values):
            if key:
                numeric = finite_float(value)
                metadata[key] = numeric if numeric is not None else value

    frame_numbers: list[int] = []
    times: list[float] = []
    visible = [0] * len(marker_names)
    missing = [0] * len(marker_names)
    bounds = [
        {"min": [math.inf, math.inf, math.inf], "max": [-math.inf, -math.inf, -math.inf]}
        for _ in marker_names
    ]

    for line in lines[header_index + 1 :]:
        cells = split_row(line)
        if len(cells) < 2:
            continue
        frame_value = finite_float(cells[0])
        time_value = finite_float(cells[1])
        if frame_value is None or time_value is None:
            continue
        frame_numbers.append(int(frame_value))
        times.append(time_value)
        payload = cells[2:]
        for marker_index in range(len(marker_names)):
            start = marker_index * 3
            xyz = [
                finite_float(payload[start + axis]) if start + axis < len(payload) else None
                for axis in range(3)
            ]
            invalid = any(value is None for value in xyz)
            if not invalid and zero_is_missing:
                invalid = all(abs(float(value)) < 1e-12 for value in xyz if value is not None)
            if invalid:
                missing[marker_index] += 1
                continue
            visible[marker_index] += 1
            for axis, value in enumerate(xyz):
                assert value is not None
                bounds[marker_index]["min"][axis] = min(bounds[marker_index]["min"][axis], value)
                bounds[marker_index]["max"][axis] = max(bounds[marker_index]["max"][axis], value)

    if not frame_numbers:
        raise RuntimeError("TRC contains no numeric data frames")

    intervals = [later - earlier for earlier, later in zip(times, times[1:]) if later > earlier]
    measured_fps = 1.0 / statistics.median(intervals) if intervals else None
    per_marker = []
    for index, name in enumerate(marker_names):
        total = visible[index] + missing[index]
        marker_bounds = bounds[index] if visible[index] else None
        per_marker.append({
            "index": index,
            "name": name,
            "visible_frames": visible[index],
            "missing_frames": missing[index],
            "visible_ratio": visible[index] / total if total else 0.0,
            "bounds": marker_bounds,
        })

    return {
        "format": "trc",
        "path": str(path.resolve()),
        "encoding": encoding,
        "metadata": metadata,
        "frame_count": len(frame_numbers),
        "first_frame": frame_numbers[0],
        "last_frame": frame_numbers[-1],
        "first_time": times[0],
        "last_time": times[-1],
        "measured_fps": measured_fps,
        "marker_count": len(marker_names),
        "marker_names": marker_names,
        "duplicate_marker_names": sorted({name for name in marker_names if marker_names.count(name) > 1}),
        "per_marker": per_marker,
    }


def inspect_c3d(path: Path, zero_is_missing: bool) -> dict[str, Any]:
    try:
        import ezc3d  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "C3D inspection requires ezc3d. Install it with: "
            "python3 -m pip install ezc3d"
        ) from exc

    import numpy as np

    archive = ezc3d.c3d(str(path))
    parameters = archive["parameters"]["POINT"]
    names = [str(value).strip() for value in parameters["LABELS"]["value"]]
    points = np.asarray(archive["data"]["points"], dtype=np.float64)
    if points.ndim != 3 or points.shape[0] < 3:
        raise RuntimeError(f"unexpected C3D point array shape: {points.shape}")
    # ezc3d stores points as (4, marker, frame); normalize to
    # (frame, marker, xyz) for the same semantics as the TRC path.
    xyz = np.transpose(points[:3], (2, 1, 0))
    valid = np.isfinite(xyz).all(axis=2)
    if zero_is_missing:
        valid &= np.linalg.norm(xyz, axis=2) > 1e-12
    fps = float(parameters["RATE"]["value"][0])
    units = str(parameters.get("UNITS", {}).get("value", ["unknown"])[0])
    per_marker = []
    for index, name in enumerate(names):
        marker_valid = valid[:, index]
        values = xyz[marker_valid, index]
        per_marker.append({
            "index": index,
            "name": name,
            "visible_frames": int(marker_valid.sum()),
            "missing_frames": int((~marker_valid).sum()),
            "visible_ratio": float(marker_valid.mean()),
            "bounds": {
                "min": values.min(axis=0).tolist(),
                "max": values.max(axis=0).tolist(),
            } if len(values) else None,
        })
    return {
        "format": "c3d",
        "path": str(path.resolve()),
        "metadata": {"units": units, "data_rate_hz": fps},
        "frame_count": int(xyz.shape[0]),
        "first_frame": 0,
        "last_frame": int(xyz.shape[0] - 1),
        "first_time": 0.0,
        "last_time": float((xyz.shape[0] - 1) / fps) if fps > 0 else None,
        "measured_fps": fps,
        "marker_count": len(names),
        "marker_names": names,
        "duplicate_marker_names": sorted({name for name in names if names.count(name) > 1}),
        "per_marker": per_marker,
    }


def inspect_sdk_csv(
    path: Path, zero_is_missing: bool, selected_markerset: str | None
) -> dict[str, Any]:
    required = {
        "frame_no", "receive_perf_ns", "markerset_name", "marker_index",
        "marker_name", "valid", "x_mm", "y_mm", "z_mm",
    }
    observations: dict[tuple[str, int, str], dict[str, Any]] = {}
    frames: set[int] = set()
    perf_by_frame: dict[int, int] = {}
    seen_sets: set[str] = set()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        missing_fields = sorted(required - fields)
        if missing_fields:
            raise RuntimeError(f"SDK CSV is missing columns: {missing_fields}")
        for row in reader:
            marker_set = (row.get("markerset_name") or "").strip()
            if selected_markerset and marker_set != selected_markerset:
                continue
            try:
                frame_no = int(row["frame_no"])
                marker_index = int(row["marker_index"])
                receive_perf_ns = int(row["receive_perf_ns"])
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"invalid SDK CSV integer field: {row}") from exc
            marker_name = (row.get("marker_name") or f"marker_{marker_index:02d}").strip()
            seen_sets.add(marker_set)
            frames.add(frame_no)
            perf_by_frame.setdefault(frame_no, receive_perf_ns)
            key = (marker_set, marker_index, marker_name)
            item = observations.setdefault(key, {
                "visible": 0,
                "observed": set(),
                "min": [math.inf, math.inf, math.inf],
                "max": [-math.inf, -math.inf, -math.inf],
            })
            item["observed"].add(frame_no)
            xyz = [finite_float(row.get(axis, "")) for axis in ("x_mm", "y_mm", "z_mm")]
            valid_flag = str(row.get("valid", "")).strip().lower() in ("1", "true", "yes")
            valid = (
                valid_flag
                and all(value is not None for value in xyz)
                # NOKOV SDK uses 9999999.0 for an untracked point while some
                # releases still report valid=1.
                and all(abs(float(value)) < 1_000_000.0 for value in xyz if value is not None)
            )
            if valid and zero_is_missing:
                valid = not all(abs(float(value)) < 1e-12 for value in xyz if value is not None)
            if not valid:
                continue
            item["visible"] += 1
            for axis, value in enumerate(xyz):
                assert value is not None
                item["min"][axis] = min(item["min"][axis], value)
                item["max"][axis] = max(item["max"][axis], value)

    if not frames:
        detail = f" for MarkerSet {selected_markerset!r}" if selected_markerset else ""
        raise RuntimeError(f"SDK CSV contains no marker observations{detail}")
    frame_numbers = sorted(frames)
    ordered_perf = [perf_by_frame[frame] for frame in frame_numbers]
    intervals = [
        (later - earlier) / 1e9
        for earlier, later in zip(ordered_perf, ordered_perf[1:]) if later > earlier
    ]
    measured_fps = 1.0 / statistics.median(intervals) if intervals else None
    keys = sorted(observations, key=lambda value: (value[0], value[1], value[2]))
    display_names = [
        name if len(seen_sets) == 1 else f"{marker_set}/{name}"
        for marker_set, _index, name in keys
    ]
    per_marker = []
    for display_name, key in zip(display_names, keys):
        item = observations[key]
        visible = int(item["visible"])
        per_marker.append({
            "index": key[1],
            "markerset_name": key[0],
            "name": display_name,
            "visible_frames": visible,
            # A missing row also means that marker was absent from that frame.
            "missing_frames": len(frames) - visible,
            "visible_ratio": visible / len(frames),
            "bounds": {"min": item["min"], "max": item["max"]} if visible else None,
        })
    return {
        "format": "nokov_sdk_csv",
        "path": str(path.resolve()),
        "metadata": {"units": "mm", "markerset_names": sorted(seen_sets)},
        "frame_count": len(frames),
        "first_frame": frame_numbers[0],
        "last_frame": frame_numbers[-1],
        "first_time": 0.0,
        "last_time": (
            (ordered_perf[-1] - ordered_perf[0]) / 1e9 if len(ordered_perf) > 1 else 0.0
        ),
        "measured_fps": measured_fps,
        "marker_count": len(keys),
        "marker_names": display_names,
        "duplicate_marker_names": sorted({
            name for name in display_names if display_names.count(name) > 1
        }),
        "per_marker": per_marker,
    }


def print_report(report: dict[str, Any], expected_markers: int) -> list[str]:
    warnings: list[str] = []
    print(f"Format:       {report['format'].upper()}")
    print(f"File:         {report['path']}")
    print(f"Frames:       {report['frame_count']}")
    print(f"Markers:      {report['marker_count']}")
    print(f"Measured FPS: {report['measured_fps']}")
    if expected_markers and report["marker_count"] != expected_markers:
        warnings.append(
            f"marker count is {report['marker_count']}, expected {expected_markers}; "
            "the export may include both hands/head markers or may be incomplete"
        )
    if report["duplicate_marker_names"]:
        warnings.append(f"duplicate marker names: {report['duplicate_marker_names']}")
    print("\nMarker visibility:")
    for marker in report["per_marker"]:
        print(
            f"  {marker['index']:02d}  {marker['name']:<28} "
            f"{100.0 * marker['visible_ratio']:6.2f}% visible "
            f"({marker['missing_frames']} missing)"
        )
        if marker["visible_ratio"] < 0.95:
            warnings.append(
                f"{marker['name']} visibility is only {100.0 * marker['visible_ratio']:.2f}%"
            )
    if warnings:
        print("\nWarnings:")
        for warning in warnings:
            print(f"  - {warning}")
    else:
        print("\nNo structural warnings.")
    return warnings


def write_marker_template(path: Path, names: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# hand,marker_index,marker_name,mediapipe_joint_or_auxiliary",
        "# Fill the last column manually after checking the XINGYING model.",
    ]
    lines.extend(f"TODO,{index},{name},TODO" for index, name in enumerate(names))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    path = args.input.resolve()
    if not path.is_file():
        print(f"[FAIL] input file does not exist: {path}", file=sys.stderr)
        return 2
    try:
        if path.suffix.lower() == ".trc":
            report = inspect_trc(path, args.zero_is_missing)
        elif path.suffix.lower() == ".c3d":
            report = inspect_c3d(path, args.zero_is_missing)
        elif path.suffix.lower() == ".csv":
            report = inspect_sdk_csv(path, args.zero_is_missing, args.markerset)
        else:
            raise RuntimeError("input extension must be .trc, .c3d or .csv")
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2

    warnings = print_report(report, args.expected_markers)
    report["warnings"] = warnings
    report["status"] = "warning" if warnings else "ok"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Report written: {args.json_out.resolve()}")
    if args.write_marker_names:
        write_marker_template(args.write_marker_names, report["marker_names"])
        print(f"Marker template written: {args.write_marker_names.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
