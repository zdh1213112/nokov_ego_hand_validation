#!/usr/bin/env python3
"""Validate EGO six-view accepted.jsonl and its timestamp table."""

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
    parser.add_argument("--fusion", required=True, type=Path, help="accepted.jsonl")
    parser.add_argument("--timestamps", required=True, type=Path, help="multiview_frames.csv")
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args()


def joints_shape_valid(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 21:
        return False
    for point in value:
        if not isinstance(point, list) or len(point) != 3:
            return False
        if not all(isinstance(cell, (int, float)) for cell in point):
            return False
    return True


def main() -> int:
    args = parse_args()
    fusion = args.fusion.resolve()
    timestamps_path = args.timestamps.resolve()
    if not fusion.is_file() or not timestamps_path.is_file():
        print("[FAIL] fusion or timestamp file does not exist", file=sys.stderr)
        return 2

    timestamps: dict[int, int] = {}
    with timestamps_path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            timestamps[int(row["sync_index"])] = int(row["reference_timestamp_ns"])

    row_count = 0
    hand_count = 0
    side_counts = {0: 0, 1: 0}
    invalid_rows = 0
    invalid_hands = 0
    missing_timestamps = 0
    duplicate_sides = 0
    partial_hands = 0
    missing_joint_count = 0
    max_abs_coordinate_m = 0.0
    accepted_indices: list[int] = []
    with fusion.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            row_count += 1
            try:
                row = json.loads(line)
                sync_index = int(row["sync_index"])
                hands = row["hands"]
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                invalid_rows += 1
                continue
            accepted_indices.append(sync_index)
            if sync_index not in timestamps:
                missing_timestamps += 1
            seen_sides: set[int] = set()
            if not isinstance(hands, list):
                invalid_rows += 1
                continue
            for hand in hands:
                hand_count += 1
                try:
                    side = int(hand["side"])
                    joints = hand["joints_base_m"]
                except (KeyError, TypeError, ValueError):
                    invalid_hands += 1
                    continue
                if side not in (0, 1) or not joints_shape_valid(joints):
                    invalid_hands += 1
                    continue
                if side in seen_sides:
                    duplicate_sides += 1
                seen_sides.add(side)
                side_counts[side] += 1
                finite_points = [
                    point for point in joints
                    if all(math.isfinite(float(cell)) for cell in point)
                ]
                if len(finite_points) < 21:
                    partial_hands += 1
                    missing_joint_count += 21 - len(finite_points)
                if finite_points:
                    max_abs_coordinate_m = max(
                        max_abs_coordinate_m,
                        max(abs(float(cell)) for point in finite_points for cell in point),
                    )

    ordered_times = [timestamps[index] for index in sorted(timestamps)]
    intervals = [later - earlier for earlier, later in zip(ordered_times, ordered_times[1:]) if later > earlier]
    median_interval_ns = statistics.median(intervals) if intervals else None
    fps = 1e9 / median_interval_ns if median_interval_ns else None
    warnings = []
    if max_abs_coordinate_m > 10.0:
        warnings.append("coordinates exceed 10 m; check whether millimetres were mislabeled as metres")
    if missing_timestamps:
        warnings.append(f"{missing_timestamps} accepted rows have no matching timestamp")
    if duplicate_sides:
        warnings.append(f"{duplicate_sides} frames contain duplicate physical hand sides")
    if partial_hands:
        warnings.append(
            f"{partial_hands} hands are partial ({missing_joint_count} non-finite joints); "
            "keep their per-joint validity mask during evaluation"
        )
    if invalid_rows or invalid_hands:
        warnings.append(f"invalid_rows={invalid_rows}, invalid_hands={invalid_hands}")

    report = {
        "schema": "ego_multiview_output_inspection_v1",
        "fusion": str(fusion),
        "timestamps": str(timestamps_path),
        "fusion_row_count": row_count,
        "hand_count": hand_count,
        "left_hand_count": side_counts[0],
        "right_hand_count": side_counts[1],
        "timestamp_row_count": len(timestamps),
        "median_frame_interval_ns": median_interval_ns,
        "measured_fps": fps,
        "max_abs_coordinate_m": max_abs_coordinate_m,
        "invalid_rows": invalid_rows,
        "invalid_hands": invalid_hands,
        "missing_timestamps": missing_timestamps,
        "duplicate_sides": duplicate_sides,
        "partial_hands": partial_hands,
        "missing_joint_count": missing_joint_count,
        "warnings": warnings,
        "status": "warning" if warnings else "ok",
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Report written: {args.json_out.resolve()}")
    return 0 if not invalid_rows and not invalid_hands else 2


if __name__ == "__main__":
    raise SystemExit(main())
