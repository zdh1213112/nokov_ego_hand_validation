#!/usr/bin/env python3
"""Estimate EGO-to-NOKOV time offset from IMU and head rigid-body rotation.

The first-stage mapping fixes clock scale a=1 and estimates b in:

    nokov_relative_s = ego_relative_s + b

EGO angular speed is read from /robot0/sensor/imu. NOKOV angular speed is
computed from consecutive rigid-body quaternions. Their norms are invariant to
the unknown fixed rotation between the raw IMU and NOKOV rigid-body frames.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego-mcap", type=Path, required=True)
    parser.add_argument("--nokov-csv", type=Path, required=True)
    parser.add_argument("--rigid-body", default="head_rigidbody")
    parser.add_argument("--imu-topic", default="/robot0/sensor/imu")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--resample-hz", type=float, default=100.0)
    parser.add_argument("--smooth-ms", type=float, default=80.0)
    parser.add_argument("--max-offset-s", type=float, default=30.0)
    parser.add_argument("--min-overlap-s", type=float, default=8.0)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=15.0)
    parser.add_argument(
        "--nokov-time-field", default="receive_perf_ns",
        choices=("receive_perf_ns", "receive_unix_ns", "device_timestamp_raw"),
    )
    parser.add_argument(
        "--nokov-time-scale", type=float, default=None,
        help="seconds per NOKOV timestamp unit; defaults to 1e-9 for receive_*_ns",
    )
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def scalar_timestamp_ns(value: Any, fallback: int) -> int:
    if value is None:
        return fallback
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, float):
        return int(value)
    if hasattr(value, "seconds") and hasattr(value, "nanos"):
        return int(value.seconds) * 1_000_000_000 + int(value.nanos)
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def read_ego_gyro(path: Path, topic: str) -> tuple[np.ndarray, np.ndarray]:
    try:
        from mcap.reader import make_reader
        from mcap_protobuf.decoder import DecoderFactory
    except ImportError as exc:
        raise RuntimeError(
            "missing MCAP packages; install tools/requirements-sync.txt"
        ) from exc

    timestamps: list[int] = []
    gyro: list[tuple[float, float, float]] = []
    with path.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _schema, _channel, message, decoded in reader.iter_decoded_messages(
            topics=[topic]
        ):
            header = getattr(decoded, "header", None)
            stamp = scalar_timestamp_ns(
                getattr(header, "timestamp", None), int(message.log_time)
            )
            angular = getattr(decoded, "angular_velocity", None)
            if angular is None:
                continue
            values = (float(angular.x), float(angular.y), float(angular.z))
            if not all(math.isfinite(value) for value in values):
                continue
            timestamps.append(stamp)
            gyro.append(values)
    if len(timestamps) < 20:
        raise RuntimeError(f"only {len(timestamps)} valid IMU rows found on {topic}")

    ts = np.asarray(timestamps, dtype=np.int64)
    values = np.asarray(gyro, dtype=np.float64)
    order = np.argsort(ts, kind="stable")
    ts, values = ts[order], values[order]
    keep = np.r_[True, np.diff(ts) > 0]
    return ts[keep], values[keep]


def parse_int(row: dict[str, str], key: str, default: int = 0) -> int:
    value = row.get(key, "")
    if value is None or value == "":
        return default
    return int(float(value))


def read_nokov_quaternions(
    path: Path, rigid_body: str, time_field: str
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    rows: list[tuple[int, np.ndarray, int]] = []
    seen_names: set[str] = set()
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {time_field, "rigid_body_name", "qx", "qy", "qz", "qw"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f"NOKOV CSV is missing fields: {sorted(missing)}")
        for row in reader:
            name = row.get("rigid_body_name", "")
            seen_names.add(name)
            if name != rigid_body or parse_int(row, "valid_numeric", 1) == 0:
                continue
            try:
                stamp = parse_int(row, time_field)
                quat = np.asarray(
                    [float(row[key]) for key in ("qx", "qy", "qz", "qw")],
                    dtype=np.float64,
                )
                params = parse_int(row, "params", 1)
            except (TypeError, ValueError):
                continue
            norm = float(np.linalg.norm(quat))
            if stamp and np.isfinite(quat).all() and norm > 0.5:
                rows.append((stamp, quat / norm, params))
    if len(rows) < 10:
        raise RuntimeError(
            f"only {len(rows)} valid rows for rigid body {rigid_body!r}; "
            f"available names={sorted(seen_names)!r}"
        )

    rows.sort(key=lambda item: item[0])
    has_tracking_bits = any(params & 1 for _, _, params in rows)
    if has_tracking_bits:
        tracked = [item for item in rows if item[2] & 1]
        if len(tracked) >= 10:
            rows = tracked
    timestamps = np.asarray([item[0] for item in rows], dtype=np.int64)
    quaternions = np.asarray([item[1] for item in rows], dtype=np.float64)
    keep = np.r_[True, np.diff(timestamps) > 0]
    return timestamps[keep], quaternions[keep], {
        "available_rigid_body_names": sorted(seen_names),
        "tracking_valid_bit_used": bool(has_tracking_bits),
    }


def quaternion_angular_speed(
    timestamps_raw: np.ndarray,
    quaternions: np.ndarray,
    seconds_per_unit: float,
    max_speed: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    time_s = (timestamps_raw - timestamps_raw[0]).astype(np.float64) * seconds_per_unit
    dt = np.diff(time_s)
    positive = dt > 0
    if not np.any(positive):
        raise RuntimeError("NOKOV timestamps are not increasing")
    median_dt = float(np.median(dt[positive]))
    total_duration = float(time_s[-1] - time_s[0])
    mean_pose_rate = (
        float(len(time_s) - 1) / total_duration if total_duration > 0 else float("nan")
    )
    dots = np.abs(np.sum(quaternions[:-1] * quaternions[1:], axis=1))
    dots = np.clip(dots, 0.0, 1.0)
    angle = 2.0 * np.arccos(dots)
    speed = np.divide(angle, dt, out=np.full_like(angle, np.nan), where=dt > 0)
    midpoint = 0.5 * (time_s[:-1] + time_s[1:])
    keep = (
        np.isfinite(speed)
        & (dt > 0)
        & (dt <= max(0.1, 5.0 * median_dt))
        & (speed <= max_speed)
    )
    if int(np.count_nonzero(keep)) < 10:
        raise RuntimeError("too few valid NOKOV angular-speed samples")
    return midpoint[keep], speed[keep], {
        "pose_rate_hz": mean_pose_rate,
        "median_timestamp_step_s": median_dt,
        "filtered_angular_speed_rows": int(np.count_nonzero(~keep)),
    }


def uniform_signal(
    time_s: np.ndarray, values: np.ndarray, rate_hz: float, smooth_ms: float
) -> tuple[np.ndarray, np.ndarray]:
    duration = float(time_s[-1])
    if duration <= 0:
        raise RuntimeError("signal duration is zero")
    grid = np.arange(0.0, duration, 1.0 / rate_hz, dtype=np.float64)
    signal = np.interp(grid, time_s, values)
    window = max(1, int(round(smooth_ms * 1e-3 * rate_hz)))
    if window > 1:
        if window % 2 == 0:
            window += 1
        half = window // 2
        padded = np.pad(signal, (half, half), mode="edge")
        signal = np.convolve(padded, np.ones(window) / window, mode="valid")
    return grid, signal


def pearson(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3:
        return float("nan")
    aa = a - float(np.mean(a))
    bb = b - float(np.mean(b))
    denom = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    return float(np.dot(aa, bb) / denom) if denom > 1e-12 else float("nan")


def estimate_offset(
    ego: np.ndarray,
    nokov: np.ndarray,
    rate_hz: float,
    max_offset_s: float,
    min_overlap_s: float,
) -> tuple[float, float, float, float, np.ndarray, np.ndarray]:
    max_shift = int(round(max_offset_s * rate_hz))
    minimum = int(round(min_overlap_s * rate_hz))
    shifts: list[int] = []
    correlations: list[float] = []
    for shift in range(-max_shift, max_shift + 1):
        ego_start = max(0, -shift)
        nokov_start = max(0, shift)
        count = min(len(ego) - ego_start, len(nokov) - nokov_start)
        if count < minimum:
            continue
        corr = pearson(
            ego[ego_start:ego_start + count],
            nokov[nokov_start:nokov_start + count],
        )
        if math.isfinite(corr):
            shifts.append(shift)
            correlations.append(corr)
    if not correlations:
        raise RuntimeError("no candidate offset has enough overlapping data")

    shifts_array = np.asarray(shifts, dtype=np.int64)
    corr_array = np.asarray(correlations, dtype=np.float64)
    best_index = int(np.argmax(corr_array))
    best_shift = float(shifts_array[best_index])
    if 0 < best_index < len(corr_array) - 1:
        left, center, right = corr_array[best_index - 1:best_index + 2]
        curvature = left - 2.0 * center + right
        if abs(curvature) > 1e-12:
            delta = 0.5 * (left - right) / curvature
            if abs(delta) <= 1.0:
                best_shift += float(delta)
    offset_s = best_shift / rate_hz

    exclusion = max(1, int(round(1.0 * rate_hz)))
    far = np.abs(shifts_array - shifts_array[best_index]) > exclusion
    second = float(np.max(corr_array[far])) if np.any(far) else float("nan")
    margin = float(corr_array[best_index] - second) if math.isfinite(second) else float("nan")
    return (
        offset_s,
        float(corr_array[best_index]),
        second,
        margin,
        shifts_array / rate_hz,
        corr_array,
    )


def write_aligned_csv(
    path: Path,
    ego_grid: np.ndarray,
    ego_signal: np.ndarray,
    nokov_grid: np.ndarray,
    nokov_signal: np.ndarray,
    offset_s: float,
    ego_origin_ns: int,
    nokov_origin_raw: int,
    nokov_scale: float,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    target_nokov_s = ego_grid + offset_s
    valid = (target_nokov_s >= nokov_grid[0]) & (target_nokov_s <= nokov_grid[-1])
    aligned_nokov = np.full_like(ego_signal, np.nan)
    aligned_nokov[valid] = np.interp(
        target_nokov_s[valid], nokov_grid, nokov_signal
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        fields = (
            "ego_relative_s", "ego_timestamp_ns", "ego_gyro_norm_rad_s",
            "nokov_target_relative_s", "nokov_target_timestamp_raw",
            "nokov_angular_speed_rad_s", "valid_overlap",
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(len(ego_grid)):
            writer.writerow({
                "ego_relative_s": f"{ego_grid[index]:.9f}",
                "ego_timestamp_ns": ego_origin_ns + int(round(ego_grid[index] * 1e9)),
                "ego_gyro_norm_rad_s": f"{ego_signal[index]:.9f}",
                "nokov_target_relative_s": f"{target_nokov_s[index]:.9f}",
                "nokov_target_timestamp_raw": (
                    f"{nokov_origin_raw + target_nokov_s[index] / nokov_scale:.3f}"
                ),
                "nokov_angular_speed_rad_s": (
                    f"{aligned_nokov[index]:.9f}" if valid[index] else ""
                ),
                "valid_overlap": int(valid[index]),
            })
    overlap_s = float(np.count_nonzero(valid)) * float(np.median(np.diff(ego_grid)))
    return overlap_s, target_nokov_s, aligned_nokov, valid


def save_plot(
    path: Path,
    ego_grid: np.ndarray,
    ego_signal: np.ndarray,
    aligned_nokov: np.ndarray,
    valid: np.ndarray,
    offsets: np.ndarray,
    correlations: np.ndarray,
    best_offset: float,
) -> str | None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return "matplotlib is unavailable; plot was not written"
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), constrained_layout=True)
    axes[0].plot(ego_grid, ego_signal, label="EGO IMU |gyro|", linewidth=1.0)
    axes[0].plot(
        ego_grid[valid], aligned_nokov[valid],
        label="NOKOV rigid |omega| aligned", linewidth=1.0,
    )
    axes[0].set_xlabel("EGO relative time (s)")
    axes[0].set_ylabel("angular speed (rad/s)")
    axes[0].legend()
    axes[0].grid(alpha=0.25)
    axes[1].plot(offsets, correlations, linewidth=1.0)
    axes[1].axvline(best_offset, color="red", linestyle="--", label=f"b={best_offset:.4f}s")
    axes[1].set_xlabel("candidate b: NOKOV relative = EGO relative + b (s)")
    axes[1].set_ylabel("Pearson correlation")
    axes[1].legend()
    axes[1].grid(alpha=0.25)
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return None


def main() -> int:
    args = parse_args()
    if args.resample_hz <= 0 or args.max_offset_s <= 0 or args.min_overlap_s <= 0:
        print("[FAIL] rates and durations must be positive", file=sys.stderr)
        return 2
    ego_path = args.ego_mcap.resolve()
    nokov_path = args.nokov_csv.resolve()
    if not ego_path.is_file() or not nokov_path.is_file():
        print(f"[FAIL] missing input: EGO={ego_path} NOKOV={nokov_path}", file=sys.stderr)
        return 2
    nokov_scale = args.nokov_time_scale
    if nokov_scale is None:
        if args.nokov_time_field in ("receive_perf_ns", "receive_unix_ns"):
            nokov_scale = 1e-9
        else:
            print(
                "[FAIL] --nokov-time-scale is required with device_timestamp_raw",
                file=sys.stderr,
            )
            return 2

    try:
        ego_ts, ego_gyro = read_ego_gyro(ego_path, args.imu_topic)
        nokov_ts, nokov_quat, nokov_info = read_nokov_quaternions(
            nokov_path, args.rigid_body, args.nokov_time_field
        )
        ego_time = (ego_ts - ego_ts[0]).astype(np.float64) * 1e-9
        ego_speed = np.linalg.norm(ego_gyro, axis=1)
        keep_ego = np.isfinite(ego_speed) & (ego_speed <= args.max_angular_speed_rad_s)
        ego_time, ego_speed = ego_time[keep_ego], ego_speed[keep_ego]
        nokov_time, nokov_speed, quaternion_info = quaternion_angular_speed(
            nokov_ts, nokov_quat, nokov_scale, args.max_angular_speed_rad_s
        )
        ego_grid, ego_uniform = uniform_signal(
            ego_time, ego_speed, args.resample_hz, args.smooth_ms
        )
        nokov_grid, nokov_uniform = uniform_signal(
            nokov_time, nokov_speed, args.resample_hz, args.smooth_ms
        )
        offset_s, corr, second_corr, margin, offsets, correlations = estimate_offset(
            ego_uniform, nokov_uniform, args.resample_hz,
            args.max_offset_s, args.min_overlap_s,
        )
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    aligned_csv = output / "imu_nokov_aligned_signals.csv"
    overlap_s, _target_nokov, aligned_nokov, valid = write_aligned_csv(
        aligned_csv, ego_grid, ego_uniform, nokov_grid, nokov_uniform,
        offset_s, int(ego_ts[0]), int(nokov_ts[0]), nokov_scale,
    )
    aligned_corr = pearson(ego_uniform[valid], aligned_nokov[valid])
    ego_activity = float(np.percentile(ego_uniform, 95) - np.percentile(ego_uniform, 20))
    nokov_activity = float(np.percentile(nokov_uniform, 95) - np.percentile(nokov_uniform, 20))
    if aligned_corr >= 0.75 and (not math.isfinite(margin) or margin >= 0.05):
        confidence = "strong"
    elif aligned_corr >= 0.55 and (not math.isfinite(margin) or margin >= 0.02):
        confidence = "usable"
    else:
        confidence = "weak"
    warnings: list[str] = []
    if confidence == "weak":
        warnings.append("sync peak is weak or ambiguous; repeat a more distinctive head-motion sequence")
    if ego_activity < 0.2 or nokov_activity < 0.2:
        warnings.append("angular activity is low; add clear yaw, pitch and roll pulses")
    if abs(offset_s) >= args.max_offset_s - 1.0 / args.resample_hz:
        warnings.append("best offset reached the search boundary; increase --max-offset-s")
    if min(float(ego_grid[-1]), float(nokov_grid[-1])) > 120.0:
        warnings.append("long recording detected; first-stage a=1 ignores clock drift")

    result = {
        "schema": "ego_nokov_imu_sync_v1",
        "status": "ok" if confidence in ("strong", "usable") else "needs_review",
        "confidence": confidence,
        "method": "angular_speed_norm_cross_correlation",
        "time_mapping": {
            "equation": "nokov_relative_s = a * ego_relative_s + b",
            "a": 1.0,
            "b_s": offset_s,
            "ego_origin_timestamp_ns": int(ego_ts[0]),
            "nokov_origin_field": args.nokov_time_field,
            "nokov_origin_timestamp_raw": int(nokov_ts[0]),
            "nokov_seconds_per_timestamp_unit": nokov_scale,
            "ego_to_nokov_raw_equation": (
                "nokov_raw = nokov_origin_raw + "
                "(((ego_timestamp_ns-ego_origin_timestamp_ns)*1e-9)+b_s) / "
                "nokov_seconds_per_timestamp_unit"
            ),
        },
        "quality": {
            "peak_correlation": aligned_corr,
            "second_peak_correlation_outside_1s": second_corr,
            "peak_margin": margin,
            "overlap_s": overlap_s,
            "resample_hz": args.resample_hz,
            "smooth_ms": args.smooth_ms,
            "offset_grid_resolution_s": 1.0 / args.resample_hz,
            "ego_activity_p95_minus_p20_rad_s": ego_activity,
            "nokov_activity_p95_minus_p20_rad_s": nokov_activity,
        },
        "inputs": {
            "ego_mcap": str(ego_path),
            "ego_imu_topic": args.imu_topic,
            "ego_imu_rows": int(len(ego_ts)),
            "ego_duration_s": float(ego_grid[-1]),
            "nokov_csv": str(nokov_path),
            "nokov_rigid_body": args.rigid_body,
            "nokov_pose_rows": int(len(nokov_ts)),
            "nokov_duration_s": float(nokov_grid[-1]),
            **nokov_info,
            **quaternion_info,
        },
        "coordinate_note": (
            "angular-speed norm is rotation invariant; raw IMU axis conversion is not "
            "required for this offset estimate"
        ),
        "clock_drift_note": (
            "this first-stage run fixes a=1; estimate a from a longer recording with "
            "distinctive motions near the start, middle and end"
        ),
        "files": {
            "aligned_signals": aligned_csv.name,
            "diagnostic_plot": "imu_nokov_sync.png" if not args.no_plot else None,
        },
        "warnings": warnings,
    }
    json_path = output / "imu_nokov_sync.json"
    json_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    plot_warning = None
    if not args.no_plot:
        plot_warning = save_plot(
            output / "imu_nokov_sync.png", ego_grid, ego_uniform,
            aligned_nokov, valid, offsets, correlations, offset_s,
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if plot_warning:
        print(f"[WARN] {plot_warning}")
    print(f"Sync result: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
