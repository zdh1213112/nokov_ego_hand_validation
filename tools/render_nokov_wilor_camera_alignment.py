#!/usr/bin/env python3
"""Overlay synchronized NOKOV Hand(24) and EGO/WiLoR Hand(21) on a GEN camera.

The tool deliberately does not assume point-to-point correspondence. NOKOV's
physical reflective markers and WiLoR's anatomical joints use different point
definitions; the overlay is a visual check of time, camera projection and
coarse hand alignment rather than a per-joint error metric.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from synchronize_ego_imu_nokov import (
    interpolate_rigid_poses,
    read_nokov_poses,
)


EGO_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
)
NOKOV_EDGES = (
    (0, 1), (1, 3), (3, 2), (2, 0),
    (3, 4), (4, 5), (5, 6), (6, 7),
    (3, 8), (8, 9), (9, 10), (10, 11),
    (3, 12), (12, 13), (13, 14), (14, 15),
    (3, 16), (16, 17), (17, 18), (18, 19),
    (3, 20), (20, 21), (21, 22), (22, 23),
)
EGO_COLORS = {0: (255, 230, 40), 1: (70, 240, 90)}
NOKOV_COLORS = {0: (255, 60, 230), 1: (0, 155, 255)}
SIDE_NAMES = {0: "Left", 1: "Right"}
NOKOV_NAMES = {0: "Body1_Left", 1: "Body1_Right"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True, type=Path)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--fusion", required=True, type=Path)
    parser.add_argument("--hand-eye-json", required=True, type=Path)
    parser.add_argument("--camera", default="camera2")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--max-interpolation-gap-s", type=float, default=0.05)
    parser.add_argument(
        "--spatial-interpretation",
        choices=("auto", "documented", "legacy_direct"),
        default="auto",
        help=(
            "documented treats T_B_E as E->B per its JSON convention; "
            "legacy_direct treats the stored matrix as B->E; auto selects the "
            "lower coarse centroid residual without fitting a new transform"
        ),
    )
    return parser.parse_args()


def load_jsonl(path: Path) -> dict[int, dict[str, Any]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    result = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            row = json.loads(line)
            result[int(row["sync_index"])] = row
    return result


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def valid_coordinate(values: np.ndarray) -> bool:
    return bool(np.isfinite(values).all() and np.max(np.abs(values)) < 1_000_000.0)


def load_marker_tracks(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    rows = read_csv(path)
    timestamps = sorted({int(float(row["device_timestamp_raw"])) for row in rows})
    time_to_index = {value: index for index, value in enumerate(timestamps)}
    names_by_index: dict[int, str] = {}
    points = np.full((len(timestamps), 2, 24, 3), np.nan, dtype=np.float64)
    valid = np.zeros((len(timestamps), 2, 24), dtype=bool)
    name_to_side = {value: key for key, value in NOKOV_NAMES.items()}
    for row in rows:
        side = name_to_side.get(row.get("markerset_name", ""))
        if side is None:
            continue
        try:
            marker = int(row["marker_index"])
            timestamp = int(float(row["device_timestamp_raw"]))
            value = np.asarray(
                [float(row[key]) for key in ("x_mm", "y_mm", "z_mm")],
                dtype=np.float64,
            )
        except (KeyError, TypeError, ValueError):
            continue
        if not 0 <= marker < 24:
            continue
        names_by_index.setdefault(marker, row.get("marker_name", f"marker{marker}"))
        index = time_to_index[timestamp]
        if row.get("valid", "1") == "1" and valid_coordinate(value):
            points[index, side, marker] = value
            valid[index, side, marker] = True
    names = [names_by_index.get(index, f"marker{index}") for index in range(24)]
    return np.asarray(timestamps, dtype=np.int64), points, valid, names


def interpolate_markers(
    times_s: np.ndarray,
    points: np.ndarray,
    point_valid: np.ndarray,
    target_s: np.ndarray,
    max_gap_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    right = np.searchsorted(times_s, target_s, side="right")
    exact_last = target_s == times_s[-1]
    right[exact_last] = len(times_s) - 1
    left = right - 1
    in_range = (left >= 0) & (right < len(times_s))
    safe_left = np.clip(left, 0, len(times_s) - 2)
    safe_right = np.clip(right, 1, len(times_s) - 1)
    gaps = times_s[safe_right] - times_s[safe_left]
    alpha = np.divide(
        target_s - times_s[safe_left], gaps,
        out=np.full(len(target_s), np.nan), where=gaps > 0,
    )
    frame_valid = (
        in_range & (gaps > 0) & (gaps <= max_gap_s)
        & (alpha >= -1e-9) & (alpha <= 1.0 + 1e-9)
    )
    result = np.full((len(target_s), 2, 24, 3), np.nan, dtype=np.float64)
    result_valid = np.zeros((len(target_s), 2, 24), dtype=bool)
    for index in np.flatnonzero(frame_valid):
        both = point_valid[safe_left[index]] & point_valid[safe_right[index]]
        result_valid[index] = both
        result[index][both] = (
            (1.0 - alpha[index]) * points[safe_left[index]][both]
            + alpha[index] * points[safe_right[index]][both]
        )
    return result, result_valid, gaps


def project_double_sphere(
    camera: dict[str, Any], points_camera_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_camera_m, dtype=np.float64)
    x, y, z = points.T
    fx, fy, cx, cy, xi, alpha = np.asarray(camera["distortion"], dtype=np.float64)
    d1 = np.linalg.norm(points, axis=1)
    z1 = alpha * d1 + (1.0 - alpha) * z
    d2 = np.sqrt(x * x + y * y + z1 * z1)
    denominator = xi * d2 + z1
    valid = np.isfinite(points).all(axis=1) & (d1 > 1e-12) & (denominator > 1e-12)
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = fx * x[valid] / denominator[valid] + cx
    pixels[valid, 1] = fy * y[valid] / denominator[valid] + cy
    width, height = camera["image_size"]
    valid &= (
        (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    )
    return pixels, valid


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    return (matrix[:3, :3] @ points.T).T + matrix[:3, 3]


def head_pose_matrix(position_mm: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position_mm * 0.001
    return result


def ego_pixels(result: dict[str, Any] | None, camera_id: str) -> dict[int, np.ndarray]:
    output: dict[int, np.ndarray] = {}
    if result is None:
        return output
    for hand in result.get("hands", []):
        view = hand.get("views", {}).get(camera_id)
        if view is None:
            continue
        values = np.asarray(view.get("joints_2d", []), dtype=np.float64)
        if values.shape == (21, 2):
            output[int(hand["side"])] = values
    return output


def coarse_centroid_cost(
    nokov: dict[int, np.ndarray], ego: dict[int, np.ndarray],
) -> float | None:
    nokov_centers = [np.nanmedian(value, axis=0) for value in nokov.values()]
    ego_centers = [np.nanmedian(value, axis=0) for value in ego.values()]
    if not nokov_centers or not ego_centers:
        return None
    if len(nokov_centers) == 1 or len(ego_centers) == 1:
        return float(min(
            np.linalg.norm(first - second)
            for first in nokov_centers for second in ego_centers
        ))
    return float(min(
        sum(np.linalg.norm(nokov_centers[i] - ego_centers[j]) for i, j in enumerate(order))
        / min(len(nokov_centers), len(ego_centers))
        for order in itertools.permutations(range(len(ego_centers)), len(nokov_centers))
        if len(order) <= len(ego_centers)
    ))


def project_nokov_frame(
    markers_mm: np.ndarray,
    marker_valid: np.ndarray,
    head_position_mm: np.ndarray,
    head_quaternion: np.ndarray,
    camera: dict[str, Any],
    transform_ego_from_body: np.ndarray,
) -> dict[int, np.ndarray]:
    transform_world_from_body = head_pose_matrix(head_position_mm, head_quaternion)
    transform_camera_from_ego = np.linalg.inv(np.asarray(camera["T_base_camera"]))
    transform_camera_from_world = (
        transform_camera_from_ego
        @ transform_ego_from_body
        @ np.linalg.inv(transform_world_from_body)
    )
    output: dict[int, np.ndarray] = {}
    for side in (0, 1):
        pixels = np.full((24, 2), np.nan, dtype=np.float64)
        use = marker_valid[side]
        if np.any(use):
            camera_points = transform_points(
                transform_camera_from_world, markers_mm[side, use] * 0.001
            )
            projected, projection_valid = project_double_sphere(camera, camera_points)
            selected = np.flatnonzero(use)
            pixels[selected[projection_valid]] = projected[projection_valid]
        if np.count_nonzero(np.isfinite(pixels).all(axis=1)):
            output[side] = pixels
    return output


def draw_skeleton(
    image: np.ndarray,
    points: np.ndarray,
    edges: tuple[tuple[int, int], ...],
    color: tuple[int, int, int],
    radius: int,
    thickness: int,
) -> None:
    finite = np.isfinite(points).all(axis=1)
    rounded = np.rint(np.nan_to_num(points)).astype(np.int32)
    for start, end in edges:
        if finite[start] and finite[end]:
            cv2.line(
                image, tuple(rounded[start]), tuple(rounded[end]), color,
                thickness, cv2.LINE_AA,
            )
    for index in np.flatnonzero(finite):
        cv2.circle(image, tuple(rounded[index]), radius, color, -1, cv2.LINE_AA)


def draw_panel(
    image: np.ndarray,
    frame_index: int,
    timestamp_ns: int,
    interpretation: str,
    cost: float | None,
) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (12, 12), (790, 142), (10, 12, 18), -1)
    cv2.addWeighted(overlay, 0.72, image, 0.28, 0.0, image)
    lines = (
        "NOKOV Hand(24) vs EGO/WiLoR Hand(21)",
        f"frame={frame_index}  ego_timestamp_ns={timestamp_ns}",
        f"spatial={interpretation}  coarse centroid gap={cost:.1f}px"
        if cost is not None else f"spatial={interpretation}  coarse centroid gap=N/A",
        "EGO Left=cyan Right=green | NOKOV Left=magenta Right=orange",
    )
    for index, text in enumerate(lines):
        cv2.putText(
            image, text, (28, 42 + index * 29), cv2.FONT_HERSHEY_SIMPLEX,
            0.63, (240, 240, 240), 2, cv2.LINE_AA,
        )


def main() -> int:
    args = parse_args()
    if args.max_frames < 0 or args.max_interpolation_gap_s <= 0:
        raise ValueError("max-frames must be non-negative and interpolation gap positive")
    session = args.session_dir.resolve()
    dataset = args.dataset.resolve()
    fusion = args.fusion.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)

    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    if args.camera not in manifest["camera_ids"]:
        raise ValueError(f"camera {args.camera!r} is not in normalized dataset")
    camera = json.loads(
        (dataset / "calibration" / f"{args.camera}.json").read_text(encoding="utf-8")
    )
    if camera["model"] != "DS":
        raise ValueError("this renderer currently expects the DAS-Ego Double-Sphere model")
    rows = read_csv(dataset / "multiview_frames.csv")
    rows = rows[: args.max_frames or None]
    accepted = load_jsonl(fusion / "accepted.jsonl")
    sync = json.loads(
        (session / "synchronization" / "imu_nokov_sync.json").read_text(encoding="utf-8")
    )
    mapping = sync["time_mapping"]
    if mapping["nokov_origin_field"] != "device_timestamp_raw":
        raise ValueError("visualization requires a device_timestamp_raw synchronization")
    image_timestamps = np.asarray(
        [int(row[f"{args.camera}_timestamp_ns"]) for row in rows], dtype=np.int64
    )
    ego_relative_s = (
        image_timestamps - int(mapping["ego_origin_timestamp_ns"])
    ).astype(np.float64) * 1e-9
    target_nokov_s = (
        float(mapping["a"]) * ego_relative_s + float(mapping["b_s"])
    )
    origin_raw = int(mapping["nokov_origin_timestamp_raw"])
    scale = float(mapping["nokov_seconds_per_timestamp_unit"])

    rigid_path = session / "nokov" / "nokov_rigid_bodies.csv"
    rigid_ts, rigid_frames, rigid_position, rigid_quaternion = read_nokov_poses(
        rigid_path, "head_rigidbody", "device_timestamp_raw"
    )
    rigid_time_s = (rigid_ts - origin_raw).astype(np.float64) * scale
    rigid = interpolate_rigid_poses(
        rigid_time_s, rigid_frames, rigid_position, rigid_quaternion,
        target_nokov_s, args.max_interpolation_gap_s,
    )
    marker_ts, marker_points, marker_valid, marker_names = load_marker_tracks(
        session / "nokov" / "nokov_markers.csv"
    )
    marker_time_s = (marker_ts - origin_raw).astype(np.float64) * scale
    interpolated_markers, interpolated_marker_valid, _marker_gaps = interpolate_markers(
        marker_time_s, marker_points, marker_valid, target_nokov_s,
        args.max_interpolation_gap_s,
    )

    calibration = json.loads(args.hand_eye_json.read_text(encoding="utf-8"))
    matrix = np.asarray(calibration["transforms"]["T_B_E"]["matrix"], dtype=np.float64)
    candidates = {
        "documented": np.linalg.inv(matrix),
        "legacy_direct": matrix,
    }
    candidate_costs: dict[str, list[float]] = {key: [] for key in candidates}
    for index, row in enumerate(rows):
        if not rigid["valid"][index]:
            continue
        ego = ego_pixels(accepted.get(int(row["sync_index"])), args.camera)
        if not ego:
            continue
        for name, transform in candidates.items():
            nokov = project_nokov_frame(
                interpolated_markers[index], interpolated_marker_valid[index],
                rigid["position_mm"][index], rigid["quaternion_xyzw"][index],
                camera, transform,
            )
            cost = coarse_centroid_cost(nokov, ego)
            if cost is not None and math.isfinite(cost):
                candidate_costs[name].append(cost)
    candidate_medians = {
        name: float(np.median(values)) if values else float("inf")
        for name, values in candidate_costs.items()
    }
    interpretation = args.spatial_interpretation
    if interpretation == "auto":
        interpretation = min(candidate_medians, key=candidate_medians.get)
    transform_ego_from_body = candidates[interpretation]

    deltas = np.diff(image_timestamps).astype(np.float64) * 1e-9
    fps = float(1.0 / np.median(deltas[deltas > 0]))
    width, height = camera["image_size"]
    video_path = output / f"{args.camera}_nokov24_wilor21_alignment.mp4"
    writer = cv2.VideoWriter(
        str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot create {video_path}")
    capture = cv2.VideoCapture(
        str(dataset / "cameras" / args.camera / manifest["storage"]["video_filename"])
    )
    if not capture.isOpened():
        raise RuntimeError("cannot open normalized camera video")
    next_video_index = 0
    rendered = 0
    ego_frames = 0
    nokov_frames_count = 0
    both_frames = 0
    frame_costs: list[float] = []
    report_rows: list[dict[str, Any]] = []
    previews: list[np.ndarray] = []
    preview_targets = set(
        np.linspace(0, max(0, len(rows) - 1), min(4, len(rows)), dtype=int).tolist()
    )
    try:
        for index, row in enumerate(rows):
            target_video_index = int(row[f"{args.camera}_frame_index"])
            frame = None
            while next_video_index <= target_video_index:
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError(f"video ended before frame {target_video_index}")
                next_video_index += 1
            assert frame is not None
            ego = ego_pixels(accepted.get(int(row["sync_index"])), args.camera)
            nokov: dict[int, np.ndarray] = {}
            if rigid["valid"][index]:
                nokov = project_nokov_frame(
                    interpolated_markers[index], interpolated_marker_valid[index],
                    rigid["position_mm"][index], rigid["quaternion_xyzw"][index],
                    camera, transform_ego_from_body,
                )
            for side, points in ego.items():
                draw_skeleton(frame, points, EGO_EDGES, EGO_COLORS[side], 6, 4)
            for side, points in nokov.items():
                draw_skeleton(frame, points, NOKOV_EDGES, NOKOV_COLORS[side], 5, 2)
            cost = coarse_centroid_cost(nokov, ego)
            draw_panel(frame, int(row["sync_index"]), int(image_timestamps[index]), interpretation, cost)
            if ego:
                ego_frames += 1
            if nokov:
                nokov_frames_count += 1
            if ego and nokov:
                both_frames += 1
            if cost is not None and math.isfinite(cost):
                frame_costs.append(cost)
            report_rows.append({
                "sync_index": int(row["sync_index"]),
                "ego_timestamp_ns": int(image_timestamps[index]),
                "nokov_target_timestamp_raw": (
                    origin_raw + target_nokov_s[index] / scale
                ),
                "ego_hand_count": len(ego),
                "nokov_hand_count": len(nokov),
                "nokov_visible_marker_count": int(
                    np.count_nonzero(interpolated_marker_valid[index])
                ),
                "coarse_centroid_gap_px": cost,
            })
            writer.write(frame)
            if index in preview_targets:
                previews.append(frame.copy())
            rendered += 1
    finally:
        capture.release()
        writer.release()

    csv_path = output / "projection_frames.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        fields = list(report_rows[0])
        csv_writer = csv.DictWriter(stream, fieldnames=fields)
        csv_writer.writeheader()
        csv_writer.writerows(report_rows)
    preview_path = output / f"{args.camera}_alignment_preview.jpg"
    if previews:
        target_width = 800
        resized = [
            cv2.resize(value, (target_width, round(value.shape[0] * target_width / value.shape[1])))
            for value in previews
        ]
        cv2.imwrite(str(preview_path), np.vstack(resized))

    summary = {
        "schema": "nokov24_wilor21_camera_alignment_visualization_v1",
        "status": "ok" if both_frames else "needs_review",
        "camera": args.camera,
        "point_semantics": {
            "nokov": "24 physical reflective markers; not anatomical WiLoR joints",
            "ego": "21 anatomical WiLoR joints from multiview fusion",
            "metric_note": (
                "coarse centroid gap is a visual alignment diagnostic, not a per-joint "
                "ground-truth error because the point definitions differ"
            ),
        },
        "time_mapping": mapping,
        "spatial": {
            "hand_eye_json": str(args.hand_eye_json.resolve()),
            "requested_interpretation": args.spatial_interpretation,
            "selected_interpretation": interpretation,
            "candidate_coarse_centroid_median_px": candidate_medians,
            "selection_note": (
                "auto only selects between the stored matrix and its documented inverse; "
                "it does not fit a transform to hand observations"
            ),
            "calibration_status": calibration.get("status"),
            "calibration_warning": calibration.get("warning"),
        },
        "coverage": {
            "rendered_frames": rendered,
            "ego_frames": ego_frames,
            "nokov_frames": nokov_frames_count,
            "both_frames": both_frames,
            "both_ratio": both_frames / rendered if rendered else 0.0,
        },
        "quality": {
            "coarse_centroid_gap_px_median": (
                float(np.median(frame_costs)) if frame_costs else None
            ),
            "coarse_centroid_gap_px_p95": (
                float(np.percentile(frame_costs, 95)) if frame_costs else None
            ),
            "max_interpolation_gap_s": args.max_interpolation_gap_s,
            "marker_names": marker_names,
        },
        "wilor_fusion": json.loads((fusion / "summary.json").read_text(encoding="utf-8")),
        "files": {
            "video": video_path.name,
            "preview": preview_path.name,
            "per_frame_csv": csv_path.name,
        },
    }
    summary_path = output / "summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"Video: {video_path}")
    print(f"Preview: {preview_path}")
    return 0 if summary["status"] == "ok" else 2


if __name__ == "__main__":
    raise SystemExit(main())
