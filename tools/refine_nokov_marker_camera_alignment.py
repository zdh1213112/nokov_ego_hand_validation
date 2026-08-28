#!/usr/bin/env python3
"""Refine a NOKOV hand-camera overlay from the visible glove marker balls.

The coordinate-chain diagnostic deliberately fits an approximate transform from
WiLoR hand centres.  This tool takes the next, more specific step for a camera
whose reflective balls are visible in the RGB image:

* use the existing fitted chain as an initial pose;
* associate bright, compact blobs near the predicted Hand(24) points;
* optimise a small camera-local SE(3) correction against those image points;
* render the physical marker balls and NOKOV skeleton-segment points as
  separate layers.

This is an image-assisted diagnostic refinement.  It must not overwrite the
formal hand-eye calibration without an independent calibration sequence.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment, least_squares
from scipy.spatial.transform import Rotation

from diagnose_nokov_coordinate_chain import inverse
from render_nokov_wilor_camera_alignment import (
    EGO_COLORS,
    EGO_EDGES,
    NOKOV_COLORS,
    ego_pixels,
    head_pose_matrix,
    interpolate_markers,
    load_jsonl,
    load_marker_tracks,
    project_double_sphere,
    read_csv,
    transform_points,
    draw_skeleton,
)
from synchronize_ego_imu_nokov import interpolate_rigid_poses, read_nokov_poses


SKELETON_SUFFIXES = (
    "Hand",
    "HandThumb0", "HandThumb1", "HandThumb2", "HandThumb3", "HandThumbEnd",
    "HandIndex0", "HandIndex1", "HandIndex2", "HandIndex3", "HandIndexEnd",
    "HandMiddle0", "HandMiddle1", "HandMiddle2", "HandMiddle3", "HandMiddleEnd",
    "HandRing0", "HandRing1", "HandRing2", "HandRing3", "HandRingEnd",
    "HandPinky0", "HandPinky1", "HandPinky2", "HandPinky3", "HandPinkyEnd",
)
SKELETON_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5),
    (0, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (0, 11), (11, 12), (12, 13), (13, 14), (14, 15),
    (0, 16), (16, 17), (17, 18), (18, 19), (19, 20),
    (0, 21), (21, 22), (22, 23), (23, 24), (24, 25),
)
SKELETON_COLORS = {0: (180, 100, 255), 1: (0, 220, 220)}
SIDE_NAMES = {0: "Left", 1: "Right"}
MARKER_COUNTS = 24


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", required=True, type=Path)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--fusion", required=True, type=Path)
    parser.add_argument("--diagnostic-summary", required=True, type=Path)
    parser.add_argument("--camera", default="camera2")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--calibration-frame-index",
        type=int,
        default=-1,
        help="camera row/frame used for image-assisted refinement; defaults to the middle row",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--max-interpolation-gap-s", type=float, default=0.05)
    parser.add_argument("--blob-threshold", type=int, default=160)
    parser.add_argument("--preview-count", type=int, default=8)
    parser.add_argument("--render-videos", action="store_true")
    return parser.parse_args()


def load_skeleton_tracks(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    """Load the 26 segment-origin points for each NOKOV Hand skeleton."""
    rows = read_csv(path)
    timestamps = sorted({int(float(row["device_timestamp_raw"])) for row in rows})
    time_to_index = {value: index for index, value in enumerate(timestamps)}
    points = np.full((len(timestamps), 2, len(SKELETON_SUFFIXES), 3), np.nan)
    valid = np.zeros((len(timestamps), 2, len(SKELETON_SUFFIXES)), dtype=bool)
    names = [f"segment_{suffix}" for suffix in SKELETON_SUFFIXES]
    for row in rows:
        skeleton_name = row.get("skeleton_name", "")
        if skeleton_name == "Body1_Left":
            side = 0
            prefix = "Left"
        elif skeleton_name == "Body1_Right":
            side = 1
            prefix = "Right"
        else:
            continue
        segment_name = row.get("segment_name", "")
        expected_prefix = prefix
        if not segment_name.startswith(expected_prefix):
            continue
        suffix = segment_name[len(expected_prefix):]
        try:
            segment = SKELETON_SUFFIXES.index(suffix)
            timestamp = int(float(row["device_timestamp_raw"]))
            value = np.asarray(
                [float(row[key]) for key in ("x_mm", "y_mm", "z_mm")],
                dtype=np.float64,
            )
        except (KeyError, TypeError, ValueError):
            continue
        if (
            row.get("valid_numeric", "1") == "1"
            and np.isfinite(value).all()
            and np.max(np.abs(value)) < 1_000_000.0
        ):
            index = time_to_index[timestamp]
            points[index, side, segment] = value
            valid[index, side, segment] = True
    return np.asarray(timestamps, dtype=np.int64), points, valid, names


def interpolate_skeleton(
    times_s: np.ndarray,
    points: np.ndarray,
    point_valid: np.ndarray,
    target_s: np.ndarray,
    max_gap_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Linearly interpolate segment positions while preserving visibility."""
    if len(times_s) < 2:
        raise RuntimeError("skeleton track needs at least two timestamps")
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
    result = np.full(
        (len(target_s),) + points.shape[1:], np.nan, dtype=np.float64
    )
    result_valid = np.zeros(
        (len(target_s),) + point_valid.shape[1:], dtype=bool
    )
    for index in np.flatnonzero(frame_valid):
        both = point_valid[safe_left[index]] & point_valid[safe_right[index]]
        result_valid[index] = both
        result[index][both] = (
            (1.0 - alpha[index]) * points[safe_left[index]][both]
            + alpha[index] * points[safe_right[index]][both]
        )
    return result, result_valid, gaps


def camera_from_world(
    position_mm: np.ndarray,
    quaternion_xyzw: np.ndarray,
    camera: dict[str, Any],
    body_to_genbase: np.ndarray,
) -> np.ndarray:
    world_to_body = inverse(head_pose_matrix(position_mm, quaternion_xyzw))
    camera_from_genbase = inverse(np.asarray(camera["T_base_camera"], dtype=np.float64))
    return camera_from_genbase @ body_to_genbase @ world_to_body


def project_track(
    points_mm: np.ndarray,
    point_valid: np.ndarray,
    camera_from_world_matrix: np.ndarray,
    camera_correction: np.ndarray,
    camera: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project a two-hand NOKOV track after an optional camera-local correction."""
    point_count = points_mm.shape[1]
    camera_points = np.full((2, point_count, 3), np.nan, dtype=np.float64)
    pixels = np.full((2, point_count, 2), np.nan, dtype=np.float64)
    projected_valid = np.zeros((2, point_count), dtype=bool)
    for side in (0, 1):
        use = point_valid[side]
        if not np.any(use):
            continue
        values = transform_points(
            camera_from_world_matrix, points_mm[side, use] * 0.001
        )
        values = transform_points(camera_correction, values)
        projected, valid = project_double_sphere(camera, values)
        selected = np.flatnonzero(use)
        camera_points[side, selected] = values
        pixels[side, selected[valid]] = projected[valid]
        projected_valid[side, selected[valid]] = True
    return pixels, camera_points, projected_valid


def bright_blob_centers(
    image: np.ndarray,
    threshold: int,
    predicted: np.ndarray,
) -> np.ndarray:
    """Find compact low-saturation bright blobs near the predicted hands."""
    finite = np.isfinite(predicted).all(axis=1)
    if not np.any(finite):
        return np.empty((0, 2), dtype=np.float64)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:, :, 1] < 100) & (hsv[:, :, 2] > threshold)).astype(np.uint8)
    count, _labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    x_min = max(0, int(np.floor(np.min(predicted[finite, 0]) - 45)))
    x_max = min(image.shape[1], int(np.ceil(np.max(predicted[finite, 0]) + 45)))
    y_min = max(0, int(np.floor(np.min(predicted[finite, 1]) - 45)))
    y_max = min(image.shape[0], int(np.ceil(np.max(predicted[finite, 1]) + 45)))
    output: list[np.ndarray] = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        aspect = max(width, height) / max(1, min(width, height))
        if not (3 <= area <= 150 and 2 <= width <= 25 and 2 <= height <= 25):
            continue
        if aspect > 3.0 or not (x_min <= x < x_max and y_min <= y < y_max):
            continue
        output.append(centers[index])
    return np.asarray(output, dtype=np.float64)


def associate_marker_blobs(
    predicted: np.ndarray,
    components: np.ndarray,
    minimum_matches: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Associate blobs after estimating a robust initial 2-D translation."""
    if len(components) == 0:
        raise RuntimeError("no bright glove-marker blobs found")
    distances = np.linalg.norm(predicted[:, None, :] - components[None, :, :], axis=2)
    nearest = np.argmin(distances, axis=1)
    nearest_distance = distances[np.arange(len(predicted)), nearest]
    initial_use = nearest_distance < 35.0
    if int(np.count_nonzero(initial_use)) < minimum_matches:
        raise RuntimeError(
            f"only {int(np.count_nonzero(initial_use))} initial marker blobs; "
            f"need {minimum_matches}"
        )
    shift = np.median(components[nearest[initial_use]] - predicted[initial_use], axis=0)
    shifted = predicted + shift
    shifted_distance = np.linalg.norm(
        shifted[:, None, :] - components[None, :, :], axis=2
    )
    match_limit = 13.0
    assignment_cost = np.where(shifted_distance <= match_limit, shifted_distance, 1e6)
    dummy = np.full((len(predicted), len(predicted)), match_limit, dtype=np.float64)
    rows, columns = linear_sum_assignment(np.hstack((assignment_cost, dummy)))
    pairs = [
        (row, column)
        for row, column in zip(rows, columns)
        if column < len(components) and shifted_distance[row, column] <= match_limit
    ]
    if len(pairs) < minimum_matches:
        raise RuntimeError(
            f"only {len(pairs)} marker blobs after one-to-one matching; "
            f"need {minimum_matches}"
        )
    source_indices = np.asarray([pair[0] for pair in pairs], dtype=int)
    observations = np.asarray([components[pair[1]] for pair in pairs], dtype=np.float64)
    return source_indices, observations, shift


def correction_matrix(parameters: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_rotvec(parameters[:3]).as_matrix()
    result[:3, 3] = parameters[3:6]
    return result


def fit_camera_correction(
    camera: dict[str, Any],
    source_camera_points: np.ndarray,
    observations: np.ndarray,
    initial_match_distances: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Fit and robustly trim a camera-local correction in metres/pixels."""
    def errors(parameters: np.ndarray, indices: np.ndarray) -> np.ndarray:
        transformed = transform_points(correction_matrix(parameters), source_camera_points[indices])
        pixels, valid = project_double_sphere(camera, transformed)
        residual = pixels - observations[indices]
        residual[~valid] = 1000.0
        return residual.reshape(-1)

    def full_errors(parameters: np.ndarray) -> np.ndarray:
        transformed = transform_points(correction_matrix(parameters), source_camera_points)
        pixels, valid = project_double_sphere(camera, transformed)
        residual = np.linalg.norm(pixels - observations, axis=1)
        residual[~valid] = 1000.0
        return residual

    inliers = np.arange(len(source_camera_points), dtype=int)
    parameters = np.zeros(6, dtype=np.float64)
    for iteration in range(4):
        result = least_squares(
            lambda value: errors(value, inliers),
            parameters,
            loss="soft_l1" if iteration == 0 else "linear",
            f_scale=2.0,
            max_nfev=600,
        )
        parameters = result.x
        residuals = full_errors(parameters)
        median = float(np.median(residuals[inliers]))
        limit = max(3.0, 2.5 * median)
        new_inliers = np.flatnonzero(residuals <= limit)
        if len(new_inliers) < 8 or np.array_equal(new_inliers, inliers):
            break
        inliers = new_inliers

    # One final linear fit on the stable inlier set gives the reported pose.
    result = least_squares(
        lambda value: errors(value, inliers),
        parameters,
        loss="linear",
        max_nfev=600,
    )
    parameters = result.x
    residuals = full_errors(parameters)
    inlier_residuals = residuals[inliers]
    matrix = correction_matrix(parameters)
    quality = {
        "matched_marker_count": int(len(source_camera_points)),
        "inlier_marker_count": int(len(inliers)),
        "seed_match_median_px": float(np.median(initial_match_distances)),
        "all_residual_median_px": float(np.median(residuals)),
        "all_residual_p95_px": float(np.percentile(residuals, 95)),
        "inlier_residual_median_px": float(np.median(inlier_residuals)),
        "inlier_residual_p95_px": float(np.percentile(inlier_residuals, 95)),
        "rotation_correction_deg": float(np.linalg.norm(parameters[:3]) * 180.0 / math.pi),
        "translation_correction_mm": (parameters[3:6] * 1000.0).tolist(),
        "parameters": parameters.tolist(),
        "correction_camera_from_initial": matrix.tolist(),
    }
    return matrix, quality


def load_video_frame(capture: cv2.VideoCapture, target_index: int) -> np.ndarray:
    frame = None
    for _ in range(target_index + 1):
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"video ended before frame {target_index}")
    assert frame is not None
    return frame


def draw_marker_points(
    image: np.ndarray,
    points: np.ndarray,
    valid: np.ndarray,
    show_labels: bool = False,
) -> None:
    for side in (0, 1):
        color = NOKOV_COLORS[side]
        for marker in np.flatnonzero(valid[side]):
            x, y = np.rint(points[side, marker]).astype(int)
            cv2.circle(image, (int(x), int(y)), 8, (8, 8, 8), 3, cv2.LINE_AA)
            cv2.circle(image, (int(x), int(y)), 7, color, 2, cv2.LINE_AA)
            cv2.drawMarker(
                image, (int(x), int(y)), color, cv2.MARKER_CROSS, 13, 2, cv2.LINE_AA
            )
            if show_labels:
                text = f"{SIDE_NAMES[side][0]}{marker:02d}"
                cv2.putText(
                    image, text, (int(x) + 8, int(y) - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (8, 8, 8), 3, cv2.LINE_AA,
                )
                cv2.putText(
                    image, text, (int(x) + 8, int(y) - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA,
                )


def draw_skeleton_segments(
    image: np.ndarray,
    points: np.ndarray,
    valid: np.ndarray,
) -> None:
    for side in (0, 1):
        draw_skeleton(
            image, points[side], SKELETON_EDGES,
            SKELETON_COLORS[side], 4, 2,
        )
        finite = valid[side] & np.isfinite(points[side]).all(axis=1)
        for index in np.flatnonzero(finite):
            x, y = np.rint(points[side, index]).astype(int)
            cv2.rectangle(image, (int(x) - 3, int(y) - 3), (int(x) + 3, int(y) + 3),
                          SKELETON_COLORS[side], 1, cv2.LINE_AA)


def draw_title(image: np.ndarray, title: str, gap_text: str = "") -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], 66), (10, 12, 18), -1)
    cv2.addWeighted(overlay, 0.78, image, 0.22, 0.0, image)
    cv2.putText(image, title, (14, 27), cv2.FONT_HERSHEY_SIMPLEX,
                0.57, (245, 245, 245), 2, cv2.LINE_AA)
    cv2.putText(image, gap_text, (14, 52), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (225, 225, 225), 1, cv2.LINE_AA)


def render_frame(
    base: np.ndarray,
    ego: dict[int, np.ndarray],
    marker_pixels: np.ndarray,
    marker_valid: np.ndarray,
    skeleton_pixels: np.ndarray,
    skeleton_valid: np.ndarray,
    title: str,
    show_labels: bool = False,
) -> np.ndarray:
    image = base.copy()
    for side, points in ego.items():
        draw_skeleton(image, points, EGO_EDGES, EGO_COLORS[side], 6, 4)
    draw_skeleton_segments(image, skeleton_pixels, skeleton_valid)
    draw_marker_points(image, marker_pixels, marker_valid, show_labels)
    draw_title(
        image,
        title,
        "EGO/WiLoR=thick cyan/green | NOKOV skeleton=thin purple/yellow squares | "
        "Hand(24) balls=magenta/orange rings",
    )
    return image


def flatten_projected(
    pixels: np.ndarray,
    camera_points: np.ndarray,
    valid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    use = valid & np.isfinite(pixels).all(axis=2) & np.isfinite(camera_points).all(axis=2)
    return camera_points[use], pixels[use]


def marker_skeleton_correspondence(
    marker_points_mm: np.ndarray,
    marker_valid: np.ndarray,
    skeleton_points_mm: np.ndarray,
    skeleton_valid: np.ndarray,
) -> dict[str, Any]:
    """Compare the 20 finger marker balls with matching skeleton segment points."""
    groups = (
        (4, 2),    # FingerThumb1..4 -> Thumb1..End
        (8, 7),    # FingerIndex1..4 -> Index1..End
        (12, 12),  # FingerMiddle1..4 -> Middle1..End
        (16, 17),  # FingerRing1..4 -> Ring1..End
        (20, 22),  # FingerPinky1..4 -> Pinky1..End
    )
    side_values: dict[str, list[float]] = {"left": [], "right": []}
    for side, side_name in ((0, "left"), (1, "right")):
        for marker_start, skeleton_start in groups:
            marker = marker_points_mm[side, marker_start:marker_start + 4]
            skeleton = skeleton_points_mm[side, skeleton_start:skeleton_start + 4]
            use = (
                marker_valid[side, marker_start:marker_start + 4]
                & skeleton_valid[side, skeleton_start:skeleton_start + 4]
            )
            if np.any(use):
                side_values[side_name].extend(
                    np.linalg.norm(marker[use] - skeleton[use], axis=1).tolist()
                )
    all_values = side_values["left"] + side_values["right"]
    return {
        side: {
            "count": len(values),
            "median_mm": float(np.median(values)) if values else None,
            "p95_mm": float(np.percentile(values, 95)) if values else None,
            "max_mm": float(np.max(values)) if values else None,
        }
        for side, values in (
            *side_values.items(),
            ("all", all_values),
        )
    }


def main() -> int:
    args = parse_args()
    if args.max_frames < 0 or args.max_interpolation_gap_s <= 0:
        raise ValueError("max-frames must be non-negative and interpolation gap positive")
    if args.blob_threshold < 0 or args.blob_threshold > 255:
        raise ValueError("blob threshold must be in [0, 255]")
    session = args.session_dir.resolve()
    dataset = args.dataset.resolve()
    fusion = args.fusion.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    selected_dir = output / "selected_frames"
    selected_dir.mkdir(parents=True, exist_ok=True)

    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    if args.camera not in manifest["camera_ids"]:
        raise ValueError(f"camera {args.camera!r} is not in normalized dataset")
    camera = json.loads(
        (dataset / "calibration" / f"{args.camera}.json").read_text(encoding="utf-8")
    )
    if camera["model"] != "DS":
        raise ValueError("marker-ball refinement currently expects the Double-Sphere camera")
    rows = read_csv(dataset / "multiview_frames.csv")
    rows = rows[: args.max_frames or None]
    accepted = load_jsonl(fusion / "accepted.jsonl")

    sync = json.loads(
        (session / "synchronization" / "imu_nokov_sync.json").read_text(encoding="utf-8")
    )["time_mapping"]
    image_timestamps = np.asarray(
        [int(row[f"{args.camera}_timestamp_ns"]) for row in rows], dtype=np.int64
    )
    ego_relative_s = (
        image_timestamps - int(sync["ego_origin_timestamp_ns"])
    ).astype(np.float64) * 1e-9
    target_nokov_s = float(sync["a"]) * ego_relative_s + float(sync["b_s"])
    origin_raw = int(sync["nokov_origin_timestamp_raw"])
    scale = float(sync["nokov_seconds_per_timestamp_unit"])

    rigid_ts, rigid_frames, rigid_position, rigid_quaternion = read_nokov_poses(
        session / "nokov" / "nokov_rigid_bodies.csv",
        "head_rigidbody", "device_timestamp_raw",
    )
    rigid = interpolate_rigid_poses(
        (rigid_ts - origin_raw).astype(np.float64) * scale,
        rigid_frames, rigid_position, rigid_quaternion,
        target_nokov_s, args.max_interpolation_gap_s,
    )
    marker_ts, marker_points, marker_valid, marker_names = load_marker_tracks(
        session / "nokov" / "nokov_markers.csv"
    )
    marker_points, marker_valid, _marker_gaps = interpolate_markers(
        (marker_ts - origin_raw).astype(np.float64) * scale,
        marker_points, marker_valid, target_nokov_s,
        args.max_interpolation_gap_s,
    )
    skeleton_ts, skeleton_points, skeleton_valid, skeleton_names = load_skeleton_tracks(
        session / "nokov" / "nokov_skeleton_segments.csv"
    )
    skeleton_points, skeleton_valid, _skeleton_gaps = interpolate_skeleton(
        (skeleton_ts - origin_raw).astype(np.float64) * scale,
        skeleton_points, skeleton_valid, target_nokov_s,
        args.max_interpolation_gap_s,
    )
    diagnostic = json.loads(args.diagnostic_summary.read_text(encoding="utf-8"))
    fitted_ego_to_gen = np.asarray(diagnostic["hand_eye"]["fitted_E_to_GEN"], dtype=np.float64)
    calibration = json.loads(
        (session / "calibration" / "T_nokov_ego_vio_provisional.json").read_text(
            encoding="utf-8"
        )
    )
    body_to_ego = inverse(
        np.asarray(calibration["transforms"]["T_B_E"]["matrix"], dtype=np.float64)
    )
    body_to_genbase = fitted_ego_to_gen @ body_to_ego

    calibration_index = args.calibration_frame_index
    if calibration_index < 0:
        calibration_index = len(rows) // 2
    if not 0 <= calibration_index < len(rows):
        raise ValueError(f"calibration frame index {calibration_index} is outside {len(rows)} rows")
    correspondence = marker_skeleton_correspondence(
        marker_points[calibration_index],
        marker_valid[calibration_index],
        skeleton_points[calibration_index],
        skeleton_valid[calibration_index],
    )
    video_path = dataset / "cameras" / args.camera / manifest["storage"]["video_filename"]
    calibration_capture = cv2.VideoCapture(str(video_path))
    if not calibration_capture.isOpened():
        raise RuntimeError(f"cannot open {video_path}")
    calibration_frame = load_video_frame(
        calibration_capture,
        int(rows[calibration_index][f"{args.camera}_frame_index"]),
    )
    calibration_capture.release()
    if not rigid["valid"][calibration_index]:
        raise RuntimeError("calibration frame has no valid interpolated head rigid body")

    initial_camera_from_world = camera_from_world(
        rigid["position_mm"][calibration_index],
        rigid["quaternion_xyzw"][calibration_index],
        camera, body_to_genbase,
    )
    identity = np.eye(4, dtype=np.float64)
    initial_marker_px, initial_marker_cam, initial_marker_projection_valid = project_track(
        marker_points[calibration_index], marker_valid[calibration_index],
        initial_camera_from_world, identity, camera,
    )
    source_camera_points, predicted_pixels = flatten_projected(
        initial_marker_px, initial_marker_cam, initial_marker_projection_valid,
    )
    components = bright_blob_centers(
        calibration_frame, args.blob_threshold, predicted_pixels
    )
    source_indices, observations, seed_shift = associate_marker_blobs(
        predicted_pixels, components, minimum_matches=12
    )
    matched_source = source_camera_points[source_indices]
    matched_predicted = predicted_pixels[source_indices]
    matched_distances = np.linalg.norm(observations - (matched_predicted + seed_shift), axis=1)
    correction, correction_quality = fit_camera_correction(
        camera, matched_source, observations, matched_distances,
    )

    width, height = camera["image_size"]
    fps_deltas = np.diff(image_timestamps).astype(np.float64) * 1e-9
    fps = float(1.0 / np.median(fps_deltas[fps_deltas > 0]))
    writers: dict[str, cv2.VideoWriter] = {}
    video_paths: dict[str, Path] = {}
    if args.render_videos:
        video_paths = {
            "refined": output / f"{args.camera}_marker_skeleton_refined_alignment.mp4",
            "before_after": output / f"{args.camera}_before_after_marker_refinement.mp4",
        }
        writers = {
            "refined": cv2.VideoWriter(
                str(video_paths["refined"]), cv2.VideoWriter_fourcc(*"mp4v"),
                fps, (width, height),
            ),
            "before_after": cv2.VideoWriter(
                str(video_paths["before_after"]), cv2.VideoWriter_fourcc(*"mp4v"),
                fps, (width * 2, height),
            ),
        }
        failed = [name for name, writer in writers.items() if not writer.isOpened()]
        if failed:
            for writer in writers.values():
                writer.release()
            raise RuntimeError(f"cannot create videos: {failed}")

    preview_count = min(max(args.preview_count, 0), len(rows))
    preview_indices = set(
        np.linspace(0, max(0, len(rows) - 1), preview_count, dtype=int).tolist()
        if preview_count else []
    )
    preview_indices.add(calibration_index)
    preview_indices = {index for index in preview_indices if index < len(rows)}
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video_path}")
    next_video_index = 0
    selected_records: list[dict[str, Any]] = []
    validation_distances: list[float] = []
    validation_frame_records: list[dict[str, Any]] = []
    validation_stride = max(1, len(rows) // 30)
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
            if rigid["valid"][index]:
                current_camera_from_world = camera_from_world(
                    rigid["position_mm"][index],
                    rigid["quaternion_xyzw"][index],
                    camera, body_to_genbase,
                )
                old_marker_px, old_marker_cam, old_marker_ok = project_track(
                    marker_points[index], marker_valid[index],
                    current_camera_from_world, identity, camera,
                )
                new_marker_px, _new_marker_cam, new_marker_ok = project_track(
                    marker_points[index], marker_valid[index],
                    current_camera_from_world, correction, camera,
                )
                old_skeleton_px, _old_skeleton_cam, old_skeleton_ok = project_track(
                    skeleton_points[index], skeleton_valid[index],
                    current_camera_from_world, identity, camera,
                )
                new_skeleton_px, _new_skeleton_cam, new_skeleton_ok = project_track(
                    skeleton_points[index], skeleton_valid[index],
                    current_camera_from_world, correction, camera,
                )
            else:
                old_marker_px = new_marker_px = np.full((2, MARKER_COUNTS, 2), np.nan)
                old_marker_ok = new_marker_ok = np.zeros((2, MARKER_COUNTS), dtype=bool)
                old_skeleton_px = new_skeleton_px = np.full((2, 26, 2), np.nan)
                old_skeleton_ok = new_skeleton_ok = np.zeros((2, 26), dtype=bool)

            old_image = render_frame(
                frame, ego, old_marker_px, old_marker_ok,
                old_skeleton_px, old_skeleton_ok,
                "INITIAL: WiLoR-centre fitted chain (marker balls may be offset)",
            )
            new_image = render_frame(
                frame, ego, new_marker_px, new_marker_ok,
                new_skeleton_px, new_skeleton_ok,
                "REFINED: image-assisted glove-ball camera correction",
            )
            if args.render_videos:
                writers["refined"].write(new_image)
                writers["before_after"].write(np.hstack((old_image, new_image)))
            if index in preview_indices:
                stem = f"sync{int(row['sync_index']):06d}"
                cv2.imwrite(str(selected_dir / f"{stem}_initial.jpg"), old_image)
                cv2.imwrite(str(selected_dir / f"{stem}_refined.jpg"), new_image)
                selected_records.append({
                    "sync_index": int(row["sync_index"]),
                    "source_frame_index": target_video_index,
                    "ego_timestamp_ns": int(image_timestamps[index]),
                    "marker_valid_count": int(np.count_nonzero(new_marker_ok)),
                    "skeleton_valid_count": int(np.count_nonzero(new_skeleton_ok)),
                    "initial_image": f"selected_frames/{stem}_initial.jpg",
                    "refined_image": f"selected_frames/{stem}_refined.jpg",
                })
            if index % validation_stride == 0 and rigid["valid"][index]:
                _validation_pred, _validation_camera, validation_ok = project_track(
                    marker_points[index], marker_valid[index],
                    current_camera_from_world, correction, camera,
                )
                _flat_cam, flat_pred = flatten_projected(
                    _validation_pred, _validation_camera, validation_ok,
                )
                blobs = bright_blob_centers(frame, args.blob_threshold, flat_pred)
                if len(blobs):
                    distances = np.linalg.norm(flat_pred[:, None] - blobs[None, :, :], axis=2).min(axis=1)
                    good = distances < 8.0
                    if np.any(good):
                        validation_distances.extend(distances[good].tolist())
                    validation_frame_records.append({
                        "row_index": index,
                        "candidate_marker_count": int(len(flat_pred)),
                        "blob_count": int(len(blobs)),
                        "matched_within_8px": int(np.count_nonzero(good)),
                        "median_nearest_px": float(np.median(distances)),
                        "median_matched_nearest_px": (
                            float(np.median(distances[good])) if np.any(good) else None
                        ),
                    })
    finally:
        capture.release()
        for writer in writers.values():
            writer.release()

    refined_contact_rows: list[np.ndarray] = []
    for record in selected_records:
        image = cv2.imread(str(output / record["refined_image"]))
        if image is not None:
            refined_contact_rows.append(cv2.resize(image, (800, 650), interpolation=cv2.INTER_AREA))
    contact_path = output / "marker_ball_refinement_contact_sheet.jpg"
    if refined_contact_rows:
        cv2.imwrite(str(contact_path), np.vstack(refined_contact_rows))

    calibration_debug = calibration_frame.copy()
    calibration_new_marker_px, _calibration_new_marker_cam, calibration_new_marker_ok = project_track(
        marker_points[calibration_index], marker_valid[calibration_index],
        initial_camera_from_world, correction, camera,
    )
    calibration_new_skeleton_px, _calibration_new_skeleton_cam, calibration_new_skeleton_ok = project_track(
        skeleton_points[calibration_index], skeleton_valid[calibration_index],
        initial_camera_from_world, correction, camera,
    )
    calibration_debug = render_frame(
        calibration_debug,
        ego_pixels(accepted.get(int(rows[calibration_index]["sync_index"])), args.camera),
        calibration_new_marker_px, calibration_new_marker_ok,
        calibration_new_skeleton_px, calibration_new_skeleton_ok,
        "CALIBRATION FRAME: projected balls vs visible glove balls",
        show_labels=True,
    )
    for point in observations:
        cv2.circle(calibration_debug, tuple(np.rint(point).astype(int)), 5,
                   (255, 255, 255), 1, cv2.LINE_AA)
    calibration_debug_path = output / "calibration_marker_ball_observations.jpg"
    cv2.imwrite(str(calibration_debug_path), calibration_debug)

    refinement = {
        "schema": "nokov_marker_ball_camera_refinement_v1",
        "status": "diagnostic_image_assisted",
        "camera": args.camera,
        "point_semantics": {
            "marker_balls": "NOKOV Hand(24) physical reflective marker centres",
            "nokov_skeleton": "NOKOV skeleton_segments.csv segment origins/end points",
            "ego": "WiLoR 21 anatomical joints from the fusion output",
        },
        "initial_chain": {
            "source": str(args.diagnostic_summary.resolve()),
            "body_to_genbase": body_to_genbase.tolist(),
            "description": "diagnostic WiLoR hand-centre fitted chain",
        },
        "time_mapping": sync,
        "calibration_frame": {
            "row_index": calibration_index,
            "sync_index": int(rows[calibration_index]["sync_index"]),
            "source_frame_index": int(rows[calibration_index][f"{args.camera}_frame_index"]),
            "ego_timestamp_ns": int(image_timestamps[calibration_index]),
            "blob_threshold": args.blob_threshold,
            "candidate_blob_count": int(len(components)),
            "projected_marker_count": int(len(predicted_pixels)),
            "matched_marker_count": int(len(source_indices)),
            "seed_shift_px": seed_shift.tolist(),
            "marker_names": marker_names,
        },
        "correction": correction_quality,
        "marker_skeleton_correspondence_mm": correspondence,
        "validation": {
            "sample_stride": validation_stride,
            "sampled_frame_count": len(validation_frame_records),
            "nearest_blob_match_count": len(validation_distances),
            "nearest_blob_median_px": (
                float(np.median(validation_distances)) if validation_distances else None
            ),
            "nearest_blob_p95_px": (
                float(np.percentile(validation_distances, 95)) if validation_distances else None
            ),
            "frames": validation_frame_records,
        },
        "skeleton_segments": skeleton_names,
        "files": {
            "selected_frame_count": len(selected_records),
            "calibration_debug": calibration_debug_path.name,
            "contact_sheet": contact_path.name if refined_contact_rows else None,
            "videos": {name: path.name for name, path in video_paths.items()},
            "selected_frames": "selected_frames",
        },
        "warning": (
            "The correction is constrained by visible RGB glove blobs in one calibration "
            "frame and is a diagnostic camera-local refinement; keep the formal hand-eye "
            "file unchanged until an independent marker/board calibration validates it."
        ),
    }
    summary_path = output / "refinement.json"
    summary_path.write_text(
        json.dumps(refinement, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    selected_path = output / "selected_frames.json"
    selected_path.write_text(
        json.dumps(selected_records, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(refinement, ensure_ascii=False, indent=2))
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
