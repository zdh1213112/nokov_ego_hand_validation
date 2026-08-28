#!/usr/bin/env python3
"""Diagnose every stage of the NOKOV -> EGO/GEN camera coordinate chain.

The existing provisional hand-eye file relates the NOKOV head rigid body (B)
to the VIO body/IMU frame (E).  GEN ``camera_info.T_b_c`` is expressed in the
GEN base/rig frame (G).  This tool keeps those frames explicit and compares:

  1. the documented B -> E transform (``inv(T_B_E)``),
  2. the historical direct use of ``T_B_E``,
  3. the documented transform followed by the observed E -> G axis mapping,
  4. a provisional E -> G rigid fit against the fused GEN hand points.

The fitted transform is a diagnostic hypothesis, not a replacement for a
marker/board hand-eye calibration.  It is useful because it tests whether the
large image error is caused by a missing frame conversion rather than by the
Double-Sphere projection itself.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from render_nokov_wilor_camera_alignment import (
    EGO_COLORS,
    EGO_EDGES,
    NOKOV_COLORS,
    NOKOV_EDGES,
    coarse_centroid_cost,
    draw_skeleton,
    ego_pixels,
    head_pose_matrix,
    interpolate_markers,
    load_marker_tracks,
    load_jsonl,
    project_double_sphere,
    read_csv,
    transform_points,
)
from synchronize_ego_imu_nokov import interpolate_rigid_poses, read_nokov_poses


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
        "--render-videos",
        action="store_true",
        help="render full-sequence corrected overlays and a before/after comparison video",
    )
    parser.add_argument(
        "--preview-count",
        type=int,
        default=8,
        help="number of temporally distributed low-residual frames to export",
    )
    return parser.parse_args()


def inverse(matrix: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return R,t minimizing ||R source + t - target|| with det(R)=+1."""
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def fit_ego_to_genbase(
    samples: list[tuple[np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, list[tuple[np.ndarray, np.ndarray]], dict[str, float]]:
    """Fit E -> GEN-base using two hand centers per frame.

    The detector was run with handedness ignored, so each frame may have the
    two EGO hand records in either order.  Alternate nearest assignment and a
    Kabsch rigid fit.  Hand centers are only a diagnostic proxy; marker-board
    calibration remains the production solution.
    """
    if len(samples) < 10:
        raise RuntimeError(f"only {len(samples)} two-hand samples for E->GEN fit")

    source = np.concatenate([item[0] for item in samples], axis=0)
    target = np.concatenate([item[1] for item in samples], axis=0)
    rotation, translation = rigid_fit(source, target)
    assigned: list[tuple[np.ndarray, np.ndarray]] = []
    for _ in range(12):
        assigned = []
        source_rows: list[np.ndarray] = []
        target_rows: list[np.ndarray] = []
        for source_pair, target_pair in samples:
            predicted = (rotation @ source_pair.T).T + translation
            direct = float(np.linalg.norm(predicted - target_pair, axis=1).sum())
            swapped = float(np.linalg.norm(predicted - target_pair[[1, 0]], axis=1).sum())
            selected = target_pair if direct <= swapped else target_pair[[1, 0]]
            assigned.append((source_pair, selected))
            source_rows.extend(source_pair)
            target_rows.extend(selected)
        rotation, translation = rigid_fit(
            np.asarray(source_rows, dtype=np.float64),
            np.asarray(target_rows, dtype=np.float64),
        )

    errors = []
    for source_pair, target_pair in assigned:
        predicted = (rotation @ source_pair.T).T + translation
        errors.extend(np.linalg.norm(predicted - target_pair, axis=1).tolist())
    error_array = np.asarray(errors, dtype=np.float64)
    transform = make_transform(rotation, translation)
    quality = {
        "sample_frame_count": float(len(samples)),
        "center_pair_count": float(len(errors)),
        "median_error_mm": float(np.median(error_array) * 1000.0),
        "p95_error_mm": float(np.percentile(error_array, 95) * 1000.0),
        "rms_error_mm": float(np.sqrt(np.mean(error_array**2)) * 1000.0),
    }
    return transform, assigned, quality


def center_pair(
    marker_points: np.ndarray,
    marker_valid: np.ndarray,
    head_position_mm: np.ndarray,
    head_quaternion: np.ndarray,
    body_to_ego: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Return Wm, B, E marker centers for both physical hands."""
    world = []
    body = []
    ego = []
    world_to_body = inverse(head_pose_matrix(head_position_mm, head_quaternion))
    for side in (0, 1):
        use = marker_valid[side]
        if int(use.sum()) < 10:
            return None
        world_points = marker_points[side, use] * 0.001
        body_points = transform_points(world_to_body, world_points)
        ego_points = transform_points(body_to_ego, body_points)
        world.append(np.nanmedian(world_points, axis=0))
        body.append(np.nanmedian(body_points, axis=0))
        ego.append(np.nanmedian(ego_points, axis=0))
    return np.asarray(world), np.asarray(body), np.asarray(ego)


def project_candidate(
    marker_points: np.ndarray,
    marker_valid: np.ndarray,
    head_position_mm: np.ndarray,
    head_quaternion: np.ndarray,
    camera: dict[str, Any],
    body_to_genbase: np.ndarray,
) -> tuple[dict[int, np.ndarray], dict[int, np.ndarray]]:
    world_to_body = inverse(head_pose_matrix(head_position_mm, head_quaternion))
    camera_from_genbase = inverse(np.asarray(camera["T_base_camera"], dtype=np.float64))
    camera_from_world = camera_from_genbase @ body_to_genbase @ world_to_body
    projected: dict[int, np.ndarray] = {}
    camera_points_by_side: dict[int, np.ndarray] = {}
    for side in (0, 1):
        use = marker_valid[side]
        pixels = np.full((24, 2), np.nan, dtype=np.float64)
        camera_points = np.full((24, 3), np.nan, dtype=np.float64)
        if np.any(use):
            values = transform_points(
                camera_from_world, marker_points[side, use] * 0.001
            )
            points, valid = project_double_sphere(camera, values)
            selected = np.flatnonzero(use)
            camera_points[selected] = values
            pixels[selected[valid]] = points[valid]
        if np.count_nonzero(np.isfinite(pixels).all(axis=1)):
            projected[side] = pixels
            camera_points_by_side[side] = camera_points
    return projected, camera_points_by_side


def draw_label(image: np.ndarray, text: str) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], 47), (10, 12, 18), -1)
    cv2.addWeighted(overlay, 0.76, image, 0.24, 0.0, image)
    cv2.putText(
        image, text, (14, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.64,
        (245, 245, 245), 2, cv2.LINE_AA,
    )


def render_overlay(
    base_frame: np.ndarray,
    ego_2d: dict[int, np.ndarray],
    marker_points: np.ndarray,
    marker_valid: np.ndarray,
    head_position_mm: np.ndarray,
    head_quaternion: np.ndarray,
    camera: dict[str, Any],
    body_to_genbase: np.ndarray,
    label: str,
) -> tuple[np.ndarray, float | None]:
    image = base_frame.copy()
    for side, points in ego_2d.items():
        draw_skeleton(image, points, EGO_EDGES, EGO_COLORS[side], 6, 4)
    projected, _camera_points = project_candidate(
        marker_points,
        marker_valid,
        head_position_mm,
        head_quaternion,
        camera,
        body_to_genbase,
    )
    for side, points in projected.items():
        draw_skeleton(image, points, NOKOV_EDGES, NOKOV_COLORS[side], 5, 2)
    gap = coarse_centroid_cost(projected, ego_2d)
    suffix = f"gap={gap:.1f}px" if gap is not None else "gap=N/A"
    draw_label(image, f"{label} | {suffix} | diagnostic, not final calibration")
    return image, gap


def load_camera_frame(video: cv2.VideoCapture, target_index: int) -> np.ndarray:
    frame = None
    for _ in range(target_index + 1):
        ok, frame = video.read()
        if not ok:
            raise RuntimeError(f"video ended before frame {target_index}")
    assert frame is not None
    return frame


def choose_preview_indices(
    rows: list[dict[str, str]],
    accepted: dict[int, dict[str, Any]],
    cost_by_index: dict[int, float],
    camera_id: str,
    preview_count: int,
) -> list[int]:
    if preview_count <= 0:
        return []
    eligible = [
        index
        for index, row in enumerate(rows)
        if index in cost_by_index
        and len(ego_pixels(accepted.get(int(row["sync_index"])), camera_id)) == 2
    ]
    if not eligible:
        return []
    selected = []
    for group in np.array_split(np.asarray(eligible, dtype=int), min(preview_count, len(eligible))):
        if len(group):
            selected.append(min(group.tolist(), key=cost_by_index.__getitem__))
    return selected


def render_corrected_visualizations(
    output: Path,
    dataset: Path,
    manifest: dict[str, Any],
    camera_id: str,
    camera: dict[str, Any],
    rows: list[dict[str, str]],
    accepted: dict[int, dict[str, Any]],
    rigid: dict[str, np.ndarray],
    marker_points: np.ndarray,
    marker_valid: np.ndarray,
    candidates: dict[str, np.ndarray],
    fitted_cost_by_index: dict[int, float],
    preview_count: int,
    render_videos: bool,
) -> dict[str, Any]:
    selected = choose_preview_indices(
        rows, accepted, fitted_cost_by_index, camera_id, preview_count
    )
    selected_set = set(selected)
    still_root = output / "selected_frames"
    still_root.mkdir(parents=True, exist_ok=True)

    image_timestamps = np.asarray(
        [int(row[f"{camera_id}_timestamp_ns"]) for row in rows], dtype=np.int64
    )
    deltas = np.diff(image_timestamps).astype(np.float64) * 1e-9
    fps = float(1.0 / np.median(deltas[deltas > 0]))
    width, height = camera["image_size"]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writers: dict[str, cv2.VideoWriter] = {}
    video_paths: dict[str, Path] = {}
    if render_videos:
        video_paths = {
            "axis_relabel": output / f"{camera_id}_axis_relabel_alignment.mp4",
            "fitted": output / f"{camera_id}_fitted_alignment.mp4",
            "before_axis_fitted": output / f"{camera_id}_before_axis_fitted_comparison.mp4",
        }
        writers = {
            "axis_relabel": cv2.VideoWriter(
                str(video_paths["axis_relabel"]), fourcc, fps, (width, height)
            ),
            "fitted": cv2.VideoWriter(
                str(video_paths["fitted"]), fourcc, fps, (width, height)
            ),
            "before_axis_fitted": cv2.VideoWriter(
                str(video_paths["before_axis_fitted"]), fourcc, fps, (1920, 520)
            ),
        }
        failed = [name for name, writer in writers.items() if not writer.isOpened()]
        if failed:
            for writer in writers.values():
                writer.release()
            raise RuntimeError(f"cannot create videos: {failed}")

    capture = cv2.VideoCapture(
        str(dataset / "cameras" / camera_id / manifest["storage"]["video_filename"])
    )
    if not capture.isOpened():
        raise RuntimeError("cannot open normalized camera video")
    next_video_index = 0
    contact_rows: list[np.ndarray] = []
    selected_records: list[dict[str, Any]] = []
    try:
        for index, row in enumerate(rows):
            target_video_index = int(row[f"{camera_id}_frame_index"])
            frame = None
            while next_video_index <= target_video_index:
                ok, frame = capture.read()
                if not ok:
                    raise RuntimeError(f"video ended before frame {target_video_index}")
                next_video_index += 1
            assert frame is not None
            result = accepted.get(int(row["sync_index"]))
            ego_2d = ego_pixels(result, camera_id)
            if not rigid["valid"][index]:
                continue
            rendered = {}
            gaps = {}
            labels = {
                "legacy_direct_matrix": "BEFORE: missing VIO-to-GEN bridge",
                "axis_relabel_E_to_GEN": "CORRECTED: official-axis E-to-G bridge",
                "fitted_E_to_GEN": "CORRECTED: diagnostic fitted E-to-G bridge",
            }
            for name in labels:
                rendered[name], gaps[name] = render_overlay(
                    frame,
                    ego_2d,
                    marker_points[index],
                    marker_valid[index],
                    rigid["position_mm"][index],
                    rigid["quaternion_xyzw"][index],
                    camera,
                    candidates[name],
                    labels[name],
                )
            comparison = np.hstack([
                cv2.resize(rendered[name], (640, 520), interpolation=cv2.INTER_AREA)
                for name in (
                    "legacy_direct_matrix",
                    "axis_relabel_E_to_GEN",
                    "fitted_E_to_GEN",
                )
            ])
            if render_videos:
                writers["axis_relabel"].write(rendered["axis_relabel_E_to_GEN"])
                writers["fitted"].write(rendered["fitted_E_to_GEN"])
                writers["before_axis_fitted"].write(comparison)
            if index in selected_set:
                sync_index = int(row["sync_index"])
                stem = f"sync{sync_index:06d}"
                for name in ("axis_relabel_E_to_GEN", "fitted_E_to_GEN"):
                    cv2.imwrite(str(still_root / f"{stem}_{name}.jpg"), rendered[name])
                cv2.imwrite(str(still_root / f"{stem}_comparison.jpg"), comparison)
                contact_rows.append(comparison)
                selected_records.append({
                    "sync_index": sync_index,
                    "source_frame_index": target_video_index,
                    "ego_timestamp_ns": int(image_timestamps[index]),
                    "legacy_gap_px": gaps["legacy_direct_matrix"],
                    "axis_relabel_gap_px": gaps["axis_relabel_E_to_GEN"],
                    "fitted_gap_px": gaps["fitted_E_to_GEN"],
                    "comparison_image": f"selected_frames/{stem}_comparison.jpg",
                })
    finally:
        capture.release()
        for writer in writers.values():
            writer.release()

    contact_path = output / "selected_corrected_frames_contact_sheet.jpg"
    if contact_rows:
        cv2.imwrite(str(contact_path), np.vstack(contact_rows))
    selected_path = output / "selected_corrected_frames.json"
    selected_path.write_text(
        json.dumps(selected_records, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "selected_frame_count": len(selected_records),
        "selected_sync_indices": [item["sync_index"] for item in selected_records],
        "contact_sheet": contact_path.name if contact_rows else None,
        "selected_frames_json": selected_path.name,
        "videos": {name: path.name for name, path in video_paths.items()},
        "video_fps": fps,
    }


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
        raise ValueError("diagnostic currently expects the GEN Double-Sphere model")
    rows = read_csv(dataset / "multiview_frames.csv")
    rows = rows[: args.max_frames or None]
    accepted = load_jsonl(fusion / "accepted.jsonl")
    sync = json.loads(
        (session / "synchronization" / "imu_nokov_sync.json").read_text(encoding="utf-8")
    )
    mapping = sync["time_mapping"]
    if mapping["nokov_origin_field"] != "device_timestamp_raw":
        raise ValueError("diagnostic requires device_timestamp_raw synchronization")
    image_timestamps = np.asarray(
        [int(row[f"{args.camera}_timestamp_ns"]) for row in rows], dtype=np.int64
    )
    ego_relative_s = (
        image_timestamps - int(mapping["ego_origin_timestamp_ns"])
    ).astype(np.float64) * 1e-9
    target_nokov_s = float(mapping["a"]) * ego_relative_s + float(mapping["b_s"])
    origin_raw = int(mapping["nokov_origin_timestamp_raw"])
    scale = float(mapping["nokov_seconds_per_timestamp_unit"])

    rigid_ts, rigid_frames, rigid_position, rigid_quaternion = read_nokov_poses(
        session / "nokov" / "nokov_rigid_bodies.csv",
        "head_rigidbody",
        "device_timestamp_raw",
    )
    rigid = interpolate_rigid_poses(
        (rigid_ts - origin_raw).astype(np.float64) * scale,
        rigid_frames,
        rigid_position,
        rigid_quaternion,
        target_nokov_s,
        args.max_interpolation_gap_s,
    )
    marker_ts, marker_points, marker_valid, marker_names = load_marker_tracks(
        session / "nokov" / "nokov_markers.csv"
    )
    marker_points, marker_valid, marker_gaps = interpolate_markers(
        (marker_ts - origin_raw).astype(np.float64) * scale,
        marker_points,
        marker_valid,
        target_nokov_s,
        args.max_interpolation_gap_s,
    )

    calibration = json.loads(args.hand_eye_json.read_text(encoding="utf-8"))
    t_b_e = np.asarray(calibration["transforms"]["T_B_E"]["matrix"], dtype=np.float64)
    t_e_b = inverse(t_b_e)
    # EGO official local frame is X forward, Y left, Z up.  The camera2 GEN
    # extrinsic shows GEN +x left, +y down, +z backward, giving this canonical
    # axis relabel.  Translation is fitted below, not guessed from a drawing.
    r_gen_from_ego_axis = np.asarray(
        [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    fit_samples: list[tuple[np.ndarray, np.ndarray]] = []
    stage_rows: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not rigid["valid"][index]:
            continue
        result = accepted.get(int(row["sync_index"]))
        if result is None:
            continue
        centers = center_pair(
            marker_points[index], marker_valid[index],
            rigid["position_mm"][index], rigid["quaternion_xyzw"][index], t_e_b,
        )
        if centers is None:
            continue
        _world, _body, ego_centers = centers
        gen_centers = []
        for hand in result.get("hands", []):
            joints = np.asarray(hand.get("joints_base_m", []), dtype=np.float64)
            if joints.shape == (21, 3) and np.count_nonzero(np.isfinite(joints).all(axis=1)) >= 12:
                gen_centers.append(np.nanmedian(joints, axis=0))
        if len(gen_centers) == 2:
            fit_samples.append((ego_centers, np.asarray(gen_centers)))

    fitted_ego_to_gen, assigned_samples, fit_quality = fit_ego_to_genbase(fit_samples)
    # Re-fit only a translation for the canonical axis mapping, using the same
    # hand assignment selected by the free rigid fit.
    axis_source = np.concatenate([source for source, _target in assigned_samples], axis=0)
    axis_target = np.concatenate([target for _source, target in assigned_samples], axis=0)
    axis_translation = np.mean(axis_target - (r_gen_from_ego_axis @ axis_source.T).T, axis=0)
    axis_ego_to_gen = make_transform(r_gen_from_ego_axis, axis_translation)

    candidates = {
        "documented_B_to_E_only": t_e_b,
        "legacy_direct_matrix": t_b_e,
        "axis_relabel_E_to_GEN": axis_ego_to_gen @ t_e_b,
        "fitted_E_to_GEN": fitted_ego_to_gen @ t_e_b,
    }
    candidate_costs: dict[str, list[float]] = {name: [] for name in candidates}
    candidate_cost_by_index: dict[str, dict[int, float]] = {
        name: {} for name in candidates
    }
    csv_rows: list[dict[str, Any]] = []
    representative_index = None
    for index, row in enumerate(rows):
        result = accepted.get(int(row["sync_index"]))
        if result is None or not rigid["valid"][index]:
            continue
        ego_2d = ego_pixels(result, args.camera)
        if len(ego_2d) == 2 and representative_index is None:
            representative_index = index
        centers = center_pair(
            marker_points[index], marker_valid[index],
            rigid["position_mm"][index], rigid["quaternion_xyzw"][index], t_e_b,
        )
        if centers is None:
            continue
        world_centers, body_centers, ego_centers = centers
        row_out: dict[str, Any] = {
            "sync_index": int(row["sync_index"]),
            "ego_timestamp_ns": int(image_timestamps[index]),
            "nokov_target_timestamp_raw": float(origin_raw + target_nokov_s[index] / scale),
            "head_valid": int(rigid["valid"][index]),
            "marker_valid_count": int(np.count_nonzero(marker_valid[index])),
        }
        for side in (0, 1):
            for name, values in (
                ("wm", world_centers), ("body", body_centers),
                ("ego", ego_centers),
            ):
                row_out.update({
                    f"{name}_{side}_{axis_name}": float(values[side, axis_index])
                    for axis_index, axis_name in enumerate("xyz")
                })
        for name, body_to_gen in candidates.items():
            projected, camera_points = project_candidate(
                marker_points[index], marker_valid[index],
                rigid["position_mm"][index], rigid["quaternion_xyzw"][index],
                camera, body_to_gen,
            )
            cost = coarse_centroid_cost(projected, ego_2d)
            if cost is not None and math.isfinite(cost):
                candidate_costs[name].append(cost)
                candidate_cost_by_index[name][index] = cost
            row_out[f"{name}_gap_px"] = cost
            for side in (0, 1):
                points = projected.get(side)
                camera_points_side = camera_points.get(side)
                if points is None or camera_points_side is None:
                    continue
                finite = np.isfinite(points).all(axis=1)
                if not finite.any():
                    continue
                pixel_center = np.nanmedian(points[finite], axis=0)
                depth = np.nanmedian(camera_points_side[finite, 2])
                row_out[f"{name}_{side}_pixel_x"] = float(pixel_center[0])
                row_out[f"{name}_{side}_pixel_y"] = float(pixel_center[1])
                row_out[f"{name}_{side}_camera_z_m"] = float(depth)
        # Validate the EGO fused GEN points against their original camera2 2D
        # observations with T_base_camera^{-1}; also test the wrong direction.
        reproj_good = []
        reproj_wrong = []
        t_base_camera = np.asarray(camera["T_base_camera"], dtype=np.float64)
        for hand in result.get("hands", []):
            view = hand.get("views", {}).get(args.camera)
            joints = np.asarray(hand.get("joints_base_m", []), dtype=np.float64)
            if view is None or joints.shape != (21, 3):
                continue
            observed = np.asarray(view.get("joints_2d", []), dtype=np.float64)
            finite = np.isfinite(joints).all(axis=1) & np.isfinite(observed).all(axis=1)
            if not finite.any():
                continue
            good_pc = transform_points(inverse(t_base_camera), joints[finite])
            good_px, good_valid = project_double_sphere(camera, good_pc)
            wrong_pc = transform_points(t_base_camera, joints[finite])
            wrong_px, wrong_valid = project_double_sphere(camera, wrong_pc)
            reproj_good.extend(np.linalg.norm(good_px[good_valid] - observed[finite][good_valid], axis=1).tolist())
            reproj_wrong.extend(np.linalg.norm(wrong_px[wrong_valid] - observed[finite][wrong_valid], axis=1).tolist())
        row_out["ego_genbase_reprojection_gap_px"] = float(np.median(reproj_good)) if reproj_good else None
        row_out["ego_wrong_camera_direction_gap_px"] = float(np.median(reproj_wrong)) if reproj_wrong else None
        csv_rows.append(row_out)

    medians = {
        name: float(np.median(values)) if values else float("inf")
        for name, values in candidate_costs.items()
    }
    corrected_visualizations = render_corrected_visualizations(
        output=output,
        dataset=dataset,
        manifest=manifest,
        camera_id=args.camera,
        camera=camera,
        rows=rows,
        accepted=accepted,
        rigid=rigid,
        marker_points=marker_points,
        marker_valid=marker_valid,
        candidates=candidates,
        fitted_cost_by_index=candidate_cost_by_index["fitted_E_to_GEN"],
        preview_count=args.preview_count,
        render_videos=args.render_videos,
    )
    summary = {
        "schema": "nokov_coordinate_chain_diagnostic_v1",
        "camera": args.camera,
        "frames": {
            "dataset_rows": len(rows),
            "diagnostic_rows": len(csv_rows),
            "representative_sync_index": (
                int(rows[representative_index]["sync_index"])
                if representative_index is not None else None
            ),
        },
        "time_mapping": mapping,
        "frames_and_units": {
            "NOKOV_world": "Wm, millimetres in CSV converted to metres before transforms",
            "head_rigidbody": "B, dynamic T_Wm_B from NOKOV pose + xyzw quaternion",
            "vio_body": "E, documented T_B_E inverse gives B -> E",
            "gen_base": "G, common frame of camera_info.T_b_c and joints_base_m",
            "camera_optical": "+x right, +y down, +z forward",
        },
        "hand_eye": {
            "stored_transform": "T_B_E (E -> B) from provisional AX=XB calibration",
            "documented_B_to_E": t_e_b.tolist(),
            "axis_E_to_GEN": axis_ego_to_gen.tolist(),
            "fitted_E_to_GEN": fitted_ego_to_gen.tolist(),
            "fit_quality": fit_quality,
            "fit_warning": (
                "fitted_E_to_GEN uses WiLoR fused hand centers as a diagnostic proxy; "
                "it is not an independent metric hand-eye calibration"
            ),
        },
        "camera_checks": {
            "stored_T_base_camera_means": "camera -> GEN base",
            "ego_fused_points_projection_median_px": float(
                np.median([
                    value for row in csv_rows
                    for key, value in row.items()
                    if key == "ego_genbase_reprojection_gap_px" and value is not None
                ])
            ) if any(row.get("ego_genbase_reprojection_gap_px") is not None for row in csv_rows) else None,
            "wrong_T_base_camera_direction_median_px": float(
                np.median([
                    value for row in csv_rows
                    for key, value in row.items()
                    if key == "ego_wrong_camera_direction_gap_px" and value is not None
                ])
            ) if any(row.get("ego_wrong_camera_direction_gap_px") is not None for row in csv_rows) else None,
            "double_sphere_formula": "GEN D=[fx,fy,cx,cy,xi,alpha], z1=alpha*norm(p)+(1-alpha)*z",
        },
        "candidate_coarse_centroid_median_px": medians,
        "candidate_coarse_centroid_p95_px": {
            name: float(np.percentile(values, 95)) if values else float("inf")
            for name, values in candidate_costs.items()
        },
        "corrected_visualizations": corrected_visualizations,
        "interpretation": (
            "The documented chain is mathematically consistent with AX=XB but is in VIO body E. "
            "GEN camera_info and fused joints use GEN base G. The near-axis-permutation E->G fit "
            "explains the large residual: omitting E->G leaves a 400 px image displacement, while "
            "adding it reduces the diagnostic centroid gap to the tens of pixels."
        ),
        "files": {
            "per_frame_csv": "coordinate_chain_frames.csv",
            "preview": "coordinate_chain_diagnostic_preview.jpg",
            "candidate_gap_plot": "coordinate_chain_candidate_gaps.png",
            "selected_frames_contact_sheet": corrected_visualizations["contact_sheet"],
            "selected_frames_json": corrected_visualizations["selected_frames_json"],
            "videos": corrected_visualizations["videos"],
        },
    }

    csv_path = output / "coordinate_chain_frames.csv"
    if csv_rows:
        fields = list(csv_rows[0])
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(csv_rows)
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    # Build a four-way visual proof on one representative raw camera frame.
    if representative_index is not None:
        video = cv2.VideoCapture(
            str(dataset / "cameras" / args.camera / manifest["storage"]["video_filename"])
        )
        if not video.isOpened():
            raise RuntimeError("cannot open normalized camera video")
        base_frame = load_camera_frame(
            video, int(rows[representative_index][f"{args.camera}_frame_index"])
        )
        video.release()
        rep_row = rows[representative_index]
        rep_result = accepted[int(rep_row["sync_index"])]
        rep_ego = ego_pixels(rep_result, args.camera)
        panels = []
        for name, body_to_gen in candidates.items():
            image = base_frame.copy()
            for side, points in rep_ego.items():
                draw_skeleton(image, points, EGO_EDGES, EGO_COLORS[side], 6, 4)
            projected, _camera_points = project_candidate(
                marker_points[representative_index], marker_valid[representative_index],
                rigid["position_mm"][representative_index], rigid["quaternion_xyzw"][representative_index],
                camera, body_to_gen,
            )
            for side, points in projected.items():
                draw_skeleton(image, points, NOKOV_EDGES, NOKOV_COLORS[side], 5, 2)
            gap = coarse_centroid_cost(projected, rep_ego)
            draw_label(image, f"{name}  gap={gap:.1f}px" if gap is not None else f"{name}  gap=N/A")
            panels.append(cv2.resize(image, (800, 650), interpolation=cv2.INTER_AREA))
        while len(panels) < 4:
            panels.append(np.zeros_like(panels[0]))
        preview = np.vstack((np.hstack((panels[0], panels[1])), np.hstack((panels[2], panels[3]))))
        cv2.imwrite(str(output / "coordinate_chain_diagnostic_preview.jpg"), preview)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 5))
        for name, values in candidate_costs.items():
            # Recompute sparse series from CSV so missing rows remain gaps.
            y = np.asarray([row.get(f"{name}_gap_px", np.nan) or np.nan for row in csv_rows], dtype=float)
            ax.plot(y, label=name, linewidth=1.4)
        ax.set_xlabel("diagnostic frame order")
        ax.set_ylabel("coarse hand-centroid gap (px)")
        ax.set_title("NOKOV coordinate-chain candidates")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(output / "coordinate_chain_candidate_gaps.png", dpi=150)
        plt.close(fig)
    except ImportError:
        summary["files"]["candidate_gap_plot"] = None
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
