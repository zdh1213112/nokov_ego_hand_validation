#!/usr/bin/env python3
"""Estimate a provisional NOKOV rigid-body to DAS-Ego VIO hand-eye transform.

Convention: T_A_B maps homogeneous coordinates from frame B into frame A.
Known synchronized poses are M_i=T_Wm_B and E_i=T_We_E. The solver uses:

    M_i X = Y E_i
    X = T_B_E
    Y = T_Wm_We

and relative motions M_i^-1 M_j X = X E_i^-1 E_j.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

try:
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation, Slerp
except ImportError as exc:  # pragma: no cover - environment dependent
    raise SystemExit(
        "SciPy is required: python -m pip install -r tools/requirements-calibration.txt"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ego-pose", type=Path, required=True)
    parser.add_argument("--nokov-csv", type=Path, required=True)
    parser.add_argument("--sync-json", type=Path, required=True)
    parser.add_argument("--rigid-body", default="head_rigidbody")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--time-correction-s",
        type=float,
        default=0.0,
        help="fine correction added to the synchronization JSON b_s",
    )
    parser.add_argument("--pair-stride", type=int, default=5)
    parser.add_argument("--min-pair-rotation-deg", type=float, default=3.0)
    return parser.parse_args()


def transforms(position: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    result = np.repeat(np.eye(4, dtype=np.float64)[None], len(position), axis=0)
    result[:, :3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    result[:, :3, 3] = position
    return result


def inverse(matrix: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def load_ego_pose(path: Path) -> np.ndarray:
    rows = np.loadtxt(path, comments="#", dtype=np.float64)
    if rows.ndim != 2 or rows.shape[1] != 8 or len(rows) < 20:
        raise RuntimeError(f"expected at least 20 TUM-format poses in {path}")
    return rows


def load_nokov(path: Path, rigid_body: str) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row.get("rigid_body_name") != rigid_body:
                continue
            if row.get("valid_numeric", "1") != "1":
                continue
            rows.append(
                [
                    float(row["device_timestamp_raw"]),
                    float(row["x_mm"]) * 0.001,
                    float(row["y_mm"]) * 0.001,
                    float(row["z_mm"]) * 0.001,
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                ]
            )
    result = np.asarray(rows, dtype=np.float64)
    if len(result) < 20:
        raise RuntimeError(f"only {len(result)} valid poses for {rigid_body!r}")
    return result


def relative_pairs(
    mocap: np.ndarray,
    ego: np.ndarray,
    begin: int,
    end: int,
    stride: int,
    minimum_rotation_rad: float,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    pairs_m: list[np.ndarray] = []
    pairs_e: list[np.ndarray] = []
    # At roughly 30 Hz these cover about 0.3, 0.7, 1.3, 2.7 and 4 seconds.
    for lag in (10, 20, 40, 80, 120):
        for first in range(begin, max(begin, end - lag), stride):
            second = first + lag
            if second >= end:
                continue
            motion_m = inverse(mocap[first]) @ mocap[second]
            motion_e = inverse(ego[first]) @ ego[second]
            angle = Rotation.from_matrix(motion_m[:3, :3]).magnitude()
            if angle >= minimum_rotation_rad:
                pairs_m.append(motion_m)
                pairs_e.append(motion_e)
    if len(pairs_m) < 20:
        raise RuntimeError(f"only {len(pairs_m)} sufficiently excited motion pairs")
    return pairs_m, pairs_e


def solve_x(
    motions_m: list[np.ndarray], motions_e: list[np.ndarray]
) -> tuple[np.ndarray, dict[str, Any]]:
    def rotation_residual(rotvec: np.ndarray) -> np.ndarray:
        rotation_x = Rotation.from_rotvec(rotvec).as_matrix()
        residuals = []
        for motion_m, motion_e in zip(motions_m, motions_e):
            closure = (
                motion_m[:3, :3]
                @ rotation_x
                @ motion_e[:3, :3].T
                @ rotation_x.T
            )
            residuals.append(Rotation.from_matrix(closure).as_rotvec())
        return np.concatenate(residuals)

    candidates = []
    for seed in (
        np.zeros(3),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
    ):
        solution = least_squares(
            rotation_residual,
            seed,
            loss="soft_l1",
            f_scale=0.03,
            max_nfev=500,
        )
        candidates.append(solution)
    solution = min(candidates, key=lambda item: float(np.linalg.norm(item.fun)))
    rotation_x = Rotation.from_rotvec(solution.x).as_matrix()

    lhs = np.vstack([motion[:3, :3] - np.eye(3) for motion in motions_m])
    rhs = np.hstack(
        [
            rotation_x @ motion_e[:3, 3] - motion_m[:3, 3]
            for motion_m, motion_e in zip(motions_m, motions_e)
        ]
    )
    translation_x, _, _, singular_values = np.linalg.lstsq(lhs, rhs, rcond=None)
    matrix_x = np.eye(4, dtype=np.float64)
    matrix_x[:3, :3] = rotation_x
    matrix_x[:3, 3] = translation_x

    rotation_errors = np.asarray(
        [
            Rotation.from_matrix(
                motion_m[:3, :3]
                @ rotation_x
                @ motion_e[:3, :3].T
                @ rotation_x.T
            ).magnitude()
            for motion_m, motion_e in zip(motions_m, motions_e)
        ]
    )
    return matrix_x, {
        "motion_pair_count": len(motions_m),
        "relative_rotation_error_median_deg": float(
            np.degrees(np.median(rotation_errors))
        ),
        "translation_linear_system_singular_values": singular_values.tolist(),
    }


def percentile_summary(values: np.ndarray, scale: float = 1.0) -> dict[str, float]:
    return {
        "median": float(np.median(values) * scale),
        "rms": float(np.sqrt(np.mean(values**2)) * scale),
        "p95": float(np.percentile(values, 95) * scale),
    }


def matrix_record(matrix: np.ndarray) -> dict[str, Any]:
    return {
        "matrix": matrix.tolist(),
        "translation_m": matrix[:3, 3].tolist(),
        "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
    }


def main() -> int:
    args = parse_args()
    ego_rows = load_ego_pose(args.ego_pose)
    nokov_rows = load_nokov(args.nokov_csv, args.rigid_body)
    sync = json.loads(args.sync_json.read_text(encoding="utf-8"))
    mapping = sync["time_mapping"]
    if mapping["nokov_origin_field"] != "device_timestamp_raw":
        raise RuntimeError("sync JSON must use device_timestamp_raw")

    ego_origin_s = float(mapping["ego_origin_timestamp_ns"]) * 1e-9
    mapped_nokov_s = (
        float(mapping["a"]) * (ego_rows[:, 0] - ego_origin_s)
        + float(mapping["b_s"])
        + args.time_correction_s
    )
    nokov_time_s = (
        nokov_rows[:, 0] - float(mapping["nokov_origin_timestamp_raw"])
    ) * float(mapping["nokov_seconds_per_timestamp_unit"])
    valid = (mapped_nokov_s >= nokov_time_s[0]) & (
        mapped_nokov_s <= nokov_time_s[-1]
    )
    ego_rows = ego_rows[valid]
    mapped_nokov_s = mapped_nokov_s[valid]
    if len(ego_rows) < 30:
        raise RuntimeError(f"only {len(ego_rows)} synchronized pose rows")

    mocap_position = np.column_stack(
        [
            np.interp(mapped_nokov_s, nokov_time_s, nokov_rows[:, 1 + axis])
            for axis in range(3)
        ]
    )
    mocap_quaternion = Slerp(
        nokov_time_s, Rotation.from_quat(nokov_rows[:, 4:8])
    )(mapped_nokov_s).as_quat()
    mocap_poses = transforms(mocap_position, mocap_quaternion)
    ego_poses = transforms(ego_rows[:, 1:4], ego_rows[:, 4:8])

    motions_m, motions_e = relative_pairs(
        mocap_poses,
        ego_poses,
        0,
        len(ego_poses),
        args.pair_stride,
        np.deg2rad(args.min_pair_rotation_deg),
    )
    matrix_x, pair_quality = solve_x(motions_m, motions_e)

    split = len(ego_poses) // 2
    first_x, _ = solve_x(
        *relative_pairs(
            mocap_poses,
            ego_poses,
            0,
            split,
            args.pair_stride,
            np.deg2rad(args.min_pair_rotation_deg),
        )
    )
    second_x, _ = solve_x(
        *relative_pairs(
            mocap_poses,
            ego_poses,
            split,
            len(ego_poses),
            args.pair_stride,
            np.deg2rad(args.min_pair_rotation_deg),
        )
    )

    per_frame_y = np.asarray(
        [
            mocap @ matrix_x @ inverse(ego)
            for mocap, ego in zip(mocap_poses, ego_poses)
        ]
    )
    rotation_y = Rotation.from_matrix(per_frame_y[:, :3, :3]).mean()
    translation_y = np.median(per_frame_y[:, :3, 3], axis=0)
    matrix_y = np.eye(4, dtype=np.float64)
    matrix_y[:3, :3] = rotation_y.as_matrix()
    matrix_y[:3, 3] = translation_y

    y_rotation_error = (
        rotation_y.inv() * Rotation.from_matrix(per_frame_y[:, :3, :3])
    ).magnitude()
    y_translation_error = np.linalg.norm(
        per_frame_y[:, :3, 3] - translation_y, axis=1
    )
    half_rotation_difference = (
        Rotation.from_matrix(first_x[:3, :3]).inv()
        * Rotation.from_matrix(second_x[:3, :3])
    ).magnitude()
    half_translation_difference = np.linalg.norm(first_x[:3, 3] - second_x[:3, 3])

    report = {
        "schema": "nokov_ego_vio_handeye_v1",
        "status": "provisional_rotation_only",
        "transform_convention": "T_A_B maps homogeneous coordinates from frame B to frame A",
        "equations": {
            "absolute": "T_Wm_B(t) * T_B_E = T_Wm_We * T_We_E(t)",
            "relative": "(M_i^-1*M_j) * X = X * (E_i^-1*E_j)",
        },
        "frames": {
            "Wm": "NOKOV/XINGYING world",
            "B": args.rigid_body,
            "We": "DAS-Ego VIO world",
            "E": "DAS-Ego local body/IMU-center frame; not an individual camera",
        },
        "inputs": {
            "ego_pose": str(args.ego_pose.resolve()),
            "ego_pose_topic": "/robot0/vio/eef_pose",
            "nokov_csv": str(args.nokov_csv.resolve()),
            "sync_json": str(args.sync_json.resolve()),
            "rigid_body": args.rigid_body,
            "synchronized_pose_rows": len(ego_poses),
            "base_b_s": float(mapping["b_s"]),
            "fine_time_correction_s": args.time_correction_s,
            "effective_b_s": float(mapping["b_s"]) + args.time_correction_s,
        },
        "transforms": {
            "T_B_E": matrix_record(matrix_x),
            "T_Wm_We": matrix_record(matrix_y),
            "T_We_Wm": matrix_record(inverse(matrix_y)),
        },
        "quality": {
            **pair_quality,
            "per_frame_Y_rotation_error_deg": percentile_summary(
                y_rotation_error, 180.0 / np.pi
            ),
            "per_frame_Y_translation_error_mm": percentile_summary(
                y_translation_error, 1000.0
            ),
            "first_vs_second_X_rotation_difference_deg": float(
                np.degrees(half_rotation_difference)
            ),
            "first_vs_second_X_translation_difference_mm": float(
                half_translation_difference * 1000.0
            ),
            "first_half_T_B_E_translation_m": first_x[:3, 3].tolist(),
            "second_half_T_B_E_translation_m": second_x[:3, 3].tolist(),
        },
        "usage": {
            "ego_world_point_to_nokov_world": "p_Wm = T_Wm_We * p_We",
            "nokov_world_point_to_ego_world": "p_We = T_We_Wm * p_Wm",
            "camera_note": "For camera k, use T_B_Ck = T_B_E * T_E_Ck from the vendor URDF.",
        },
        "warning": (
            "Rotation is repeatable on this session, but translation is not: "
            "do not use the provisional translation as high-accuracy ground truth."
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Wrote {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
