#!/usr/bin/env python3
"""Run EGO-NOKOV time synchronization and VIO spatial hand-eye calibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--rigid-body", default="head_rigidbody")
    parser.add_argument("--ego-mcap", type=Path)
    parser.add_argument("--ego-vio-mcap", type=Path)
    parser.add_argument("--ego-pose", type=Path)
    parser.add_argument("--max-offset-s", type=float, default=30.0)
    parser.add_argument(
        "--fine-time-correction-s",
        type=float,
        default=0.0,
        help="session-specific correction added after IMU correlation",
    )
    parser.add_argument(
        "--force-pose-export",
        action="store_true",
        help="replace ego/pose.txt from the VIO MCAP even when it already exists",
    )
    parser.add_argument(
        "--allow-weak-sync",
        action="store_true",
        help="continue spatial calibration when time synchronization needs review",
    )
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def exactly_one(paths: list[Path], description: str) -> Path:
    if len(paths) != 1:
        listed = ", ".join(str(path.name) for path in paths) or "none"
        raise RuntimeError(f"expected exactly one {description}; found {listed}")
    return paths[0]


def resolve_input(path: Path | None, session: Path) -> Path | None:
    if path is None:
        return None
    return path.resolve() if path.is_absolute() else (session / path).resolve()


def locate_raw_mcap(ego_dir: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(explicit)
        return explicit
    conventional = ego_dir / "recording.mcap"
    if conventional.is_file():
        return conventional
    candidates = sorted(
        path
        for path in ego_dir.glob("*.mcap")
        if not path.name.endswith("_ego_vio.mcap")
    )
    return exactly_one(candidates, "raw EGO MCAP (excluding *_ego_vio.mcap)")


def locate_vio_mcap(ego_dir: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(explicit)
        return explicit
    return exactly_one(sorted(ego_dir.glob("*_ego_vio.mcap")), "EGO VIO MCAP")


def run(command: list[str]) -> None:
    print("\n+ " + " ".join(command), flush=True)
    subprocess.run(command, check=True)


def compact_summary(
    session: Path,
    raw_mcap: Path,
    pose_path: Path,
    nokov_csv: Path,
    sync_result: dict[str, Any],
    calibration: dict[str, Any],
) -> dict[str, Any]:
    sync_ok = sync_result.get("status") == "ok"
    spatial_status = str(calibration.get("status", "unknown"))
    status = "ok" if sync_ok and spatial_status == "ok" else "ok_with_warnings"
    if not sync_ok:
        status = "needs_review"
    return {
        "schema": "ego_nokov_time_and_space_alignment_v1",
        "status": status,
        "session_dir": str(session),
        "inputs": {
            "ego_raw_mcap": str(raw_mcap),
            "ego_pose": str(pose_path),
            "nokov_rigid_body_csv": str(nokov_csv),
        },
        "time_alignment": {
            "status": sync_result.get("status"),
            "confidence": sync_result.get("confidence"),
            "method": sync_result.get("method"),
            "time_mapping": sync_result.get("time_mapping"),
            "fine_time_correction_s": calibration.get("inputs", {}).get(
                "fine_time_correction_s"
            ),
            "effective_b_s": calibration.get("inputs", {}).get("effective_b_s"),
            "quality": sync_result.get("quality"),
        },
        "space_alignment": {
            "status": spatial_status,
            "transform_convention": calibration.get("transform_convention"),
            "frames": calibration.get("frames"),
            "transforms": calibration.get("transforms"),
            "quality": calibration.get("quality"),
            "warning": calibration.get("warning"),
        },
        "usage": {
            "time": "nokov_relative_s = a * ego_relative_s + effective_b_s",
            "nokov_world_to_ego_world": "p_We = T_We_Wm * p_Wm",
            "unit_note": "convert NOKOV millimetres to metres before applying T_We_Wm",
        },
    }


def main() -> int:
    args = parse_args()
    session = args.session_dir.resolve()
    ego_dir = session / "ego"
    nokov_dir = session / "nokov"
    sync_dir = session / "synchronization"
    calibration_dir = session / "calibration"
    if not ego_dir.is_dir() or not nokov_dir.is_dir():
        raise RuntimeError(f"session must contain ego/ and nokov/: {session}")

    raw_mcap = locate_raw_mcap(ego_dir, resolve_input(args.ego_mcap, session))
    nokov_csv = nokov_dir / "nokov_rigid_bodies.csv"
    if not nokov_csv.is_file():
        raise FileNotFoundError(nokov_csv)

    sync_dir.mkdir(parents=True, exist_ok=True)
    calibration_dir.mkdir(parents=True, exist_ok=True)
    sync_command = [
        sys.executable,
        str(SCRIPT_DIR / "synchronize_ego_imu_nokov.py"),
        "--ego-mcap",
        str(raw_mcap),
        "--nokov-csv",
        str(nokov_csv),
        "--rigid-body",
        args.rigid_body,
        "--output-dir",
        str(sync_dir),
        "--nokov-time-field",
        "device_timestamp_raw",
        "--nokov-time-scale",
        "0.001",
        "--max-offset-s",
        str(args.max_offset_s),
    ]
    if args.no_plot:
        sync_command.append("--no-plot")
    run(sync_command)
    sync_json = sync_dir / "imu_nokov_sync.json"
    sync_result = json.loads(sync_json.read_text(encoding="utf-8"))
    if sync_result.get("status") != "ok" and not args.allow_weak_sync:
        raise RuntimeError(
            "time synchronization did not pass; inspect imu_nokov_sync.json or "
            "rerun deliberately with --allow-weak-sync"
        )

    pose_path = resolve_input(args.ego_pose, session) or (ego_dir / "pose.txt")
    if args.force_pose_export or not pose_path.is_file():
        vio_mcap = locate_vio_mcap(
            ego_dir, resolve_input(args.ego_vio_mcap, session)
        )
        run(
            [
                sys.executable,
                str(SCRIPT_DIR / "export_ego_vio_pose.py"),
                "--mcap",
                str(vio_mcap),
                "--output",
                str(pose_path),
            ]
        )
    if not pose_path.is_file():
        raise FileNotFoundError(pose_path)

    calibration_json = calibration_dir / "T_nokov_ego_vio_provisional.json"
    run(
        [
            sys.executable,
            str(SCRIPT_DIR / "calibrate_ego_vio_nokov.py"),
            "--ego-pose",
            str(pose_path),
            "--nokov-csv",
            str(nokov_csv),
            "--sync-json",
            str(sync_json),
            "--rigid-body",
            args.rigid_body,
            "--time-correction-s",
            str(args.fine_time_correction_s),
            "--output",
            str(calibration_json),
        ]
    )

    calibration = json.loads(calibration_json.read_text(encoding="utf-8"))
    summary = compact_summary(
        session, raw_mcap, pose_path, nokov_csv, sync_result, calibration
    )
    summary_path = calibration_dir / "ego_nokov_alignment_summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"\nAlignment pipeline status: {summary['status']}")
    print(f"Combined report: {summary_path}")
    return 0 if summary["status"] in ("ok", "ok_with_warnings") else 2


if __name__ == "__main__":
    raise SystemExit(main())
