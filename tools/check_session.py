#!/usr/bin/env python3
"""Check whether one NOKOV/EGO validation session is ready for processing."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any


PLACEHOLDER_WORDS = ("PLACEHOLDER_DO_NOT_USE", "TODO")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument(
        "--stage", choices=("capture", "calibrated"), default="capture",
        help="capture checks raw acquisition; calibrated additionally requires mapping and extrinsic",
    )
    parser.add_argument(
        "--allow-incomplete", action="store_true",
        help="print missing items but return success; useful before field capture",
    )
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args()


def contains_placeholder(path: Path) -> bool:
    if not path.is_file():
        return True
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    return any(word in text for word in PLACEHOLDER_WORDS)


def load_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def rotation_is_valid(value: Any) -> bool:
    if not isinstance(value, list) or len(value) != 3:
        return False
    try:
        rows = [[float(cell) for cell in row] for row in value]
    except (TypeError, ValueError):
        return False
    if any(len(row) != 3 or not all(math.isfinite(cell) for cell in row) for row in rows):
        return False
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    return abs(determinant - 1.0) < 1e-3


def csv_has_data(path: Path) -> bool:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = [row for row in csv.reader(stream) if any(cell.strip() for cell in row)]
            return len(rows) >= 2
    except OSError:
        return False


def main() -> int:
    args = parse_args()
    session = args.session.resolve()
    checks: list[dict[str, Any]] = []

    def add(name: str, ok: bool, detail: str, required: bool = True) -> None:
        checks.append({"name": name, "ok": bool(ok), "required": required, "detail": detail})

    add("session_directory", session.is_dir(), str(session))
    trc = session / "nokov" / "hand24.trc"
    c3d = session / "nokov" / "hand24.c3d"
    sdk_markers = session / "nokov" / "nokov_markers.csv"
    trc_ok = trc.is_file() and trc.stat().st_size > 0 if trc.exists() else False
    sdk_markers_ok = csv_has_data(sdk_markers)
    add(
        "nokov_marker_data", trc_ok or sdk_markers_ok,
        f"TRC={trc} ({'OK' if trc_ok else 'missing'}); "
        f"SDK CSV={sdk_markers} ({'OK' if sdk_markers_ok else 'missing'})",
    )
    add(
        "nokov_c3d_backup", c3d.is_file() and c3d.stat().st_size > 0 if c3d.exists() else False,
        str(c3d), required=False,
    )
    ego_dir = session / "ego"
    ego_recordings = []
    if ego_dir.is_dir():
        ego_recordings = [
            path for path in ego_dir.iterdir()
            if path.is_file() and path.suffix.lower() in (".mcap", ".mp4", ".mkv")
        ]
    add("ego_recording", bool(ego_recordings), ", ".join(map(str, ego_recordings)) or str(ego_dir))

    marker_names = session / "nokov" / "marker_names.txt"
    add(
        "marker_names", marker_names.is_file() and not contains_placeholder(marker_names),
        str(marker_names), required=args.stage == "calibrated",
    )
    capture_metadata_path = session / "nokov" / "capture_metadata.json"
    capture_metadata = load_json(capture_metadata_path)
    add(
        "capture_metadata",
        capture_metadata is not None and capture_metadata.get("status") != "PLACEHOLDER_DO_NOT_USE"
        and not contains_placeholder(capture_metadata_path),
        str(capture_metadata_path),
    )

    rigid_path = session / "calibration" / "head_rigidbody_definition.json"
    rigid = load_json(rigid_path)
    rigid_markers = rigid.get("marker_names", []) if rigid else []
    add(
        "head_rigidbody",
        rigid is not None and rigid.get("status") != "PLACEHOLDER_DO_NOT_USE"
        and isinstance(rigid_markers, list) and len(rigid_markers) >= 3,
        f"{rigid_path}; marker_count={len(rigid_markers) if isinstance(rigid_markers, list) else 0}",
    )

    extrinsic_path = session / "calibration" / "T_head_ego_base.json"
    extrinsic = load_json(extrinsic_path)
    translation = extrinsic.get("translation_m") if extrinsic else None
    translation_ok = (
        isinstance(translation, list) and len(translation) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(value) for value in translation)
    )
    add(
        "head_to_ego_extrinsic",
        extrinsic is not None and extrinsic.get("status") != "PLACEHOLDER_DO_NOT_USE"
        and rotation_is_valid(extrinsic.get("rotation_matrix")) and translation_ok,
        str(extrinsic_path), required=args.stage == "calibrated",
    )

    sync_path = session / "synchronization" / "sync_events.csv"
    add("sync_events", csv_has_data(sync_path), str(sync_path))
    mapping_path = session / "config" / "nokov24_to_ego21.yaml"
    add(
        "marker_mapping", mapping_path.is_file() and not contains_placeholder(mapping_path),
        str(mapping_path), required=args.stage == "calibrated",
    )

    required_failures = [check for check in checks if check["required"] and not check["ok"]]
    print(f"Session: {session}")
    for check in checks:
        status = "OK" if check["ok"] else ("WARN" if not check["required"] else "MISSING")
        print(f"[{status:7}] {check['name']:<24} {check['detail']}")
    print(
        f"\nRequired checks: {len(checks) - sum(not item['required'] for item in checks)}; "
        f"missing: {len(required_failures)}"
    )
    report = {
        "schema": "nokov_ego_session_preflight_v1",
        "session": str(session),
        "stage": args.stage,
        "ready": not required_failures,
        "checks": checks,
    }
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    if required_failures and not args.allow_incomplete:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
