#!/usr/bin/env python3
"""Record NOKOV Hand(24), head rigid-body and timing data through the SDK.

Run XINGYING and its Data Adapter first. Use --list-only before recording to
discover the exact MarkerSet and rigid-body names exposed by the live scene.
The recorder writes normalized CSV files and never modifies the XINGYING CAP
project.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from typing import Any
import zipfile


FRAME_FIELDS = (
    "frame_no", "device_timestamp_raw", "timecode", "timecode_subframe",
    "receive_perf_ns", "receive_unix_ns", "sdk_latency_s", "marker_set_count",
    "rigid_body_count", "skeleton_count", "unidentified_marker_count",
)
MARKER_FIELDS = (
    "frame_no", "device_timestamp_raw", "receive_perf_ns", "receive_unix_ns",
    "sdk_latency_s", "markerset_name", "marker_index", "marker_name", "valid",
    "x_mm", "y_mm", "z_mm",
)
RIGID_FIELDS = (
    "frame_no", "device_timestamp_raw", "receive_perf_ns", "receive_unix_ns",
    "sdk_latency_s", "rigid_body_id", "rigid_body_name", "valid_numeric",
    "x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw", "mean_error", "params",
)
RIGID_MARKER_FIELDS = (
    "frame_no", "device_timestamp_raw", "receive_perf_ns", "receive_unix_ns",
    "rigid_body_id", "rigid_body_name", "marker_index", "marker_id", "valid",
    "x_mm", "y_mm", "z_mm", "size_mm",
)
SKELETON_FIELDS = (
    "frame_no", "device_timestamp_raw", "receive_perf_ns", "receive_unix_ns",
    "skeleton_id", "skeleton_name", "segment_id", "segment_name", "valid_numeric",
    "x_mm", "y_mm", "z_mm", "qx", "qy", "qz", "qw", "mean_error", "params",
)


def parse_args() -> argparse.Namespace:
    package_dir = Path(__file__).resolve().parent.parent
    compact_wheel = (
        package_dir / "vendor" / "nokov_python_sdk"
        / "nokovpy-3.0.1-py3-none-any.whl"
    )
    legacy_wheel = (
        package_dir / "vendor" / "nokov_python_sdk" / "xing_python_sdk_4.1.0.5645"
        / "dist" / "nokovpy-3.0.1-py3-none-any.whl"
    )
    default_wheel = compact_wheel if compact_wheel.is_file() else legacy_wheel
    default_output = package_dir / "sessions" / "session_001" / "nokov"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="10.1.1.198", help="XINGYING SDK server address")
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument(
        "--hand-markerset", action="append", default=[],
        help="exact Hand(24) MarkerSet name; repeat for two hands; default records all MarkerSets",
    )
    parser.add_argument(
        "--rigid-only", action="store_true",
        help="record the selected head rigid body without writing any MarkerSet rows",
    )
    parser.add_argument("--head-rigidbody", default="head_rigidbody")
    parser.add_argument("--expected-hand-markers", type=int, default=24)
    parser.add_argument("--duration", type=float, default=30.0, help="seconds; 0 records until Ctrl+C")
    parser.add_argument("--start-delay", type=float, default=0.0)
    parser.add_argument("--queue-size", type=int, default=512)
    parser.add_argument("--list-only", action="store_true", help="list live assets without recording")
    parser.add_argument(
        "--interactive-events", action="store_true",
        help="Enter adds a sync event; type q then Enter to stop",
    )
    parser.add_argument("--include-unidentified", action="store_true")
    parser.add_argument("--allow-missing-assets", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--sdk-wheel", type=Path, default=default_wheel)
    return parser.parse_args()


def decode_name(value: Any) -> str:
    if isinstance(value, bytes):
        raw = value.split(b"\0", 1)[0]
    else:
        try:
            raw = bytes(value).split(b"\0", 1)[0]
        except Exception:
            return str(value)
    for encoding in ("utf-8", "gb18030", "latin-1"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", errors="replace")


def load_sdk(wheel_path: Path):
    try:
        return importlib.import_module("nokov.nokovsdk")
    except (ImportError, OSError):
        pass
    wheel_path = wheel_path.resolve()
    if not wheel_path.is_file():
        raise RuntimeError(
            f"NOKOV SDK is not installed and bundled wheel was not found: {wheel_path}"
        )
    runtime = wheel_path.parent.parent / "runtime_extracted"
    sdk_py = runtime / "nokov" / "nokovsdk.py"
    if not sdk_py.is_file():
        runtime.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(wheel_path) as archive:
            archive.extractall(runtime)
    sys.path.insert(0, str(runtime))
    try:
        return importlib.import_module("nokov.nokovsdk")
    except OSError as exc:
        raise RuntimeError(
            "NOKOV native SDK failed to load. On Windows install the bundled "
            "VC_redist.x64.exe; also verify that Python and the SDK are both 64-bit. "
            f"Original error: {exc}"
        ) from exc


def numeric_valid(values: tuple[float, ...], quaternion: bool = False) -> bool:
    if not all(math.isfinite(value) for value in values):
        return False
    if quaternion:
        return math.sqrt(sum(value * value for value in values[-4:])) > 0.5
    return not all(abs(value) < 1e-12 for value in values)


def read_descriptions(client: Any, sdk: Any) -> dict[str, Any]:
    pointer = sdk.POINTER(sdk.DataDescriptions)()
    result = client.PyGetDataDescriptions(pointer)
    if result != 0 or not pointer:
        raise RuntimeError(f"PyGetDataDescriptions failed with code {result}")
    descriptions = pointer.contents
    marker_sets: dict[str, list[str]] = {}
    rigid_bodies: dict[int, str] = {}
    skeletons: dict[int, dict[str, Any]] = {}
    frame_rate = None
    unknown_descriptor_types: list[int] = []
    for index in range(descriptions.nDataDescriptions):
        description = descriptions.arrDataDescriptions[index]
        kind = int(description.type)
        if kind == sdk.DataDescriptors.Descriptor_MarkerSet.value:
            value = description.Data.MarkerSetDescription.contents
            name = decode_name(value.szName)
            marker_sets[name] = [
                decode_name(value.szMarkerNames[marker_index])
                for marker_index in range(value.nMarkers)
            ]
        elif kind == sdk.DataDescriptors.Descriptor_RigidBody.value:
            value = description.Data.RigidBodyDescription.contents
            rigid_bodies[int(value.ID)] = decode_name(value.szName)
        elif kind == sdk.DataDescriptors.Descriptor_Skeleton.value:
            value = description.Data.SkeletonDescription.contents
            segments = {
                int(value.RigidBodies[segment].ID): decode_name(value.RigidBodies[segment].szName)
                for segment in range(value.nRigidBodies)
            }
            skeletons[int(value.skeletonID)] = {
                "name": decode_name(value.szName),
                "segments": segments,
            }
        elif kind == sdk.DataDescriptors.Descriptor_Param.value:
            frame_rate = int(description.Data.DataParam.contents.nFrameRate)
        else:
            unknown_descriptor_types.append(kind)
    return {
        "marker_sets": marker_sets,
        "rigid_bodies": rigid_bodies,
        "skeletons": skeletons,
        "frame_rate_hz": frame_rate,
        "unknown_descriptor_types": sorted(set(unknown_descriptor_types)),
    }


def serializable_descriptions(descriptions: dict[str, Any]) -> dict[str, Any]:
    return {
        "marker_sets": descriptions["marker_sets"],
        "rigid_bodies": {str(key): value for key, value in descriptions["rigid_bodies"].items()},
        "skeletons": {
            str(key): {
                "name": value["name"],
                "segments": {str(seg): name for seg, name in value["segments"].items()},
            }
            for key, value in descriptions["skeletons"].items()
        },
        "frame_rate_hz": descriptions["frame_rate_hz"],
        "unknown_descriptor_types": descriptions["unknown_descriptor_types"],
    }


def validate_assets(args: argparse.Namespace, descriptions: dict[str, Any]) -> list[str]:
    problems = []
    available_sets = descriptions["marker_sets"]
    if args.hand_markerset:
        missing = [name for name in args.hand_markerset if name not in available_sets]
        if missing:
            problems.append(f"MarkerSet not found: {missing}")
        for name in args.hand_markerset:
            if name in available_sets and len(available_sets[name]) != args.expected_hand_markers:
                problems.append(
                    f"MarkerSet {name!r} has {len(available_sets[name])} described markers, "
                    f"expected {args.expected_hand_markers}"
                )
    available_rigids = set(descriptions["rigid_bodies"].values())
    if args.head_rigidbody and args.head_rigidbody not in available_rigids:
        problems.append(f"head rigid body {args.head_rigidbody!r} was not found")
    return problems


class CsvWriterThread(threading.Thread):
    def __init__(self, output: Path, packets: queue.Queue, sentinel: object):
        super().__init__(name="nokov-csv-writer", daemon=True)
        self.output = output
        self.packets = packets
        self.sentinel = sentinel
        self.error: Exception | None = None
        self.counts = {"frames": 0, "markers": 0, "rigid_bodies": 0, "rigid_markers": 0, "skeleton_segments": 0}

    def run(self) -> None:
        try:
            self._run()
        except Exception as exc:  # reported to the main thread after join
            self.error = exc

    def _run(self) -> None:
        paths = {
            "frames": (self.output / "nokov_frames.csv", FRAME_FIELDS),
            "markers": (self.output / "nokov_markers.csv", MARKER_FIELDS),
            "rigid_bodies": (self.output / "nokov_rigid_bodies.csv", RIGID_FIELDS),
            "rigid_markers": (self.output / "nokov_rigid_body_markers.csv", RIGID_MARKER_FIELDS),
            "skeleton_segments": (self.output / "nokov_skeleton_segments.csv", SKELETON_FIELDS),
        }
        streams = {}
        writers = {}
        try:
            for key, (path, fields) in paths.items():
                stream = path.open("w", encoding="utf-8-sig", newline="")
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                streams[key] = stream
                writers[key] = writer
            while True:
                packet = self.packets.get()
                if packet is self.sentinel:
                    break
                writers["frames"].writerow(packet["frame"])
                self.counts["frames"] += 1
                for key in ("markers", "rigid_bodies", "rigid_markers", "skeleton_segments"):
                    writers[key].writerows(packet[key])
                    self.counts[key] += len(packet[key])
                if self.counts["frames"] % 100 == 0:
                    for stream in streams.values():
                        stream.flush()
        finally:
            for stream in streams.values():
                stream.close()


def write_marker_names(path: Path, marker_sets: dict[str, list[str]], selected: list[str]) -> Path:
    chosen = selected or list(marker_sets)
    lines = ["# markerset_name,marker_index,marker_name,mediapipe_joint_or_auxiliary"]
    for marker_set in chosen:
        for index, name in enumerate(marker_sets.get(marker_set, [])):
            lines.append(f"{marker_set},{index},{name},TODO")
    # Keep a manually completed semantic mapping. The session template contains
    # TODO and is safe to replace; any non-placeholder file is preserved.
    destination = path
    if path.is_file():
        existing = path.read_text(encoding="utf-8-sig", errors="replace")
        if "TODO" not in existing and "PLACEHOLDER_DO_NOT_USE" not in existing:
            destination = path.with_name("marker_names_from_sdk.txt")
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination


def write_events(path: Path, events: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("event", "perf_ns", "unix_ns", "note"))
        writer.writeheader()
        writer.writerows(events)


def main() -> int:
    args = parse_args()
    if args.duration < 0 or args.start_delay < 0 or args.queue_size < 8:
        print("[FAIL] duration/start-delay must be non-negative and queue-size >= 8", file=sys.stderr)
        return 2
    if args.rigid_only and args.hand_markerset:
        print("[FAIL] --rigid-only cannot be combined with --hand-markerset", file=sys.stderr)
        return 2
    try:
        sdk = load_sdk(args.sdk_wheel)
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    recorded_names = (
        "nokov_frames.csv", "nokov_markers.csv", "nokov_rigid_bodies.csv",
        "nokov_rigid_body_markers.csv", "nokov_skeleton_segments.csv", "events.csv",
    )
    existing = [
        output / name for name in recorded_names
        if (output / name).is_file() and (output / name).stat().st_size > 0
    ]
    metadata_path = output / "capture_metadata.json"
    if metadata_path.is_file():
        old_metadata = metadata_path.read_text(encoding="utf-8-sig", errors="replace")
        if "PLACEHOLDER_DO_NOT_USE" not in old_metadata:
            existing.append(metadata_path)
    if existing and not args.overwrite and not args.list_only:
        print("[FAIL] capture outputs already exist; use a new session or --overwrite:", file=sys.stderr)
        for path in existing:
            print(f"  {path}", file=sys.stderr)
        return 2

    client = sdk.PySDKClient()
    version = tuple(int(value) for value in client.PyNokovVersion())
    print(f"NOKOV SDK version: {'.'.join(map(str, version))}")
    result = client.Initialize(args.server.encode("utf-8"))
    if result != 0:
        print(f"[FAIL] SDK connection to {args.server} failed with code {result}", file=sys.stderr)
        return 2
    print(f"Connected to XINGYING/NOKOV: {args.server}")

    try:
        descriptions = read_descriptions(client, sdk)
    except Exception as exc:
        print(f"[FAIL] cannot read live asset descriptions: {exc}", file=sys.stderr)
        return 2
    serializable = serializable_descriptions(descriptions)
    (output / "asset_descriptions.json").write_text(
        json.dumps(serializable, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(serializable, ensure_ascii=False, indent=2))
    problems = validate_assets(args, descriptions)
    if problems:
        print("Asset warnings:")
        for problem in problems:
            print(f"  - {problem}")
        if not args.allow_missing_assets and not args.list_only:
            print("[FAIL] fix asset names or pass --allow-missing-assets for diagnostics", file=sys.stderr)
            return 2
    if args.list_only:
        print(f"Asset description written: {output / 'asset_descriptions.json'}")
        return 0

    selected_sets = set() if args.rigid_only else (
        set(args.hand_markerset) if args.hand_markerset else None
    )
    selected_head_ids = {
        rigid_id for rigid_id, name in descriptions["rigid_bodies"].items()
        if name == args.head_rigidbody
    }
    packet_queue: queue.Queue = queue.Queue(maxsize=args.queue_size)
    sentinel = object()
    writer = CsvWriterThread(output, packet_queue, sentinel)
    state = {
        "accepting": True,
        "previous_frame": None,
        "callback_frames": 0,
        "dropped_frames": 0,
        "callback_errors": 0,
        "observed_marker_counts": {},
        "observed_head_marker_ids": set(),
    }
    state_lock = threading.Lock()

    def data_callback(frame_pointer: Any, _user_data: Any) -> None:
        if not state["accepting"] or frame_pointer is None:
            return
        try:
            frame = frame_pointer.contents
            frame_no = int(frame.iFrame)
            if frame_no == state["previous_frame"]:
                return
            state["previous_frame"] = frame_no
            receive_perf_ns = time.perf_counter_ns()
            receive_unix_ns = time.time_ns()
            device_timestamp = int(frame.iTimeStamp)
            latency = float(frame.fLatency)
            frame_common = {
                "frame_no": frame_no,
                "device_timestamp_raw": device_timestamp,
                "receive_perf_ns": receive_perf_ns,
                "receive_unix_ns": receive_unix_ns,
                "sdk_latency_s": latency,
            }
            marker_rows = []
            for set_index in range(int(frame.nMarkerSets)):
                marker_set = frame.MocapData[set_index]
                set_name = decode_name(marker_set.szName)
                if selected_sets is not None and set_name not in selected_sets:
                    continue
                described_names = descriptions["marker_sets"].get(set_name, [])
                marker_count = int(marker_set.nMarkers)
                state["observed_marker_counts"].setdefault(set_name, set()).add(marker_count)
                for marker_index in range(marker_count):
                    point = tuple(float(marker_set.Markers[marker_index][axis]) for axis in range(3))
                    marker_rows.append({
                        **frame_common,
                        "markerset_name": set_name,
                        "marker_index": marker_index,
                        "marker_name": (
                            described_names[marker_index]
                            if marker_index < len(described_names)
                            else f"marker_{marker_index:02d}"
                        ),
                        "valid": int(numeric_valid(point)),
                        "x_mm": point[0], "y_mm": point[1], "z_mm": point[2],
                    })

            rigid_rows = []
            rigid_marker_rows = []
            for body_index in range(int(frame.nRigidBodies)):
                body = frame.RigidBodies[body_index]
                rigid_id = int(body.ID)
                if args.head_rigidbody and rigid_id not in selected_head_ids:
                    continue
                name = descriptions["rigid_bodies"].get(rigid_id, f"id:{rigid_id}")
                values = (
                    float(body.x), float(body.y), float(body.z), float(body.qx),
                    float(body.qy), float(body.qz), float(body.qw),
                )
                rigid_rows.append({
                    **frame_common,
                    "rigid_body_id": rigid_id, "rigid_body_name": name,
                    "valid_numeric": int(numeric_valid(values, quaternion=True)),
                    "x_mm": values[0], "y_mm": values[1], "z_mm": values[2],
                    "qx": values[3], "qy": values[4], "qz": values[5], "qw": values[6],
                    "mean_error": float(body.MeanError), "params": int(body.params),
                })
                for marker_index in range(int(body.nMarkers)):
                    point = tuple(float(body.Markers[marker_index][axis]) for axis in range(3))
                    marker_id = int(body.MarkerIDs[marker_index])
                    state["observed_head_marker_ids"].add(marker_id)
                    rigid_marker_rows.append({
                        "frame_no": frame_no, "device_timestamp_raw": device_timestamp,
                        "receive_perf_ns": receive_perf_ns, "receive_unix_ns": receive_unix_ns,
                        "rigid_body_id": rigid_id, "rigid_body_name": name,
                        "marker_index": marker_index, "marker_id": marker_id,
                        "valid": int(numeric_valid(point)),
                        "x_mm": point[0], "y_mm": point[1], "z_mm": point[2],
                        "size_mm": float(body.MarkerSizes[marker_index]),
                    })

            skeleton_rows = []
            for skeleton_index in range(int(frame.nSkeletons)):
                skeleton = frame.Skeletons[skeleton_index]
                skeleton_id = int(skeleton.skeletonID)
                skeleton_description = descriptions["skeletons"].get(
                    skeleton_id, {"name": f"skeleton:{skeleton_id}", "segments": {}}
                )
                for segment_index in range(int(skeleton.nRigidBodies)):
                    segment = skeleton.RigidBodyData[segment_index]
                    segment_id = int(segment.ID)
                    values = (
                        float(segment.x), float(segment.y), float(segment.z), float(segment.qx),
                        float(segment.qy), float(segment.qz), float(segment.qw),
                    )
                    skeleton_rows.append({
                        "frame_no": frame_no, "device_timestamp_raw": device_timestamp,
                        "receive_perf_ns": receive_perf_ns, "receive_unix_ns": receive_unix_ns,
                        "skeleton_id": skeleton_id, "skeleton_name": skeleton_description["name"],
                        "segment_id": segment_id,
                        "segment_name": skeleton_description["segments"].get(
                            segment_id, f"segment:{segment_id}"
                        ),
                        "valid_numeric": int(numeric_valid(values, quaternion=True)),
                        "x_mm": values[0], "y_mm": values[1], "z_mm": values[2],
                        "qx": values[3], "qy": values[4], "qz": values[5], "qw": values[6],
                        "mean_error": float(segment.MeanError), "params": int(segment.params),
                    })

            if args.include_unidentified:
                for marker_index in range(int(frame.nOtherMarkers)):
                    point = tuple(float(frame.OtherMarkers[marker_index][axis]) for axis in range(3))
                    marker_rows.append({
                        **frame_common,
                        "markerset_name": "__unidentified__", "marker_index": marker_index,
                        "marker_name": f"unidentified_{marker_index:04d}",
                        "valid": int(numeric_valid(point)),
                        "x_mm": point[0], "y_mm": point[1], "z_mm": point[2],
                    })

            packet = {
                "frame": {
                    **frame_common,
                    "timecode": int(frame.Timecode),
                    "timecode_subframe": int(frame.TimecodeSubframe),
                    "marker_set_count": int(frame.nMarkerSets),
                    "rigid_body_count": int(frame.nRigidBodies),
                    "skeleton_count": int(frame.nSkeletons),
                    "unidentified_marker_count": int(frame.nOtherMarkers),
                },
                "markers": marker_rows,
                "rigid_bodies": rigid_rows,
                "rigid_markers": rigid_marker_rows,
                "skeleton_segments": skeleton_rows,
            }
            try:
                packet_queue.put_nowait(packet)
                with state_lock:
                    state["callback_frames"] += 1
            except queue.Full:
                with state_lock:
                    state["dropped_frames"] += 1
        except Exception:
            with state_lock:
                state["callback_errors"] += 1

    if args.start_delay:
        print(f"Waiting {args.start_delay:.1f} seconds before recording...")
        time.sleep(args.start_delay)
    started_perf_ns = time.perf_counter_ns()
    started_unix_ns = time.time_ns()
    events = [{"event": "capture_started", "perf_ns": started_perf_ns, "unix_ns": started_unix_ns, "note": ""}]
    writer.start()
    callback_result = client.PySetDataCallback(data_callback, None)
    if callback_result != 0:
        state["accepting"] = False
        packet_queue.put(sentinel)
        writer.join()
        print(f"[FAIL] PySetDataCallback failed with code {callback_result}", file=sys.stderr)
        return 2
    print(f"Recording to: {output}")
    print("Press Ctrl+C to stop.")
    try:
        if args.interactive_events:
            print("Interactive events: Enter=sync_event, text=note event, q=stop")
            while True:
                command = input().strip()
                now = {"perf_ns": time.perf_counter_ns(), "unix_ns": time.time_ns()}
                if command.lower() == "q":
                    events.append({"event": "stop_requested", **now, "note": "interactive q"})
                    break
                events.append({
                    "event": "sync_event" if not command else "user_event",
                    **now, "note": command,
                })
                print(f"Event saved: {events[-1]['event']} {command}")
        else:
            deadline = time.monotonic() + args.duration if args.duration > 0 else None
            last_printed = -1
            while deadline is None or time.monotonic() < deadline:
                time.sleep(0.1)
                elapsed = int((time.perf_counter_ns() - started_perf_ns) / 1e9)
                if elapsed != last_printed:
                    last_printed = elapsed
                    print(
                        f"  {elapsed:4d}s | frames={state['callback_frames']} "
                        f"queue_drops={state['dropped_frames']} callback_errors={state['callback_errors']}",
                        end="\r", flush=True,
                    )
    except KeyboardInterrupt:
        events.append({
            "event": "stop_requested", "perf_ns": time.perf_counter_ns(),
            "unix_ns": time.time_ns(), "note": "KeyboardInterrupt",
        })
    finally:
        state["accepting"] = False
    print()
    time.sleep(0.2)
    packet_queue.put(sentinel)
    writer.join()
    ended_perf_ns = time.perf_counter_ns()
    ended_unix_ns = time.time_ns()
    events.append({"event": "capture_completed", "perf_ns": ended_perf_ns, "unix_ns": ended_unix_ns, "note": ""})
    write_events(output / "events.csv", events)
    if writer.error:
        print(f"[FAIL] CSV writer failed: {writer.error}", file=sys.stderr)
        return 2

    marker_names_path = write_marker_names(
        output / "marker_names.txt", descriptions["marker_sets"],
        ["__record_no_markersets__"] if args.rigid_only else args.hand_markerset,
    )
    head_marker_ids = sorted(int(value) for value in state["observed_head_marker_ids"])
    calibration_dir = output.parent / "calibration"
    calibration_dir.mkdir(parents=True, exist_ok=True)
    head_definition_path = calibration_dir / "head_rigidbody_definition.json"
    existing_head_definition = None
    if head_definition_path.is_file():
        try:
            existing_head_definition = json.loads(head_definition_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError):
            existing_head_definition = None
    if not existing_head_definition or existing_head_definition.get("status") == "PLACEHOLDER_DO_NOT_USE":
        head_definition_path.write_text(
            json.dumps({
                "schema": "nokov_head_rigidbody_definition_v1",
                "status": "captured_definition_needs_extrinsic_calibration",
                "rigid_body_name": args.head_rigidbody,
                "marker_ids": head_marker_ids,
                "marker_names": [f"marker_id:{value}" for value in head_marker_ids],
                "marker_positions_head_mm": None,
                "minimum_marker_count": 3,
                "recommended_marker_count": 4,
                "source_capture": str(output),
                "notes": "Marker IDs came from live SDK frames; T_head_ego_base is still required.",
            }, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    observed_counts = {
        name: sorted(int(value) for value in values)
        for name, values in state["observed_marker_counts"].items()
    }
    metadata = {
        "schema": "nokov_hand24_capture_metadata_v1",
        "status": "complete" if writer.counts["frames"] > 0 and not state["callback_errors"] else "completed_with_warnings",
        "server": args.server,
        "sdk_version": list(version),
        "started_unix_ns": started_unix_ns,
        "ended_unix_ns": ended_unix_ns,
        "duration_s": (ended_perf_ns - started_perf_ns) / 1e9,
        "position_unit": "mm",
        "capture_mode": "rigid_only" if args.rigid_only else "hand_and_rigid",
        "selected_hand_markersets": (
            [] if args.rigid_only else (args.hand_markerset or "all")
        ),
        "expected_hand_markers": args.expected_hand_markers,
        "head_rigidbody_name": args.head_rigidbody,
        "frame_rate_hz_from_description": descriptions["frame_rate_hz"],
        "observed_marker_counts": observed_counts,
        "head_marker_ids": head_marker_ids,
        "rows": writer.counts,
        "callback_frames": state["callback_frames"],
        "queue_dropped_frames": state["dropped_frames"],
        "callback_errors": state["callback_errors"],
        "hardware_synchronized_with_ego": False,
        "raw_xingying_cap_recorded_separately": False,
        "files": {
            "frames": "nokov_frames.csv",
            "markers": "nokov_markers.csv",
            "rigid_bodies": "nokov_rigid_bodies.csv",
            "rigid_body_markers": "nokov_rigid_body_markers.csv",
            "skeleton_segments": "nokov_skeleton_segments.csv",
            "events": "events.csv",
            "descriptions": "asset_descriptions.json",
            "marker_names": marker_names_path.name,
        },
        "notes": [
            "SDK CSV is independent of XINGYING CAP/TRC/C3D recording.",
            "Set hardware_synchronized_with_ego and raw_xingying_cap_recorded_separately manually after verification.",
        ],
    }
    (output / "capture_metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    if writer.counts["frames"] == 0:
        print("[FAIL] no mocap frames were received", file=sys.stderr)
        return 2
    if state["dropped_frames"]:
        print("[WARN] queue drops occurred; reduce other load or increase --queue-size")
    if len(head_marker_ids) < 3:
        print("[WARN] fewer than three head rigid-body markers were observed")
    print(f"Capture complete: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
