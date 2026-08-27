#!/usr/bin/env python3
"""Export DAS-Ego /robot0/vio/eef_pose from an MCAP to TUM pose text."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/robot0/vio/eef_pose")
    return parser.parse_args()


def timestamp_ns(value: Any, fallback: int) -> int:
    if value is None:
        return fallback
    if isinstance(value, (int, float)):
        return int(value)
    if hasattr(value, "seconds") and hasattr(value, "nanos"):
        return int(value.seconds) * 1_000_000_000 + int(value.nanos)
    return int(value)


def main() -> int:
    args = parse_args()
    try:
        from mcap.reader import make_reader
        from mcap_protobuf.decoder import DecoderFactory
    except ImportError as exc:
        raise SystemExit(
            "MCAP dependencies are required: "
            "python -m pip install -r tools/requirements-sync.txt"
        ) from exc

    rows: list[tuple[int, float, float, float, float, float, float, float]] = []
    frame_ids: set[str] = set()
    with args.mcap.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _schema, _channel, message, decoded in reader.iter_decoded_messages(
            topics=[args.topic]
        ):
            pose = getattr(decoded, "pose", None)
            if pose is None:
                continue
            position = pose.position
            orientation = pose.orientation
            header = getattr(decoded, "header", None)
            stamp = timestamp_ns(
                getattr(header, "timestamp", None), int(message.log_time)
            )
            frame_ids.add(str(getattr(decoded, "frame_id", "")))
            rows.append(
                (
                    stamp,
                    float(position.x),
                    float(position.y),
                    float(position.z),
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                )
            )
    if len(rows) < 20:
        raise RuntimeError(f"only {len(rows)} poses found on {args.topic!r}")
    rows.sort(key=lambda row: row[0])
    deduplicated = []
    last_stamp = None
    for row in rows:
        if row[0] != last_stamp:
            deduplicated.append(row)
            last_stamp = row[0]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in deduplicated:
            stream.write(
                f"{row[0] * 1e-9:.9f} "
                + " ".join(f"{value:.9f}" for value in row[1:])
                + "\n"
            )
    print(f"Exported {len(deduplicated)} poses from {args.topic}")
    print(f"frame_id values: {sorted(frame_ids)!r}")
    print(f"Wrote {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
