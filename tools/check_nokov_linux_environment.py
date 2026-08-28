#!/usr/bin/env python3
"""Check a Linux XINGYING/NOKOV acquisition environment and live assets."""

from __future__ import annotations

import argparse
import json
import platform
import struct
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
DEFAULT_WHEEL = (
    PROJECT_DIR
    / "vendor"
    / "nokov_python_sdk"
    / "nokovpy-3.0.1-py3-none-any.whl"
)

# The recorder contains the single SDK loader used on both Linux and Windows.
sys.path.insert(0, str(SCRIPT_DIR))
from capture_nokov_hand24 import (  # noqa: E402
    load_sdk,
    read_descriptions,
    serializable_descriptions,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="10.1.1.198")
    parser.add_argument("--sdk-wheel", type=Path, default=DEFAULT_WHEEL)
    parser.add_argument(
        "--xingying-binary",
        type=Path,
        default=Path("/usr/local/XINGYING/bin/XINGYING"),
    )
    parser.add_argument("--skip-live", action="store_true")
    parser.add_argument("--head-rigidbody")
    parser.add_argument("--hand-markerset", action="append", default=[])
    parser.add_argument("--expected-hand-markers", type=int, default=24)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    failures: list[str] = []
    warnings: list[str] = []

    print(f"Python: {sys.version.split()[0]}")
    print(f"Executable: {sys.executable}")
    print(f"Platform: {platform.platform()}")
    bits = struct.calcsize("P") * 8
    print(f"Python architecture: {bits}-bit")
    if platform.system() != "Linux":
        failures.append("this entry point is for Linux")
    if bits != 64:
        failures.append("NOKOV acquisition requires 64-bit Python")
    if sys.version_info < (3, 10):
        failures.append("Python 3.10 or newer is required")
    elif sys.version_info[:2] not in ((3, 10), (3, 11)):
        warnings.append("the delivered SDK was field-tested with Python 3.10/3.11")

    if args.xingying_binary.is_file():
        print(f"[OK] XINGYING binary: {args.xingying_binary}")
    else:
        failures.append(f"XINGYING binary not found: {args.xingying_binary}")

    try:
        sdk = load_sdk(args.sdk_wheel)
        client = sdk.PySDKClient()
        version = ".".join(str(value) for value in client.PyNokovVersion())
        print(f"[OK] NOKOV SDK native load: {version}")
    except Exception as exc:
        failures.append(f"cannot load NOKOV Linux SDK: {exc}")
        sdk = None
        client = None

    if not args.skip_live and client is not None and sdk is not None:
        result = client.Initialize(args.server.encode("utf-8"))
        if result != 0:
            failures.append(
                f"SDK connection to {args.server} failed with code {result}; "
                "start XINGYING and enable SDK/Data Adapter broadcasting"
            )
        else:
            print(f"[OK] connected to XINGYING/NOKOV: {args.server}")
            try:
                descriptions = read_descriptions(client, sdk)
            except Exception as exc:
                failures.append(f"cannot read live asset descriptions: {exc}")
            else:
                print(json.dumps(
                    serializable_descriptions(descriptions),
                    ensure_ascii=False,
                    indent=2,
                ))
                rigid_names = set(descriptions["rigid_bodies"].values())
                if args.head_rigidbody and args.head_rigidbody not in rigid_names:
                    failures.append(
                        f"head rigid body {args.head_rigidbody!r} is not live; "
                        f"available={sorted(rigid_names)!r}"
                    )
                for name in args.hand_markerset:
                    markers = descriptions["marker_sets"].get(name)
                    if markers is None:
                        failures.append(
                            f"hand MarkerSet {name!r} is not live; "
                            f"available={sorted(descriptions['marker_sets'])!r}"
                        )
                    elif len(markers) != args.expected_hand_markers:
                        failures.append(
                            f"hand MarkerSet {name!r} has {len(markers)} markers; "
                            f"expected {args.expected_hand_markers}"
                        )

    for warning in warnings:
        print(f"[WARN] {warning}")
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}", file=sys.stderr)
        return 2
    mode = "SDK load only" if args.skip_live else "live connection"
    print(f"NOKOV Linux environment: OK ({mode})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
