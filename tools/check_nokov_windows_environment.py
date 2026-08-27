#!/usr/bin/env python3
"""Check the Python, NOKOV SDK and synchronization dependencies on Windows."""

from __future__ import annotations

import importlib
import platform
import struct
import sys


def main() -> int:
    failures: list[str] = []
    print(f"Python: {sys.version.split()[0]}")
    print(f"Executable: {sys.executable}")
    print(f"Platform: {platform.platform()}")
    bits = struct.calcsize("P") * 8
    print(f"Python architecture: {bits}-bit")
    if bits != 64:
        failures.append("NOKOV deployment requires 64-bit Python")

    for module_name in ("numpy", "mcap", "mcap_protobuf", "matplotlib"):
        try:
            module = importlib.import_module(module_name)
            version = getattr(module, "__version__", "installed")
            print(f"[OK] {module_name}: {version}")
        except Exception as exc:
            failures.append(f"cannot import {module_name}: {exc}")

    try:
        sdk = importlib.import_module("nokov.nokovsdk")
        client = sdk.PySDKClient()
        version = ".".join(str(value) for value in client.PyNokovVersion())
        print(f"[OK] NOKOV SDK native load: {version}")
    except Exception as exc:
        failures.append(
            "cannot load NOKOV SDK; install the official nokovpy wheel and, "
            f"on Windows, the official VC++ x64 runtime if required: {exc}"
        )

    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}", file=sys.stderr)
        return 2
    print("NOKOV Windows environment: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
