#!/usr/bin/env python3
"""Verify ignored vendor/model assets declared by a reproducible profile."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
MANIFEST = PROJECT_DIR / "docs" / "external_assets_manifest.json"


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("linux-capture", "windows-capture", "linux-sync", "linux-full"),
        required=True,
    )
    parser.add_argument("--skip-hash", action="store_true")
    args = parser.parse_args()

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    profile = manifest["profiles"][args.profile]
    assets = manifest["assets"]
    failures = 0

    print(f"Profile: {args.profile}")
    print(profile["description"])
    if not profile["required_files"]:
        print("[OK] no ignored binary/model assets are required")

    for asset_id in profile["required_files"]:
        item = assets[asset_id]
        path = PROJECT_DIR / item["path"]
        if not path.is_file():
            failures += 1
            print(f"[MISSING] {item['path']}")
            if item.get("public_url"):
                print(f"          download: {item['public_url']}")
            else:
                print(f"          obtain: {item['source']}")
            continue
        expected_size = item.get("tested_size_bytes")
        if expected_size is not None and path.stat().st_size != expected_size:
            failures += 1
            print(f"[FAIL] {item['path']}: unexpected size {path.stat().st_size}")
            continue
        expected_hash = item.get("tested_sha256")
        if expected_hash and not args.skip_hash:
            actual = digest(path)
            if actual != expected_hash:
                failures += 1
                print(f"[FAIL] {item['path']}: SHA-256 mismatch")
                print(f"       expected {expected_hash}")
                print(f"       actual   {actual}")
                continue
        print(f"[OK] {item['path']}")

    for asset_id in profile.get("external_prerequisites", []):
        item = assets[asset_id]
        print(f"[MANUAL CHECK] {item['source']}")

    if failures:
        print(f"Result: FAIL ({failures} missing or invalid asset(s))")
        return 2
    print("Result: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
