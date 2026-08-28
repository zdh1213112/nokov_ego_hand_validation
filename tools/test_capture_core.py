#!/usr/bin/env python3
"""Small regression tests for SDK numeric-validity handling."""

from capture_nokov_hand24 import numeric_valid


def main() -> int:
    assert numeric_valid((12.0, -34.0, 56.0))
    assert not numeric_valid((9_999_999.0, 9_999_999.0, 9_999_999.0))
    assert not numeric_valid((float("nan"), 1.0, 2.0))
    assert numeric_valid((100.0, 200.0, 300.0, 0.0, 0.0, 0.0, 1.0), quaternion=True)
    assert not numeric_valid(
        (9_999_999.0, 9_999_999.0, 9_999_999.0, 9_999_999.0, 0.0, 0.0, 1.0),
        quaternion=True,
    )
    print("capture core tests: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
