#!/usr/bin/env python3
"""Dependency-light regression tests for the time-offset estimator."""

from __future__ import annotations

import unittest

import numpy as np

import synchronize_ego_imu_nokov as sync


class OffsetEstimatorTest(unittest.TestCase):
    def test_known_negative_offset(self) -> None:
        rate = 100.0
        time_s = np.arange(0.0, 30.0, 1.0 / rate)

        def pulses(value: np.ndarray) -> np.ndarray:
            return (
                1.2 * np.exp(-((value - 5.0) / 0.12) ** 2)
                + 0.7 * np.exp(-((value - 9.3) / 0.18) ** 2)
                + 1.6 * np.exp(-((value - 15.8) / 0.09) ** 2)
                + 0.9 * np.exp(-((value - 23.1) / 0.22) ** 2)
            )

        true_b = -2.37
        ego = pulses(time_s)
        nokov = pulses(time_s - true_b)
        estimated_b, correlation, *_ = sync.estimate_offset(
            ego, nokov, rate, max_offset_s=8.0, min_overlap_s=8.0
        )
        self.assertAlmostEqual(estimated_b, true_b, places=2)
        self.assertGreater(correlation, 0.99)


if __name__ == "__main__":
    unittest.main()
