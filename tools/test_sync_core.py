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

    def test_interpolates_90_hz_pose_at_200_hz_timestamps(self) -> None:
        nokov_time = np.arange(0.0, 1.0 + 1e-9, 1.0 / 90.0)
        ego_time = np.arange(0.0, 1.0, 1.0 / 200.0)
        frames = np.arange(len(nokov_time), dtype=np.int64)
        positions = np.column_stack(
            [100.0 * nokov_time, -50.0 * nokov_time, 20.0 * nokov_time]
        )
        half_angle = 0.5 * (np.pi / 2.0) * nokov_time
        quaternions = np.column_stack(
            [
                np.zeros(len(nokov_time)),
                np.zeros(len(nokov_time)),
                np.sin(half_angle),
                np.cos(half_angle),
            ]
        )

        result = sync.interpolate_rigid_poses(
            nokov_time,
            frames,
            positions,
            quaternions,
            ego_time,
            max_gap_s=0.02,
        )

        self.assertTrue(np.all(result["valid"]))
        np.testing.assert_allclose(result["position_mm"][:, 0], 100.0 * ego_time)
        np.testing.assert_allclose(
            np.linalg.norm(result["quaternion_xyzw"], axis=1), 1.0, atol=1e-12
        )
        midpoint = int(np.argmin(np.abs(ego_time - 0.5)))
        self.assertAlmostEqual(
            result["quaternion_xyzw"][midpoint, 2], np.sin(np.pi / 8.0), places=10
        )

    def test_rejects_interpolation_across_pose_dropout(self) -> None:
        pose_time = np.asarray([0.0, 0.01, 0.02, 0.20, 0.21])
        frames = np.arange(len(pose_time), dtype=np.int64)
        positions = np.zeros((len(pose_time), 3))
        quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (len(pose_time), 1))
        result = sync.interpolate_rigid_poses(
            pose_time,
            frames,
            positions,
            quaternions,
            np.asarray([0.015, 0.10, 0.205]),
            max_gap_s=0.05,
        )
        np.testing.assert_array_equal(result["valid"], [True, False, True])


if __name__ == "__main__":
    unittest.main()
