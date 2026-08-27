#!/usr/bin/env python3
"""Synthetic regression test for AX=XB spatial hand-eye calibration."""

from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

import calibrate_ego_vio_nokov as calibration


def make_transform(rotvec: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    result[:3, 3] = translation
    return result


class SpatialHandEyeTest(unittest.TestCase):
    def test_recovers_known_rigid_to_ego_transform(self) -> None:
        known_x = make_transform(
            np.array([0.23, -0.31, 0.17]), np.array([0.06, -0.04, 0.09])
        )
        known_y = make_transform(
            np.array([-0.15, 0.08, 0.42]), np.array([0.7, -0.2, 1.1])
        )
        ego_poses = []
        mocap_poses = []
        for value in np.linspace(0.0, 4.0, 240):
            ego = make_transform(
                np.array(
                    [
                        0.45 * np.sin(value),
                        0.38 * np.cos(0.7 * value),
                        0.32 * np.sin(1.3 * value),
                    ]
                ),
                np.array(
                    [
                        0.35 * np.sin(0.6 * value),
                        0.25 * np.cos(0.9 * value),
                        0.18 * np.sin(1.1 * value),
                    ]
                ),
            )
            mocap = known_y @ ego @ calibration.inverse(known_x)
            ego_poses.append(ego)
            mocap_poses.append(mocap)
        ego_array = np.asarray(ego_poses)
        mocap_array = np.asarray(mocap_poses)
        motions_m, motions_e = calibration.relative_pairs(
            mocap_array,
            ego_array,
            0,
            len(ego_array),
            stride=5,
            minimum_rotation_rad=np.deg2rad(3.0),
        )
        estimated_x, quality = calibration.solve_x(motions_m, motions_e)
        rotation_error = Rotation.from_matrix(
            estimated_x[:3, :3].T @ known_x[:3, :3]
        ).magnitude()
        translation_error = np.linalg.norm(
            estimated_x[:3, 3] - known_x[:3, 3]
        )
        self.assertLess(np.degrees(rotation_error), 1e-5)
        self.assertLess(translation_error, 1e-7)
        self.assertLess(quality["relative_rotation_error_median_deg"], 1e-6)


if __name__ == "__main__":
    unittest.main()
