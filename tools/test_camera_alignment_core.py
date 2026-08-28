#!/usr/bin/env python3
"""Regression tests for NOKOV/WiLoR camera-overlay geometry."""

from __future__ import annotations

import unittest

import numpy as np

import render_nokov_wilor_camera_alignment as overlay
import refine_nokov_marker_camera_alignment as marker_refinement


class CameraAlignmentCoreTest(unittest.TestCase):
    def test_double_sphere_optical_axis_projects_to_principal_point(self) -> None:
        camera = {
            "image_size": [640, 480],
            "distortion": [500.0, 505.0, 321.0, 239.0, -0.003, 0.57],
        }
        pixels, valid = overlay.project_double_sphere(
            camera, np.asarray([[0.0, 0.0, 1.0]])
        )
        self.assertTrue(valid[0])
        np.testing.assert_allclose(pixels[0], [321.0, 239.0], atol=1e-12)

    def test_marker_interpolation_rejects_a_large_gap(self) -> None:
        times = np.asarray([0.0, 0.01, 0.20, 0.21])
        points = np.zeros((4, 2, 24, 3), dtype=np.float64)
        points[:, :, :, 0] = times[:, None, None] * 100.0
        valid = np.ones((4, 2, 24), dtype=bool)
        result, result_valid, _ = overlay.interpolate_markers(
            times, points, valid, np.asarray([0.005, 0.10, 0.205]), 0.05
        )
        self.assertTrue(np.all(result_valid[0]))
        self.assertFalse(np.any(result_valid[1]))
        self.assertTrue(np.all(result_valid[2]))
        self.assertAlmostEqual(result[0, 0, 0, 0], 0.5)

    def test_centroid_cost_is_independent_of_hand_order(self) -> None:
        first = {
            0: np.asarray([[10.0, 20.0], [12.0, 20.0]]),
            1: np.asarray([[100.0, 30.0], [102.0, 30.0]]),
        }
        swapped = {
            0: np.asarray([[100.0, 30.0], [102.0, 30.0]]),
            1: np.asarray([[10.0, 20.0], [12.0, 20.0]]),
        }
        self.assertAlmostEqual(overlay.coarse_centroid_cost(first, swapped), 0.0)

    def test_skeleton_interpolation_preserves_all_segment_visibility(self) -> None:
        times = np.asarray([0.0, 0.01])
        points = np.zeros((2, 2, 26, 3), dtype=np.float64)
        points[1, :, :, 0] = 10.0
        valid = np.ones((2, 2, 26), dtype=bool)
        result, result_valid, _ = marker_refinement.interpolate_skeleton(
            times, points, valid, np.asarray([0.005]), 0.05
        )
        self.assertTrue(np.all(result_valid))
        np.testing.assert_allclose(result[0, :, :, 0], 5.0)


if __name__ == "__main__":
    unittest.main()
