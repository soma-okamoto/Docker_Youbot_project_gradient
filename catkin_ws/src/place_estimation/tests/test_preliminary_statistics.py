"""ROS-free tests for preliminary-experiment statistics."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np


path = Path(__file__).resolve().parents[1] / "analysis/preliminary_statistics.py"
spec = importlib.util.spec_from_file_location("preliminary_statistics", path)
statistics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(statistics)


class PreliminaryStatisticsTests(unittest.TestCase):
    def test_descriptive_statistics_ignores_nonfinite_values(self):
        result = statistics.descriptive_statistics([1.0, 2.0, np.nan, 3.0])
        self.assertEqual(result["count"], 3)
        self.assertAlmostEqual(result["mean"], 2.0)
        self.assertAlmostEqual(result["p50"], 2.0)

    def test_vector_statistics_returns_sample_covariance(self):
        result = statistics.vector_statistics([[0, 0, 0], [2, 0, 0]])
        self.assertEqual(result["count"], 2)
        np.testing.assert_allclose(result["mean"], [1, 0, 0])
        self.assertAlmostEqual(result["sample_covariance"][0][0], 2.0)

    def test_learning_stability_candidate(self):
        history = [
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.101, 0.0, 0.0],
            [0.1015, 0.0, 0.0],
            [0.1017, 0.0, 0.0],
        ]
        self.assertEqual(
            statistics.learning_stability_candidate(
                history, tolerance_m=0.002, stable_updates=2
            ),
            3,
        )

    def test_report_produces_candidates_and_floor_warning(self):
        trials = []
        for operation_id, distance in enumerate((1.0, 2.0, 3.0), start=1):
            trials.append(
                {
                    "operation_id": operation_id,
                    "selected_yolo_d2": distance,
                    "current_x": 1.1,
                    "current_y": 0.0,
                    "current_z": 0.0,
                    "tf_x": 0.9,
                    "tf_y": 0.0,
                    "tf_z": 0.0,
                    "used_x": 1.0,
                    "used_y": 0.0,
                    "used_z": 0.0,
                    "_yolo_principal_stds": [0.02],
                }
            )
        learning = [
            {
                "operation_id": index,
                "sample_count": index,
                "bias_current": [0.1, 0.0, 0.0],
                "bias_tf": [-0.1, 0.0, 0.0],
                "covariance_current": (np.eye(3) * 1.0e-8).tolist(),
                "covariance_tf": (np.eye(3) * 1.0e-8).tolist(),
            }
            for index in range(1, 5)
        ]
        result = statistics.build_parameter_report(
            trials, learning, stable_updates=2
        )
        self.assertEqual(result["trial_count"], 3)
        self.assertEqual(
            result["parameter_candidates"]["gate_yolo_p95_diagnostic"],
            2.9,
        )
        self.assertGreater(result["min_variance_floor_hit_count"], 0)
        self.assertEqual(result["data_completeness"]["P_current"], 3)
        self.assertEqual(result["data_completeness"]["prior_distribution"], 0)
        self.assertTrue(any("incomplete" in item for item in result["warnings"]))
        self.assertTrue(any("min_variance" in item for item in result["warnings"]))


if __name__ == "__main__":
    unittest.main()
