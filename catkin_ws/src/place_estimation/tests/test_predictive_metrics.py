"""Predictive scoring tests with analytic answers and missing/ambiguous data."""
import copy
import importlib.util
from pathlib import Path
import unittest

import numpy as np


path = Path(__file__).resolve().parents[1] / "evaluation/predictive_metrics.py"
spec = importlib.util.spec_from_file_location("predictive_metrics_under_test", path)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def snapshot(operation_id=1):
    prior = {"mean": [0., 0., 0.], "covariance": np.eye(3).reshape(-1).tolist()}
    candidate = {"index": 0, "mean": [0.1, 0., 0.],
                 "covariance": (np.eye(3) * .01).reshape(-1).tolist()}
    sensor = {"enabled": True, "received": True, "quality_candidates": [candidate],
              "algorithm_selected_index": 0, "selected_for_feedback": True}
    return {"schema_version": 1, "operation_id": operation_id, "bottle_id": 7,
            "frame_id": "base_footprint", "prior_stamp_ns": operation_id * 100,
            "learning_sample_count_before_trial": 0, "observation_status": "prior_yolo_meta_fused",
            "priors": {"adaptive": copy.deepcopy(prior), "fixed": copy.deepcopy(prior)},
            "sensors": {"meta": copy.deepcopy(sensor), "realsense": copy.deepcopy(sensor)}}


class PredictiveMetricsTests(unittest.TestCase):
    def test_gaussian_formula_includes_sensor_covariance(self):
        score = metrics.gaussian_observation_score(
            [0., 0., 0.], np.diag([1., 2., 3.]), [1., 2., 3.], np.eye(3)
        )
        expected_d2 = .5 + 4. / 3. + 9. / 4.
        expected_nll = .5 * (3. * np.log(2. * np.pi) + np.log(24.) + expected_d2)
        self.assertAlmostEqual(score["mahalanobis_squared"], expected_d2)
        self.assertAlmostEqual(score["nll"], expected_nll)
        self.assertAlmostEqual(score["squared_position_error_m2"], 14.)
        self.assertTrue(score["inside_95_predictive_ellipsoid"])

    def test_negative_nll_is_retained(self):
        score = metrics.gaussian_observation_score(
            [0., 0., 0.], np.eye(3) * 1.e-4, [0., 0., 0.], np.eye(3) * 1.e-4
        )
        self.assertLess(score["nll"], 0.)

    def test_identical_models_have_zero_paired_difference(self):
        result = metrics.evaluate_trial(snapshot())
        for sensor in metrics.SENSORS:
            self.assertEqual(result["scores"][sensor]["trial_score"]["delta_nll"], 0.)

    def test_positive_difference_favors_better_prediction(self):
        data = snapshot()
        data["priors"]["adaptive"]["mean"] = [.1, 0., 0.]
        result = metrics.evaluate_trial(data)
        self.assertGreater(result["scores"]["meta"]["trial_score"]["delta_nll"], 0.)
        self.assertEqual(data["priors"]["fixed"]["mean"], [0., 0., 0.])

    def test_correlated_covariance_matches_direct_closed_form(self):
        covariance = np.array([[.04, .01, 0.], [.01, .03, .005], [0., .005, .02]])
        residual = np.array([.1, -.2, .3])
        score = metrics.gaussian_observation_score(
            np.zeros(3), covariance, residual, covariance / 2.
        )
        expected = residual @ np.linalg.solve(covariance * 1.5, residual)
        self.assertAlmostEqual(score["mahalanobis_squared"], expected)

    def test_invalid_covariance_is_rejected_and_floor_is_reported(self):
        for invalid in (np.diag([-1., 1., 1.]), np.eye(3) * np.nan):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                metrics.gaussian_observation_score([0., 0., 0.], invalid, [0., 0., 0.], np.eye(3))
        score = metrics.gaussian_observation_score(
            [0., 0., 0.], np.zeros((3, 3)), [0., 0., 0.], np.eye(3) * 1.e-8
        )
        self.assertTrue(score["covariance_regularized"])
        self.assertEqual(score["prior_floor_eigenvalue_count"], 3)
        self.assertTrue(np.isfinite(score["nll"]))

    def test_ambiguous_candidates_are_not_selected_by_prediction(self):
        data = snapshot()
        second = copy.deepcopy(data["sensors"]["meta"]["quality_candidates"][0])
        second.update(index=1, mean=[8., 0., 0.])
        data["sensors"]["meta"]["quality_candidates"].append(second)
        result = metrics.evaluate_trial(data)
        meta = result["scores"]["meta"]
        self.assertEqual(meta["status"], "ambiguous_candidates")
        self.assertIsNone(meta["trial_score"])
        self.assertEqual(len(meta["candidate_scores"]), 2)
        self.assertEqual(meta["selected_diagnostic"]["index"], 0)

    def test_prior_gate_rejection_does_not_remove_unique_quality_score(self):
        data = snapshot()
        data["observation_status"] = "no_valid_observation"
        for sensor in metrics.SENSORS:
            data["sensors"][sensor]["algorithm_selected_index"] = -1
            data["sensors"][sensor]["selected_for_feedback"] = False
        result = metrics.evaluate_trial(data)
        self.assertIsNotNone(result["scores"]["meta"]["trial_score"])
        self.assertIsNone(result["scores"]["meta"]["selected_diagnostic"])

    def test_missing_sensor_is_not_zero_scored_or_pooled(self):
        data = snapshot()
        data["sensors"]["meta"].update(received=False, quality_candidates=[])
        result = metrics.evaluate_trial(data)
        history = metrics.PredictiveHistory()
        summary = history.add(result)
        self.assertIsNone(result["scores"]["meta"]["trial_score"])
        self.assertIsNone(summary["sensors"]["meta"]["mean_delta_nll"])
        self.assertEqual(summary["sensors"]["meta"]["scored_trials"], 0)
        self.assertEqual(summary["sensors"]["realsense"]["scored_trials"], 1)

    def test_missing_baseline_and_invalid_prediction_do_not_fake_scores(self):
        data = snapshot()
        del data["priors"]["fixed"]
        self.assertEqual(metrics.evaluate_trial(data)["scores"]["meta"]["status"],
                         "missing_prediction_snapshot")
        data = snapshot()
        data["priors"]["adaptive"]["covariance"][0] = -1.
        result = metrics.evaluate_trial(data)
        self.assertEqual(result["scores"]["meta"]["status"], "invalid_predictive_distribution")
        self.assertIsNone(result["scores"]["meta"]["trial_score"])

    def test_summary_averages_paired_differences_and_bounds_window(self):
        history = metrics.PredictiveHistory(window_size=2)
        differences = []
        for operation, mean in enumerate((.02, .04, .08), 1):
            data = snapshot(operation)
            data["priors"]["adaptive"]["mean"] = [mean, 0., 0.]
            result = metrics.evaluate_trial(data)
            differences.append(result["scores"]["meta"]["trial_score"]["delta_nll"])
            summary = history.add(result)
        self.assertAlmostEqual(summary["sensors"]["meta"]["mean_delta_nll"], np.mean(differences))
        self.assertAlmostEqual(summary["sensors"]["meta"]["recent_mean_delta_nll"], np.mean(differences[-2:]))
        self.assertNotIn("adaptation_state", summary)


if __name__ == "__main__":
    unittest.main()
