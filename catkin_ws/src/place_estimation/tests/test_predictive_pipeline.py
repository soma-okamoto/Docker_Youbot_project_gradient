"""ROS-free callback integration: prediction snapshots survive later feedback."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

import test_bottle_identity as identity
import test_predictive_metrics as scoring


class PredictivePipelineTests(unittest.TestCase):
    def setUp(self):
        self.h = identity.BottleIdentityTests()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.h.settings.update({"~publish_predictive_snapshot": True})
        self.h.enable_guard()

    def prior(self, bottle=7, current=1.1, tf=1.):
        return self.h.guarded_prior(bottle, current, tf)

    def test_startup_reference_and_learning_count_are_atomic(self):
        context = self.prior()
        self.assertTrue(context.fixed_prior_available)
        self.assertEqual(context.learning_sample_count, 0)
        self.assertEqual(context.ci_weight_current, context.fixed_ci_weight_current)
        self.assertAlmostEqual(context.position.x, context.fixed_position.x)
        np.testing.assert_allclose(context.covariance, context.fixed_covariance)

    def test_baseline_stays_fixed_and_previous_snapshot_survives_learning(self):
        first = self.prior()
        original = copy.deepcopy(first)
        self.h.fusion._pred_context_cb(first)
        self.h.fusion._yolo_array_cb(identity.Array(
            [.9, 0., 0.] + (np.eye(3) * .0001).reshape(-1).tolist()
        ))
        used = self.h.fusion.used_observation_pub.publish.call_args.args[0]
        self.h.prior._used_observation_callback(used)
        self.assertEqual(self.h.prior.learning_count, 1)
        # A second operation uses learned bias but the startup reference remains TF=1.
        second = self.prior(bottle=8)
        self.assertEqual(second.learning_sample_count, 1)
        self.assertAlmostEqual(second.fixed_position.x, 1.)
        self.assertAlmostEqual(second.position.x, .9)
        self.assertAlmostEqual(first.position.x, original.position.x)
        self.assertEqual(first.learning_sample_count, 0)
        snapshot = json.loads(self.h.fusion.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot["learning_sample_count_before_trial"], 0)
        result = scoring.metrics.evaluate_trial(snapshot)
        self.assertEqual(result["scores"]["realsense"]["trial_score"]["delta_nll"], 0.)

    def test_prior_rejected_observation_is_still_scored_without_feedback(self):
        self.h.fusion._pred_context_cb(self.prior(current=1., tf=1.))
        self.h.fusion._yolo_array_cb(identity.Array(
            [5., 0., 0.] + (np.eye(3) * .0001).reshape(-1).tolist()
        ))
        self.assertEqual(self.h.fusion.status_pub.publish.call_args.args[0].data,
                         "no_valid_observation")
        self.h.fusion.used_observation_pub.publish.assert_not_called()
        snapshot = json.loads(self.h.fusion.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot["sensors"]["realsense"]["algorithm_selected_index"], -1)
        result = scoring.metrics.evaluate_trial(snapshot)
        self.assertIsNotNone(result["scores"]["realsense"]["trial_score"])

    def test_covariance_learning_does_not_change_fixed_reference(self):
        startup = self.h.prior.initial_cov_tf.copy()
        for bottle, observed_x in ((7, .9), (8, .8)):
            self.h.fusion._pred_context_cb(self.prior(bottle=bottle))
            self.h.fusion._yolo_array_cb(identity.Array(
                [observed_x, 0., 0.] + (np.eye(3) * .0001).reshape(-1).tolist()
            ))
            self.h.prior._used_observation_callback(
                self.h.fusion.used_observation_pub.publish.call_args.args[0]
            )
        context = self.prior(bottle=9)
        self.assertEqual(context.learning_sample_count, 2)
        self.assertAlmostEqual(context.fixed_position.x, 1.)
        np.testing.assert_allclose(np.array(context.fixed_covariance).reshape(3, 3), startup)
        self.assertFalse(np.allclose(context.covariance, context.fixed_covariance))

    def test_new_prior_keeps_previous_operation_snapshot_and_bottle(self):
        self.h.fusion.enable_meta = True
        first = self.prior(bottle=7)
        second = self.prior(bottle=8)
        self.h.fusion._pred_context_cb(first)
        self.h.fusion._pred_context_cb(second)
        snapshot = json.loads(self.h.fusion.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot["operation_id"], 1)
        self.assertEqual(snapshot["bottle_id"], 7)
        self.assertEqual(self.h.fusion.prediction_snapshot["operation_id"], 2)
        self.assertEqual(self.h.fusion.prediction_snapshot["bottle_id"], 8)

    def test_snapshot_publication_failure_does_not_change_place_or_reset(self):
        self.h.fusion.predictive_snapshot_pub.publish.side_effect = RuntimeError("diagnostic failure")
        self.h.fusion._pred_context_cb(self.prior())
        self.h.fusion._yolo_array_cb(identity.Array(
            [1., 0., 0.] + (np.eye(3) * .0001).reshape(-1).tolist()
        ))
        self.h.fusion.place_pub.publish.assert_called_once()
        self.assertFalse(self.h.fusion.active)
        self.assertIsNone(self.h.fusion.prediction_snapshot)

    def test_evaluation_node_deduplicates_atomic_operations(self):
        path = Path(__file__).resolve().parents[1] / "evaluation/predictive_evaluation_node.py"
        spec = importlib.util.spec_from_file_location("predictive_node_under_test", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"predictive_metrics": scoring.metrics}):
            spec.loader.exec_module(module)
        node = module.PredictiveEvaluationNode()
        payload = json.dumps(scoring.snapshot())
        node._callback(NS(data=payload))
        node._callback(NS(data=payload))
        self.assertEqual(node.history.trial_count, 1)
        node.metrics_pub.publish.assert_called_once()
        invalid = scoring.snapshot(2)
        invalid["frame_id"] = "map"
        node._callback(NS(data=json.dumps(invalid)))
        self.assertEqual(node.history.trial_count, 1)


if __name__ == "__main__":
    unittest.main()
