"""Keep sparse posterior publications aligned with their original priors."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_bottle_identity as identity
import test_gaussian_kl as gaussian
import test_kl_trend as trend


class KLOperationMatchingTests(unittest.TestCase):
    def setUp(self):
        self.h = identity.BottleIdentityTests()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.h.ros.on_shutdown = Mock()
        path = Path(__file__).resolve().parents[1] / 'evaluation/kl_evaluation_node.py'
        spec = importlib.util.spec_from_file_location('kl_node_under_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'gaussian_kl': gaussian.module, 'kl_trend': trend.kl_trend}):
            spec.loader.exec_module(module)
        self.node = module.KLEvaluationNode()

    def distribution(self, kind, seq, stamp, x=1.):
        pose = identity.PoseStamped()
        pose.header.seq = seq
        pose.header.stamp = identity.Time(stamp)
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = x
        self.node._pose_callback(kind, pose)
        self.node._covariance_callback(kind, identity.Array((np.eye(3)*.01).reshape(-1).tolist()))

    def test_cancelled_prior_does_not_steal_next_posterior(self):
        self.distribution('prior', 1, 100, x=10.)
        self.distribution('prior', 2, 200, x=2.)
        self.distribution('posterior', 1, 200, x=2.)
        result = self.node.metrics_pub.publish.call_args.args[0].data
        self.assertEqual(result[0], 2.)
        self.assertAlmostEqual(result[1], 0.)
        self.assertEqual(self.node.running_count, 1)

    def test_posterior_can_arrive_before_its_prior(self):
        self.distribution('prior', 1, 100, x=10.)
        self.distribution('posterior', 1, 200, x=2.)
        self.node.metrics_pub.publish.assert_not_called()
        self.distribution('prior', 2, 200, x=2.)
        self.assertEqual(self.node.metrics_pub.publish.call_args.args[0].data[0], 2.)

    def test_unknown_timestamp_cannot_fall_back_to_equal_sequence(self):
        self.distribution('prior', 1, 100)
        self.distribution('posterior', 1, 999)
        self.node.metrics_pub.publish.assert_not_called()

    def test_cancelled_priors_are_bounded_in_pending_state(self):
        self.node.max_pending_operations = 2
        for i in range(1, 5):
            self.distribution('prior', i, i * 100)
        self.assertEqual(len(self.node.distributions['prior']), 2)
        self.distribution('posterior', 1, 400)
        self.assertEqual(self.node.metrics_pub.publish.call_args.args[0].data[0], 4.)


if __name__ == '__main__':
    unittest.main()
