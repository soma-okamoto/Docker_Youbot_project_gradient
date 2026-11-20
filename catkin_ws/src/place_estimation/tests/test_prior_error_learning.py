"""ROS-free tests for sequential operation-error learning."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


def load_node():
    rospy = ModuleType('rospy')
    rospy.Publisher = Mock(side_effect=lambda *args, **kwargs: Mock())
    for name in ('Subscriber', 'loginfo', 'logwarn', 'logerr',
                 'logerr_throttle', 'logwarn_throttle'):
        setattr(rospy, name, Mock())
    geometry = ModuleType('geometry_msgs.msg')
    geometry.PoseStamped = SimpleNamespace
    geometry.PoseWithCovarianceStamped = SimpleNamespace
    std = ModuleType('std_msgs.msg')
    std.Float32MultiArray = SimpleNamespace
    std.Float64MultiArray = SimpleNamespace
    std.Int32 = SimpleNamespace
    std.MultiArrayDimension = SimpleNamespace
    modules = {'rospy': rospy, 'geometry_msgs': ModuleType('geometry_msgs'),
               'geometry_msgs.msg': geometry, 'std_msgs': ModuleType('std_msgs'),
               'std_msgs.msg': std}
    path = Path(__file__).resolve().parents[1] / 'scripts/prior_distribution_node.py'
    spec = importlib.util.spec_from_file_location('prior_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.PriorDistributionNode, rospy


Node, rospy = load_node()


def used_observation(seq, position, covariance, frame='base_footprint'):
    covariance_6d = np.zeros((6, 6), dtype=float)
    covariance_6d[:3, :3] = covariance
    return SimpleNamespace(
        header=SimpleNamespace(seq=seq, stamp=seq, frame_id=frame),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=position[0], y=position[1], z=position[2]
                )
            ),
            covariance=covariance_6d.reshape(-1).tolist(),
        ),
    )


class ErrorLearningTests(unittest.TestCase):
    def node(self, **params):
        settings = {
            '~expected_frame': 'base_footprint',
            '~ci_weight_mode': 'fixed',
        }
        settings.update({'~' + key: value for key, value in params.items()})
        rospy.get_param = lambda name, default=None: settings.get(name, default)
        rospy.has_param = lambda name: name in settings
        return Node()

    def test_bias_and_covariance_are_updated_from_used_observations(self):
        node = self.node(learning_min_samples=2, covariance_initial_weight=0.)
        observation_covariance = np.eye(3) * 1.e-4

        node.pending_learning_samples[1] = (
            np.array([1.1, 0., 0.]), np.array([.9, 0., 0.])
        )
        node.pending_learning_stamps[1] = 1
        node._used_observation_callback(
            used_observation(1, [1., 0., 0.], observation_covariance)
        )
        np.testing.assert_allclose(node.bias_current, [.1, 0., 0.])
        np.testing.assert_allclose(node.bias_tf, [-.1, 0., 0.])

        node.pending_learning_samples[2] = (
            np.array([1.3, 0., 0.]), np.array([.7, 0., 0.])
        )
        node.pending_learning_stamps[2] = 2
        node._used_observation_callback(
            used_observation(2, [1., 0., 0.], observation_covariance)
        )

        self.assertEqual(node.learning_count, 2)
        np.testing.assert_allclose(node.bias_current, [.2, 0., 0.])
        np.testing.assert_allclose(node.bias_tf, [-.2, 0., 0.])
        self.assertAlmostEqual(node.cov_current[0, 0], .0199)
        self.assertAlmostEqual(node.cov_tf[0, 0], .0199)
        self.assertTrue(np.all(np.linalg.eigvalsh(node.cov_current) > 0.))
        self.assertTrue(np.all(np.linalg.eigvalsh(node.cov_tf) > 0.))
        self.assertEqual(node.learned_bias_current_pub.publish.call_count, 2)
        published = node.learned_bias_current_pub.publish.call_args.args[0]
        np.testing.assert_allclose(published.data, [2., 2., .2, 0., 0.])
        self.assertEqual(node.learning_sample_count_pub.publish.call_count, 2)

    def test_sparse_feedback_matches_timestamp_instead_of_publisher_sequence(self):
        node = self.node()
        node.pending_learning_samples = {
            1: (np.array([9., 0., 0.]), np.array([8., 0., 0.])),
            41: (np.array([1.1, 0., 0.]), np.array([.9, 0., 0.])),
        }
        node.pending_learning_stamps = {1: 100, 41: 200}
        message = used_observation(1, [1., 0., 0.], np.eye(3) * 1.e-4)
        message.header.stamp = 200
        node._used_observation_callback(message)
        np.testing.assert_allclose(node.bias_current, [.1, 0., 0.])
        np.testing.assert_allclose(node.bias_tf, [-.1, 0., 0.])
        self.assertIn(1, node.pending_learning_samples)
        self.assertNotIn(41, node.pending_learning_samples)
        self.assertEqual(node.learned_bias_current_pub.publish.call_args.args[0].data[0], 41.)

    def test_missing_or_ambiguous_feedback_timestamp_cannot_update_learning(self):
        for stamps in ({1: 100}, {1: 200, 2: 200}):
            with self.subTest(stamps=stamps):
                node = self.node()
                node.pending_learning_samples = {
                    key: (np.zeros(3), np.zeros(3)) for key in stamps
                }
                node.pending_learning_stamps = stamps
                message = used_observation(1, [1., 0., 0.], np.eye(3) * 1.e-4)
                message.header.stamp = 200
                node._used_observation_callback(message)
                self.assertEqual(node.learning_count, 0)
                self.assertEqual(len(node.pending_learning_samples), len(stamps))

    def test_unknown_or_wrong_frame_feedback_is_ignored(self):
        node = self.node()
        covariance = np.eye(3) * 1.e-4
        node._used_observation_callback(
            used_observation(99, [1., 0., 0.], covariance)
        )
        node.pending_learning_samples[1] = (
            np.array([1., 0., 0.]), np.array([1., 0., 0.])
        )
        node._used_observation_callback(
            used_observation(1, [1., 0., 0.], covariance, frame='map')
        )
        self.assertEqual(node.learning_count, 0)
        self.assertIn(1, node.pending_learning_samples)

    def test_learning_can_be_disabled(self):
        node = self.node(enable_error_learning=False)
        node.pending_learning_samples[1] = (
            np.array([1., 0., 0.]), np.array([1., 0., 0.])
        )
        node._used_observation_callback(
            used_observation(1, [0., 0., 0.], np.eye(3) * 1.e-4)
        )
        self.assertEqual(node.learning_count, 0)

    @staticmethod
    def learn(node, operation, current_error, tf_error, observation_covariance=None):
        position = np.array([1., 2., 3.])
        node.pending_learning_samples[operation] = (
            position + np.asarray(current_error), position + np.asarray(tf_error)
        )
        node.pending_learning_stamps[operation] = operation
        node._used_observation_callback(used_observation(
            operation, position,
            np.eye(3) * 1.e-4 if observation_covariance is None else observation_covariance,
        ))

    def test_warmup_learns_bias_but_keeps_startup_covariance_for_nine_updates(self):
        node = self.node()
        self.assertEqual(node.learning_min_samples, 10)
        self.assertEqual(node.covariance_initial_weight, .5)
        current_cov, tf_cov = node.cov_current.copy(), node.cov_tf.copy()
        for operation in range(1, 10):
            self.learn(node, operation, [.02, -.03, .01], [-.01, .04, .02])
            np.testing.assert_allclose(node.bias_current, [.02, -.03, .01])
            np.testing.assert_allclose(node.bias_tf, [-.01, .04, .02])
            np.testing.assert_array_equal(node.cov_current, current_cov)
            np.testing.assert_array_equal(node.cov_tf, tf_cov)

    def test_tenth_update_blends_and_prevents_degenerate_error_overconfidence(self):
        node = self.node(ci_fixed_weight=.85)
        initial_current, initial_tf = node.initial_cov_current.copy(), node.initial_cov_tf.copy()
        for operation in range(1, 11):
            self.learn(node, operation, [.02, -.03, .01], [-.01, .04, .02])
        # Identical error samples give negative noise-subtracted variance.
        # Repair it before mixing, so 50% startup uncertainty remains in every direction.
        np.testing.assert_allclose(node.cov_current, .5*initial_current + .5*node.min_variance*np.eye(3))
        np.testing.assert_allclose(node.cov_tf, .5*initial_tf + .5*node.min_variance*np.eye(3))
        np.testing.assert_array_equal(node.initial_cov_current, initial_current)
        np.testing.assert_array_equal(node.initial_cov_tf, initial_tf)
        self.assertFalse(node.initial_cov_current.flags.writeable)
        self.assertFalse(node.initial_cov_tf.flags.writeable)
        _, prior_cov, _ = node._ci_fusion(np.zeros(3), node.cov_current,
                                         np.zeros(3), node.cov_tf)
        self.assertGreater(np.min(np.linalg.eigvalsh(prior_cov)), 1.e-4)
        published = node.learned_covariance_current_pub.publish.call_args.args[0]
        np.testing.assert_allclose(np.array(published.data[2:]).reshape(3, 3), node.cov_current)

    def test_blending_retains_startup_uncertainty_for_correlated_indefinite_estimate(self):
        node = self.node()
        raw = np.array([[.01, .02, 0.], [.02, .01, 0.], [0., 0., -1.]])
        stabilized = node._stabilize_learned_covariance(raw, node.initial_cov_tf)
        self.assertGreaterEqual(np.min(np.linalg.eigvalsh(stabilized - .5*node.initial_cov_tf)), -1.e-12)
        self.assertTrue(np.all(np.isfinite(stabilized)))

    def test_initial_weight_one_keeps_covariance_while_bias_still_learns(self):
        node = self.node(learning_min_samples=2, covariance_initial_weight=1.)
        initial_current, initial_tf = node.cov_current.copy(), node.cov_tf.copy()
        for operation in (1, 2):
            self.learn(node, operation, [.1, 0., 0.], [-.1, 0., 0.])
        np.testing.assert_allclose(node.bias_current, [.1, 0., 0.])
        np.testing.assert_allclose(node.cov_current, initial_current)
        np.testing.assert_allclose(node.cov_tf, initial_tf)

    def test_invalid_covariance_initial_weights_are_rejected(self):
        for weight in (-.01, 1.01, float('nan'), float('inf')):
            with self.subTest(weight=weight), self.assertRaises(ValueError):
                self.node(covariance_initial_weight=weight)


if __name__ == '__main__':
    unittest.main()
