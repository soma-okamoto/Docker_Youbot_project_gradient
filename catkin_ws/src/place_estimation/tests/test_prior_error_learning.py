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
    for name in ('Publisher', 'Subscriber', 'loginfo', 'logwarn', 'logerr',
                 'logerr_throttle', 'logwarn_throttle'):
        setattr(rospy, name, Mock())
    geometry = ModuleType('geometry_msgs.msg')
    geometry.PoseStamped = SimpleNamespace
    geometry.PoseWithCovarianceStamped = SimpleNamespace
    std = ModuleType('std_msgs.msg')
    std.Float32MultiArray = SimpleNamespace
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
        header=SimpleNamespace(seq=seq, frame_id=frame),
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
        node = self.node(learning_min_samples=2)
        observation_covariance = np.eye(3) * 1.e-4

        node.pending_learning_samples[1] = (
            np.array([1.1, 0., 0.]), np.array([.9, 0., 0.])
        )
        node._used_observation_callback(
            used_observation(1, [1., 0., 0.], observation_covariance)
        )
        np.testing.assert_allclose(node.bias_current, [.1, 0., 0.])
        np.testing.assert_allclose(node.bias_tf, [-.1, 0., 0.])

        node.pending_learning_samples[2] = (
            np.array([1.3, 0., 0.]), np.array([.7, 0., 0.])
        )
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


if __name__ == '__main__':
    unittest.main()
