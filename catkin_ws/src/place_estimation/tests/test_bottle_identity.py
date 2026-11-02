"""Bottle ID propagation through the existing prior/fusion callbacks, ROS-free."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np


class Time:
    def __init__(self, value=0):
        self.value = value

    def __eq__(self, other):
        return isinstance(other, Time) and self.value == other.value

    @staticmethod
    def now():
        return Time(100)


class PoseStamped:
    def __init__(self):
        self.header = NS(seq=0, stamp=Time(), frame_id='')
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


class PoseWithCovarianceStamped:
    def __init__(self):
        pose = PoseStamped()
        self.header = pose.header
        self.pose = NS(pose=pose.pose, covariance=[0.] * 36)


class Array:
    def __init__(self, data=None):
        self.data = [] if data is None else data
        self.layout = NS(dim=[], data_offset=0)


class BottleDistribution:
    def __init__(self):
        pose = PoseStamped()
        self.header, self.position = pose.header, pose.pose.position
        self.bottle_id = 0
        self.covariance = [0.] * 9


class BottleIdentityTests(unittest.TestCase):
    def setUp(self):
        self.settings = {'~enable_bottle_id': True, '~expected_frame': 'base_footprint',
                         '~output_frame': 'base_footprint', '~ci_weight_mode': 'fixed',
                         '~enable_yolo': False, '~meta_message_type': 'disabled'}
        self.ros = ModuleType('rospy')
        self.ros.get_param = lambda name, default=None: self.settings.get(name, default)
        self.ros.has_param = lambda name: name in self.settings
        self.ros.Time = Time
        self.ros.Duration = lambda value: value
        for name in ('Subscriber', 'Timer', 'loginfo', 'logwarn', 'logerr',
                     'logerr_throttle', 'logwarn_throttle'):
            setattr(self.ros, name, Mock())
        self.ros.Publisher = Mock(side_effect=lambda *args, **kwargs: Mock())
        geometry, std, custom = ModuleType('geometry_msgs.msg'), ModuleType('std_msgs.msg'), ModuleType('place_estimation.msg')
        geometry.PoseStamped = PoseStamped
        geometry.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        std.Float32MultiArray = std.Float64MultiArray = Array
        std.MultiArrayDimension = std.Int32 = std.String = NS
        custom.BottleDistribution = BottleDistribution
        self.modules = patch.dict(sys.modules, {
            'rospy': self.ros, 'geometry_msgs': ModuleType('geometry_msgs'),
            'geometry_msgs.msg': geometry, 'std_msgs': ModuleType('std_msgs'),
            'std_msgs.msg': std, 'place_estimation': ModuleType('place_estimation'),
            'place_estimation.msg': custom})
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.prior = self.load('prior_distribution_node').PriorDistributionNode()
        self.fusion = self.load('observation_fusion_node_new').ObservationFusionNode()

    @staticmethod
    def load(name):
        path = Path(__file__).resolve().parents[1] / 'scripts' / (name + '.py')
        spec = importlib.util.spec_from_file_location('identity_' + name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def feed(self, bottle, x, tf_first=False):
        tf = PoseStamped()
        tf.header.frame_id = 'base_footprint'
        tf.pose.position.x = x
        current = Array([bottle, x, 0., 0.])
        if tf_first:
            self.prior._tf_callback(tf)
            self.prior._current_callback(current)
        else:
            self.prior._current_callback(current)
            self.prior._tf_callback(tf)
        identified = self.prior.pred_with_id_pub.publish.call_args.args[0]
        self.fusion._pred_with_id_cb(identified)
        return identified

    def results(self):
        return [call.args[0] for call in self.fusion.place_with_id_pub.publish.call_args_list]

    def test_different_bottles_and_tf_arrival_orders(self):
        for bottle, x, tf_first in ((7, 1., False), (12, 2., True), (0, 3., False)):
            pred = self.feed(bottle, x, tf_first)
            self.assertEqual(pred.bottle_id, bottle)
            self.assertAlmostEqual(pred.position.x, x)
        self.assertEqual([r.bottle_id for r in self.results()], [7, 12, 0])
        np.testing.assert_allclose([r.position.x for r in self.results()], [1., 2., 3.])
        self.assertEqual([r.header.seq for r in self.results()], [1, 2, 3])
        self.assertEqual(self.fusion.place_pub.publish.call_count, 3)

    def test_same_bottle_can_be_operated_repeatedly(self):
        self.feed(7, 1.)
        self.feed(7, 2.)
        self.assertEqual([r.bottle_id for r in self.results()], [7, 7])
        np.testing.assert_allclose([r.position.x for r in self.results()], [1., 2.])

    def test_new_prior_finalizes_previous_with_previous_id(self):
        self.fusion.enable_meta = True  # Leave each operation open until timeout/new prior.
        self.feed(7, 1.)
        self.assertEqual(self.results(), [])
        self.feed(12, 2.)
        self.assertEqual([r.bottle_id for r in self.results()], [7])
        self.assertEqual(self.fusion.bottle_id, 12)
        self.fusion._timeout_cb(None)
        self.assertEqual([r.bottle_id for r in self.results()], [7, 12])
        np.testing.assert_allclose([r.position.x for r in self.results()], [1., 2.])
        self.assertIsNone(self.fusion.bottle_id)

    def test_selected_observation_keeps_manipulated_bottle_id(self):
        self.fusion.enable_yolo = True
        self.feed(42, 1.)
        self.fusion._yolo_array_cb(Array([1.01, 0., 0.] + (np.eye(3)*.0001).reshape(-1).tolist()))
        result = self.results()[0]
        self.assertEqual(result.bottle_id, 42)
        self.assertAlmostEqual(result.position.x, 1.01)
        self.assertEqual(len(result.covariance), 9)
        self.assertEqual(self.fusion.used_observation_pub.publish.call_count, 1)

    def test_duplicate_pending_current_does_not_replace_id(self):
        self.prior._current_callback(Array([7., 1., 0., 0.]))
        self.prior._current_callback(Array([12., 2., 0., 0.]))
        self.assertEqual(self.prior.pending_bottle_id, 7)
        np.testing.assert_allclose(self.prior.pending_current, [1., 0., 0.])

    def test_invalid_id_does_not_become_pending_input(self):
        for value in (-1., 1.5, float('nan'), float('inf'), 16777218.):
            self.prior._current_callback(Array([value, 1., 0., 0.]))
            self.assertIsNone(self.prior.pending_current)
            self.assertIsNone(self.prior.pending_bottle_id)

    def test_identity_mode_subscribes_only_to_atomic_prior(self):
        self.assertIsNone(self.fusion.pred_sub)
        self.assertIsNone(self.fusion.pred_cov_sub)
        self.assertIsNotNone(self.fusion.pred_with_id_sub)

    def test_malformed_prior_does_not_replace_active_id(self):
        self.fusion.enable_meta = True
        prior = self.feed(7, 1.)
        for field, value in (('bottle_id', -1), ('covariance', [float('nan')]*9)):
            import copy
            invalid = copy.deepcopy(prior)
            setattr(invalid, field, value)
            self.fusion._pred_with_id_cb(invalid)
        self.assertEqual(self.fusion.bottle_id, 7)
        self.assertEqual(self.results(), [])


if __name__ == '__main__':
    unittest.main()
