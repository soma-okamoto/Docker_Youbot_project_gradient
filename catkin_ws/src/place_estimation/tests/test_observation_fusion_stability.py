"""ROS-free regression tests; run with python3 -m unittest discover -s tests."""
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
    std = ModuleType('std_msgs.msg')
    for name in ('Float32MultiArray', 'Int32', 'MultiArrayDimension', 'String'):
        setattr(std, name, SimpleNamespace)
    modules = {'rospy': rospy, 'geometry_msgs': ModuleType('geometry_msgs'),
               'geometry_msgs.msg': geometry, 'std_msgs': ModuleType('std_msgs'),
               'std_msgs.msg': std}
    path = Path(__file__).resolve().parents[1] / 'scripts/observation_fusion_node_new.py'
    spec = importlib.util.spec_from_file_location('fusion_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.ObservationFusionNode, rospy


Node, rospy = load_node()


def cov(std):
    return np.eye(3) * std ** 2


def record(x, covariance):
    return [x, 0., 0.] + np.asarray(covariance).reshape(-1).tolist()


class StabilityTests(unittest.TestCase):
    def node(self, **params):
        settings = {'~meta_message_type': 'float32_multi_array'}
        settings.update({'~' + k: v for k, v in params.items()})
        rospy.get_param = lambda name, default=None: settings.get(name, default)
        rospy.has_param = lambda name: name in settings
        node = Node()
        node.active = True
        node.prior = np.array([5., 0., 0.])
        node.prior_cov = cov(.05)
        node._publish = Mock()
        return node

    def finish(self, node, yolo, meta):
        node.yolo_candidates = [np.array([x, 0., 0.]) for x, _ in yolo]
        node.yolo_covariances = [c for _, c in yolo]
        node.meta_candidates = [np.array([x, 0., 0.]) for x, _ in meta]
        node.meta_covariances = [c for _, c in meta]
        node.yolo_received = bool(yolo)
        node.meta_received = bool(meta)
        node._finalize_locked('test')
        node._publish.assert_called_once()
        self.assertFalse(node.active)
        self.assertEqual(node.meta_covariances, [])
        return node._publish.call_args.kwargs

    def test_mismatch_uses_more_stable_sensor_in_both_directions(self):
        for ystd, mstd, expected in ((.02, .06, 'yolo'), (.06, .02, 'meta')):
            with self.subTest(expected=expected):
                # Legacy preference must not override stability.
                node = self.node(mismatch_preferred_sensor='meta' if expected == 'yolo' else 'yolo')
                out = self.finish(node, [(0., cov(ystd))], [(1., cov(mstd))])
                self.assertEqual(out['status'], 'sensor_mismatch_' + expected + '_selected')
                self.assertEqual(out['position'][0], 0. if expected == 'yolo' else 1.)
                np.testing.assert_allclose(out['covariance'], cov(min(ystd, mstd)))

    def test_equal_uncertainty_conflict_uses_prior(self):
        out = self.finish(self.node(), [(0., cov(.02))], [(1., cov(.02))])
        self.assertEqual(out['status'], 'sensor_mismatch_equal_uncertainty_prior_used')
        np.testing.assert_allclose(out['position'], [5., 0., 0.])
        np.testing.assert_allclose(out['covariance'], cov(.05))

    def test_consistent_observations_still_fuse(self):
        out = self.finish(self.node(), [(0., cov(.02))], [(.01, cov(.02))])
        self.assertEqual(out['status'], 'both_valid')
        np.testing.assert_allclose(out['position'], [.005, 0., 0.])
        np.testing.assert_allclose(out['covariance'], cov(.02))

    def test_large_covariance_rejected_even_when_positions_agree(self):
        for sensor in ('yolo', 'meta'):
            with self.subTest(sensor=sensor):
                yc = cov(.2 if sensor == 'yolo' else .02)
                mc = cov(.2 if sensor == 'meta' else .02)
                out = self.finish(self.node(), [(0., yc)], [(0., mc)])
                self.assertEqual(out['status'], 'meta_only' if sensor == 'yolo' else 'yolo_only')
                self.assertEqual(out[sensor + '_index'], -1)
                np.testing.assert_array_equal(out[sensor + '_scores'], [0.])

    def test_both_unstable_use_prior(self):
        out = self.finish(self.node(), [(0., cov(.2))], [(0., cov(.3))])
        self.assertEqual(out['status'], 'no_valid_observation')
        np.testing.assert_allclose(out['position'], [5., 0., 0.])
        self.assertEqual((out['yolo_index'], out['meta_index']), (-1, -1))

    def test_single_unstable_uses_prior(self):
        out = self.finish(self.node(), [(0., cov(.2))], [])
        self.assertEqual(out['status'], 'no_valid_observation')

    def test_no_observation_uses_prior(self):
        out = self.finish(self.node(), [], [])
        self.assertEqual(out['status'], 'no_observation')

    def test_invalid_covariance_does_not_hide_valid_candidate(self):
        invalids = [np.full((3, 3), np.nan), np.full((3, 3), np.inf),
                    np.zeros((3, 3)), np.diag([-.01, .01, .01]),
                    np.array([[.001, .002, 0.], [.002, .001, 0.], [0., 0., .001]])]
        for invalid in invalids:
            with self.subTest(covariance=invalid):
                out = self.finish(self.node(), [(0., invalid), (.1, cov(.02))], [])
                self.assertEqual(out['status'], 'yolo_only')
                self.assertEqual(out['yolo_index'], 1)
                self.assertEqual(out['yolo_scores'][0], 0.)

    def test_rejected_candidate_cannot_reenter_pair_search(self):
        # Rejected pair has perfect positional agreement; valid pair also exists.
        for sensor in ('yolo', 'meta'):
            with self.subTest(sensor=sensor):
                mixed = [(0., cov(.2)), (.01, cov(.02))]
                single = [(0., cov(.02))]
                out = self.finish(self.node(), mixed if sensor == 'yolo' else single,
                                  mixed if sensor == 'meta' else single)
                self.assertEqual(out['status'], 'both_valid')
                self.assertEqual(out[sensor + '_index'], 1)
                self.assertEqual(out[sensor + '_scores'][0], 0.)
                self.assertAlmostEqual(out['position'][0], .005)

    def test_gate_uses_principal_variance_including_correlations(self):
        c = np.array([[.006, .005, 0.], [.005, .006, 0.], [0., 0., .001]])
        out = self.finish(self.node(), [(0., c)], [])
        self.assertEqual(out['status'], 'no_valid_observation')

    def test_threshold_boundary_and_sensor_specific_configuration(self):
        out = self.finish(self.node(max_std_yolo=.1), [(0., cov(.1))], [])
        self.assertEqual(out['status'], 'yolo_only')
        out = self.finish(self.node(max_std_yolo=.03, max_std_meta=.07),
                          [(0., cov(.04))], [(1., cov(.06))])
        self.assertEqual(out['status'], 'meta_only')

    def test_invalid_threshold_configuration_is_rejected(self):
        for name in ('max_std_yolo', 'max_std_meta'):
            for value in (0., -1., float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.node(**{name: value})

    def test_meta_covariance_input_affects_actual_selection(self):
        node = self.node(meta_candidate_stride=12, meta_covariance_indices=list(range(3, 12)))
        node._meta_array_cb(SimpleNamespace(data=record(1., cov(.01))))
        node._yolo_array_cb(SimpleNamespace(data=record(0., cov(.06))))
        out = node._publish.call_args.kwargs
        self.assertEqual(out['status'], 'sensor_mismatch_meta_selected')
        np.testing.assert_allclose(out['covariance'], cov(.01))

    def test_invalid_yolo_input_covariance_is_not_replaced_by_fixed_covariance(self):
        node = self.node(enable_yolo=True)
        node._yolo_array_cb(SimpleNamespace(data=record(0., np.full((3, 3), np.nan))))
        node._meta_array_cb(SimpleNamespace(data=[1., 0., 0.]))
        out = node._publish.call_args.kwargs
        self.assertEqual(out['status'], 'meta_only')

    def test_legacy_meta_xyz_input_uses_fixed_covariance(self):
        node = self.node(enable_yolo=False)
        node._meta_array_cb(SimpleNamespace(data=[1., 0., 0.]))
        out = node._publish.call_args.kwargs
        self.assertEqual(out['status'], 'meta_only')
        np.testing.assert_allclose(out['covariance'], node.cov_meta)

    def test_legacy_meta_pose_input_uses_fixed_covariance(self):
        node = self.node(enable_yolo=False, meta_message_type='pose_stamped')
        msg = SimpleNamespace(header=SimpleNamespace(frame_id='base_footprint'),
                              pose=SimpleNamespace(position=SimpleNamespace(x=1., y=0., z=0.)))
        node._meta_pose_cb(msg)
        self.assertEqual(node._publish.call_args.kwargs['status'], 'meta_only')

    def test_meta_stream_keeps_covariances_aligned_and_truncates_together(self):
        node = self.node(meta_input_mode='stream', meta_candidate_stride=12,
                         meta_covariance_indices=list(range(3, 12)), max_meta_candidates=2)
        node._meta_array_cb(SimpleNamespace(data=record(float('nan'), cov(.09)) + record(1., cov(.01))))
        node._meta_array_cb(SimpleNamespace(data=record(2., cov(.02)) + record(3., cov(.03))))
        self.assertEqual(len(node.meta_candidates), 2)
        self.assertEqual(len(node.meta_covariances), 2)
        np.testing.assert_allclose(node.meta_covariances[0], cov(.01))
        np.testing.assert_allclose(node.meta_covariances[1], cov(.02))
        node._publish.assert_not_called()
        node._timeout_cb(None)
        self.assertEqual(node._publish.call_args.kwargs['status'], 'meta_only')
        self.assertEqual(node.meta_covariances, [])

    def test_bad_or_empty_meta_covariance_messages(self):
        node = self.node(enable_yolo=False, meta_candidate_stride=12,
                         meta_covariance_indices=list(range(3, 12)))
        node._meta_array_cb(SimpleNamespace(data=[1., 2., 3.]))
        self.assertFalse(node.meta_received)
        node._meta_array_cb(SimpleNamespace(data=[]))
        self.assertEqual(node._publish.call_args.kwargs['status'], 'no_valid_observation')

    def test_meta_covariance_configuration_is_validated(self):
        for params in ({'meta_covariance_indices': list(range(3, 12))},
                       {'meta_candidate_stride': 12, 'meta_covariance_indices': [3] * 9},
                       {'meta_message_type': 'pose_stamped', 'meta_candidate_stride': 12,
                        'meta_covariance_indices': list(range(3, 12))}):
            with self.subTest(params=params), self.assertRaises(ValueError):
                self.node(**params)


if __name__ == '__main__':
    unittest.main()
