"""Early valid observations, five-second deadlines and cancellation callbacks."""
import json
import unittest
from unittest.mock import patch

import numpy as np

import test_bottle_identity as identity


class ObservationWindowTests(unittest.TestCase):
    def setUp(self):
        self.h = identity.BottleIdentityTests()
        self.h.setUp()
        self.addCleanup(self.h.doCleanups)
        self.h.settings.pop('~wait_full_observation_window')
        self.h.settings.update({
            '~observation_timeout': 5., '~enable_yolo': True,
            '~min_observation_frames': 3,
            '~meta_message_type': 'float32_multi_array', '~meta_input_mode': 'stream',
            '~meta_candidate_stride': 12, '~meta_covariance_indices': list(range(3, 12)),
            '~publish_predictive_snapshot': True, '~observation_ci_weight_mode': 'fixed',
        })
        self.module = self.h.load('observation_fusion_node_new')
        clock_patch = patch.object(self.module.time, 'monotonic', return_value=0.)
        self.clock = clock_patch.start()
        self.addCleanup(clock_patch.stop)
        self.node = self.module.ObservationFusionNode()
        self.assertFalse(self.node.wait_full_observation_window)

    def prior(self, operation=1, bottle=7, x=1.):
        context = identity.PriorContext()
        context.operation_id = operation
        context.bottle_id = bottle
        context.header.frame_id = 'base_footprint'
        context.header.stamp = identity.Time(operation)
        context.position.x = context.fixed_position.x = x
        context.covariance = context.fixed_covariance = (np.eye(3) * .0025).reshape(-1).tolist()
        context.fixed_prior_available = True
        for source in ('current', 'tf'):
            getattr(context, source + '_position').x = x
            getattr(context, 'raw_' + source + '_position').x = x
            setattr(context, source + '_covariance', context.covariance[:])
        self.node._pred_context_cb(context)
        return context

    @staticmethod
    def observation(x=1., variance=.0004):
        return identity.Array([x, 0., 0.] + (np.eye(3) * variance).reshape(-1).tolist())

    def expire(self, now):
        self.clock.return_value = now
        self.node._timeout_cb(None)

    def results(self):
        return [call.args[0] for call in self.node.place_with_id_pub.publish.call_args_list]

    def meta_frames(self, count=3, x=1.):
        for _ in range(count):
            self.node._meta_array_cb(self.observation(x))

    def test_yolo_and_three_stable_meta_frames_finish_before_five_seconds(self):
        self.prior()
        self.clock.return_value = .2
        self.node._yolo_array_cb(self.observation())
        self.meta_frames(2)
        self.assertEqual(self.results(), [])
        self.node._meta_array_cb(self.observation())
        self.assertEqual(len(self.results()), 1)
        self.assertFalse(self.node.active)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data,
                         'prior_yolo_meta_fused')
        snapshot = json.loads(self.node.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot['trigger'], 'all_valid_observations_received')
        self.assertEqual(snapshot['sensors']['meta']['frame_count'], 3)
        self.expire(5.)
        self.assertEqual(len(self.results()), 1)

    def test_only_yolo_waits_until_five_seconds(self):
        self.prior()
        self.node._yolo_array_cb(self.observation())
        self.expire(4.999)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(len(self.results()), 1)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'prior_yolo_fused')

    def test_only_meta_waits_until_five_seconds(self):
        self.prior()
        self.meta_frames()
        self.assertEqual(len(self.node.meta_frames), 3)
        self.assertEqual(len(self.node.meta_candidates), 3)  # previews keep raw accumulation intact
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(self.node.meta_selected_index_pub.publish.call_args.args[0].data, 0)
        self.assertEqual(len(self.results()), 1)

    def test_no_observations_waits_then_returns_prior(self):
        self.prior()
        self.expire(4.)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(self.results()[0].position.x, 1.)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'no_observation')
        self.node.used_observation_pub.publish.assert_not_called()

    def test_empty_packed_yolo_does_not_count_as_ready(self):
        self.prior()
        self.node._yolo_array_cb(identity.Array([]))
        self.meta_frames()
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'prior_meta_fused')

    def test_meta_detection_ratio_is_checked_before_early_completion(self):
        self.prior()
        self.node._yolo_array_cb(self.observation())
        for _ in range(3):
            self.node._meta_array_cb(identity.Array([]))
        self.meta_frames(4)  # 4/7 < 60%, even though at least three detections exist
        self.assertEqual(self.results(), [])
        self.assertEqual(len(self.node.meta_frames), 7)
        self.node._meta_array_cb(self.observation())  # 5/8 >= 60%
        self.assertEqual(len(self.results()), 1)

    def test_invalid_meta_covariance_does_not_finish_early(self):
        self.node.meta_input_mode = 'packed'
        self.prior()
        self.node._yolo_array_cb(self.observation())
        self.node._meta_array_cb(self.observation(variance=.04))
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'prior_yolo_fused')

    def test_reference_rejected_meta_does_not_finish_early(self):
        self.prior()
        self.node._yolo_array_cb(self.observation())
        self.meta_frames(x=5.)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'prior_yolo_fused')

    def test_new_prior_discards_old_result_feedback_and_snapshot(self):
        self.prior()
        self.node._yolo_array_cb(self.observation())
        old_timer = self.node.timer
        old_callback = self.h.ros.Timer.call_args.args[1]
        self.clock.return_value = 1.
        self.prior(2, 12, 2.)
        old_timer.shutdown.assert_called_once()
        self.assertEqual(self.results(), [])
        self.node.used_observation_pub.publish.assert_not_called()
        self.node.predictive_snapshot_pub.publish.assert_not_called()
        self.assertEqual(self.node.bottle_id, 12)
        self.assertEqual(self.node.prior_sources['current'][0][0], 2.)
        self.assertEqual(self.node.yolo_candidates, [])
        self.assertFalse(self.node.yolo_received)
        self.assertEqual(self.node.observation_deadline, 6.)
        self.clock.return_value = 5.
        old_callback(None)
        self.assertEqual(self.results(), [])
        self.node._yolo_array_cb(self.observation(2.))
        self.meta_frames(x=2.)
        self.assertEqual([r.bottle_id for r in self.results()], [12])
        snapshot = json.loads(self.node.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot['operation_id'], 2)
        self.assertEqual(snapshot['bottle_id'], 12)
        self.assertEqual(snapshot['priors']['adaptive']['mean'][0], 2.)

    def test_early_timer_is_rearmed_only_for_remaining_time(self):
        self.prior()
        self.expire(2.)
        self.assertEqual(self.h.ros.Timer.call_args.args[0], 3.)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(len(self.results()), 1)

    def test_full_window_opt_in_waits_even_when_both_are_ready(self):
        self.node.wait_full_observation_window = True
        self.prior()
        self.node._yolo_array_cb(self.observation())
        self.meta_frames()
        self.assertEqual(self.results(), [])
        self.expire(4.999)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(len(self.results()), 1)

    def test_full_window_opt_in_also_cancels_on_new_prior(self):
        self.node.wait_full_observation_window = True
        self.prior()
        self.clock.return_value = 1.
        self.prior(2, 12, 2.)
        self.assertEqual(self.results(), [])
        self.assertEqual(self.node.bottle_id, 12)
        self.assertEqual(self.node.observation_deadline, 6.)

    def enable_single_frame(self):
        self.h.settings['~min_observation_frames'] = 1
        self.node = self.module.ObservationFusionNode()

    def test_default_two_frames_waits_for_second_meta_detection(self):
        self.h.settings.pop('~min_observation_frames')
        self.node = self.module.ObservationFusionNode()
        self.assertEqual(self.node.min_observation_frames, 2)
        self.prior()
        self.node._yolo_array_cb(self.observation())
        self.node._meta_array_cb(self.observation())
        self.assertEqual(self.results(), [])
        self.node._meta_array_cb(self.observation())
        self.assertEqual(len(self.results()), 1)
        snapshot = json.loads(self.node.predictive_snapshot_pub.publish.call_args.args[0].data)
        self.assertEqual(snapshot['sensors']['meta']['frame_count'], 2)

    def test_one_frame_finishes_early_and_retains_supplied_covariance(self):
        self.enable_single_frame()
        self.prior()
        self.clock.return_value = .2
        self.node._yolo_array_cb(self.observation())
        with np.errstate(divide='raise', invalid='raise'):
            self.node._meta_array_cb(self.observation())
        self.assertEqual(len(self.results()), 1)
        snapshot = json.loads(self.node.predictive_snapshot_pub.publish.call_args.args[0].data)
        meta = snapshot['sensors']['meta']
        self.assertEqual(meta['frame_count'], 1)
        self.assertEqual(len(meta['quality_candidates']), 1)
        np.testing.assert_allclose(np.array(meta['quality_candidates'][0]['covariance']).reshape(3, 3),
                                   np.eye(3) * .0004)

    def test_one_meta_frame_without_yolo_still_waits_until_timeout(self):
        self.enable_single_frame()
        self.prior()
        self.node._meta_array_cb(self.observation())
        self.expire(4.999)
        self.assertEqual(self.results(), [])
        self.expire(5.)
        self.assertEqual(len(self.results()), 1)
        self.assertEqual(self.node.status_pub.publish.call_args.args[0].data, 'prior_meta_fused')

    def test_multiple_frames_still_add_temporal_covariance_with_minimum_one(self):
        self.enable_single_frame()
        self.prior()
        for x in (.99, 1., 1.01):
            self.node._meta_array_cb(self.observation(x))
        self.expire(5.)
        snapshot = json.loads(self.node.predictive_snapshot_pub.publish.call_args.args[0].data)
        covariance = np.array(snapshot['sensors']['meta']['quality_candidates'][0]['covariance'])
        expected = np.diag([(.0001 + .0004) / 3., .0004 / 3., .0004 / 3.])
        np.testing.assert_allclose(covariance.reshape(3, 3), expected)

    def test_nonpositive_minimum_frame_count_is_rejected(self):
        for value in (0, -1):
            with self.subTest(value=value):
                self.h.settings['~min_observation_frames'] = value
                with self.assertRaises(ValueError):
                    self.module.ObservationFusionNode()


if __name__ == '__main__':
    unittest.main()
