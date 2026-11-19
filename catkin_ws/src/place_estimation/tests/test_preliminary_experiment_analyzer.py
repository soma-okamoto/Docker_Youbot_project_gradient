"""ROS-free checks for sparse physical-observation attribution in bags."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


analysis = Path(__file__).resolve().parents[1] / 'analysis'
spec = importlib.util.spec_from_file_location(
    'preliminary_analyzer_under_test', analysis / 'preliminary_experiment_analyzer.py'
)
analyzer = importlib.util.module_from_spec(spec)
with patch.object(sys, 'path', [str(analysis)] + sys.path):
    spec.loader.exec_module(analyzer)


def prior(operation_id, stamp):
    return NS(header=NS(seq=operation_id, stamp=NS(to_nsec=lambda: stamp)),
              pose=NS(position=NS(x=0., y=0., z=0.)))


def feedback(seq, stamp):
    return NS(header=NS(seq=seq, stamp=NS(to_nsec=lambda: stamp)),
              pose=NS(pose=NS(position=NS(x=1., y=2., z=3.)),
                      covariance=[1. if i % 7 == 0 else 0. for i in range(36)]))


class FeedbackAttributionTests(unittest.TestCase):
    def test_bottle_id_is_not_used_as_operation_id(self):
        extractor = analyzer.BagExtractor()
        time = NS(to_sec=lambda: 1.)
        for operation in (1, 2):
            extractor.consume('/P_current', NS(data=[7., 1., 2., 3.]), time)
            extractor.consume('/P_pred', prior(operation, operation * 100), time)
        self.assertEqual(set(extractor.trials), {1, 2})
        self.assertEqual(extractor.trials[2]['bottle_id'], 7)
        self.assertEqual(extractor.trials[2]['current_z'], 3.)

    def test_context_preserves_raw_and_corrected_sources_for_explicit_operation(self):
        extractor = analyzer.BagExtractor()
        context = NS(operation_id=41, bottle_id=7,
                     raw_current_position=NS(x=1.1, y=0., z=.1),
                     raw_tf_position=NS(x=2., y=0., z=.3),
                     current_position=NS(x=1., y=0., z=.1),
                     tf_position=NS(x=2., y=0., z=.3),
                     current_covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .01],
                     tf_covariance=[.01, 0., 0., 0., .01, 0., 0., 0., .01])
        extractor.consume('/P_pred_context', context, NS(to_sec=lambda: 1.))
        row = extractor.trials[41]
        self.assertEqual(row['current_x'], 1.1)
        self.assertEqual(row['source_current_x'], 1.)
        self.assertEqual(row['source_raw_tf_z'], .3)
        self.assertEqual(row['source_current_cov_xx'], .01)

    def test_first_feedback_can_belong_to_operation_41(self):
        extractor = analyzer.BagExtractor()
        time = NS(to_sec=lambda: 1.)
        extractor.consume('/P_pred', prior(1, 100), time)
        extractor.consume('/P_pred', prior(41, 200), time)
        extractor.consume('/used_physical_observation', feedback(1, 200), time)
        self.assertNotIn('used_x', extractor.trials[1])
        self.assertEqual(extractor.trials[41]['used_x'], 1.)

    def test_unknown_and_ambiguous_timestamps_are_ignored(self):
        for stamp in (100, 999):
            with self.subTest(stamp=stamp):
                extractor = analyzer.BagExtractor()
                time = NS(to_sec=lambda: 1.)
                extractor.consume('/P_pred', prior(1, 100), time)
                extractor.consume('/P_pred', prior(2, 100), time)
                extractor.consume('/used_physical_observation', feedback(1, stamp), time)
                self.assertFalse(any('used_x' in row for row in extractor.trials.values()))


if __name__ == '__main__':
    unittest.main()
