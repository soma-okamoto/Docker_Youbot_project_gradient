"""ROS-free tests for sequential KL trend evaluation."""
import importlib.util
from pathlib import Path
import unittest


path = Path(__file__).resolve().parents[1] / "evaluation/kl_trend.py"
spec = importlib.util.spec_from_file_location("kl_trend", path)
kl_trend = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kl_trend)


def records(kl_values, shift_values=None):
    if shift_values is None:
        shift_values = kl_values
    return [
        {
            "operation_id": index + 1,
            "kl": kl_value,
            "mean_shift": shift_value,
            "mean_component": kl_value * 0.6,
            "covariance_component": kl_value * 0.4,
        }
        for index, (kl_value, shift_value) in enumerate(
            zip(kl_values, shift_values)
        )
    ]


class KLTrendTests(unittest.TestCase):
    def test_short_history_is_insufficient(self):
        summary = kl_trend.evaluate_adaptation_history(records([3.0, 2.0]))
        self.assertEqual(summary["adaptation_state"], "insufficient_data")
        self.assertEqual(summary["sample_count"], 2)

    def test_decreasing_sequence_is_adapting(self):
        summary = kl_trend.evaluate_adaptation_history(
            records([8.0, 7.0, 6.0, 4.0, 3.0, 2.0])
        )
        self.assertEqual(summary["adaptation_state"], "adapting")
        self.assertEqual(summary["kl_trend"]["overall_change"], "improved")
        self.assertEqual(
            summary["kl_trend"]["recent_direction"], "decreasing"
        )
        self.assertGreater(summary["kl_trend"]["relative_improvement"], 0.0)

    def test_improvement_followed_by_plateau_is_stable(self):
        summary = kl_trend.evaluate_adaptation_history(
            records([8.0, 7.0, 6.0, 2.0, 2.0, 2.0])
        )
        self.assertEqual(summary["adaptation_state"], "adapted_stable")

    def test_increasing_sequence_is_worsening(self):
        summary = kl_trend.evaluate_adaptation_history(
            records([1.0, 2.0, 3.0, 5.0, 6.0, 7.0])
        )
        self.assertEqual(summary["adaptation_state"], "worsening")

    def test_disagreement_is_mixed(self):
        summary = kl_trend.evaluate_adaptation_history(
            records(
                [8.0, 7.0, 6.0, 3.0, 2.0, 1.0],
                [1.0, 2.0, 3.0, 5.0, 6.0, 7.0],
            )
        )
        self.assertEqual(summary["adaptation_state"], "mixed")


if __name__ == "__main__":
    unittest.main()
