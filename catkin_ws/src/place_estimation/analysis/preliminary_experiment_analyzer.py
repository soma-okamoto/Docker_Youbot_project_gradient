#!/usr/bin/env python3
"""Convert a preliminary-experiment rosbag into CSV and evidence reports."""
import argparse
from collections import deque
import csv
import json
from pathlib import Path
import sys
from typing import Dict, Optional

import numpy as np

try:
    from preliminary_statistics import (
        build_parameter_report,
        principal_standard_deviation,
    )
except ImportError:
    # catkin's devel-space wrapper changes sys.path[0]. Resolve the package's
    # source/share directory so the same command works through rosrun.
    import rospkg

    analysis_directory = (
        Path(rospkg.RosPack().get_path("place_estimation")) / "analysis"
    )
    sys.path.insert(0, str(analysis_directory))
    from preliminary_statistics import (  # noqa: E402
        build_parameter_report,
        principal_standard_deviation,
    )


class BagExtractor:
    """Collect trial-aligned fields from the package's ROS topics."""

    def __init__(self, yolo_stride: int = 12, meta_stride: int = 3):
        self.yolo_stride = yolo_stride
        self.meta_stride = meta_stride
        self.trials: Dict[int, Dict[str, object]] = {}
        self.learning: Dict[int, Dict[str, object]] = {}
        self.pending_current = None
        self.pending_tf = None
        self.active_operation_id: Optional[int] = None
        self.prior_covariance_queue = deque()
        self.posterior_covariance_queue = deque()
        self.final_adaptation_summary = None

    def _trial(self, operation_id: int) -> Dict[str, object]:
        return self.trials.setdefault(
            int(operation_id), {"operation_id": int(operation_id)}
        )

    @staticmethod
    def _position(message):
        return [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ]

    @staticmethod
    def _assign_vector(row, prefix, values):
        values = np.asarray(values, dtype=float).reshape(3)
        for axis, value in zip("xyz", values):
            row[f"{prefix}_{axis}"] = float(value)

    @staticmethod
    def _assign_covariance(row, prefix, values):
        covariance = np.asarray(values, dtype=float).reshape(3, 3)
        labels = (
            "xx", "xy", "xz", "yx", "yy", "yz", "zx", "zy", "zz"
        )
        for label, value in zip(labels, covariance.reshape(-1)):
            row[f"{prefix}_{label}"] = float(value)

    @staticmethod
    def _operation_id(message, fallback=None):
        try:
            value = int(message.header.seq)
            return value if value > 0 else fallback
        except (AttributeError, TypeError, ValueError):
            return fallback

    def _learning_message(self, topic, message):
        values = np.asarray(message.data, dtype=float)
        if values.size < 2:
            return
        operation_id = int(values[0])
        row = self.learning.setdefault(
            operation_id,
            {
                "operation_id": operation_id,
                "sample_count": int(values[1]),
            },
        )
        row["sample_count"] = int(values[1])
        if topic.endswith("bias_current") and values.size == 5:
            row["bias_current"] = values[2:5].tolist()
        elif topic.endswith("bias_tf") and values.size == 5:
            row["bias_tf"] = values[2:5].tolist()
        elif topic.endswith("covariance_current") and values.size == 11:
            row["covariance_current"] = values[2:11].reshape(3, 3).tolist()
        elif topic.endswith("covariance_tf") and values.size == 11:
            row["covariance_tf"] = values[2:11].reshape(3, 3).tolist()

    def consume(self, topic, message, bag_time):
        timestamp = float(bag_time.to_sec())

        if topic == "/P_current":
            values = np.asarray(message.data, dtype=float)
            if values.size >= 4 and np.all(np.isfinite(values[1:4])):
                # In the current data contract, element 0 is metadata.  If it
                # is a positive integer, use it as the operation ID; otherwise
                # retain arrival-order matching as a compatibility fallback.
                metadata_id = (
                    int(values[0])
                    if np.isfinite(values[0]) and values[0].is_integer()
                    else 0
                )
                if metadata_id > 0:
                    self._assign_vector(
                        self._trial(metadata_id), "current", values[1:4]
                    )
                else:
                    self.pending_current = values[1:4].copy()
            return

        if topic == "/P_tf":
            operation_id = self._operation_id(message)
            position = np.asarray(self._position(message), dtype=float)
            if operation_id is not None:
                self._assign_vector(
                    self._trial(operation_id), "tf", position
                )
            else:
                self.pending_tf = position
            return

        if topic == "/P_pred":
            operation_id = self._operation_id(message)
            if operation_id is None:
                return
            self.active_operation_id = operation_id
            row = self._trial(operation_id)
            row["prior_time_s"] = timestamp
            self._assign_vector(row, "pred", self._position(message))
            if self.pending_current is not None:
                self._assign_vector(row, "current", self.pending_current)
                self.pending_current = None
            if self.pending_tf is not None:
                self._assign_vector(row, "tf", self.pending_tf)
                self.pending_tf = None
            self.prior_covariance_queue.append(operation_id)
            return

        if topic == "/Sigma_pred":
            if self.prior_covariance_queue and len(message.data) == 9:
                operation_id = self.prior_covariance_queue.popleft()
                self._assign_covariance(
                    self._trial(operation_id), "pred_cov", message.data
                )
            return

        if topic == "/P_place":
            operation_id = self._operation_id(
                message, self.active_operation_id
            )
            if operation_id is None:
                return
            row = self._trial(operation_id)
            row["posterior_time_s"] = timestamp
            self._assign_vector(row, "place", self._position(message))
            self.posterior_covariance_queue.append(operation_id)
            return

        if topic == "/Sigma_place":
            if self.posterior_covariance_queue and len(message.data) == 9:
                operation_id = self.posterior_covariance_queue.popleft()
                self._assign_covariance(
                    self._trial(operation_id), "place_cov", message.data
                )
            return

        if topic == "/used_physical_observation":
            operation_id = self._operation_id(
                message, self.active_operation_id
            )
            if operation_id is None:
                return
            row = self._trial(operation_id)
            self._assign_vector(row, "used", self._position(message.pose))
            covariance = np.asarray(
                message.pose.covariance, dtype=float
            ).reshape(6, 6)[:3, :3]
            self._assign_covariance(row, "used_cov", covariance)
            row["used_principal_std_m"] = principal_standard_deviation(
                covariance
            )
            return

        if topic == "/observation_status" and self.active_operation_id:
            self._trial(self.active_operation_id)["observation_status"] = str(
                message.data
            )
            return

        if topic == "/observation_distances" and self.active_operation_id:
            values = np.asarray(message.data, dtype=float)
            row = self._trial(self.active_operation_id)
            labels = (
                "selected_yolo_d2",
                "selected_meta_d2",
                "selected_yolo_meta_d2",
            )
            for label, value in zip(labels, values[:3]):
                row[label] = float(value) if np.isfinite(value) else None
            return

        if topic in {
            "/yolo_candidate_distances",
            "/meta_candidate_distances",
        } and self.active_operation_id:
            key = topic.strip("/")
            values = np.asarray(message.data, dtype=float)
            self._trial(self.active_operation_id)[key] = [
                float(value) if np.isfinite(value) else None for value in values
            ]
            return

        if topic == "/P_yolo" and self.active_operation_id:
            row = self._trial(self.active_operation_id)
            row["yolo_message_count"] = int(
                row.get("yolo_message_count", 0)
            ) + 1
            values = np.asarray(message.data, dtype=float)
            if self.yolo_stride > 0 and values.size % self.yolo_stride == 0:
                records = values.reshape(-1, self.yolo_stride)
                row["yolo_candidate_count"] = int(
                    row.get("yolo_candidate_count", 0)
                ) + records.shape[0]
                if self.yolo_stride >= 12:
                    standard_deviations = row.setdefault(
                        "_yolo_principal_stds", []
                    )
                    for record in records:
                        value = principal_standard_deviation(record[3:12])
                        if np.isfinite(value):
                            standard_deviations.append(value)
            return

        if topic == "/P_meta" and self.active_operation_id:
            row = self._trial(self.active_operation_id)
            row["meta_frame_count"] = int(row.get("meta_frame_count", 0)) + 1
            values = np.asarray(message.data, dtype=float)
            if self.meta_stride > 0 and values.size % self.meta_stride == 0:
                row["meta_candidate_count"] = int(
                    row.get("meta_candidate_count", 0)
                ) + values.size // self.meta_stride
            return

        if topic == "/kl_evaluation":
            values = np.asarray(message.data, dtype=float)
            if values.size != 10:
                return
            operation_id = int(values[0])
            row = self._trial(operation_id)
            labels = (
                "kl_prior_to_posterior",
                "kl_posterior_to_prior",
                "symmetric_kl",
                "mean_shift_m",
                "kl_mean_component",
                "kl_covariance_component",
                "running_mean_kl",
                "running_std_kl",
                "ema_kl",
            )
            for label, value in zip(labels, values[1:]):
                row[label] = float(value)
            return

        if topic == "/kl_adaptation_summary":
            try:
                self.final_adaptation_summary = json.loads(message.data)
            except (TypeError, ValueError):
                pass
            return

        if topic in {
            "/learned_bias_current",
            "/learned_bias_tf",
            "/learned_covariance_current",
            "/learned_covariance_tf",
        }:
            self._learning_message(topic, message)


def _csv_value(value):
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, separators=(",", ":"), sort_keys=True)
    return value


def write_dictionary_csv(path: Path, rows):
    rows = list(rows)
    fieldnames = sorted(
        {key for row in rows for key in row if not key.startswith("_")},
        key=lambda key: (key != "operation_id", key),
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {key: _csv_value(row.get(key, "")) for key in fieldnames}
            )


def learning_csv_rows(rows):
    output = []
    for row in rows:
        flattened = {
            "operation_id": row["operation_id"],
            "sample_count": row.get("sample_count"),
        }
        for prefix in ("bias_current", "bias_tf"):
            value = row.get(prefix)
            if value is not None:
                for axis, component in zip("xyz", value):
                    flattened[f"{prefix}_{axis}"] = component
        for prefix in ("covariance_current", "covariance_tf"):
            value = row.get(prefix)
            if value is not None:
                labels = (
                    "xx", "xy", "xz", "yx", "yy", "yz", "zx", "zy", "zz"
                )
                for label, component in zip(
                    labels, np.asarray(value).reshape(-1)
                ):
                    flattened[f"{prefix}_{label}"] = float(component)
        output.append(flattened)
    return output


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Extract preliminary experiment statistics from a place_estimation rosbag"
        )
    )
    parser.add_argument("bag", help="input .bag file")
    parser.add_argument("output_directory", help="directory for CSV/JSON output")
    parser.add_argument("--yolo-stride", type=int, default=12)
    parser.add_argument("--meta-stride", type=int, default=3)
    parser.add_argument("--current-min-variance", type=float, default=1.0e-8)
    parser.add_argument("--bias-stability-tolerance", type=float, default=0.002)
    parser.add_argument("--stable-updates", type=int, default=3)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    try:
        import rosbag
    except ImportError as error:
        raise SystemExit(
            "rosbag Python module is unavailable; source the ROS Noetic setup first"
        ) from error

    if arguments.yolo_stride <= 0 or arguments.meta_stride <= 0:
        raise SystemExit("candidate strides must be positive")
    if arguments.current_min_variance <= 0.0:
        raise SystemExit("--current-min-variance must be positive")
    if arguments.bias_stability_tolerance <= 0.0:
        raise SystemExit("--bias-stability-tolerance must be positive")
    if arguments.stable_updates <= 0:
        raise SystemExit("--stable-updates must be positive")

    extractor = BagExtractor(arguments.yolo_stride, arguments.meta_stride)
    topics = [
        "/P_current", "/P_tf", "/P_pred", "/Sigma_pred",
        "/P_yolo", "/P_meta", "/P_place", "/Sigma_place",
        "/observation_status", "/observation_distances",
        "/yolo_candidate_distances", "/meta_candidate_distances",
        "/used_physical_observation", "/kl_evaluation",
        "/kl_adaptation_summary", "/learned_bias_current",
        "/learned_bias_tf", "/learned_covariance_current",
        "/learned_covariance_tf",
    ]
    with rosbag.Bag(arguments.bag, "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            extractor.consume(topic, message, bag_time)

    output_directory = Path(arguments.output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    trials = sorted(
        extractor.trials.values(), key=lambda row: int(row["operation_id"])
    )
    learning = sorted(
        extractor.learning.values(), key=lambda row: int(row["sample_count"])
    )
    write_dictionary_csv(output_directory / "trials.csv", trials)
    write_dictionary_csv(
        output_directory / "learning_history.csv",
        learning_csv_rows(learning),
    )
    report = build_parameter_report(
        trials,
        learning,
        current_min_variance=arguments.current_min_variance,
        bias_stability_tolerance_m=arguments.bias_stability_tolerance,
        stable_updates=arguments.stable_updates,
    )
    report["source_bag"] = str(Path(arguments.bag).resolve())
    report["final_adaptation_summary"] = extractor.final_adaptation_summary
    with (output_directory / "parameter_report.json").open(
        "w", encoding="utf-8"
    ) as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    print(
        f"Wrote {len(trials)} trials and {len(learning)} learning updates to "
        f"{output_directory}"
    )


if __name__ == "__main__":
    main()
