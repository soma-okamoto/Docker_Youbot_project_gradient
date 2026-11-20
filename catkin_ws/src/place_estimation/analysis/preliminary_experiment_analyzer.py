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

try:
    from predictive_metrics import evaluate_trial, PredictiveHistory, SENSORS
except ModuleNotFoundError as error:
    if error.name != "predictive_metrics":
        raise
    evaluation_directory = Path(__file__).resolve().parents[1] / "evaluation"
    if not evaluation_directory.is_dir():
        import rospkg
        evaluation_directory = Path(rospkg.RosPack().get_path("place_estimation")) / "evaluation"
    sys.path.insert(0, str(evaluation_directory))
    from predictive_metrics import evaluate_trial, PredictiveHistory, SENSORS


class BagExtractor:
    """Collect trial-aligned fields from the package's ROS topics."""

    def __init__(self, yolo_stride: int = 12, meta_stride: int = 12,
                 min_variance: float = 1.e-8):
        self.yolo_stride = yolo_stride
        self.meta_stride = meta_stride
        self.trials: Dict[int, Dict[str, object]] = {}
        self.learning: Dict[int, Dict[str, object]] = {}
        self.pending_current = None
        self.pending_bottle_id = None
        self.pending_tf = None
        self.active_operation_id: Optional[int] = None
        self.prior_covariance_queue = deque()
        self.posterior_covariance_queue = deque()
        self.final_adaptation_summary = None
        self.prior_operations_by_stamp = {}
        self.atomic_prior_operations = set()
        self.min_variance = min_variance
        self.predictive_inputs = {}
        self.predictive_results = {}
        self.predictive_errors = []
        self.experiment_configurations = {}
        self.configuration_errors = []

    @staticmethod
    def _predictive_key(record):
        return (int(record["operation_id"]), int(record["prior_stamp_ns"]), record["frame_id"])

    def _register_prior_stamp(self, operation_id, stamp):
        operations = self.prior_operations_by_stamp.setdefault(stamp, [])
        if operation_id not in operations:
            operations.append(operation_id)

    def _assign_predictive_result(self, result):
        row = self._trial(result["operation_id"])
        row["bottle_id"] = result.get("bottle_id", -1)
        row["prior_stamp_ns"] = result["prior_stamp_ns"]
        row["learning_sample_count_before_trial"] = result.get("learning_sample_count_before_trial")
        row["observation_status"] = result["observation_status"]
        for source, values in result.get("sources", {}).items():
            self._assign_vector(row, source, values["raw_mean"])
            self._assign_vector(row, "source_" + source, values["corrected_mean"])
            self._assign_covariance(row, "source_" + source + "_cov", values["covariance"])
        for model, prior in result.get("priors", {}).items():
            prefix = "pred" if model == "adaptive" else "fixed_pred"
            self._assign_vector(row, prefix, prior["mean"])
            self._assign_covariance(row, prefix + "_cov", prior["covariance"])
        for sensor, scores in result["scores"].items():
            row["predictive_" + sensor + "_status"] = scores["status"]
            score = scores["trial_score"]
            if score is not None:
                row["delta_nll_" + sensor] = score["delta_nll"]
                for model in ("adaptive", "fixed"):
                    row["nll_" + model + "_" + sensor] = score[model]["nll"]
                    row["error_" + model + "_" + sensor + "_m"] = score[model]["position_error_m"]

    def finalize_predictive_evaluation(self):
        """Recover scores from atomic inputs when a live result was not recorded."""
        for key, snapshot in self.predictive_inputs.items():
            if key not in self.predictive_results:
                try:
                    self.predictive_results[key] = evaluate_trial(snapshot, self.min_variance)
                except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as error:
                    self.predictive_errors.append(str(error))
        history = PredictiveHistory()
        for result in sorted(self.predictive_results.values(),
                             key=lambda value: (value["prior_stamp_ns"], value["operation_id"])):
            self._assign_predictive_result(result)
            history.add(result)
        return history.summary()

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

        if topic == '/experiment_configuration':
            try:
                record = json.loads(message.data)
                if (record.get('schema_version') != 1
                        or not isinstance(record.get('recording_id'), str)
                        or not record['recording_id']
                        or not isinstance(record.get('runtime_parameters'), dict)):
                    raise ValueError('Invalid experiment configuration record')
                self.experiment_configurations.setdefault(record['recording_id'], record)
            except (AttributeError, TypeError, ValueError) as error:
                self.configuration_errors.append(str(error))
            return

        if topic in {"/predictive_evaluation_input", "/predictive_evaluation"}:
            try:
                record = json.loads(message.data)
                key = self._predictive_key(record)
                if topic == "/predictive_evaluation_input":
                    self.predictive_inputs[key] = record
                else:
                    self.predictive_results[key] = record
            except (KeyError, TypeError, ValueError) as error:
                self.predictive_errors.append(str(error))
            return

        if topic == "/P_current":
            values = np.asarray(message.data, dtype=float)
            if values.size >= 4 and np.all(np.isfinite(values[1:4])):
                # Element 0 identifies a bottle, not a Place operation.
                self.pending_bottle_id = (
                    int(values[0])
                    if np.isfinite(values[0]) and values[0].is_integer()
                    else None
                )
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
            stamp = message.header.stamp.to_nsec()
            self._register_prior_stamp(operation_id, stamp)
            row = self._trial(operation_id)
            row["prior_time_s"] = timestamp
            if operation_id not in self.atomic_prior_operations:
                self._assign_vector(row, "pred", self._position(message))
            if self.pending_current is not None:
                if operation_id not in self.atomic_prior_operations:
                    self._assign_vector(row, "current", self.pending_current)
                    row["bottle_id"] = self.pending_bottle_id
                self.pending_current = None
                self.pending_bottle_id = None
            if self.pending_tf is not None:
                if operation_id not in self.atomic_prior_operations:
                    self._assign_vector(row, "tf", self.pending_tf)
                self.pending_tf = None
            self.prior_covariance_queue.append(operation_id)
            return

        if topic == "/P_pred_context":
            row = self._trial(message.operation_id)
            row["bottle_id"] = int(message.bottle_id)
            # Old bag schemas/tests may only contain source hypotheses.
            if hasattr(message, "position") and hasattr(message, "covariance"):
                self.atomic_prior_operations.add(message.operation_id)
                self.active_operation_id = message.operation_id
                self._assign_vector(row, "pred", [message.position.x, message.position.y, message.position.z])
                self._assign_covariance(row, "pred_cov", message.covariance)
                row.setdefault("prior_time_s", timestamp)
                self._register_prior_stamp(message.operation_id, message.header.stamp.to_nsec())
            if getattr(message, "fixed_prior_available", False):
                point = message.fixed_position
                self._assign_vector(row, "fixed_pred", [point.x, point.y, point.z])
                self._assign_covariance(row, "fixed_pred_cov", message.fixed_covariance)
                row["learning_sample_count_before_trial"] = message.learning_sample_count
            for name in ("raw_current", "raw_tf", "current", "tf"):
                point = getattr(message, name + "_position")
                self._assign_vector(row, "source_" + name, [point.x, point.y, point.z])
                if name.startswith("raw_"):
                    self._assign_vector(row, name[4:], [point.x, point.y, point.z])
            for name in ("current", "tf"):
                self._assign_covariance(row, "source_" + name + "_cov",
                                        getattr(message, name + "_covariance"))
            return

        if topic == "/Sigma_pred":
            if self.prior_covariance_queue and len(message.data) == 9:
                operation_id = self.prior_covariance_queue.popleft()
                if operation_id not in self.atomic_prior_operations:
                    self._assign_covariance(
                        self._trial(operation_id), "pred_cov", message.data
                    )
            return

        if topic == "/P_place":
            # A cancelled operation has no output, so posterior Header.seq can
            # differ from the prior operation ID. Match its echoed timestamp.
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            if stamp is not None:
                matches = self.prior_operations_by_stamp.get(stamp.to_nsec(), [])
                if len(matches) != 1:
                    return
                operation_id = matches[0]
            else:
                operation_id = self._operation_id(message, self.active_operation_id)
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
            # Header.seq counts feedback publications, not Place operations.
            # This topic echoes the selected operation's prior timestamp.
            matches = self.prior_operations_by_stamp.get(
                message.header.stamp.to_nsec(), []
            )
            if len(matches) != 1:
                return
            operation_id = matches[0]
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

        if topic == "/observation_reference_distances" and self.active_operation_id:
            labels = [sensor + "_to_" + reference + "_d2"
                      for sensor in ("yolo", "meta") for reference in ("prior", "current", "tf")]
            for label, value in zip(labels, message.data):
                self._trial(self.active_operation_id)[label] = (
                    float(value) if np.isfinite(value) else None
                )
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


def predictive_csv_rows(results):
    """One row per Place operation and sensor, including unscored trials."""
    rows = []
    for result in results:
        for sensor in SENSORS:
            data = result["sensors"][sensor]
            scores = result["scores"][sensor]
            row = {
                "operation_id": result["operation_id"],
                "bottle_id": result.get("bottle_id", -1),
                "prior_stamp_ns": result["prior_stamp_ns"],
                "frame_id": result["frame_id"],
                "sensor": sensor,
                "learning_sample_count_before_trial": result.get("learning_sample_count_before_trial"),
                "observation_status": result["observation_status"],
                "evaluation_status": scores["status"],
                "quality_candidate_count": scores["quality_candidate_count"],
                "observation_received": data["received"],
                "sensor_enabled": data["enabled"],
                "algorithm_selected_index": data["algorithm_selected_index"],
                "selected_for_feedback": data["selected_for_feedback"],
                "target_identity_verified": False,
                "error_independence_verified": False,
                "priors": result.get("priors", {}),
                "quality_candidates": data["quality_candidates"],
                "candidate_scores": scores["candidate_scores"],
                "selected_diagnostic": scores["selected_diagnostic"],
            }
            score = scores["trial_score"]
            if score is not None:
                row["delta_nll"] = score["delta_nll"]
                row["delta_squared_error_m2"] = score["delta_squared_error_m2"]
                for model in ("adaptive", "fixed"):
                    for name, value in score[model].items():
                        row[model + "_" + name] = value
            rows.append(row)
    return rows


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Extract preliminary experiment statistics from a place_estimation rosbag"
        )
    )
    parser.add_argument("bag", help="input .bag file")
    parser.add_argument("output_directory", help="directory for CSV/JSON output")
    parser.add_argument("--yolo-stride", type=int, default=12)
    parser.add_argument("--meta-stride", type=int, default=12,
                        help="Meta record length; use 3 for legacy XYZ-only bags")
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

    extractor = BagExtractor(arguments.yolo_stride, arguments.meta_stride,
                             arguments.current_min_variance)
    topics = [
        "/P_current", "/P_tf", "/P_pred", "/Sigma_pred",
        "/P_pred_context", "/observation_reference_distances",
        "/P_yolo", "/P_meta", "/P_place", "/Sigma_place",
        "/observation_status", "/observation_distances",
        "/yolo_candidate_distances", "/meta_candidate_distances",
        "/used_physical_observation", "/kl_evaluation",
        "/kl_adaptation_summary", "/learned_bias_current",
        "/learned_bias_tf", "/learned_covariance_current",
        "/learned_covariance_tf",
        "/predictive_evaluation_input", "/predictive_evaluation",
        "/experiment_configuration",
    ]
    with rosbag.Bag(arguments.bag, "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=topics):
            extractor.consume(topic, message, bag_time)

    output_directory = Path(arguments.output_directory)
    predictive_summary = extractor.finalize_predictive_evaluation()
    output_directory.mkdir(parents=True, exist_ok=True)
    configurations = list(extractor.experiment_configurations.values())
    with (output_directory / 'experiment_configuration.json').open('w', encoding='utf-8') as stream:
        json.dump({'schema_version': 1, 'records': configurations}, stream,
                  ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')
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
    write_dictionary_csv(
        output_directory / "predictive_evaluation.csv",
        predictive_csv_rows(sorted(extractor.predictive_results.values(),
                                   key=lambda value: (value["prior_stamp_ns"], value["operation_id"]))),
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
    report["predictive_evaluation"] = predictive_summary
    report['experiment_configurations'] = [
        {key: record.get(key) for key in (
            'recording_id', 'captured_at_utc', 'git_revision',
            'git_tracked_files_modified', 'runtime_parameters', 'capture_errors',
        )} for record in configurations
    ]
    if not configurations:
        report['warnings'].append(
            'No startup experiment configuration was recorded; current config files '
            'must not be treated as the settings used for this historical bag.'
        )
    elif len(configurations) > 1:
        report['warnings'].append('Multiple startup configuration records were found; review each recording session.')
    report['warnings'].extend('Experiment configuration: ' + error for error in extractor.configuration_errors)
    if not extractor.predictive_results:
        report["warnings"].append(
            "No atomic predictive snapshots/results were recorded; sensor NLL is unavailable."
        )
    report["warnings"].extend("Predictive evaluation: " + error
                              for error in extractor.predictive_errors)
    unmatched_learning = [
        row["operation_id"] for row in learning
        if extractor.trials.get(row["operation_id"], {}).get("used_x") is None
    ]
    if unmatched_learning:
        report["warnings"].append(
            "Learning updates have no timestamp-matched physical observation "
            "for operation IDs %s; their trial association may be incorrect."
            % unmatched_learning
        )
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
