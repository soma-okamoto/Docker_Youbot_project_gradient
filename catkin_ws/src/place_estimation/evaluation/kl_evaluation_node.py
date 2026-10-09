#!/usr/bin/env python3
"""ROS node evaluating KL divergence between prior and posterior Gaussians."""
from collections import deque
import json
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String

from gaussian_kl import evaluate_prior_posterior
from kl_trend import evaluate_adaptation_history


class KLEvaluationNode:
    METRIC_LABELS = [
        "operation_id",
        "kl_prior_to_posterior",
        "kl_posterior_to_prior",
        "symmetric_kl",
        "mean_shift_m",
        "prior_to_posterior_mean_component",
        "prior_to_posterior_covariance_component",
        "running_mean_kl_prior_to_posterior",
        "running_std_kl_prior_to_posterior",
        "ema_kl_prior_to_posterior",
    ]

    def __init__(self):
        self.lock = threading.RLock()
        self.prior_pose_topic = rospy.get_param("~prior_pose_topic", "/P_pred")
        self.prior_cov_topic = rospy.get_param(
            "~prior_cov_topic", "/Sigma_pred"
        )
        self.posterior_pose_topic = rospy.get_param(
            "~posterior_pose_topic", "/P_place"
        )
        self.posterior_cov_topic = rospy.get_param(
            "~posterior_cov_topic", "/Sigma_place"
        )
        self.metrics_topic = rospy.get_param(
            "~metrics_topic", "/kl_evaluation"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/kl_evaluation_status"
        )
        self.summary_topic = rospy.get_param(
            "~summary_topic", "/kl_adaptation_summary"
        )
        self.expected_frame = rospy.get_param(
            "~expected_frame", "base_footprint"
        )
        self.ema_alpha = float(rospy.get_param("~ema_alpha", 0.20))
        self.max_pending_operations = int(
            rospy.get_param("~max_pending_operations", 100)
        )
        self.trend_window_size = int(
            rospy.get_param("~trend_window_size", 3)
        )
        self.trend_min_samples = int(
            rospy.get_param("~trend_min_samples", 6)
        )
        self.relative_change_threshold = float(
            rospy.get_param("~relative_change_threshold", 0.10)
        )
        self.normalized_slope_threshold = float(
            rospy.get_param("~normalized_slope_threshold", 0.02)
        )
        if not np.isfinite(self.ema_alpha) or not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("~ema_alpha must be in (0, 1]")
        if self.max_pending_operations <= 0:
            raise ValueError("~max_pending_operations must be positive")
        if self.trend_window_size <= 0:
            raise ValueError("~trend_window_size must be positive")
        if self.trend_min_samples < 2:
            raise ValueError("~trend_min_samples must be at least 2")
        if not 0.0 <= self.relative_change_threshold < 1.0:
            raise ValueError("~relative_change_threshold must be in [0, 1)")
        if self.normalized_slope_threshold < 0.0:
            raise ValueError("~normalized_slope_threshold must be non-negative")

        self.pose_queues = {"prior": deque(), "posterior": deque()}
        self.covariance_queues = {"prior": deque(), "posterior": deque()}
        self.distributions = {"prior": {}, "posterior": {}}

        self.running_count = 0
        self.running_mean = 0.0
        self.running_m2 = 0.0
        self.ema = None
        self.history = []
        self.last_summary = None

        self.metrics_pub = rospy.Publisher(
            self.metrics_topic, Float64MultiArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10
        )
        self.summary_pub = rospy.Publisher(
            self.summary_topic, String, queue_size=1, latch=True
        )

        self.subscribers = [
            rospy.Subscriber(
                self.prior_pose_topic,
                PoseStamped,
                lambda message: self._pose_callback("prior", message),
                queue_size=100,
            ),
            rospy.Subscriber(
                self.prior_cov_topic,
                Float32MultiArray,
                lambda message: self._covariance_callback("prior", message),
                queue_size=100,
            ),
            rospy.Subscriber(
                self.posterior_pose_topic,
                PoseStamped,
                lambda message: self._pose_callback("posterior", message),
                queue_size=100,
            ),
            rospy.Subscriber(
                self.posterior_cov_topic,
                Float32MultiArray,
                lambda message: self._covariance_callback(
                    "posterior", message
                ),
                queue_size=100,
            ),
        ]
        rospy.loginfo(
            "KLEvaluationNode started: D_KL(prior||posterior) on %s",
            self.metrics_topic,
        )
        rospy.on_shutdown(self._log_final_summary)

    @staticmethod
    def _position(message):
        return np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )

    def _pose_callback(self, distribution, message):
        if self.expected_frame and message.header.frame_id != self.expected_frame:
            self._publish_error(
                f"{distribution} frame mismatch: {message.header.frame_id}"
            )
            return
        position = self._position(message)
        if not np.all(np.isfinite(position)):
            self._publish_error(f"{distribution} position contains NaN/Inf")
            return
        with self.lock:
            self.pose_queues[distribution].append(
                (int(message.header.seq), position)
            )
            self._form_distribution_locked(distribution)

    def _covariance_callback(self, distribution, message):
        if len(message.data) != 9:
            self._publish_error(
                f"{distribution} covariance must contain 9 values"
            )
            return
        covariance = np.asarray(message.data, dtype=float).reshape(3, 3)
        with self.lock:
            self.covariance_queues[distribution].append(covariance)
            self._form_distribution_locked(distribution)

    def _form_distribution_locked(self, distribution):
        poses = self.pose_queues[distribution]
        covariances = self.covariance_queues[distribution]
        while poses and covariances:
            operation_id, position = poses.popleft()
            covariance = covariances.popleft()
            self.distributions[distribution][operation_id] = (
                position, covariance
            )
            self._trim_pending_locked(distribution)
            self._evaluate_if_ready_locked(operation_id)

    def _trim_pending_locked(self, distribution):
        pending = self.distributions[distribution]
        while len(pending) > self.max_pending_operations:
            del pending[min(pending)]

    def _evaluate_if_ready_locked(self, operation_id):
        if (
            operation_id not in self.distributions["prior"]
            or operation_id not in self.distributions["posterior"]
        ):
            return
        prior_mean, prior_covariance = self.distributions["prior"].pop(
            operation_id
        )
        posterior_mean, posterior_covariance = self.distributions[
            "posterior"
        ].pop(operation_id)
        try:
            metrics = evaluate_prior_posterior(
                prior_mean,
                prior_covariance,
                posterior_mean,
                posterior_covariance,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self._publish_error(f"operation {operation_id}: {error}")
            return

        # The manuscript defines the adaptation index as D_KL(prior||posterior).
        primary = metrics["kl_prior_to_posterior"]
        self.running_count += 1
        delta = primary - self.running_mean
        self.running_mean += delta / float(self.running_count)
        self.running_m2 += delta * (primary - self.running_mean)
        running_std = (
            np.sqrt(self.running_m2 / float(self.running_count - 1))
            if self.running_count >= 2
            else 0.0
        )
        self.ema = (
            primary
            if self.ema is None
            else self.ema_alpha * primary + (1.0 - self.ema_alpha) * self.ema
        )

        values = [
            float(operation_id),
            metrics["kl_prior_to_posterior"],
            metrics["kl_posterior_to_prior"],
            metrics["symmetric_kl"],
            metrics["mean_shift_m"],
            metrics["prior_to_posterior_mean_component"],
            metrics["prior_to_posterior_covariance_component"],
            self.running_mean,
            float(running_std),
            float(self.ema),
        ]
        message = Float64MultiArray(data=values)
        self.metrics_pub.publish(message)

        payload = dict(zip(self.METRIC_LABELS, values))
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.history.append(
            {
                "operation_id": operation_id,
                "kl": metrics["kl_prior_to_posterior"],
                "mean_shift": metrics["mean_shift_m"],
                "mean_component": metrics[
                    "prior_to_posterior_mean_component"
                ],
                "covariance_component": metrics[
                    "prior_to_posterior_covariance_component"
                ],
            }
        )
        summary = evaluate_adaptation_history(
            self.history,
            window_size=self.trend_window_size,
            min_samples=self.trend_min_samples,
            relative_change_threshold=self.relative_change_threshold,
            normalized_slope_threshold=self.normalized_slope_threshold,
        )
        self.last_summary = summary
        self.summary_pub.publish(
            String(data=json.dumps(summary, sort_keys=True))
        )
        rospy.loginfo(
            "KL operation %d: prior||posterior=%.6f, posterior||prior=%.6f, "
            "symmetric=%.6f, mean_shift=%.6f m",
            operation_id,
            metrics["kl_prior_to_posterior"],
            metrics["kl_posterior_to_prior"],
            metrics["symmetric_kl"],
            metrics["mean_shift_m"],
        )

    def _log_final_summary(self):
        if self.last_summary is None:
            return
        kl_trend = self.last_summary["kl_trend"]
        rospy.loginfo(
            "Final KL adaptation summary: samples=%d, state=%s, "
            "latest=%.6f, relative_improvement=%.3f",
            self.last_summary["sample_count"],
            self.last_summary["adaptation_state"],
            kl_trend["latest"],
            kl_trend["relative_improvement"],
        )

    def _publish_error(self, description):
        rospy.logerr_throttle(2.0, "KL evaluation error: %s", description)
        self.status_pub.publish(
            String(data=json.dumps({"error": description}, sort_keys=True))
        )


def main():
    rospy.init_node("kl_evaluation_node")
    try:
        KLEvaluationNode()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("KLEvaluationNode failed: %s", error)


if __name__ == "__main__":
    main()
