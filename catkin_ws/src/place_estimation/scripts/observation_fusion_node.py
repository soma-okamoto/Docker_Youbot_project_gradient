#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
observation_fusion_node.py

事前分布 N(P_pred, Sigma_pred) を、利用可能なYOLO / Meta観測で更新し、
最終配置位置 N(P_place, Sigma_place) を出力するROS1ノード。

初期開発版:
- 学習機能なし
- 観測バイアス・共分散は固定ROSパラメータ
- YOLOだけ、Metaだけでも更新
- 両方なし／全観測がゲート外なら事前分布を維持
"""

import threading
from typing import Optional, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String


class ObservationFusionNode:
    def __init__(self) -> None:
        self.lock = threading.RLock()

        # Topics
        self.pred_topic = rospy.get_param("~pred_topic", "/P_pred")
        self.pred_cov_topic = rospy.get_param("~pred_cov_topic", "/Sigma_pred")
        self.yolo_topic = rospy.get_param("~yolo_topic", "/P_yolo")
        self.meta_topic = rospy.get_param("~meta_topic", "/P_meta")
        self.place_topic = rospy.get_param("~place_topic", "/P_place")
        self.place_cov_topic = rospy.get_param("~place_cov_topic", "/Sigma_place")
        self.status_topic = rospy.get_param("~status_topic", "/observation_status")
        self.distance_topic = rospy.get_param(
            "~distance_topic", "/observation_distances"
        )

        # Frames
        self.expected_frame = rospy.get_param(
            "~expected_frame", "base_footprint"
        )
        self.output_frame = rospy.get_param(
            "~output_frame", self.expected_frame
        )

        # Sensors
        self.enable_yolo = bool(rospy.get_param("~enable_yolo", True))
        self.meta_message_type = str(
            rospy.get_param("~meta_message_type", "disabled")
        ).lower()
        if self.meta_message_type not in {
            "disabled", "float32_multi_array", "pose_stamped"
        }:
            raise ValueError(
                "~meta_message_type: disabled, float32_multi_array, "
                "pose_stamped のいずれかを指定してください"
            )
        self.enable_meta = self.meta_message_type != "disabled"

        self.yolo_xyz_indices = self._load_indices(
            "~yolo_xyz_indices", [0, 1, 2]
        )
        self.meta_xyz_indices = self._load_indices(
            "~meta_xyz_indices", [0, 1, 2]
        )

        # Fixed error model
        self.min_variance = float(rospy.get_param("~min_variance", 1.0e-8))
        self.bias_yolo = self._load_vector("~bias_yolo", [0.0, 0.0, 0.0])
        self.bias_meta = self._load_vector("~bias_meta", [0.0, 0.0, 0.0])
        self.cov_yolo = self._load_covariance(
            "~covariance_yolo", "~sigma_yolo", [0.03, 0.03, 0.04]
        )
        self.cov_meta = self._load_covariance(
            "~covariance_meta", "~sigma_meta", [0.02, 0.02, 0.03]
        )

        # Gates
        self.gate_yolo = float(rospy.get_param("~gate_yolo", 7.815))
        self.gate_meta = float(rospy.get_param("~gate_meta", 7.815))
        self.gate_yolo_meta = float(
            rospy.get_param("~gate_yolo_meta", 7.815)
        )

        # Waiting / fixed CI weights
        self.observation_timeout = float(
            rospy.get_param("~observation_timeout", 2.0)
        )
        if self.observation_timeout <= 0.0:
            raise ValueError("~observation_timeout は正数にしてください")

        self.single_weight_prior = float(
            rospy.get_param("~single_weight_prior", 0.5)
        )
        if not 0.0 <= self.single_weight_prior <= 1.0:
            raise ValueError("~single_weight_prior は0～1にしてください")

        self.both_weights = self._load_weights(
            "~both_weights", [0.34, 0.33, 0.33]
        )

        # Pending prior pair
        self.pending_pred: Optional[np.ndarray] = None
        self.pending_pred_header = None
        self.pending_cov: Optional[np.ndarray] = None

        # Active operation
        self.active = False
        self.operation_id = 0
        self.prior: Optional[np.ndarray] = None
        self.prior_cov: Optional[np.ndarray] = None
        self.prior_header = None
        self.yolo: Optional[np.ndarray] = None
        self.meta: Optional[np.ndarray] = None
        self.timer = None

        # Publishers
        self.place_pub = rospy.Publisher(
            self.place_topic, PoseStamped, queue_size=10
        )
        self.place_cov_pub = rospy.Publisher(
            self.place_cov_topic, Float32MultiArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10
        )
        self.distance_pub = rospy.Publisher(
            self.distance_topic, Float32MultiArray, queue_size=10
        )

        # Subscribers
        self.pred_sub = rospy.Subscriber(
            self.pred_topic, PoseStamped, self._pred_cb, queue_size=10
        )
        self.pred_cov_sub = rospy.Subscriber(
            self.pred_cov_topic,
            Float32MultiArray,
            self._pred_cov_cb,
            queue_size=10,
        )

        self.yolo_sub = None
        if self.enable_yolo:
            self.yolo_sub = rospy.Subscriber(
                self.yolo_topic,
                Float32MultiArray,
                self._yolo_cb,
                queue_size=10,
            )

        self.meta_sub = None
        if self.meta_message_type == "float32_multi_array":
            self.meta_sub = rospy.Subscriber(
                self.meta_topic,
                Float32MultiArray,
                self._meta_array_cb,
                queue_size=10,
            )
        elif self.meta_message_type == "pose_stamped":
            self.meta_sub = rospy.Subscriber(
                self.meta_topic,
                PoseStamped,
                self._meta_pose_cb,
                queue_size=10,
            )

        rospy.loginfo("ObservationFusionNode started")
        rospy.loginfo(
            "YOLO=%s, Meta=%s, timeout=%.2f s",
            self.enable_yolo,
            self.meta_message_type,
            self.observation_timeout,
        )

    @staticmethod
    def _load_indices(name: str, default: Sequence[int]) -> Tuple[int, int, int]:
        values = rospy.get_param(name, list(default))
        if len(values) != 3:
            raise ValueError(f"{name} は3要素にしてください")
        indices = tuple(int(v) for v in values)
        if min(indices) < 0:
            raise ValueError(f"{name} に負の添字は使えません")
        return indices

    @staticmethod
    def _load_vector(name: str, default: Sequence[float]) -> np.ndarray:
        value = np.asarray(rospy.get_param(name, list(default)), dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} は有限な3要素にしてください")
        return value

    @staticmethod
    def _load_weights(name: str, default: Sequence[float]) -> np.ndarray:
        value = np.asarray(rospy.get_param(name, list(default)), dtype=float)
        if value.shape != (3,) or np.any(value < 0.0):
            raise ValueError(f"{name} は非負の3要素にしてください")
        total = float(value.sum())
        if total <= 0.0:
            raise ValueError(f"{name} の合計を正数にしてください")
        return value / total

    def _load_covariance(
        self,
        covariance_name: str,
        sigma_name: str,
        default_sigma: Sequence[float],
    ) -> np.ndarray:
        if rospy.has_param(covariance_name):
            value = np.asarray(rospy.get_param(covariance_name), dtype=float)
            if value.size != 9:
                raise ValueError(f"{covariance_name} は9要素にしてください")
            covariance = value.reshape(3, 3)
        else:
            sigma = np.asarray(
                rospy.get_param(sigma_name, list(default_sigma)), dtype=float
            )
            if sigma.shape != (3,) or np.any(sigma <= 0.0):
                raise ValueError(f"{sigma_name} は正の3要素にしてください")
            covariance = np.diag(sigma ** 2)
        return self._regularize(covariance)

    def _regularize(self, covariance: np.ndarray) -> np.ndarray:
        covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
        covariance = 0.5 * (covariance + covariance.T)
        values, vectors = np.linalg.eigh(covariance)
        values = np.maximum(values, self.min_variance)
        return vectors @ np.diag(values) @ vectors.T

    @staticmethod
    def _valid_position(position: np.ndarray) -> bool:
        return position.shape == (3,) and bool(np.all(np.isfinite(position)))

    @staticmethod
    def _pose_position(message: PoseStamped) -> np.ndarray:
        return np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=float,
        )

    @staticmethod
    def _array_position(
        message: Float32MultiArray,
        indices: Tuple[int, int, int],
        topic: str,
    ) -> np.ndarray:
        required = max(indices) + 1
        if len(message.data) < required:
            raise ValueError(
                f"{topic}: XYZ添字{indices}には最低{required}要素必要です"
            )
        return np.array([message.data[i] for i in indices], dtype=float)

    def _pred_cb(self, message: PoseStamped) -> None:
        position = self._pose_position(message)
        if not self._valid_position(position):
            rospy.logerr_throttle(2.0, "/P_pred にNaN/Infがあります")
            return
        if self.expected_frame and message.header.frame_id != self.expected_frame:
            rospy.logerr_throttle(
                2.0,
                "P_pred frame mismatch: expected=%s, received=%s",
                self.expected_frame,
                message.header.frame_id,
            )
            return

        with self.lock:
            self.pending_pred = position
            self.pending_pred_header = message.header
            self._try_start_locked()

    def _pred_cov_cb(self, message: Float32MultiArray) -> None:
        if len(message.data) != 9:
            rospy.logerr_throttle(2.0, "/Sigma_pred は9要素必要です")
            return
        covariance = self._regularize(
            np.asarray(message.data, dtype=float).reshape(3, 3)
        )
        with self.lock:
            self.pending_cov = covariance
            self._try_start_locked()

    def _try_start_locked(self) -> None:
        if self.pending_pred is None or self.pending_cov is None:
            return

        if self.active:
            rospy.logwarn(
                "新しい事前分布を受信したため、操作%dを先に確定します",
                self.operation_id,
            )
            self._finalize_locked("new_prior")

        self.operation_id += 1
        self.active = True
        self.prior = self.pending_pred.copy()
        self.prior_cov = self.pending_cov.copy()
        self.prior_header = self.pending_pred_header
        self.pending_pred = None
        self.pending_cov = None
        self.pending_pred_header = None
        self.yolo = None
        self.meta = None

        self.timer = rospy.Timer(
            rospy.Duration(self.observation_timeout),
            self._timeout_cb,
            oneshot=True,
        )
        rospy.loginfo(
            "Operation %d started: P_pred=%s",
            self.operation_id,
            np.array2string(self.prior, precision=6),
        )

        if not self.enable_yolo and not self.enable_meta:
            self._finalize_locked("sensors_disabled")

    def _yolo_cb(self, message: Float32MultiArray) -> None:
        try:
            position = self._array_position(
                message, self.yolo_xyz_indices, self.yolo_topic
            )
        except ValueError as error:
            rospy.logerr_throttle(2.0, str(error))
            return
        self._accept_observation("yolo", position - self.bias_yolo)

    def _meta_array_cb(self, message: Float32MultiArray) -> None:
        try:
            position = self._array_position(
                message, self.meta_xyz_indices, self.meta_topic
            )
        except ValueError as error:
            rospy.logerr_throttle(2.0, str(error))
            return
        self._accept_observation("meta", position - self.bias_meta)

    def _meta_pose_cb(self, message: PoseStamped) -> None:
        if self.expected_frame and message.header.frame_id != self.expected_frame:
            rospy.logerr_throttle(
                2.0,
                "P_meta frame mismatch: expected=%s, received=%s",
                self.expected_frame,
                message.header.frame_id,
            )
            return
        self._accept_observation(
            "meta", self._pose_position(message) - self.bias_meta
        )

    def _accept_observation(self, sensor: str, position: np.ndarray) -> None:
        if not self._valid_position(position):
            rospy.logerr_throttle(2.0, "%s観測にNaN/Infがあります", sensor)
            return
        with self.lock:
            if not self.active:
                rospy.logwarn_throttle(
                    2.0, "%s観測を受信しましたが事前分布がないため破棄します", sensor
                )
                return

            if sensor == "yolo":
                if self.yolo is not None:
                    rospy.logwarn_throttle(
                        2.0, "同一操作内の2個目のYOLO観測は破棄します"
                    )
                    return
                self.yolo = position
            else:
                if self.meta is not None:
                    rospy.logwarn_throttle(
                        2.0, "同一操作内の2個目のMeta観測は破棄します"
                    )
                    return
                self.meta = position

            rospy.loginfo(
                "Operation %d: P_%s=%s",
                self.operation_id,
                sensor,
                np.array2string(position, precision=6),
            )
            self._try_finalize_early_locked()

    def _try_finalize_early_locked(self) -> None:
        yolo_done = not self.enable_yolo or self.yolo is not None
        meta_done = not self.enable_meta or self.meta is not None
        if yolo_done and meta_done:
            self._finalize_locked("all_enabled_observations_received")

    def _timeout_cb(self, _event) -> None:
        with self.lock:
            if self.active:
                self._finalize_locked("timeout")

    @staticmethod
    def _mahalanobis_squared(
        residual: np.ndarray, covariance: np.ndarray
    ) -> float:
        return float(residual.T @ np.linalg.solve(covariance, residual))

    def _gate(
        self,
        observation: np.ndarray,
        observation_covariance: np.ndarray,
        threshold: float,
    ) -> Tuple[bool, float]:
        residual = observation - self.prior
        covariance = self._regularize(
            self.prior_cov + observation_covariance
        )
        distance = self._mahalanobis_squared(residual, covariance)
        return distance <= threshold, distance

    def _ci_two(
        self,
        position_a: np.ndarray,
        covariance_a: np.ndarray,
        position_b: np.ndarray,
        covariance_b: np.ndarray,
        weight_a: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        weight_a = float(np.clip(weight_a, 0.0, 1.0))
        weight_b = 1.0 - weight_a
        info_a = np.linalg.inv(covariance_a)
        info_b = np.linalg.inv(covariance_b)
        information = weight_a * info_a + weight_b * info_b
        covariance = self._regularize(np.linalg.inv(information))
        vector = (
            weight_a * (info_a @ position_a)
            + weight_b * (info_b @ position_b)
        )
        return covariance @ vector, covariance

    def _ci_three(
        self,
        positions: Sequence[np.ndarray],
        covariances: Sequence[np.ndarray],
    ) -> Tuple[np.ndarray, np.ndarray]:
        information = np.zeros((3, 3), dtype=float)
        vector = np.zeros(3, dtype=float)
        for position, covariance, weight in zip(
            positions, covariances, self.both_weights
        ):
            inv_covariance = np.linalg.inv(covariance)
            information += weight * inv_covariance
            vector += weight * (inv_covariance @ position)
        covariance = self._regularize(np.linalg.inv(information))
        return covariance @ vector, covariance

    def _finalize_locked(self, trigger: str) -> None:
        if not self.active:
            return

        if self.timer is not None:
            try:
                self.timer.shutdown()
            except Exception:
                pass
            self.timer = None

        d2_yolo = np.nan
        d2_meta = np.nan
        d2_pair = np.nan
        yolo_valid = False
        meta_valid = False

        try:
            if self.yolo is not None:
                yolo_valid, d2_yolo = self._gate(
                    self.yolo, self.cov_yolo, self.gate_yolo
                )
            if self.meta is not None:
                meta_valid, d2_meta = self._gate(
                    self.meta, self.cov_meta, self.gate_meta
                )

            position = self.prior.copy()
            covariance = self.prior_cov.copy()
            status = "prior_only"

            if yolo_valid and meta_valid:
                pair_residual = self.yolo - self.meta
                pair_covariance = self._regularize(
                    self.cov_yolo + self.cov_meta
                )
                d2_pair = self._mahalanobis_squared(
                    pair_residual, pair_covariance
                )

                if d2_pair <= self.gate_yolo_meta:
                    position, covariance = self._ci_three(
                        [self.prior, self.yolo, self.meta],
                        [self.prior_cov, self.cov_yolo, self.cov_meta],
                    )
                    status = "both_valid"
                elif d2_yolo <= d2_meta:
                    position, covariance = self._ci_two(
                        self.prior,
                        self.prior_cov,
                        self.yolo,
                        self.cov_yolo,
                        self.single_weight_prior,
                    )
                    status = "sensor_mismatch_yolo_selected"
                else:
                    position, covariance = self._ci_two(
                        self.prior,
                        self.prior_cov,
                        self.meta,
                        self.cov_meta,
                        self.single_weight_prior,
                    )
                    status = "sensor_mismatch_meta_selected"

            elif yolo_valid:
                position, covariance = self._ci_two(
                    self.prior,
                    self.prior_cov,
                    self.yolo,
                    self.cov_yolo,
                    self.single_weight_prior,
                )
                status = "yolo_only"

            elif meta_valid:
                position, covariance = self._ci_two(
                    self.prior,
                    self.prior_cov,
                    self.meta,
                    self.cov_meta,
                    self.single_weight_prior,
                )
                status = "meta_only"

            elif self.yolo is not None or self.meta is not None:
                status = "all_received_observations_rejected"
            else:
                status = "no_observation"

            self._publish(
                position, covariance, status, d2_yolo, d2_meta, d2_pair
            )
            rospy.loginfo(
                "Operation %d finalized (%s): status=%s, P_place=%s, "
                "d2_yolo=%s, d2_meta=%s, d2_pair=%s",
                self.operation_id,
                trigger,
                status,
                np.array2string(position, precision=6),
                self._distance_text(d2_yolo),
                self._distance_text(d2_meta),
                self._distance_text(d2_pair),
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            rospy.logerr(
                "Fusion failed: %s. Prior is published unchanged.", error
            )
            self._publish(
                self.prior,
                self.prior_cov,
                "fusion_error_prior_used",
                d2_yolo,
                d2_meta,
                d2_pair,
            )
        finally:
            self._reset_locked()

    @staticmethod
    def _distance_text(value: float) -> str:
        return "N/A" if np.isnan(value) else f"{value:.5f}"

    def _publish(
        self,
        position: np.ndarray,
        covariance: np.ndarray,
        status: str,
        d2_yolo: float,
        d2_meta: float,
        d2_pair: float,
    ) -> None:
        pose = PoseStamped()
        if self.prior_header is not None:
            pose.header = self.prior_header
        if self.output_frame:
            pose.header.frame_id = self.output_frame
        if pose.header.stamp == rospy.Time():
            pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0

        self.place_pub.publish(pose)
        self.place_cov_pub.publish(self._covariance_message(covariance))
        self.status_pub.publish(String(data=status))

        distances = Float32MultiArray()
        distances.data = [
            float(d2_yolo), float(d2_meta), float(d2_pair)
        ]
        self.distance_pub.publish(distances)

    @staticmethod
    def _covariance_message(covariance: np.ndarray) -> Float32MultiArray:
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label="rows", size=3, stride=9),
            MultiArrayDimension(label="columns", size=3, stride=3),
        ]
        message.data = (
            np.asarray(covariance, dtype=np.float32).reshape(-1).tolist()
        )
        return message

    def _reset_locked(self) -> None:
        self.active = False
        self.prior = None
        self.prior_cov = None
        self.prior_header = None
        self.yolo = None
        self.meta = None
        self.timer = None


def main() -> None:
    rospy.init_node("observation_fusion_node")
    try:
        ObservationFusionNode()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("ObservationFusionNode failed: %s", error)


if __name__ == "__main__":
    main()
