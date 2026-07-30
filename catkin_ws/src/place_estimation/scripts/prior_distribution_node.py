#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
prior_distribution_node.py

各Place操作につき1回だけ届く以下の位置をCIで融合し、
事前分布 N(P_pred, Sigma_pred) を生成するROS1ノード。

Input
  /P_current : std_msgs/Float32MultiArray  [x, y, z]
  /P_tf      : geometry_msgs/PoseStamped

Output
  /P_pred     : geometry_msgs/PoseStamped
  /Sigma_pred : std_msgs/Float32MultiArray
                [Sxx,Sxy,Sxz,Syx,Syy,Syz,Szx,Szy,Szz]

方針
  - P_current, P_tf は過去と平均しない。
  - 今回受信したXYZを、そのPlace操作の位置推定値として使う。
  - バイアスと共分散はROSパラメータから読み込む。
  - 初回はデフォルト値を使用し、将来は逐次学習した値へ差し替える。
"""

import threading
from typing import Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class PriorDistributionNode:
    def __init__(self) -> None:
        self.lock = threading.RLock()

        # Topic settings
        self.current_topic = rospy.get_param("~current_topic", "/P_current")
        self.tf_topic = rospy.get_param("~tf_topic", "/P_tf")
        self.pred_topic = rospy.get_param("~pred_topic", "/P_pred")
        self.cov_topic = rospy.get_param("~cov_topic", "/Sigma_pred")

        # P_currentにはHeaderがないため、P_tfと同じ座標系であることを前提とする。
        self.expected_frame = rospy.get_param("~expected_frame", "")
        self.output_frame = rospy.get_param("~output_frame", self.expected_frame)

        # 数値安定化
        self.min_variance = float(rospy.get_param("~min_variance", 1.0e-8))

        # 現時点で使用する誤差モデル
        self.bias_current = self._load_vector(
            "~bias_current", [0.0, 0.0, 0.0]
        )
        self.bias_tf = self._load_vector(
            "~bias_tf", [0.0, 0.0, 0.0]
        )

        self.cov_current = self._load_covariance(
            covariance_param="~covariance_current",
            sigma_param="~sigma_current",
            default_sigma=[0.05, 0.05, 0.05],
        )
        self.cov_tf = self._load_covariance(
            covariance_param="~covariance_tf",
            sigma_param="~sigma_tf",
            default_sigma=[0.02, 0.02, 0.03],
        )

        # CI weight settings
        self.weight_mode = str(
            rospy.get_param("~ci_weight_mode", "optimize")
        ).lower()
        self.fixed_weight = float(
            rospy.get_param("~ci_fixed_weight", 0.5)
        )
        self.weight_min = float(
            rospy.get_param("~ci_weight_min", 0.0)
        )
        self.weight_max = float(
            rospy.get_param("~ci_weight_max", 1.0)
        )
        self.weight_step = float(
            rospy.get_param("~ci_weight_step", 0.01)
        )
        self.objective = str(
            rospy.get_param("~ci_objective", "trace")
        ).lower()

        self._validate_settings()

        # 1操作分の受信待ちデータ
        self.pending_current: Optional[np.ndarray] = None
        self.pending_tf: Optional[np.ndarray] = None
        self.pending_tf_header = None
        self.operation_count = 0

        self.pred_pub = rospy.Publisher(
            self.pred_topic, PoseStamped, queue_size=10
        )
        self.cov_pub = rospy.Publisher(
            self.cov_topic, Float32MultiArray, queue_size=10
        )

        self.current_sub = rospy.Subscriber(
            self.current_topic,
            Float32MultiArray,
            self._current_callback,
            queue_size=10,
        )
        self.tf_sub = rospy.Subscriber(
            self.tf_topic,
            PoseStamped,
            self._tf_callback,
            queue_size=10,
        )

        rospy.loginfo("PriorDistributionNode started")
        rospy.loginfo("bias_current = %s", self.bias_current)
        rospy.loginfo("bias_tf      = %s", self.bias_tf)
        rospy.loginfo("cov_current =\n%s", self.cov_current)
        rospy.loginfo("cov_tf      =\n%s", self.cov_tf)

    def _validate_settings(self) -> None:
        if self.weight_mode not in ("optimize", "fixed"):
            raise ValueError(
                "~ci_weight_mode must be 'optimize' or 'fixed'"
            )
        if self.objective not in ("trace", "determinant"):
            raise ValueError(
                "~ci_objective must be 'trace' or 'determinant'"
            )
        if not 0.0 <= self.weight_min <= self.weight_max <= 1.0:
            raise ValueError(
                "CI weight must satisfy 0 <= min <= max <= 1"
            )
        if self.weight_step <= 0.0:
            raise ValueError("~ci_weight_step must be positive")

    @staticmethod
    def _load_vector(param_name: str, default) -> np.ndarray:
        vector = np.asarray(
            rospy.get_param(param_name, default),
            dtype=np.float64,
        )
        if vector.shape != (3,):
            raise ValueError(
                f"{param_name} must contain exactly 3 values"
            )
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{param_name} contains NaN or Inf")
        return vector

    def _load_covariance(
        self,
        covariance_param: str,
        sigma_param: str,
        default_sigma,
    ) -> np.ndarray:
        if rospy.has_param(covariance_param):
            values = np.asarray(
                rospy.get_param(covariance_param),
                dtype=np.float64,
            )
            if values.size != 9:
                raise ValueError(
                    f"{covariance_param} must contain 9 values"
                )
            covariance = values.reshape(3, 3)
        else:
            sigma = np.asarray(
                rospy.get_param(sigma_param, default_sigma),
                dtype=np.float64,
            )
            if sigma.shape != (3,) or np.any(sigma <= 0.0):
                raise ValueError(
                    f"{sigma_param} must contain "
                    "3 positive standard deviations"
                )
            covariance = np.diag(np.square(sigma))

        return self._regularize_covariance(covariance)

    def _regularize_covariance(
        self,
        covariance: np.ndarray,
    ) -> np.ndarray:
        covariance = np.asarray(
            covariance,
            dtype=np.float64,
        ).reshape(3, 3)

        # 数値誤差で非対称になった場合に備えて対称化
        covariance = 0.5 * (covariance + covariance.T)

        # 固有値が0以下になることを防ぐ
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(
            eigenvalues,
            self.min_variance,
        )

        return (
            eigenvectors
            @ np.diag(eigenvalues)
            @ eigenvectors.T
        )

    @staticmethod
    def _pose_position(
        message: PoseStamped,
    ) -> np.ndarray:
        return np.array(
            [
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _is_valid_position(
        position: np.ndarray,
    ) -> bool:
        return (
            position.shape == (3,)
            and bool(np.all(np.isfinite(position)))
        )

    def _current_callback(
        self,
        message: Float32MultiArray,
    ) -> None:
        if len(message.data) < 4:
            rospy.logerr_throttle(
                2.0,
                "/P_current requires at least "
                "4 values [metadata, x, y, z]",
            )
            return

        position = np.asarray(
            message.data[1:4],
            dtype=np.float64,
        )

        if not self._is_valid_position(position):
            rospy.logerr_throttle(
                2.0,
                "/P_current contains NaN or Inf",
            )
            return

        with self.lock:
            if self.pending_current is not None:
                rospy.logwarn_throttle(
                    2.0,
                    "Duplicate P_current received before P_tf; "
                    "keeping the first value",
                )
                return

            self.pending_current = position
            rospy.loginfo(
                "P_current received: %s",
                position,
            )
            self._try_generate_prior_locked()

    def _tf_callback(
        self,
        message: PoseStamped,
    ) -> None:
        position = self._pose_position(message)

        if not self._is_valid_position(position):
            rospy.logerr_throttle(
                2.0,
                "/P_tf contains NaN or Inf",
            )
            return

        if (
            self.expected_frame
            and message.header.frame_id != self.expected_frame
        ):
            rospy.logerr_throttle(
                2.0,
                "P_tf frame mismatch: expected '%s', "
                "received '%s'",
                self.expected_frame,
                message.header.frame_id,
            )
            return

        with self.lock:
            if self.pending_tf is not None:
                rospy.logwarn_throttle(
                    2.0,
                    "Duplicate P_tf received before P_current; "
                    "keeping the first value",
                )
                return

            self.pending_tf = position
            self.pending_tf_header = message.header
            rospy.loginfo(
                "P_tf received: %s",
                position,
            )
            self._try_generate_prior_locked()

    def _try_generate_prior_locked(self) -> None:
        if (
            self.pending_current is None
            or self.pending_tf is None
        ):
            return

        raw_current = self.pending_current.copy()
        raw_tf = self.pending_tf.copy()
        tf_header = self.pending_tf_header

        # 同じ入力を次のPlace操作で再利用しない。
        self.pending_current = None
        self.pending_tf = None
        self.pending_tf_header = None

        self.operation_count += 1

        # e = P_source - P_ref と定義するため、
        # バイアス補正では推定位置からbiasを引く。
        corrected_current = (
            raw_current - self.bias_current
        )
        corrected_tf = raw_tf - self.bias_tf

        try:
            (
                pred_position,
                pred_covariance,
                omega,
            ) = self._ci_fusion(
                corrected_current,
                self.cov_current,
                corrected_tf,
                self.cov_tf,
            )
        except np.linalg.LinAlgError as error:
            rospy.logerr(
                "CI calculation failed: %s",
                error,
            )
            return

        frame_id = self.output_frame
        stamp = rospy.Time.now()

        if tf_header is not None:
            if not frame_id:
                frame_id = tf_header.frame_id
            if tf_header.stamp != rospy.Time():
                stamp = tf_header.stamp

        self._publish_result(
            pred_position,
            pred_covariance,
            frame_id,
            stamp,
        )

        rospy.loginfo(
            "Operation %d: "
            "P_pred=%s, "
            "omega_current=%.3f, "
            "diag(Sigma_pred)=%s",
            self.operation_count,
            np.array2string(
                pred_position,
                precision=5,
            ),
            omega,
            np.array2string(
                np.diag(pred_covariance),
                precision=8,
            ),
        )

    def _ci_fusion(
        self,
        p_current: np.ndarray,
        covariance_current: np.ndarray,
        p_tf: np.ndarray,
        covariance_tf: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        info_current = np.linalg.inv(
            covariance_current
        )
        info_tf = np.linalg.inv(
            covariance_tf
        )

        if self.weight_mode == "fixed":
            candidates = [
                float(
                    np.clip(
                        self.fixed_weight,
                        self.weight_min,
                        self.weight_max,
                    )
                )
            ]
        else:
            candidates = np.arange(
                self.weight_min,
                self.weight_max
                + 0.5 * self.weight_step,
                self.weight_step,
            )

        best_objective = np.inf
        best_result = None

        for omega in candidates:
            omega = float(
                np.clip(
                    omega,
                    self.weight_min,
                    self.weight_max,
                )
            )

            info_pred = (
                omega * info_current
                + (1.0 - omega) * info_tf
            )

            covariance_pred = np.linalg.inv(
                info_pred
            )
            covariance_pred = (
                self._regularize_covariance(
                    covariance_pred
                )
            )

            if self.objective == "determinant":
                sign, value = np.linalg.slogdet(
                    covariance_pred
                )
                if sign <= 0:
                    continue
                objective_value = value
            else:
                objective_value = float(
                    np.trace(covariance_pred)
                )

            info_vector = (
                omega
                * (info_current @ p_current)
                + (1.0 - omega)
                * (info_tf @ p_tf)
            )

            position_pred = (
                covariance_pred @ info_vector
            )

            if objective_value < best_objective:
                best_objective = objective_value
                best_result = (
                    position_pred,
                    covariance_pred,
                    omega,
                )

        if best_result is None:
            raise np.linalg.LinAlgError(
                "No valid CI solution was found"
            )

        return best_result

    def _publish_result(
        self,
        position: np.ndarray,
        covariance: np.ndarray,
        frame_id: str,
        stamp,
    ) -> None:
        pred_message = PoseStamped()
        pred_message.header.frame_id = frame_id
        pred_message.header.stamp = stamp

        pred_message.pose.position.x = float(
            position[0]
        )
        pred_message.pose.position.y = float(
            position[1]
        )
        pred_message.pose.position.z = float(
            position[2]
        )

        # 今回は位置のみを推定する。
        pred_message.pose.orientation.x = 0.0
        pred_message.pose.orientation.y = 0.0
        pred_message.pose.orientation.z = 0.0
        pred_message.pose.orientation.w = 1.0

        covariance_message = Float32MultiArray()
        covariance_message.layout.dim = [
            MultiArrayDimension(
                label="rows",
                size=3,
                stride=9,
            ),
            MultiArrayDimension(
                label="columns",
                size=3,
                stride=3,
            ),
        ]
        covariance_message.layout.data_offset = 0
        covariance_message.data = (
            covariance
            .astype(np.float32)
            .reshape(-1)
            .tolist()
        )

        self.pred_pub.publish(pred_message)
        self.cov_pub.publish(
            covariance_message
        )


def main() -> None:
    rospy.init_node(
        "prior_distribution_node"
    )

    try:
        PriorDistributionNode()
        rospy.spin()
    except (
        ValueError,
        rospy.ROSException,
    ) as error:
        rospy.logfatal(
            "PriorDistributionNode failed: %s",
            error,
        )


if __name__ == "__main__":
    main()
