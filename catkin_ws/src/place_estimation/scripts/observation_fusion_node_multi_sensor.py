#!/usr/bin/env python3
# -*- coding: utf-8 -*-



"""
observation_fusion_node.py

事前分布 N(P_pred, Sigma_pred) を、YOLO / Meta の観測候補で更新し、
最終配置位置 N(P_place, Sigma_place) を出力するROS1ノード。

初期開発版:
- 学習機能なし
- YOLOは候補ごとの受信共分散を使用（不正時のみ固定共分散へフォールバック）
- Metaのバイアス・共分散とCI重みは固定ROSパラメータ
- YOLOとMetaの両方で可変個数候補に対応
- 各センサで全候補をマハラノビスゲーティング
- ゲート内で距離最小（関連スコア最大）の候補を選択
- YOLOだけ／Metaだけでも事前分布を更新
- 両方有効なら観測間整合性を確認して3入力CI
- 両方なし／全候補がゲート外なら事前分布を維持

候補入力モード:
- packed:
    1つのFloat32MultiArrayに可変個数の候補をまとめる。
    YOLO例:
    [x,y,z,Sxx,Sxy,Sxz,Syx,Syy,Syz,Szx,Szy,Szz, ...]
- stream:
    observation_timeoutの間に届いた複数メッセージを蓄積する。
    各メッセージは1候補でも複数候補でもよい。

Metaメッセージ型:
- disabled
- float32_multi_array:
    packed / stream の両方に対応
- pose_stamped:
    1メッセージ=1候補。複数候補ならstreamを使用
"""

import threading
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import (
    Float32MultiArray,
    Int32,
    MultiArrayDimension,
    String,
)


class ObservationFusionNode:
    def __init__(self) -> None:
        self.lock = threading.RLock()

        # ================================================================
        # Topics
        # ================================================================
        self.pred_topic = rospy.get_param("~pred_topic", "/P_pred")
        self.pred_cov_topic = rospy.get_param(
            "~pred_cov_topic", "/Sigma_pred"
        )
        self.yolo_topic = rospy.get_param("~yolo_topic", "/P_yolo")
        self.meta_topic = rospy.get_param("~meta_topic", "/P_meta")

        self.place_topic = rospy.get_param("~place_topic", "/P_place")
        self.place_cov_topic = rospy.get_param(
            "~place_cov_topic", "/Sigma_place"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/observation_status"
        )
        self.distance_topic = rospy.get_param(
            "~distance_topic", "/observation_distances"
        )

        self.yolo_distances_topic = rospy.get_param(
            "~yolo_distances_topic", "/yolo_candidate_distances"
        )
        self.yolo_scores_topic = rospy.get_param(
            "~yolo_scores_topic", "/yolo_candidate_scores"
        )
        self.yolo_selected_index_topic = rospy.get_param(
            "~yolo_selected_index_topic", "/yolo_selected_index"
        )

        self.meta_distances_topic = rospy.get_param(
            "~meta_distances_topic", "/meta_candidate_distances"
        )
        self.meta_scores_topic = rospy.get_param(
            "~meta_scores_topic", "/meta_candidate_scores"
        )
        self.meta_selected_index_topic = rospy.get_param(
            "~meta_selected_index_topic", "/meta_selected_index"
        )

        # ================================================================
        # Frames
        # ================================================================
        self.expected_frame = rospy.get_param(
            "~expected_frame", "base_footprint"
        )
        self.output_frame = rospy.get_param(
            "~output_frame", self.expected_frame
        )

        # ================================================================
        # Sensor availability / message types
        # ================================================================
        self.enable_yolo = bool(rospy.get_param("~enable_yolo", True))

        self.meta_message_type = str(
            rospy.get_param("~meta_message_type", "disabled")
        ).lower()
        if self.meta_message_type not in {
            "disabled",
            "float32_multi_array",
            "pose_stamped",
        }:
            raise ValueError(
                "~meta_message_type must be disabled, "
                "float32_multi_array, or pose_stamped"
            )
        self.enable_meta = self.meta_message_type != "disabled"

        # ================================================================
        # Candidate input formats
        # ================================================================
        self.yolo_input_mode = self._load_input_mode(
            "~yolo_input_mode", "packed"
        )
        self.meta_input_mode = self._load_input_mode(
            "~meta_input_mode", "packed"
        )

        self.yolo_candidate_stride = self._load_positive_int(
            "~yolo_candidate_stride", 3
        )
        self.meta_candidate_stride = self._load_positive_int(
            "~meta_candidate_stride", 3
        )

        self.yolo_xyz_indices = self._load_indices(
            "~yolo_xyz_indices", [0, 1, 2]
        )
        self.yolo_covariance_indices = self._load_n_indices(
            "~yolo_covariance_indices",
            [3, 4, 5, 6, 7, 8, 9, 10, 11],
            expected_count=9,
        )
        self.meta_xyz_indices = self._load_indices(
            "~meta_xyz_indices", [0, 1, 2]
        )

        self._validate_indices(
            "~yolo_xyz_indices",
            self.yolo_xyz_indices,
            self.yolo_candidate_stride,
        )
        self._validate_indices(
            "~yolo_covariance_indices",
            self.yolo_covariance_indices,
            self.yolo_candidate_stride,
        )
        self._validate_indices(
            "~meta_xyz_indices",
            self.meta_xyz_indices,
            self.meta_candidate_stride,
        )

        self.max_yolo_candidates = self._load_positive_int(
            "~max_yolo_candidates", 200
        )
        self.max_meta_candidates = self._load_positive_int(
            "~max_meta_candidates", 200
        )

        # ================================================================
        # Fixed error model
        # ================================================================
        self.min_variance = float(
            rospy.get_param("~min_variance", 1.0e-8)
        )
        if self.min_variance <= 0.0:
            raise ValueError("~min_variance must be positive")

        self.bias_yolo = self._load_vector(
            "~bias_yolo", [0.0, 0.0, 0.0]
        )
        self.bias_meta = self._load_vector(
            "~bias_meta", [0.0, 0.0, 0.0]
        )

        self.cov_yolo = self._load_covariance(
            "~covariance_yolo",
            "~sigma_yolo",
            [0.03, 0.03, 0.04],
        )
        self.cov_meta = self._load_covariance(
            "~covariance_meta",
            "~sigma_meta",
            [0.02, 0.02, 0.03],
        )

        # ================================================================
        # Gates
        # ================================================================
        self.gate_yolo = float(
            rospy.get_param("~gate_yolo", 7.815)
        )
        self.gate_meta = float(
            rospy.get_param("~gate_meta", 7.815)
        )
        self.gate_yolo_meta = float(
            rospy.get_param("~gate_yolo_meta", 7.815)
        )

        if min(
            self.gate_yolo,
            self.gate_meta,
            self.gate_yolo_meta,
        ) <= 0.0:
            raise ValueError("gate thresholds must be positive")

        # ================================================================
        # Waiting / fixed CI weights
        # ================================================================
        self.observation_timeout = float(
            rospy.get_param("~observation_timeout", 2.0)
        )
        if self.observation_timeout <= 0.0:
            raise ValueError(
                "~observation_timeout must be positive"
            )

        self.single_weight_prior = float(
            rospy.get_param("~single_weight_prior", 0.5)
        )
        if not 0.0 <= self.single_weight_prior <= 1.0:
            raise ValueError(
                "~single_weight_prior must be in [0, 1]"
            )

        self.both_weights = self._load_weights(
            "~both_weights", [0.34, 0.33, 0.33]
        )

        # ================================================================
        # Pending prior pair
        # ================================================================
        self.pending_pred: Optional[np.ndarray] = None
        self.pending_pred_header = None
        self.pending_cov: Optional[np.ndarray] = None

        # ================================================================
        # Active operation
        # ================================================================
        self.active = False
        self.operation_id = 0

        self.prior: Optional[np.ndarray] = None
        self.prior_cov: Optional[np.ndarray] = None
        self.prior_header = None

        self.yolo_received = False
        self.meta_received = False
        self.yolo_candidates: List[np.ndarray] = []
        self.yolo_covariances: List[np.ndarray] = []
        self.meta_candidates: List[np.ndarray] = []
        self.timer = None

        # ================================================================
        # Publishers
        # ================================================================
        self.place_pub = rospy.Publisher(
            self.place_topic, PoseStamped, queue_size=10
        )
        self.place_cov_pub = rospy.Publisher(
            self.place_cov_topic,
            Float32MultiArray,
            queue_size=10,
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10
        )
        self.distance_pub = rospy.Publisher(
            self.distance_topic,
            Float32MultiArray,
            queue_size=10,
        )

        self.yolo_distances_pub = rospy.Publisher(
            self.yolo_distances_topic,
            Float32MultiArray,
            queue_size=10,
        )
        self.yolo_scores_pub = rospy.Publisher(
            self.yolo_scores_topic,
            Float32MultiArray,
            queue_size=10,
        )
        self.yolo_selected_index_pub = rospy.Publisher(
            self.yolo_selected_index_topic,
            Int32,
            queue_size=10,
        )

        self.meta_distances_pub = rospy.Publisher(
            self.meta_distances_topic,
            Float32MultiArray,
            queue_size=10,
        )
        self.meta_scores_pub = rospy.Publisher(
            self.meta_scores_topic,
            Float32MultiArray,
            queue_size=10,
        )
        self.meta_selected_index_pub = rospy.Publisher(
            self.meta_selected_index_topic,
            Int32,
            queue_size=10,
        )

        # ================================================================
        # Subscribers
        # ================================================================
        self.pred_sub = rospy.Subscriber(
            self.pred_topic,
            PoseStamped,
            self._pred_cb,
            queue_size=10,
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
                self._yolo_array_cb,
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
            "YOLO enabled=%s, mode=%s, stride=%d, "
            "xyz_indices=%s, covariance_indices=%s",
            self.enable_yolo,
            self.yolo_input_mode,
            self.yolo_candidate_stride,
            self.yolo_xyz_indices,
            self.yolo_covariance_indices,
        )
        rospy.loginfo(
            "Meta type=%s, mode=%s, stride=%d, indices=%s",
            self.meta_message_type,
            self.meta_input_mode,
            self.meta_candidate_stride,
            self.meta_xyz_indices,
        )
        rospy.loginfo(
            "observation_timeout=%.3f s",
            self.observation_timeout,
        )

    # ====================================================================
    # Parameter helpers
    # ====================================================================

    @staticmethod
    def _load_input_mode(name: str, default: str) -> str:
        value = str(rospy.get_param(name, default)).lower()
        if value not in {"packed", "stream"}:
            raise ValueError(
                f"{name} must be packed or stream"
            )
        return value

    @staticmethod
    def _load_positive_int(name: str, default: int) -> int:
        value = int(rospy.get_param(name, default))
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    @staticmethod
    def _load_indices(
        name: str,
        default: Sequence[int],
    ) -> Tuple[int, int, int]:
        values = rospy.get_param(name, list(default))
        if len(values) != 3:
            raise ValueError(
                f"{name} must contain 3 indices"
            )
        indices = tuple(int(value) for value in values)
        if min(indices) < 0:
            raise ValueError(
                f"{name} cannot contain negative indices"
            )
        return indices

    @staticmethod
    def _load_n_indices(
        name: str,
        default: Sequence[int],
        expected_count: int,
    ) -> Tuple[int, ...]:
        values = rospy.get_param(name, list(default))
        if len(values) != expected_count:
            raise ValueError(
                f"{name} must contain {expected_count} indices"
            )
        indices = tuple(int(value) for value in values)
        if min(indices) < 0:
            raise ValueError(
                f"{name} cannot contain negative indices"
            )
        if len(set(indices)) != len(indices):
            raise ValueError(
                f"{name} cannot contain duplicate indices"
            )
        return indices

    @staticmethod
    def _validate_indices(
        name: str,
        indices: Sequence[int],
        stride: int,
    ) -> None:
        if max(indices) >= stride:
            raise ValueError(
                f"{name} values must be smaller than stride={stride}"
            )

    @staticmethod
    def _load_vector(
        name: str,
        default: Sequence[float],
    ) -> np.ndarray:
        value = np.asarray(
            rospy.get_param(name, list(default)),
            dtype=float,
        )
        if (
            value.shape != (3,)
            or not np.all(np.isfinite(value))
        ):
            raise ValueError(
                f"{name} must contain 3 finite values"
            )
        return value

    @staticmethod
    def _load_weights(
        name: str,
        default: Sequence[float],
    ) -> np.ndarray:
        value = np.asarray(
            rospy.get_param(name, list(default)),
            dtype=float,
        )
        if value.shape != (3,) or np.any(value < 0.0):
            raise ValueError(
                f"{name} must contain 3 non-negative values"
            )

        total = float(value.sum())
        if total <= 0.0:
            raise ValueError(
                f"{name} must have a positive sum"
            )
        return value / total

    def _load_covariance(
        self,
        covariance_name: str,
        sigma_name: str,
        default_sigma: Sequence[float],
    ) -> np.ndarray:
        if rospy.has_param(covariance_name):
            value = np.asarray(
                rospy.get_param(covariance_name),
                dtype=float,
            )
            if value.size != 9:
                raise ValueError(
                    f"{covariance_name} must contain 9 values"
                )
            covariance = value.reshape(3, 3)
        else:
            sigma = np.asarray(
                rospy.get_param(
                    sigma_name, list(default_sigma)
                ),
                dtype=float,
            )
            if sigma.shape != (3,) or np.any(sigma <= 0.0):
                raise ValueError(
                    f"{sigma_name} must contain "
                    "3 positive values"
                )
            covariance = np.diag(sigma ** 2)

        return self._regularize(covariance)

    def _regularize(
        self,
        covariance: np.ndarray,
    ) -> np.ndarray:
        covariance = np.asarray(
            covariance, dtype=float
        ).reshape(3, 3)
        covariance = 0.5 * (
            covariance + covariance.T
        )

        values, vectors = np.linalg.eigh(covariance)
        values = np.maximum(values, self.min_variance)
        return vectors @ np.diag(values) @ vectors.T

    # ====================================================================
    # Message parsing
    # ====================================================================

    @staticmethod
    def _valid_position(position: np.ndarray) -> bool:
        return (
            position.shape == (3,)
            and bool(np.all(np.isfinite(position)))
        )

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

    def _parse_yolo_candidate_array(
        self,
        message: Float32MultiArray,
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        values = np.asarray(message.data, dtype=float)

        if values.size == 0:
            return (
                np.empty((0, 3), dtype=float),
                [],
            )

        stride = self.yolo_candidate_stride
        if values.size % stride != 0:
            raise ValueError(
                f"{self.yolo_topic}: data length {values.size} "
                f"is not divisible by candidate stride {stride}"
            )

        records = values.reshape(-1, stride)
        positions = []
        covariances = []
        discarded_count = 0
        fallback_count = 0

        for record in records:
            position = record[list(self.yolo_xyz_indices)]

            if not np.all(np.isfinite(position)):
                discarded_count += 1
                continue

            covariance_values = record[
                list(self.yolo_covariance_indices)
            ]

            use_fallback = (
                covariance_values.size != 9
                or not np.all(np.isfinite(covariance_values))
            )

            if not use_fallback:
                covariance_raw = covariance_values.reshape(3, 3)
                covariance_raw = 0.5 * (
                    covariance_raw + covariance_raw.T
                )

                # 分散が0以下のデータは受信共分散として採用しない。
                use_fallback = bool(
                    np.any(np.diag(covariance_raw) <= 0.0)
                )

            if use_fallback:
                covariance = self.cov_yolo.copy()
                fallback_count += 1
            else:
                covariance = self._regularize(covariance_raw)

            positions.append(
                position.astype(float) - self.bias_yolo
            )
            covariances.append(covariance)

        if discarded_count > 0:
            rospy.logwarn(
                "%s: discarded %d candidates containing "
                "invalid XYZ values",
                self.yolo_topic,
                discarded_count,
            )

        if fallback_count > 0:
            rospy.logwarn(
                "%s: used fixed YOLO covariance for %d candidates "
                "because the received covariance was invalid",
                self.yolo_topic,
                fallback_count,
            )

        if not positions:
            return (
                np.empty((0, 3), dtype=float),
                [],
            )

        return np.vstack(positions), covariances

    def _parse_candidate_array(
        self,
        message: Float32MultiArray,
        topic: str,
        stride: int,
        xyz_indices: Tuple[int, int, int],
        bias: np.ndarray,
    ) -> np.ndarray:
        values = np.asarray(message.data, dtype=float)

        if values.size == 0:
            return np.empty((0, 3), dtype=float)

        if values.size % stride != 0:
            raise ValueError(
                f"{topic}: data length {values.size} "
                f"is not divisible by candidate stride {stride}"
            )

        records = values.reshape(-1, stride)
        positions = records[
            :,
            [
                xyz_indices[0],
                xyz_indices[1],
                xyz_indices[2],
            ],
        ]

        finite_mask = np.all(np.isfinite(positions), axis=1)
        invalid_count = int(np.count_nonzero(~finite_mask))
        if invalid_count > 0:
            rospy.logwarn(
                "%s: discarded %d candidates containing NaN/Inf",
                topic,
                invalid_count,
            )

        positions = positions[finite_mask]
        return positions - bias.reshape(1, 3)

    # ====================================================================
    # Prior callbacks
    # ====================================================================

    def _pred_cb(self, message: PoseStamped) -> None:
        position = self._pose_position(message)

        if not self._valid_position(position):
            rospy.logerr_throttle(
                2.0, "/P_pred contains NaN/Inf"
            )
            return

        if (
            self.expected_frame
            and message.header.frame_id != self.expected_frame
        ):
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

    def _pred_cov_cb(
        self, message: Float32MultiArray
    ) -> None:
        if len(message.data) != 9:
            rospy.logerr_throttle(
                2.0, "/Sigma_pred must contain 9 values"
            )
            return

        covariance = self._regularize(
            np.asarray(
                message.data, dtype=float
            ).reshape(3, 3)
        )

        with self.lock:
            self.pending_cov = covariance
            self._try_start_locked()

    def _try_start_locked(self) -> None:
        if (
            self.pending_pred is None
            or self.pending_cov is None
        ):
            return

        if self.active:
            rospy.logwarn(
                "New prior received; finalizing operation %d first",
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

        self.yolo_received = False
        self.meta_received = False
        self.yolo_candidates = []
        self.yolo_covariances = []
        self.meta_candidates = []

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

    # ====================================================================
    # Candidate callbacks and accumulation
    # ====================================================================

    def _yolo_array_cb(
        self, message: Float32MultiArray
    ) -> None:
        try:
            (
                candidates,
                covariances,
            ) = self._parse_yolo_candidate_array(message)
        except ValueError as error:
            rospy.logerr_throttle(2.0, str(error))
            return

        self._accept_candidates(
            sensor="yolo",
            candidates=candidates,
            covariances=covariances,
        )

    def _meta_array_cb(
        self, message: Float32MultiArray
    ) -> None:
        try:
            candidates = self._parse_candidate_array(
                message=message,
                topic=self.meta_topic,
                stride=self.meta_candidate_stride,
                xyz_indices=self.meta_xyz_indices,
                bias=self.bias_meta,
            )
        except ValueError as error:
            rospy.logerr_throttle(2.0, str(error))
            return

        self._accept_candidates(
            sensor="meta",
            candidates=candidates,
        )

    def _meta_pose_cb(self, message: PoseStamped) -> None:
        if (
            self.expected_frame
            and message.header.frame_id != self.expected_frame
        ):
            rospy.logerr_throttle(
                2.0,
                "P_meta frame mismatch: expected=%s, received=%s",
                self.expected_frame,
                message.header.frame_id,
            )
            return

        position = self._pose_position(message) - self.bias_meta
        if not self._valid_position(position):
            rospy.logerr_throttle(
                2.0, "P_meta contains NaN/Inf"
            )
            return

        self._accept_candidates(
            sensor="meta",
            candidates=position.reshape(1, 3),
        )

    def _accept_candidates(
        self,
        sensor: str,
        candidates: np.ndarray,
        covariances: Optional[List[np.ndarray]] = None,
    ) -> None:
        with self.lock:
            if not self.active:
                rospy.logwarn_throttle(
                    2.0,
                    "%s candidates received without an active prior; "
                    "ignored",
                    sensor,
                )
                return

            if candidates.ndim != 2 or candidates.shape[1] != 3:
                rospy.logerr(
                    "%s candidates must have shape (N, 3), received %s",
                    sensor,
                    candidates.shape,
                )
                return

            if sensor == "yolo":
                mode = self.yolo_input_mode
                received = self.yolo_received
                target = self.yolo_candidates
                covariance_target = self.yolo_covariances
                maximum = self.max_yolo_candidates

                if covariances is None:
                    covariances = [
                        self.cov_yolo.copy()
                        for _ in range(candidates.shape[0])
                    ]

                if len(covariances) != candidates.shape[0]:
                    rospy.logerr(
                        "YOLO candidate/covariance count mismatch: "
                        "%d candidates, %d covariances",
                        candidates.shape[0],
                        len(covariances),
                    )
                    return

            elif sensor == "meta":
                mode = self.meta_input_mode
                received = self.meta_received
                target = self.meta_candidates
                covariance_target = None
                maximum = self.max_meta_candidates
            else:
                raise ValueError(f"unknown sensor: {sensor}")

            if mode == "packed" and received:
                rospy.logwarn_throttle(
                    2.0,
                    "Second packed %s message in operation %d "
                    "was ignored",
                    sensor,
                    self.operation_id,
                )
                return

            remaining = maximum - len(target)
            if remaining <= 0:
                rospy.logwarn_throttle(
                    2.0,
                    "Maximum %s candidate count reached",
                    sensor,
                )
                return

            if candidates.shape[0] > remaining:
                rospy.logwarn(
                    "Operation %d: truncating %s candidates "
                    "from %d to %d",
                    self.operation_id,
                    sensor,
                    candidates.shape[0],
                    remaining,
                )
                candidates = candidates[:remaining]
                if covariances is not None:
                    covariances = covariances[:remaining]

            for index, candidate in enumerate(candidates):
                target.append(candidate.copy())

                if covariance_target is not None:
                    covariance_target.append(
                        self._regularize(covariances[index])
                    )

            if sensor == "yolo":
                self.yolo_received = True
            else:
                self.meta_received = True

            rospy.loginfo(
                "Operation %d: received %d %s candidates; total=%d",
                self.operation_id,
                candidates.shape[0],
                sensor,
                len(target),
            )

            self._try_finalize_early_locked()

    # ====================================================================
    # Timing
    # ====================================================================

    def _sensor_done(
        self,
        enabled: bool,
        mode: str,
        received: bool,
    ) -> bool:
        if not enabled:
            return True
        if mode == "stream":
            return False
        return received

    def _try_finalize_early_locked(self) -> None:
        yolo_done = self._sensor_done(
            self.enable_yolo,
            self.yolo_input_mode,
            self.yolo_received,
        )
        meta_done = self._sensor_done(
            self.enable_meta,
            self.meta_input_mode,
            self.meta_received,
        )

        if yolo_done and meta_done:
            self._finalize_locked(
                "all_packed_observations_received"
            )

    def _timeout_cb(self, _event) -> None:
        with self.lock:
            if self.active:
                self._finalize_locked("timeout")

    # ====================================================================
    # Gating and candidate selection
    # ====================================================================

    @staticmethod
    def _mahalanobis_squared(
        residual: np.ndarray,
        covariance: np.ndarray,
    ) -> float:
        return float(
            residual.T
            @ np.linalg.solve(covariance, residual)
        )

    def _select_candidate(
        self,
        candidates_list: List[np.ndarray],
        observation_covariance: np.ndarray,
        gate_threshold: float,
        candidate_covariances: Optional[
            List[np.ndarray]
        ] = None,
    ) -> Tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        bool,
        float,
        int,
        np.ndarray,
        np.ndarray,
    ]:
        if len(candidates_list) == 0:
            return (
                None,
                None,
                False,
                np.nan,
                -1,
                np.empty(0, dtype=float),
                np.empty(0, dtype=float),
            )

        candidates = np.vstack(candidates_list)

        if candidate_covariances is None:
            covariances = [
                observation_covariance
                for _ in range(candidates.shape[0])
            ]
        else:
            if len(candidate_covariances) != candidates.shape[0]:
                raise ValueError(
                    "candidate/covariance count mismatch"
                )
            covariances = candidate_covariances

        distances = np.empty(candidates.shape[0], dtype=float)

        for index, candidate in enumerate(candidates):
            candidate_covariance = self._regularize(
                covariances[index]
            )
            innovation_covariance = self._regularize(
                self.prior_cov + candidate_covariance
            )
            residual = candidate - self.prior
            distances[index] = self._mahalanobis_squared(
                residual,
                innovation_covariance,
            )

        scores = np.exp(-0.5 * distances)

        best_overall_index = int(np.argmin(distances))
        best_overall_distance = float(
            distances[best_overall_index]
        )

        valid_indices = np.flatnonzero(
            distances <= gate_threshold
        )
        if valid_indices.size == 0:
            return (
                None,
                None,
                False,
                best_overall_distance,
                -1,
                distances,
                scores,
            )

        selected_index = int(
            valid_indices[
                np.argmin(distances[valid_indices])
            ]
        )

        selected_covariance = self._regularize(
            covariances[selected_index]
        )

        return (
            candidates[selected_index].copy(),
            selected_covariance,
            True,
            float(distances[selected_index]),
            selected_index,
            distances,
            scores,
        )

    # ====================================================================
    # CI fusion
    # ====================================================================

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

        information = (
            weight_a * info_a
            + weight_b * info_b
        )
        covariance = self._regularize(
            np.linalg.inv(information)
        )

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
            positions,
            covariances,
            self.both_weights,
        ):
            inv_covariance = np.linalg.inv(covariance)
            information += weight * inv_covariance
            vector += weight * (
                inv_covariance @ position
            )

        covariance = self._regularize(
            np.linalg.inv(information)
        )
        return covariance @ vector, covariance

    # ====================================================================
    # Finalization
    # ====================================================================

    def _finalize_locked(self, trigger: str) -> None:
        if not self.active:
            return

        if self.timer is not None:
            try:
                self.timer.shutdown()
            except Exception:
                pass
            self.timer = None

        selected_yolo = None
        selected_yolo_cov = None
        selected_meta = None
        selected_meta_cov = None
        yolo_valid = False
        meta_valid = False
        d2_yolo = np.nan
        d2_meta = np.nan
        d2_pair = np.nan
        yolo_index = -1
        meta_index = -1
        yolo_distances = np.empty(0, dtype=float)
        yolo_scores = np.empty(0, dtype=float)
        meta_distances = np.empty(0, dtype=float)
        meta_scores = np.empty(0, dtype=float)

        try:
            (
                selected_yolo,
                selected_yolo_cov,
                yolo_valid,
                d2_yolo,
                yolo_index,
                yolo_distances,
                yolo_scores,
            ) = self._select_candidate(
                candidates_list=self.yolo_candidates,
                candidate_covariances=self.yolo_covariances,
                observation_covariance=self.cov_yolo,
                gate_threshold=self.gate_yolo,
            )

            (
                selected_meta,
                selected_meta_cov,
                meta_valid,
                d2_meta,
                meta_index,
                meta_distances,
                meta_scores,
            ) = self._select_candidate(
                candidates_list=self.meta_candidates,
                observation_covariance=self.cov_meta,
                gate_threshold=self.gate_meta,
            )

            position = self.prior.copy()
            covariance = self.prior_cov.copy()
            status = "prior_only"

            if yolo_valid:
                rospy.loginfo(
                        "Operation %d: selected YOLO covariance=\n%s",
                        self.operation_id,
                        np.array2string(
                            selected_yolo_cov,
                            precision=10,
                        ),
                    )

            if yolo_valid and meta_valid:
                pair_residual = selected_yolo - selected_meta
                pair_covariance = self._regularize(
                    selected_yolo_cov + selected_meta_cov
                )
                d2_pair = self._mahalanobis_squared(
                    pair_residual,
                    pair_covariance,
                )

                if d2_pair <= self.gate_yolo_meta:
                    position, covariance = self._ci_three(
                        positions=[
                            self.prior,
                            selected_yolo,
                            selected_meta,
                        ],
                        covariances=[
                            self.prior_cov,
                            selected_yolo_cov,
                            selected_meta_cov,
                        ],
                    )
                    status = "both_valid"
                elif d2_yolo <= d2_meta:
                    position, covariance = self._ci_two(
                        self.prior,
                        self.prior_cov,
                        selected_yolo,
                        selected_yolo_cov,
                        self.single_weight_prior,
                    )
                    status = (
                        "sensor_mismatch_yolo_selected"
                    )
                else:
                    position, covariance = self._ci_two(
                        self.prior,
                        self.prior_cov,
                        selected_meta,
                        selected_meta_cov,
                        self.single_weight_prior,
                    )
                    status = (
                        "sensor_mismatch_meta_selected"
                    )

            elif yolo_valid:
                position, covariance = self._ci_two(
                    self.prior,
                    self.prior_cov,
                    selected_yolo,
                    selected_yolo_cov,
                    self.single_weight_prior,
                )
                status = "yolo_only"

            elif meta_valid:
                position, covariance = self._ci_two(
                    self.prior,
                    self.prior_cov,
                    selected_meta,
                    selected_meta_cov,
                    self.single_weight_prior,
                )
                status = "meta_only"

            elif self.yolo_received or self.meta_received:
                status = (
                    "all_received_observations_rejected"
                )
            else:
                status = "no_observation"

            self._publish(
                position=position,
                covariance=covariance,
                status=status,
                d2_yolo=d2_yolo,
                d2_meta=d2_meta,
                d2_pair=d2_pair,
                yolo_distances=yolo_distances,
                yolo_scores=yolo_scores,
                yolo_index=yolo_index,
                meta_distances=meta_distances,
                meta_scores=meta_scores,
                meta_index=meta_index,
            )

            rospy.loginfo(
                "Operation %d finalized (%s): status=%s, "
                "YOLO candidates=%d selected=%d, "
                "Meta candidates=%d selected=%d, "
                "P_place=%s, d2_yolo=%s, "
                "d2_meta=%s, d2_pair=%s",
                self.operation_id,
                trigger,
                status,
                len(self.yolo_candidates),
                yolo_index,
                len(self.meta_candidates),
                meta_index,
                np.array2string(position, precision=6),
                self._distance_text(d2_yolo),
                self._distance_text(d2_meta),
                self._distance_text(d2_pair),
            )

        except (ValueError, np.linalg.LinAlgError) as error:
            rospy.logerr(
                "Fusion failed: %s. Prior is published unchanged.",
                error,
            )
            self._publish(
                position=self.prior,
                covariance=self.prior_cov,
                status="fusion_error_prior_used",
                d2_yolo=d2_yolo,
                d2_meta=d2_meta,
                d2_pair=d2_pair,
                yolo_distances=yolo_distances,
                yolo_scores=yolo_scores,
                yolo_index=-1,
                meta_distances=meta_distances,
                meta_scores=meta_scores,
                meta_index=-1,
            )
        finally:
            self._reset_locked()

    @staticmethod
    def _distance_text(value: float) -> str:
        if np.isnan(value):
            return "N/A"
        return f"{value:.5f}"

    # ====================================================================
    # Publishing
    # ====================================================================

    def _publish(
        self,
        position: np.ndarray,
        covariance: np.ndarray,
        status: str,
        d2_yolo: float,
        d2_meta: float,
        d2_pair: float,
        yolo_distances: np.ndarray,
        yolo_scores: np.ndarray,
        yolo_index: int,
        meta_distances: np.ndarray,
        meta_scores: np.ndarray,
        meta_index: int,
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
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.place_pub.publish(pose)
        self.place_cov_pub.publish(
            self._covariance_message(covariance)
        )
        self.status_pub.publish(String(data=status))

        distances = Float32MultiArray()
        distances.data = [
            float(d2_yolo),
            float(d2_meta),
            float(d2_pair),
        ]
        self.distance_pub.publish(distances)

        self.yolo_distances_pub.publish(
            self._vector_message(yolo_distances)
        )
        self.yolo_scores_pub.publish(
            self._vector_message(yolo_scores)
        )
        self.yolo_selected_index_pub.publish(
            Int32(data=int(yolo_index))
        )

        self.meta_distances_pub.publish(
            self._vector_message(meta_distances)
        )
        self.meta_scores_pub.publish(
            self._vector_message(meta_scores)
        )
        self.meta_selected_index_pub.publish(
            Int32(data=int(meta_index))
        )

    @staticmethod
    def _vector_message(
        values: np.ndarray,
    ) -> Float32MultiArray:
        message = Float32MultiArray()
        message.data = np.asarray(
            values, dtype=np.float32
        ).tolist()
        return message

    @staticmethod
    def _covariance_message(
        covariance: np.ndarray,
    ) -> Float32MultiArray:
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(
                label="rows", size=3, stride=9
            ),
            MultiArrayDimension(
                label="columns", size=3, stride=3
            ),
        ]
        message.layout.data_offset = 0
        message.data = (
            np.asarray(covariance, dtype=np.float32)
            .reshape(-1)
            .tolist()
        )
        return message

    def _reset_locked(self) -> None:
        self.active = False
        self.prior = None
        self.prior_cov = None
        self.prior_header = None
        self.yolo_received = False
        self.meta_received = False
        self.yolo_candidates = []
        self.yolo_covariances = []
        self.meta_candidates = []
        self.timer = None


def main() -> None:
    rospy.init_node("observation_fusion_node")

    try:
        ObservationFusionNode()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal(
            "ObservationFusionNode failed: %s", error
        )


if __name__ == "__main__":
    main()
