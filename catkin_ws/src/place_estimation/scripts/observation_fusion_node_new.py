#!/usr/bin/env python3
# -*- coding: utf-8 -*-



"""
observation_fusion_node_new.py

事前分布 N(P_pred, Sigma_pred) とPlace後のYOLO/RealSense・Meta観測を
受け取り、局所適応レジストレーションの推定結果
N(P_place, Sigma_place) を生成するROS1ノード。

前提:
- 入力位置は上流でボトル中心、メートル、共通座標系へ変換済みとする。
- このノードはセンサ観測のバイアスを固定ROSパラメータで補正する。
- 試行間の誤差学習状態は保持しない。実際に採用した物理観測分布を
  /used_physical_observationへ配信し、prior_distribution_node.pyが
  P_current・P_tfのバイアスと共分散を逐次更新する。

入力:
- /P_pred, /Sigma_pred: Place操作ごとの事前位置分布。
- /P_yolo: YOLO/RealSenseの可変個数候補。各候補に3x3共分散が必須。
- /P_meta: 無効化、Float32MultiArray、PoseStampedから選択可能。
  Float32MultiArrayでは位置のみ、または候補ごとの3x3共分散を受信できる。

候補入力モード:
- packed: 1つのメッセージに形成済みの可変個数候補を格納する。
  共分散付きレコード例:
  [x,y,z,Sxx,Sxy,Sxz,Syx,Syy,Syz,Szx,Szy,Szz, ...]
- stream: observation_timeout内の各メッセージを1観測フレームとして蓄積する。
  距離に基づいてフレーム間対応を取り、検出継続率と位置ばらつきから
  安定候補を形成する。代表位置はフレーム平均、代表共分散は標本共分散/N
  とフレームごとのセンサ共分散/Nの和から計算する。

候補選択と融合:
- 非有限、非正定値、または最大主軸標準偏差が上限を超える候補を棄却する。
- 事前分布とのマハラノビス距離は候補診断と、事前をCIへ含めるかの判定に
  使用する。事前から遠いという理由だけでは物理観測を棄却しない。
- YOLOとMetaが整合する場合は、最も整合する候補組を選択してCI融合する。
- 両観測が競合する場合は共分散traceが小さい方を採用し、同程度なら
  事前分布へフォールバックする。
- 事前と採用観測が近ければ事前を含むCI、遠ければ物理観測のみのCIを行う。
- CI重みは固定値、または融合後共分散のtrace/determinant最小化で決定する。
- 有効な物理観測がない場合は事前分布をそのまま採用する。

主な出力:
- /P_place, /Sigma_place: 推定したボトル中心位置と3x3共分散。
- /used_physical_observation: 事前を含める前の採用物理観測分布。
- /observation_status, /observation_distances: 分岐状態と選択距離。
- センサ別の候補距離、整合度、選択インデックス診断トピック。

観測待機時間、マハラノビス距離、共分散上限、検出継続率などの閾値は
config/local_registration_thresholds.yamlから設定する。
"""

import threading
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
        self.used_observation_topic = rospy.get_param(
            "~used_observation_topic", "/used_physical_observation"
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
            "~yolo_candidate_stride", 12
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

        # Empty indices preserve the existing XYZ-only Meta input.
        self.meta_covariance_indices = None
        if rospy.get_param("~meta_covariance_indices", []):
            self.meta_covariance_indices = self._load_n_indices(
                "~meta_covariance_indices", list(range(3, 12)), 9
            )
            self._validate_indices(
                "~meta_covariance_indices",
                self.meta_covariance_indices,
                self.meta_candidate_stride,
            )
            if self.meta_message_type != "float32_multi_array":
                raise ValueError(
                    "~meta_covariance_indices requires float32_multi_array"
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
        if not np.isfinite(self.min_variance) or self.min_variance <= 0.0:
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
        # Prior gates decide whether the prior participates in CI. A distant
        # physical observation is still valid and is fused without the prior.
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

        if not all(
            np.isfinite(value) and value > 0.0
            for value in (
                self.gate_yolo,
                self.gate_meta,
                self.gate_yolo_meta,
            )
        ):
            raise ValueError("gate thresholds must be finite and positive")

        self.uncertainty_trace_rtol = float(
            rospy.get_param(
                "~uncertainty_trace_relative_tolerance", 0.10
            )
        )
        self.uncertainty_trace_atol = float(
            rospy.get_param(
                "~uncertainty_trace_absolute_tolerance", 1.0e-6
            )
        )
        if (
            not np.isfinite(self.uncertainty_trace_rtol)
            or self.uncertainty_trace_rtol < 0.0
            or not np.isfinite(self.uncertainty_trace_atol)
            or self.uncertainty_trace_atol < 0.0
        ):
            raise ValueError(
                "uncertainty trace tolerances must be finite and non-negative"
            )

        # ================================================================
        # Waiting / CI weights
        # ================================================================
        self.observation_timeout = float(
            rospy.get_param("~observation_timeout", 2.0)
        )
        if (
            not np.isfinite(self.observation_timeout)
            or self.observation_timeout <= 0.0
        ):
            raise ValueError(
                "~observation_timeout must be finite and positive"
            )

        self.ci_weight_mode = str(
            rospy.get_param("~observation_ci_weight_mode", "optimize")
        ).lower()
        self.ci_weight_step = float(
            rospy.get_param("~observation_ci_weight_step", 0.01)
        )
        self.ci_objective = str(
            rospy.get_param("~observation_ci_objective", "trace")
        ).lower()
        if self.ci_weight_mode not in {"optimize", "fixed"}:
            raise ValueError(
                "~observation_ci_weight_mode must be optimize or fixed"
            )
        if self.ci_objective not in {"trace", "determinant"}:
            raise ValueError(
                "~observation_ci_objective must be trace or determinant"
            )
        if not np.isfinite(self.ci_weight_step) or self.ci_weight_step <= 0.0:
            raise ValueError(
                "~observation_ci_weight_step must be finite and positive"
            )

        self.single_weight_prior = float(
            rospy.get_param("~single_weight_prior", 0.5)
        )
        if (
            not np.isfinite(self.single_weight_prior)
            or not 0.0 <= self.single_weight_prior <= 1.0
        ):
            raise ValueError("~single_weight_prior must be in [0, 1]")

        self.both_weights = self._load_weights(
            "~both_weights", [0.34, 0.33, 0.33]
        )
        observation_weight_sum = float(self.both_weights[1:].sum())
        if observation_weight_sum <= 0.0:
            raise ValueError(
                "~both_weights must give positive weight to YOLO or Meta"
            )
        self.max_std_yolo = self._load_positive_float("~max_std_yolo", 0.10)
        self.max_std_meta = self._load_positive_float("~max_std_meta", 0.10)
        self.min_observation_frames = self._load_positive_int(
            "~min_observation_frames", 3
        )
        if self.min_observation_frames < 2:
            raise ValueError("~min_observation_frames must be at least 2")
        self.min_detection_ratio = float(
            rospy.get_param("~min_detection_ratio", 0.60)
        )
        if (
            not np.isfinite(self.min_detection_ratio)
            or not 0.0 < self.min_detection_ratio <= 1.0
        ):
            raise ValueError("~min_detection_ratio must be in (0, 1]")
        self.max_frame_position_std_yolo = self._load_positive_float(
            "~max_frame_position_std_yolo", 0.10
        )
        self.max_frame_position_std_meta = self._load_positive_float(
            "~max_frame_position_std_meta", 0.10
        )
        self.candidate_association_distance = self._load_positive_float(
            "~candidate_association_distance", 0.10
        )
        if rospy.has_param("~mismatch_preferred_sensor"):
            rospy.logwarn(
                "~mismatch_preferred_sensor is ignored; conflicting observations "
                "are selected by covariance trace"
            )
        if self.enable_meta and self.meta_covariance_indices is None:
            if self.meta_input_mode == "stream":
                rospy.loginfo(
                    "Meta uses fixed per-frame covariance plus temporal "
                    "sample covariance from stream observations."
                )
            else:
                rospy.logwarn(
                    "Packed Meta XYZ uses fixed covariance; send covariance "
                    "with ~meta_covariance_indices to reflect uncertainty."
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
        self.meta_covariances: List[np.ndarray] = []
        self.yolo_frames = []
        self.meta_frames = []
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
        self.used_observation_pub = rospy.Publisher(
            self.used_observation_topic,
            PoseWithCovarianceStamped,
            queue_size=10,
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
    def _load_positive_float(name: str, default: float) -> float:
        value = float(rospy.get_param(name, default))
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
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
        return self._parse_covariance_candidate_array(
            message, self.yolo_topic, self.yolo_candidate_stride,
            self.yolo_xyz_indices, self.yolo_covariance_indices, self.bias_yolo,
        )

    def _parse_covariance_candidate_array(
        self,
        message: Float32MultiArray,
        topic: str,
        stride: int,
        xyz_indices: Sequence[int],
        covariance_indices: Sequence[int],
        bias: np.ndarray,
    ) -> Tuple[np.ndarray, List[np.ndarray]]:
        values = np.asarray(message.data, dtype=float)
        if values.size % stride != 0:
            raise ValueError(
                f"{topic}: data length {values.size} "
                f"is not divisible by candidate stride {stride}"
            )
        records = values.reshape(-1, stride)
        positions = records[:, list(xyz_indices)] - bias.reshape(1, 3)
        finite_mask = np.all(np.isfinite(positions), axis=1)
        if not np.all(finite_mask):
            rospy.logwarn(
                "%s: discarded %d candidates containing invalid XYZ values",
                topic, int(np.count_nonzero(~finite_mask)),
            )
        # Keep covariance and position indices aligned. Selection rejects
        # invalid/high covariance and reports distance=inf, score=0.
        covariances = [
            record[list(covariance_indices)].reshape(3, 3).copy()
            for record in records[finite_mask]
        ]
        return positions[finite_mask], covariances

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
        self.meta_covariances = []
        self.yolo_frames = []
        self.meta_frames = []

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
            covariances = None
            if self.meta_covariance_indices is not None:
                candidates, covariances = self._parse_covariance_candidate_array(
                    message, self.meta_topic, self.meta_candidate_stride,
                    self.meta_xyz_indices, self.meta_covariance_indices,
                    self.bias_meta,
                )
            else:
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
            covariances=covariances,
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
                frame_target = self.yolo_frames
                maximum = self.max_yolo_candidates

                default_covariance = self.cov_yolo

            elif sensor == "meta":
                mode = self.meta_input_mode
                received = self.meta_received
                target = self.meta_candidates
                covariance_target = self.meta_covariances
                frame_target = self.meta_frames
                maximum = self.max_meta_candidates
                default_covariance = self.cov_meta
            else:
                raise ValueError(f"unknown sensor: {sensor}")

            if covariances is None:
                covariances = [
                    default_covariance.copy() for _ in range(candidates.shape[0])
                ]
            if len(covariances) != candidates.shape[0]:
                rospy.logerr("%s candidate/covariance count mismatch", sensor)
                return

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

                covariance_target.append(covariances[index].copy())

            if mode == "stream":
                frame_target.append(
                    (
                        [candidate.copy() for candidate in candidates],
                        [covariance.copy() for covariance in covariances],
                    )
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
    # Candidate selection and prior discrepancy diagnostics
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

    def _stable_covariance(
        self, covariance: np.ndarray, max_std: float,
    ) -> Optional[np.ndarray]:
        """Reject unknown/non-PSD uncertainty and excessive principal variance."""
        covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(covariance)):
            return None
        covariance = 0.5 * (covariance + covariance.T)
        if np.any(np.diag(covariance) <= 0.0):
            return None
        values = np.linalg.eigvalsh(covariance)
        if values[0] < -self.min_variance:
            return None
        if np.sqrt(max(values[-1], self.min_variance)) > max_std:
            return None
        return self._regularize(covariance)

    def _summarize_stream_candidates(self, sensor: str) -> None:
        """Associate stream detections and form stable representative observations."""
        if sensor == "yolo":
            if self.yolo_input_mode != "stream":
                return
            frames = self.yolo_frames
            max_frame_std = self.max_frame_position_std_yolo
            max_observation_std = self.max_std_yolo
        elif sensor == "meta":
            if self.meta_input_mode != "stream":
                return
            frames = self.meta_frames
            max_frame_std = self.max_frame_position_std_meta
            max_observation_std = self.max_std_meta
        else:
            raise ValueError(f"unknown sensor: {sensor}")

        tracks = []
        for frame_index, (positions, covariances) in enumerate(frames):
            detections = []
            for position, covariance in zip(positions, covariances):
                valid_covariance = self._stable_covariance(
                    covariance, np.inf
                )
                if valid_covariance is not None:
                    detections.append((position, valid_covariance))

            possible_matches = []
            for track_index, track in enumerate(tracks):
                for detection_index, (position, _covariance) in enumerate(
                    detections
                ):
                    distance = float(
                        np.linalg.norm(position - track["last_position"])
                    )
                    if distance <= self.candidate_association_distance:
                        possible_matches.append(
                            (distance, track_index, detection_index)
                        )

            assigned_tracks = set()
            assigned_detections = set()
            for _distance, track_index, detection_index in sorted(
                possible_matches
            ):
                if (
                    track_index in assigned_tracks
                    or detection_index in assigned_detections
                ):
                    continue
                position, covariance = detections[detection_index]
                track = tracks[track_index]
                track["positions"].append(position.copy())
                track["covariances"].append(covariance.copy())
                track["frame_indices"].append(frame_index)
                track["last_position"] = position.copy()
                assigned_tracks.add(track_index)
                assigned_detections.add(detection_index)

            for detection_index, (position, covariance) in enumerate(
                detections
            ):
                if detection_index in assigned_detections:
                    continue
                tracks.append(
                    {
                        "positions": [position.copy()],
                        "covariances": [covariance.copy()],
                        "frame_indices": [frame_index],
                        "last_position": position.copy(),
                    }
                )

        representative_positions = []
        representative_covariances = []
        frame_count = len(frames)
        for track in tracks:
            count = len(track["positions"])
            detection_ratio = count / frame_count if frame_count else 0.0
            if (
                count < self.min_observation_frames
                or detection_ratio < self.min_detection_ratio
            ):
                continue

            positions = np.vstack(track["positions"])
            mean_position = np.mean(positions, axis=0)
            centered = positions - mean_position
            frame_covariance = (centered.T @ centered) / float(count - 1)
            frame_covariance = 0.5 * (
                frame_covariance + frame_covariance.T
            )
            frame_values = np.linalg.eigvalsh(frame_covariance)
            frame_std = float(
                np.sqrt(max(frame_values[-1], 0.0))
            )
            if frame_std > max_frame_std:
                continue

            # C_frame/N follows the manuscript. The average per-frame sensor
            # covariance contributes another mean-estimate uncertainty term.
            sensor_covariance = np.mean(
                np.stack(track["covariances"], axis=0), axis=0
            )
            representative_covariance = self._regularize(
                frame_covariance / float(count)
                + sensor_covariance / float(count)
            )
            if self._stable_covariance(
                representative_covariance, max_observation_std
            ) is None:
                continue

            representative_positions.append(mean_position)
            representative_covariances.append(representative_covariance)

        if sensor == "yolo":
            self.yolo_candidates = representative_positions
            self.yolo_covariances = representative_covariances
        else:
            self.meta_candidates = representative_positions
            self.meta_covariances = representative_covariances

        rospy.loginfo(
            "Operation %d: %s stream frames=%d tracks=%d stable=%d",
            self.operation_id,
            sensor,
            frame_count,
            len(tracks),
            len(representative_positions),
        )

    def _select_candidate(
        self,
        candidates_list: List[np.ndarray],
        observation_covariance: np.ndarray,
        gate_threshold: float,
        max_std: float,
        candidate_covariances: Optional[
            List[np.ndarray]
        ] = None,
        sensor: str = "observation",
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

        distances = np.full(candidates.shape[0], np.inf, dtype=float)

        for index, candidate in enumerate(candidates):
            candidate_covariance = self._stable_covariance(
                covariances[index], max_std
            )
            if candidate_covariance is None:
                rospy.logwarn(
                    "Operation %d: rejected %s candidate %d: invalid covariance "
                    "or principal std exceeds %.4f m",
                    self.operation_id, sensor, index, max_std,
                )
                continue
            innovation_covariance = self._regularize(
                self.prior_cov + candidate_covariance
            )
            residual = candidate - self.prior
            distances[index] = self._mahalanobis_squared(
                residual,
                innovation_covariance,
            )

        scores = np.exp(-0.5 * distances)
        if not np.any(np.isfinite(distances)):
            return None, None, False, np.nan, -1, distances, scores

        best_overall_index = int(np.argmin(distances))
        best_overall_distance = float(
            distances[best_overall_index]
        )
        if best_overall_distance > gate_threshold:
            rospy.logwarn(
                "Operation %d: all candidates differ from the MR prior "
                "(minimum d2=%.3f, threshold=%.3f); "
                "the observation remains eligible",
                self.operation_id,
                best_overall_distance,
                gate_threshold,
            )

        # A large distance can mean that MR prediction violated a physical
        # constraint. It must not invalidate a finite sensor measurement.
        selected_index = best_overall_index

        selected_covariance = self._regularize(
            covariances[selected_index]
        )

        return (
            candidates[selected_index].copy(),
            selected_covariance,
            True,
            best_overall_distance,
            selected_index,
            distances,
            scores,
        )

    def _best_consistent_pair(
        self,
    ) -> Optional[Tuple[int, int, float]]:
        """Find the most mutually consistent YOLO/Meta candidate pair."""
        best = None
        meta_covariances = [
            self._stable_covariance(covariance, self.max_std_meta)
            for covariance in self.meta_covariances
        ]
        for yi, yolo in enumerate(self.yolo_candidates):
            yolo_cov = self._stable_covariance(
                self.yolo_covariances[yi], self.max_std_yolo
            )
            if yolo_cov is None:
                continue
            for mi, meta in enumerate(self.meta_candidates):
                meta_cov = meta_covariances[mi]
                if meta_cov is None:
                    continue
                covariance = self._regularize(yolo_cov + meta_cov)
                d2 = self._mahalanobis_squared(yolo - meta, covariance)
                if np.isfinite(d2) and (best is None or d2 < best[2]):
                    best = (yi, mi, d2)
        if best is not None and best[2] <= self.gate_yolo_meta:
            return best
        return None

    # ====================================================================
    # CI fusion
    # ====================================================================

    @staticmethod
    def _integer_compositions(total: int, count: int):
        if count == 1:
            yield (total,)
            return
        for value in range(total + 1):
            for suffix in ObservationFusionNode._integer_compositions(
                total - value, count - 1
            ):
                yield (value,) + suffix

    def _simplex_weight_candidates(
        self,
        count: int,
        preferred: np.ndarray,
    ):
        """Yield the preferred weights first, then a simplex search grid."""
        yield preferred
        units = max(1, int(np.ceil(1.0 / self.ci_weight_step)))
        for composition in self._integer_compositions(units, count):
            weights = np.asarray(composition, dtype=float) / float(units)
            if not np.allclose(weights, preferred, rtol=0.0, atol=1.0e-12):
                yield weights

    def _ci_fuse(
        self,
        positions: Sequence[np.ndarray],
        covariances: Sequence[np.ndarray],
        preferred_weights: Optional[Sequence[float]] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Fuse one to three correlated estimates with generalized CI."""
        if len(positions) == 0 or len(positions) != len(covariances):
            raise ValueError("CI requires matching non-empty input lists")

        count = len(positions)
        if count > 3:
            raise ValueError("CI currently supports at most three inputs")

        normalized_positions = [
            np.asarray(position, dtype=float).reshape(3)
            for position in positions
        ]
        normalized_covariances = [
            self._regularize(covariance) for covariance in covariances
        ]

        if count == 1:
            return (
                normalized_positions[0].copy(),
                normalized_covariances[0].copy(),
                np.ones(1, dtype=float),
            )

        if preferred_weights is None:
            preferred = np.full(count, 1.0 / count, dtype=float)
        else:
            preferred = np.asarray(preferred_weights, dtype=float)
            if (
                preferred.shape != (count,)
                or not np.all(np.isfinite(preferred))
                or np.any(preferred < 0.0)
                or float(preferred.sum()) <= 0.0
            ):
                raise ValueError("invalid preferred CI weights")
            preferred = preferred / float(preferred.sum())

        if self.ci_weight_mode == "fixed":
            candidates = (preferred,)
        else:
            candidates = self._simplex_weight_candidates(count, preferred)

        information_matrices = [
            np.linalg.inv(covariance)
            for covariance in normalized_covariances
        ]
        best = None
        best_objective = np.inf

        for weights in candidates:
            information = sum(
                weight * matrix
                for weight, matrix in zip(weights, information_matrices)
            )
            covariance = self._regularize(np.linalg.inv(information))
            if self.ci_objective == "determinant":
                sign, objective = np.linalg.slogdet(covariance)
                if sign <= 0.0:
                    continue
                objective = float(objective)
            else:
                objective = float(np.trace(covariance))

            tolerance = 1.0e-12 * max(1.0, abs(best_objective))
            if best is not None and objective >= best_objective - tolerance:
                continue

            information_vector = sum(
                weight * (matrix @ position)
                for weight, matrix, position in zip(
                    weights, information_matrices, normalized_positions
                )
            )
            position = covariance @ information_vector
            best = (position, covariance, np.asarray(weights, dtype=float))
            best_objective = objective

        if best is None:
            raise np.linalg.LinAlgError("no valid CI solution was found")
        return best

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
            self._summarize_stream_candidates("yolo")
            self._summarize_stream_candidates("meta")

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
                max_std=self.max_std_yolo,
                sensor="YOLO",
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
                candidate_covariances=self.meta_covariances,
                observation_covariance=self.cov_meta,
                gate_threshold=self.gate_meta,
                max_std=self.max_std_meta,
                sensor="Meta",
            )

            position = self.prior.copy()
            covariance = self.prior_cov.copy()
            status = "prior_only"

            pair = (
                self._best_consistent_pair()
                if yolo_valid and meta_valid
                else None
            )
            if pair is not None:
                yolo_index, meta_index, d2_pair = pair
                selected_yolo = self.yolo_candidates[yolo_index].copy()
                selected_yolo_cov = self._regularize(
                    self.yolo_covariances[yolo_index]
                )
                selected_meta = self.meta_candidates[meta_index].copy()
                selected_meta_cov = self._regularize(
                    self.meta_covariances[meta_index]
                )
                d2_yolo = float(yolo_distances[yolo_index])
                d2_meta = float(meta_distances[meta_index])

            if yolo_valid:
                rospy.loginfo(
                        "Operation %d: selected YOLO covariance=\n%s",
                        self.operation_id,
                        np.array2string(
                            selected_yolo_cov,
                            precision=10,
                        ),
                    )

            selected_positions = []
            selected_covariances = []
            selected_sensors = []
            selection_kind = None
            used_observation_position = None
            used_observation_covariance = None

            if yolo_valid and meta_valid:
                pair_residual = selected_yolo - selected_meta
                pair_covariance = self._regularize(
                    selected_yolo_cov + selected_meta_cov
                )
                d2_pair = self._mahalanobis_squared(
                    pair_residual,
                    pair_covariance,
                )

                if pair is not None:
                    selected_positions = [selected_yolo, selected_meta]
                    selected_covariances = [
                        selected_yolo_cov, selected_meta_cov
                    ]
                    selected_sensors = ["yolo", "meta"]
                    selection_kind = "consistent_both"
                elif np.isclose(
                    np.trace(selected_yolo_cov), np.trace(selected_meta_cov),
                    rtol=self.uncertainty_trace_rtol,
                    atol=self.uncertainty_trace_atol,
                ):
                    # Similar uncertainty gives no evidence to prefer a sensor.
                    status = "sensor_mismatch_equal_uncertainty_prior_used"
                elif np.trace(selected_yolo_cov) < np.trace(selected_meta_cov):
                    selected_positions = [selected_yolo]
                    selected_covariances = [selected_yolo_cov]
                    selected_sensors = ["yolo"]
                    selection_kind = "mismatch_yolo"
                else:
                    selected_positions = [selected_meta]
                    selected_covariances = [selected_meta_cov]
                    selected_sensors = ["meta"]
                    selection_kind = "mismatch_meta"

            elif yolo_valid:
                selected_positions = [selected_yolo]
                selected_covariances = [selected_yolo_cov]
                selected_sensors = ["yolo"]
                selection_kind = "yolo"

            elif meta_valid:
                selected_positions = [selected_meta]
                selected_covariances = [selected_meta_cov]
                selected_sensors = ["meta"]
                selection_kind = "meta"

            elif self.yolo_received or self.meta_received:
                status = "no_valid_observation"
            else:
                status = "no_observation"

            if selected_positions:
                distance_by_sensor = {
                    "yolo": d2_yolo,
                    "meta": d2_meta,
                }
                gate_by_sensor = {
                    "yolo": self.gate_yolo,
                    "meta": self.gate_meta,
                }
                include_prior = all(
                    np.isfinite(distance_by_sensor[sensor])
                    and distance_by_sensor[sensor] <= gate_by_sensor[sensor]
                    for sensor in selected_sensors
                )

                if selected_sensors == ["yolo", "meta"]:
                    observation_weights = self.both_weights[1:]
                else:
                    observation_weights = np.ones(1, dtype=float)

                (
                    used_observation_position,
                    used_observation_covariance,
                    _used_observation_weights,
                ) = self._ci_fuse(
                    selected_positions,
                    selected_covariances,
                    observation_weights,
                )

                if include_prior:
                    if len(selected_positions) == 1:
                        preferred_weights = np.array(
                            [
                                self.single_weight_prior,
                                1.0 - self.single_weight_prior,
                            ],
                            dtype=float,
                        )
                    else:
                        preferred_weights = self.both_weights

                    position, covariance, ci_weights = self._ci_fuse(
                        [self.prior] + selected_positions,
                        [self.prior_cov] + selected_covariances,
                        preferred_weights,
                    )
                    status_by_kind = {
                        "consistent_both": "prior_yolo_meta_fused",
                        "yolo": "prior_yolo_fused",
                        "meta": "prior_meta_fused",
                        "mismatch_yolo": "sensor_mismatch_prior_yolo_fused",
                        "mismatch_meta": "sensor_mismatch_prior_meta_fused",
                    }
                else:
                    position, covariance, ci_weights = self._ci_fuse(
                        selected_positions,
                        selected_covariances,
                        observation_weights,
                    )
                    status_by_kind = {
                        "consistent_both": "yolo_meta_fused",
                        "yolo": "yolo_only",
                        "meta": "meta_only",
                        "mismatch_yolo": "sensor_mismatch_yolo_selected",
                        "mismatch_meta": "sensor_mismatch_meta_selected",
                    }

                status = status_by_kind[selection_kind]
                rospy.loginfo(
                    "Operation %d: CI sources=%s weights=%s prior_included=%s",
                    self.operation_id,
                    (["prior"] if include_prior else []) + selected_sensors,
                    np.array2string(ci_weights, precision=4),
                    include_prior,
                )

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
                used_observation_position=used_observation_position,
                used_observation_covariance=used_observation_covariance,
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
                used_observation_position=None,
                used_observation_covariance=None,
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
        used_observation_position: Optional[np.ndarray],
        used_observation_covariance: Optional[np.ndarray],
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

        if (
            used_observation_position is not None
            and used_observation_covariance is not None
        ):
            used = PoseWithCovarianceStamped()
            used.header = pose.header
            used.pose.pose.position.x = float(used_observation_position[0])
            used.pose.pose.position.y = float(used_observation_position[1])
            used.pose.pose.position.z = float(used_observation_position[2])
            used.pose.pose.orientation.w = 1.0
            covariance_6d = np.zeros((6, 6), dtype=float)
            covariance_6d[:3, :3] = used_observation_covariance
            used.pose.covariance = covariance_6d.reshape(-1).tolist()
            self.used_observation_pub.publish(used)

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
        self.meta_covariances = []
        self.yolo_frames = []
        self.meta_frames = []
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
