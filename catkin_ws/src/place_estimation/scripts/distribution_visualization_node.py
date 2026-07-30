#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""事前分布と事後分布をRViz用の共分散楕円体として可視化する。"""

import threading
from typing import Dict, Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class DistributionVisualizationNode:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.sigma_scale = float(rospy.get_param("~sigma_scale", 2.0))
        self.min_axis = float(rospy.get_param("~min_axis", 0.002))
        self.marker_lifetime = float(rospy.get_param("~marker_lifetime", 0.0))

        if self.sigma_scale <= 0.0:
            raise ValueError("~sigma_scale must be positive")
        if self.min_axis <= 0.0:
            raise ValueError("~min_axis must be positive")

        self.states: Dict[str, Dict[str, Optional[object]]] = {
            "prior": {"pose": None, "covariance": None},
            "posterior": {"pose": None, "covariance": None},
        }
        self.colors = {
            "prior": tuple(rospy.get_param(
                "~prior_color", [0.15, 0.45, 1.0, 0.35]
            )),
            "posterior": tuple(rospy.get_param(
                "~posterior_color", [1.0, 0.25, 0.15, 0.35]
            )),
        }
        for name, color in self.colors.items():
            if len(color) != 4:
                raise ValueError(f"~{name}_color must contain [r, g, b, a]")

        output_topic = rospy.get_param(
            "~marker_topic", "/place_distribution_markers"
        )
        self.publisher = rospy.Publisher(
            output_topic, MarkerArray, queue_size=10, latch=True
        )

        prior_pose_topic = rospy.get_param("~prior_pose_topic", "/P_pred")
        prior_cov_topic = rospy.get_param("~prior_cov_topic", "/Sigma_pred")
        posterior_pose_topic = rospy.get_param(
            "~posterior_pose_topic", "/P_place"
        )
        posterior_cov_topic = rospy.get_param(
            "~posterior_cov_topic", "/Sigma_place"
        )

        self.subscribers = [
            rospy.Subscriber(
                prior_pose_topic, PoseStamped,
                lambda msg: self._pose_callback("prior", msg), queue_size=10
            ),
            rospy.Subscriber(
                prior_cov_topic, Float32MultiArray,
                lambda msg: self._covariance_callback("prior", msg),
                queue_size=10,
            ),
            rospy.Subscriber(
                posterior_pose_topic, PoseStamped,
                lambda msg: self._pose_callback("posterior", msg), queue_size=10
            ),
            rospy.Subscriber(
                posterior_cov_topic, Float32MultiArray,
                lambda msg: self._covariance_callback("posterior", msg),
                queue_size=10,
            ),
        ]
        rospy.loginfo(
            "DistributionVisualizationNode started: %.1f-sigma ellipsoids on %s",
            self.sigma_scale,
            output_topic,
        )

    def _pose_callback(self, name: str, message: PoseStamped) -> None:
        position = np.array([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ])
        if not np.all(np.isfinite(position)):
            rospy.logwarn_throttle(2.0, "%s pose contains NaN or Inf", name)
            return
        with self.lock:
            self.states[name]["pose"] = message
            self._publish_locked()

    def _covariance_callback(
        self, name: str, message: Float32MultiArray
    ) -> None:
        if len(message.data) != 9:
            rospy.logwarn_throttle(
                2.0, "%s covariance requires exactly 9 values", name
            )
            return
        covariance = np.asarray(message.data, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(covariance)):
            rospy.logwarn_throttle(
                2.0, "%s covariance contains NaN or Inf", name
            )
            return
        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues = np.linalg.eigvalsh(covariance)
        if np.min(eigenvalues) < -1.0e-10:
            rospy.logwarn_throttle(
                2.0, "%s covariance is not positive semidefinite", name
            )
            return
        with self.lock:
            self.states[name]["covariance"] = covariance
            self._publish_locked()

    def _publish_locked(self) -> None:
        markers = MarkerArray()
        for marker_id, name in enumerate(("prior", "posterior")):
            pose = self.states[name]["pose"]
            covariance = self.states[name]["covariance"]
            if pose is None or covariance is None:
                continue
            markers.markers.append(
                self._ellipsoid_marker(name, marker_id * 2, pose, covariance)
            )
            markers.markers.append(
                self._label_marker(name, marker_id * 2 + 1, pose, covariance)
            )
        if markers.markers:
            self.publisher.publish(markers)

    def _ellipsoid_marker(
        self, name: str, marker_id: int, pose: PoseStamped,
        covariance: np.ndarray,
    ) -> Marker:
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, 0.0)
        if np.linalg.det(eigenvectors) < 0.0:
            eigenvectors[:, 0] *= -1.0

        marker = self._base_marker(name, marker_id, pose)
        marker.type = Marker.SPHERE
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z
        marker.pose.orientation = self._rotation_to_quaternion(eigenvectors)
        diameters = 2.0 * self.sigma_scale * np.sqrt(eigenvalues)
        marker.scale.x = max(float(diameters[0]), self.min_axis)
        marker.scale.y = max(float(diameters[1]), self.min_axis)
        marker.scale.z = max(float(diameters[2]), self.min_axis)
        color = self.colors[name]
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _label_marker(
        self, name: str, marker_id: int, pose: PoseStamped,
        covariance: np.ndarray,
    ) -> Marker:
        marker = self._base_marker(name, marker_id, pose)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z
        offset = max(
            2.0 * self.sigma_scale * np.sqrt(
                max(float(np.max(np.linalg.eigvalsh(covariance))), 0.0)
            ),
            0.03,
        )
        marker.pose.position.z += offset
        marker.pose.orientation.w = 1.0
        marker.scale.z = max(offset * 0.35, 0.025)
        color = self.colors[name]
        marker.color.r, marker.color.g, marker.color.b = color[:3]
        marker.color.a = 1.0
        marker.text = "Prior" if name == "prior" else "Posterior"
        return marker

    def _base_marker(
        self, name: str, marker_id: int, pose: PoseStamped
    ) -> Marker:
        marker = Marker()
        marker.header = pose.header
        if marker.header.stamp == rospy.Time():
            marker.header.stamp = rospy.Time.now()
        marker.ns = f"place_distribution/{name}"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(self.marker_lifetime)
        return marker

    @staticmethod
    def _rotation_to_quaternion(rotation: np.ndarray) -> Quaternion:
        # 安定な行列→クォータニオン変換。rotationの列が楕円体の主軸。
        q = np.empty(4, dtype=np.float64)  # x, y, z, w
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            q[:] = [
                (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s,
                0.25 * s,
            ]
        else:
            index = int(np.argmax(np.diag(rotation)))
            j = (index + 1) % 3
            k = (index + 2) % 3
            s = np.sqrt(
                max(1.0 + rotation[index, index]
                    - rotation[j, j] - rotation[k, k], 0.0)
            ) * 2.0
            if s < 1.0e-12:
                q[:] = [0.0, 0.0, 0.0, 1.0]
            else:
                q[index] = 0.25 * s
                q[j] = (rotation[j, index] + rotation[index, j]) / s
                q[k] = (rotation[k, index] + rotation[index, k]) / s
                q[3] = (rotation[k, j] - rotation[j, k]) / s
        q /= np.linalg.norm(q)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def main() -> None:
    rospy.init_node("distribution_visualization_node")
    DistributionVisualizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
