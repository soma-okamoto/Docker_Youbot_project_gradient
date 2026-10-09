#!/usr/bin/env python3
"""End-to-end ROS topic smoke test for place_estimation_pipeline.launch."""
import json
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
from visualization_msgs.msg import MarkerArray


class Collector:
    def __init__(self):
        self.condition = threading.Condition()
        self.predictions = []
        self.prior_covariances = []
        self.places = []
        self.place_covariances = []
        self.statuses = []
        self.used_observations = []
        self.markers = []
        self.kl_metrics = []
        self.kl_summaries = []

    def append(self, target, message):
        with self.condition:
            target.append(message)
            self.condition.notify_all()

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise AssertionError(f"timeout waiting for {description}")
                self.condition.wait(remaining)


def wait_for_connections(publishers, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if all(publisher.get_num_connections() > 0 for publisher in publishers):
            return
        rospy.sleep(0.05)
    raise AssertionError("input publishers did not connect to pipeline nodes")


def pose(position):
    message = PoseStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "base_footprint"
    message.pose.position.x = position[0]
    message.pose.position.y = position[1]
    message.pose.position.z = position[2]
    message.pose.orientation.w = 1.0
    return message


def yolo_record(position, standard_deviation=0.02):
    covariance = np.eye(3) * standard_deviation ** 2
    return Float32MultiArray(
        data=list(position) + covariance.reshape(-1).tolist()
    )


def publish_observation_window(yolo_pub, meta_pub):
    yolo_pub.publish(yolo_record([1.0, 0.0, 0.0]))
    for x in (0.99, 1.0, 1.01):
        meta_pub.publish(Float32MultiArray(data=[x, 0.0, 0.0]))
        rospy.sleep(0.08)


def main():
    rospy.init_node("place_estimation_pipeline_smoke_test", anonymous=True)
    collector = Collector()

    rospy.Subscriber(
        "/P_pred", PoseStamped,
        lambda msg: collector.append(collector.predictions, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/Sigma_pred", Float32MultiArray,
        lambda msg: collector.append(collector.prior_covariances, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/P_place", PoseStamped,
        lambda msg: collector.append(collector.places, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/Sigma_place", Float32MultiArray,
        lambda msg: collector.append(collector.place_covariances, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/observation_status", String,
        lambda msg: collector.append(collector.statuses, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/used_physical_observation", PoseWithCovarianceStamped,
        lambda msg: collector.append(collector.used_observations, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/place_distribution_markers", MarkerArray,
        lambda msg: collector.append(collector.markers, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/kl_evaluation", Float64MultiArray,
        lambda msg: collector.append(collector.kl_metrics, msg),
        queue_size=10,
    )
    rospy.Subscriber(
        "/kl_adaptation_summary", String,
        lambda msg: collector.append(collector.kl_summaries, msg),
        queue_size=10,
    )

    current_pub = rospy.Publisher(
        "/P_current", Float32MultiArray, queue_size=1
    )
    tf_pub = rospy.Publisher("/P_tf", PoseStamped, queue_size=1)
    yolo_pub = rospy.Publisher("/P_yolo", Float32MultiArray, queue_size=1)
    meta_pub = rospy.Publisher("/P_meta", Float32MultiArray, queue_size=10)
    wait_for_connections([current_pub, tf_pub, yolo_pub, meta_pub])

    # Trial 1: raw operation sources straddle the physical observation.
    current_pub.publish(Float32MultiArray(data=[1.0, 1.1, 0.0, 0.0]))
    tf_pub.publish(pose([0.9, 0.0, 0.0]))
    collector.wait_for(
        lambda: len(collector.predictions) >= 1, 3.0, "first prior"
    )
    collector.wait_for(
        lambda: len(collector.prior_covariances) >= 1,
        3.0,
        "first prior covariance",
    )
    rospy.sleep(0.1)
    first_prior_x = collector.predictions[0].pose.position.x
    publish_observation_window(yolo_pub, meta_pub)

    collector.wait_for(
        lambda: len(collector.statuses) >= 1, 6.0, "first fusion result"
    )
    collector.wait_for(
        lambda: len(collector.places) >= 1, 1.0, "P_place"
    )
    collector.wait_for(
        lambda: len(collector.place_covariances) >= 1,
        1.0,
        "Sigma_place",
    )
    collector.wait_for(
        lambda: len(collector.used_observations) >= 1,
        1.0,
        "used physical observation",
    )
    collector.wait_for(
        lambda: len(collector.markers) >= 1, 1.0, "RViz markers"
    )
    collector.wait_for(
        lambda: len(collector.kl_metrics) >= 1, 1.0, "KL evaluation"
    )

    # Trace-minimizing prior CI selects the lower-uncertainty TF estimate
    # (x=0.9), so the x=1.0 observations are outside the prior gate and the
    # first result correctly uses physical observations only.
    assert collector.statuses[0].data == "yolo_meta_fused"
    assert abs(collector.places[0].pose.position.x - 1.0) < 0.02
    covariance = np.asarray(
        collector.place_covariances[0].data, dtype=float
    ).reshape(3, 3)
    assert np.all(np.isfinite(covariance))
    assert np.all(np.linalg.eigvalsh(covariance) > 0.0)
    assert collector.used_observations[0].header.seq == 1
    assert collector.used_observations[0].header.frame_id == "base_footprint"
    first_kl = np.asarray(collector.kl_metrics[0].data, dtype=float)
    assert first_kl.shape == (10,)
    assert first_kl[0] == 1.0
    assert np.all(np.isfinite(first_kl))
    assert np.all(first_kl[1:4] >= 0.0)

    # Allow the feedback callback to update operation-source biases.
    rospy.sleep(0.3)

    # Trial 2: the same raw inputs should now be bias-corrected to x=1.0.
    current_pub.publish(Float32MultiArray(data=[2.0, 1.1, 0.0, 0.0]))
    tf_pub.publish(pose([0.9, 0.0, 0.0]))
    collector.wait_for(
        lambda: len(collector.predictions) >= 2, 3.0, "learned second prior"
    )
    collector.wait_for(
        lambda: len(collector.prior_covariances) >= 2,
        3.0,
        "second prior covariance",
    )
    rospy.sleep(0.1)
    second_prior_x = collector.predictions[1].pose.position.x
    assert abs(second_prior_x - 1.0) < 1.0e-6
    assert abs(first_prior_x - second_prior_x) > 0.05

    publish_observation_window(yolo_pub, meta_pub)
    collector.wait_for(
        lambda: len(collector.statuses) >= 2, 6.0, "second fusion result"
    )
    collector.wait_for(
        lambda: len(collector.kl_metrics) >= 2,
        1.0,
        "second KL evaluation",
    )
    assert collector.statuses[1].data == "prior_yolo_meta_fused"

    # Trial 3: a stable physical observation far from the prior remains valid;
    # because no Meta observation arrives, the output must be YOLO-only.
    rospy.sleep(0.3)
    current_pub.publish(Float32MultiArray(data=[3.0, 1.1, 0.0, 0.0]))
    tf_pub.publish(pose([0.9, 0.0, 0.0]))
    collector.wait_for(
        lambda: len(collector.predictions) >= 3, 3.0, "third prior"
    )
    collector.wait_for(
        lambda: len(collector.prior_covariances) >= 3,
        3.0,
        "third prior covariance",
    )
    rospy.sleep(0.1)
    yolo_pub.publish(yolo_record([2.0, 0.0, 0.0]))
    collector.wait_for(
        lambda: len(collector.statuses) >= 3, 6.0, "far-observation result"
    )
    collector.wait_for(
        lambda: len(collector.places) >= 3, 1.0, "far P_place"
    )
    collector.wait_for(
        lambda: len(collector.kl_metrics) >= 3,
        1.0,
        "far-observation KL evaluation",
    )
    collector.wait_for(
        lambda: len(collector.kl_summaries) >= 3,
        1.0,
        "KL adaptation summary",
    )
    assert collector.statuses[2].data == "yolo_only"
    assert abs(collector.places[2].pose.position.x - 2.0) < 1.0e-6
    final_summary = json.loads(collector.kl_summaries[-1].data)
    assert final_summary["sample_count"] == 3
    assert final_summary["latest_operation_id"] == 3
    assert final_summary["adaptation_state"] == "insufficient_data"
    assert final_summary["kl_trend"]["latest"] >= 0.0

    print(
        "ROS pipeline smoke test passed: "
        f"first_prior_x={first_prior_x:.6f}, "
        f"learned_prior_x={second_prior_x:.6f}, "
        f"P_place_x={collector.places[0].pose.position.x:.6f}, "
        f"first_kl={first_kl[1]:.6f}, "
        f"trend_state={final_summary['adaptation_state']}, "
        f"near_status={collector.statuses[0].data}, "
        f"far_status={collector.statuses[2].data}"
    )


if __name__ == "__main__":
    main()
