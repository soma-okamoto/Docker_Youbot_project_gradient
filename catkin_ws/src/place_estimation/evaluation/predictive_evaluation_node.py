#!/usr/bin/env python3
"""Evaluate atomic prediction/observation snapshots without affecting learning."""
from collections import deque
import json
from pathlib import Path
import sys
import threading

import numpy as np
import rospy
from std_msgs.msg import String

try:
    from predictive_metrics import evaluate_trial, PredictiveHistory
except ModuleNotFoundError as error:
    if error.name != "predictive_metrics":
        raise
    import rospkg
    sys.path.insert(0, str(
        Path(rospkg.RosPack().get_path("place_estimation")) / "evaluation"
    ))
    from predictive_metrics import evaluate_trial, PredictiveHistory


class PredictiveEvaluationNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.expected_frame = rospy.get_param("~expected_frame", "base_footprint")
        self.min_variance = float(rospy.get_param("~min_variance", 1.e-8))
        if not np.isfinite(self.min_variance) or self.min_variance <= 0:
            raise ValueError("min_variance must be finite and positive")
        self.history = PredictiveHistory(int(rospy.get_param("~summary_window_size", 10)))
        self.max_completed = int(rospy.get_param("~max_completed_operations", 1000))
        if self.max_completed <= 0:
            raise ValueError("max_completed_operations must be positive")
        self.completed = set()
        self.completed_order = deque()
        self.metrics_pub = rospy.Publisher(
            rospy.get_param("~metrics_topic", "/predictive_evaluation"),
            String, queue_size=10,
        )
        self.summary_pub = rospy.Publisher(
            rospy.get_param("~summary_topic", "/predictive_evaluation_summary"),
            String, queue_size=1, latch=True,
        )
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~input_topic", "/predictive_evaluation_input"),
            String, self._callback, queue_size=100,
        )

    def _callback(self, message):
        try:
            snapshot = json.loads(message.data)
            if self.expected_frame and snapshot["frame_id"] != self.expected_frame:
                raise ValueError("predictive snapshot frame mismatch")
            key = (snapshot["operation_id"], snapshot["prior_stamp_ns"], snapshot["frame_id"])
            with self.lock:
                if key in self.completed:
                    return
                result = evaluate_trial(snapshot, self.min_variance)
                payload = json.dumps(result, allow_nan=False, sort_keys=True)
                summary = self.history.add(result)
                self.completed.add(key)
                self.completed_order.append(key)
                if len(self.completed_order) > self.max_completed:
                    self.completed.remove(self.completed_order.popleft())
                self.metrics_pub.publish(String(data=payload))
                self.summary_pub.publish(String(data=json.dumps(
                    summary, allow_nan=False, sort_keys=True
                )))
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError) as error:
            rospy.logerr_throttle(2.0, "Predictive evaluation failed: %s", error)


def main():
    rospy.init_node("predictive_evaluation_node")
    try:
        PredictiveEvaluationNode()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("PredictiveEvaluationNode failed: %s", error)


if __name__ == "__main__":
    main()
