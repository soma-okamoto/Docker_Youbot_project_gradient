#!/usr/bin/env python3
"""Local ROS ID propagation test; starts only prior/fusion, no robot/YOLO/QR."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xmlrpc.client


def wait(predicate, label, timeout=8.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('timeout: ' + label)


def main():
    port = os.environ.get('BOTTLE_ID_TEST_PORT', '11335')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + port, ROS_IP='127.0.0.1',
                      ROS_HOME='/tmp/bottle_id_test_home', ROS_LOG_DIR='/tmp/bottle_id_test_logs',
                      PYTHONDONTWRITEBYTECODE='1')
    Path(os.environ['ROS_LOG_DIR']).mkdir(exist_ok=True)
    processes, logs = [], []
    try:
        commands = [
            ['roscore', '-p', port],
            ['rosrun', 'place_estimation', 'prior_distribution_node.py',
             '_enable_bottle_id:=true', '_expected_frame:=base_footprint',
             '_output_frame:=base_footprint', '_enable_error_learning:=false'],
            ['rosrun', 'place_estimation', 'observation_fusion_node_new.py',
             '_enable_bottle_id:=true', '_expected_frame:=base_footprint',
             '_output_frame:=base_footprint', '_enable_yolo:=false', '_meta_message_type:=disabled'],
        ]
        for index, command in enumerate(commands):
            log = open('/tmp/bottle_id_test_%d.log' % index, 'w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
            if index == 0:
                def ready():
                    try:
                        return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('test')[0] == 1
                    except OSError:
                        return False
                wait(ready, 'isolated ROS master')
        check_topics()
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for log in logs:
            log.close()


def check_topics():
    import numpy as np
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray
    from place_estimation.msg import BottleDistribution

    rospy.init_node('bottle_identity_smoke', anonymous=True)
    priors, results, legacy_results = [], [], []
    subscribers = [
        rospy.Subscriber('/P_pred_with_id', BottleDistribution, priors.append),
        rospy.Subscriber('/P_place_with_id', BottleDistribution, results.append),
        rospy.Subscriber('/P_place', PoseStamped, legacy_results.append),
    ]
    current = rospy.Publisher('/P_current', Float32MultiArray, queue_size=10)
    tf = rospy.Publisher('/P_tf', PoseStamped, queue_size=10)
    wait(lambda: current.get_num_connections() and tf.get_num_connections(), 'prior subscribers')
    wait(lambda: all(s.get_num_connections() for s in subscribers), 'output publishers')
    time.sleep(.2)
    for index, (bottle, x) in enumerate(((7, 1.), (12, 2.), (7, 3.)), 1):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        # Exercise both P_current/P_tf arrival orders with the same ID format.
        if index % 2:
            current.publish(Float32MultiArray(data=[bottle, x, 0., 0.]))
            tf.publish(pose)
        else:
            tf.publish(pose)
            current.publish(Float32MultiArray(data=[bottle, x, 0., 0.]))
        wait(lambda: len(results) >= index and len(legacy_results) >= index and len(priors) >= index,
             'ID-bearing result %d' % index)
        prior, result = priors[index-1], results[index-1]
        assert prior.bottle_id == result.bottle_id == bottle
        assert prior.header.frame_id == result.header.frame_id == 'base_footprint'
        assert abs(prior.position.x-x) < 1.e-6 and abs(result.position.x-x) < 1.e-6
        np.testing.assert_allclose(result.covariance, prior.covariance)
        assert abs(legacy_results[index-1].pose.position.x-result.position.x) < 1.e-6
    print('PASS: real ROS /P_current -> /P_pred_with_id -> /P_place_with_id; '
          'IDs 7,12,7; fallback covariance; legacy /P_place retained')


if __name__ == '__main__':
    main()
