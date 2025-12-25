#!/usr/bin/env python3

import rospy
from brics_actuator.msg import JointPositions, JointValue
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def cb(msg: JointPositions):
    names  = [jv.joint_uri for jv in msg.positions]
    angles = [jv.value     for jv in msg.positions]
    traj   = JointTrajectory()
    traj.joint_names    = names
    point = JointTrajectoryPoint()
    point.positions        = angles
    point.time_from_start  = rospy.Duration(1.0)
    traj.points = [point]
    pub.publish(traj)

if __name__=='__main__':
    rospy.init_node('position_to_trajectory_bridge')
    pub = rospy.Publisher(
        '/arm_1/arm_controller/command',
        JointTrajectory,
        queue_size=1)
    sub = rospy.Subscriber(
        '/arm_1/arm_controller/position_command',
        JointPositions,
        cb)
    rospy.spin()
