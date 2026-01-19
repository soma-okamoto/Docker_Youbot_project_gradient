#!/usr/bin/env python3

import sys
import copy
import rospy
import math
from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler #, quaternion_about_axis, quaternion_from_matrix, rotation_matrix

import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import time

arm_2_topic_name = "arm_2/arm_controller/position_command"
arm_2_msg_type = JointPositions
grip_2_topic_name = "arm_2/gripper_controller/position_command"
grip_2_msg_type = JointPositions

joint_uri_2 = ['arm_2_joint_1','arm_2_joint_2','arm_2_joint_3','arm_2_joint_4','arm_2_joint_5','gripper_2_finger_joint_l','gripper_2_finger_joint_r']


# Turn a desired gripper opening into a brics_actuator-friendly message
def make_arm_msg(arm_js, joint_uri):
    
    A1 = arm_js[0]
    A2 = arm_js[1]
    A3 = arm_js[2]
    A4 = arm_js[3]
    A5 = arm_js[4] 
    # create joint positions message
    jp = JointPositions()
    # create joint values message for all the joints
    jv1 = JointValue()
    jv2 = JointValue()
    jv3 = JointValue()
    jv4 = JointValue()
    jv5 = JointValue()
    # Fill in the arm positions. 
    jv1.joint_uri = joint_uri[0]
    jv1.unit = 'rad'
    jv1.value = A1
    jv2.joint_uri = joint_uri[1]
    jv2.unit = 'rad'
    jv2.value = A2
    jv3.joint_uri = joint_uri[2]
    jv3.unit = 'rad'
    jv3.value = A3
    jv4.joint_uri = joint_uri[3]
    jv4.unit = 'rad'
    jv4.value = A4
    jv5.joint_uri = joint_uri[4]
    jv5.unit = 'rad'
    jv5.value = A5
    # Append those onto JointPositions
    jp.positions.append(copy.deepcopy(jv1))
    jp.positions.append(copy.deepcopy(jv2))
    jp.positions.append(copy.deepcopy(jv3))
    jp.positions.append(copy.deepcopy(jv4))
    jp.positions.append(copy.deepcopy(jv5))
    
    return jp

def DegToRad(th):
    rad = (np.pi/180)*th
    return rad


if __name__ == '__main__':

    # q1 = -DegToRad(-90)+DegToRad(169)
    q1 = -DegToRad(-60)+DegToRad(169)
    q2 = DegToRad(70)+DegToRad(65)
    # q3 = DegToRad(-110)-DegToRad(146)
    q3 = DegToRad(-105)-DegToRad(146)
    q4 = DegToRad(-60)+DegToRad(102.5)
    q5 = -DegToRad(0)+DegToRad(167.5)
    
    rospy.init_node('youbot_camera_trajectory_publisher')

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        time_sta = time.time()

        arm_2_command_publisher = rospy.Publisher(arm_2_topic_name, arm_2_msg_type, queue_size = 5)
        cand = [q1,q2,q3,q4,q5]
        #cand = [1.57,0,-math.pi,math.pi/2,math.pi]
        arm_2_cmd = make_arm_msg(cand, joint_uri_2)
        arm_2_command_publisher.publish(arm_2_cmd)

        rate.sleep()
        time_end = time.time()

        tim = time_end- time_sta

