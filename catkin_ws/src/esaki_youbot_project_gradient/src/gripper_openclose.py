#!/usr/bin/env python3

from traceback import print_tb
from turtle import pos
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


arm1 = math.pi/2
arm2 = 0
arm3 = math.pi
arm4 = math.pi/2
arm5 = 0
rospy.init_node('youbot_initialpotisition_publisher')
contoller_name='youbot/arm_1/gripper_controller/position_command'
trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
argv = sys.argv[1:]
arm_joints = ['gripper_finger_joint_l','gripper_finger_joint_r']

goal_positions = [-1.5,-1.5]

rospy.loginfo("Goal Position set lets go ! ")
rospy.sleep(0.1)

trajectory_msg = JointTrajectory()
trajectory_msg.joint_names = arm_joints
trajectory_msg.points.append(JointTrajectoryPoint())
trajectory_msg.points[0].positions = goal_positions
trajectory_msg.points[0].velocities = [0.0 for i in arm_joints]
trajectory_msg.points[0].accelerations = [0.0 for i in arm_joints]
trajectory_msg.points[0].time_from_start = rospy.Duration(3)
rospy.sleep(0.1)

trajectory_publihser.publish(trajectory_msg)