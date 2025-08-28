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
from std_msgs.msg import String , Float32MultiArray
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import time

arm_1_topic_name = "arm_1/arm_controller/position_command"
arm_1_msg_type = JointPositions
grip_1_topic_name = "arm_1/gripper_controller/position_command"
grip_1_msg_type = JointPositions
joint_uri_1 = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5','gripper_finger_joint_l','gripper_finger_joint_r']

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



def move_arm_1(cmd):
  # Create a publisher to arm controller
  arm_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)

  # setup the loop rate for the node
  r = rospy.Rate(10) # 10hz
  
  # Make sure you put in valid joint states or bad things happen
  arm_cmd = make_arm_msg(cmd, joint_uri_1)
  # Initialize the timer for arm publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving arm")
      arm_command_publisher.publish(arm_cmd)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break



def move_arms(cmd_1):
  # Create a publisher to arm controller
  arm_1_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)
  # setup the loop rate for the node
  r = rospy.Rate(10) # 10hz
  
  # Make sure you put in valid joint states or bad things happen
  arm_1_cmd = make_arm_msg(cmd_1, joint_uri_1)
  # Initialize the timer for arm publisher
  while not rospy.is_shutdown():
      rospy.loginfo("moving arms")
      arm_1_command_publisher.publish(arm_1_cmd)



# ***MAIN FUNCTION***
def youbot_exec(arm_1_topic_name, arm_1_msg_type, grip_1_topic_name, grip_1_msg_type):

  # Initialize rospy
  rospy.init_node('youbot_trajectory_publisher',
                  anonymous=False)

  pose_sub = rospy.Subscriber("palm_pose",PoseStamped,move_callback)
  pose = PoseStamped()


  while not rospy.is_shutdown(): 
      pose_holo = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z+0.3]
      theta_0 = math.pi/2 - Theta0(pose_holo[1],pose_holo[0])
      if theta_0 >= math.pi/2 + 0.5:
        theta_0 = math.pi/2 + 0.5
      print(pose_holo)
      cand = [theta_0,0,0,0,0]
      move_arms(cand)



def Theta0(x,y):
    if x == 0 and y == 0:
        theta_0 = 0
    theta_0 = math.atan2(x,y)
    return theta_0

def move_callback(msg):
    global pose
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    pose = msg



def move_subscriber():

    rospy.Subscriber("palm_pose",PoseStamped,move_callback)


def Theta0(x,y):
    if x == 0 and y == 0:
        theta_0 = 0
    theta_0 = math.atan2(y,x)
    return theta_0


def fk(L,theta0,theta1,theta2):

    x = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1) + L[2]*math.sin(theta0+theta1+theta2)
    y = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1) + L[2]*math.cos(theta0+theta1+theta2)
    theta = theta0 + theta1 + theta2
    p = [x,y,theta]
    return p

def velocity(pd,p):
    k = 0.1
    vx = k * (pd[0] - p[0])
    vz = k * (pd[1] - p[1])
    omega = k * (pd[2] - p[2])
    
    v = np.array([[vx], [vz],[omega]])
    return v

def jacobian(L,theta0,theta1,theta2):

    r11 = L[0]*math.cos(theta0)+L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r12 = L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r13 = L[2]*math.cos(theta0+theta1+theta2)

    r21 = -L[0]*math.sin(theta0)-L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r22 = -L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r23 = -L[2]*math.sin(theta0+theta1+theta2)

    r31 = 1
    r32 = 1
    r33 = 1

    J = np.matrix(
        [
            [r11,r12,r13],
            [r21,r22,r23],
            [r31,r32,r33]

        ]
    )

    return J


def inversekinematics(L,pd,p0,ramda):
    p_fk = fk(L,p0[0],p0[1],p0[2])
    v = velocity(pd,p_fk)


    J = jacobian(L,p0[0],p0[1],p0[2])

    i = np.identity(3)
    J_T = np.transpose(J)

    SR = (np.dot(J_T,J) + ramda*i)

    SR_1 = np.linalg.pinv(SR)
    
    J_SR = np.dot(SR_1,J_T)

    angular_velocity = np.dot(J_SR,v)
    
    theta = 0.1 * angular_velocity
    
    return theta

import tf

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """

    e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    euler = [e[0],e[1],e[2]]

    return euler



def DegToRad(th):
    rad = (np.pi/180)*th
    return rad

def RadToDeg(th):
    Deg = (180/np.pi)*th
    return Deg


import csv
import datetime
if __name__ == '__main__':

    #pd = [0.404,0.3-0.147,0]
    pd = [1.419,0.318-0.147,0]
    p0 = [0.1,0.2,0.3]
    L = [0.155,0.135,0.218]

    theta0 = 0
    theta1 = 0
    theta2 = 0

    rospy.init_node('youbot_trajectory_publisher')

    pose_sub = rospy.Subscriber("palm_pose",PoseStamped,move_callback)

    pose = PoseStamped()
    
    arm_joint_1 = math.pi/2
    arm_joint_2 = 0
    arm_joint_3 = math.pi
    arm_joint_4 = math.pi/2
    arm_joint_5 = 0

    arm_joint_1_0 = math.pi/2
    arm_joint_2_0 = math.pi/4
    arm_joint_3_0 = math.pi*2/3
    arm_joint_4_0 = math.pi/2
    arm_joint_5_0 = 0


    rate = rospy.Rate(10)
    
    w = np.diag([0.1,0.1])
    j = 0
    while not rospy.is_shutdown():

        if(pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z == 0):
            #print("initialize position")
            pose_holo = [0,0,0.2]
        else:
            pose_holo = [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]

        time_sta = time.time()

        arm_1_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)
 

        arm_1_joint_publisher = rospy.Publisher('/arm_joint_command', Float32MultiArray, queue_size=10)

        quaternion = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        r = quaternion_to_euler(quaternion)
        #print(r)
        
        pose_xy = math.sqrt(pose_holo[0]*pose_holo[0] + pose_holo[1]*pose_holo[1])
        pose_z = pose_holo[2]
        pose_holo_2d = [pose_holo[0],pose_holo[2],0]

        pose_test = [0.419,0.318-0.147,r[1]*4]
        pose_3d = [pose_xy,pose_z,DegToRad(90)]
        #print(pose_3d)
        

        ramda = 1 
        
        for i in range(1000):
            
            p_old = fk(L,p0[0],p0[1],p0[2])
            error = np.array([[pose_3d[0] - p_old[0]], [pose_3d[1] - p_old[1]]])
            error_T = np.transpose(error)

            squarederror = np.dot(error_T,w)
            e = np.dot(squarederror,error)

            """if e <0.001:
                break"""
                
            ramda = e + 0.002
            theta_np = inversekinematics(L,pose_3d,p0,ramda)

            theta0 += float(theta_np[0,0])
            theta1 += float(theta_np[1,0])
            theta2 += float(theta_np[2,0])

            p0[0] = theta0
            p0[1] = theta1
            p0[2] = theta2

            theta = [p0[0],p0[1],p0[2]]

        theta_0 = DegToRad(169/2) - Theta0(pose_holo[1],pose_holo[0])

        if(theta_0 > DegToRad(110)):
            theta_0 = DegToRad(110)

        if(theta_0 < DegToRad(15)):
            theta_0 = DegToRad(15)
    
        cand = [theta_0,DegToRad(65)- theta[0] ,-DegToRad(146) - theta[1],DegToRad(102.5) - theta[2],DegToRad(167.5) ]
        print(cand)
        cand_deg = [RadToDeg(theta_0) ,RadToDeg(theta[0]),RadToDeg(theta[1]),RadToDeg(theta[2]),RadToDeg(cand[4])]

        arm1_joint_pub = Float32MultiArray(data=cand_deg)
        arm_1_joint_publisher.publish(arm1_joint_pub)

        
        arm_1_cmd = make_arm_msg(cand, joint_uri_1)
        arm_1_command_publisher.publish(arm_1_cmd)


        rate.sleep()
        time_end = time.time()

        tim = time_end- time_sta

        test_pub = rospy.Publisher("test_pose",PoseStamped,queue_size = 5)

        test = fk(L,theta[0],theta[1],theta[2])
        test_pose = PoseStamped()


        test_pose.pose.position.x ,test_pose.pose.position.y,test_pose.pose.position.z = test[0]*np.cos(Theta0(pose_holo[1],pose_holo[0])),test[0]*np.sin(Theta0(pose_holo[1],pose_holo[0])),test[1]-0.2

        test_pub.publish(test_pose)
   


def plan_grasp_targets(bottle_xyz, q_curr, L):
    """
    入力:
      bottle_xyz: [bx, by, bz]  世界（ベース）座標  /identified_bottle_pose"をサブスクライブする
      q_curr:     [q1..q5]      現在関節角（rad）
      L:          アーム各リンク長 [L1,L2,L3]
    出力:
      target_pose_3d: [radius, z, phi]  (元コードの IK 入力形式)
      stage: "APPROACH" | "ALIGN" | "INSERT"
    """
    bx, by, bz = bottle_xyz

    # 半径 r と 高さ z を生成（元の fk/jacobian に合わせて半径・高さで制御）
    r_goal = math.sqrt(bx*bx + by*by)
    z_goal = bz + GRASP_CLEAR_Z

    # 現在の末端(半径,高さ,合成角) を forward kinematics で推定
    p_curr = fk(L, q_curr[1], q_curr[2], q_curr[3])  # [radius, height, theta] 相当
    r_now, z_now, phi_now = p_curr

    # ステージ判定
    if abs(r_now - r_goal) > (STANDOFF_R + INSERT_THRESH_R):
        # まだ横方向に遠い → STANDOFF まで寄る（姿勢は変えない）
        r_tgt  = max(r_goal - STANDOFF_R, 0.0)
        z_tgt  = z_goal
        phi_tgt = phi_now  # いきなり姿勢を変えない
        stage  = "APPROACH"
    elif abs(z_now - z_goal) > INSERT_THRESH_Z:
        # 横はOK → 高さを合わせる（姿勢はまだ固定しない）
        r_tgt  = r_now
        z_tgt  = z_goal
        phi_tgt = phi_now
        stage  = "ALIGN"
    else:
        # 最終突入 → 半径を目標に寄せながら姿勢を“つかめる角度”へ
        r_tgt  = r_goal
        z_tgt  = z_goal
        phi_tgt = math.radians(YAW_LOCK_DEG)       # 例: 水平方向にグリッパ
        # 角度誤差は wrap
        phi_tgt = phi_now + angle_wrap(phi_tgt - phi_now)
        stage  = "INSERT"

    return [r_tgt, z_tgt, phi_tgt], stage
