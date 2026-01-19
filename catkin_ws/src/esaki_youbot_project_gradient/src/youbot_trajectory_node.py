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


def DegToRad(th):
    rad = (np.pi/180)*th
    return rad
def perform_trajectory(arm1,arm2,arm3,arm4,arm5):
    rospy.init_node('youbot_trajectory_publisher')
    contoller_name='youbot/arm_1/arm_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
    argv = sys.argv[1:]                         
    arm_joints = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']

    q1 = -DegToRad(-90)+DegToRad(169)
    q2 = DegToRad(60)+DegToRad(65)
    q3 = DegToRad(-90)-DegToRad(146)
    q4 = DegToRad(-80)+DegToRad(102.5)
    q5 = DegToRad(0)+DegToRad(167.5)
    #goal_positions = [q1,q2,q3,q4,q5]
    goal_positions = [arm1,arm2,arm3,arm4,arm5]

    #rospy.loginfo("Goal Position set lets go ! ")
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


def tap_callback(msg):
    rospy.loginfo("Message '{}' recieved".format(msg.data))

def tap_subscriber():
    rospy.Subscriber("airtap_command",String,tap_callback)

def move_callback(msg):
    global pose
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    pose = msg

def move_subscriber():
    rospy.Subscriber("palm_pose",PoseStamped,move_callback)


'''

def Theta0(x,y):
    if x == 0 and y == 0:
        theta_0 = 0
    theta_0 = math.atan2(x,y)
    return theta_0

def Theta1(x,y,z,L):

    """
    theta_2 = Theta2(x,y,z,L)
    L14 = math.sqrt(x**2+y**2+(z-L[0])**2)
    theta1_1 = math.asin((L[2]/L14)*math.sin(math.pi-theta_2))
    theta1_2 = math.atan2(z - L[0],math.sqrt(x*x+y*y))
    theta_1 = math.pi/2 - (theta1_1 + theta1_2)
    """
    L14 = math.sqrt(x**2+y**2+(z-L[0])**2)
    L04 = math.sqrt(x**2+y**2+z**2)
    print("L04",L04)
    theta1_1  =math.acos((L[1]**2 + L14**2 - L[2]**2)/2*L[1]*L14)
    theta1_2 = math.atan2(z - L[0],math.sqrt(x*x+y*y))
    theta_1 = math.pi/2 - (theta1_1+theta1_2)
    """    print("z",z)
        print("L[0]",L[0])
        print("L[14]",L14)
        print("theta1_1",theta1_1)
        print("theta1_2",theta1_2)
        print("theta_1",theta_1)"""


    return theta_1


def Theta2(x,y,z,L):

    L14 = math.sqrt(x**2+y**2+(z-L[0])**2)
    print("z",z)
    print("L[0]",L[0])
    print("L[14]",L14)
    print(L14**2)
    print(L[1]**2)
    print(L[2]**2)
    print(-(L[1]**2 + L[2]**2 - L14**2))
    print((2*L[1]*L[2]))
    print(-(L[1]**2 + L[2]**2 - L14**2)/(2*L[1]*L[2]))

    theta_2 = math.pi - math.acos((L[1]**2 + L[2]**2 - L14**2)/(2*L[1]*L[2]))
    
    print(theta_2)
    return theta_2

def theta234(x,y,z,L):
    px = x
    py = y
    pz = z

    L = [0.147,0.155,0.135,0.218]


    xc = math.sqrt(px*px+py*py)
    zc = pz - L[0]

    phi_c = 0

    xw = xc - L[3]*math.cos(phi_c)
    zw = zc - L[3]*math.sin(phi_c)

    alpha = math.atan2(zw,xw)

    cos_beta = (L[1]*L[1] + L[2]*L[2] - xw*xw -zw*zw)/(2*L[1]*L[2])
    sin_beta = math.sqrt(abs(1-(cos_beta*cos_beta)))
    theta_link_2 = math.pi - math.atan2(sin_beta,cos_beta)

    cos_gama = (xw*xw + zw*zw + L[1]*L[1] - L[2]*L[2])/(2*L[1]*math.sqrt(xw*xw + zw*zw))
    sin_gama = math.sqrt(abs(1 - (cos_gama*cos_gama)))

    theta_link_1 = math.pi/2 - alpha - math.atan2(sin_gama,cos_gama)

    theta_link_1p = theta_link_1 + 2* math.atan2(sin_gama,cos_gama)
    theta_link_2p = theta_link_2 - math.pi

    theta_2 = theta_link_1p
    theta_3 = theta_link_2p
    theta_4 = theta_2 + theta_3

    print("theta_2",theta_2)
    print("theta_3",theta_3)
    print("theta_4",theta_4)
    rad_2 = theta_2 * (180/math.pi)
    rad_3 = theta_3 * (180/math.pi)
    rad_4 = theta_4 * (180/math.pi)

    print(rad_2,rad_3,rad_4)

    return theta_2,theta_3,theta_4

'''


import math
import numpy as np


def Theta0(x,y):
    if x == 0 and y == 0:
        theta_0 = 0
    theta_0 = math.atan2(x,y)
    return theta_0


def fk(L,theta0,theta1,theta2):
    y = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1) + L[2]*math.cos(theta0+theta1+theta2)
    x = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1) + L[2]*math.sin(theta0+theta1+theta2)
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

    r21 = -L[0]*math.sin(theta0)-L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r22 = -L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r23 = -L[2]*math.sin(theta0+theta1+theta2)

    r11 = L[0]*math.cos(theta0)+L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r12 = L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r13 = L[2]*math.cos(theta0+theta1+theta2)
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
    
    arm_joint_1 = DegToRad(169)
    arm_joint_2 = 0
    arm_joint_3 = math.pi
    arm_joint_4 = math.pi/2
    arm_joint_5 = 0

    arm_joint_1_0 = DegToRad(169)
    arm_joint_2_0 = -DegToRad(65)
    arm_joint_3_0 = DegToRad(146)
    arm_joint_4_0 = -DegToRad(102.5)
    arm_joint_5_0 = DegToRad(167.5)


    rate = rospy.Rate(10)
    
    w = np.diag([0.1,0.1])
    j = 0
    while not rospy.is_shutdown():

        pose_holo = [pose.pose.position.y,pose.pose.position.x,pose.pose.position.z]
        quaternion = [pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w]
        r = quaternion_to_euler(quaternion)
        #print("quaternion",quaternion)
        #print("eular",r)
        
        pose_xy = math.sqrt(pose_holo[0]*pose_holo[0] + pose_holo[1]*pose_holo[1])
        pose_z = pose_holo[2]
        pose_holo_2d = [pose_holo[0],pose_holo[2],0]

        pose_test = [0.419,0.318-0.147,r[1]*4]
        #pose_3d = [pose_xy,pose_z,r[2]]
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
        theta_0 = DegToRad(169/2)- Theta0(pose_holo[1],pose_holo[0])
        tehta_5 = r[1]*4

        if theta_0 >= math.pi/2 + 0.5:
            theta_0 = math.pi/2 + 0.5

        if theta[0] >= math.pi/2:
            theta[0] = math.pi/2
        if theta[1] <= -math.pi/2:
            theta[1] = - math.pi/2
            
        

        perform_trajectory( theta_0,theta[0]+DegToRad(65),theta[1]-DegToRad(146),theta[2]+DegToRad(102.5),DegToRad(167.5))
        cand = [theta_0,theta[0]+DegToRad(65),theta[1]-DegToRad(146),theta[2]+DegToRad(102.5),DegToRad(167.5)]
        #perform_trajectory(arm_joint_1_0 + Theta0(pose_holo[1],pose_holo[0]),arm_joint_2_0 +theta[0],arm_joint_3_0+theta[1],arm_joint_4_0+theta[2],arm_joint_5_0)
        #perform_trajectory(math.pi/2,0.1,0.1,0.1,0)


        if j >10:
            #print("theta_0",theta)
            #print("theta",theta)
            #print("pd",pose_holo)
            #print("pd_3d",pose_3d)
            #print("p",fk(L,theta[0],theta[1],theta[2]))
            j = 0
        j+=1

        #print(theta[0] * 180/math.pi,theta[1] * 180/math.pi,theta[2] * 180/math.pi)
        #perform_trajectory(arm_joint_1,theta[0],theta[1],theta[2],arm_joint_5)
        rate.sleep()




"""

if __name__ == '__main__':
    rospy.init_node('youbot_trajectory_publisher')
    #L = [0.147,0.155,0.135,0.218]
    arm_joint_1 = math.pi/2
    arm_joint_2 = 0
    arm_joint_3 = math.pi
    arm_joint_4 = math.pi/2
    arm_joint_5 = 0

    pos = [0,0,0.655]
    q = [0,0,0,0]
    #theta = inverse_kinematic(pos[0],pos[1],pos[2],q)
    #print(theta)
    #tap_subscriber()
    #move_subscriber()
    pose_sub = rospy.Subscriber("palm_pose",PoseStamped,move_callback)
    pose = PoseStamped()
    perform_trajectory(arm_joint_1,arm_joint_2,arm_joint_3,arm_joint_4,arm_joint_5)
    #1.5694864886023367
    #1.5687758502618183
    #0.2899986331002269
    #0.2899986331002269
    rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    '''
        x = 0.3
        y = 0
        #z = (0.2 + pose.pose.position.z)/2
        z  = 0
        theta1,theta2,theta3 =theta234(x,y,z,L)
        theta0 = Theta0(pose.pose.position.x,pose.pose.position.y)
        
        #theta1 = Theta1(x,y,z,L)
        #theta2 = Theta2(x,y,z,L)
        print(x*x+y*y+z*z)
        d = math.sqrt(x*x+y*y+z*z) 
        print("d",d)
        if (d>0.5):
            
            rospy.loginfo("Points are out of the work space")

        else:
            
            #perform_trajectory(arm_joint_1,theta1,theta2,theta3,arm_joint_5)
        
            perform_trajectory(arm_joint_1,arm_joint_2,arm_joint_3,arm_joint_4,arm_joint_5)

            #rospy.loginfo(theta2)'''
    p0 = [0.1,0.2,0.3]

    for i in range(1000):
        
        theta_np = inversekinematics(L,pd,p0)
        p0[0] += float(theta_np[0,0])
        p0[1] += float(theta_np[1,0])
        p0[2] += float(theta_np[2,0])
        theta = [p0[0],p0[1],p0[2]]
            
    print(theta)
    print(p0[0] * 180/math.pi,p0[1] * 180/math.pi,p0[2] * 180/math.pi)
    perform_trajectory(0, 0,0,0,0)
        #rate.sleep()
"""