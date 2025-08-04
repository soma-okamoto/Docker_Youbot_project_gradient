#! /usr/bin/env python3

import rospy, sys

from std_msgs.msg import Bool,Float32,String
# from std_msgs.msg import Float64
# from dataset_gen_msgs import GripperState
#from Dynamixel-workbench-msgs
# from dynamixel_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from dynamixel_workbench_operators.srv import *
from sensor_msgs.msg import JointState
import numpy as np


class gripper:

    pub_state=rospy.Publisher("/gripper_state", Bool, queue_size=100)

    def Range_callback(self, msg):
        global range
        range = msg
        print(range.data)


    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        # self.gripper_pub = rospy.Publisher('/grasp_command', String, queue_size=1)
        # self.flag_pub = rospy.Publisher('grasp_state', Bool, queue_size =1000)

            
        self.set_torque = 12

        # TODO : set_torque の値を変更する. ->計算式知らん
        
        self.open = False
        # subscribed Topic
        rospy.Subscriber("/dynamixel_workbench/joint_states",JointState, self.torqueCB,  queue_size = 10)
        rospy.Subscriber("/grasp_command",String, self.graspCB,  queue_size = 10)
        rospy.Subscriber("/graspRange",Float32,self.Range_callback,queue_size = 10)

        range = Float32()
        print(range)
        # self.pose_offset=Float64()
        self.offset_recorded=False
        # rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        # try:
        #     print('trying')
        #     #motor_1 set max torque
        #     command_1 = 'command'
        #     id_1 = 1
        #     addr_name_1 = 'Goal_Current'
        #     value_1 = self.set_torque
        #     command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        #     command_to_motor(command_1, id_1, addr_name_1, value_1)

        #     print('succeeded?')
            
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        
        # self.ef_state=rospy.Publisher("/ef_state", String, queue_size=1000)
        

    def graspCB(self, msg):
        #if there is command to grasp
        #call grasping server
        
        command = msg.data
        if (command =="close" and self.open):
            grasp_cmd = 'close'
            command_1 = 'command'
            id_1 = 1
            addr_name_1 = 'Goal_Current'
            value_1 = self.set_torque
            command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            command_to_motor(command_1, id_1, addr_name_1, value_1)
            print("closed")
            self.open = False

        elif (command=="open" and not self.open):
            grasp_cmd = 'open'
            command_1 = 'command'
            id_1 = 1
            addr_name_1 = 'Goal_Current'
            value_1 = self.set_torque
            command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            command_to_motor(command_1, id_1, addr_name_1, value_1)
            print("opened")
            self.open = True
        else:
            print("ignoring")
                

    def torqueCB(self, msg):
        #if there is command to grasp
        #call grasping server
        # print("checking the torque")
        temp = msg
        m_1_tor = temp.effort[0]

        if (m_1_tor < -100):
            object_grasped=True
            
        else:
            object_grasped=False
        # if torque >= 0.5:
        self.pub_state.publish(object_grasped)
        
        self.pub_state.publish(object_grasped)

        self.m_1_pos = temp.position[0]
        print(self.m_1_pos)
        print(self.m_1_pos/np.pi * 180)

        m_1_vel = temp.velocity[0]
       

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('gripper', anonymous=True)


    ic = gripper()
    print('init')

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
