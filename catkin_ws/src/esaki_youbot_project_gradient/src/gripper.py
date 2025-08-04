#! /usr/bin/env python3

import rospy, sys
from std_msgs.msg import String
from std_msgs.msg import Bool
# from std_msgs.msg import Float64
# from dataset_gen_msgs import GripperState
#from Dynamixel-workbench-msgs
# from dynamixel_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from dynamixel_workbench_operators.srv import *
from sensor_msgs.msg import JointState


class gripper:

    
    set_torque = 75
    pub_state=rospy.Publisher("/gripper_state", Bool, queue_size=100)
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        # self.gripper_pub = rospy.Publisher('/grasp_command', String, queue_size=1)
        # self.flag_pub = rospy.Publisher('grasp_state', Bool, queue_size =1000)


        # subscribed Topic
        rospy.Subscriber("/dynamixel_workbench/joint_states",JointState, self.torqueCB,  queue_size = 10)
        rospy.Subscriber("/grasp_command",String, self.graspCB,  queue_size = 10)

        self.open = False
        # self.pose_offset=Float64()
        self.offset_recorded=False

        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        
        print('trying')
        #motor_1 set max torque
        command_1 = 'command'
        id_1 = 1
        addr_name_1 = 'Goal_Current'
        value_1 = (75)
        #value_1 = (100)
        command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        command_to_motor(command_1, id_1, addr_name_1, value_1)
        
        #motor_2 set max torque
        command_0 = 'command'
        id_0 = 0
        addr_name_0 = 'Goal_Current'
        value_0 = (75)
        #value_2 = (100)
        command_to_motor(command_0, id_0, addr_name_0, value_0)
        grasp_cmd = 'close'
        command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/execution', GripperCmd)
        command_to_motor(grasp_cmd)
        print('succeeded?')

    def graspCB(self, msg):
        command = msg.data
        if (command =="close" and self.open):
            grasp_cmd = 'close'
            command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/execution', GripperCmd)
            command_to_motor(grasp_cmd)
            print("closed")
            self.open = False

        elif (command=="open" and not self.open):
            grasp_cmd = 'open'
            command_to_motor = rospy.ServiceProxy('/dynamixel_workbench/execution', GripperCmd)
            command_to_motor(grasp_cmd)
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
        m_2_tor = temp.effort[1]
        avg = abs((m_1_tor + m_2_tor)/2)
        if (avg > 50):
            object_grasped=True
            
        else :
            object_grasped=False
        # if torque >= 0.5:
        self.pub_state.publish(object_grasped)
        pos1 = round(temp.position[0],2)
        pos2 = round(temp.position[1],2)
       


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('gripper', anonymous=True)
    ic = gripper()
    

    print('init')
    rospy.spin()
    
    # try:
        
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print "Shutting down ..."

if __name__ == '__main__':
    
    main(sys.argv)
