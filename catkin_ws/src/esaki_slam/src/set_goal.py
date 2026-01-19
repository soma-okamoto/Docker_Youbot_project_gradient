#!/usr/bin/env python3

from email.mime import base
import imp
from traceback import print_tb
from rospkg import get_package_name
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class Goal:
    def __init__(self):
        self.ps_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    
    def setGoal(self,px,py,pz,ow):
        rospy.sleep(1.0)

        now = rospy.Time.now()
        goal_point = PoseStamped()
        
        goal_point.pose.position.x = py
        goal_point.pose.position.y = px
        goal_point.pose.position.z = pz
        goal_point.pose.orientation.z = np.pi/2
        goal_point.pose.orientation.w = ow
        goal_point.header.stamp = now
        goal_point.header.frame_id = 'map'

        self.ps_pub.publish(goal_point)


def basemove_callback(msg):
    global basepose
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    basepose = msg
    #print(basepose)



if __name__ == '__main__':

    rospy.init_node('set_goal')
    pose_sub = rospy.Subscriber("/basepose_command",PoseStamped,basemove_callback)
    basepose = PoseStamped()
    
    while not rospy.is_shutdown():
        if basepose.pose.position.x ==0:
            print(basepose.pose.position)
            goal_ob = Goal()
            goal_ob.setGoal(-basepose.pose.position.x,basepose.pose.position.y,0,1)
        rospy.sleep(0.2)