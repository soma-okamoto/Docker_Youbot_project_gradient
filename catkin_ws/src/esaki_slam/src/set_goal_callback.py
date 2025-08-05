#!/usr/bin/env python3

from traceback import print_tb
from rospkg import get_package_name
import rospy
from geometry_msgs.msg import PoseStamped

def basemove_callback(msg):
    global basepose
    #rospy.loginfo("Message '{}' recieved".format(msg.pose.position))
    basepose = msg
    #print(basepose)




if __name__ == '__main__':

    rospy.init_node('goal_subscriber')
    pose_sub = rospy.Subscriber("/basepose_command",PoseStamped,basemove_callback)
    basepose = PoseStamped()
    
    while not rospy.is_shutdown():
        print(basepose)
        rospy.sleep(0.2)

