#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def move_callback(msg):
    rospy.loginfo("Message '{}' recieved".format(msg.pose.position))

def move_subscriber():
    rospy.init_node("move_subscriber")

    rospy.Subscriber("palm_pose",PoseStamped,move_callback)

    rospy.spin()


if __name__ == '__main__':
    move_subscriber()