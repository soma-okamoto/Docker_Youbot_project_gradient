#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

if __name__ == '__main__':
    rospy.init_node('edit_plan_node', anonymous=True)
    threshold = 1
    
    # Publisherの設定
    editplan_pub = rospy.Publisher('plan_pc', PointCloud2, queue_size=1)



    rospy.spin()