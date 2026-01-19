#!/usr/bin/env python3
import rospy
import numpy as np
from brics_actuator.msg import JointPositions, JointValue
from geometry_msgs.msg import PoseStamped

#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def position_callback(msg):
    global pose
    #rospy.loginfo("Message '{}' recieved".format(msg))
    pose = msg

def subscriber():
    # ノードを初期化する。
    rospy.init_node("subscriber")

    # 受信者を作成する。
    rospy.Subscriber("arm_2/arm_controller/position_command", JointPositions, position_callback)
    
    # ノードが終了するまで待機する。
    rospy.spin()


if __name__ == "__main__":
    subscriber()