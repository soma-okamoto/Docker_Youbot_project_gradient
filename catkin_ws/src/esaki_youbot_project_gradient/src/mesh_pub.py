#!/usr/bin/env python3

import rospy
from esaki_youbot_project_gradient.msg import mesh

def publisher():
    # ノードの初期化
    rospy.init_node('my_publisher')

    # パブリッシャの作成
    pub = rospy.Publisher('mesh_command', mesh, queue_size=10)

    # メッセージの作成
    msg = mesh()
    msg.mesh = "Hello ROS"
    print(type(msg))
    # メッセージのパブリッシュ
    rate = rospy.Rate(10)  # パブリッシュの周波数を設定
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
