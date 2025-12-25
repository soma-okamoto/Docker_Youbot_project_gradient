#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

def remove_obstacle():
    # 障害物がなくなったことを検出するロジックを実装

    # 障害物がなくなった場合、その位置とサイズを特定
    x = 0
    y = 0
    width = 1000
    height = 1000
    cost_value = 0  # 低いコスト値を設定

    # 更新する領域の情報を作成
    update_data = Int32MultiArray()
    update_data.data = [x, y, width, height, cost_value]

    # メッセージをパブリッシュ
    pub.publish(update_data)

    rospy.loginfo("Obstacle removed from costmap")

if __name__ == '__main__':
    try:
        rospy.init_node('update_costmap_example', anonymous=True)
        pub = rospy.Publisher('/move_base/global_costmap/costmap_updates', Int32MultiArray, queue_size=1)
        rate = rospy.Rate(1)  # メッセージ送信のレートを設定

        while not rospy.is_shutdown():
            remove_obstacle()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
