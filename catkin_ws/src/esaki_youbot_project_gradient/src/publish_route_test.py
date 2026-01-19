#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

def publish_path():
    rospy.init_node('path_publisher', anonymous=True)
    path_publisher = rospy.Publisher('/path_with_penalty', Path, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Path メッセージの生成
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"  # フレームIDを設定 (例: "map")

        # 経路上のポーズを設定
        for i in range(10):  # 10個のポーズを生成
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"  # フレームIDを設定 (例: "map")

            # ポーズの位置 (x, y, z) を設定
            pose.pose.position.x = i * 1.0  # x方向に1m間隔で増加
            pose.pose.position.y = i * 0.5  # y方向に0.5m間隔で増加
            pose.pose.position.z = 0.0      # zは常に0 (地面)

            # 向き (四元数) を設定
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # 単位四元数 (回転なし)

            # Pathにポーズを追加
            path.poses.append(pose)

        # トピックにPathをパブリッシュ
        rospy.loginfo("Publishing Path message")
        path_publisher.publish(path)

        rate.sleep()
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
