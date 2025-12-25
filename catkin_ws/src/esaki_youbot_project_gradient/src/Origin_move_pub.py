#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

class YoubotOffsetPublisher:
    def __init__(self):
        # ノードの初期化
        rospy.init_node('youbot_offset_publisher')

        # 購読トピック（Odometry）と配信トピックを設定
        odom_topic = rospy.get_param('~odom_topic', '/odom')

        offset_topic = rospy.get_param('~offset_topic', '/youbot/offset')

        # Publisher（Vector3Stamped）を用意
        self.pub = rospy.Publisher(offset_topic, Vector3Stamped, queue_size=1)

        # 初期位置フラグ
        self.initialized = False

        # Odometry を購読
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position

        # 最初のコールバックで初期位置を記録
        if not self.initialized:
            self.x0 = pos.x
            self.y0 = pos.y
            self.z0 = pos.z
            self.initialized = True
            rospy.loginfo("YouBot origin set: x0=%.3f, y0=%.3f, z0=%.3f", self.x0, self.y0, self.z0)

        # 現在位置との差分（オフセット）を計算
        dx = pos.x - self.x0
        dy = pos.y - self.y0
        dz = pos.z - self.z0

        # Vector3Stamped メッセージに詰め替え
        offset_msg = Vector3Stamped()
        offset_msg.header.stamp = rospy.Time.now()
        offset_msg.vector.x = dx
        offset_msg.vector.y = dy
        offset_msg.vector.z = dz

        # 配信
        self.pub.publish(offset_msg)
        #print(offset_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = YoubotOffsetPublisher()
    node.run()
