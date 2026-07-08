#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import Vector3Stamped

class YoubotMapOffsetPublisher:
    def __init__(self):
        # ノードの初期化
        rospy.init_node('youbot_map_offset_publisher')

        # パラメータ設定
        # map_frame: SLAMの地図座標系
        # base_frame: ロボットの基準座標系 
        self.map_frame  = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        offset_topic = rospy.get_param('~offset_topic', '/youbot/offset')

        # Publisher（Vector3Stamped）を用意 (元のコードと同じ型)
        self.pub = rospy.Publisher(offset_topic, Vector3Stamped, queue_size=1)

        # TFリスナーの準備 (Subscriberの代わり)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 初期位置フラグと変数
        self.initialized = False
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0

        # ループ周期 (30Hz)
        self.rate = rospy.Rate(30.0)

    def run(self):
        rospy.loginfo(f"Start monitoring TF: {self.map_frame} -> {self.base_frame}")

        while not rospy.is_shutdown():
            try:
                # TFツリーから現在のロボット位置を取得
                # map から見た base_frame の位置
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame, 
                    self.base_frame, 
                    rospy.Time(0)
                )

                # 現在位置を取得
                pos = trans.transform.translation

                # 最初の取得時、その位置を「原点」として記録
                if not self.initialized:
                    self.x0 = pos.x
                    self.y0 = pos.y
                    self.z0 = pos.z
                    self.initialized = True
                    rospy.loginfo("YouBot Map Origin set: x0=%.3f, y0=%.3f, z0=%.3f", self.x0, self.y0, self.z0)

                # 現在位置との差分（オフセット）を計算
                dx = pos.x - self.x0
                dy = pos.y - self.y0
                dz = pos.z - self.z0

                # Vector3Stamped メッセージに詰め替え
                offset_msg = Vector3Stamped()
                offset_msg.header.stamp = rospy.Time.now()
                # header.frame_id は必要に応じて設定（Unity側で無視するならそのままでOK）
                offset_msg.header.frame_id = "unity_origin" 

                offset_msg.vector.x = dx
                offset_msg.vector.y = dy
                offset_msg.vector.z = dz

                # 配信
                self.pub.publish(offset_msg)
                rospy.loginfo("Origin set: x0=%.3f, y0=%.3f, z0=%.3f", dx, dy, dz)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # TFの取得に失敗した場合（起動直後など）はスキップ
                pass
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = YoubotMapOffsetPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass