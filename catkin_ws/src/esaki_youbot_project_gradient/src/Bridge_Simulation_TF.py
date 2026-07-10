#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped


class BridgeSimulationTF:
    def __init__(self):
        rospy.init_node("bridge_simulation_tf")

        self.listener = tf.TransformListener()
        self.br = tf2_ros.TransformBroadcaster()

        # =========================
        # Base frame
        # =========================
        self.base_frame = "base_footprint"

        # =========================
        # Simulation TF names
        # =========================
        self.sim_arm1_link5_frame = "arm_link_5"
        self.sim_camera_rgb_optical_frame = "youbot/camera_rgb_optical_frame"

        # =========================
        # Bridge output TF names
        # 実機側コードで使いやすい名前に合わせる
        # =========================
        self.out_gripper_tip_frame = "gripper_tip"
        self.out_camera_rgb_optical_frame = "asus_rgb_optical_frame"

        # =========================
        # Static relative transforms
        # =========================
        # gripper_tip relative to arm_link_5
        self.tip_offset_xyz = (0.0, 0.0, 0.09)
        self.tip_offset_qxyzw = (0.0, -0.70711, 0.0, 0.70711)

        rospy.loginfo("BridgeSimulationTF ready")

    def make_transform_msg(self, parent, child, xyz, qxyzw):
        msg = TransformStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent
        msg.child_frame_id = child

        msg.transform.translation.x = xyz[0]
        msg.transform.translation.y = xyz[1]
        msg.transform.translation.z = xyz[2]

        msg.transform.rotation.x = qxyzw[0]
        msg.transform.rotation.y = qxyzw[1]
        msg.transform.rotation.z = qxyzw[2]
        msg.transform.rotation.w = qxyzw[3]

        return msg

    def lookup_transform(self, parent, child):
        try:
            self.listener.waitForTransform(
                parent,
                child,
                rospy.Time(0),
                rospy.Duration(0.5)
            )

            trans, rot = self.listener.lookupTransform(
                parent,
                child,
                rospy.Time(0)
            )

            return trans, rot

        except tf.Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "Failed to lookup TF %s -> %s : %s",
                parent,
                child,
                str(e)
            )
            return None, None

    def publish_gripper_tip_tf(self):
        """
        シミュレーション側に gripper_tip がないので、
        arm_link_5 からの相対TFとして gripper_tip を作る。
        """
        msg = self.make_transform_msg(
            parent=self.sim_arm1_link5_frame,
            child=self.out_gripper_tip_frame,
            xyz=self.tip_offset_xyz,
            qxyzw=self.tip_offset_qxyzw
        )

        self.br.sendTransform(msg)

    def publish_camera_alias_tf(self):
        """
        シミュレーション側:
            youbot/camera_rgb_optical_frame

        実機側コードで使いたい名前:
            asus_rgb_optical_frame

        base_footprint基準で同じ姿勢を持つ別名TFとして再配信する。
        """
        trans, rot = self.lookup_transform(
            self.base_frame,
            self.sim_camera_rgb_optical_frame
        )

        if trans is None:
            return

        msg = self.make_transform_msg(
            parent=self.base_frame,
            child=self.out_camera_rgb_optical_frame,
            xyz=trans,
            qxyzw=rot
        )

        self.br.sendTransform(msg)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publish_gripper_tip_tf()
            self.publish_camera_alias_tf()
            rate.sleep()


if __name__ == "__main__":
    node = BridgeSimulationTF()
    node.run()