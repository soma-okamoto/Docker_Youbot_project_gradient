#!/usr/bin/env python3

import copy
import rospy
import math
import time
import numpy as np
import tf

from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


arm_2_topic_name = "arm_2/arm_controller/position_command"
arm_2_msg_type = JointPositions

joint_uri_2 = [
    'arm_2_joint_1',
    'arm_2_joint_2',
    'arm_2_joint_3',
    'arm_2_joint_4',
    'arm_2_joint_5'
]


def make_arm_msg(arm_js, joint_uri):
    jp = JointPositions()

    for i in range(5):
        jv = JointValue()
        jv.joint_uri = joint_uri[i]
        jv.unit = 'rad'
        jv.value = arm_js[i]
        jp.positions.append(copy.deepcopy(jv))

    return jp


def DegToRad(th):
    return (np.pi / 180.0) * th


class YoubotCameraArmController:
    def __init__(self):
        rospy.init_node('youbot_camera_trajectory_publisher')

        # =========================
        # TF settings
        # =========================
        self.listener = tf.TransformListener()

        self.base_frame = "base_footprint"

        # Arm1のEEフレーム
        # 必要なら gripper_palm_link などに変更
        # self.arm1_ee_frame = "arm_link_5"
        self.arm1_ee_frame = "gripper_tip"

        self.arm2_camera_frame = "asus_rgb_optical_frame"


        # =========================
        # Hold settings
        # =========================
        self.hold_active = False
        self.last_hold_time = rospy.Time(0)

        # Holdコマンドが途切れてから何秒でHold解除するか
        self.hold_timeout = rospy.Duration(0.3)

        # Place時に記録したArm1 EE
        self.place_ee_pose = None

        # =========================
        # Publisher
        # =========================
        self.arm_2_command_publisher = rospy.Publisher(
            arm_2_topic_name,
            arm_2_msg_type,
            queue_size=5
        )

        # 確認用：Place時点のArm1 EE
        self.place_ee_pub = rospy.Publisher(
            "/arm1_ee_at_place",
            PoseStamped,
            queue_size=10
        )

        # 確認用：Hold中のArm1 EE
        self.hold_ee_pub = rospy.Publisher(
            "/arm1_ee_current_on_hold",
            PoseStamped,
            queue_size=10
        )

        # =========================
        # Subscriber
        # =========================
        rospy.Subscriber('/place_command', String, self.callback_place_command)
        rospy.Subscriber('/Hold_command', String, self.callback_hold_command)

        # =========================
        # Initial Arm2 posture
        # =========================
        self.q1 = -DegToRad(-60) + DegToRad(169)
        self.q2 = DegToRad(70) + DegToRad(65)
        self.q3 = DegToRad(-105) - DegToRad(146)
        self.q4 = DegToRad(-60) + DegToRad(102.5)
        self.q5 = -DegToRad(0) + DegToRad(167.5)

        rospy.loginfo("YoubotCameraArmController started")

    def get_arm1_ee_pose(self):
        """
        base_footprint から見た Arm1 EE の PoseStamped を取得
        """
        try:
            self.listener.waitForTransform(
                self.base_frame,
                self.arm1_ee_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )

            trans, rot = self.listener.lookupTransform(
                self.base_frame,
                self.arm1_ee_frame,
                rospy.Time(0)
            )

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.base_frame

            msg.pose.position.x = trans[0]
            msg.pose.position.y = trans[1]
            msg.pose.position.z = trans[2]

            msg.pose.orientation.x = rot[0]
            msg.pose.orientation.y = rot[1]
            msg.pose.orientation.z = rot[2]
            msg.pose.orientation.w = rot[3]

            return msg

        except tf.Exception as e:
            rospy.logwarn("Failed to get Arm1 EE TF: %s", str(e))
            return None

    def callback_place_command(self, data):
        command = data.data.strip()

        if command != "Place":
            return

        rospy.loginfo("Received Place command")

        ee_pose = self.get_arm1_ee_pose()

        if ee_pose is None:
            rospy.logwarn("Place command received, but Arm1 EE pose could not be obtained")
            return

        self.place_ee_pose = ee_pose
        self.place_ee_pub.publish(ee_pose)

        rospy.loginfo(
            "Recorded Arm1 EE at Place: x=%.3f, y=%.3f, z=%.3f",
            ee_pose.pose.position.x,
            ee_pose.pose.position.y,
            ee_pose.pose.position.z
        )

    def callback_hold_command(self, data):
        command = data.data.strip()

        if command != "Hold":
            return

        # Holdを受け取った時刻を更新
        self.last_hold_time = rospy.Time.now()
        self.hold_active = True

    def update_hold_state(self):
        """
        Holdコマンドが一定時間来なければHold解除
        """
        now = rospy.Time.now()

        if self.hold_active:
            if now - self.last_hold_time > self.hold_timeout:
                self.hold_active = False
                rospy.loginfo("Hold released")

    def calc_arm2_look_at_joints(self, target_pose):
        """
        Arm1 EEを見るためのArm2関節角を計算する場所。

        今は仮で固定姿勢を返す。
        あとでここに Look-at IK / DLS IK を入れる。
        """
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_z = target_pose.pose.position.z

        rospy.loginfo_throttle(
            1.0,
            "Hold target Arm1 EE: x=%.3f, y=%.3f, z=%.3f",
            target_x,
            target_y,
            target_z
        )

        # TODO:
        # ここで Arm2カメラが target_pose を見るようにIKを解く
        # 現時点では固定姿勢
        return [self.q1, self.q2, self.q3, self.q4, self.q5]

    def publish_arm2_joints(self, joint_angles):
        arm_2_cmd = make_arm_msg(joint_angles, joint_uri_2)
        self.arm_2_command_publisher.publish(arm_2_cmd)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.update_hold_state()

            if self.hold_active:
                # Hold中は現在のArm1 EEを見る
                ee_pose = self.get_arm1_ee_pose()

                if ee_pose is not None:
                    self.hold_ee_pub.publish(ee_pose)

                    cand = self.calc_arm2_look_at_joints(ee_pose)
                    self.publish_arm2_joints(cand)

            else:
                # Holdしていないときは固定姿勢
                cand = [self.q1, self.q2, self.q3, self.q4, self.q5]
                self.publish_arm2_joints(cand)

            rate.sleep()


if __name__ == '__main__':
    controller = YoubotCameraArmController()
    controller.run()