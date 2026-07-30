#!/usr/bin/env python3

import copy
import rospy
import math
import numpy as np
import tf

from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String,Float32MultiArray
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


def RadToDeg(rad):
    return rad * 180.0 / np.pi


def normalize_0_to_2pi(angle):
    """
    angle を 0 ～ 2pi に正規化
    """
    angle = math.fmod(angle, 2.0 * math.pi)

    if angle < 0.0:
        angle += 2.0 * math.pi

    return angle


def shortest_angle_diff(target, current):
    """
    current から target への最短角度差を -pi ～ pi で返す
    """
    diff = target - current
    return math.atan2(math.sin(diff), math.cos(diff))



#############################カメラ追従クラス##########################
class YoubotCameraArmController:
    def __init__(self):
        rospy.init_node('youbot_camera_trajectory_publisher')

        # =========================
        # Arm1 simple capsule occupancy
        # =========================
        self.arm1_capsule_radius = 0.075
        self.safety_margin = 0.03

        self.arm1_occupancy_frames = [
            "arm_link_1",
            "arm_link_2",
            "arm_link_3",
            "arm_link_4",
            "arm_link_5",
            "gripper_tip"
        ]

        # =========================
        # TF settings
        # =========================
        self.listener = tf.TransformListener()

        self.base_frame = "base_footprint"

        # Arm1側はTipでOK
        self.arm1_ee_frame = "gripper_tip"

        # Arm2カメラ位置
        # optical_frameだと軸がややこしいが、位置として使うならOK
        self.arm2_camera_frame = "asus_rgb_optical_frame"

        # =========================
        # Hold settings
        # =========================
        self.hold_active = False
        self.last_hold_time = rospy.Time(0)
        self.hold_timeout = rospy.Duration(1.0)

        self.place_ee_pose = None

        # P_current保存用
        self.p_current = np.zeros(3, dtype=np.float64)
        self.p_current_received = False



        self.arm_2_command_publisher = rospy.Publisher(
            arm_2_topic_name,
            arm_2_msg_type,
            queue_size=5
        )


        #######P_TF用

        self.place_ee_pub = rospy.Publisher(
            "/P_tf",
            PoseStamped,
            queue_size=10
        )

        # =========================
        # Subscriber
        # =========================
        rospy.Subscriber('/P_current', Float32MultiArray, self.callback_p_current)

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

        # =========================
        # q1 horizontal look-at settings
        # =========================

        # q1_target = -yaw + 180deg
        self.q1_yaw_sign = -1.0
        self.q1_yaw_offset = DegToRad(180)

        self.q1_min = DegToRad(10)
        self.q1_max = DegToRad(330)

        # 1ループあたりの最大変化量
        # 10Hzなら 3deg/loop = 約30deg/s
        self.q1_step_limit = DegToRad(3)
        self.last_q1_cmd = self.q1

        # =========================
        # q4 vertical look-at settings
        # =========================
        # q4で上下方向を見る
        # 逆に動いたら -1.0 にする
        self.q4_pitch_sign = 1.0

        # q4の基準角
        self.q4_pitch_offset = DegToRad(0)

        # q4の制限範囲
        self.q4_min = DegToRad(-10)
        self.q4_max = DegToRad(110)

        # 1ループあたりの最大変化量
        self.q4_step_limit = DegToRad(3)
        self.last_q4_cmd = self.q4

        # =========================
        # q3 height adjustment settings
        # =========================
        # target_z - camera_z が正のとき、q3をどちらへ動かすか
        # 逆なら -1.0 にする
        self.q3_height_sign = 1.0

        # 高さ誤差[m] → 関節角[rad] の変換ゲイン
        # まずは小さめ
        self.q3_height_gain = 1

        # q3の制限範囲
        # 現在 q3 は -251deg 付近
        # self.q3_min = DegToRad(-290)
        self.q3_min = DegToRad(-340)
        # self.q3_max = DegToRad(-190)
        self.q3_max = DegToRad(-100)

        # 1ループあたりの最大変化量
        self.q3_step_limit = DegToRad(2)
        self.last_q3_cmd = self.q3

        # =========================
        # q4 level compensation settings
        # =========================

        # この角度以内なら「水平気味に見る」モード
        # これを超えたら「q4で対象を見る」モード
        self.pitch_level_limit = DegToRad(20)

        # q3で動かした分をq4でどれくらい打ち消すか
        self.q4_level_gain = 0.7

        # q4で対象を見るときのゲイン
        self.q4_pitch_gain = 0.7


        rospy.loginfo("YoubotCameraArmController started")


    def callback_hold_command(self, data):
        command = data.data.strip()

        if command != "Hold":
            return

        self.last_hold_time = rospy.Time.now()
        self.hold_active = True

    def callback_p_current(self, msg):
        if len(msg.data) < 4:
            rospy.logwarn_throttle(
                1.0,
                "/P_current data is too short: len=%d",
                len(msg.data)
            )
            return

        self.p_current[0] = float(msg.data[1])
        self.p_current[1] = float(msg.data[2])
        self.p_current[2] = float(msg.data[3])
        self.p_current_received = True



    
#######################################################################
############################メイン処理##################################
    def update_hold_state(self):
        """
        Holdコマンドが一定時間来なければHold解除
        """
        now = rospy.Time.now()

        if self.hold_active:
            if now - self.last_hold_time > self.hold_timeout:
                self.hold_active = False



                rospy.loginfo("Hold released")



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
        

    def calc_arm2_look_at_joints(self, target_pose):
        """
        Arm1 EEを見るためのArm2関節角を計算する。
        q1: 左右方向
        q4: 上下方向
        q2, q3, q5: 固定
        """
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_z = target_pose.pose.position.z

        target_pos = np.array([target_x, target_y, target_z], dtype=np.float64)

        camera_pos = self.get_arm2_camera_position()

        if camera_pos is None:
            return [self.q1, self.q2, self.q3, self.q4, self.q5]

        vec = target_pos - camera_pos

        dist_to_target = np.linalg.norm(vec)
        xy_dist = math.sqrt(vec[0] ** 2 + vec[1] ** 2)

        clearance = self.distance_to_arm1_capsules(camera_pos)

        # =========================
        # q1: horizontal look-at
        # =========================

        yaw = math.atan2(vec[1], vec[0])

        q1_target = self.q1_yaw_sign * yaw + self.q1_yaw_offset
        q1_target = normalize_0_to_2pi(q1_target)
        q1_target = np.clip(q1_target, self.q1_min, self.q1_max)

        q1_diff = shortest_angle_diff(q1_target, self.last_q1_cmd)
        q1_diff = np.clip(
            q1_diff,
            -self.q1_step_limit,
            self.q1_step_limit
        )

        q1_new = self.last_q1_cmd + q1_diff
        q1_new = normalize_0_to_2pi(q1_new)
        q1_new = np.clip(q1_new, self.q1_min, self.q1_max)

        self.last_q1_cmd = q1_new

        # # # =========================
        # # # q4: vertical look-at
        # # # =========================

        # # target がカメラより上なら pitch は正
        # pitch = math.atan2(vec[2], xy_dist)

        # # q4を上下方向に動かす
        # q4_target = self.q4 + self.q4_pitch_sign * pitch + self.q4_pitch_offset

        # q4_target = np.clip(q4_target, self.q4_min, self.q4_max)

        # q4_diff = q4_target - self.last_q4_cmd
        # q4_diff = np.clip(
        #     q4_diff,
        #     -self.q4_step_limit,
        #     self.q4_step_limit
        # )

        # q4_new = self.last_q4_cmd + q4_diff
        # q4_new = np.clip(q4_new, self.q4_min, self.q4_max)

        # self.last_q4_cmd = q4_new

        # rospy.loginfo_throttle(
        #     1.0,
        #     "Look-at: yaw=%.3f, pitch=%.3f, q1=%.1f deg, q4=%.1f deg, q1_target=%.1f deg, q4_target=%.1f deg",
        #     yaw,
        #     pitch,
        #     RadToDeg(q1_new),
        #     RadToDeg(q4_new),
        #     RadToDeg(q1_target),
        #     RadToDeg(q4_target)
        # )

        # return [q1_new, self.q2, self.q3, q4_new, self.q5]
    
        # =========================
        # q3 + q4 adaptive vertical control
        #
        # pitch が小さい:
        #   q3で高さを合わせて、q4で水平気味に保つ
        #
        # pitch が大きい:
        #   水平維持をやめて、q4で対象を見る
        # =========================

        pitch = math.atan2(vec[2], xy_dist)
        height_error = target_pos[2] - camera_pos[2]

        # -------------------------
        # q3: height adjustment
        # -------------------------
        q3_target = self.q3 + self.q3_height_sign * self.q3_height_gain * height_error
        q3_target = np.clip(q3_target, self.q3_min, self.q3_max)

        q3_diff = q3_target - self.last_q3_cmd
        q3_diff = np.clip(
            q3_diff,
            -self.q3_step_limit,
            self.q3_step_limit
        )

        q3_new = self.last_q3_cmd + q3_diff
        q3_new = np.clip(q3_new, self.q3_min, self.q3_max)

        self.last_q3_cmd = q3_new

        # q3が初期値からどれだけ動いたか
        q3_delta = q3_new - self.q3

        # -------------------------
        # mode switch
        # -------------------------
        if abs(pitch) < self.pitch_level_limit:
            # =========================
            # Mode A:
            # ほぼ横から見られるとき
            # q3で高さ調整し、q4で水平気味に補正
            # =========================

            q4_target = self.q4 - self.q4_level_gain * q3_delta

            mode_name = "LEVEL_SIDE_VIEW"

        else:
            # =========================
            # Mode B:
            # 上下差が大きく、水平では見えないとき
            # 水平維持をやめて、q4で対象を見る
            # =========================

            q4_target = (
                self.q4
                + self.q4_pitch_sign * self.q4_pitch_gain * pitch
                + self.q4_pitch_offset
            )

            mode_name = "Q4_LOOK_AT"

        q4_target = np.clip(q4_target, self.q4_min, self.q4_max)

        q4_diff = q4_target - self.last_q4_cmd
        q4_diff = np.clip(
            q4_diff,
            -self.q4_step_limit,
            self.q4_step_limit
        )

        q4_new = self.last_q4_cmd + q4_diff
        q4_new = np.clip(q4_new, self.q4_min, self.q4_max)

        self.last_q4_cmd = q4_new

        return [q1_new, self.q2, q3_new, q4_new, self.q5]



    def get_arm2_camera_position(self):
        """
        Arm2カメラ位置を取得
        """
        return self.get_tf_position(self.arm2_camera_frame)

        
    def get_tf_position(self, frame_name):
        """
        base_footprint から見た任意TFフレームの位置を取得
        """
        try:
            self.listener.waitForTransform(
                self.base_frame,
                frame_name,
                rospy.Time(0),
                rospy.Duration(0.5)
            )

            trans, rot = self.listener.lookupTransform(
                self.base_frame,
                frame_name,
                rospy.Time(0)
            )

            return np.array([trans[0], trans[1], trans[2]], dtype=np.float64)

        except tf.Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "Failed to get TF position: %s -> %s : %s",
                self.base_frame,
                frame_name,
                str(e)
            )
            return None
        

########################################################################
#######################簡易カプセル占有モデル関連#########################
    def distance_to_arm1_capsules(self, point):
        """
        任意点とArm1簡易カプセル群との最小距離を計算する。
        """
        capsules = self.make_arm1_capsules()

        if capsules is None:
            return None

        min_clearance = float("inf")

        for p0, p1, radius in capsules:
            d_centerline = self.point_to_segment_distance(point, p0, p1)
            clearance = d_centerline - radius

            if clearance < min_clearance:
                min_clearance = clearance

        return min_clearance
    
    def make_arm1_capsules(self):
        """
        Arm1の簡易カプセル占有モデルを作成する。
        """
        points = self.get_arm1_link_points()

        if points is None:
            return None

        capsules = []

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            radius = self.arm1_capsule_radius + self.safety_margin
            capsules.append((p0, p1, radius))

        return capsules  

    def get_arm1_link_points(self):
        """
        Arm1の各リンク代表点をbase_footprint基準で取得
        """
        points = []

        for frame in self.arm1_occupancy_frames:
            p = self.get_tf_position(frame)

            if p is None:
                return None

            points.append(p)

        return points

    def point_to_segment_distance(self, p, a, b):
        """
        点pと線分abの最短距離を計算
        """
        ab = b - a
        ap = p - a

        denom = np.dot(ab, ab)

        if denom < 1e-9:
            return np.linalg.norm(p - a)

        t = np.dot(ap, ab) / denom
        t = np.clip(t, 0.0, 1.0)

        closest = a + t * ab

        return np.linalg.norm(p - closest)

##########################################################################
##############################Publish部分#######################################

    def publish_arm2_joints(self, joint_angles):
        arm_2_cmd = make_arm_msg(joint_angles, joint_uri_2)
        self.arm_2_command_publisher.publish(arm_2_cmd)
    

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
        
        sa = np.zeros(3, dtype=np.float64)

        sa[0] = self.p_current[0] - float(ee_pose.pose.position.x)
        sa[1] = self.p_current[1] - float(ee_pose.pose.position.y)
        sa[2] = self.p_current[2] - float(ee_pose.pose.position.z)

        rospy.loginfo(
            "P_current - EE: dx=%.3f, dy=%.3f, dz=%.3f",
            sa[0],
            sa[1],
            sa[2]
        )

        # rospy.loginfo(
        #     "Recorded Arm1 EE at Place: x=%.3f, y=%.3f, z=%.3f",
        #     ee_pose.pose.position.x,
        #     ee_pose.pose.position.y,
        #     ee_pose.pose.position.z
        # )

###############################メインループ#######################################

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.update_hold_state()

            if self.hold_active:
                ee_pose = self.get_arm1_ee_pose()

                if ee_pose is not None:
                    # self.hold_ee_pub.publish(ee_pose)

                    cand = self.calc_arm2_look_at_joints(ee_pose)
                    self.publish_arm2_joints(cand)

            # else:
                # cand = [self.q1, self.q2, self.q3, self.q4, self.q5]

                # self.last_q1_cmd = self.q1
                # self.last_q3_cmd = self.q3
                # self.last_q4_cmd = self.q4

                # self.publish_arm2_joints(cand)
                # print("test")

            rate.sleep()
    

if __name__ == '__main__':
    controller = YoubotCameraArmController()
    controller.run()