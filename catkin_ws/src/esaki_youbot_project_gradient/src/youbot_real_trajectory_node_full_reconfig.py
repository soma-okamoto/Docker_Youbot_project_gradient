#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
youbot_real_trajectory_node_full_reconfig.py

ベース: 元スクリプト構成（publish先/メッセージ/関数名など）を維持しつつ、
以下を追加：
- /identified_bottle_pose (PoseStamped) を購読
- EE(ユーザー手)追従を主タスクに保持（/palm_pose）
- ボトル座標に対する到達性/可操作性の判定（SVD: 最小特異値・条件数）
- 悪条件なら nullspace でジョイント再構成（可操作性 + 関節限界余裕のメリット関数）
- 3リンク (r,z,phi) IK は元コード相当の DLS を流用

依存: rospy, numpy, geometry_msgs, std_msgs, brics_actuator（実機出力するなら）

注意: 実機のリンク長/角度マッピングは環境に合わせて調整してください。
"""

import sys
import copy
import rospy
import math
import numpy as np
import time

from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import tf

# ======== 元コードのトピック/定数を継承 ========
arm_1_topic_name = "arm_1/arm_controller/position_command"
arm_1_msg_type = JointPositions
grip_1_topic_name = "arm_1/gripper_controller/position_command"
grip_1_msg_type = JointPositions


joint_uri_1 = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5','gripper_finger_joint_l','gripper_finger_joint_r']

# ======== 追加パラメータ ========
# 3リンク等価長 [m]（必要に応じて rosparam で上書き）
DEFAULT_L = [0.155, 0.135, 0.218]

# EE 追従で使う目標 phi（用途で置換OK。簡易には 90deg ロック）
YAW_LOCK_DEG = 90.0

# reachability / manipulability 判定閾値
SIGMA_MIN_THR  = 0.03   # 最小特異値がこれ未満は悪条件
COND_MAX_THR   = 60.0   # 条件数がこれ以上は悪条件
MAX_IK_ITERS   = 200
ERR_THRESH     = 0.01   # IK 誤差閾値

# nullspace 再構成の重み
W_MANI   = 1.0    # 可操作性 (log det(JJ^T))
W_LIMIT  = 0.3    # 関節限界余裕
K_NULL   = 0.02   # nullspace ステップ係数

# 関節のソフトリミット（等価3リンク）: 環境に合わせて更新推奨
JOINT_LIMITS = [
    (-math.radians(120),  math.radians(120)),   # th1
    (-math.radians(146),  math.radians(146)),   # th2
    (-math.radians(102.5),math.radians(102.5)), # th3
]

# ======== 元コードの関数たち ========
def make_arm_msg(arm_js, joint_uri):
    A1, A2, A3, A4, A5 = arm_js[0], arm_js[1], arm_js[2], arm_js[3], arm_js[4]
    jp = JointPositions()
    def jv(uri, val):
        j = JointValue()
        j.joint_uri = uri; j.unit='rad'; j.value = float(val)
        return j
    jp.positions.append(copy.deepcopy(jv(joint_uri[0], A1)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[1], A2)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[2], A3)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[3], A4)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[4], A5)))
    return jp


def Theta0(x,y):
    if x == 0 and y == 0:
        theta_0 = 0
    theta_0 = math.atan2(y,x)
    return theta_0

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    return [e[0], e[1], e[2]]

def DegToRad(th): return (np.pi/180.0)*th
def RadToDeg(th): return (180.0/np.pi)*th

# ---- 運動学 (元コード準拠: fk は [x, y, theta] 返却だが、ここでは (r,z,phi) 用に使う) ----
def fk(L,theta0,theta1,theta2):
    x = L[0]*math.sin(theta0) + L[1]*math.sin(theta0+theta1) + L[2]*math.sin(theta0+theta1+theta2)
    y = L[0]*math.cos(theta0) + L[1]*math.cos(theta0+theta1) + L[2]*math.cos(theta0+theta1+theta2)
    theta = theta0 + theta1 + theta2
    # ここで x→r, y→z として扱う（元コードの座標系解釈に合わせる）
    return [x, y, theta]

def velocity(pd,p):
    k = 0.1
    vx = k * (pd[0] - p[0])
    vz = k * (pd[1] - p[1])
    omega = k * angle_wrap(pd[2] - p[2])
    v = np.array([[vx], [vz],[omega]], dtype=np.float64)
    return v

def jacobian(L,theta0,theta1,theta2):
    r11 = L[0]*math.cos(theta0)+L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r12 = L[1]*math.cos(theta0+theta1)+L[2]*math.cos(theta0+theta1+theta2)
    r13 = L[2]*math.cos(theta0+theta1+theta2)

    r21 = -L[0]*math.sin(theta0)-L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r22 = -L[1]*math.sin(theta0+theta1)-L[2]*math.sin(theta0+theta1+theta2)
    r23 = -L[2]*math.sin(theta0+theta1+theta2)

    r31 = 1; r32 = 1; r33 = 1

    J = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]], dtype=np.float64)
    return J

def inversekinematics(L,pd,p0,lam):
    p_fk = fk(L,p0[0],p0[1],p0[2])
    v = velocity(pd,p_fk)
    J = jacobian(L,p0[0],p0[1],p0[2])
    i = np.identity(3)
    J_T = np.transpose(J)
    SR = (np.dot(J_T,J) + lam*i)
    SR_1 = np.linalg.pinv(SR)
    J_SR = np.dot(SR_1,J_T)
    angular_velocity = np.dot(J_SR,v)
    theta = 0.1 * angular_velocity
    return theta

def angle_wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi

# ======== IKユーティリティ（到達性判定） ========
def dls_step(L, target, th, lam=1.0):
    cur = fk(L, th[0], th[1], th[2])
    err = np.array([[target[0]-cur[0]], [target[1]-cur[1]], [angle_wrap(target[2]-cur[2])]], dtype=np.float64)
    J = jacobian(L, th[0], th[1], th[2])
    JJt = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt + lam*np.eye(3))
    dth = (J_pinv @ err).reshape(3)
    return dth, err, J

def ik_converges(L, target, th0, max_iters=MAX_IK_ITERS):
    th = th0.copy()
    lam = 1.0
    J = jacobian(L, th[0],th[1],th[2])  # 予備
    for _ in range(max_iters):
        dth, err, J = dls_step(L, target, th, lam)
        e = float(err.T @ err)
        lam = e + 0.002
        th += dth
        if e < ERR_THRESH**2:
            U,S,Vt = np.linalg.svd(J, full_matrices=False)
            smin = S.min()
            cond = (S.max()/S.min()) if S.min()>1e-8 else float('inf')
            return True, e, smin, cond, th
    U,S,Vt = np.linalg.svd(J, full_matrices=False)
    smin = S.min()
    cond = (S.max()/S.min()) if S.min()>1e-8 else float('inf')
    return False, float(err.T @ err), smin, cond, th

# ======== 可操作性/関節限界メトリクス ========
def manipulability(J):
    JJt = J @ J.T
    det = np.linalg.det(JJt)
    if det <= 1e-12: return -1e6
    return math.log(det)

def limit_margin_cost(th, limits=JOINT_LIMITS, eps=1e-3):
    s = 0.0
    for i,(lo,hi) in enumerate(limits):
        m = min(th[i]-lo, hi-th[i])
        m = max(m, eps)
        s += math.log(m)
    return s

def merit_function(th, L):
    J = jacobian(L, th[0], th[1], th[2])
    return W_MANI * manipulability(J) + W_LIMIT * limit_margin_cost(th)

def finite_diff_grad(f, th, L, h=1e-3):
    g = np.zeros(3, dtype=np.float64)
    base = f(th, L)
    for i in range(3):
        th_p = th.copy(); th_p[i] += h
        g[i] = (f(th_p, L) - base)/h
    return g

def nullspace_projector(J):
    JJt = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt + 1e-6*np.eye(3))
    I = np.eye(3)
    return I - J_pinv @ J

# ======== サブスクライバ ========
palm_pose = PoseStamped()
bottle_xyz = np.zeros(3, dtype=np.float64)     # 追加: [x,y,z]

def palm_cb(msg):   # ユーザー手
    global palm_pose
    palm_pose = msg

def bottle_cb(msg):  # Float32MultiArray: [x, y, z, ...] を想定
    global bottle_xyz
    if len(msg.data) >= 3:
        bottle_xyz[0] = float(msg.data[0])
        bottle_xyz[1] = float(msg.data[1])
        bottle_xyz[2] = float(msg.data[2])


# ======== メイン ========
if __name__ == '__main__':
    rospy.init_node('youbot_trajectory_publisher')

    # リンク長（rosparamで上書き可）
    L = [
        rospy.get_param("~L1", DEFAULT_L[0]),
        rospy.get_param("~L2", DEFAULT_L[1]),
        rospy.get_param("~L3", DEFAULT_L[2]),
    ]

    # 初期角
    p0 = [
        rospy.get_param("~th1_init", 0.1),
        rospy.get_param("~th2_init", 0.2),
        rospy.get_param("~th3_init", 0.3),
    ]
    theta0 = p0[0]; theta1 = p0[1]; theta2 = p0[2]

    # 購読
    rospy.Subscriber("palm_pose", PoseStamped, palm_cb)
    rospy.Subscriber('/identified_bottle_pose', Float32MultiArray, bottle_cb)

    # pub
    arm_1_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)
    arm_1_joint_publisher   = rospy.Publisher('/arm_joint_command', Float32MultiArray, queue_size=10)


    rate = rospy.Rate(30)

    w = np.diag([0.1,0.1])  #（元コード継承・未使用）

    # nullspace のゲイン等
    alpha  = rospy.get_param("~alpha", 0.05)   # 主タスク（EE追従）のブレンド
    k_null = rospy.get_param("~k_null", K_NULL)

    while not rospy.is_shutdown():

        # === 1) 主タスク: ユーザー手 EE 追従 ===
        if (palm_pose.pose.position.x == 0 and palm_pose.pose.position.y == 0 and palm_pose.pose.position.z == 0):
            pose_holo = [0,0,0.2]
        else:
            pose_holo = [palm_pose.pose.position.x, palm_pose.pose.position.y, palm_pose.pose.position.z]

        r_palm = math.sqrt(pose_holo[0]**2 + pose_holo[1]**2)
        z_palm = pose_holo[2]
        phi_palm = DegToRad(YAW_LOCK_DEG)  # 必要なら手の姿勢から算出に変更可
        pose_3d = [r_palm, z_palm, phi_palm]

        ramda = 1 
        
        for i in range(1000):
            
            p_old = fk(L, theta0, theta1, theta2)   
            error = np.array([[pose_3d[0] - p_old[0]], [pose_3d[1] - p_old[1]]])
            error_T = np.transpose(error)

            squarederror = np.dot(error_T,w)
            e = np.dot(squarederror,error)
                
            ramda = e + 0.002

            theta_np = inversekinematics(L, pose_3d, [theta0,theta1,theta2], ramda)

            theta0 += alpha * float(theta_np[0,0])
            theta1 += alpha * float(theta_np[1,0])
            theta2 += alpha * float(theta_np[2,0])

        # === 2) Reachability / manipulability 判定（ボトル） ===
        if np.linalg.norm(bottle_xyz) > 1e-6:
            bx, by, bz = bottle_xyz.tolist()
            r_b = math.sqrt(bx*bx + by*by)
            z_b = bz
            phi_b = DegToRad(YAW_LOCK_DEG)
            target_bottle = [r_b, z_b, phi_b]
            print(target_bottle)

            ok, e_b, smin_b, cond_b, th_try = ik_converges(
                L, target_bottle, np.array([theta0,theta1,theta2], dtype=np.float64)
            )

            if (not ok) or (smin_b < SIGMA_MIN_THR) or (cond_b > COND_MAX_THR):
                J = jacobian(L, theta0, theta1, theta2)
                P = nullspace_projector(J)
                gradH = finite_diff_grad(merit_function, np.array([theta0,theta1,theta2], dtype=np.float64), L)
                dth_null = k_null * (P @ gradH)
                theta0 += float(dth_null[0]); theta1 += float(dth_null[1]); theta2 += float(dth_null[2])
                print("nullspace push:", dth_null)
        else:
            print("not bottle")

        # === 3) ベース方位 & 実機角マッピング（元コード踏襲） ===
        theta_base = DegToRad(169/2) - Theta0(pose_holo[1], pose_holo[0])
        theta_base = max(min(theta_base, DegToRad(110)), DegToRad(15))

        theta = [theta0, theta1, theta2]
        cand = [
            theta_base,
            DegToRad(65)   - theta[0],
            -DegToRad(146) - theta[1],
            DegToRad(102.5)- theta[2],
            DegToRad(167.5)
        ]
        cand_deg = [RadToDeg(theta_base), RadToDeg(theta[0]), RadToDeg(theta[1]), RadToDeg(theta[2]), RadToDeg(cand[4])]

        # === 4) Publish（元コード準拠） ===
        arm1_joint_pub = Float32MultiArray(data=cand_deg)
        arm_1_joint_publisher.publish(arm1_joint_pub)

        arm_1_cmd = make_arm_msg(cand, joint_uri_1)
        arm_1_command_publisher.publish(arm_1_cmd)


        rate.sleep()
