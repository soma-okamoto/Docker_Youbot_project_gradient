#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
youbot_real_trajectory_node_full_reconfig.py

機能:
- ユーザー手 (palm_pose) に追従（主タスク：DLS IK）
- ボトルのプレグラスプ姿勢を常時生成し、距離に応じて手→プレグラスプへ滑らかに吸着
- ヌル空間で 可操作性↑ + 関節リミット余裕↑ + プレグラスプ関節θ_preへの弱い引き戻し
- 実機角マッピング（元コード踏襲）して Publish

"""

import copy
import math
import numpy as np
import rospy
import tf

from brics_actuator.msg import JointPositions, JointValue
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray,String

# ================== 設定 / 定数 ==================
arm_1_topic_name  = "arm_1/arm_controller/position_command"
arm_1_msg_type    = JointPositions
joint_uri_1       = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5',
                     'gripper_finger_joint_l','gripper_finger_joint_r']

# 3リンク等価長 [m]（ROS paramで上書き可）
DEFAULT_L = [0.155, 0.135, 0.218]

# 把持向け yaw ロック（簡易）
YAW_LOCK_DEG = 90.0

# IK/到達性関連
SIGMA_MIN_THR = 0.03   # 最小特異値閾値
COND_MAX_THR  = 60.0   # 条件数閾値
ERR_THRESH    = 0.01   # 収束誤差閾値 (r,z,phiの二乗和)

# ヌル空間関連
W_MANI  = 1.0          # 可操作性重み
W_LIMIT = 0.3          # 関節余裕重み
K_NULL  = 0.02         # ヌル空間ステップ

# ソフト関節リミット（等価3リンク）: 環境に合わせて更新推奨
JOINT_LIMITS = [
    (-math.radians(120),  math.radians(120)),    # th1
    (-math.radians(146),  math.radians(146)),    # th2
    (-math.radians(102.5),math.radians(102.5)),  # th3
]

# ブレンドと吸着挙動
PREGRASP_OFFSET = 0.06   # r方向の手前オフセット[m]
D_IN   = 0.18            # 吸着開始距離（以下で吸着）
D_OUT  = 0.22            # 吸着解除距離（以上で解除）
SIG_K  = 40.0            # シグモイド鋭さ
BETA_SMOOTH = 0.2        # βの滑らか化係数




def joint_margins(th, limits):
    m = []
    for i,(lo,hi) in enumerate(limits):
        m.append(min(th[i]-lo, hi-th[i]))
    return m  # [m0,m1,m2]

def e_task_norm(th, target, L):
    cur = fk(L, th[0], th[1], th[2])
    dr = target[0]-cur[0]
    dz = target[1]-cur[1]
    dphi = angle_wrap(target[2]-cur[2])
    return math.sqrt(dr*dr + dz*dz + dphi*dphi)

def cond_and_sigma(J):
    U,S,Vt = np.linalg.svd(J, full_matrices=False)
    smin = float(S.min())
    cond = float(S.max()/S.min()) if S.min() > 1e-8 else float('inf')
    return cond, smin


# ================== ユーティリティ ==================
def DegToRad(th): return (np.pi/180.0)*th
def RadToDeg(th): return (180.0/np.pi)*th

def angle_wrap(a):
    # [-pi, pi) 正規化
    return (a + math.pi) % (2*math.pi) - math.pi

def Theta0(x,y):
    if x == 0 and y == 0:
        return 0.0
    return math.atan2(y,x)

def make_arm_msg(arm_js, joint_uri):
    """
    arm_js: [A1, A2, A3, A4, A5] (rad)
    """
    A1, A2, A3, A4, A5 = arm_js[0], arm_js[1], arm_js[2], arm_js[3], arm_js[4]
    jp = JointPositions()

    def jv(uri, val):
        j = JointValue()
        j.joint_uri = uri
        j.unit = 'rad'
        j.value = float(val)
        return j

    jp.positions.append(copy.deepcopy(jv(joint_uri[0], A1)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[1], A2)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[2], A3)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[3], A4)))
    jp.positions.append(copy.deepcopy(jv(joint_uri[4], A5)))
    return jp

# ---- 運動学（等価3リンク: (r,z,phi) に対応） ----
def fk(L, th0, th1, th2):
    # 元コードの(x,y,theta)を r=z軸平面用として流用
    # x→r, y→z
    r = L[0]*math.sin(th0) + L[1]*math.sin(th0+th1) + L[2]*math.sin(th0+th1+th2)
    z = L[0]*math.cos(th0) + L[1]*math.cos(th0+th1) + L[2]*math.cos(th0+th1+th2)
    phi = th0 + th1 + th2
    return [r, z, phi]

def jacobian(L, th0, th1, th2):
    r11 = L[0]*math.cos(th0)+L[1]*math.cos(th0+th1)+L[2]*math.cos(th0+th1+th2)
    r12 = L[1]*math.cos(th0+th1)+L[2]*math.cos(th0+th1+th2)
    r13 = L[2]*math.cos(th0+th1+th2)

    r21 = -L[0]*math.sin(th0)-L[1]*math.sin(th0+th1)-L[2]*math.sin(th0+th1+th2)
    r22 = -L[1]*math.sin(th0+th1)-L[2]*math.sin(th0+th1+th2)
    r23 = -L[2]*math.sin(th0+th1+th2)

    r31 = 1; r32 = 1; r33 = 1
    J = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]], dtype=np.float64)
    return J

def velocity(pd, p):
    # 簡易P制御: (r,z,phi)
    k = 0.1
    vr = k * (pd[0] - p[0])
    vz = k * (pd[1] - p[1])
    w  = k * angle_wrap(pd[2] - p[2])
    return np.array([[vr],[vz],[w]], dtype=np.float64)

def inversekinematics(L, pd, p0, lam):
    # DLS一歩分（増分Δθ）
    p_fk = fk(L, p0[0], p0[1], p0[2])
    v = velocity(pd, p_fk)
    J = jacobian(L, p0[0], p0[1], p0[2])
    I = np.identity(3)
    JT = J.T
    dtheta = (np.linalg.pinv(JT @ J + lam*I) @ JT) @ v
    # ステップスケール
    return 0.1 * dtheta

def dls_step(L, target, th, lam=1.0):
    cur = fk(L, th[0], th[1], th[2])
    err = np.array([[target[0]-cur[0]], [target[1]-cur[1]], [angle_wrap(target[2]-cur[2])]], dtype=np.float64)
    J = jacobian(L, th[0], th[1], th[2])
    JJt = J @ J.T
    J_pinv = J.T @ np.linalg.inv(JJt + lam*np.eye(3))
    dth = (J_pinv @ err).reshape(3)
    return dth, err, J

def ik_converges(L, target, th0, max_iters=200):
    th = th0.copy()
    lam = 1.0
    J = jacobian(L, th[0], th[1], th[2])
    err = np.zeros((3,1))
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

# ===== プレグラスプ生成 & ブレンド =====
def make_pregrasp_from_bottle(bxyz, approach_yaw_deg=YAW_LOCK_DEG, offset=PREGRASP_OFFSET):
    """bxyz=[x,y,z]。r方向に offset 手前で yaw固定"""
    bx, by, bz = bxyz
    r = math.sqrt(bx*bx + by*by)
    yaw = DegToRad(approach_yaw_deg)
    r_pre = max(r - offset, 0.02)
    return [r_pre, bz, yaw]

_beta = 0.0
_near = False
def blend_weight(distance, d_in=D_IN, d_out=D_OUT, k=SIG_K, alpha=BETA_SMOOTH):
    """ヒステリシス付きスムーズβ"""
    global _beta, _near
    if _near:
        if distance >= d_out: _near = False
    else:
        if distance <= d_in: _near = True
    d0 = d_in if _near else d_out
    b_inst = 1.0 / (1.0 + math.exp(k*(distance - d0)))
    _beta = (1-alpha)*_beta + alpha*b_inst
    return _beta

def merit_pregrasp_aug(th, L, th_pre=None):
    base = merit_function(th, L)
    extra = 0.0
    if th_pre is not None:
        extra += 0.3 * (- np.sum((th - th_pre)**2))  # 弱い引き戻し
    return base + extra

# ================== 購読コールバック ==================
palm_pose   = PoseStamped()
bottle_xyz  = np.zeros(3, dtype=np.float64)

def palm_cb(msg):
    global palm_pose
    palm_pose = msg

def bottle_cb(msg):
    """Float32MultiArray: data=[x,y,z,...] を想定"""
    global bottle_xyz
    if len(msg.data) >= 3:
        bottle_xyz[0] = float(msg.data[0])
        bottle_xyz[1] = float(msg.data[1])
        bottle_xyz[2] = float(msg.data[2])

# ================== メイン ==================
if __name__ == '__main__':
    rospy.init_node('youbot_trajectory_publisher')

    # パラメータ
    L = [
        rospy.get_param("~L1", DEFAULT_L[0]),
        rospy.get_param("~L2", DEFAULT_L[1]),
        rospy.get_param("~L3", DEFAULT_L[2]),
    ]
    p0 = [
        rospy.get_param("~th1_init", 0.1),
        rospy.get_param("~th2_init", 0.2),
        rospy.get_param("~th3_init", 0.3),
    ]
    theta0, theta1, theta2 = p0
    alpha  = rospy.get_param("~alpha", 0.05)   # 主タスクステップ
    k_null = rospy.get_param("~k_null", K_NULL)

    # 購読
    rospy.Subscriber("palm_pose", PoseStamped, palm_cb)
    rospy.Subscriber("/identified_bottle_pose", Float32MultiArray, bottle_cb)

    # 出力
    arm_cmd_pub   = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size=5)
    joint_pub_deg = rospy.Publisher("/arm_joint_command", Float32MultiArray, queue_size=10)

    metrics_pub = rospy.Publisher("/ybt_metrics", Float32MultiArray, queue_size=20)
    names_pub   = rospy.Publisher("/ybt_metric_names", String, queue_size=1, latch=True)
    names_pub.publish(String(data="t,dist,beta,e_task,sigma_min,cond,mani,margin0,margin1,margin2,H,dH_null,Jdnull_norm,dnull_norm,de_due_null"))
    t0 = rospy.get_time()


    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # ---- 手ポーズ取得 ----
        if (palm_pose.pose.position.x == 0 and
            palm_pose.pose.position.y == 0 and
            palm_pose.pose.position.z == 0):
            pose_holo = [0.0, 0.0, 0.2]  # フォールバック
        else:
            pose_holo = [palm_pose.pose.position.x,
                         palm_pose.pose.position.y,
                         palm_pose.pose.position.z]

        # 手の(r,z,phi)
        r_palm = math.sqrt(pose_holo[0]**2 + pose_holo[1]**2)
        z_palm = pose_holo[2]
        phi_palm = DegToRad(YAW_LOCK_DEG)
        palm_rzp = [r_palm, z_palm, phi_palm]

        # ---- プレグラスプ生成＆ブレンド ----
        if np.linalg.norm(bottle_xyz) > 1e-6:
            pre_rzp = make_pregrasp_from_bottle(bottle_xyz)
            # 平面距離（手→ボトル）
            d = math.sqrt((pose_holo[0]-bottle_xyz[0])**2 + (pose_holo[1]-bottle_xyz[1])**2)
            beta = blend_weight(d)
        else:
            pre_rzp = palm_rzp
            beta = 0.0

        target_rzp = [
            (1-beta)*palm_rzp[0] + beta*pre_rzp[0],
            (1-beta)*palm_rzp[1] + beta*pre_rzp[1],
            (1-beta)*palm_rzp[2] + beta*pre_rzp[2],
        ]

        # ---- プレグラスプ関節 θ_pre を計算（軽量IK） ----
        th_pre = None
        if np.linalg.norm(bottle_xyz) > 1e-6:
            ok_pg, _, _, _, th_try = ik_converges(L, pre_rzp, np.array([theta0,theta1,theta2], dtype=np.float64))
            if ok_pg:
                th_pre = th_try

        # ---- 主タスク: DLS-IK(適応ラムダ) ----
        lam = 1.0
        for _ in range(150):  # 早めに終了
            cur = fk(L, theta0, theta1, theta2)
            err = np.array([[target_rzp[0]-cur[0]], [target_rzp[1]-cur[1]]], dtype=np.float64)
            e = float(err.T @ err)
            lam = e + 0.002
            dth = inversekinematics(L, target_rzp, [theta0,theta1,theta2], lam)
            theta0 += float(alpha * dth[0,0])
            theta1 += float(alpha * dth[1,0])
            theta2 += float(alpha * dth[2,0])
            if e < 1e-6:
                break

 # ---- ヌル空間: 可操作性 + リミット余裕 + θ_pre引き戻し ----
        # === 評価(前) ===
        th_vec_before = np.array([theta0, theta1, theta2], dtype=np.float64)
        e_before = e_task_norm(th_vec_before, target_rzp, L)

        J_now = jacobian(L, theta0, theta1, theta2)
        cond_now, sigma_min_now = cond_and_sigma(J_now)

        mani_now = manipulability(J_now)
        m0, m1, m2 = joint_margins(th_vec_before, JOINT_LIMITS)
        H_before = merit_pregrasp_aug(th_vec_before, L, th_pre)

        # === ヌル空間更新（既存処理） ===
        P = nullspace_projector(J_now)
        gradH = finite_diff_grad(lambda th, L_: merit_pregrasp_aug(th, L_, th_pre), th_vec_before, L)
        dth_null = k_null * (P @ gradH)
        theta0 += float(dth_null[0]); theta1 += float(dth_null[1]); theta2 += float(dth_null[2])

        # ---- ソフト関節リミットで安全クリップ ----
        theta0 = float(np.clip(theta0, JOINT_LIMITS[0][0], JOINT_LIMITS[0][1]))
        theta1 = float(np.clip(theta1, JOINT_LIMITS[1][0], JOINT_LIMITS[1][1]))
        theta2 = float(np.clip(theta2, JOINT_LIMITS[2][0], JOINT_LIMITS[2][1]))

        # === 評価(後) & Publish ===
        th_vec_after = np.array([theta0, theta1, theta2], dtype=np.float64)
        H_after = merit_pregrasp_aug(th_vec_after, L, th_pre)
        dH_null = H_after - H_before

        e_after = e_task_norm(th_vec_after, target_rzp, L)
        de_due_null = e_after - e_before

        # 主タスク影響度 ||J dθ_null||
        Jdnull = J_now @ dth_null.reshape(3,1)
        Jdnull_norm = float(np.linalg.norm(Jdnull))
        dnull_norm  = float(np.linalg.norm(dth_null))

        # 距離を再計算（ボトルが無いときは -1）
        if np.linalg.norm(bottle_xyz) > 1e-6:
            dist = math.hypot(pose_holo[0]-bottle_xyz[0], pose_holo[1]-bottle_xyz[1])
        else:
            dist = -1.0

        t = rospy.get_time() - t0
        metrics = [
            t, dist, float(beta), float(e_after),
            float(sigma_min_now), float(cond_now),
            float(mani_now), float(m0), float(m1), float(m2),
            float(H_after), float(dH_null),
            float(Jdnull_norm), float(dnull_norm), float(de_due_null)
        ]
        metrics_pub.publish(Float32MultiArray(data=metrics))


        # ---- ベース方位 & 実機角マッピング（元コード踏襲） ----
        theta_base = DegToRad(169/2) - Theta0(pose_holo[1], pose_holo[0])
        theta_base = max(min(theta_base, DegToRad(110)), DegToRad(15))

        theta_vec = [theta0, theta1, theta2]
        cand = [
            theta_base,
            DegToRad(65)    - theta_vec[0],
            -DegToRad(146)  - theta_vec[1],
            DegToRad(102.5) - theta_vec[2],
            DegToRad(167.5)
        ]
        cand_deg = [RadToDeg(c) for c in cand]  # 可視化/下流向けは実機角(deg)で統一

        # ---- Publish ----
        joint_pub_deg.publish(Float32MultiArray(data=cand_deg))
        arm_cmd_pub.publish(make_arm_msg(cand, joint_uri_1))

        # ---- 任意ログ（重ければコメントアウト）----
        # rospy.loginfo_throttle(1.0, f"beta={beta:.2f} θ=[{theta0:.2f},{theta1:.2f},{theta2:.2f}] cand_deg={cand_deg}")

        rate.sleep()
