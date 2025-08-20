#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from tf.transformations import (
    quaternion_matrix, quaternion_conjugate, quaternion_multiply,
    quaternion_from_matrix
)

def normalize(v, eps=1e-9):
    n = np.linalg.norm(v)
    return v/(n+eps)

def quat_from_R(R):
    M = np.eye(4); M[:3,:3] = R
    return np.array(quaternion_from_matrix(M))

def R_from_quat(q):
    M = quaternion_matrix(q)
    return np.array(M[:3,:3])

def quat_err(qc, qd):
    # q_err = qd * qc^{-1}; small-angle approx uses imaginary part
    e = quaternion_multiply(qd, quaternion_conjugate(qc))
    return np.array(e[:3], dtype=float)

class QPPostureServo(object):
    """
    速度レベルのDLSサーボ:
      - 位置: /palm_pose（Float32MultiArray [x,y,z]）にリアルタイム追従（主タスク）
      - 姿勢: /identified_bottle_pose から合成（look-at / align）
      - ヌルスペース: Φの勾配（中央化 + 関節限界バリア + 可操作度FD勾配 + 前把持距離）
    """

    def __init__(self):
        # Links / joints / topics
        self.base = rospy.get_param("~base_link", "base_link")
        self.tip  = rospy.get_param("~tip_link",  "arm_1_gripper_tip")
        self.joint_names = rospy.get_param("~joint_names",
                          ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"])
        self.cmd_topic   = rospy.get_param("~command_topic", "/arm_1/arm_controller/command")
        self.rate_hz     = rospy.get_param("~rate_hz", 100.0)
        self.dt_cmd      = rospy.get_param("~dt_cmd", 0.10)

        # Gains
        self.k_pos   = rospy.get_param("~k_pos", 1.0)
        self.k_ori   = rospy.get_param("~k_ori", 1.0)
        self.damping = rospy.get_param("~damping", 0.03)
        self.null_k  = rospy.get_param("~null_k", 0.5)

        # Joint limits
        self.q_min = np.array(rospy.get_param("~q_min", []), dtype=float)
        self.q_max = np.array(rospy.get_param("~q_max", []), dtype=float)
        self.have_limits = self.q_min.size==len(self.joint_names) and self.q_max.size==len(self.joint_names)

        # Orientation policy wrt bottle
        self.ori_policy  = rospy.get_param("~ori_policy", "lookat_bottle")  # hold|fixed|lookat_bottle|align_bottle_axis
        self.fixed_quat  = np.array(rospy.get_param("~fixed_quat", [0,0,0,1]), dtype=float)
        self.look_axis   = rospy.get_param("~look_axis", "x")  # EE軸: x|y|z
        self.bottle_axis = rospy.get_param("~bottle_axis", "z")# ボトル軸: x|y|z
        self.min_tan     = rospy.get_param("~min_tangent", 1e-3)

        # Nullspace Φ weights
        self.w_center = rospy.get_param("~w_center", 1.0)
        self.w_limit  = rospy.get_param("~w_limit",  0.3)
        self.w_manip  = rospy.get_param("~w_manip",  0.4)
        self.w_pre    = rospy.get_param("~w_pre",    0.0)   # 前把持距離を使うなら >0 に
        self.pre_d    = rospy.get_param("~pregrasp_offset", 0.07)

        # KDL
        ok, tree = treeFromParam("/youbot_description")
        if not ok:
            raise RuntimeError("URDF not found at /youbot_description")
        if not tree.getChain(self.base, self.tip):
            raise RuntimeError("KDL chain build failed")
        self.chain = tree.getChain(self.base, self.tip)
        self.ndof  = self.chain.getNrOfJoints()
        self.fk    = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac   = kdl.ChainJntToJacSolver(self.chain)

        # State
        self.q = np.zeros(self.ndof)
        self.have_js = False
        self.ee_quat = np.array([0,0,0,1], dtype=float)
        self.palm = None
        self.bpos = None
        self.bquat = None

        # ROS I/O
        rospy.Subscriber("/joint_states", JointState, self.cb_js, queue_size=50)
        rospy.Subscriber("/palm_pose", Float32MultiArray, self.cb_palm, queue_size=50)
        rospy.Subscriber("/identified_bottle_pose", Float32MultiArray, self.cb_bottle, queue_size=20)
        self.pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=10)

        # manipulability FD cadence
        self._manip_grad_cache = np.zeros(self.ndof)
        self._fd_every = rospy.get_param("~manip_fd_period", 5)  # 何サイクルごとにFD再計算
        self._tick = 0

    # --- Callbacks ---
    def cb_js(self, m):
        name_to_idx = {n:i for i,n in enumerate(m.name)}
        qq = []
        for jn in self.joint_names:
            if jn not in name_to_idx: return
            qq.append(m.position[name_to_idx[jn]])
        self.q = np.array(qq, dtype=float)
        self.have_js = True

    def cb_palm(self, m):
        d = list(m.data)
        if len(d)>=3:
            self.palm = np.array(d[:3], dtype=float)

    def cb_bottle(self, m):
        d = list(m.data)
        if len(d)>=3:
            self.bpos = np.array(d[:3], dtype=float)
            if len(d)>=7:
                q = np.array(d[3:7], dtype=float)
                n = np.linalg.norm(q)
                self.bquat = (q/n) if n>1e-9 else None
            else:
                self.bquat = None

    # --- Kinematics ---
    def fk_jac(self, q):
        qk = kdl.JntArray(self.ndof)
        for i in range(self.ndof): qk[i] = q[i]
        fr = kdl.Frame()
        self.fk.JntToCart(qk, fr)
        p = np.array([fr.p[0], fr.p[1], fr.p[2]])
        R = np.array([[fr.M[0,0], fr.M[0,1], fr.M[0,2]],
                      [fr.M[1,0], fr.M[1,1], fr.M[1,2]],
                      [fr.M[2,0], fr.M[2,1], fr.M[2,2]]])
        self.ee_quat = quat_from_R(R)
        Jk = kdl.Jacobian(self.ndof)
        self.jac.JntToJac(qk, Jk)
        J = np.zeros((6, self.ndof))
        for r in range(6):
            for c in range(self.ndof):
                J[r,c] = Jk[r,c]
        return p, R, J

    def axis_from_R(self, R, ch):
        idx = {'x':0,'y':1,'z':2}[ch]
        return R[:, idx]

    # --- Desired orientation from bottle ---
    def desired_quat(self, p_current):
        pol = self.ori_policy.lower()
        if pol == "fixed":
            return self.fixed_quat
        if pol == "hold" or self.bpos is None:
            return self.ee_quat

        if pol == "lookat_bottle":
            dirv = normalize(self.bpos - p_current)
            up = np.array([0,0,1.0])
            if abs(np.dot(dirv, up)) > 0.98:
                up = np.array([0,1,0])
            x = dirv
            z = normalize(np.cross(x, up))
            y = normalize(np.cross(z, x))
            if   self.look_axis == 'x': R = np.column_stack((x,y,z))
            elif self.look_axis == 'y': R = np.column_stack((y,z,x))
            else:                       R = np.column_stack((z,x,y))
            return quat_from_R(R)

        if pol == "align_bottle_axis" and self.bquat is not None:
            Rb = R_from_quat(self.bquat)
            t  = self.axis_from_R(Rb, self.bottle_axis)
            up = np.array([0,0,1.0])
            if abs(np.dot(t, up)) > 0.98:
                up = np.array([0,1,0])
            x = normalize(t)
            z = normalize(np.cross(x, up))
            y = normalize(np.cross(z, x))
            if   self.look_axis == 'x': R = np.column_stack((x,y,z))
            elif self.look_axis == 'y': R = np.column_stack((y,z,x))
            else:                       R = np.column_stack((z,x,y))
            return quat_from_R(R)

        return self.ee_quat

    # --- Manipulability & its FD gradient ---
    def _manip(self, J):
        JJt = J.dot(J.T)
        try:
            return float(np.sqrt(np.linalg.det(JJt)))
        except np.linalg.LinAlgError:
            return 0.0

    def _grad_manip_fd(self, q, J, eps_fd=1e-4):
        base_w = self._manip(J)
        g = np.zeros_like(q)
        for i in range(self.ndof):
            q_pert = q.copy(); q_pert[i] += eps_fd
            _, _, Jp = self.fk_jac(q_pert)
            g[i] = (self._manip(Jp) - base_w) / eps_fd
        # Φ_manip = -w ⇒ ∇Φ = -∇w
        return -g

    # --- Nullspace gradient of Φ ---
    def nullspace_gradient(self, q, p, J):
        # 1) centralization
        if self.have_limits:
            q_center = 0.5*(self.q_min + self.q_max)
        else:
            q_center = np.zeros_like(q)
        grad_center = (q - q_center)

        # 2) joint-limit barrier
        eps = 1e-6
        if self.have_limits:
            dmin = (q - self.q_min) + eps
            dmax = (self.q_max - q) + eps
            grad_limit = -2.0/(dmin**3) + 2.0/(dmax**3)
        else:
            grad_limit = np.zeros_like(q)

        # 3) manipulability FD gradient (compute sparsely)
        if self._tick % max(1, int(self._fd_every)) == 0:
            grad_manip = self._grad_manip_fd(q, J, eps_fd=1e-4)
            self._manip_grad_cache = grad_manip
        else:
            grad_manip = self._manip_grad_cache

        # 4) pre-grasp distance (optional)
        if self.w_pre > 0.0 and self.bpos is not None:
            if self.bquat is not None:
                Rb = R_from_quat(self.bquat)
                idx = {'x':0,'y':1,'z':2}[self.bottle_axis]
                a_obj = Rb[:, idx]
            else:
                ref = self.palm if self.palm is not None else (self.bpos + np.array([0,0,0.1]))
                a_obj = normalize(self.bpos - ref)
            p_pre = self.bpos - self.pre_d * a_obj
            J_pos = J[:3, :]
            grad_pre = J_pos.T.dot(p - p_pre)
        else:
            grad_pre = np.zeros_like(q)

        grad_total = (
            self.w_center * grad_center +
            self.w_limit  * grad_limit  +
            self.w_manip  * grad_manip  +
            self.w_pre    * grad_pre
        )

        # tame extremely large gradients
        gnorm = np.linalg.norm(grad_total)
        if gnorm > 1e3:
            grad_total *= (1e3/gnorm)

        self._tick += 1
        return grad_total

    # --- Main step ---
    def step(self, dt):
        if not (self.have_js and self.palm is not None):
            return

        # Current pose & Jacobian
        p, R, J = self.fk_jac(self.q)

        # Task-space velocity target: position + orientation (from bottle policy)
        pd  = self.palm.copy()
        qd  = self.desired_quat(p)
        dp  = pd - p
        do  = quat_err(self.ee_quat, qd)
        v   = np.hstack((self.k_pos*dp, self.k_ori*do))  # 6D

        # DLS inverse
        JJt   = J.dot(J.T)
        lam2I = (self.damping**2) * np.eye(6)
        inv   = np.linalg.pinv(JJt + lam2I)
        Jdls  = J.T.dot(inv)
        qdot  = Jdls.dot(v)

        # Nullspace: z = -∇Φ, projected
        I  = np.eye(self.ndof)
        N  = I - Jdls.dot(J)
        grad = self.nullspace_gradient(self.q, p, J)  # ∇Φ
        qdot += self.null_k * N.dot(-grad)

        # Integrate & publish
        q_next = self.q + qdot*dt
        if self.have_limits:
            q_next = np.clip(q_next, self.q_min, self.q_max)

        jt = JointTrajectory(); jt.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = q_next.tolist()
        pt.time_from_start = rospy.Duration(self.dt_cmd)
        jt.points.append(pt)
        self.pub.publish(jt)

        self.q = q_next  # update internal state

    def run(self):
        r = rospy.Rate(self.rate_hz)
        last = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - last).to_sec()
            if dt <= 0 or dt > 1.0:
                dt = 1.0 / self.rate_hz
            self.step(dt)
            last = now
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("qp_posture_servo")
    node = QPPostureServo()
    rospy.loginfo("qp_posture_servo started. ori_policy=%s", rospy.get_param("~ori_policy","lookat_bottle"))
    node.run()



# rosrun <your_pkg> qp_posture_servo.py \
#   _base_link:=arm_1_base_link _tip_link:=arm_1_gripper_tip \
#   _command_topic:=/arm_1/arm_controller/command \
#   _joint_names:="['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']" \
#   _ori_policy:=lookat_bottle _look_axis:=x _bottle_axis:=z \
#   _k_pos:=1.0 _k_ori:=1.0 _damping:=0.03 _null_k:=0.6 \
#   _w_center:=1.0 _w_limit:=0.3 _w_manip:=0.4 _w_pre:=0.0 \
#   _pregrasp_offset:=0.07 _manip_fd_period:=5 \
#   _rate_hz:=100 _dt_cmd:=0.10


# 典型トラブルと対処

# 発振/ブルブル：k_pos/k_ori を下げる、damping を上げる、rate_hz を適正化

# 届かない/特異姿勢：w_manip↑、null_k↑、pregrasp_offset を+1–2 cm

# 関節端で詰まる：w_limit↑、関節リミットを正確に設定

# CPU負荷大（A）：manip_fd_period を 5–10 に、rate_hz を 80–100へ

# IK解が不安定（B）：cone_deg を少し広げる、num_samples を増やす、pregrasp_offset を微調整