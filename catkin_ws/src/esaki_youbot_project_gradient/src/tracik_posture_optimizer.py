#!/usr/bin/env python3
import rospy, numpy as np, math, random
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kdl_parser_py.urdf import treeFromParam
import PyKDL as kdl
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_multiply, quaternion_conjugate

def normalize(v, eps=1e-9): n=np.linalg.norm(v); return v/(n+eps)
def R_from_quat(q): M=quaternion_matrix(q); return np.array(M[:3,:3])
def quat_from_R(R): M=np.eye(4); M[:3,:3]=R; return np.array(quaternion_from_matrix(M))
def manipulability(J):
    JJt=J.dot(J.T)
    try: return math.sqrt(np.linalg.det(JJt))
    except: return 0.0

class TRACIKPostureOpt:
    """
    角度レベル最適化:
      - /identified_bottle_pose から前把持フレームを生成
      - 候補姿勢をサンプル→TRAC-IKでIK→スコア最大の q* を出力
      - /palm_pose は“ユーザーが置きたい位置”としてスコアに寄与
    """
    def __init__(self):
        self.base = rospy.get_param("~base_link","base_link")
        self.tip  = rospy.get_param("~tip_link","arm_1_gripper_tip")
        self.joint_names = rospy.get_param("~joint_names", ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"])
        self.cmd_topic = rospy.get_param("~command_topic","/arm_1/arm_controller/command")
        self.rate_hz   = rospy.get_param("~rate_hz", 20.0)
        self.dt_cmd    = rospy.get_param("~dt_cmd", 0.20)

        # 前把持設定
        self.approach_axis = rospy.get_param("~approach_axis","z")  # ボトルのどの軸から近づくか
        self.look_axis     = rospy.get_param("~look_axis","x")      # EEのどの軸をアプローチに合わせるか
        self.d_pre         = rospy.get_param("~pregrasp_offset", 0.07)  # [m]
        self.cone_deg      = rospy.get_param("~cone_deg", 20.0)         # 許容円錐
        self.num_samples   = rospy.get_param("~num_samples", 8)         # 姿勢候補数

        # スコア重み
        self.w_palm   = rospy.get_param("~w_palm", 1.0)
        self.w_pre    = rospy.get_param("~w_pre", 1.0)
        self.w_align  = rospy.get_param("~w_align", 0.7)
        self.w_manip  = rospy.get_param("~w_manip", 0.4)
        self.w_center = rospy.get_param("~w_center", 0.2)
        self.w_limits = rospy.get_param("~w_limits", 0.3)

        # 関節制限
        self.q_min = np.array(rospy.get_param("~q_min", []), dtype=float)
        self.q_max = np.array(rospy.get_param("~q_max", []), dtype=float)
        self.have_limits = self.q_min.size==len(self.joint_names) and self.q_max.size==len(self.joint_names)

        # KDL & IK
        ok, tree = treeFromParam("/robot_description")
        if not ok: raise RuntimeError("URDF not found at /robot_description")
        if not tree.getChain(self.base, self.tip): raise RuntimeError("KDL chain failed")
        self.chain = tree.getChain(self.base, self.tip)
        self.ndof = self.chain.getNrOfJoints()
        self.fk   = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac  = kdl.ChainJntToJacSolver(self.chain)
        self.ik   = IK(self.base, self.tip, timeout=0.015, epsilon=1e-5)

        # 状態
        self.q = np.zeros(self.ndof); self.have_js=False
        self.palm=None; self.bpos=None; self.bquat=None

        # ROS I/O
        rospy.Subscriber("/joint_states", JointState, self.cb_js, queue_size=50)
        rospy.Subscriber("/palm_pose", Float32MultiArray, self.cb_palm, queue_size=50)
        rospy.Subscriber("/identified_bottle_pose", Float32MultiArray, self.cb_bottle, queue_size=20)
        self.pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=10)

    def cb_js(self, m):
        name_to_idx={n:i for i,n in enumerate(m.name)}
        qq=[]
        for jn in self.joint_names:
            if jn not in name_to_idx: return
            qq.append(m.position[name_to_idx[jn]])
        self.q=np.array(qq,dtype=float); self.have_js=True

    def cb_palm(self, m):
        d=list(m.data)
        if len(d)>=3: self.palm=np.array(d[:3], dtype=float)

    def cb_bottle(self, m):
        d=list(m.data)
        if len(d)>=3:
            self.bpos=np.array(d[:3], dtype=float)
            if len(d)>=7:
                q=np.array(d[3:7], dtype=float); n=np.linalg.norm(q)
                self.bquat=(q/n) if n>1e-9 else None
            else: self.bquat=None

    def fk_of(self, q):
        qk=kdl.JntArray(self.ndof)
        for i in range(self.ndof): qk[i]=q[i]
        fr=kdl.Frame(); self.fk.JntToCart(qk, fr)
        p=np.array([fr.p[0], fr.p[1], fr.p[2]])
        R=np.array([[fr.M[0,0], fr.M[0,1], fr.M[0,2]],
                    [fr.M[1,0], fr.M[1,1], fr.M[1,2]],
                    [fr.M[2,0], fr.M[2,1], fr.M[2,2]]])
        Jk=kdl.Jacobian(self.ndof); self.jac.JntToJac(qk, Jk)
        J=np.zeros((6,self.ndof))
        for r in range(6):
            for c in range(self.ndof): J[r,c]=Jk[r,c]
        return p,R,J

    def axis_from_R(self, R, ch):
        idx={'x':0,'y':1,'z':2}[ch]; return R[:,idx]

    def build_pregrasp(self):
        # ボトル姿勢（無ければ look-at 用に擬似軸）
        if self.bpos is None: return None
        if self.bquat is not None:
            Rb = R_from_quat(self.bquat)
            a  = self.axis_from_R(Rb, self.approach_axis)  # 近づく方向（ボトル軸）
        else:
            # 姿勢なし → Palmからボトルへ向くベクトルを近似アプローチ軸に
            ref = self.palm if self.palm is not None else self.bpos + np.array([0,0,0.1])
            a  = normalize(self.bpos - ref)

        # 前把持位置：ボトル位置から -a 方向に d_pre
        p_pre = self.bpos - self.d_pre * normalize(a)

        # EEのlook_axis を +a に合わせ、rollは world-up で解く
        up=np.array([0,0,1.0])
        if abs(np.dot(a,up))>0.98: up=np.array([0,1,0])
        x = normalize(a);  z = normalize(np.cross(x, up)); y=normalize(np.cross(z,x))
        if   self.look_axis=='x': R_pre=np.column_stack((x,y,z))
        elif self.look_axis=='y': R_pre=np.column_stack((y,z,x))
        else:                      R_pre=np.column_stack((z,x,y))
        return p_pre, R_pre

    def sample_orientations(self, R_pre):
        # 円錐内±cone_degで少数サンプル（rollも少し振る）
        deg=self.cone_deg; rad=math.radians(deg)
        cands=[R_pre]
        for _ in range(int(self.num_samples)-1):
            # 回転軸を R_pre のZで近似、±rad内の小回転
            axis = R_pre[:,2]  # 適当な軸（手先のz）を微調整の基準に
            theta = random.uniform(-rad, rad)
            K = np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
            R = R_pre.dot(np.eye(3)+math.sin(theta)*K+(1-math.cos(theta))*(K.dot(K)))
            # 軽くrollも
            roll = random.uniform(-rad*0.5, rad*0.5)
            K2 = np.array([[0,-R[:,0][2],R[:,0][1]],[R[:,0][2],0,-R[:,0][0]],[-R[:,0][1],R[:,0][0],0]])
            R2 = R.dot(np.eye(3)+math.sin(roll)*K2+(1-math.cos(roll))*(K2.dot(K2)))
            cands.append(R2)
        return cands

    def IK_try(self, p, R, q_seed):
        qsol = self.ik.get_ik(q_seed,
                              p[0],p[1],p[2],
                              *quat_from_R(R))
        return np.array(qsol) if qsol is not None else None

    def joint_limit_penalty(self, q):
        if not self.have_limits: return 0.0
        # 端に近いほどペナルティ（0〜1）
        mid=0.5*(self.q_min+self.q_max); span=0.5*(self.q_max-self.q_min)+1e-6
        z = np.abs(q - mid)/span
        return float(np.clip(z,0,1).mean())

    def score(self, q, p_pre, a_pre):
        # いろいろ評価
        p,R,J = self.fk_of(q)
        # 位置: palm と pregrasp の両方
        e_palm = 0.0 if self.palm is None else np.linalg.norm(p - self.palm)
        e_pre  = np.linalg.norm(p - p_pre)
        # 軸整合: EEのlook_axis と a_pre のなす角
        idx={'x':0,'y':1,'z':2}[self.look_axis]; u=R[:,idx]
        cosang = np.clip(np.dot(normalize(u), normalize(a_pre)), -1.0, 1.0)
        align = 1.0 - (cosang+1.0)/2.0  # 0(良)〜1(悪)
        # 可操作度（高いほど良い）→コストは -manip
        manip = manipulability(J)
        # 関節中央性・限界
        center = 0.0
        if self.have_limits:
            mid=0.5*(self.q_min+self.q_max)
            center = float(np.linalg.norm(q - mid))
        limpen = self.joint_limit_penalty(q)

        # 総合（小さいほど良い）
        cost = self.w_palm*e_palm + self.w_pre*e_pre + self.w_align*align \
             - self.w_manip*manip + self.w_center*center + self.w_limits*limpen
        return cost

    def step(self):
        if not (self.have_js and self.bpos is not None): return
        pre = self.build_pregrasp()
        if pre is None: return
        p_pre, R_pre = pre
        a_pre = R_pre[:,0] if self.look_axis=='x' else R_pre[:,1] if self.look_axis=='y' else R_pre[:,2]

        # 候補サンプル → IK → スコア
        best=None; best_cost=1e18
        for Rcand in self.sample_orientations(R_pre):
            qsol = self.IK_try(p_pre, Rcand, self.q.tolist())
            if qsol is None: continue
            c = self.score(qsol, p_pre, a_pre)
            if c < best_cost: best_cost=c; best=qsol

        if best is None: return  # 解なし

        # 出力（短いホールド）
        jt=JointTrajectory(); jt.joint_names=self.joint_names
        pt=JointTrajectoryPoint(); pt.positions=best.tolist(); pt.time_from_start=rospy.Duration(self.dt_cmd)
        jt.points.append(pt); self.pub.publish(jt)
        self.q = best  # 次回のseedを更新（ウォームスタート）

    def run(self):
        r=rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            r.sleep()

if __name__=="__main__":
    rospy.init_node("tracik_posture_optimizer")
    TRACIKPostureOpt().run()


# TRAC-IK が未導入なら:
# sudo apt-get install ros-<distro>-trac-ik ros-<distro>-trac-ik-python

# rosrun <your_pkg> tracik_posture_optimizer.py \
#   _base_link:=arm_1_base_link _tip_link:=arm_1_gripper_tip \
#   _approach_axis:=z _look_axis:=x _pregrasp_offset:=0.07 \
#   _cone_deg:=20 _num_samples:=8 \
#   _w_palm:=1.0 _w_pre:=1.0 _w_align:=0.7 _w_manip:=0.4 _w_center:=0.2 _w_limits:=0.3