#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from brics_actuator.msg import JointPositions, JointValue

class GripperCommandBridge:
    def __init__(self):
        # ノード初期化
        rospy.init_node('gripper_command_bridge', anonymous=True)

        # ブリッジ先 Publisher
        self.pub = rospy.Publisher(
            '/arm_1/gripper_controller/position_command',
            JointPositions,
            queue_size=1
        )

        # コマンド受け口 Subscriber
        rospy.Subscriber(
            '/arm_1/gripper_controller/command',
            String,
            self.command_callback,
            queue_size=1
        )

        # 開閉幅は ROS パラメータでも上書き可能
        self.open_pos  = rospy.get_param('~open_position',  0.0115)  # [m]
        self.close_pos = rospy.get_param('~close_position', 0.0)     # [m]

    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            pos = self.open_pos
        elif cmd == 'close':
            pos = self.close_pos
        else:
            rospy.logwarn(f"[GripperBridge] 未知のコマンド “{msg.data}”")
            return

        # 現在時刻で JointValue を作成
        now = rospy.Time.now()
        jv_l = JointValue(
            timeStamp=now,
            joint_uri='gripper_finger_joint_l',
            unit='m',
            value=pos
        )
        jv_r = JointValue(
            timeStamp=now,
            joint_uri='gripper_finger_joint_r',
            unit='m',
            value=pos
        )

        # まとめて publish
        goal = JointPositions(positions=[jv_l, jv_r])
        self.pub.publish(goal)
        rospy.loginfo(f"[GripperBridge] Gripper → {cmd.upper()} (pos={pos:.4f} m)")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    GripperCommandBridge().run()
