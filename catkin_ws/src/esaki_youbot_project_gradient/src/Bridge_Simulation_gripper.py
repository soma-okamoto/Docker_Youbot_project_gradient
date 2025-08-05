#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandResponse
from dynamixel_workbench_operators.srv import GripperCmd, GripperCmdResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperSimBridge:
    def __init__(self):
        rospy.init_node('bridge_gripper_sim', anonymous=False)

        # Publication of gripper state (object grasped) mimic real node
        self.pub_state = rospy.Publisher('/gripper_state', Bool, queue_size=10)

        # Subscribe to simulation joint_states
        rospy.Subscriber('/joint_states', JointState, self.torque_cb, queue_size=10)

        # Stub service for DynamixelCommand
        self.srv_dxl = rospy.Service(
            '/dynamixel_workbench/dynamixel_command',
            DynamixelCommand,
            self.handle_dynamixel_command
        )

        # Stub service for GripperCmd
        self.srv_gripper = rospy.Service(
            '/dynamixel_workbench/execution',
            GripperCmd,
            self.handle_gripper_cmd
        )

        # Publisher to simulation gripper controller
        self.traj_pub = rospy.Publisher(
            '/arm_1/gripper_controller/command',
            JointTrajectory,
            queue_size=1
        )

        # Open/close positions for simulation, set via ROS params
        self.open_pos = rospy.get_param('~open_positions', [0.0, 0.0])
        self.close_pos = rospy.get_param('~close_positions', [0.5, 0.5])

        rospy.loginfo('[Bridge] GripperSimBridge initialized')

    def handle_dynamixel_command(self, req):
        # No-op stub: always succeed
        rospy.loginfo('[Bridge] Received DynamixelCommand stub')
        return DynamixelCommandResponse(comm_result=True)

    def handle_gripper_cmd(self, req):
        # Service to open/close gripper in simulation
        command = req.command  # 'open' or 'close'
        traj = JointTrajectory()
        traj.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        pt = JointTrajectoryPoint()
        if command == 'close':
            pt.positions = self.close_pos
        else:
            pt.positions = self.open_pos
        pt.time_from_start = rospy.Duration(1.0)
        traj.points = [pt]
        self.traj_pub.publish(traj)
        rospy.loginfo(f'[Bridge] GripperCmd stub -> published {command} positions')
        return GripperCmdResponse(result='True')

    def torque_cb(self, msg: JointState):
        # Mimic object grasp detection: average effort of two fingers
        try:
            i0 = msg.name.index('gripper_finger_joint_l')
            i1 = msg.name.index('gripper_finger_joint_r')
        except ValueError:
            # Not a gripper joint report
            return
        effort0 = msg.effort[i0] if len(msg.effort) > i0 else 0.0
        effort1 = msg.effort[i1] if len(msg.effort) > i1 else 0.0
        avg = abs((effort0 + effort1) / 2.0)
        object_grasped = avg > rospy.get_param('~grasp_threshold', 50.0)
        self.pub_state.publish(object_grasped)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    bridge = GripperSimBridge()
    bridge.spin()
