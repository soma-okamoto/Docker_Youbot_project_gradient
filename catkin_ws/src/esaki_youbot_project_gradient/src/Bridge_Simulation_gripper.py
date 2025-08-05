#!/usr/bin/env python3

import rospy
from brics_actuator.msg           import JointPositions
from trajectory_msgs.msg          import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg                 import Bool
from sensor_msgs.msg              import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandResponse
from dynamixel_workbench_operators.srv import GripperCmd, GripperCmdResponse

class BridgeSimulationGripper:
    def __init__(self):
        rospy.init_node('bridge_simulation_gripper')

        # Gripper stub services
        rospy.Service(
            '/dynamixel_workbench/dynamixel_command',
            DynamixelCommand,
            self.handle_dynamixel_stub)
        rospy.Service(
            '/dynamixel_workbench/execution',
            GripperCmd,
            self.handle_gripper_cmd)

        # Publisher for arm1 gripper trajectory
        self.pub_gripper1 = rospy.Publisher(
            '/arm_1/gripper_controller/command',
            JointTrajectory,
            queue_size=1
        )

        # Gripper state mimic using effort from joint_states
        self.pub_state = rospy.Publisher('/gripper_state', Bool, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.torque_cb)

        # Open/Close positions (param or default)
        self.open_pos  = rospy.get_param('~open_positions', [0.04, 0.04])
        self.close_pos = rospy.get_param('~close_positions', [0.0, 0.0])

        rospy.loginfo('bridge_simulation_gripper ready')
        rospy.spin()

    def handle_dynamixel_stub(self, req):
        # Stub for legacy DynamixelCommand, always succeed
        rospy.loginfo('[BridgeGripper] Received DynamixelCommand stub')
        return DynamixelCommandResponse(comm_result=True)

    def handle_gripper_cmd(self, req):
        # Open/close gripper in simulation for arm1 only
        cmd = req.command  # 'open' or 'close'
        pos = self.close_pos if cmd == 'close' else self.open_pos
        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.time_from_start = rospy.Duration(1.0)

        traj = JointTrajectory()
        traj.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        traj.points = [pt]

        # Publish only to arm1 gripper controller
        self.pub_gripper1.publish(traj)
        rospy.loginfo(f"[BridgeGripper] Published sim gripper '{cmd}' trajectory to /arm_1/gripper_controller/command")

        return GripperCmdResponse(result='True')

    def torque_cb(self, msg: JointState):
        # Mimic grasp detection: average effort on gripper fingers
        try:
            i0 = msg.name.index('gripper_finger_joint_l')
            i1 = msg.name.index('gripper_finger_joint_r')
            e0 = msg.effort[i0] if len(msg.effort) > i0 else 0.0
            e1 = msg.effort[i1] if len(msg.effort) > i1 else 0.0
            avg = abs((e0 + e1) / 2.0)
            grasped = avg > rospy.get_param('~grasp_threshold', 50.0)
            self.pub_state.publish(grasped)
        except ValueError:
            pass

if __name__ == '__main__':
    BridgeSimulationGripper()