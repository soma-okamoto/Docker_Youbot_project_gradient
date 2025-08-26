#!/usr/bin/env python3

import rospy
from brics_actuator.msg           import JointPositions
from trajectory_msgs.msg          import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg                 import Bool
from sensor_msgs.msg              import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandResponse
from dynamixel_workbench_operators.srv import GripperCmd, GripperCmdResponse

class BridgeSimulationCommand:
    def __init__(self):
        rospy.init_node('bridge_simulation_command')

        # --- Arm1 command conversion ---
        self.pub_arm1 = rospy.Publisher(
            '/youbot/arm_1/arm_controller/command',
            JointTrajectory,
            queue_size=1
        )
        rospy.Subscriber(
            '/arm_1/arm_controller/position_command',
            JointPositions,
            self.cb_arm1
        )

        # --- Arm2 command conversion ---
        self.pub_arm2 = rospy.Publisher(
            '/youbot/arm_2/arm_controller/command',
            JointTrajectory,
            queue_size=1
        )
        rospy.Subscriber(
            '/arm_2/arm_controller/position_command',
            JointPositions,
            self.cb_arm2
        )



        rospy.loginfo('bridge_simulation_command ready')
        rospy.spin()


    def cb_arm1(self, msg: JointPositions):
        traj = self.to_trajectory(msg.positions)
        self.pub_arm1.publish(traj)
        rospy.loginfo('[BridgeCmd] Published sim arm1 trajectory')

    def cb_arm2(self, msg: JointPositions):
        traj = self.to_trajectory(msg.positions)
        self.pub_arm2.publish(traj)
        rospy.loginfo('[BridgeCmd] Published sim arm2 trajectory')

    def to_trajectory(self, positions):
        # Convert JointPositions to JointTrajectory with correct joint names
        traj = JointTrajectory()
        # Strip any namespaces from joint_uri to match controller's expected joint names
        traj.joint_names = [jv.joint_uri.rsplit('/', 1)[-1] for jv in positions]
        pt = JointTrajectoryPoint()
        pt.positions = [jv.value for jv in positions]
        pt.time_from_start = rospy.Duration(1.0)
        traj.points = [pt]
        return traj

    def handle_dynamixel_stub(self, req):
        # Stub for legacy DynamixelCommand, always succeed
        rospy.loginfo('[BridgeCmd] Received DynamixelCommand stub')
        return DynamixelCommandResponse(comm_result=True)

    def handle_gripper_cmd(self, req):
        # Service callback to open/close gripper in simulation
        command = req.command  # 'open' or 'close'
        # choose positions
        positions = self.close_pos if command=='close' else self.open_pos
        # build trajectory point
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = rospy.Duration(1.0)

        # publish to both gripper controllers using correct joint names
        traj = JointTrajectory()
        traj.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        traj.points = [pt]
        self.pub_gripper1.publish(traj)
        self.pub_gripper2.publish(traj)
        rospy.loginfo(f"[BridgeCmd] Published sim gripper trajectory for '{command}' to both arms")

        return GripperCmdResponse(result='True')(result='True')


    def torque_cb(self, msg: JointState):
        # Mimic grasp detection: average effort on gripper fingers
        try:
            idx_l = msg.name.index('gripper_finger_joint_l')
            idx_r = msg.name.index('gripper_finger_joint_r')
            efforts = msg.effort
            avg_effort = abs((efforts[idx_l] + efforts[idx_r]) / 2.0)
            grasped = avg_effort > rospy.get_param('~grasp_threshold', 50.0)
            self.pub_state.publish(grasped)
        except ValueError:
            # JointState does not include gripper joints
            pass

if __name__ == '__main__':
    BridgeSimulationCommand()