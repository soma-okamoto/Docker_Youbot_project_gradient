#!/usr/bin/env python3

import rospy
import actionlib

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


GRIPPER_OPEN_POS  = 0.5
GRIPPER_CLOSE_POS = 0.0
MAX_EFFORT        = 10.0
TORQUE_THRESHOLD  = 50.0

ACTION_GRIPPER = '/gripper_controller/gripper_cmd'


class AmirGripperNode:
    def __init__(self):
        rospy.init_node('amir_gripper')

        self.is_open = False

        # ROS1 action client
        self.gripper_client = actionlib.SimpleActionClient(
            ACTION_GRIPPER,
            GripperCommandAction
        )

        self.state_pub = rospy.Publisher('/gripper_state', Bool, queue_size=10)

        rospy.Subscriber('/grasp_command', String, self.grasp_cb, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb, queue_size=10)

        rospy.loginfo('Waiting for gripper action server: %s', ACTION_GRIPPER)
        server_ok = self.gripper_client.wait_for_server(rospy.Duration(5.0))

        if server_ok:
            rospy.loginfo('Connected to gripper action server')
        else:
            rospy.logwarn('Gripper action server not found: %s', ACTION_GRIPPER)

        rospy.loginfo('AmirGripperNode started')

    def send_gripper_goal(self, position):
        goal = GripperCommandGoal()
        goal.command.position = float(position)
        goal.command.max_effort = float(MAX_EFFORT)

        if not self.gripper_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logwarn('Gripper action server is not available')
            return

        self.gripper_client.send_goal(goal)
        rospy.loginfo('Sent gripper goal: position=%.3f', position)

    def grasp_cb(self, msg):
        cmd = msg.data.lower().strip()
        rospy.loginfo('Received grasp command: %s', cmd)

        if cmd == 'open':
            self.send_gripper_goal(GRIPPER_OPEN_POS)
            self.is_open = True

        elif cmd == 'close':
            self.send_gripper_goal(GRIPPER_CLOSE_POS)
            self.is_open = False

        else:
            rospy.logwarn('Unknown grasp command: %s', cmd)

    def joint_state_cb(self, msg):
        if 'Gripper' not in msg.name:
            return

        idx = msg.name.index('Gripper')

        if idx < len(msg.effort):
            grasped = abs(msg.effort[idx]) > TORQUE_THRESHOLD
            self.state_pub.publish(Bool(data=grasped))


def main():
    node = AmirGripperNode()
    rospy.spin()


if __name__ == '__main__':
    main()