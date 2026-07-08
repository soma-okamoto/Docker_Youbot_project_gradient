#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty

import rospy
from control_msgs.msg import JointJog


class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class AMIRKeyboardJointJogROS1:
    def __init__(self):
        rospy.init_node("amir_keyboard_joint_jog_ros1")

        self.topic = rospy.get_param("~topic", "/servo_node/delta_joint_cmds")
        self.frame_id = rospy.get_param("~frame_id", "base_footprint")

        self.joint_names = rospy.get_param("~joint_names", [
            "Joint_1",
            "Joint_2",
            "Joint_3",
            "Joint_4",
            "Joint_5",
        ])

        self.speed = rospy.get_param("~speed", 0.10)
        self.speed_step = rospy.get_param("~speed_step", 0.02)
        self.rate_hz = rospy.get_param("~rate", 30.0)
        self.duration = rospy.get_param("~duration", 0.1)

        self.velocities = [0.0 for _ in self.joint_names]

        self.pub = rospy.Publisher(self.topic, JointJog, queue_size=1)

        rospy.loginfo(
            "\n==== AMIR ROS1 Keyboard JointJog ====\n"
            "publish topic : %s\n"
            "frame_id      : %s\n"
            "joint_names   : %s\n"
            "\n"
            "keys:\n"
            "  1/q : joint1 +/-\n"
            "  2/w : joint2 +/-\n"
            "  3/e : joint3 +/-\n"
            "  4/r : joint4 +/-\n"
            "  5/t : joint5 +/-\n"
            "  space : stop\n"
            "  [ / ] : speed down/up\n"
            "  x : quit\n"
            "speed=%.3f\n"
            "=====================================",
            self.topic,
            self.frame_id,
            self.joint_names,
            self.speed
        )

    def stop(self):
        self.velocities = [0.0 for _ in self.joint_names]

    def set_joint_velocity(self, index, sign):
        self.stop()
        if 0 <= index < len(self.velocities):
            self.velocities[index] = sign * self.speed

    def read_keys(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            c = sys.stdin.read(1)

            if c == "1":
                self.set_joint_velocity(0, +1)
            elif c == "q":
                self.set_joint_velocity(0, -1)

            elif c == "2":
                self.set_joint_velocity(1, +1)
            elif c == "w":
                self.set_joint_velocity(1, -1)

            elif c == "3":
                self.set_joint_velocity(2, +1)
            elif c == "e":
                self.set_joint_velocity(2, -1)

            elif c == "4":
                self.set_joint_velocity(3, +1)
            elif c == "r":
                self.set_joint_velocity(3, -1)

            elif c == "5":
                self.set_joint_velocity(4, +1)
            elif c == "t":
                self.set_joint_velocity(4, -1)

            elif c == " ":
                self.stop()
                rospy.loginfo("stop")

            elif c == "[":
                self.speed = max(0.0, self.speed - self.speed_step)
                rospy.loginfo("speed=%.3f", self.speed)

            elif c == "]":
                self.speed += self.speed_step
                rospy.loginfo("speed=%.3f", self.speed)

            elif c == "x":
                self.stop()
                self.publish()
                rospy.signal_shutdown("quit")

    def publish(self):
        msg = JointJog()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = self.frame_id

        msg.joint_names = list(self.joint_names)
        msg.displacements = []
        msg.velocities = [float(v) for v in self.velocities]
        msg.duration = float(self.duration)

        self.pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        with RawTerminal():
            while not rospy.is_shutdown():
                self.read_keys()
                self.publish()
                rate.sleep()


if __name__ == "__main__":
    node = AMIRKeyboardJointJogROS1()
    node.spin()