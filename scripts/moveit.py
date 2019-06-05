#!/usr/bin/env python
import sys
import copy
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from math import pi as pi
import random

"""
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false
roslaunch panda_moveit_config panda_moveit.launch load_gripper:=false
rosrun uber-rl moveit.py
"""


class Joints(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("panda_move_group", anonymous=True,
                        disable_signals=True)
        panda = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.print_joints()
        self.group.clear_pose_targets()
        self.q_start = (
            (0, -pi / 2),
            (1, 0),
            (2, -pi / 2),
            (3, -pi / 2),
            (4, 0),
            (5, pi / 2),
            (6, pi / 2)
        )

    def print_joints(self):
        joints = self.group.get_current_joint_values()
        print "Current joint angles:"
        for i, joint in enumerate(joints):
            if abs(joint) < 1e-3:
                joint = 0
            print " ", i, ":", joint

    def print_position(self):
        pose = self.group.get_current_pose().pose
        position = pose.position
        print "Current pos:", position.x, position.y, position.z

    def set_q_i(self, joint_id, target):
        self.group.clear_pose_targets()

        joints = self.group.get_current_joint_values()
        joints[joint_id] = target

        self.group.set_joint_value_target(joints)
        self.group.go()

    def set_joints(self, *args):
        self.group.clear_pose_targets()

        joints = self.group.get_current_joint_values()
        for joint_id, target in args:
            joints[joint_id] = target

        self.group.set_joint_value_target(joints)
        self.group.go()

    def set_q_start(self):
        panda.set_joints(*self.q_start)

    def move_joints_2_3(self, d2, d3):
        # group.clear_pose_targets()
        # joints = group.get_current_joint_values()
        joints = [-pi / 2, 0, 0, -pi / 2, 0, pi / 2, pi / 4]
        joints[2] += d2
        joints[3] += d3
        self.group.set_joint_value_target(joints)
        self.group.go()


if __name__ == '__main__':
    panda = Joints()
    panda.set_q_start()

    # d_q = 0.5
    # panda.set_q_i(4, d_q)
    # panda.set_q_i(4, -d_q)
    # panda.set_q_i(4, d_q)
    # panda.set_q_i(4, -d_q)

    panda.set_q_start()
