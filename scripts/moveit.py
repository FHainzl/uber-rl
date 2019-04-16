#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from math import pi as pi
import random


def move_relative(group, dx=0.0, dy=0.0, dz=0.0):
    pose = group.get_current_pose()
    print pose
    pose.pose.position.x += dx
    pose.pose.position.y += dy
    pose.pose.position.z += dz
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def move_z_relative(group, z_delta):
    pose = group.get_current_pose()
    print pose
    pose.pose.position.z += z_delta
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(0.1)


def move_randomly(group, max_dist_per_dimension=0.01):
    pose = group.get_current_pose()
    print pose
    m = max_dist_per_dimension
    x_delta = random.uniform(-m, m)
    y_delta = random.uniform(-m, m)
    z_delta = random.uniform(-m, m)
    pose.pose.position.x += x_delta
    pose.pose.position.y += y_delta
    pose.pose.position.z += z_delta
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def move_absolute(group, x, y, z):
    pose = group.get_current_pose()
    print pose
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(0.1)


def set_joint(group, joint_id, target):
    group.clear_pose_targets()
    joints = group.get_current_joint_values()
    joints[joint_id] = target
    group.set_joint_value_target(joints)
    group.go()


def go_to_start(group):
    set_joint(group, 0, -pi / 2)
    set_joint(group, 1, 0)
    set_joint(group, 2, 0)
    set_joint(group, 3, -pi / 2)
    set_joint(group, 4, 0)
    set_joint(group, 5, pi / 2)
    set_joint(group, 6, pi / 4)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panda_move_group", anonymous=True, disable_signals=True)
    panda = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.clear_pose_targets()
    pose = group.get_current_pose().pose
    joints = group.get_current_joint_values()
    position = pose.position
    print "Current pos:", position.x, position.y, position.z
    print "Current joint angles:", joints

    go_to_start(group)

    # rate = 2  # Hz
    # r = rospy.Rate(rate)
    # while True:
    #     try:
    #         group.clear_pose_targets()
    #         pose = group.get_current_pose().pose
    #         joints = group.get_current_joint_values()
    #         position = pose.position
    #         print "Current pos:", position.x, position.y, position.z
    #         print pose
    #         print "Current joint angles:", joints
    #
    #         set_joint(group, 0, -pi / 2)
    #     except KeyboardInterrupt:
    #         break


if __name__ == '__main__':
    main()
