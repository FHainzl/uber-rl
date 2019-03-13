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


def move_z_relative(group, z_delta):
    pose = group.get_current_pose()
    print pose
    pose.pose.position.z += z_delta
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def move_randomly(group, max_dist_per_dimension=0.01):
    pose = group.get_current_pose()
    print pose
    m = max_dist_per_dimension
    y_delta = random.uniform(-m, m)
    z_delta = random.uniform(-m, m)
    pose.pose.position.y += y_delta
    pose.pose.position.z += z_delta
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panda_move_group", anonymous=True, disable_signals=True)
    panda = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    # display_trajectory_publisher = rospy.Publisher(
    #     '/move_group/display_planned_path',
    #     moveit_msgs.msg.DisplayTrajectory,
    #     queue_size=20)

    # joints = group.get_current_joint_values()
    # joints[0] += pi / 8
    # group.

    while True:
        try:
            move_randomly(group,max_dist_per_dimension=0.05)
            # rospy.sleep(0.1)
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()
