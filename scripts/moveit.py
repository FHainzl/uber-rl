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


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panda_move_group", anonymous=True)
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


    pose = group.get_current_pose()
    print pose
    pose.pose.position.z -= 0.1
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

if __name__ == '__main__':
    main()
