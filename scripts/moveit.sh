#!/usr/bin/env bash
source /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash

roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=false &
sleep 3
rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}"
sleep 1
roslaunch panda_moveit_config panda_moveit.launch load_gripper:=false &
sleep 1
rosrun uber-rl moveit.py
sleep 1
