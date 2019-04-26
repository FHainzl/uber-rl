#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash
. ~/catkin_ws/devel/setup.bash
export ROSCONSOLE_FORMAT='[${severity}] - ${node}: [${time}] ${message}'

roslaunch uber-rl connector.launch

sleep 600