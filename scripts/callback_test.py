#!/usr/bin/env python

import numpy as np

import rospy
import message_filters
from sensor_msgs.msg import JointState, TimeReference
from franka_msgs.msg import FrankaState

from config import config
from client import Client


class Connector(object):
    def __init__(self, config):
        self.c = config
        self.last_clock = None
        self.clock_sub = message_filters.Subscriber("/tick", TimeReference)
        self.angle_sub = message_filters.Subscriber("/angle", JointState)

        rospy.sleep(1)
        rospy.loginfo("Starting to transmit Data to RL machine!")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.clock_sub, self.angle_sub],
            slop=self.c["msg_proximity"],
            queue_size=self.c["message_filter_q_size"])
        self.ts.registerCallback(self.callback)

    def callback(self, clock, angle):
        rospy.loginfo("Message filters callback triggered!")
        self.print_times(clock, angle)

    @staticmethod
    def print_times(clock, angle):
        clock_time = clock.header.stamp
        angle_time = angle.header.stamp
        print "Now  : ", rospy.Time.now().to_sec()
        print "clock: ", clock_time.to_sec()
        print "Angle: ", angle_time.to_sec()


if __name__ == '__main__':
    node_name = "connector"
    rospy.init_node(node_name, anonymous=False)

    conn = Connector(config)

    try:
        rospy.loginfo("Spinning...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
