#!/usr/bin/env python

import numpy as np

import rospy
import message_filters
from sensor_msgs.msg import JointState, TimeReference
from franka_msgs.msg import FrankaState

from config import config as c
from client import Client


class Connector(object):
    def __init__(self):
        self.last_clock = None
        self.clock_sub = message_filters.Subscriber("/tick", TimeReference)
        self.angle_sub = message_filters.Subscriber("/angle", JointState)
        self.panda_sub = message_filters.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState)

        self.client = Client(port=c["port_remote_server"], host=c["remote_ip"],
                             bufsize=c["bufsize"])

        print "Starting..."
        rospy.sleep(1)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.clock_sub, self.angle_sub, self.panda_sub],
            slop=c["msg_proximity"],
            queue_size=c["message_filter_q_size"])
        self.ts.registerCallback(self.callback)

    def callback(self, clock, angle, panda):
        this_clock = clock.header.stamp.to_sec()
        if c["verbose"]:
            self.print_times(clock, angle, panda)
        try:
            # Check if every time step is sent
            # If time difference bigger than 1.5 times period, let server know
            dt = this_clock - self.last_clock
            print dt
            if dt > 1.5 * c["clock_freq"] ** -1:
                self.client.send_flush()
            else:
                self.send_state(angle, panda)
        except TypeError:
            # For first iteration
            print "Away we go!"

        self.last_clock = clock.header.stamp.to_sec()

    def send_state(self, angle, panda):
        a = angle.position[0]
        # pose is tuple with 16 entries, column-major
        pose = panda.O_T_EE
        x, y, z = pose[-4:-1]
        total_state = np.array([a, x, y, z])
        print total_state
        print total_state.dtype
        self.client.send_array(total_state)

    @staticmethod
    def print_times(clock, angle, panda):
        clock_time = clock.header.stamp
        angle_time = angle.header.stamp
        panda_time = panda.header.stamp
        print "Now  : ", rospy.Time.now().to_sec(), angle.position[0]
        print "clock: ", clock_time.to_sec()
        print "Angle: ", angle_time.to_sec()
        print "Panda: ", panda_time.to_sec()


if __name__ == '__main__':
    node_name = "connector"
    rospy.init_node(node_name, anonymous=False)

    conn = Connector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
