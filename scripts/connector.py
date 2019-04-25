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
        self.panda_sub = message_filters.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState)

        self.client = Client(port=self.c["RL_PORT"],
                             host=self.c["RL_IP"],
                             bufsize=self.c["bufsize"])

        sleep_time = 1
        rospy.loginfo("Sleeping for {} seconds...".format(sleep_time))
        rospy.sleep(sleep_time)
        rospy.loginfo("Starting to transmit Data to RL machine!")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.clock_sub, self.angle_sub, self.panda_sub],
            slop=self.c["msg_proximity"],
            queue_size=self.c["message_filter_q_size"])
        self.ts.registerCallback(self.callback)

    def callback(self, clock, angle, panda):
        # rospy.loginfo("Message filters callback triggered!")
        current_clock = clock.header.stamp.to_sec()
        if self.c["verbose"]:
            self.print_times(clock, angle, panda)

        try:
            # Check if every time step is sent
            # If time difference bigger than 1.5 times period, let server know
            dt = current_clock - self.last_clock
            print dt
            if dt > 1.5 * self.c["clock_freq"] ** -1:
                self.client.send_flush()
            else:
                self.send_state(current_clock, angle, panda)
        except TypeError:
            # For first iteration
            rospy.loginfo("Away we go!")

        self.last_clock = current_clock

    def send_state(self, timestamp, angle, panda):
        angle = angle.position[0]
        # pose is tuple with 16 entries, column-major
        pose = panda.O_T_EE
        x, y, z = pose[-4:-1]  # Last entry is 1 by convention
        q = panda.q
        dq = panda.dq

        # q2, q3 = q[2], q[3]
        # dq2, dq3 = dq[2], dq[3]
        # total_state = np.array([timestamp, angle, q2, q3, dq2, dq3])

        q4 = q[4]
        dq4 = dq[4]
        total_state = np.array([timestamp, angle, q4, dq4])

        self.client.send_array(total_state)

    @staticmethod
    def print_times(clock, angle, panda):
        clock_time = clock.header.stamp
        angle_time = angle.header.stamp
        panda_time = panda.header.stamp
        rospy.loginfo("Now  : " + str(rospy.Time.now().to_sec()))
        rospy.loginfo("clock: " + str(clock_time.to_sec()))
        rospy.loginfo("Angle: " + str(angle_time.to_sec()))
        rospy.loginfo("Panda: " + str(panda_time.to_sec()))

    def __del__(self):
        send_zero_action()


def send_zero_action():
    topic = "/ros_subscriber_controller/controller_command/joint_velocity"
    vel_pub = rospy.Publisher(topic, JointState, queue_size=1)
    vel_msg = JointState()
    vel_msg.velocity = [0, 0, 0, 0, 0, 0, 0]
    vel_pub.publish(vel_msg)


if __name__ == '__main__':
    node_name = "connector"
    rospy.init_node(node_name, anonymous=False)

    conn = Connector(config)

    try:
        rospy.loginfo("Spinning...")
        rospy.spin()
    except KeyboardInterrupt:
        send_zero_action()
        print("Shutting down")
