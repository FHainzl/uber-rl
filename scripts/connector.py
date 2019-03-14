#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState

from utils import human_time

old_time = None


def angle_sub():
    sub_angle = rospy.Subscriber("/angle", JointState, angle_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


def angle_callback(data):
    time = human_time(data.header.stamp)
    rospy.loginfo("Angle received at {} a".format(time))


def botstate_sub():
    sub_botstate = rospy.Subscriber("/franka_state_controller/franka_states",
                                    FrankaState, botstate_callback,
                                    queue_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


def botstate_callback(data):
    time = human_time(data.header.stamp)
    rospy.loginfo("Robot received at {} r".format(time))


def test_delay():
    msg_proximity = 0.03  # sec
    q_size = 2

    angle_sub = message_filters.Subscriber("/angle", JointState)
    panda_sub = message_filters.Subscriber(
        "/franka_state_controller/franka_states",
        FrankaState)

    ts = message_filters.ApproximateTimeSynchronizer([angle_sub, panda_sub],
                                                     slop=msg_proximity,
                                                     queue_size=q_size)
    ts.registerCallback(test_delay_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


def test_delay_callback(angle, panda):
    # print_received_callback(angle, panda)

    time_angle = angle.header.stamp
    time_panda = panda.header.stamp
    now = rospy.rostime.get_rostime()
    print "Angle: ", time_angle.to_sec()
    print "Panda: ", time_panda.to_sec()
    print "Now  : ", now.to_sec()
    print "Delay A: ", (now - time_angle).to_sec(), "s"
    print "Delay P: ", (now - time_panda).to_sec(), "s"


def print_received_callback(angle, panda):
    time = human_time(angle.header.stamp)
    rospy.loginfo("Angle received at {} a".format(time))
    time = human_time(panda.header.stamp)
    rospy.loginfo("Robot received at {} r".format(time))


class Connector(object):
    def __init__(self, msg_proximity=0.02, dt=0.2, q_size=10):
        self.dt = dt

        self.angle_sub = message_filters.Subscriber("/angle", JointState)
        self.panda_sub = message_filters.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.angle_sub, self.panda_sub],
            slop=msg_proximity,
            queue_size=q_size)
        self.ts.registerCallback(self.callback)

        self.last_delay = None

    def callback(self, angle, panda):
        time = angle.header.stamp
        sec = time.to_sec() % 1
        delay = sec % self.dt
        if delay < self.last_delay:
            print delay
            print_received_callback(angle, panda)
        self.last_delay = delay


if __name__ == '__main__':
    node_name = "connector"
    rospy.init_node(node_name, anonymous=False)

    # angle_sub()
    # test_delay()

    c = Connector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
