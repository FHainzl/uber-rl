#!/usr/bin/env python
import datetime
import time

import rospy
from sensor_msgs.msg import TimeReference, JointState


def print_time_angle(data):
    t = data.header.stamp
    now = rospy.Time.now().to_sec()
    print "Now   : ", now
    now = datetime.datetime.fromtimestamp(now)
    print(now.strftime('%Y-%m-%d %H:%M:%S'))
    print "Angle : ", t.to_sec()
    t = datetime.datetime.fromtimestamp(t.to_sec())
    print(t.strftime('%Y-%m-%d %H:%M:%S'))


def print_time_clock(data):
    t = data.header.stamp
    now = rospy.Time.now().to_sec()
    print "Now   : ", now
    now = datetime.datetime.fromtimestamp(now)
    print(now.strftime('%Y-%m-%d %H:%M:%S'))
    print "Clock : ", t.to_sec()
    t = datetime.datetime.fromtimestamp(t.to_sec())
    print(t.strftime('%Y-%m-%d %H:%M:%S'))


if __name__ == '__main__':
    node_name = "subscriber_test"
    rospy.init_node(node_name, anonymous=False)

    print rospy.Time.now().to_sec()
    print time.time()

    angle_sub = rospy.Subscriber("/angle", JointState, print_time_angle)
    tick_sub = rospy.Subscriber("/tick", TimeReference, print_time_clock)

    try:
        rospy.loginfo("Spinning...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
