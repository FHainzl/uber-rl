#!/usr/bin/env python


import rospy
from sensor_msgs.msg import TimeReference

from config import config as c


class Clock(object):
    def __init__(self, freq):
        self.freq = freq

        self.pub = rospy.Publisher('/tick', TimeReference, queue_size=10)
        self.run()

    def run(self):
        r = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            t = TimeReference()
            t.source = str(self.freq)
            t.header.stamp = rospy.Time.now()
            self.pub.publish(t)
            r.sleep()


if __name__ == '__main__':
    node_name = "clock"
    rospy.init_node(node_name, anonymous=False)

    m = Clock(freq=c["clock_freq"])
