#!/usr/bin/env python
import rospy

from config import config as c
from server import Server


class Actor(Server):
    def __init__(self, port, bufsize):
        Server.__init__(self, port, bufsize)

    def callback(self, timestamp, data):
        now = rospy.Time.now().to_sec()
        rospy.loginfo("Received: " + str(now))
        rospy.loginfo("Timestamp:" + str(timestamp))
        rospy.loginfo("Delay:    " + str(now - timestamp))
        rospy.loginfo("Actions:  " + str(data))


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Actor(c["ROS_PORT"], c["bufsize"])
    server.loop()
