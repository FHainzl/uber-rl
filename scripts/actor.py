#!/usr/bin/env python
import rospy

from config import config as c
from server import Server


def server_callback(data):
    rospy.Time.now().to_sec()
    print "Received: ", rospy.Time.now().to_sec(), data


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Server(c["ROS_PORT"], c["bufsize"])
    server.callback = server_callback
    server.loop()
