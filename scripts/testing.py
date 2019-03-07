#!/usr/bin/env python

from math import atan2

import numpy as np
import rospy
import cv2
from std_msgs import msg
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError

from franka_msgs.msg import FrankaState
from publisher import Publisher

from utils import human_time


class FciReceiver:
    def __init__(self):
        # Subscriber node
        self.image_sub = rospy.Subscriber(topic, JointState, self.callback)
        print "__init__"

        # Publisher nodes
        self.pub_joint_states_desired = rospy.Publisher("/franka_state_controller/joint_states_desired",
                                                        JointState)

    def callback(self, data):
        print data

        if print_received:
            time = human_time(data.header.stamp)
            rospy.loginfo("Message received from {}".format(time))

        position = list(data.position)
        position[0] = 0
        data.position = tuple(position)
        self.pub_joint_states_desired.publish(data)


def main():
    node = rospy.get_param("node_name")
    rospy.init_node(node, anonymous=False)
    fr = FciReceiver()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.set_param("node_name", "uber_fci_node")
    # rospy.set_param("server_name", "fci_dyn_rec")

    topic = "joint_states"
    type = JointState

    print_received = False
    main()
