#!/usr/bin/env python
from math import atan2

import numpy as np
import rospy
import cv2
from std_msgs import msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from franka_msgs.msg import FrankaState
from publisher import Publisher

from utils import human_time


class FciReceiver:
    def __init__(self):
        # Subscriber node
        self.image_sub = rospy.Subscriber(topic, FrankaState, self.callback)

    def callback(self, data):
        print data.dtheta

        if print_received:
            time = human_time(data.header.stamp)
            rospy.loginfo("Message received from {}".format(time))


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

    topic = "/franka_state_controller/franka_states"

    print_received = False
    main()
