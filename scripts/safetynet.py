#!/usr/bin/env python
from config import config as c
import rospy
from franka_msgs.msg import FrankaState

from panda_publisher import PandaPublisher


class SafetyNet(object):
    def __init__(self):
        self.panda_pub = PandaPublisher()
        self.panda_sub = rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.safety_callback)

    def safety_callback(self, data):
        q = data.q
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]

        if q1 < c["state_space_constraint"]['q1'][0] or \
                q1 > c["state_space_constraint"]['q1'][1]:
            self.stop()
        if q2 < c["state_space_constraint"]['q2'][0] or \
                q2 > c["state_space_constraint"]['q2'][1]:
            self.stop()
        # Check joint constraints
        if q3 < c["state_space_constraint"]['q3'][0] or \
                q3 > c["state_space_constraint"]['q3'][1]:
            self.stop()

    def stop(self):
        self.panda_pub.stop()


if __name__ == '__main__':
    node_name = "safetynet"
    rospy.init_node(node_name, anonymous=False)

    server = SafetyNet()

    try:
        rospy.loginfo("Safety first...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")