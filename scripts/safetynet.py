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
        dq = data.dq
        q1 = q[1]
        dq1 = dq[1]
        q2 = q[2]
        dq2 = dq[2]
        q3 = q[3]
        dq3 = dq[3]

        tol = 0.3
        if (q1 < c["state_space_constraint"]['q1'][0] - tol and dq1 < -0.1) or \
                (q2 < c["state_space_constraint"]['q2'][
                    0] - tol and dq2 < -0.1) or \
                (q3 < c["state_space_constraint"]['q3'][
                    0] - tol and dq3 < -0.1) or \
                (q1 > c["state_space_constraint"]['q1'][
                    1] + tol and dq1 > 0.1) or \
                (q2 > c["state_space_constraint"]['q2'][
                    1] + tol and dq2 > 0.1) or \
                (q3 > c["state_space_constraint"]['q3'][1] + tol and dq3 > 0.1):
            self.reset()

    def reset(self):
        rospy.loginfo("Resetting")
        self.panda_pub.move_to_start()


if __name__ == '__main__':
    node_name = "safetynet"
    rospy.init_node(node_name, anonymous=False)

    server = SafetyNet()

    try:
        rospy.loginfo("Safety first...")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
