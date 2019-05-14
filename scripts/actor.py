#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, TimeReference

from config import config as c
from panda_publisher import PandaPublisher
from server import Server


class Actor(Server):
    def __init__(self, port, bufsize):
        Server.__init__(self, port, bufsize)

        self.msg = PandaPublisher.stop_msg
        self.panda_pub = PandaPublisher()
        self.clock_sub = rospy.Subscriber("/tick", TimeReference,
                                          self.clock_callback)

    def clock_callback(self, data):
        self.panda_pub.publish(self.msg)
        if self.msg != PandaPublisher.stop_msg:
            now = rospy.Time.now().to_sec()
            rospy.loginfo("Published:" + str(now))

    def callback(self, timestamp, data):
        """
        Server callback, called upon receiving array from RL
        :param timestamp:
        :param data:
        :return:
        """
        now = rospy.Time.now().to_sec()
        rospy.loginfo("Received: " + str(now))
        rospy.loginfo("Timestamp:" + str(timestamp))
        rospy.loginfo("Delay:    " + str(now - timestamp))
        rospy.loginfo("Actions:  " + str(data))
        self.set_msg(data)

    def set_msg(self, data):
        self.msg = JointState(velocity=[0, 0, 0, 0, data[0], 0, 0])


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Actor(c["ROS_PORT"], c["bufsize"])
    server.loop()
