#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, TimeReference

from config import config as c
from panda_publisher import PandaPublisher
from server import Server


class Actor(Server):
    def __init__(self, port, bufsize):
        Server.__init__(self, port, bufsize)

        self.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.panda_pub = PandaPublisher()
        self.clock_sub = rospy.Subscriber("/tick", TimeReference,
                                          self.clock_callback)

        self.timestamp = None
        self.last_published_timestamp = None

    def clock_callback(self, data):
        # Only publish if you have new information
        if not self.timestamp == self.last_published_timestamp:
            now = rospy.Time.now().to_sec()
            try:
                since_state_recoreded = now - self.timestamp
            except TypeError:
                since_state_recoreded = 0.0

            if since_state_recoreded > 2.5 * (1 / c["clock_freq"]):
                rospy.loginfo("SINCE STATE RECORDED: " +
                              str(since_state_recoreded))
            elif since_state_recoreded > 1.5 * (1 / c["clock_freq"]):
                rospy.loginfo("Since state recorded: " +
                              str(since_state_recoreded))
                self.panda_pub.stop()

            self.panda_pub.publish_effort(self.effort)
            self.last_published_timestamp = self.timestamp

            if c["verbose"]:
                rospy.loginfo("Published:" + str(now))
                rospy.loginfo("Action:   " + str(self.effort[4]))
                rospy.loginfo(
                    "Since state recorded:" + str(since_state_recoreded))

    def callback(self, timestamp, data):
        """
        Server callback, called upon receiving array from RL
        :param timestamp:
        :param data:
        :return:
        """
        self.set_msg(data)
        self.timestamp = timestamp

        now = rospy.Time.now().to_sec()

        if c["verbose"]:
            rospy.loginfo("Received:  " + str(now))
            rospy.loginfo("Timestamp: " + str(timestamp))
            rospy.loginfo("Since sent:" + str(now - timestamp))
            rospy.loginfo("Actions:   " + str(data))

    def set_msg(self, data):
        a_q4 = data[0]
        self.effort = [0.0, 0.0, 0.0, 0.0, a_q4, 0.0, 0.0]


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Actor(c["ROS_PORT"], c["bufsize"])
    server.loop()
