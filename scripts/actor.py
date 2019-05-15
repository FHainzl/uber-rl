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

        self.timestamp = None

    def clock_callback(self, data):
        if self.msg != PandaPublisher.stop_msg:
            now = rospy.Time.now().to_sec()
            since_state_recoreded = now - self.timestamp

            if since_state_recoreded > 2.5 * (1 / c["clock_freq"]):
                rospy.loginfo("SINCE STATE RECORDED: " +
                              str(since_state_recoreded))
            if since_state_recoreded > 1.5 * (1 / c["clock_freq"]):
                self.msg = PandaPublisher.stop_msg

            self.panda_pub.publish(self.msg)

            if c["verbose"]:
                rospy.loginfo("Published:" + str(now))
                rospy.loginfo("Action:   " + str(self.msg.velocity[4]))
                rospy.loginfo("Since state recorded:" + str(since_state_recoreded))


    def callback(self, timestamp, data):
        """
        Server callback, called upon receiving array from RL
        :param timestamp:
        :param data:
        :return:
        """
        self.set_msg(data)

        now = rospy.Time.now().to_sec()

        if c["verbose"]:
            rospy.loginfo("Received:  " + str(now))
            rospy.loginfo("Timestamp: " + str(timestamp))
            rospy.loginfo("Since sent:" + str(now - timestamp))
            rospy.loginfo("Actions:   " + str(data))

        self.timestamp = timestamp

    def set_msg(self, data):
        a_q4 = data[0]
        self.msg = JointState(velocity=[0, 0, 0, 0, a_q4, 0, 0])


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Actor(c["ROS_PORT"], c["bufsize"])
    server.loop()
