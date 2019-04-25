#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from config import config as c
from server import Server


class Actor(Server):
    def __init__(self, port, bufsize):
        topic = "/ros_subscriber_controller/controller_command/joint_velocity"
        self.vel_pub = rospy.Publisher(topic, JointState, queue_size=1)
        Server.__init__(self, port, bufsize)

    def callback(self, timestamp, data):
        now = rospy.Time.now().to_sec()
        rospy.loginfo("Received: " + str(now))
        rospy.loginfo("Timestamp:" + str(timestamp))
        rospy.loginfo("Delay:    " + str(now - timestamp))
        rospy.loginfo("Actions:  " + str(data))
        self.pub_data(data)

    def pub_data(self, data):
        vel_msg = JointState()
        vel_msg.velocity = [0, 0, 0, 0, data[0], 0, 0]
        self.vel_pub.publish(vel_msg)


if __name__ == '__main__':
    node_name = "actor"
    rospy.init_node(node_name, anonymous=False)

    server = Actor(c["ROS_PORT"], c["bufsize"])
    server.loop()
