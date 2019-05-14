from math import pi

import rospy
from sensor_msgs.msg import JointState


class PandaPublisher(object):
    q_start = [-pi / 2, 0, -pi / 2, -pi / 2, 0, pi / 2, pi / 4]
    start_position_msg = JointState(position=q_start)
    stop_msg = JointState(velocity=[0, 0, 0, 0, 0, 0, 0])

    def __init__(self):
        topic = "/ros_subscriber_controller/controller_command/joint_command"
        self.pub = rospy.Publisher(topic, JointState, queue_size=1)

    def publish(self, msg):
        self.pub.publish(msg)

    def move_to_start(self):
        self.publish(self.start_position_msg)

    def stop(self):
        self.publish(self.stop_msg)
