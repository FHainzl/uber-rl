from math import pi

import rospy
from sensor_msgs.msg import JointState


class PandaPublisher(object):
    q_start = [-pi / 2, 0, -pi / 2, -pi / 2, 0, pi / 2, pi / 4]
    q_dot_stop = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        topic = "/ros_subscriber_controller/controller_command/joint_command"
        self.pub = rospy.Publisher(topic, JointState, queue_size=1)

    def publish(self, msg):
        self.pub.publish(msg)

    def publish_position(self, position):
        msg = JointState()
        msg.name = 7 * ("position",)
        msg.position = position
        self.publish(msg)

    def publish_velocity(self, velocity):
        msg = JointState()
        msg.name = 7 * ("velocity",)
        msg.velocity = velocity
        self.publish(msg)

    def publish_effort(self, effort):
        msg = JointState()
        msg.name = 7 * ("effort",)
        msg.effort = effort
        self.publish(msg)

    def move_to_start(self):
        self.publish_position(self.q_start)

    def stop(self):
        self.publish_velocity(self.q_dot_stop)
