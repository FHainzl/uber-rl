#!/usr/bin/env python
import time

import rospy
from sensor_msgs.msg import JointState


def set_velocity(pub, d_q):
    msg = JointState()
    msg.velocity = d_q
    pub.publish(msg)


def set_velocitiy_two_joints(pub, d_q2, d_q3):
    velocity = (0, 0, d_q2, d_q3, 0, 0, 0)
    set_velocity(pub, velocity)


if __name__ == '__main__':
    node = "joint_velocity_test"
    rospy.init_node(node, anonymous=False)
    topic = "/ros_subscriber_controller/controller_command/joint_velocity"
    pub = rospy.Publisher(topic, JointState, queue_size=1)

    move_up = [0, 0, 0, 0.3, 0, 0, 0]
    move_up_msg = JointState()
    move_up_msg.velocity = move_up

    move_down = [0, 0, 0, -0.3, 0, 0, 0]
    move_down_msg = JointState()
    move_down_msg.velocity = move_down

    stop = [0, 0, 0, 0, 0, 0, 0]
    stop_msg = JointState()
    stop_msg.velocity = stop

    rospy.sleep(1)

    dq = 0.3
    r = rospy.Rate(hz=10)
    for _ in range(5):
        t1 = time.time()

        set_velocitiy_two_joints(pub, dq, 0)
        r.sleep()
        set_velocitiy_two_joints(pub, 0, dq)
        r.sleep()
        set_velocitiy_two_joints(pub, -dq, 0)
        r.sleep()
        set_velocitiy_two_joints(pub, 0, -dq)
        r.sleep()
        t2 = time.time()
        print t2 - t1
    pub.publish(stop_msg)
