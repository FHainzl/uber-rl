#!/usr/bin/env python

from math import pi

import rospy
from panda_publisher import PandaPublisher

if __name__ == '__main__':
    node = "joint_velocity_test"
    rospy.init_node(node, anonymous=False)
    # topic = "/ros_subscriber_controller/controller_command/joint_command"
    # pub = rospy.Publisher(topic, JointState, queue_size=1)
    pub = PandaPublisher()
    rospy.sleep(1)
    # pub.move_to_start()
    # pub.move_to_start()
    # rospy.sleep(3)
    position = [-2.8, 0, 0, -pi / 2, 0, pi / 2, pi / 2]
    # position[1] = +0.3
    # position[2] = -0.3
    pub.publish_position(position)
    # rospy.sleep(3)
    # pub.move_to_start()

    # vel = [0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0]
    # vel_neg = [0.0, 0.0, 0.0, 0.0, -10.0, 0.0, 0.0]
    # rospy.sleep(1)
    # pub.publish_effort(vel)
    # rospy.sleep(0.5)
    # pub.publish_effort(vel_neg)
    # rospy.sleep(1)
    # pub.publish_effort(vel)
    # rospy.sleep(0.5)
    # pub.stop()
    # rospy.sleep(1)
    # pub.move_to_start()
    # vel_msg = JointState(velocity=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0])
    #
    # r = rospy.Rate(0.2)
    # for _ in range(1):
    #     pub.publish(vel_msg)
    #     print "Velocity published"
    #     r.sleep()
    #     start = time.time()
    #     pub.move_to_start()
    #     end = time.time()
    #     print "Time taken:", end - start
    #     print "Position published"
    #     r.sleep()
