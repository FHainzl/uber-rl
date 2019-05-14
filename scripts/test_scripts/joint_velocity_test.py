#!/usr/bin/env python
import time
from math import pi

import rospy
from sensor_msgs.msg import JointState
from panda_publisher import PandaPublisher

if __name__ == '__main__':
    node = "joint_velocity_test"
    rospy.init_node(node, anonymous=False)
    # topic = "/ros_subscriber_controller/controller_command/joint_command"
    # pub = rospy.Publisher(topic, JointState, queue_size=1)
    pub = PandaPublisher()

    vel_msg = JointState(velocity=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.0])

    r = rospy.Rate(0.2)
    for _ in range(1):
        pub.publish(vel_msg)
        print "Velocity published"
        r.sleep()
        start = time.time()
        pub.move_to_start()
        end = time.time()
        print "Time taken:", end - start
        print "Position published"
        r.sleep()
