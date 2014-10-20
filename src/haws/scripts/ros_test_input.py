#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: Rafael Figueroa
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist


def start():
    'Creates a test input for the robotic agent'
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 100)
    # starts the node
    rospy.init_node('test_inputs')

    # ROS main loop
    while not rospy.is_shutdown():
        now = rospy.get_time()
        print 't:', now

        tw = Twist()
        tw.linear.x = 0.5
        tw.angular.z = 0.0
        pub.publish(tw)


if __name__ == '__main__':
    start()

