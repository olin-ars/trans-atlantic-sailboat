#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist


class NeatoRC:

    def __init__(self):

        rospy.init_node('rc_control')

        r = rospy.Rate(5)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)


        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 1
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        while not rospy.is_shutdown():

            self.cmd_pub.publish(self.cmd_vel)

            r.sleep()

if __name__ == '__main__':
    NeatoRC()