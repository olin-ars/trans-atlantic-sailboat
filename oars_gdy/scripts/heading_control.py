#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class NeatoSail:

    MIN_SHIFT = 20
    MAIN_MAX_ANGLE = 100
    JIB_MAX_ANGLE = 100

    main_pos = 0
    jib_pos = 0

    def __init__(self):

        rospy.init_node('sail_control', anonymous = True)

        r = rospy.Rate(2)

        wind_angle = rospy.Subscriber("/weather/wind/rel", Float32MultiArray, self.update_wind, queue_size=5)
        self.new_sail_pos = 0

        self.main_pub = rospy.Publisher('/main_pos', Float32, queue_size=0)
        self.jib_pub = rospy.Publisher('/jib_pos', Float32, queue_size=0)

        self.main_pos = 0
        self.jib_pos = 0

        print("initialized")
        while not rospy.is_shutdown():
            self.main_pub.publish(self.main_pos)
            self.jib_pub.publish(self.jib_pos)
            # print("published")

            r.sleep()
            
    def update_wind(self, msg):
        angle = msg.data[1]
        print('Wind angle:', angle)
        self.main_pos = self.calc_main_pos(angle)
        self.jib_pos = self.calc_jib_pos(angle)
        print('Setting main to {} and jib to {}'.format(self.main_pos, self.jib_pos))

    def calc_main_pos(self, wind_angle):
        res = (180 - abs(180 - (wind_angle+self.MIN_SHIFT))) / self.MAIN_MAX_ANGLE
        if res > 1:
            res = 1
        elif res < 0:
            res = 0
        return res

    def calc_jib_pos(self, wind_angle):
        res = (180 - abs(180 - (wind_angle+self.MIN_SHIFT))) / self.JIB_MAX_ANGLE
        if res > 1:
            res = 1
        elif res < 0:
            res = 0
        return res


if __name__ == '__main__':
    NeatoSail()
