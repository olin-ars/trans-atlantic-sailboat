#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

class NeatoSail:

    def __init__(self):

        rospy.init_node('sail_control', anonymous = True)

        r = rospy.Rate(5)

        wind_angle = rospy.Subscriber("/rel_wind_vel", Int8MultiArray, self.update_wind, queue_size=5)
        sail_angle = rospy.Subscriber("/encoder", Int8MultiArray, self.update_sail, queue_size=5)
        self.new_sail_pos = 0
        self.current_sail_pos = 0

        rospy.spin()

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)


        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        while not rospy.is_shutdown():
            rotation = (self.new_sail_pos - self.current_sail_pos) * math.pi/ 180
            print('Turn amount:', rotation)
            self.cmd_vel.angular.z = rotation

            self.cmd_pub.publish(self.cmd_vel)
            print("published")

            r.sleep()
            
    def update_wind(self, msg):
        angle = msg.data[0]
        print(angle)
        angle = angle / 2
        if angle > 90:
            angle -= 180
	self.new_sail_pos = angle
        print('Turning to:', self.new_sail_pos)

    def update_sail(self, msg):
        angle = msg.data
        print(angle)
        self.current_sail_pos = angle * 180 /(math.pi*0.122)
        print('Position:', self.current_sail_pos)

if __name__ == '__main__':
    NeatoSail()
