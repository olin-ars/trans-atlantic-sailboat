#!/usr/bin/env python
'''
Works best if sail is calibrated to the lowest position,
rudder is calibrated to the leftmost position,
and rate is set to high.
'''
import math
import rospy
from geometry_msgs.msg import Twist
from evdev import InputDevice, categorize, ecodes

gamepad = InputDevice('/dev/input/by-id/usb-Horizon_Hobby_SPEKTRUM_RECEIVER_00000000001A-event-joystick')

sailDownMax = 0
sailUpMax = 1700
rudderLeftMax = 170
rudderRightMax = 1478
rudderRight = -1
rudderLeft = 1
sailOut = 1
sailIn = -1

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


class NeatoRC:

    def __init__(self):

        rospy.init_node('rc_control')

        r = rospy.Rate(5)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)


        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        while not rospy.is_shutdown():

            for event in gamepad.read_loop():
                print('Got event')
                if not event.type == 0:
                    if event.code == 1:
                        x_vel = translate(event.value, sailDownMax, sailUpMax, sailIn, sailOut)
                        print(x_vel)
                        self.cmd_vel.linear.x = x_vel
                    elif event.code == 2:
                        z_ang = translate(event.value, rudderLeftMax, rudderRightMax, rudderLeft, rudderRight)
                        print(z_ang)
                        self.cmd_vel.angular.z = z_ang

                self.cmd_pub.publish(self.cmd_vel)
                print("published")

            r.sleep()

if __name__ == '__main__':
    NeatoRC()