#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8MultiArray


def listener():
    print("node started")
    rospy.init_node('listener', anonymous=True)

    bump = rospy.Subscriber("/bump", Int8MultiArray, message_receive, queue_size=5)

    # Keep this from exiting until the node stops
    rospy.spin()


def message_receive(msg):
    data = msg.data
    if data[0]:
        print('Front left bumper pressed')
    if data[1]:
        print('Left bumper pressed')
    if data[2]:
        print('Front right bumper pressed')
    if data[3]:
        print('Right bumper pressed')

    if not (data[0] or data[1] or data[2] or data[3]):
        print('No bumper pressed. Neato happy!')


if __name__ == '__main__':
    listener()