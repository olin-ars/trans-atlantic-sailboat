# !/usr/bin/env python

import rospy
from msg import Bump


def listener():
    print("node started")
    rospy.init_node('listener', anonymous=True)

    bump = rospy.Subscriber("/bump", Bump, queue_size=0)

    while not rospy.is_shutdown():
        if bump.leftFront == 1 or bump.rightFront == 1:
            print("Bump")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()