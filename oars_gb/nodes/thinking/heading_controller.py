#!/usr/bin/env python

import math
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class HeadingController():

    MIN_SHIFT = 20
    MAIN_MAX_ANGLE = 100
    JIB_MAX_ANGLE = 100

    main_pos = 0
    jib_pos = 0

    def __init__(self):

        #   Establish ROS node and create topics
        rospy.init_node('sail_control', anonymous=True)
        self.rudder_pub = rospy.Publisher('/rudder_pos', Float32, queue_size=1)
        self.current_heading = rospy.Subscriber("/boat/heading", Float32, queue_size=2)
        #   TODO make publisher to publish a target heading
        self.target_heading = rospy.Subscriber("/control/target_heading", Float32, queue_size=1)

        #   Subscribers for PI control constants
        #   TODO make controller to publish these values
        self.p_term = rospy.Subscriber("/control/p", Float32, queue_size=1)
        self.i_term = rospy.Subscriber("/control/i", Float32, queue_size=1)

        #   Set initial conditions
        r = rospy.Rate(2)
        self.rudder_pos = 0
        self.heading_integral = 0   #   To be multiplied by I term
        self.current_time = time.time() #   Used in integral calculation

        print("Heading controller initialized.")

        #   Main loop
        while not rospy.is_shutdown():
            self.rudder_pub.publish(calculate_rudder_pos())

            r.sleep()


    def angle_diff(self, ref_angle, target_angle):
        """ Calculates the minimum distance the reference angle must change to
            reach the target angle.

            Inputs: float ref_angle (degrees), float target_angle (degrees)
            Output: Float representing minimum distance between angles, between
                -180 and 180.
        """

        #   Range of potential angles - e.g. 360
        a_scale = 360

        differences = [target_angle - ref_angle,
            target_angle + a_scale - ref_angle,
            target_angle - ref_angle - a_scale]
        return min(differences, key=abs)


    def calculate_rudder_pos(self):
        """ Uses PI control to calculate a desired position for the rudder.

            Inputs: none (reads from ROS subscribers)
            Outputs: float for desired rudder position
        """

        #   Calculate the difference from your desired heading
        heading_difference = self.angle_diff(self.heading, self.target_heading)
        p = self.p_term
        i = self.i_term

        #   Update the heading integral based on the time that has passed
        dt = self.current_time - time.time()
        self.current_time = time.time()
        self.heading_integral += heading_difference*dt

        return p*heading_difference + i*self.heading_integral


if __name__ == '__main__':
    HeadingController()
