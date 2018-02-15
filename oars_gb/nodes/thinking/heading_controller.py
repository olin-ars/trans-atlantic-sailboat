#!/usr/bin/env python

import math
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class HeadingController():

    def __init__(self, verbose=False):

        self.verbose = verbose

        #   Establish ROS node and create topics
        rospy.init_node('sail_control', anonymous=True)
        self.rudder_pub = rospy.Publisher('/rudder_pos', Float32, queue_size=1)
        rospy.Subscriber("/boat/heading", Float32, self.process_current_heading, queue_size=2)
        #   TODO make publisher to publish a target heading
        rospy.Subscriber("/control/target_heading", Float32, self.process_target_heading, queue_size=1)

        #   Subscribers for PI control constants
        #   TODO make controller to publish these values
        rospy.Subscriber("/control/p", Float32, self.process_p, queue_size=1)
        rospy.Subscriber("/control/i", Float32, self.process_i, queue_size=1)

        #   Set initial conditions
        r = rospy.Rate(5)
        self.heading_integral = 0   #   To be multiplied by I term
        self.current_time = time.time() #   Used in integral calculation

        #   Set subscriber constants to None while no messages recieved yet
        self.p_term = None
        self.i_term = None
        self.current_heading = None
        self.target_heading = None

        if self.verbose:
            print("Heading controller initialized.")

        #   Main loop
        while not rospy.is_shutdown():

            #   Only start publishing rudder positions if all other nodes
            #   are being published to
            if all([self.p_term, self.i_term,
                self.current_heading, self.target_heading]):

                #   Publish PI-controlled rudder position
                rudder_pos = self.calculate_rudder_pos()
                self.rudder_pub.publish(rudder_pos)

                if self.verbose:
                    print("Target rudder position: %s" % rudder_pos)
                    print("Target heading: %s   Current heading: %s" % \
                        (self.target_heading, self.current_heading))

            r.sleep()


    def process_target_heading(self, msg):
        """ Update target heading every time subscriber is updated """
        self.target_heading = msg.data


    def process_i(self, msg):
        """ Update i term every time subscriber is updated """
        self.i_term = msg.data


    def process_p(self, msg):
        """ Update p term every time subscriber is updated """
        self.p_term = msg.data


    def process_current_heading(self, msg):
        """ Update current heading every time subscriber is updated """
        self.current_heading = msg.data


    def angle_diff(self, ref_angle, target_angle):
        """ Calculates the minimum distance the reference angle must change to
            reach the target angle.

            Inputs: float ref_angle (degrees), float target_angle (degrees)
            Output: Float representing minimum distance between angles, between
                -180 and 180.
        """

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
        heading_difference = self.angle_diff(self.current_heading, self.target_heading)
        p = self.p_term
        i = self.i_term

        #   Update the heading integral based on the time that has passed
        dt = time.time() - self.current_time
        self.current_time = time.time()
        self.heading_integral += heading_difference*dt

        return p*heading_difference + i*self.heading_integral


if __name__ == '__main__':
    HeadingController(verbose=True)
