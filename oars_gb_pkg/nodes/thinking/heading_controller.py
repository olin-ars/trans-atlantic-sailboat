#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32, Float32MultiArray, UInt8


class HeadingController:

    def __init__(self, verbose=False):

        self.verbose = verbose
        self.enabled = True

        #   Establish ROS node and create topics
        rospy.init_node('sail_control', anonymous=True)
        self.rudder_pub = rospy.Publisher('/rudder_pos', Float32, queue_size=1)
        rospy.Subscriber("/boat/heading", Float32, self.process_current_heading, queue_size=2)
        #   TODO make publisher to publish a target heading
        rospy.Subscriber("/control/heading/target", Float32, self.process_target_heading, queue_size=1)
        # Subscribe to the topic dictating which operating mode the boat should be in
        rospy.Subscriber('/control/mode', UInt8, self.received_operating_mode_msg, queue_size=1)

        #   Subscribers for PI control constants
        #   TODO make controller to publish these values
        rospy.Subscriber("/control/heading/kp", Float32, self.process_p, queue_size=1)
        rospy.Subscriber("/control/heading/ki", Float32, self.process_i, queue_size=1)

        # Publisher for accumulated error and desired rudder pos
        self.error_pub = rospy.Publisher('/control/heading/error_desired_rudder_pos', Float32MultiArray, queue_size=1)

        #   Set initial conditions
        r = rospy.Rate(5)
        self.accumulated_error = 0  # To be multiplied by I term
        self.current_time = time.time()  # Used in integral calculation

        #   Set subscriber constants to None while no messages recieved yet
        self.p_term = None
        self.i_term = None
        self.current_heading = None
        self.target_heading = None

        if self.verbose:
            print("Heading controller initialized.")

        #   Main loop
        while not rospy.is_shutdown():
            if self.enabled:
                profile = self.profile()

                #   Only start publishing rudder positions if all other nodes
                #   are being published to
                if None not in profile:

                    #   Publish PI-controlled rudder position
                    rudder_pos = self.calculate_rudder_pos()
                    self.rudder_pub.publish(rudder_pos)

                    if self.verbose:
                        print("Target rudder position: %s" % rudder_pos)
                        print("Target heading: %s   Current heading: %s" %
                              (self.target_heading, self.current_heading))

                elif self.verbose:
                    print("P: %s, I: %s, CH: %s, TH: %s" % profile)

            r.sleep()

    def profile(self):
        """ Returns the state of the P term, I term, heading, and target heading in a tuple. """
        return (self.p_term, self.i_term, self.current_heading, self.target_heading)

    def process_target_heading(self, msg):
        """ Update target heading every time subscriber is updated """
        self.target_heading = msg.data

        #   Reset accumulated error so it doesn't accumulate between heading targets
        self.accumulated_error = 0

    def process_i(self, msg):
        """ Update i term every time subscriber is updated """
        self.accumulated_error = 0
        self.i_term = msg.data

    def process_p(self, msg):
        """ Update p term every time subscriber is updated """
        self.accumulated_error = 0
        self.p_term = msg.data

    def received_operating_mode_msg(self, msg):
        """
        Called when a ROS message is received on /control/mode
        :param msg: the ROS message describing which operating mode the boat is in
        :type msg: UInt8
        """
        # Only enable if in fully autonomous mode
        self.enabled = msg.data == 2
        if self.verbose:
            print('Heading controller %s' % 'enabled' if self.enabled else 'disabled')

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
        self.accumulated_error += heading_difference * dt

        output = p * heading_difference + i * self.accumulated_error

        #   Change range to (0, 1) to match rudder controller
        output = output / 2.0 + 0.5

        # Publish accumulated error and desired rudder position
        error_msg = Float32MultiArray(data=[self.accumulated_error, output])
        self.error_pub.publish(error_msg)

        #   limit output to range between 0 and 1
        output = min(output, 1)
        output = max(output, 0)

        return output


if __name__ == '__main__':
    HeadingController(verbose=True)
