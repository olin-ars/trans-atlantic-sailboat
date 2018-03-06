#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Float32, String, Float32MultiArray
from geometry_msgs.msg import Pose2D
from oars_gb_pkg.utils.serial_utils import resolve_device_port

"""
This ROS node is responsible for handling messages from the Airmar weather station (which contains an
ultrasonic wind speed and direction sensor, a GPS, a magnetometer, an accelerometer, and a gyroscope).
Documentation for how to wire up the Airmar, connect to it, and interpret its messages can be found
here: https://docs.google.com/document/d/1-QkyLn6OJibyypdUsjXektReH0U_7Tz7uKivNQoVPIY
"""


class AirmarParser:

    read_fail_count = 0  # Keep track of the number of consecutive failed Airmar read attempts
    READ_ERROR_ABORT_THRESHOLD = 10  # After this many consecutive failed read attempts, quit

    def __init__(self, port="/dev/ttyUSB0", use_ros=True):

        # Initialize a serial connection to the Airmar
        if port:
            self.serial = serial.Serial()
            self.serial.port = port
            self.serial.baudrate = 4800
            self.serial.open()

        # Initialize the ROS node
        self.use_ros = use_ros
        if use_ros:
            rospy.init_node('airmar')

            # Initialize publishers
            # Boat position (lat/long where pos is N and E, neg is S and W, heading in decimal minutes)
            self.pos_pub = rospy.Publisher('/boat/position', Pose2D, queue_size=5)
            # Boat heading (relative to true north)
            self.heading_true_pub = rospy.Publisher('/boat/heading', Float32, queue_size=2)
            # Speed (in knots)
            self.true_speed_pub = rospy.Publisher('/boat/speed/true', Float32, queue_size=5)
            # Track "made good" (relative to ground and true north)
            self.track_pub = rospy.Publisher('/boat/track', Float32, queue_size=10)
            # Relative wind speed (knots) and direction
            self.rel_wind_pub = rospy.Publisher('/weather/wind/rel', Pose2D, queue_size=5)
            # True wind speed and direction
            self.true_wind_pub = rospy.Publisher('/weather/wind/true', Pose2D, queue_size=5)
            # Error strings we want to see
            self.error_pub = rospy.Publisher('/logging/airmar/errors', String, queue_size=5)
            # Full message publisher for debugging
            self.full_msg_pub = rospy.Publisher('/logging/airmar/fullmsg', String, queue_size=5)

    def run(self):
        """
        This function loops until the node is shut down, reading messages sent from the Airmar
        and publishing their contents to the appropriate ROS topics.
        """
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():

            try:
                # Check if there is data waiting to be read from serial
                if self.serial.inWaiting() == 0:
                    rate.sleep()  # No messages currently, so wait a little bit and then try again
                    continue

                # Read data sent from Airmar
                msg = self.serial.readline()
            except rospy.ROSInterruptException:  # Error connecting to Airmar
                self.read_fail_count += 1
                if self.read_fail_count >= self.READ_ERROR_ABORT_THRESHOLD:
                    self.error_pub.publish('Could not connect to Airmar. Attempt limit reached.')
                    return
                else:
                    self.error_pub.publish('Could not connect to Airmar ({} attempts remaining)'
                                           .format(self.READ_ERROR_ABORT_THRESHOLD - self.read_fail_count))
                    rate.sleep()  # Sleep for 1/10th of a second
                    continue

            # Reset the failed read counter to 0
            self.read_fail_count = 0

            # Publish full message (in case anyone wants it)
            self.full_msg_pub.publish(msg)

            # Break the message up at the commas into an array
            message_components = msg.split(',')

            if message_components[0] == '$WIMWV':  # Wind speed and angle message
                self.handle_wind_message(message_components)

            elif message_components[0] == '$HCHDT':  # Heading relative to true north message
                self.handle_true_heading_message(message_components)

            elif message_components[0] == '$GPGLL':  # GPS coordinates message
                self.handle_gps_message(message_components)

            elif message_components[0] == '$GPVTG':  # Ground speed and track (direction) message
                self.handle_true_course_message(message_components)

            # Flush the serial input buffer
            # self.serial.flushInput()

            # Publish the latest status
            # self.status_pub.publish(self.status)

            # Sleep for 1/10th of a second
            # rate.sleep()

    def handle_wind_message(self, msg):
        """
            Parses the WIMWV message containing either true or relative wind speed and publishes it over ROS.
        """
        if msg[-1][0] == 'A':  # Data is valid
            if msg[2] == 'R':  # Relative wind
                rel_wind = Pose2D()
                rel_wind.theta = float(msg[1])  # Wind direction (knots?)
                rel_wind.x = float(msg[3])  # Wind speed
                # Publish the wind readings
                self.rel_wind_pub.publish(rel_wind)
            else:  # True wind ('T')
                true_wind = Pose2D()
                true_wind.theta = float(msg[1])  # Wind direction (knots?)
                true_wind.x = float(msg[3])  # Wind speed
                self.true_wind_pub.publish(true_wind)
        else:
            self.error_pub.publish('Invalid wind speed reading')

    def handle_true_heading_message(self, msg):
        """
            Parses the HCHDT message containing the heading relative to true north and publishes it over ROS.
        """
        if len(msg[1]) < 1:
            return
        heading = float(msg[1])
        # Publish the heading
        self.heading_true_pub.publish(heading)

    def handle_gps_message(self, msg):
        """
            Parses the GPGLL message containing the position from GPS and publishes it over ROS.
        """

    def parse_gps_message(self, msg):
        def convert_angle(string, sign):
            if not string:
                return 0
            if sign == 'N' or sign == 'S':  # Latitude is only 2-digit degrees
                string = '0' + string
            degrees = int(string[:3])
            minutes = float(string[3:])
            res = degrees + minutes / 60.0
            if sign == 'W' or sign == 'S':
                res *= -1
            return res

        position = None
        if msg[-2][0] == 'A' or msg[-1][0] == 'A':  # Data valid
            position = Pose2D()
            position.y = convert_angle(msg[1], msg[2])  # Latitude
            position.x = convert_angle(msg[3], msg[4])  # Longitude (decimal minutes)
            # Publish the position
            if self.use_ros:
                self.pos_pub.publish(position)
        elif self.use_ros:
            self.error_pub.publish('Invalid GPS reading')

        return position

    def handle_true_course_message(self, msg):
        """
            Parses the GPVTG message containing the true heading and speed (relative to the ground, or
            "course made good" in boat speak) and publishes it over ROS.
        """
        if len(msg[1]) > 0:
            track = float(msg[1])  # True heading
            speed = float(msg[5])  # Speed (knots)
            self.track_pub.publish(track)
            self.true_speed_pub.publish(speed)


if __name__ == '__main__':
    crash_count = 0
    while crash_count < 5:
        try:
            # Find the port of the serial to USB adapter by its serial number
            port = resolve_device_port('FTYZZZOR')
            if port:
                core = AirmarParser(port)
                core.run()
            else:
                crash_count += 1
        except rospy.ROSInterruptException:
            crash_count += 1
            print('Airmar node crashed.')
