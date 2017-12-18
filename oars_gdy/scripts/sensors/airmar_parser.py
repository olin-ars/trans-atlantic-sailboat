#!/usr/bin/env python
import rospy
import serial
import sys
from std_msgs.msg import Int16, Float32, Bool, String, Float32MultiArray
from geometry_msgs.msg import Pose2D, Vector3


"""
Airmar message format:
HCHDT - heading, hopefully true (vs. magnetic)
WIMWV - wind speed and angle
    R - "relative" to boat heading - no extra math
    T - true - still relative to boat, but corrected for boat's speed
WIMWD - wind speed and angle relative to north
    <1> - true north (<2> should be 'T')
    <3> - magnetic north (<4> should be 'M')
    <5> - wind speed knots (<6> should be 'N')
    <7> - wind speed m/s (don't use this, we are using knots)
GPGLL - GPS
GPVTG - speed over ground (SOG), course over ground (COG)

$PAMTC,EN,MWD - enable MWD command

$YXXDR,A,86.0,D,PTCH,A,-85.2,D,ROLL*71
$GPVTG,,,,,,,,,N*30
$HCHDT,115.1,T*2D
$WIMWV,250.0,R,0.7,N,A*23
$GPGLL,,,,,,V,N*64
$GPZDA,,,,*48
$WIMWV,,T,,,V*7C
$GPVTG,,,,,,,,,N*30
"""


class AirmarParser:

    def __init__(self, port = "/dev/ttyUSB1"):

        # Initialize a serial connection to the Airmar
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = 4800
        self.serial.open()

        # Initialize the ROS node
        rospy.init_node('airmar')

        # Initialize publishers
        # Airmar status
        self.status_pub = rospy.Publisher('airmar/status', Bool, queue_size=5) #A=active or V=void
        # Boat position (lat/long where pos is N and E, neg is S and W, heading in decimal minutes)
        self.pos_pub = rospy.Publisher('airmar/position', Pose2D, queue_size=5)
        # Speed (in knots)
        self.speed_pub = rospy.Publisher('airmar/speed', Float32, queue_size=5)
        # Track made good (relative to true north)
        self.track_pub = rospy.Publisher('airmar/track', Float32, queue_size=10)
        # Relative wind speed (knots) and direction
        self.rel_wind_pub = rospy.Publisher('/weather/wind/rel', Float32MultiArray, queue_size=5)
        # True wind speed and direction
        self.true_wind_pub = rospy.Publisher('airmar/true_wind', Pose2D, queue_size=5)
        # Compass heading? ????
        self.magnetic_direction_pub = rospy.Publisher('airmar/magnetic_direction', Float32, queue_size=2)  # NSEW
        # Error strings we want to see
        self.error_pub = rospy.Publisher('airmar/errors', String, queue_size=5)
        # Full message publisher for debugging
        self.full_msg_pub = rospy.Publisher('airmar/fullmsg', String, queue_size=5)

        self.status = True

    def run(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            # Read data from Airmar
            try:
                msg = self.serial.readline()
            except rospy.ROSInterruptException:  # Error connecting to Airmar
                self.error_pub.publish('Could not connect to Airmar')
                self.status = False
                continue

            self.full_msg_pub.publish(msg)  # Publish full message (in case anyone wants it)

            # Break the message up at the commas into an array
            message_array = msg.split(',')

            if message_array[0] == '$WIMWV':  # Parse GPRMC message
                self.parse_and_publish_wimwv(message_array)

            elif message_array[0] == '$HCHDT':  # Parse PASHR message
                self.parse_and_publish_hchdt(message_array)

            elif message_array[0] == '$GPGLL':  # Parse PASHR message
                self.parse_and_publish_gpgll(message_array)

            elif message_array[0] == '$GPVTG':  # Parse PASHR message
                self.parse_and_publish_gpvtg(message_array)

            # TODO Add code for more messages here

            # Flush the serial input buffer
            self.serial.flushInput()

            # Publish the latest status
            self.status_pub.publish(self.status)

            # Sleep for 1/10th of a second
            rate.sleep()

    def parse_and_publish_wimwv(self, msg):
        """
            Parse the WIMWV message containing either true or relative wind speed and publishes it over ROS.
        """
        if msg[-1][0] == 'A':  # A = valid, V = void
            self.status = True
            if msg[2] == 'R':  # Relative wind
                rel_wind = Float32MultiArray()
                rel_wind.data = [float(msg[3]), float(msg[1])]  # [speed (knots?), direction]
                # Publish the wind readings
                self.rel_wind_pub.publish(rel_wind)
            else:  # True wind ('T')
                true_wind = Pose2D()
                true_wind.theta = float(msg[1])  # Wind direction (knots?)
                true_wind.x = float(msg[3])  # Wind speed
                self.true_wind_pub.publish(true_wind)
        else:
            self.status = False

    def parse_and_publish_hchdt(self, msg):
        """
            Parses the WIMWV message containing ???
        """
        position = Pose2D()
        position.theta = int((msg[2])[2:-2], 16)
        # Publish the position
        self.pos_pub.publish(position)

    def parse_and_publish_gpgll(self, msg):
        """
            Parses the GPGLL message containing the position and publishes it over ROS.
        """

        def convert_angle(string, longitude=False):
            if not longitude:
                string = '0' + string
            if not string:
                return 0
            degrees = int(string[:3])
            minutes = float(string[3:])
            return degrees + minutes/60

        if msg[-2][0] == 'A' or msg[-1][0] == 'A':  # A = valid, V = void
            position = Pose2D()
            position.x = convert_angle(msg[1])  # Latitude (decimal minutes)
            position.y = convert_angle(msg[3], True)  # Longitude
            # Convert S and W to negatives for lat/long
            if msg[2] == 'S':
                position.x *= -1
            if msg[4] == 'W':
                position.y *= -1
            # Publish the position
            self.pos_pub.publish(position)
        else:
            self.status = False

    def parse_and_publish_gpvtg(self, msg):
        """
            Parse GPVTGG message from hemisphere give us true heading, roll, pitch and heave
        """
        if msg[1] != '':
            track = float(msg[1])  # True heading
            speed = float(msg[5])  # Speed (knots)
            self.track_pub.publish(track)
            self.speed_pub.publish(speed)


if __name__ == '__main__':
    try:
        # port = sys.argv[1]
        core = AirmarParser()
        core.run()
    except rospy.ROSInterruptException:
        print('Airmar node crashed')
