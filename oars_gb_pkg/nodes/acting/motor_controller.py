#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from oars_gb_pkg.libraries.dynamixels.dynamixel_motor import DynamixelMotor
from oars_gb_pkg.utils.serial_utils import resolve_device_port


class MotorController:

    MAIN_PROFILE = {
        'id': 1,
        'min': 0,
        'max': 13200
    }
    JIB_PROFILE = {
        'id': 2,
        'min': 0,
        'max': -13200
    }
    RUDDER_PROFILE = {
        'id': 0,
        'min': 300,
        'max': 710
    }

    def __init__(self):
        """
        Initialize the motors.
        """

        # Determine the USB port the USB2Dynamixel serial adapter is connected to
        port = resolve_device_port('AL03EQAH')

        # Initialize the motors
        self.main = DynamixelMotor(self.MAIN_PROFILE, port)
        self.jib = DynamixelMotor(self.JIB_PROFILE, port)
        self.rudder = DynamixelMotor(self.RUDDER_PROFILE, port)
        print('Initialized motor controller node')

    def callback_main_pos(self, msg, debug = False):
        pos = msg.data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pos)
        # Make sure the value is within the accepted range of 0 to 1
        if pos > 1:
            pos = 1
        elif pos < 0:
            pos = 0
        self.main.set_position(pos, debug)

    def callback_jib_pos(self, msg, debug = False):
        pos = msg.data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pos)
        # Make sure the value is within the accepted range of 0 to 1
        if pos > 1:
            pos = 1
        elif pos < 0:
            pos = 0
        self.jib.set_position(pos, debug)

    def callback_rudder_pos(self, msg, debug = False):
        pos = msg.data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pos)
        # Make sure the value is within the accepted range of 0 to 1
        if pos > 1:
            pos = 1
        elif pos < 0:
            pos = 0
        self.rudder.set_position(pos, debug)

    def run(self, debug = False):
        """
        Create ROS node named motorcontroller, which subscribes to desired sail position topics and sets the sails accordingly
        """
        rospy.init_node('motorcontroller', anonymous=True)
        rospy.Subscriber("main_pos", Float32, self.callback_main_pos)
        rospy.Subscriber("jib_pos", Float32, self.callback_jib_pos)
        rospy.Subscriber("rudder_pos", Float32, self.callback_rudder_pos)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    MotorController().run(True)
