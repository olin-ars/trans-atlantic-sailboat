#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from evdev import InputDevice


class SpektrumRCController:

    def __init__(self, device):
        """
        Creates a new ControllerListener for listening to messages from an RC controller.
        """
        if not device:
            raise ValueError('Parameter "device" not specified')

        # Connect to the controller
        self.gamepad = InputDevice(device)

        rospy.init_node('motor_control')

        self.r = rospy.Rate(5)

        self.jib_pos = rospy.Publisher('/jib_pos', Float32, queue_size=0)
        self.main_pos = rospy.Publisher('/main_pos', Float32, queue_size=0)
        self.rudder_pos = rospy.Publisher('/rudder_pos', Float32, queue_size=0)
        print('Spectrum controller initialized.')

    @staticmethod
    def normalize(pos, max_pos, min_pos):
        """
        Normalizes a position.
        :param pos: the position
        :param max_pos: the maximum of its range
        :param min_pos: the minimum of its range
        :return: the normalized value (between 0 and 1, inclusive). If the computed value is out of range,
            i.e. < 0 or > 1), it will be rounded to either 0 or 1, respectively.
        """
        res = float(pos - min_pos) / float(max_pos - min_pos)
        if res > 1:
            res = 1
        elif res < 0:
            res = 0
        return res

    def run(self, debug=False):

        while not rospy.is_shutdown():

            for event in self.gamepad.read_loop():
                if not event.type == 0:
                    if event.code == 1:
                        position = self.normalize(event.value, 1700, 0)
                        if debug:
                            print('Setting sails to position {:0.4f}'.format(position))
                        msg = Float32(data=position)
                        self.main_pos.publish(msg)
                        self.jib_pos.publish(msg)

                    elif event.code == 2:
                        position = self.normalize(event.value, 1700, 170)
                        if debug:
                            print('Setting rudder to position {:0.4f}'.format(position))
                        msg = Float32(data=position)
                        self.rudder_pos.publish(msg)

            self.r.sleep()


if __name__ == '__main__':
    SpektrumRCController('/dev/input/by-id/usb-Horizon_Hobby_SPEKTRUM_RECEIVER_00000000001A-event-joystick').run()